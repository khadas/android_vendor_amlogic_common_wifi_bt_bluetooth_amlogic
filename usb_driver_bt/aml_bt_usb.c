#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/skbuff.h>
#include <linux/usb.h>
#include <linux/cdev.h>
#include <linux/ioctl.h>
#include <linux/io.h>
#include <linux/firmware.h>
#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/reboot.h>
#include <linux/time.h>
#include <linux/kthread.h>
#include <linux/semaphore.h>
#include <linux/poll.h>
#include <linux/platform_device.h>

#define AML_BT_ROM_CHECK        0

#include "bt_fucode.h"
#include "aml_bt_usb.h"

#define AML_BT_VERSION  (0x20230704)
#define CONFIG_BLUEDROID        1 /* bleuz 0, bluedroid 1 */
#define INDEPENDENT_USB			0

#define AML_USB_DEBUG			0
#define AML_RW_DEBUG 			0
#define AML_BT_RW_DEBUG         0
#define REG_DEV_RESET           0xf03058
#define REG_PMU_POWER_CFG       0xf03040
#define REG_RAM_PD_SHUTDWONW_SW 0xf03050
#define BIT_PHY                 1
#define BIT_MAC                 (1 << 1)
#define BIT_CPU                 (1 << 2)
#define DEV_RESET_SW            16
#define DEV_RESET_HW            0
#define BIT_RF_NUM              28

#if AML_USB_DEBUG
    #define AMLBT_DBG(fmt, arg...) printk(KERN_INFO "aml_btusb: " fmt "\n" , ## arg)
#else
    #define AMLBT_DBG(fmt, arg...)
#endif

#if AML_RW_DEBUG
    #define AMLRW_DBG(fmt, arg...) printk(KERN_INFO "aml_btusb: " fmt "\n" , ## arg)
#else
    #define AMLRW_DBG(fmt, arg...)
#endif

#define FAMILY_TYPE_IS_W1(x)        ((AMLBT_PD_ID_FAMILY & x) == AMLBT_FAMILY_W1)
#define FAMILY_TYPE_IS_W1U(x)       ((AMLBT_PD_ID_FAMILY & x) == AMLBT_FAMILY_W1U)
#define INTF_TYPE_IS_SDIO(x)        ((AMLBT_PD_ID_INTF & x) == AMLBT_INTF_SDIO)
#define INTF_TYPE_IS_USB(x)         ((AMLBT_PD_ID_INTF & x) == AMLBT_INTF_USB)

#define TYPE_RETURN_STR(type) \
    case type:                \
        return #type;

const char* amlbt_family_intf(int type) {
    switch (type) {
        TYPE_RETURN_STR(AMLBT_FAMILY_W1)
        TYPE_RETURN_STR(AMLBT_FAMILY_W1U)
        TYPE_RETURN_STR(AMLBT_INTF_SDIO)
        TYPE_RETURN_STR(AMLBT_INTF_USB)
        default:
            break;
    }

    return "unknown type";
}

static unsigned int amlbt_if_type = AMLBT_TRANS_UNKNOWN;

static int evt_state = 0;
static unsigned int type[256] = {0};
static unsigned char p_acl_buf[HCI_MAX_FRAME_SIZE] = {0};
static unsigned char download_fw = 0;
static unsigned char download_flag = 0;
static unsigned char download_end = 0;
static unsigned int iccm_base_addr = 0;
static unsigned int dccm_base_addr = 0;
static unsigned int close_state = 0;
static unsigned suspend_value = 0;

static unsigned char cmd[4][2] = {{0xf2, 0xfe}, {0xf1, 0xfe}, {0xf0, 0xfe}, {0xf3, 0xfe}};
static unsigned char cmd_cpt[64] = {0x04, 0x0e, 0x04, 0x01, 0x98, 0xfc, 0x00};
static volatile unsigned char cmd_index = 0xff;
static unsigned char dw_state = 0;
static unsigned char type_buff[RX_TYPE_FIFO_LEN] = {0};

extern struct auc_hif_ops g_auc_hif_ops;
extern struct usb_device *g_udev;
extern int auc_send_cmd(unsigned int addr, unsigned int len);

struct completion usb_completion;
static unsigned int fw_cmd_w = 0;
static unsigned int fw_cmd_r = 0;

#define ICCM_RAM_BASE           (0x000000)
#define DCCM_RAM_BASE           (0xd00000)

static struct task_struct *check_fw_rx_task = NULL;
static struct mutex fw_type_fifo_mutex;
static struct mutex fw_evt_fifo_mutex;
static struct mutex fw_data_fifo_mutex;

static dev_t bt_devid; /* bt char device number */
static struct cdev bt_char_dev; /* bt character device structure */
static struct class *bt_char_class; /* device class for usb char driver */

static gdsl_fifo_t *g_cmd_fifo = 0;
static gdsl_fifo_t *g_event_fifo = 0;
static gdsl_fifo_t *g_rx_fifo = 0;
static gdsl_tx_q_t *g_tx_q = 0;
static gdsl_fifo_t *g_rx_type_fifo = 0;
static gdsl_fifo_t *g_fw_data_fifo = 0;
static gdsl_fifo_t *g_fw_evt_fifo = 0;
static gdsl_fifo_t *g_fw_type_fifo = 0;
static gdsl_fifo_t *g_lib_cmd_fifo = 0;

static unsigned char g_lib_cmd_buff[128] = {0};

static unsigned int gdsl_fifo_is_valid(gdsl_fifo_t *p_fifo, unsigned int len);

static int reg_config_complete = 0;
static int amlbt_sdio_major;
static struct cdev amlbt_sdio_cdev;
static int amlbt_sdio_devs = 1;
static struct class *amlbt_sdio_class;
static struct device *amlbt_sdio_dev;
static unsigned int rf_num = -1;

extern void set_wifi_bt_sdio_driver_bit(bool is_register, int shift);
extern int  aml_sdio_init(void);
extern unsigned char (*host_wake_req)(void);
extern void aml_wifi_sdio_power_lock(void);
extern void aml_wifi_sdio_power_unlock(void);
static int amlbt_usb_check_fw_rx(void *data);

extern unsigned char wifi_in_insmod;
extern int g_sdio_driver_insmoded;
extern unsigned char g_sdio_after_porbe;
extern struct amlw1_hif_ops g_w1_hif_ops;

DECLARE_WAIT_QUEUE_HEAD(poll_amlbt_queue);

static gdsl_fifo_t *gdsl_fifo_init(unsigned int len, unsigned char *base_addr)
{
    gdsl_fifo_t *p_fifo = (gdsl_fifo_t *)kzalloc(sizeof(gdsl_fifo_t), GFP_KERNEL);

    AMLBT_DBG("%s \n", __func__);

    if (p_fifo)
    {
        memset(p_fifo, 0, sizeof(gdsl_fifo_t));
        p_fifo->w = 0;
        p_fifo->r = 0;
        p_fifo->base_addr = base_addr;
        p_fifo->size = len;
    }

    return p_fifo;
}

static void gdsl_fifo_deinit(gdsl_fifo_t *p_fifo)
{
    if (p_fifo == NULL)
    {
        return ;
    }

    kfree(p_fifo);
}

unsigned int gdsl_fifo_is_valid(gdsl_fifo_t *p_fifo, unsigned int len)
{
    unsigned int unused = 0;

    AMLBT_DBG("func gdsl_fifo_is_valid \n");
    AMLBT_DBG("p_fifo->w:%#lx , p_fifo->r:%#lx\n", (unsigned long)p_fifo->w, (unsigned long)p_fifo->r);

    if (p_fifo->w > p_fifo->r)
    {
        unused = p_fifo->size - (p_fifo->w - p_fifo->r) - 4;
    }
    else if (p_fifo->w == p_fifo->r)
    {
        unused = p_fifo->size - 4;
    }
    else
    {
        unused = p_fifo->r - p_fifo->w - 4;
    }

    AMLBT_DBG("unused:%d, len:%d\n", unused, len);

    return (unused >= len) ? GDSL_ERR_SPACE_VALID : GDSL_ERR_SPACE_INVALID;
}

unsigned int gdsl_fifo_copy_data(gdsl_fifo_t *p_fifo, unsigned char *data, unsigned int len)
{
    unsigned int ret = 0;
    unsigned int offset = 0;

    //len = ((len + 3) & 0xFFFFFFFC);
    ret = gdsl_fifo_is_valid(p_fifo, len);

    if (ret == GDSL_ERR_SPACE_VALID)
    {
        offset = p_fifo->size - (unsigned int)(p_fifo->w - p_fifo->base_addr);
        if (len < offset)
        {
            memcpy(p_fifo->w, data, len);
            p_fifo->w += len;
        }
        else
        {
            memcpy(p_fifo->w, data, offset);
            p_fifo->w = p_fifo->base_addr;
            memcpy(p_fifo->w, &data[offset], (len - offset));
            p_fifo->w += (len - offset);
        }
    }
    else
    {
        printk("gdsl_fifo_copy_data no space!!!\n");
    }
    return ret;
}


unsigned int gdsl_fifo_write_data(gdsl_fifo_t *p_fifo, unsigned char *data, unsigned int len)
{
    unsigned int ret = 0;
    unsigned int offset = 0;

    AMLBT_DBG("%s len:%d\n", __func__, len);

    len = ((len + 3) & 0xFFFFFFFC);
    ret = gdsl_fifo_is_valid(p_fifo, len);

    if (ret == GDSL_ERR_SPACE_VALID)
    {
        offset = p_fifo->size - (unsigned int)(p_fifo->w - p_fifo->base_addr);
        if (len < offset)
        {
            g_auc_hif_ops.bt_hi_write_sram(data, p_fifo->w, len);
            p_fifo->w += len;
        }
        else
        {
            g_auc_hif_ops.bt_hi_write_sram(data, p_fifo->w, offset);
            p_fifo->w = p_fifo->base_addr;
            g_auc_hif_ops.bt_hi_write_sram(&data[offset], p_fifo->w, (len - offset));
            p_fifo->w += (len - offset);
        }
    }
    else
    {
        printk("gdsl_fifo_write_data no space!!!\n");
    }
    return ret;
}

unsigned int gdsl_fifo_used_size(gdsl_fifo_t *p_fifo)
{
    if (p_fifo->w == p_fifo->r)
    {
        return 0;
    }

    if (p_fifo->w > p_fifo->r)
    {
        return (p_fifo->w - p_fifo->r);
    }

    if (p_fifo->w < p_fifo->r)
    {
        return (p_fifo->size - (p_fifo->r - p_fifo->w));
    }

    return 0;
}

unsigned int gdsl_fifo_update_r(gdsl_fifo_t *p_fifo, unsigned int len)
{
    unsigned int offset = 0;
    unsigned int read_len = 0;
    unsigned char *p_end = 0;

    //printk("%s p_fifo->w %#x, p_fifo->r %#x\n", __func__, (unsigned long)p_fifo->w, (unsigned long)p_fifo->r);
    //printk("%s len %d\n", __func__, len);

    if (p_fifo->w == p_fifo->r)
    {
        printk("%s no data!!!\n", __func__);
        return 0;
    }

    if (p_fifo->w > p_fifo->r)
    {
        read_len = (unsigned int)(p_fifo->w - p_fifo->r);
        if (len <= read_len)
        {
            read_len = len;
        }
        //printk("%s read len A %d\n", __func__, read_len);
        p_fifo->r += read_len;
    }
    else
    {
        p_end = (p_fifo->base_addr + p_fifo->size);
        AMLBT_DBG("%s w %#x, r %#x\n", __func__, (unsigned long)p_fifo->w, (unsigned long)p_fifo->r);
        AMLBT_DBG("%s read p_end %#x\n", __func__, (unsigned long)p_end);
        offset = (unsigned int)(p_end - p_fifo->r);
        read_len = offset;
        if (len < offset)
        {
            p_fifo->r += len;
            read_len = len;
            AMLBT_DBG("%s 111 len %#x \n", __func__, len);
        }
        else
        {
            p_fifo->r = p_fifo->base_addr;
            read_len += (len - offset);
            p_fifo->r += (len - offset);
            AMLBT_DBG("%s 222 len %#x \n", __func__, len);
        }
        //printk("%s read len B %#x \n", __func__, read_len);
    }

    //printk("%s actual len %#x \n", __func__, read_len);

    return read_len;
}

unsigned int gdsl_fifo_get_data(gdsl_fifo_t *p_fifo, unsigned char *buff, unsigned int len)
{
    unsigned int offset = 0;
    unsigned int read_len = 0;
    unsigned char *p_end = 0;

    //printk("%s p_fifo->w %#x, p_fifo->r %#x\n", __func__, (unsigned long)p_fifo->w, (unsigned long)p_fifo->r);
    //printk("%s len %d\n", __func__, len);

    if (p_fifo->w == p_fifo->r)
    {
        printk("%s no data!!!\n", __func__);
        return 0;
    }

    if (p_fifo->w > p_fifo->r)
    {
        read_len = (unsigned int)(p_fifo->w - p_fifo->r);
        if (len <= read_len)
        {
            read_len = len;
        }
        //printk("%s read len A %d\n", __func__, read_len);
        memcpy(buff, p_fifo->r, read_len);
        p_fifo->r += read_len;
    }
    else
    {
        p_end = (p_fifo->base_addr + p_fifo->size);
        AMLBT_DBG("%s w %#x, r %#x\n", __func__, (unsigned long)p_fifo->w, (unsigned long)p_fifo->r);
        AMLBT_DBG("%s read p_end %#x\n", __func__, (unsigned long)p_end);
        offset = (unsigned int)(p_end - p_fifo->r);
        read_len = offset;
        if (len < offset)
        {
            memcpy(buff, p_fifo->r, len);
            p_fifo->r += len;
            read_len = len;
            AMLBT_DBG("%s 111 len %#x \n", __func__, len);
        }
        else
        {
            memcpy(buff, p_fifo->r, offset);
            p_fifo->r = p_fifo->base_addr;
            memcpy(&buff[offset], p_fifo->r, len - offset);
            read_len += (len - offset);
            p_fifo->r += (len - offset);
            AMLBT_DBG("%s 222 len %#x \n", __func__, len);
        }
        AMLBT_DBG("%s read len B %#x \n", __func__, read_len);
    }

    AMLBT_DBG("%s actual len %#x \n", __func__, read_len);

    return read_len;
}


unsigned int gdsl_read_data(gdsl_fifo_t *p_fifo, unsigned char *data, unsigned int len)
{
    unsigned int offset = 0;
    unsigned int read_len = 0;
    unsigned int remain_len = 0;
    unsigned char *p_end = 0;

    AMLBT_DBG("%s p_fifo->w %#lx, p_fifo->r %#lx\n", __func__, (unsigned long)p_fifo->w, (unsigned long)p_fifo->r);
    AMLBT_DBG("%s len %d\n", __func__, len);

    if (p_fifo->w == p_fifo->r)
    {
        printk("%s no data!!!\n", __func__);
        return 0;
    }

    if (p_fifo->w > p_fifo->r)
    {
        read_len = (unsigned int)(p_fifo->w - p_fifo->r);
        if (len <= read_len)
        {
            read_len = len;
        }
        AMLBT_DBG("%s read len A %d\n", __func__, read_len);
        g_auc_hif_ops.bt_hi_read_sram(data, p_fifo->r, read_len);
        p_fifo->r += read_len;
    }
    else
    {
        p_end = (p_fifo->base_addr + p_fifo->size);
        AMLBT_DBG("%s w %#x, r %#x\n", __func__, (unsigned long)p_fifo->w, (unsigned long)p_fifo->r);
        AMLBT_DBG("%s read p_end %#x\n", __func__, (unsigned long)p_end);
        offset = (unsigned int)(p_end - p_fifo->r);
        if (len < offset)
        {
            g_auc_hif_ops.bt_hi_read_sram(data, p_fifo->r, len);
            p_fifo->r += len;
            read_len = len;
            AMLBT_DBG("%s 111 len %#x \n", __func__, len);
        }
        else
        {
            AMLBT_DBG("r %#x offset %#x\n", (unsigned long)p_fifo->r, offset);
            g_auc_hif_ops.bt_hi_read_sram(data, p_fifo->r, offset);
            p_fifo->r = p_fifo->base_addr;
            read_len = offset;
            remain_len = (p_fifo->w - p_fifo->r);
            if (((len - offset) != 0) && (remain_len != 0))
            {
                if ((len - offset) > remain_len)
                {
                    AMLBT_DBG("r1 %#x len %#x\n", (unsigned long)p_fifo->r, remain_len);
                    g_auc_hif_ops.bt_hi_read_sram(&data[offset], p_fifo->r, remain_len);
                    read_len += remain_len;
                    p_fifo->r += remain_len;
                }
                else
                {
                    AMLBT_DBG("r2 %#x len %#x\n", p_fifo->r, remain_len);
                    g_auc_hif_ops.bt_hi_read_sram(&data[offset], p_fifo->r, len - offset);
                    read_len += (len - offset);
                    p_fifo->r += (len - offset);
                }
            }
            AMLBT_DBG("%s 222 len %#x \n", __func__, len);
        }
        AMLBT_DBG("%s read len B %#x \n", __func__, read_len);
    }

    AMLBT_DBG("%s actual len %#x \n", __func__, read_len);

    return read_len;
}

unsigned int amlbt_usb_get_tx_prio(gdsl_tx_q_t *p_fifo, unsigned int acl_handle)
{
    unsigned int prio = 0;
    unsigned int i = 0;
    unsigned int find = 0;

    for (i = 0; i < WF_SRAM_TX_Q_NUM; i++)
    {
        if (p_fifo[i].tx_q_dev_index == acl_handle && p_fifo[i].tx_q_status == GDSL_TX_Q_USED)
        {
            if (p_fifo[i].tx_q_prio >= prio)
            {
                prio = p_fifo[i].tx_q_prio;
                find = 1;
            }
        }
    }

    if (!find)
    {
        prio = 7;
    }

    return prio;
}

void amlbt_usb_update_tx_q(gdsl_tx_q_t *p_fifo)
{
    unsigned int i = 0, j = 0;
    unsigned int acl_handle = 0;
    unsigned int tx_q_status[WF_SRAM_TX_Q_NUM] = {0};
    //unsigned int changed = 0;
    //unsigned int tx_q_info[WF_SRAM_TX_Q_NUM * 3] = {0};

    //AMLBT_DBG("up tx\n");

    AMLBT_DBG("up q\n");

    if (FAMILY_TYPE_IS_W1U(amlbt_if_type))
    {
        g_auc_hif_ops.bt_hi_read_sram((unsigned char *)tx_q_status, (unsigned char *)p_fifo[0].tx_q_status_addr,
                                  sizeof(tx_q_status));
    }

    AMLBT_DBG("[%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x]", tx_q_status[0], tx_q_status[1], tx_q_status[2],
              tx_q_status[3], tx_q_status[4], tx_q_status[5], tx_q_status[6], tx_q_status[7]);

    for (i = 0; i < WF_SRAM_TX_Q_NUM; i++)
    {
        //tx_q_status = g_auc_hif_ops.bt_hi_read_word((unsigned long)p_fifo[i].tx_q_status_addr);

        if (tx_q_status[i] == GDSL_TX_Q_COMPLETE)
        {
            acl_handle = p_fifo[i].tx_q_dev_index;
            AMLBT_DBG("up:%#x,%#x\n", i, p_fifo[i].tx_q_prio);
            p_fifo[i].tx_q_dev_index = 0;
            p_fifo[i].tx_q_status = GDSL_TX_Q_UNUSED;
            p_fifo[i].tx_q_prio = (WF_SRAM_TX_Q_NUM - 1);
            if (FAMILY_TYPE_IS_W1U(amlbt_if_type))
            {
                g_auc_hif_ops.bt_hi_write_word((unsigned int)g_tx_q[i].tx_q_dev_index_addr, g_tx_q[i].tx_q_dev_index);
                g_auc_hif_ops.bt_hi_write_word((unsigned int)g_tx_q[i].tx_q_prio_addr, g_tx_q[i].tx_q_prio);
                g_auc_hif_ops.bt_hi_write_word((unsigned int)g_tx_q[i].tx_q_status_addr, g_tx_q[i].tx_q_status);
            }

            for (j = 0; j < WF_SRAM_TX_Q_NUM; j++)
            {
                if (p_fifo[j].tx_q_dev_index == acl_handle)
                {
                    if (p_fifo[j].tx_q_status == GDSL_TX_Q_USED && p_fifo[j].tx_q_prio)
                    {
                        p_fifo[j].tx_q_prio--;
                        if (FAMILY_TYPE_IS_W1U(amlbt_if_type))
                        {
                            g_auc_hif_ops.bt_hi_write_word((unsigned int)g_tx_q[j].tx_q_prio_addr, g_tx_q[j].tx_q_prio);
                        }

                        AMLBT_DBG("dec:%#x,%#x,%#x\n", j, p_fifo[j].tx_q_prio, p_fifo[j].tx_q_status);
                    }
                }
            }
        }
    }
}


unsigned int amlbt_usb_get_tx_q(gdsl_tx_q_t *p_fifo, unsigned int acl_handle)
{
    unsigned int prio = 0xff;
    unsigned int i = 0;
    //unsigned int find = 0;
    unsigned int index = WF_SRAM_TX_Q_NUM;

    for (i = 0; i < WF_SRAM_TX_Q_NUM; i++)
    {
        if (p_fifo[i].tx_q_dev_index == acl_handle && p_fifo[i].tx_q_status == GDSL_TX_Q_USED)
        {
            if (prio == 0xff)
            {
                prio = p_fifo[i].tx_q_prio;
                index = i;
            }
            else if (p_fifo[i].tx_q_prio < prio)
            {
                prio = p_fifo[i].tx_q_prio;
                //find = 1;
                index = i;
            }
        }
    }

    return index;
}

void amlbt_usb_rx_type_fifo_init(void)
{
    if (g_rx_type_fifo == 0)
    {
        g_rx_type_fifo = gdsl_fifo_init(RX_TYPE_FIFO_LEN, (unsigned char *)WF_SRAM_RX_TYPE_FIFO_ADDR);

        if (FAMILY_TYPE_IS_W1U(amlbt_if_type))
        {
            //update read pointer
            g_auc_hif_ops.bt_hi_write_word(WF_SRAM_RX_TYPE_FIFO_R_ADDR, (unsigned int)(unsigned long)g_rx_type_fifo->r);
            //update write pointer
            g_auc_hif_ops.bt_hi_write_word(WF_SRAM_RX_TYPE_FIFO_W_ADDR, (unsigned int)(unsigned long)g_rx_type_fifo->w);
        }

        g_rx_type_fifo->r = (unsigned char *)(unsigned long)(WF_SRAM_RX_TYPE_FIFO_ADDR);
    }
}

void amlbt_usb_rx_type_fifo_deinit(void)
{
    if (g_rx_type_fifo)
    {
        if (FAMILY_TYPE_IS_W1U(amlbt_if_type))
        {
            g_auc_hif_ops.bt_hi_write_word(WF_SRAM_RX_TYPE_FIFO_R_ADDR, 0);
            g_auc_hif_ops.bt_hi_write_word(WF_SRAM_RX_TYPE_FIFO_W_ADDR, 0);
        }

        gdsl_fifo_deinit(g_rx_type_fifo);
        g_rx_type_fifo = 0;
    }
}

void amlbt_usb_hci_cmd_fifo_init(void)
{
    if (g_cmd_fifo == 0)
    {
        g_cmd_fifo = gdsl_fifo_init(WF_SRAM_CMD_LEN, (unsigned char *)(WF_SRAM_CMD_Q_ADDR));

        if (FAMILY_TYPE_IS_W1U(amlbt_if_type))
        {
            //update read pointer
            g_auc_hif_ops.bt_hi_write_word(WF_SRAM_CMD_FIFO_R_ADDR, (unsigned int)(unsigned long)g_cmd_fifo->r);
            AMLBT_DBG("cmd fifo init r: %#lx\n", (unsigned long)g_cmd_fifo->r);
            //update write pointer
            g_auc_hif_ops.bt_hi_write_word(WF_SRAM_CMD_FIFO_W_ADDR, (unsigned int)(unsigned long)g_cmd_fifo->w);
        }

        AMLBT_DBG("cmd fifo init w : %#lx\n", (unsigned long)g_cmd_fifo->w);
        g_cmd_fifo->w = (unsigned char *)(unsigned long)(WF_SRAM_CMD_Q_ADDR);
    }
    AMLBT_DBG("%s end \n", __func__);
}

void amlbt_usb_hci_cmd_fifo_deinit(void)
{
    AMLBT_DBG("%s \n", __func__);
    if (g_cmd_fifo != 0)
    {
        if (FAMILY_TYPE_IS_W1U(amlbt_if_type))
        {
            g_auc_hif_ops.bt_hi_write_word(WF_SRAM_CMD_FIFO_R_ADDR, 0);
            g_auc_hif_ops.bt_hi_write_word(WF_SRAM_CMD_FIFO_W_ADDR, 0);
        }

        gdsl_fifo_deinit(g_cmd_fifo);
        g_cmd_fifo = 0;
    }
}

void amlbt_usb_hci_tx_data_init(void)
{
    unsigned int i = 0;
    unsigned tx_info[WF_SRAM_TX_Q_NUM * 3] = {0};

    AMLBT_DBG("%s \n", __func__);

    if (g_tx_q == 0)
    {
        g_tx_q = (gdsl_tx_q_t *)kzalloc(sizeof(gdsl_tx_q_t) * WF_SRAM_TX_Q_NUM, GFP_KERNEL);
    }

    for (i = 0; i < WF_SRAM_TX_Q_NUM; i++)
    {
        g_tx_q[i].tx_q_addr = (unsigned char *)(unsigned long)(WF_SRAM_TX_Q_ADDR + i * TX_Q_LEN);
        g_tx_q[i].tx_q_status_addr = (unsigned int *)(unsigned long)(WF_SRAM_TX_Q_STATUS_ADDR + i * 4);
        g_tx_q[i].tx_q_prio_addr = (unsigned int *)(unsigned long)(WF_SRAM_TX_Q_PRIO_ADDR + i * 4);
        g_tx_q[i].tx_q_dev_index_addr = (unsigned int *)(unsigned long)(WF_SRAM_TX_Q_INDEX_ADDR + i * 4);

        g_tx_q[i].tx_q_dev_index = 0;
        g_tx_q[i].tx_q_prio = (WF_SRAM_TX_Q_NUM - 1);
        g_tx_q[i].tx_q_status = GDSL_TX_Q_UNUSED;
        tx_info[i] = g_tx_q[i].tx_q_status;
        tx_info[i + 8] = g_tx_q[i].tx_q_dev_index;
        tx_info[i + 16] = g_tx_q[i].tx_q_prio;
    }

    if (FAMILY_TYPE_IS_W1U(amlbt_if_type))
    {
        g_auc_hif_ops.bt_hi_write_sram((unsigned char *)tx_info,
                                   (unsigned char *)WF_SRAM_TX_Q_STATUS_ADDR, sizeof(tx_info));
    }

    AMLBT_DBG("%s end \n", __func__);
}

void amlbt_usb_hci_tx_data_deinit(void)
{
    unsigned tx_info[WF_SRAM_TX_Q_NUM * 3] = {0};

    AMLBT_DBG("%s \n", __func__);

    if (g_tx_q)
    {
        if (FAMILY_TYPE_IS_W1U(amlbt_if_type))
        {
            g_auc_hif_ops.bt_hi_write_sram((unsigned char *)tx_info,
                                       (unsigned char *)WF_SRAM_TX_Q_STATUS_ADDR, sizeof(tx_info));
        }

        kfree(g_tx_q);
        g_tx_q = 0;
    }
    AMLBT_DBG("%s end \n", __func__);
}

void amlbt_usb_hci_evt_fifo_init(void)
{
    AMLBT_DBG("%s \n", __func__);

    if (g_event_fifo == 0)
    {
        g_event_fifo = gdsl_fifo_init(WF_SRAM_EVENT_LEN, (unsigned char *)(WF_SRAM_EVENT_Q_ADDR));
        if (FAMILY_TYPE_IS_W1U(amlbt_if_type))
        {
            g_auc_hif_ops.bt_hi_write_word(WF_SRAM_EVT_FIFO_R_ADDR, (unsigned int)(unsigned long)g_event_fifo->r);
            AMLBT_DBG("event fifo init r: %#lx\n", (unsigned long)g_event_fifo->r);
            g_auc_hif_ops.bt_hi_write_word(WF_SRAM_EVT_FIFO_W_ADDR, (unsigned int)(unsigned long)g_event_fifo->w);
        }

        AMLBT_DBG("event fifo init w : %#lx\n", (unsigned long)g_event_fifo->w);
        g_event_fifo->r = (unsigned char *)(unsigned long)(WF_SRAM_EVENT_Q_ADDR);
    }
    AMLBT_DBG("%s end \n", __func__);
}

void amlbt_usb_hci_evt_fifo_deinit(void)
{
    AMLBT_DBG("%s \n", __func__);
    if (g_event_fifo != 0)
    {
        if (FAMILY_TYPE_IS_W1U(amlbt_if_type))
        {
            g_auc_hif_ops.bt_hi_write_word(WF_SRAM_EVT_FIFO_R_ADDR, 0);
            g_auc_hif_ops.bt_hi_write_word(WF_SRAM_EVT_FIFO_W_ADDR, 0);
        }

        gdsl_fifo_deinit(g_event_fifo);
        g_event_fifo = 0;
    }
    AMLBT_DBG("%s end \n", __func__);
}

void amlbt_usb_fw_recv_fifo_init(void)
{
    AMLBT_DBG("%s \n", __func__);

    if (g_rx_fifo == 0)
    {
        g_rx_fifo = gdsl_fifo_init(WF_SRAM_RX_FIFO_LEN, (unsigned char *)(WF_SRAM_RX_Q_FIFO_ADDR));
        if (FAMILY_TYPE_IS_W1U(amlbt_if_type))
        {
            g_auc_hif_ops.bt_hi_write_word(WF_SRAM_RX_FIFO_R_ADDR, (unsigned int)(unsigned long)g_rx_fifo->r);
            AMLBT_DBG("recv fifo init r: %#lx\n", (unsigned long)g_rx_fifo->r);
            g_auc_hif_ops.bt_hi_write_word(WF_SRAM_RX_FIFO_W_ADDR, (unsigned int)(unsigned long)g_rx_fifo->w);
        }

        AMLBT_DBG("recv fifo init w : %#lx\n", (unsigned long)g_rx_fifo->w);
        g_rx_fifo->r = (unsigned char *)(unsigned long)(WF_SRAM_RX_Q_FIFO_ADDR);
    }
    AMLBT_DBG("%s end \n", __func__);
}

void amlbt_usb_fw_recv_fifo_deinit(void)
{
    AMLBT_DBG("%s \n", __func__);
    if (g_rx_fifo != 0)
    {
        if (FAMILY_TYPE_IS_W1U(amlbt_if_type))
        {
            g_auc_hif_ops.bt_hi_write_word(WF_SRAM_RX_FIFO_R_ADDR, 0);
            g_auc_hif_ops.bt_hi_write_word(WF_SRAM_RX_FIFO_W_ADDR, 0);
        }

        gdsl_fifo_deinit(g_rx_fifo);
        g_rx_fifo = 0;
    }
}

static void amlbt_usb_send_hci_cmd(unsigned char *data, unsigned int len)
{
    AMLBT_DBG("%s, len %d \n", __func__, len);

    if (g_cmd_fifo == NULL)
    {
        printk("%s: bt_usb_hci_cmd_fifo NULL!!!!\n", __func__);
        return ;
    }

    len = ((len + 3) & 0xFFFFFFFC);//Keep 4 bytes aligned

    AMLBT_DBG("%s, Actual length %d \n", __func__, len);
    // step 1: Update the command FIFO read pointer
    if (FAMILY_TYPE_IS_W1U(amlbt_if_type))
    {
        g_cmd_fifo->r = (unsigned char *)(unsigned long)g_auc_hif_ops.bt_hi_read_word(WF_SRAM_CMD_FIFO_R_ADDR);
    }

    g_cmd_fifo->r += WF_SRAM_CMD_Q_ADDR;
    // step 2: Check the command FIFO space

    //step 3: Write HCI commands to WiFi SRAM
    if (FAMILY_TYPE_IS_W1U(amlbt_if_type))
    {
        gdsl_fifo_write_data(g_cmd_fifo, data, len);
    }

    //step 4: Update the write pointer and write to WiFi SRAM

    //	AMLBT_DBG("before write:r:%#lx, w:%#lx\n", r, w);

    if (FAMILY_TYPE_IS_W1U(amlbt_if_type))
    {
        g_auc_hif_ops.bt_hi_write_word(WF_SRAM_CMD_FIFO_W_ADDR, ((unsigned long)g_cmd_fifo->w - WF_SRAM_CMD_Q_ADDR) & 0xfff);
    }

    AMLBT_DBG("len %#x:w %#lx, r %#lx\n", len, (unsigned long)g_cmd_fifo->w, (unsigned long)g_cmd_fifo->r);

#if AML_BT_RW_DEBUG
    printk("w:r:%#lx, w:%#lx\n", (unsigned long)g_cmd_fifo->r, (unsigned long)g_cmd_fifo->w);
#endif
    //	AMLBT_DBG("after write: r:%#lx, w:%#lx\n", (unsigned long)g_cmd_fifo->r, (unsigned long)g_cmd_fifo->w);
}

static void amlbt_usb_send_hci_data(unsigned char *data, unsigned int len)
{
    unsigned int i = 0;
    unsigned int acl_handle = (((data[1] << 8) | data[0]) & 0xfff);
    unsigned int prio = 0;
    //unsigned int tx_q_info[3 * WF_SRAM_TX_Q_NUM] = {0};
    //unsigned int tx_q_status[WF_SRAM_TX_Q_NUM] = {0};
    //	AMLBT_DBG("s d r:%#x, w:%#x\n", g_tx_fifo->r, g_tx_fifo->w);
    AMLBT_DBG("%s, len:%d\n", __func__, len);

    amlbt_usb_update_tx_q(g_tx_q);

    for (i = 0; i < WF_SRAM_TX_Q_NUM; i++)
    {
        if (g_tx_q[i].tx_q_status == GDSL_TX_Q_UNUSED)
        {
            break;
        }
    }

    if (i == WF_SRAM_TX_Q_NUM)
    {
        printk("%s: hci data space invalid!!!! \n", __func__);
        //gdsl_tx_unlock();
        return ;
    }

    AMLBT_DBG("%s idle queue index : %d, handle:%#x\n", __func__, i, acl_handle);

    prio = amlbt_usb_get_tx_prio(g_tx_q, acl_handle);

    g_tx_q[i].tx_q_prio = (++prio & 7);
    g_tx_q[i].tx_q_dev_index = acl_handle;
    g_tx_q[i].tx_q_status = GDSL_TX_Q_USED;

    AMLBT_DBG("D(%#x):%#x,%#x,%#x\n", i, (unsigned long)g_tx_q[i].tx_q_dev_index,
              (unsigned long)g_tx_q[i].tx_q_prio, len);

    if (FAMILY_TYPE_IS_W1U(amlbt_if_type))
    {
        g_auc_hif_ops.bt_hi_write_sram(data, g_tx_q[i].tx_q_addr, len);
    }


#if 0
    g_auc_hif_ops.bt_hi_write_sram((unsigned char *)&g_tx_q[i].tx_q_dev_index,
                                   (unsigned char *)g_tx_q[i].tx_q_dev_index_addr, sizeof(g_tx_q[i].tx_q_dev_index));
    g_auc_hif_ops.bt_hi_write_sram((unsigned char *)&g_tx_q[i].tx_q_prio,
                                   (unsigned char *)g_tx_q[i].tx_q_prio_addr, sizeof(g_tx_q[i].tx_q_prio));
    g_auc_hif_ops.bt_hi_write_sram((unsigned char *)&g_tx_q[i].tx_q_status,
                                   (unsigned char *)g_tx_q[i].tx_q_status_addr, sizeof(g_tx_q[i].tx_q_status));
#else
    if (FAMILY_TYPE_IS_W1U(amlbt_if_type))
    {
        g_auc_hif_ops.bt_hi_write_word((unsigned int)g_tx_q[i].tx_q_dev_index_addr, g_tx_q[i].tx_q_dev_index);
        g_auc_hif_ops.bt_hi_write_word((unsigned int)g_tx_q[i].tx_q_prio_addr, g_tx_q[i].tx_q_prio);
        g_auc_hif_ops.bt_hi_write_word((unsigned int)g_tx_q[i].tx_q_status_addr, g_tx_q[i].tx_q_status);
    }

#endif

    AMLBT_DBG("%s, Actual length:%d\n", __func__, len);

    //g_auc_hif_ops.bt_hi_write_sram((unsigned char *)tx_q_info,
    //                                   (unsigned char *)g_tx_q[0].tx_q_status_addr, sizeof(tx_q_info));

    //g_auc_hif_ops.bt_hi_write_sram((unsigned char *)&tx_q_info[8], (unsigned char *)g_tx_q[0].tx_q_dev_index_addr,
    //                               sizeof(int)*WF_SRAM_TX_Q_NUM*2);
    //g_auc_hif_ops.bt_hi_write_sram((unsigned char *)tx_q_info, (unsigned char *)g_tx_q[0].tx_q_status_addr,
    //                               sizeof(int)*WF_SRAM_TX_Q_NUM);
    AMLBT_DBG("((%#x,%#x,%#x,%#x,%#x) \n", i, g_tx_q[i].tx_q_status, g_tx_q[i].tx_q_prio, len, g_tx_q[i].tx_q_dev_index);
}

static unsigned int amlbt_usb_recv_hci_event(unsigned char *buff, unsigned int cnt)
{
    unsigned int len = 0;
    unsigned int i = 0;

    if (FAMILY_TYPE_IS_W1U(amlbt_if_type))
    {
        g_event_fifo->w = (unsigned char *)(unsigned long)g_auc_hif_ops.bt_hi_read_word(WF_SRAM_EVT_FIFO_W_ADDR);
    }

    g_event_fifo->w += (WF_SRAM_EVENT_Q_ADDR);

    AMLBT_DBG("%s\n", __func__);

    AMLBT_DBG("r:%#lx,w:%#lx\n", (unsigned long)g_event_fifo->r, (unsigned long)g_event_fifo->w);

    if (FAMILY_TYPE_IS_W1U(amlbt_if_type))
    {
        len = gdsl_read_data(g_event_fifo, buff, cnt);
    }

    AMLBT_DBG("read event fifo len %d\n", len);
    if (len)
    {
        AMLBT_DBG("event data:\n");
        for (i = 0; i < len; i++)
        {
            AMLBT_DBG("%#x|", buff[i]);
        }
    }

    return len;
}

void amlbt_usb_fifo_init(void)
{
    unsigned int st_reg = 0;

    AMLBT_DBG("%s\n", __func__);

    //g_auc_hif_ops.bt_hi_write_word((unsigned int)0x00a0d0e4, 0x8000007f);
    if (FAMILY_TYPE_IS_W1U(amlbt_if_type))
    {
        st_reg = g_auc_hif_ops.bt_hi_read_word(WF_SRAM_FW_DRIVER_STATUS_ADDR);
        st_reg |= WF_SRAM_FD_INIT_FLAG;
        g_auc_hif_ops.bt_hi_write_word(WF_SRAM_FW_DRIVER_STATUS_ADDR, st_reg);
    }

    amlbt_usb_rx_type_fifo_init();
    amlbt_usb_hci_cmd_fifo_init();
    amlbt_usb_hci_tx_data_init();
    amlbt_usb_hci_evt_fifo_init();
    amlbt_usb_fw_recv_fifo_init();
    memset(BT_fwICCM, 0, BT_CCM_SIZE);
    memset(BT_fwDCCM, 0, BT_CCM_SIZE);
    memset(type, 0, sizeof(type));
    g_fw_data_fifo = gdsl_fifo_init(BT_CCM_SIZE, BT_fwDCCM);
    g_fw_data_fifo->w = BT_fwDCCM;
    g_fw_data_fifo->r = BT_fwDCCM;
    g_fw_evt_fifo = gdsl_fifo_init(BT_CCM_SIZE, BT_fwICCM);
    g_fw_evt_fifo->w = BT_fwICCM;
    g_fw_evt_fifo->r = BT_fwICCM;

    g_fw_type_fifo = gdsl_fifo_init(sizeof(type), (unsigned char *)type);
    g_fw_type_fifo->w = (unsigned char *)type;
    g_fw_type_fifo->r = (unsigned char *)type;

    st_reg &= ~(WF_SRAM_FD_INIT_FLAG);
    if (FAMILY_TYPE_IS_W1U(amlbt_if_type))
    {
        g_auc_hif_ops.bt_hi_write_word(WF_SRAM_FW_DRIVER_STATUS_ADDR, st_reg);
    }

}

void amlbt_usb_fifo_deinit(void)
{
    unsigned int st_reg = 0;

    if (FAMILY_TYPE_IS_W1U(amlbt_if_type))
    {
        st_reg = g_auc_hif_ops.bt_hi_read_word(WF_SRAM_FW_DRIVER_STATUS_ADDR);
        st_reg |= WF_SRAM_FD_INIT_FLAG;
        g_auc_hif_ops.bt_hi_write_word(WF_SRAM_FW_DRIVER_STATUS_ADDR, st_reg);
    }

    amlbt_usb_rx_type_fifo_deinit();
    amlbt_usb_hci_cmd_fifo_deinit();
    amlbt_usb_hci_tx_data_deinit();
    amlbt_usb_hci_evt_fifo_deinit();
    amlbt_usb_fw_recv_fifo_deinit();
    gdsl_fifo_deinit(g_fw_data_fifo);
    gdsl_fifo_deinit(g_fw_evt_fifo);
    gdsl_fifo_deinit(g_fw_type_fifo);
    g_fw_data_fifo = 0;
    g_fw_evt_fifo = 0;
    g_fw_type_fifo = 0;
    st_reg &= ~(WF_SRAM_FD_INIT_FLAG);
    if (FAMILY_TYPE_IS_W1U(amlbt_if_type))
    {
        g_auc_hif_ops.bt_hi_write_word(WF_SRAM_FW_DRIVER_STATUS_ADDR, st_reg);
    }

}

static void amlbt_usb_init(void)
{
    AMLBT_DBG("%s\n", __func__);
    if (!download_fw)
    {
        amlbt_usb_fifo_init();
    }
    printk("%s set the semaphore\n", __func__);
    printk("%s start read fw thread\n", __func__);
    mutex_init(&fw_type_fifo_mutex);
    mutex_init(&fw_evt_fifo_mutex);
    mutex_init(&fw_data_fifo_mutex);
    check_fw_rx_task = kthread_run(amlbt_usb_check_fw_rx, NULL, "check_fw_rx_thread");
    if (!check_fw_rx_task) {
        printk("start read fw task fail\n");
        return;
    }
    AMLBT_DBG("%s end\n", __func__);
}

static void amlbt_usb_deinit(void)
{
    AMLBT_DBG("%s\n", __func__);
    if (check_fw_rx_task)
    {
        kthread_stop(check_fw_rx_task);
        check_fw_rx_task = NULL;
    }
    AMLBT_DBG("%s end\n", __func__);
}

static void amlbt_usb_reset(void)
{
    memset(p_acl_buf, 0, sizeof(p_acl_buf));
    memset(type, 0, sizeof(type));
    evt_state = 0;
}

void amlbt_usb_firmware_check(void)
{
    unsigned int offset = 0;
    unsigned int st_reg = 0;
    unsigned int iccm_base_addr = BT_ICCM_AHB_BASE + BT_ICCM_ROM_LEN;
    unsigned int dccm_base_addr = BT_DCCM_AHB_BASE;
    unsigned int *p_check = NULL;
    uint32_t fw_iccmLen = 0;
    uint8_t *fw_iccmBuf = NULL;
    uint32_t fw_dccmLen = 0;
    uint8_t *fw_dccmBuf = NULL;

    fw_iccmLen = BT_CCM_SIZE;
    fw_iccmBuf = BT_fwICCM;
    fw_dccmLen = BT_CCM_SIZE;
    fw_dccmBuf = BT_fwDCCM;

    printk("iccm check:\n");

    iccm_base_addr = BT_ICCM_AHB_BASE + BT_ICCM_ROM_LEN;
    p_check = (unsigned int *)fw_iccmBuf;
    for (offset = 0; offset < (fw_iccmLen / 4); offset++)
    {
        if (FAMILY_TYPE_IS_W1U(amlbt_if_type))
        {
            st_reg = g_auc_hif_ops.bt_hi_read_word(iccm_base_addr);
        }

        iccm_base_addr += 4;
        if (st_reg != *p_check)
        {
            printk("iccm download data:%#x, raw data:%#x\n", st_reg, *p_check);
            printk("iccm no match, offset = %#x!\n", offset);
            break;
        }
        p_check++;
    }
    printk("iccm check size : %#x\n", offset);

    if (offset == (fw_iccmLen / 4))
    {
        printk("iccm check pass\n");
    }

    //printk("dccm check:\n");

    dccm_base_addr = BT_DCCM_AHB_BASE;
    p_check = (unsigned int *)fw_dccmBuf;
    for (offset = 0; offset < fw_dccmLen / 4; offset++)
    {
        if (FAMILY_TYPE_IS_W1U(amlbt_if_type))
        {
            st_reg = g_auc_hif_ops.bt_hi_read_word(dccm_base_addr);
        }

        dccm_base_addr += 4;
        if (st_reg != *p_check)
        {
            printk("dccm download data:%#x, raw data:%#x\n", st_reg, *p_check);
            printk("dccm no match!\n");
            break;
        }
        p_check++;
    }
    printk("dccm check size : %#x\n", offset);
    if (offset == fw_dccmLen / 4)
    {
        printk("dccm check pass\n");
    }

}

#if AML_BT_ROM_CHECK
void amlbt_usb_rom_check(void)
{
    unsigned int offset = 0;
    unsigned long addr = 0;

    for (offset = 0; offset < 256*1024; offset += 512)
    {
        addr= (BT_ICCM_AHB_BASE + offset);
        g_auc_hif_ops.bt_hi_read_sram(read_buff, (unsigned char *)addr, 512);
        if (memcmp(read_buff, &bt_rom_code[offset], 512))
        {
            printk("amlbt_usb_rom_check fail,%#x \n", offset);
            printk("[%#x,%#x,%#x,%#x] \n", read_buff[0],read_buff[1],read_buff[2],read_buff[3]);
            printk("[%#x,%#x,%#x,%#x] \n", bt_rom_code[offset],bt_rom_code[offset+1],
                bt_rom_code[offset+2],bt_rom_code[offset+3]);
            return ;
        }
    }

    printk("amlbt_usb_rom_check pass,%#x \n", offset);
}
#endif

void amlbt_usb_write_firmware(unsigned char *buf, unsigned int len, unsigned int addr)
{
    unsigned int st_reg = 0;

    if (FAMILY_TYPE_IS_W1U(amlbt_if_type))
    {
        g_auc_hif_ops.bt_hi_write_sram(buf, (unsigned char *)(unsigned long)(WF_SRAM_RFU_ADDR), len);
        st_reg = g_auc_hif_ops.bt_hi_read_word(WF_SRAM_FW_DRIVER_STATUS_ADDR);
        st_reg |= WF_SRAM_FD_DOWNLOAD_W;
        g_auc_hif_ops.bt_hi_write_word(WF_SRAM_FW_DRIVER_STATUS_ADDR, st_reg);
        auc_send_cmd(addr, len);
        while (WF_SRAM_FD_DOWNLOAD_W & g_auc_hif_ops.bt_hi_read_word(WF_SRAM_FW_DRIVER_STATUS_ADDR))
        {

        }
    }

}

void amlbt_usb_download_firmware(void)
{
    unsigned int offset = 0;
    unsigned int remain_len = 0;
    unsigned int iccm_base_addr = BT_ICCM_AHB_BASE + BT_ICCM_ROM_LEN;
    unsigned int dccm_base_addr = BT_DCCM_AHB_BASE;
    unsigned int download_size = 0;
    uint32_t fw_iccmLen = 0;
    uint8_t *fw_iccmBuf = NULL;
    uint32_t fw_dccmLen = 0;
    uint8_t *fw_dccmBuf = NULL;

    fw_iccmLen = BT_CCM_SIZE;
    fw_iccmBuf = BT_fwICCM;
    fw_dccmLen = BT_CCM_SIZE;
    fw_dccmBuf = BT_fwDCCM;

    download_size = fw_iccmLen;

    //to do download bt fw
    AMLBT_DBG("bt_usb_download_firmware:iccm size %#x\n", download_size);
    if (FAMILY_TYPE_IS_W1U(amlbt_if_type))
    {
        g_auc_hif_ops.bt_hi_write_word(REG_DEV_RESET, (unsigned int)((BIT_CPU|BIT_MAC|BIT_PHY) << DEV_RESET_HW));    //pmu up
    }

    //g_auc_hif_ops.bt_hi_write_word(0xf03050, 0);    //ram power up
    remain_len = (download_size - offset);

    while (offset < download_size)
    {
        if (remain_len < WF_SRAM_FW_DOWNLOAD_SIZE)
        {
            amlbt_usb_write_firmware((unsigned char *)&fw_iccmBuf[offset], remain_len, iccm_base_addr);
            offset += remain_len;
            iccm_base_addr += remain_len;
            AMLBT_DBG("bt_usb_download_firmware iccm1 offset %#x, write_len %#x\n", offset, write_len);
        }
        else
        {
            amlbt_usb_write_firmware((unsigned char *)&fw_iccmBuf[offset], WF_SRAM_FW_DOWNLOAD_SIZE, iccm_base_addr);
            offset += WF_SRAM_FW_DOWNLOAD_SIZE;
            remain_len -= WF_SRAM_FW_DOWNLOAD_SIZE;
            iccm_base_addr += WF_SRAM_FW_DOWNLOAD_SIZE;
            AMLBT_DBG("bt_usb_download_firmware iccm2 offset %#x, write_len %#x\n", offset, write_len);
        }
        AMLBT_DBG("bt_usb_download_firmware iccm remain_len %#x\n", remain_len);
    }

    download_size = fw_dccmLen;

    //to do download bt fw
    AMLBT_DBG("bt_usb_download_firmware:dccm size %#x\n", download_size);
    offset = 0;
    remain_len = download_size;
    while (offset < download_size)
    {
        if (remain_len < WF_SRAM_FW_DOWNLOAD_SIZE)
        {
            amlbt_usb_write_firmware((unsigned char *)&fw_dccmBuf[offset], remain_len, dccm_base_addr);
            offset += remain_len;
            dccm_base_addr += remain_len;
            AMLBT_DBG("bt_usb_download_firmware dccm1 offset %#x, write_len %#x\n", offset, write_len);
        }
        else
        {
            amlbt_usb_write_firmware((unsigned char *)&fw_dccmBuf[offset], WF_SRAM_FW_DOWNLOAD_SIZE, dccm_base_addr);
            offset += WF_SRAM_FW_DOWNLOAD_SIZE;
            remain_len -= WF_SRAM_FW_DOWNLOAD_SIZE;
            dccm_base_addr += WF_SRAM_FW_DOWNLOAD_SIZE;
            AMLBT_DBG("bt_usb_download_firmware dccm2 offset %#x, write_len %#x\n", offset, write_len);
        }
        AMLBT_DBG("bt_usb_download_firmware dccm remain_len %#x \n", remain_len);
    }

}

static int amlbt_usb_char_open(struct inode *inode_p, struct file *file_p)
{
    unsigned char cnt=-1;
    close_state = 0;
    printk("%s, %#x, %#x\n", __func__, AML_BT_VERSION, download_fw);
    if (download_fw)
    {
        amlbt_usb_reset();
        amlbt_usb_init();
        download_fw = 1;
        download_end = 0;
        download_flag = 1;
        fw_cmd_w = 0;
        fw_cmd_r = 0;
        cnt = ((g_auc_hif_ops.bt_hi_read_word(REG_PMU_POWER_CFG) >> BIT_RF_NUM) & 0x03);
        printk("%s rf num:%#x\n", __func__, cnt);
    }
    init_completion(&usb_completion);
    return nonseekable_open(inode_p, file_p);
}
static void amlbt_usb_char_deinit(void);

static int amlbt_usb_char_close(struct inode *inode_p, struct file *file_p)
{
    printk("%s $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ %#x \n", __func__, download_fw);

    if (g_event_fifo != 0)
    {
        printk("event w:%p,r:%p\n", g_event_fifo->w, g_event_fifo->r);
    }

    if (download_fw)
    {
        amlbt_usb_deinit();
    }

    if (close_state == 2)
    {
        amlbt_usb_char_deinit();
    }
    close_state = 0;
    return 0;
}

static ssize_t amlbt_usb_char_read_fw(struct file *file_p,
                                      char __user *buf_p,
                                      size_t count,
                                      loff_t *pos_p)
{
    unsigned char cmd_opcode[2] = {0};
    //unsigned int n  = 0;
    static unsigned int fw_r_state = 0;
    static unsigned int fw_r_index = 0;

#if AML_BT_RW_DEBUG
    AMLBT_DBG("R FW:%#x", count);
#endif

    if (fw_r_state == 0)
    {
        while (g_lib_cmd_fifo->w == g_lib_cmd_fifo->r)
        {
            wait_for_completion(&usb_completion);
        }
        gdsl_fifo_get_data(g_lib_cmd_fifo, cmd_opcode, 2);
        cmd_cpt[4] = cmd_opcode[0];
        cmd_cpt[5] = cmd_opcode[1];
    }

    //AMLRW_DBG("RE:\n");
    //for (n = 0; n < 7; n++)
    {
        //    AMLRW_DBG("%#x|", cmd_cpt[n]);
    }
    //AMLRW_DBG("\n");

    if (copy_to_user(buf_p, &cmd_cpt[fw_r_index], count))
    {
        return -EFAULT;
    }

    switch (fw_r_state)
    {
        case 0:
            fw_r_state = 1;
            fw_r_index += count;
            break;
        case 1:
            fw_r_state = 2;
            fw_r_index += count;
            break;
        case 2:
        {
            fw_r_state = 0;
            fw_r_index = 0;
            cmd_index = 0xff;
            fw_cmd_r++;
            if (download_end)
            {
                amlbt_usb_reset();
                amlbt_usb_init();
                download_fw = 1;
                download_end = 0;
                download_flag = 1;
                printk("%s end \n", __func__);
            }
        }
        break;
    }

    return count;
}

int amlbt_usb_check_fw_rx(void *data)
{
    bool pkt_type;
    unsigned int read_len = 0;
    unsigned int data_index = 0;
    unsigned long tmp = 0;
    unsigned int type_size = 0;
    unsigned int data_size = 0;
    static unsigned char fw_read_buff[RX_Q_LEN] = {0};
    while (!kthread_should_stop())
    {
        if (close_state)
        {
            printk("%s R CLOSE\n", __func__);
            check_fw_rx_task = NULL;
            return 0;
        }
        if (!suspend_value)
        {
            if (FAMILY_TYPE_IS_W1U(amlbt_if_type))
            {
                g_rx_type_fifo->w = (unsigned char *)(unsigned long)g_auc_hif_ops.bt_hi_read_word(WF_SRAM_RX_TYPE_FIFO_W_ADDR);
            }
            AMLBT_DBG("%s g_rx_type_fifo->w:%#x", __func__, g_rx_type_fifo->w);
            g_rx_type_fifo->w += (WF_SRAM_RX_TYPE_FIFO_ADDR);
            pkt_type = g_rx_type_fifo->w != g_rx_type_fifo->r;
            AMLBT_DBG("%s g_rx_type_fifo->w:%#x g_rx_type_fifo->r:%#x", __func__, g_rx_type_fifo->w, g_rx_type_fifo->r);
            if (pkt_type)
            {
                AMLBT_DBG("%s g_rx_type_fifo->w:%#x g_rx_type_fifo->r:%#x", __func__, g_rx_type_fifo->w, g_rx_type_fifo->r);
                memset(type_buff, 0, RX_TYPE_FIFO_LEN);
                if (FAMILY_TYPE_IS_W1U(amlbt_if_type))
                {
                    type_size = gdsl_read_data(g_rx_type_fifo, type_buff, sizeof(int));
                }
                if (type_size == 0)
                {
                    printk("read type fifo err!!\n");
                    return -EFAULT;
                }
                if (type_buff[0] == HCI_EVENT_PKT)
                {
                    amlbt_usb_recv_hci_event(fw_read_buff, 4);
#if AML_BT_RW_DEBUG
                    printk("HEAD:[%#x,%#x,%#x,%#x]\n", fw_read_buff[0],
                           fw_read_buff[1], fw_read_buff[2], fw_read_buff[3]);
#endif
                    read_len = fw_read_buff[2];
                    read_len -= 1;
                    read_len = ((read_len + 3) & 0xFFFFFFFC);
                    amlbt_usb_recv_hci_event(&fw_read_buff[4], read_len);
                    mutex_lock(&fw_evt_fifo_mutex);
                    gdsl_fifo_copy_data(g_fw_evt_fifo, fw_read_buff, fw_read_buff[2] + 3);
                    mutex_unlock(&fw_evt_fifo_mutex);
#if AML_BT_RW_DEBUG
                    printk("read 1 r:%#x, w:%#x\n", (unsigned int)g_fw_evt_fifo->r, (unsigned int)g_fw_evt_fifo->w);
                    printk("{1 %#x|%#x|%#x|%#x}\n", g_fw_evt_fifo->r[0], g_fw_evt_fifo->r[1], g_fw_evt_fifo->r[2], g_fw_evt_fifo->r[3]);
#endif
                    tmp = (unsigned long)g_event_fifo->r;
                    tmp = ((tmp + 3) & 0xFFFFFFFC);
                    g_event_fifo->r = (unsigned char *)tmp;
                    if (FAMILY_TYPE_IS_W1U(amlbt_if_type))
                    {
                        g_auc_hif_ops.bt_hi_write_word(WF_SRAM_EVT_FIFO_R_ADDR,
                                                           ((unsigned int)(unsigned long)g_event_fifo->r - WF_SRAM_EVENT_Q_ADDR) & 0x7ff);
                    }
                }
                else if (type_buff[0] == HCI_ACLDATA_PKT)
                {
                    if (FAMILY_TYPE_IS_W1U(amlbt_if_type))
                    {
                        g_rx_fifo->w = (unsigned char *)(unsigned long)g_auc_hif_ops.bt_hi_read_word(WF_SRAM_RX_FIFO_W_ADDR);
                    }
                    g_rx_fifo->w += (WF_SRAM_RX_Q_FIFO_ADDR);

#if AML_BT_RW_DEBUG
                    printk("%s acl data r:%#lx, w:%#lx\n", __func__, (unsigned long)g_rx_fifo->r, (unsigned long)g_rx_fifo->w);
#endif
                    while (g_rx_fifo->r == g_rx_fifo->w)
                    {
                        if (close_state)
                        {
                            printk("R CLOSE 2\n");
                            //close_state = 0;
                            return -EFAULT;
                        }
                        if (FAMILY_TYPE_IS_W1U(amlbt_if_type))
                        {
                            g_rx_fifo->w = (unsigned char *)(unsigned long)g_auc_hif_ops.bt_hi_read_word(WF_SRAM_RX_FIFO_W_ADDR);
                        }
                        g_rx_fifo->w += (WF_SRAM_RX_Q_FIFO_ADDR);
                        AMLBT_DBG("rf2 r %#x, w %#x\n", (unsigned long)g_rx_fifo->r, (unsigned long)g_rx_fifo->w);
                    }
                    if (FAMILY_TYPE_IS_W1U(amlbt_if_type))
                    {
                        data_size = gdsl_read_data(g_rx_fifo, (unsigned char *)&data_index, 4);
                    }
#if AML_BT_RW_DEBUG
                    printk("ds:%#x,%#x\n", data_size, data_index);
#endif
                    if (data_size > 0)
                    {
                        if (FAMILY_TYPE_IS_W1U(amlbt_if_type))
                        {
                            g_auc_hif_ops.bt_hi_write_word(WF_SRAM_RX_FIFO_R_ADDR,
                                                               ((unsigned int)(unsigned long)g_rx_fifo->r - WF_SRAM_RX_Q_FIFO_ADDR) & 0x1f);
                            g_auc_hif_ops.bt_hi_read_sram(&fw_read_buff[0],
                                                              (unsigned char *)(unsigned long)(WF_SRAM_RX_Q_ADDR + data_index * RX_Q_LEN), 8);
                            read_len = ((fw_read_buff[7] << 8) | (fw_read_buff[6]));
                            read_len = ((read_len + 3) & 0xFFFFFFFC);
#if AML_BT_RW_DEBUG
                            printk("!%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x!\n", fw_read_buff[0],
                                   fw_read_buff[1], fw_read_buff[2], fw_read_buff[3], fw_read_buff[4], fw_read_buff[5],
                                   fw_read_buff[6], fw_read_buff[7]);
                            printk("r dh %#x\n", read_len);
#endif
                            g_auc_hif_ops.bt_hi_read_sram(&fw_read_buff[8],
                                                              (unsigned char *)(unsigned long)(WF_SRAM_RX_Q_ADDR + data_index * RX_Q_LEN + 8), read_len);
#if AML_BT_RW_DEBUG
                            printk("r dh end\n");
#endif
                        }
                        mutex_lock(&fw_data_fifo_mutex);
                        gdsl_fifo_copy_data(g_fw_data_fifo, fw_read_buff, (((fw_read_buff[7] << 8) | (fw_read_buff[6])) + 8));
                        mutex_unlock(&fw_data_fifo_mutex);
#if AML_BT_RW_DEBUG
                        printk("HEAD1:[%#x,%#x,%#x,%#x]\n", fw_read_buff[0],
                               fw_read_buff[1], fw_read_buff[2], fw_read_buff[3]);
                        printk("HEAD2:[%#x,%#x,%#x,%#x]\n", fw_read_buff[4],
                               fw_read_buff[5], fw_read_buff[6], fw_read_buff[7]);
                        printk("HEAD3:[%#x,%#x,%#x,%#x]\n", fw_read_buff[8],
                               fw_read_buff[9], fw_read_buff[10], fw_read_buff[11]);
                        printk("HEAD4:[%#x,%#x,%#x,%#x]\n", fw_read_buff[12],
                               fw_read_buff[13], fw_read_buff[14], fw_read_buff[15]);
#endif
                    }
                    else
                    {
                        printk("data size err!!\n");
                        return -EFAULT;
                    }
                }
                else
                {
                    printk("type error!\n");
                    return -EFAULT;
                }

                mutex_lock(&fw_type_fifo_mutex);
                gdsl_fifo_copy_data(g_fw_type_fifo, type_buff, type_size);
                mutex_unlock(&fw_type_fifo_mutex);

#if AML_BT_RW_DEBUG
                printk("%s TYPE:[%#x,%#x,%#x,%#x]\n", __func__, type_buff[0],
                       type_buff[4], type_buff[8], type_buff[12]);
#endif
                if (FAMILY_TYPE_IS_W1U(amlbt_if_type))
                {
                    g_auc_hif_ops.bt_hi_write_word(WF_SRAM_RX_TYPE_FIFO_R_ADDR,
                                                       ((unsigned int)(unsigned long)g_rx_type_fifo->r - WF_SRAM_RX_TYPE_FIFO_ADDR) & 0x1fff);
                }
                wake_up_interruptible(&poll_amlbt_queue);
            }
            else
            {
                usleep_range(3000, 3000);
            }
        }
    }
    printk("%s exit read fw rx thread\n", __func__);
    return 0;
}

static ssize_t amlbt_usb_char_read(struct file *file_p,
                                   char __user *buf_p,
                                   size_t count,
                                   loff_t *pos_p)
{
    unsigned char close_evt[7] = {0x04,0x0e,0x04,0x01,0x00,0x00,0x00};
    static unsigned int bt_type = 0;
    unsigned int fw_data = 0;
    unsigned int fw_evt = 0;
    static unsigned char host_read_buff[TX_Q_LEN] = {0};
    //AMLBT_DBG("R:%#x, r %#x, w %#x\n", count, (unsigned long)g_host_tx_fifo->r, (unsigned long)g_host_tx_fifo->w);
    AMLBT_DBG("R:%#x, %#x, %#x\n", count, fw_cmd_r, fw_cmd_w);

    AMLBT_DBG("%s start, count : %ld\n", __func__, count);
    AMLBT_DBG("fw recv fifo r:%#lx, w:%#lx\n", (unsigned long)g_rx_fifo->r, (unsigned long)g_rx_fifo->w);
    AMLBT_DBG("%s, data : %d\n", __func__, data);

    if (!download_fw || (fw_cmd_r < fw_cmd_w))
    {
        return amlbt_usb_char_read_fw(file_p, buf_p, count, pos_p);
    }

    if (close_state)
    {
        printk(" %s R CLOSE\n", __func__);
        //close_state = 0;
        return 0;
    }

#if AML_BT_RW_DEBUG
    printk("R:%#x\n", count);
#endif

    mutex_lock(&fw_evt_fifo_mutex);
    fw_evt = gdsl_fifo_used_size(g_fw_evt_fifo);
    mutex_unlock(&fw_evt_fifo_mutex);

    mutex_lock(&fw_data_fifo_mutex);
    fw_data = gdsl_fifo_used_size(g_fw_data_fifo);
    mutex_unlock(&fw_data_fifo_mutex);

    if (fw_evt == 0 && fw_data == 0)
    {
        printk("fifo to host fail \n");
        if (copy_to_user(buf_p, close_evt, 1))
        {
            printk("%s, copy_to_user error \n", __func__);
            return -EFAULT;
        }
        return 1;
    }
    AMLBT_DBG("recv(%#x)\n", evt_state);

    switch (evt_state)
        {
            case 0:     //read type
                evt_state = 1;
                mutex_lock(&fw_type_fifo_mutex);
                gdsl_fifo_get_data(g_fw_type_fifo, (unsigned char *)&bt_type, sizeof(bt_type));
                mutex_unlock(&fw_type_fifo_mutex);
                if (bt_type == HCI_EVENT_PKT)
                {
#if AML_BT_RW_DEBUG
                    printk("evt r:%#x, w:%#x\n", (unsigned int)g_fw_evt_fifo->r, (unsigned int)g_fw_evt_fifo->w);
#endif
                    mutex_lock(&fw_evt_fifo_mutex);
                    gdsl_fifo_get_data(g_fw_evt_fifo, host_read_buff, count);
                    mutex_unlock(&fw_evt_fifo_mutex);
                    AMLBT_DBG("%#x|", host_read_buff[0]);
                    if (copy_to_user(buf_p, host_read_buff, count))
                    {
                        printk("%s, copy_to_user error \n", __func__);
                        return -EFAULT;
                    }
                    //gdsl_fifo_update_r(g_fw_evt_fifo, count);
                }
                else if (bt_type == HCI_ACLDATA_PKT)
                {
                    mutex_lock(&fw_data_fifo_mutex);
                    gdsl_fifo_get_data(g_fw_data_fifo, host_read_buff, sizeof(bt_type));
                    mutex_unlock(&fw_data_fifo_mutex);
                    if (copy_to_user(buf_p, host_read_buff, count))
                    {
                        printk("%s, copy_to_user error \n", __func__);
                        return -EFAULT;
                    }
                    //gdsl_fifo_update_r(g_fw_data_fifo, sizeof(bt_type));    //data type cost 4 bytes
                }
                //gdsl_fifo_update_r(g_fw_type_fifo, sizeof(bt_type));
#if AML_BT_RW_DEBUG
                printk("RT:%#x\n", bt_type);
#endif
                break;
            case 1:                 // read header
                evt_state = 2;
                if (bt_type == HCI_EVENT_PKT)
                {
                    mutex_lock(&fw_evt_fifo_mutex);
                    gdsl_fifo_get_data(g_fw_evt_fifo, host_read_buff, count);
                    mutex_unlock(&fw_evt_fifo_mutex);
                    if (copy_to_user(buf_p, host_read_buff, count))
                    {
                        AMLBT_DBG("%s, copy_to_user error \n", __func__);
                        return -EFAULT;
                    }
#if AML_BT_RW_DEBUG
                    printk("%#x|%#x|%#x|%#x\n", host_read_buff[0], host_read_buff[1],
                           host_read_buff[2], host_read_buff[3]);
#endif
                    //gdsl_fifo_update_r(g_fw_evt_fifo, count);
                }
                else if (bt_type == HCI_ACLDATA_PKT)
                {
                    mutex_lock(&fw_data_fifo_mutex);
                    gdsl_fifo_get_data(g_fw_data_fifo, host_read_buff, count);
                    mutex_unlock(&fw_data_fifo_mutex);
#if AML_BT_RW_DEBUG
                    printk("%#x|%#x|%#x|%#x\n", host_read_buff[0], host_read_buff[1],
                           host_read_buff[2], host_read_buff[3]);
#endif
                    if (copy_to_user(buf_p, host_read_buff, count))
                    {
                        printk("%s, copy_to_user error \n", __func__);
                        return -EFAULT;
                    }
                    //gdsl_fifo_update_r(g_fw_data_fifo, count);
                }
                break;
            case 2:                 //read payload
                evt_state = 0;
                if (bt_type == HCI_EVENT_PKT)
                {
                    mutex_lock(&fw_evt_fifo_mutex);
                    gdsl_fifo_get_data(g_fw_evt_fifo, host_read_buff, count);
                    mutex_unlock(&fw_evt_fifo_mutex);
#if AML_BT_RW_DEBUG
                    printk("%#x|%#x|%#x|%#x\n", host_read_buff[0], host_read_buff[1],
                           host_read_buff[2], host_read_buff[3]);
#endif
                    if (copy_to_user(buf_p, host_read_buff, count))
                    {
                        AMLBT_DBG("%s, copy_to_user error \n", __func__);
                        return -EFAULT;
                    }
                    //gdsl_fifo_update_r(g_fw_evt_fifo, count);
                }
                else if (bt_type == HCI_ACLDATA_PKT)
                {
                    unsigned int offset = ((unsigned int)g_fw_data_fifo->r - (unsigned int)g_fw_data_fifo->base_addr);
                    if ((offset + count) >= g_fw_data_fifo->size)
                    {
                        mutex_lock(&fw_data_fifo_mutex);
                        gdsl_fifo_get_data(g_fw_data_fifo, host_read_buff, count);
                        mutex_unlock(&fw_data_fifo_mutex);
#if AML_BT_RW_DEBUG
                        printk("%#x|%#x|%#x|%#x|%#x|%#x|%#x|%#x\n", host_read_buff[0], host_read_buff[1],
                               host_read_buff[2], host_read_buff[3], host_read_buff[4],
                               host_read_buff[5], host_read_buff[6], host_read_buff[7]);
#endif
                        if (copy_to_user(buf_p, host_read_buff, count))
                        {
                            AMLBT_DBG("%s, copy_to_user error \n", __func__);
                            return -EFAULT;
                        }
                    }
                    else
                    {
                        if (copy_to_user(buf_p, g_fw_data_fifo->r, count))
                        {
                            AMLBT_DBG("%s, copy_to_user error \n", __func__);
                            return -EFAULT;
                        }
                        mutex_lock(&fw_data_fifo_mutex);
                        gdsl_fifo_update_r(g_fw_data_fifo, count);
                        mutex_unlock(&fw_data_fifo_mutex);
                    }
                }
                break;
            default:
                printk("%s, evt_state error!!\n", __func__);
                break;
        }
        AMLBT_DBG("R END\n");
        return count;
}


static ssize_t amlbt_usb_char_write_fw(struct file *file_p,
                                       const char __user *buf_p,
                                       size_t count,
                                       loff_t *pos_p)
{
    unsigned int n = 0;
    unsigned int size = sizeof(cmd) / sizeof(cmd[0]);
    unsigned char len  = 0;
    unsigned int offset = 0;
    unsigned int reg_value = 0;


    AMLBT_DBG("W_FW:%#x\n", count);

    if (count == 1)
    {
        return count;
    }

    memset(p_acl_buf, 0, HCI_MAX_FRAME_SIZE);

    if (copy_from_user(p_acl_buf, buf_p, count))
    {
        AMLBT_DBG("%s: Failed to get data from user space\n", __func__);
        return -EFAULT;
    }

    for (n = 0; n < size; n++)
    {
        if (!memcmp(p_acl_buf, cmd[n], 2))
        {
            cmd_index = n;
            break;
        }
    }

    if (n == size)
    {
        printk("CMD_I:%#x\n", cmd_index);
        for (n = 0; n < 11; n++)
        {
            printk("%#x|", p_acl_buf[n]);
        }
        printk("\n");
        printk("---------------cmd error!------------");
        return -EINVAL;
    }

    if (p_acl_buf[0] != 0xf3 && p_acl_buf[1] == 0xfe)
    {
        gdsl_fifo_copy_data(g_lib_cmd_fifo, p_acl_buf, 2);
    }

    if (p_acl_buf[0] == 0xf3 && p_acl_buf[1] == 0xfe)   //download fw
    {
        len = count - 7;
        offset = ((p_acl_buf[6] << 24) | (p_acl_buf[5] << 16) | (p_acl_buf[4] << 8) | p_acl_buf[3]);
        AMLRW_DBG("%#x,%#x,%#x\n", len, offset, dw_state);
        AMLRW_DBG("%#x,%#x,%#x,%#x\n", p_acl_buf[7], p_acl_buf[8], p_acl_buf[9], p_acl_buf[10]);
        if (offset == BT_ICCM_ROM_LEN)
        {
            //printk("W S\n");
            if (!download_flag)
            {
                if (FAMILY_TYPE_IS_W1U(amlbt_if_type))
                {
                    g_auc_hif_ops.bt_hi_write_word(REG_RAM_PD_SHUTDWONW_SW, 0); //rg_ram_pd_shutdown_sw
                }

            }
            iccm_base_addr = 0;
            dccm_base_addr = 0;
            dw_state = 0;
        }

        if (offset == DCCM_RAM_BASE)
        {
            dw_state = 1;
        }

        if (dw_state == 0)
        {
            //amlbt_usb_write_firmware(&p_acl_buf[7], len, iccm_base_addr);
            //printk("W S 1\n");
            memcpy(&BT_fwICCM[iccm_base_addr], &p_acl_buf[7], len);
            //printk("W S 2\n");
            iccm_base_addr += len;
        }
        else
        {
            //amlbt_usb_write_firmware(&p_acl_buf[7], len, dccm_base_addr);
            memcpy(&BT_fwDCCM[dccm_base_addr], &p_acl_buf[7], len);
            dccm_base_addr += len;
        }
    }
    else if (p_acl_buf[0] == 0xf1 && p_acl_buf[1] == 0xfe)
    {
        offset = ((p_acl_buf[6] << 24) | (p_acl_buf[5] << 16) | (p_acl_buf[4] << 8) | p_acl_buf[3]);
        reg_value = ((p_acl_buf[10] << 24) | (p_acl_buf[9] << 16) | (p_acl_buf[8] << 8) | p_acl_buf[7]);
        printk("WR:%#x,%#x\n", offset, reg_value);
        if (offset == 0xa7000c) //rf calibration
        {
            //amlbt_usb_firmware_check();
            if (!download_flag)
            {
                //QA temporary test use bt_en register
                reg_value = g_auc_hif_ops.bt_hi_read_word(0x00f0003c);
                AMLBT_DBG("0x00f0003c value:%#x \n", reg_value);
                g_auc_hif_ops.bt_hi_write_word(0x00f0003c, 0xc0000000);
                reg_value = g_auc_hif_ops.bt_hi_read_word(0x00f0003c);
                AMLBT_DBG("0x00f0003c after value:%#x \n", reg_value);
                amlbt_usb_download_firmware();
            }
            printk("W E %#x,%#x\n", iccm_base_addr, dccm_base_addr);
        }
        else if (offset == REG_PMU_POWER_CFG)
        {
            if (FAMILY_TYPE_IS_W1U(amlbt_if_type))
            {
                g_auc_hif_ops.bt_hi_write_word(REG_PMU_POWER_CFG, reg_value);
                printk("%s set rf num %#x", __func__, reg_value);
            }
        }
        else if (offset == REG_DEV_RESET)
        {
            download_end = 1;
            if (FAMILY_TYPE_IS_W1U(amlbt_if_type))
            {
                g_auc_hif_ops.bt_hi_write_word(REG_DEV_RESET, (unsigned int)((BIT_CPU|BIT_MAC|BIT_PHY) << DEV_RESET_SW));
                printk("%s end \n", __func__);
                printk("%s end 15:32,bt reg : %#x\n", __func__, g_auc_hif_ops.bt_hi_read_word(REG_DEV_RESET));
            }
        }
    }
    else if (p_acl_buf[0] == 0xf2 && p_acl_buf[1] == 0xfe)
    {
        offset = ((p_acl_buf[6] << 24) | (p_acl_buf[5] << 16) | (p_acl_buf[4] << 8) | p_acl_buf[3]);
        reg_value = ((p_acl_buf[10] << 24) | (p_acl_buf[9] << 16) | (p_acl_buf[8] << 8) | p_acl_buf[7]);
        printk("WC:%#x,%#x\n", offset, reg_value);
    }
#if 0
    else if (p_acl_buf[0] == 0x1a && p_acl_buf[1] == 0xfc)  //HCI_VSC_WRITE_BD_ADDR
    {
        memcpy(bt_addr, &p_acl_buf[3], sizeof(bt_addr));
        printk("ADDR:[%#x|%#x|%#x|%#x|%#x|%#x]",
               bt_addr[0], bt_addr[1], bt_addr[2], bt_addr[3], bt_addr[4], bt_addr[5]);
        download_end = 1;
    }
#endif
    fw_cmd_w++;
    complete(&usb_completion);
    return count;
}

static ssize_t amlbt_usb_char_write(struct file *file_p,
                                    const char __user *buf_p,
                                    size_t count,
                                    loff_t *pos_p)
{
    unsigned int i = 0;
    static unsigned int w_type = 0;

    AMLBT_DBG("W:%#x, %#x\n", count, download_fw);

    AMLBT_DBG("%s, count:%ld\n", __func__, count);

    if (count > HCI_MAX_FRAME_SIZE)
    {
        printk("count > HCI_MAX_FRAME_SIZE \n");
        return -EINVAL;
    }

    if (!download_fw)
    {
        return amlbt_usb_char_write_fw(file_p, buf_p, count, pos_p);
    }

#if AML_BT_RW_DEBUG
    printk("W:%#x\n", count);
#endif
    if (count == 1)	//host write hci type
    {
        get_user(w_type, buf_p);
        AMLBT_DBG("%s:get type %#x\n", __func__, w_type);
        //drv_gdsl_fifo_write_data(g_host_tx_fifo, (unsigned char *)&type, 4);
        //AMLBT_DBG("T:%#x, r %#x, w %#x\n", type, (unsigned long)g_host_tx_fifo->r, (unsigned long)g_host_tx_fifo->w);
        AMLBT_DBG("T:%#x\n", w_type);
        //if (w_type == HCI_ACLDATA_PKT)
        {
            //    bt_usb_update_tx_data();
        }
        return count;
    }

    //p_acl_buf = kmalloc(count, GFP_KERNEL);

    memset(p_acl_buf, 0, HCI_MAX_FRAME_SIZE);

    if (copy_from_user(p_acl_buf, buf_p, count))
    {
        AMLBT_DBG("%s: Failed to get data from user space\n", __func__);
        return -EFAULT;
    }

    AMLBT_DBG("%s:get hci raw data:\n", __func__);

#if AML_BT_RW_DEBUG
    if (count > 1 && w_type == HCI_COMMAND_PKT)
    {
        printk("c:");
        for (i = 0; i < 8; i++)
        {
            printk("%#x|", p_acl_buf[i]);
        }
        printk("\n");
    }
#endif

    AMLBT_DBG("\n");

    if (w_type == HCI_COMMAND_PKT)
    {
        if (count == 0x0f && p_acl_buf[0] == 0x27 && p_acl_buf[1] == 0xfc)   //close
        {
            close_state = 1;
            for (i = 0; i < 8; i++)
            {
                printk("%#x|", p_acl_buf[i]);
            }
            printk("\n");
            printk("W CLOSE\n");
            return count;
        }
        amlbt_usb_send_hci_cmd(p_acl_buf, count);
    }
    else if (w_type == HCI_ACLDATA_PKT)
    {
        amlbt_usb_send_hci_data(p_acl_buf, count);
    }

    AMLBT_DBG("W END\n");

    return count;
}

unsigned int btchr_poll(struct file *file, poll_table *wait)
{
    int mask = 0;
    unsigned int fw_data = 0;
    unsigned int fw_evt = 0;
    if (!download_fw)
    {
        return POLLIN | POLLRDNORM;
    }
    poll_wait(file, &poll_amlbt_queue, wait);
    mutex_lock(&fw_evt_fifo_mutex);
    fw_evt = gdsl_fifo_used_size(g_fw_evt_fifo);
    mutex_unlock(&fw_evt_fifo_mutex);

    mutex_lock(&fw_data_fifo_mutex);
    fw_data = gdsl_fifo_used_size(g_fw_data_fifo);
    mutex_unlock(&fw_data_fifo_mutex);
    if (fw_evt || fw_data)
    {
        mask |= POLLIN | POLLRDNORM;
    }
    return mask;
}


static struct file_operations amlbt_usb_fops =
{
    .open = amlbt_usb_char_open,
    .release = amlbt_usb_char_close,
    .read = amlbt_usb_char_read,
    .write = amlbt_usb_char_write,
    .poll = btchr_poll,
    //.unlocked_ioctl = btchr_ioctl,
};


static int amlbt_usb_char_init(void)
{
    int res = 0;
    struct device *dev;

    AMLBT_DBG("%s\n", __func__);

    //	init_timer(&r_timer);

    bt_char_class = class_create(THIS_MODULE, AML_BT_CHAR_DEVICE_NAME);
    if (IS_ERR(bt_char_class))
    {
        AMLBT_DBG("%s:Failed to create bt char class\n", __func__);
        return PTR_ERR(bt_char_class);
    }

    res = alloc_chrdev_region(&bt_devid, 0, 1, AML_BT_CHAR_DEVICE_NAME);
    if (res < 0)
    {
        AMLBT_DBG("%s:Failed to allocate bt char device\n", __func__);
        goto err_alloc;
    }

    dev = device_create(bt_char_class, NULL, bt_devid, NULL, AML_BT_CHAR_DEVICE_NAME);
    if (IS_ERR(dev))
    {
        AMLBT_DBG("%s:Failed to create bt char device\n", __func__);
        res = PTR_ERR(dev);
        goto err_create;
    }

    cdev_init(&bt_char_dev, &amlbt_usb_fops);
    res = cdev_add(&bt_char_dev, bt_devid, 1);
    if (res < 0)
    {
        AMLBT_DBG("%s:Failed to add bt char device\n", __func__);
        goto err_add;
    }
    //g_cdev = dev;
    AMLBT_DBG("%s end", __func__);
    return 0;

err_add:
    device_destroy(bt_char_class, bt_devid);
err_create:
    unregister_chrdev_region(bt_devid, 1);
err_alloc:
    class_destroy(bt_char_class);
    return res;
}

static void amlbt_usb_char_deinit(void)
{
    printk("%s \n", __func__);
    device_destroy(bt_char_class, bt_devid);
    class_destroy(bt_char_class);
    cdev_del(&bt_char_dev);
    unregister_chrdev_region(bt_devid, 1);
}

static int amlbt_sdio_cfg_pmu(bool is_power_on)
{
    unsigned int value_pmu_A12 = 0;
    unsigned int value_pmu_A13 = 0;
    unsigned int value_pmu_A14 = 0;
    unsigned int value_pmu_A15 = 0;
    unsigned int value_pmu_A16 = 0;
    unsigned int value_pmu_A17 = 0;
    unsigned int value_pmu_A18 = 0;
    unsigned int value_pmu_A20 = 0;
    unsigned int value_pmu_A22 = 0;
    unsigned int bt_pmu_status = 0;
    unsigned int host_req_status = 0;
    unsigned int value_aon_a15 = 0;
    unsigned int reg_addr_mapping_form_pmu_fsm = 0;
    unsigned int reg_data = 0;
    unsigned int bt_pmu_status_check = 0;
    unsigned int wait_count = 0;
    unsigned char pmu_fsm = 0;
    int ret = 0;

    /* set wifi keep alive, BIT(5)*/
    if (g_w1_hif_ops.hi_bottom_read8 == NULL)
    {
        printk("amlbt_sdio_cfg_pmu(): can't get g_auc_hif_ops interface!!!!!!!!!!\n");
        return (-1);
    }
    host_req_status = g_w1_hif_ops.hi_bottom_read8(SDIO_FUNC1, RG_BT_SDIO_PMU_HOST_REQ);
    printk("host_req_status = 0x%x\n", host_req_status);
    host_req_status |= BIT(5);
    g_w1_hif_ops.hi_bottom_write8(SDIO_FUNC1, RG_BT_SDIO_PMU_HOST_REQ, host_req_status);

    /* wake wifi fw firstly */
    if (host_wake_req != NULL)
    {
        printk("BT lock\n");
        aml_wifi_sdio_power_lock();
        while (host_wake_req() == 0)
        {
            msleep(10);
            printk("BT insmod, wake wifi failed\n");
        }
        aml_wifi_sdio_power_unlock();
        printk("BT unlock\n");
    }
    else
    {
        /* wifi doesn't insmod */
    }

    if (is_power_on)
    {
        value_pmu_A12 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A12);
        value_pmu_A13 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A13);
        value_pmu_A14 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A14);
        value_pmu_A15 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A15);
        value_pmu_A16 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A16);
        value_pmu_A17 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A17);
        value_pmu_A18 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A18);
        value_pmu_A20 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A20);
        value_pmu_A22 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A22);
        printk("BT power on: before write A12=0x%x, A13=0x%x, A14=0x%x, A15=0x%x, A16=0x%x, A17=0x%x, A18=0x%x, A20=0x%x, A22=0x%x\n",
               value_pmu_A12, value_pmu_A13, value_pmu_A14, value_pmu_A15, value_pmu_A16, value_pmu_A17, value_pmu_A18, value_pmu_A20,
               value_pmu_A22);

        /* set bt work flag && xosc/bbpll/ao_iso/pmu mask*/
        g_w1_hif_ops.bt_hi_write_word(RG_BT_PMU_A14, 0x1f);
        printk("BT power on:RG_BT_PMU_A14 = 0x%x\n", g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A14));

        /* release pmu fsm : bit0 */
        host_req_status = g_w1_hif_ops.hi_bottom_read8(SDIO_FUNC1, RG_BT_SDIO_PMU_HOST_REQ);
        host_req_status &= ~BIT(0);
        g_w1_hif_ops.hi_bottom_write8(SDIO_FUNC1, RG_BT_SDIO_PMU_HOST_REQ, host_req_status);
        msleep(10);

        /* reset bt, then release bt */
        g_w1_hif_ops.bt_hi_write_word(RG_BT_PMU_A17, 0x700);
        msleep(1);
        printk("BT power on:RG_BT_PMU_A17 = 0x%x\n", g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A17));
        g_w1_hif_ops.bt_hi_write_word(RG_BT_PMU_A20, 0x0);
        msleep(1);
        printk("BT power on:RG_BT_PMU_A20 = 0x%x\n", g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A20));
        //g_auc_hif_ops.bt_hi_write_word(RG_BT_PMU_A12, value_pmu_A12|0xc0);
        //msleep(1);
        //printk("BT power on:RG_BT_PMU_A12 = 0x%x\n", g_auc_hif_ops.bt_hi_read_word(RG_BT_PMU_A12));
        g_w1_hif_ops.bt_hi_write_word(RG_BT_PMU_A18, 0x1700);
        printk("BT power on: %s, line=%d\n", __func__, __LINE__);
        msleep(1);
        printk("BT power on:RG_BT_PMU_A18 = 0x%x\n", g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A18));
        g_w1_hif_ops.bt_hi_write_word(RG_BT_PMU_A22, 0x704);
        msleep(10);
        printk("BT power on:RG_BT_PMU_A22 = 0x%x\n", g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A22));

        /* rg_bb_reset_man */
        value_pmu_A12 = value_pmu_A12 & 0xffffff3f;
        g_w1_hif_ops.bt_hi_write_word(RG_BT_PMU_A12, value_pmu_A12 | 0x80);
        msleep(1);
        g_w1_hif_ops.bt_hi_write_word(RG_BT_PMU_A12, value_pmu_A12 | 0xc0);
        msleep(1);
        g_w1_hif_ops.bt_hi_write_word(RG_BT_PMU_A12, value_pmu_A12 | 0x80);
        msleep(1);
        g_w1_hif_ops.bt_hi_write_word(RG_BT_PMU_A12, value_pmu_A12 | 0x00);
        msleep(1);
        printk("BT power on:RG_BT_PMU_A12 = 0x%x\n", g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A12));

        /* set bt pmu fsm to PMU_PWR_OFF */
        host_req_status = g_w1_hif_ops.hi_bottom_read8(SDIO_FUNC1, RG_BT_SDIO_PMU_HOST_REQ);
        host_req_status |= (PMU_PWR_OFF << 1) | BIT(0);
        g_w1_hif_ops.hi_bottom_write8(SDIO_FUNC1, RG_BT_SDIO_PMU_HOST_REQ, host_req_status);

        /* release bt pmu fsm */
        host_req_status = g_w1_hif_ops.hi_bottom_read8(SDIO_FUNC1, RG_BT_SDIO_PMU_HOST_REQ);
        host_req_status &= ~(0x1f);
        g_w1_hif_ops.hi_bottom_write8(SDIO_FUNC1, RG_BT_SDIO_PMU_HOST_REQ, host_req_status);
        msleep(20);

        bt_pmu_status = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A15);
        printk("%s bt_pmu_status:0x%x\n", __func__, bt_pmu_status);

        /* wait bt pmu fsm to PMU_ACT_MODE*/
        while ((bt_pmu_status & 0xF) != PMU_ACT_MODE)
        {
            msleep(5);
            printk("%s bt_pmu_status:0x%x\n", __func__, bt_pmu_status);
            bt_pmu_status = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A15);
        }

        value_pmu_A12 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A12);
        value_pmu_A13 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A13);
        value_pmu_A14 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A14);
        value_pmu_A15 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A15);
        value_pmu_A16 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A16);
        value_pmu_A17 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A17);
        value_pmu_A18 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A18);
        value_pmu_A20 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A20);
        value_pmu_A22 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A22);
        printk("BT power on: after write A12=0x%x, A13=0x%x, A14=0x%x, A15=0x%x, A16=0x%x, A17=0x%x, A18=0x%x, A20=0x%x, A22=0x%x\n",
               value_pmu_A12, value_pmu_A13, value_pmu_A14, value_pmu_A15, value_pmu_A16, value_pmu_A17, value_pmu_A18, value_pmu_A20,
               value_pmu_A22);
    }
    else    // turn off bt
    {
        value_pmu_A16 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A16);

        value_pmu_A16 |= (0x1 << 30); //bit30 = 1, make sure can't enter power down when turn off BT
        g_w1_hif_ops.bt_hi_write_word(RG_BT_PMU_A16, value_pmu_A16);
        printk("Enable RG_BT_PMU_A16 BIT30, FW can't enter power down!\n");

        value_pmu_A12 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A12);
        value_pmu_A13 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A13);
        value_pmu_A14 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A14);
        value_pmu_A15 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A15);

        value_pmu_A17 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A17);
        value_pmu_A18 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A18);
        value_pmu_A20 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A20);
        value_pmu_A22 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A22);
        value_aon_a15 = g_w1_hif_ops.bt_hi_read_word(RG_AON_A15);

        printk("BT power off: before write A12=0x%x, A13=0x%x, A14=0x%x, A15=0x%x, A16=0x%x, A17=0x%x, A18=0x%x, A20=0x%x, A22=0x%x, aon_a15=0x%x\n",
               value_pmu_A12, value_pmu_A13, value_pmu_A14, value_pmu_A15, value_pmu_A16, value_pmu_A17, value_pmu_A18, value_pmu_A20,
               value_pmu_A22, value_aon_a15);

        //if (value_pmu_A15 == 8)
        {
            value_pmu_A16 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A16);
            value_pmu_A16 &= 0xFffffffE; //bit0 =0
            g_w1_hif_ops.bt_hi_write_word(RG_BT_PMU_A16, value_pmu_A16);
            msleep(10);

            value_pmu_A16 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A16);
            printk("RG_BT_PMU_A16_0 = 0x%x\n", value_pmu_A16);

            value_pmu_A16 &= 0xFffffffD; //bit1 =0
            g_w1_hif_ops.bt_hi_write_word(RG_BT_PMU_A16, value_pmu_A16);
            msleep(10);

            if (value_pmu_A15 == 8)
            {
                printk("Is sleep mode, wakeup first!\n");
                value_pmu_A16 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A16);
                printk("RG_BT_PMU_A16_1 = 0x%x\n", value_pmu_A16);
                value_pmu_A16 |= 0x2; //bit1 = 1
                g_w1_hif_ops.bt_hi_write_word(RG_BT_PMU_A16, value_pmu_A16);
                msleep(10);
            }

            value_pmu_A16 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A16);
            printk("RG_BT_PMU_A16_2 = 0x%x\n", value_pmu_A16);
        }

        value_pmu_A16 &= 0x7fffffff;
        g_w1_hif_ops.bt_hi_write_word(RG_BT_PMU_A16, value_pmu_A16);
        msleep(1);
        printk("BT power off:RG_BT_PMU_A16 = 0x%x\n", g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A16));

        printk("Check whether is active mode\n");
        bt_pmu_status = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A15);
        bt_pmu_status_check = bt_pmu_status & 0x0f;
        while (bt_pmu_status_check != PMU_ACT_MODE)
        {
            msleep(10);
            bt_pmu_status = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A15);
            printk("wait wakeup, RG_BT_PMU_A15 = 0x%x\n", bt_pmu_status);
            wait_count++;

            if (wait_count > 50)    //
            {
#if 0
                /* set bt pmu fsm to PMU_SLEEP_MODE */
                printk("Force BT power off\n");
                host_req_status = g_auc_hif_ops.hi_bottom_read8(SDIO_FUNC1, RG_BT_SDIO_PMU_HOST_REQ);
                host_req_status |= (PMU_SLEEP_MODE << 1) | BIT(0);
                g_auc_hif_ops.hi_bottom_write8(SDIO_FUNC1, RG_BT_SDIO_PMU_HOST_REQ, host_req_status);

                printk("Force BT power on\n");
                /* release pmu fsm : bit0 */
                host_req_status = g_auc_hif_ops.hi_bottom_read8(SDIO_FUNC1, RG_BT_SDIO_PMU_HOST_REQ);
                host_req_status &= ~BIT(0);
                g_auc_hif_ops.hi_bottom_write8(SDIO_FUNC1, RG_BT_SDIO_PMU_HOST_REQ, host_req_status);
                msleep(10);
#else
                // add workaroud for pmu lock, when pmu lock trig the bnd let pmu fsm switch to next state.
                pmu_fsm = (g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A15) & 0xf);

                //if (pmu_fsm == PMU_SLEEP_MODE)
                //{
                //	value_pmu_A16 = g_auc_hif_ops.bt_hi_read_word(RG_BT_PMU_A16);
                //	value_pmu_A16 |= 0x2;
                //	g_auc_hif_ops.bt_hi_write_word(RG_BT_PMU_A16, value_pmu_A16);
                //}
                //else
                {
                    printk("trig pmu bnd to unlock pmu fsm, pmu_state=0x%x\n", pmu_fsm);
                    //if ((pmu_fsm == PMU_PWR_OFF) || (pmu_fsm == PMU_PWR_XOSC))
                    //	;                               // pmu fsm should not be 0 or 1 when chip on.
                    if (pmu_fsm == PMU_ACT_MODE)       // act mode wait cpu start
                        reg_addr_mapping_form_pmu_fsm = RG_BT_PMU_A11;
                    else
                        reg_addr_mapping_form_pmu_fsm = CHIP_BT_PMU_REG_BASE + ((pmu_fsm - 1) * 4);

                    reg_data = g_w1_hif_ops.bt_hi_read_word(reg_addr_mapping_form_pmu_fsm);
                    g_w1_hif_ops.bt_hi_write_word(reg_addr_mapping_form_pmu_fsm, ((reg_data & 0x7fffffff) + 1));
                }
                msleep(50); // wait pmu wake
                //g_funcs.aml_pwr_pwrsave_wake(g_power_down_domain);
#endif
                break;
            }
        }

        /* rg_bb_reset_man */
        g_w1_hif_ops.bt_hi_write_word(RG_BT_PMU_A12, value_pmu_A12 | 0x100c0);
        msleep(1);

        /* reset bt */
        g_w1_hif_ops.bt_hi_write_word(RG_BT_PMU_A22, 0x707);
        msleep(1);
        printk("BT power off:RG_BT_PMU_A22 = 0x%x\n", g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A22));
        g_w1_hif_ops.bt_hi_write_word(RG_BT_PMU_A18, 0x1787);
        msleep(1);
        printk("BT power off:RG_BT_PMU_A18 = 0x%x\n", g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A18));
        g_w1_hif_ops.bt_hi_write_word(RG_BT_PMU_A20, 0x0);      //0x1f703007
        msleep(1);
        printk("BT power off:RG_BT_PMU_A20 = 0x%x\n", g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A20));
        g_w1_hif_ops.bt_hi_write_word(RG_BT_PMU_A17, 0x707);
        msleep(1);
        printk("BT power off:RG_BT_PMU_A17 = 0x%x\n", g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A17));

        /* clear bt work flag for coex */
        //value_pmu_A16 &= 0x7fffffff;
        //g_auc_hif_ops.bt_hi_write_word(RG_BT_PMU_A16, value_pmu_A16);
        //printk("BT power off:RG_BT_PMU_A16 = 0x%x\n", g_auc_hif_ops.bt_hi_read_word(RG_BT_PMU_A16));

        value_pmu_A12 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A12);
        value_pmu_A13 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A13);
        value_pmu_A14 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A14);
        value_pmu_A15 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A15);
        value_pmu_A16 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A16);
        value_pmu_A17 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A17);
        value_pmu_A18 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A18);
        value_pmu_A20 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A20);
        value_pmu_A22 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A22);
        printk("BT power off: after write A12=0x%x, A13=0x%x, A14=0x%x, A15=0x%x, A16=0x%x, A17=0x%x, A18=0x%x, A20=0x%x, A22=0x%x\n",
               value_pmu_A12, value_pmu_A13, value_pmu_A14, value_pmu_A15, value_pmu_A16, value_pmu_A17, value_pmu_A18, value_pmu_A20,
               value_pmu_A22);

        /* clear bt work flag && mask */
        g_w1_hif_ops.bt_hi_write_word(RG_BT_PMU_A14, 0x0);
        printk("BT power off: %s, line=%d\n", __func__, __LINE__);
        //printk("BT power off:RG_BT_PMU_A14 = 0x%x\n", g_auc_hif_ops.bt_hi_read_word(RG_BT_PMU_A14));

        /* set bt pmu fsm to PMU_SLEEP_MODE */
        host_req_status = g_w1_hif_ops.hi_bottom_read8(SDIO_FUNC1, RG_BT_SDIO_PMU_HOST_REQ);
        host_req_status |= (PMU_SLEEP_MODE << 1) | BIT(0);
        g_w1_hif_ops.hi_bottom_write8(SDIO_FUNC1, RG_BT_SDIO_PMU_HOST_REQ, host_req_status);
        printk("BT power off: %s, line=%d\n", __func__, __LINE__);
    }

    /* clear wifi keep alive, BIT(5)*/
    host_req_status = g_w1_hif_ops.hi_bottom_read8(SDIO_FUNC1, RG_BT_SDIO_PMU_HOST_REQ);
    host_req_status &= ~BIT(5);
    g_w1_hif_ops.hi_bottom_write8(SDIO_FUNC1, RG_BT_SDIO_PMU_HOST_REQ, host_req_status);

    reg_config_complete = 1;

    return ret;
}

static int amlbt_sdio_fops_open(struct inode *inode, struct file *file)
{
    if (FAMILY_TYPE_IS_W1(amlbt_if_type)&&
        !reg_config_complete)
    {
        printk("%s reg_config_complete is %d return\n",
            __func__, reg_config_complete);
        return -EFAULT;
    }
    if (FAMILY_TYPE_IS_W1(amlbt_if_type))
    {
        unsigned char cnt = ((g_w1_hif_ops.bt_hi_read_word(REG_PMU_POWER_CFG) >> BIT_RF_NUM) & 0x03);
        printk("%s bt opened rf num:%#x\n", __func__, cnt);
        return nonseekable_open(inode, file);
    }

    printk("%s bt opened rf num:%#x\n", __func__, rf_num);
    return nonseekable_open(inode, file);
}

static int amlbt_sdio_fops_close(struct inode *inode, struct file *file)
{
    printk("%s BT closed\n", __func__);
    return 0;
}


static ssize_t amlbt_sdio_char_read(struct file *file_p,
                                 char __user *buf_p,
                                 size_t count,
                                 loff_t *pos_p)
{
    if (FAMILY_TYPE_IS_W1(amlbt_if_type))
    {
        unsigned char cnt = (g_w1_hif_ops.bt_hi_read_word(REG_PMU_POWER_CFG)  >> BIT_RF_NUM);
        printk("%s rf num:%#x\n", __func__, cnt);
        if (copy_to_user(buf_p, &cnt, 1))
        {
            return -EFAULT;
        }
    }
    return count;
}


static ssize_t amlbt_sdio_char_write(struct file *file_p,
                                    const char __user *buf_p,
                                    size_t count,
                                    loff_t *pos_p)
{
    unsigned int offset = 0;
    unsigned int reg_value = 0;
    if (copy_from_user(p_acl_buf, buf_p, count))
    {
        AMLBT_DBG("%s: Failed to get data from user space\n", __func__);
        return -EFAULT;
    }

    offset = ((p_acl_buf[3] << 24) | (p_acl_buf[2] << 16) | (p_acl_buf[1] << 8) | p_acl_buf[0]);
    reg_value = ((p_acl_buf[7] << 24) | (p_acl_buf[6] << 16) | (p_acl_buf[5] << 8) | p_acl_buf[4]);
    printk("WR:%#x,%#x\n", offset, reg_value);
    if (offset == REG_PMU_POWER_CFG)
    {
        if (FAMILY_TYPE_IS_W1U(amlbt_if_type))
        {
            rf_num = reg_value >> BIT_RF_NUM;
            printk("%s set rf num %#x", __func__, rf_num);
        }
    }
    return count;
}


const struct file_operations amlbt_sdio_fops =
{
    .open       = amlbt_sdio_fops_open,
    .release    = amlbt_sdio_fops_close,
    .read       = amlbt_sdio_char_read,
    .write      = amlbt_sdio_char_write,
    .poll       = NULL,
    .unlocked_ioctl = NULL,
    .fasync     = NULL
};


int  amlbt_sdio_hw_init(void)
{
    int ret = 0;
    int cnt = 0;

    //amlwifi_set_sdio_host_clk(50000000);

    if (!g_sdio_driver_insmoded)
    {
        printk("===start register sdio common driver through bt sdio driver===\n");
        aml_sdio_init();
        msleep(10);

        while (!g_sdio_after_porbe)
        {
            printk("waiting sdio common driver probe complete\n");
            msleep(10);
            if ((cnt++) > 500)
            {
                return (-1);
            }
        }
    }

    //if (!strcmp(chip_name, "aml_w1"))
    {
        while (wifi_in_insmod)
        {
            printk("WIFI in insmod\n");
            msleep(10);
        }
    }

    ret = amlbt_sdio_cfg_pmu(BT_PWR_ON);

    return ret;
}


static int amlbt_sdio_init(void)
{
    int ret = 0;
    int cdevErr = 0;

    dev_t devID = MKDEV(amlbt_sdio_major, 0);
    printk("amlbt_sdio_init\n");

    ret = alloc_chrdev_region(&devID, 0, 1, AML_BT_NOTE);
    if (ret)
    {
        pr_err("fail to allocate chrdev\n");
        return ret;
    }

    amlbt_sdio_major = MAJOR(devID);
    printk("major number:%d\n", amlbt_sdio_major);
    cdev_init(&amlbt_sdio_cdev, &amlbt_sdio_fops);
    amlbt_sdio_cdev.owner = THIS_MODULE;

    cdevErr = cdev_add(&amlbt_sdio_cdev, devID, amlbt_sdio_devs);
    if (cdevErr)
        goto error;

    printk("%s driver(major %d) installed.\n",
           "BT_sdiodev", amlbt_sdio_major);

    amlbt_sdio_class = class_create(THIS_MODULE, AML_BT_NOTE);
    if (IS_ERR(amlbt_sdio_class))
    {
        pr_err("class create fail, error code(%ld)\n",
               PTR_ERR(amlbt_sdio_class));
        goto err1;
    }

    amlbt_sdio_dev = device_create(amlbt_sdio_class, NULL, devID, NULL, AML_BT_NOTE);
    if (IS_ERR(amlbt_sdio_dev))
    {
        pr_err("device create fail, error code(%ld)\n",
               PTR_ERR(amlbt_sdio_dev));
        goto err2;
    }

    printk("%s: BT_major %d\n", __func__, amlbt_sdio_major);
    printk("%s: devID %d\n", __func__, devID);

    return 0;

err2:
    if (amlbt_sdio_class)
    {
        class_destroy(amlbt_sdio_class);
        amlbt_sdio_class = NULL;
    }

err1:

error:
    if (cdevErr == 0)
        cdev_del(&amlbt_sdio_cdev);

    if (ret == 0)
        unregister_chrdev_region(devID, amlbt_sdio_devs);

    return -1;
}


static int amlbt_sdio_insmod(void)
{
    int ret = 0;
    printk("BTAML version:%#x\n", AML_BT_VERSION);
    printk("++++++bt driver insmod start.++++++\n");
    reg_config_complete = 0;
    if (FAMILY_TYPE_IS_W1(amlbt_if_type))
    {
        set_wifi_bt_sdio_driver_bit(REGISTER_BT_SDIO, BT_BIT);
        ret = amlbt_sdio_hw_init();
        if (ret)
        {
            pr_err("%s: bt driver init failed!\n", __func__);
            return ret;
        }
    }
    ret = amlbt_sdio_init();
    if (ret)
    {
        pr_err("%s: amlbt_sdio_init failed!\n", __func__);
        return ret;
    }

    printk("------bt driver insmod end.------\n");

    return ret;
}

static void amlbt_sdio_exit(void)
{
    dev_t dev = MKDEV(amlbt_sdio_major, 0);

    if (amlbt_sdio_dev)
    {
        device_destroy(amlbt_sdio_class, dev);
        amlbt_sdio_dev = NULL;
    }
    if (amlbt_sdio_class)
    {
        class_destroy(amlbt_sdio_class);
        amlbt_sdio_class = NULL;
    }
    cdev_del(&amlbt_sdio_cdev);

    unregister_chrdev_region(dev, 1);

    printk("%s driver removed.\n", AML_BT_NOTE);
}


static void amlbt_sdio_rmmod(void)
{
    //if (amlbt_poweron == AML_SDIO_EN)
    {
        printk("++++++sdio bt driver rmmod start++++++\n");
        if (FAMILY_TYPE_IS_W1(amlbt_if_type))
        {
            while (wifi_in_insmod)
            {
                printk("WIFI in insmod\n");
                msleep(10);
            }
            amlbt_sdio_cfg_pmu(BT_PWR_OFF);
            set_wifi_bt_sdio_driver_bit(UNREGISTER_BT_SDIO, BT_BIT);
        }
        amlbt_sdio_exit();
        printk("------sdio bt driver rmmod end------\n");
    }
}

static int amlbt_sdio_probe(struct platform_device *dev)
{
    unsigned int ret = amlbt_sdio_insmod();
    if (FAMILY_TYPE_IS_W1(amlbt_if_type))
    {
        unsigned int reg_value = g_w1_hif_ops.bt_hi_read_word(RG_AON_A15);

        printk("%s RG_AON_A15:%#x\n", __func__, reg_value);

        reg_value &= ~(1 << 31);
        reg_value &= ~(1 << 30);
        g_w1_hif_ops.bt_hi_write_word(RG_AON_A15, reg_value);
        printk("RG_AON_A15:%#x", g_w1_hif_ops.bt_hi_read_word(RG_AON_A15));
    }
    return ret;
}

static int amlbt_sdio_remove(struct platform_device *dev)
{
    printk("%s \n", __func__);

    amlbt_sdio_rmmod();

    return 0;
}

static int amlbt_sdio_suspend(struct platform_device *dev, pm_message_t state)
{
    if (FAMILY_TYPE_IS_W1(amlbt_if_type))
    {
        unsigned int reg_value = g_w1_hif_ops.bt_hi_read_word(RG_AON_A15);
        printk("%s RG_AON_A15:%#x\n", __func__, reg_value);
        reg_value |= (1 << 31);
        g_w1_hif_ops.bt_hi_write_word(RG_AON_A15, reg_value);
        printk("RG_AON_A15:%#x", g_w1_hif_ops.bt_hi_read_word(RG_AON_A15));
    }
    return 0;
}

static int amlbt_sdio_resume(struct platform_device *dev)
{
    if (FAMILY_TYPE_IS_W1(amlbt_if_type))
    {
        unsigned int reg_value = g_w1_hif_ops.bt_hi_read_word(RG_AON_A15);
        printk("%s RG_AON_A15:%#x\n", __func__, reg_value);
        reg_value &= ~(1 << 31);
        g_w1_hif_ops.bt_hi_write_word(RG_AON_A15, reg_value);
        printk("RG_AON_A15:%#x", g_w1_hif_ops.bt_hi_read_word(RG_AON_A15));
    }
    return 0;
}

static void amlbt_sdio_shutdown(struct platform_device *dev)
{
    printk("%s \n", __func__);
    //unsigned int reg_value = g_w1_hif_ops.bt_hi_read_word(RG_AON_A15);
    //printk("%s RG_AON_A15:%#x\n", __func__, reg_value);
    //reg_value |= (1<<30);
    //g_w1_hif_ops.bt_hi_write_word(RG_AON_A15, reg_value);
    //printk("RG_AON_A15:%#x", g_w1_hif_ops.bt_hi_read_word(RG_AON_A15));
}


static void amlbt_dev_release(struct device *dev)
{
    return;
}

static int amlbt_usb_probe(struct platform_device *dev)
{
    int err = 0;

    //g_auc_hif_ops.bt_hi_write_word((unsigned int)0x00a0d0e4, 0x8000007f);
    //printk("%s, %#x", __func__, g_auc_hif_ops.bt_hi_read_word(0x00a0d0e4));
    printk("%s \n", __func__);
    err = amlbt_usb_char_init();
    if (err < 0)
    {
        /* usb register will go on, even bt char register failed */
        AMLBT_DBG("%s:Failed to register usb char device interfaces\n", __func__);
    }
    download_fw = 0;
    BT_fwICCM = vmalloc(BT_CCM_SIZE);
    BT_fwDCCM = vmalloc(BT_CCM_SIZE);
    memset(g_lib_cmd_buff, 0, sizeof(g_lib_cmd_buff));
    g_lib_cmd_fifo = gdsl_fifo_init(sizeof(g_lib_cmd_buff), g_lib_cmd_buff);
    g_lib_cmd_fifo->w = g_lib_cmd_buff;
    g_lib_cmd_fifo->r = g_lib_cmd_buff;
#if AML_BT_ROM_CHECK
    amlbt_usb_rom_check();
#endif
    printk("%s, end \n", __func__);
    return err;
}

static int amlbt_usb_remove(struct platform_device *dev)
{
    printk("%s\n", __func__);
    close_state = 2;
    if (g_fw_data_fifo == 0)
    {
        amlbt_usb_char_deinit();
    }
    vfree(BT_fwICCM);
    vfree(BT_fwDCCM);

    gdsl_fifo_deinit(g_lib_cmd_fifo);
    g_lib_cmd_fifo = 0;
    msleep(500);
    printk("%s end\n", __func__);
    return 0;
}

static int amlbt_usb_suspend(struct platform_device *dev, pm_message_t state)
{
    suspend_value = 1;
    if (FAMILY_TYPE_IS_W1U(amlbt_if_type))
    {
        unsigned int reg_value = g_auc_hif_ops.bt_hi_read_word(RG_AON_A15);
        printk("%s RG_AON_A15:%#x\n", __func__, reg_value);
        reg_value |= (1 << 31);
        g_auc_hif_ops.bt_hi_write_word(RG_AON_A15, reg_value);
        printk("RG_AON_A15:%#x", g_auc_hif_ops.bt_hi_read_word(RG_AON_A15));
    }
    printk("%s \n", __func__);
    return 0;
}

static int amlbt_usb_resume(struct platform_device *dev)
{
    printk("%s\n", __func__);
    msleep(1500);
    if (FAMILY_TYPE_IS_W1U(amlbt_if_type))
    {
        unsigned int reg_value = g_auc_hif_ops.bt_hi_read_word(RG_AON_A15);
        printk("%s RG_AON_A15:%#x\n", __func__, reg_value);
        reg_value &= ~(1 << 31);
        g_auc_hif_ops.bt_hi_write_word(RG_AON_A15, reg_value);
        printk("RG_AON_A15:%#x", g_auc_hif_ops.bt_hi_read_word(RG_AON_A15));
    }
    suspend_value = 0;
    printk("%s \n", __func__);
    return 0;

}

static void amlbt_usb_shutdown(struct platform_device *dev)
{
    printk("%s \n", __func__);
}


static struct platform_device amlbt_sdio_device =
{
    .name    = "sdio_bt",
    .id      = -1,
    .dev     = {
        .release = &amlbt_dev_release,
    }
};

static struct platform_driver amlbt_sdio_driver =
{
    .probe = amlbt_sdio_probe,
    .remove = amlbt_sdio_remove,
    .suspend = amlbt_sdio_suspend,
    .resume = amlbt_sdio_resume,
    .shutdown = amlbt_sdio_shutdown,

    .driver = {
        .name = "sdio_bt",
        .owner = THIS_MODULE,
    },
};

static struct platform_device amlbt_usb_device =
{
    .name    = "aml_btusb",
    .id      = -1,
    .dev     = {
        .release = &amlbt_dev_release,
    }
};

static struct platform_driver amlbt_usb_driver =
{
    .probe = amlbt_usb_probe,
    .remove = amlbt_usb_remove,
    .suspend = amlbt_usb_suspend,
    .resume = amlbt_usb_resume,
    .shutdown = amlbt_usb_shutdown,

    .driver = {
        .name = "aml_btusb",
        .owner = THIS_MODULE,
    },
};



static int amlbt_init(void)
{
    int ret = 0;
    struct platform_device *p_device = NULL;
    struct platform_driver *p_driver = NULL;

    printk("%s, type:%#x inter %s, family %s\n", __func__, amlbt_if_type,
        amlbt_family_intf((AMLBT_PD_ID_INTF & amlbt_if_type)),
        amlbt_family_intf(AMLBT_PD_ID_FAMILY & amlbt_if_type));

    if (!FAMILY_TYPE_IS_W1(amlbt_if_type) &&
        !FAMILY_TYPE_IS_W1U(amlbt_if_type))
    {
        printk("%s amlbt_if_type family type invalid!! \n", __func__);
        return ret;
    }
    if (INTF_TYPE_IS_SDIO(amlbt_if_type))  //sdio interface
    {
        p_device = &amlbt_sdio_device;
        p_driver = &amlbt_sdio_driver;
    }
    else if (INTF_TYPE_IS_USB(amlbt_if_type))        //usb interface
    {
        p_device = &amlbt_usb_device;
        p_driver = &amlbt_usb_driver;
    }
    else
    {
        printk("%s amlbt_if_type interface invalid!! \n", __func__);
        return ret;
    }

    printk("%s 1 \n", __func__);

    ret = platform_device_register(p_device);
    if (ret)
    {
        dev_err(&p_device->dev, "platform_device_register failed!\n");
        return ret;
    }
    printk("%s 2 \n", __func__);
    ret = platform_driver_register(p_driver);
    if (ret)
    {
        dev_err(&p_device->dev, "platform_driver_register failed!\n");
        return ret;
    }

    dev_info(&p_device->dev, "Init %#x OK!\n", amlbt_if_type);

    return ret;
}

static void amlbt_exit(void)
{
    struct platform_device *p_device = NULL;
    struct platform_driver *p_driver = NULL;

    printk("%s, type:%#x \n", __func__, amlbt_if_type);

    if (INTF_TYPE_IS_SDIO(amlbt_if_type))  //sdio interface
    {
        p_device = &amlbt_sdio_device;
        p_driver = &amlbt_sdio_driver;
    }
    else if (INTF_TYPE_IS_USB(amlbt_if_type))        //usb interface
    {
        p_device = &amlbt_usb_device;
        p_driver = &amlbt_usb_driver;
    }

    platform_driver_unregister(p_driver);
    platform_device_unregister(p_device);
}


module_param(amlbt_if_type, uint, S_IRUGO);
module_init(amlbt_init);
module_exit(amlbt_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("2022-09-06");

