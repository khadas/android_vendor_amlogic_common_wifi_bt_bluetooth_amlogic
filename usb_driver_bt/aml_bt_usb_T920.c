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
#include "bt_fucode_T920.h"
#include "aml_bt_usb_T920.h"

#define AML_BT_USB_VERSION  (0x05010118)
//#define AML_BT_PRODUCTION_TOOLS
#define CONFIG_BLUEDROID        1 /* bleuz 0, bluedroid 1 */
#define INDEPENDENT_USB			0

#define AML_USB_DEBUG			0
#define AML_RW_DEBUG 			0

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

//#define AML_BT_FW_DOWNLOAD_MAGIC	(0x20160623)

static unsigned int type[256] = {0};
static int total_evt_len = 0;
static int read_evt_len = 0;
static int read_index = 0;
static unsigned char p_acl_buf[HCI_MAX_FRAME_SIZE] = {0};
static unsigned char download_fw = 0;
static unsigned char download_flag = 0;
static unsigned char download_end = 0;
static unsigned int iccm_base_addr = 0;
static unsigned int dccm_base_addr = 0;
static unsigned int close_state = 0;

static unsigned char cmd[4][2] = {{0xf2, 0xfe}, {0xf1, 0xfe}, {0xf0, 0xfe}, {0xf3, 0xfe}};
static unsigned char cmd_cpt[0x405] = {0x04, 0x0e, 0x04, 0x01, 0x98, 0xfc, 0x00};
static volatile unsigned char cmd_index = 0xff;
static unsigned char dw_state = 0;
static unsigned char read_buff[0x405] = {0};
static unsigned char type_buff[0x405] = {0};

extern struct auc_hif_ops g_auc_hif_ops;
extern struct usb_device *g_udev;
extern int auc_send_cmd(unsigned int addr, unsigned int len);

#define ICCM_RAM_BASE           (0x000000)
#define DCCM_RAM_BASE           (0xd00000)

struct completion usb_completion;
static unsigned int fw_cmd_w = 0;
static unsigned int fw_cmd_r = 0;
static int outstanding_ptks = 0;

#if CONFIG_BLUEDROID

    static dev_t bt_devid; /* bt char device number */
    static struct cdev bt_char_dev; /* bt character device structure */
    static struct class *bt_char_class; /* device class for usb char driver */

#endif

static struct device *g_cdev = NULL;

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

static gdsl_fifo_t *gdsl_fifo_init(unsigned int len, unsigned char *base_addr)
{
    gdsl_fifo_t *p_fifo = (gdsl_fifo_t *)kzalloc(sizeof(gdsl_fifo_t), GFP_KERNEL);

    AMLBT_DBG("%s \n", __func__);

    if (p_fifo)
    {
        memset(p_fifo, 0, sizeof(gdsl_fifo_t));
        //p_fifo->w = (unsigned char *)((unsigned long)base_addr & 0xfffff);
        //p_fifo->r = (unsigned char *)((unsigned long)base_addr & 0xfffff);
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
        printk("drv_gdsl_send_data no space!!!\n");
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
        printk("gdsl_send_data no space!!!\n");
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

#if 0
unsigned int gdsl_fifo_update_w(gdsl_fifo_t *p_fifo, unsigned int len)
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
            p_fifo->w += len;
        }
        else
        {
            p_fifo->w = p_fifo->base_addr;
            p_fifo->w += (len - offset);
        }
    }
    else
    {
        printk("gdsl_fifo_update_w gdsl_send_data no space!!!\n");
    }
    return ret;
}

#endif

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
        printk("[%#x]\n", (unsigned int)(unsigned long)p_fifo);
        printk("[%#x, %#x]\n", (unsigned int)(unsigned long)p_fifo->w, (unsigned int)(unsigned long)p_fifo->r);
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

unsigned int bt_usb_get_tx_prio(gdsl_tx_q_t *p_fifo, unsigned int acl_handle)
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

void bt_usb_update_tx_q(gdsl_tx_q_t *p_fifo)
{
    unsigned int i = 0, j = 0;
    unsigned int acl_handle = 0;
    unsigned int tx_q_status[WF_SRAM_TX_Q_NUM] = {0};
    //unsigned int changed = 0;
    //unsigned int tx_q_info[WF_SRAM_TX_Q_NUM * 3] = {0};

    //AMLBT_DBG("up tx\n");

    AMLBT_DBG("up q\n");

    g_auc_hif_ops.bt_hi_read_sram((unsigned char *)tx_q_status, (unsigned char *)p_fifo[0].tx_q_status_addr,
                                  sizeof(tx_q_status));

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
            g_auc_hif_ops.bt_hi_write_word((unsigned int)(unsigned long)g_tx_q[i].tx_q_dev_index_addr, g_tx_q[i].tx_q_dev_index);
            g_auc_hif_ops.bt_hi_write_word((unsigned int)(unsigned long)g_tx_q[i].tx_q_prio_addr, g_tx_q[i].tx_q_prio);
            g_auc_hif_ops.bt_hi_write_word((unsigned int)(unsigned long)g_tx_q[i].tx_q_status_addr, g_tx_q[i].tx_q_status);
            for (j = 0; j < WF_SRAM_TX_Q_NUM; j++)
            {
                if (p_fifo[j].tx_q_dev_index == acl_handle)
                {
                    if (p_fifo[j].tx_q_status == GDSL_TX_Q_USED && p_fifo[j].tx_q_prio)
                    {
                        p_fifo[j].tx_q_prio--;
						g_auc_hif_ops.bt_hi_write_word((unsigned int)(unsigned long)g_tx_q[j].tx_q_prio_addr, g_tx_q[j].tx_q_prio);
                        AMLBT_DBG("dec:%#x,%#x,%#x\n", j, p_fifo[j].tx_q_prio, p_fifo[j].tx_q_status);
                    }
                }
            }
        }
    }
}


unsigned int bt_usb_get_tx_q(gdsl_tx_q_t *p_fifo, unsigned int acl_handle)
{
    unsigned int prio = 0xff;
    unsigned int i = 0;
    unsigned int find = 0;
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
                find = 1;
                index = i;
            }
        }
    }

    return index;
}

void bt_drv_rx_type_fifo_init(void)
{
    if (g_rx_type_fifo == 0)
    {
        g_rx_type_fifo = gdsl_fifo_init(RX_TYPE_FIFO_LEN, (unsigned char *)WF_SRAM_RX_TYPE_FIFO_ADDR);

        //update read pointer
        g_auc_hif_ops.bt_hi_write_word(WF_SRAM_RX_TYPE_FIFO_R_ADDR, (unsigned int)(unsigned long)g_rx_type_fifo->r);
        //update write pointer
        g_auc_hif_ops.bt_hi_write_word(WF_SRAM_RX_TYPE_FIFO_W_ADDR, (unsigned int)(unsigned long)g_rx_type_fifo->w);
        g_rx_type_fifo->r = (unsigned char *)(unsigned long)(WF_SRAM_RX_TYPE_FIFO_ADDR);
    }
}

void bt_drv_rx_type_fifo_deinit(void)
{
    if (g_rx_type_fifo)
    {
        g_auc_hif_ops.bt_hi_write_word(WF_SRAM_RX_TYPE_FIFO_R_ADDR, 0);
        g_auc_hif_ops.bt_hi_write_word(WF_SRAM_RX_TYPE_FIFO_W_ADDR, 0);
        gdsl_fifo_deinit(g_rx_type_fifo);
        g_rx_type_fifo = 0;
    }
}

void bt_drv_hci_cmd_fifo_init(void)
{
    if (g_cmd_fifo == 0)
    {
        g_cmd_fifo = gdsl_fifo_init(WF_SRAM_CMD_LEN, (unsigned char *)(WF_SRAM_CMD_Q_ADDR));

        //update read pointer
        g_auc_hif_ops.bt_hi_write_word(WF_SRAM_CMD_FIFO_R_ADDR, (unsigned int)(unsigned long)g_cmd_fifo->r);
        AMLBT_DBG("cmd fifo init r: %#lx\n", (unsigned long)g_cmd_fifo->r);
        //update write pointer
        g_auc_hif_ops.bt_hi_write_word(WF_SRAM_CMD_FIFO_W_ADDR, (unsigned int)(unsigned long)g_cmd_fifo->w);
        AMLBT_DBG("cmd fifo init w : %#lx\n", (unsigned long)g_cmd_fifo->w);
        g_cmd_fifo->w = (unsigned char *)(unsigned long)(WF_SRAM_CMD_Q_ADDR);
    }
    AMLBT_DBG("%s end \n", __func__);
}

void bt_drv_hci_cmd_fifo_deinit(void)
{
    AMLBT_DBG("%s \n", __func__);
    if (g_cmd_fifo != 0)
    {
        g_auc_hif_ops.bt_hi_write_word(WF_SRAM_CMD_FIFO_R_ADDR, 0);
        g_auc_hif_ops.bt_hi_write_word(WF_SRAM_CMD_FIFO_W_ADDR, 0);
        gdsl_fifo_deinit(g_cmd_fifo);
        g_cmd_fifo = 0;
    }
}

void bt_drv_hci_tx_data_init(void)
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
        g_tx_q[i].tx_q_addr = (unsigned char *)(unsigned long)(WF_SRAM_TX_Q_ADDR + i * 1024);
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

    g_auc_hif_ops.bt_hi_write_sram((unsigned char *)tx_info,
                                   (unsigned char *)WF_SRAM_TX_Q_STATUS_ADDR, sizeof(tx_info));

    AMLBT_DBG("%s end \n", __func__);
}

void bt_drv_hci_tx_data_deinit(void)
{
    unsigned tx_info[WF_SRAM_TX_Q_NUM * 3] = {0};

    AMLBT_DBG("%s \n", __func__);

    if (g_tx_q)
    {
        g_auc_hif_ops.bt_hi_write_sram((unsigned char *)tx_info,
                                       (unsigned char *)WF_SRAM_TX_Q_STATUS_ADDR, sizeof(tx_info));
        kfree(g_tx_q);
        g_tx_q = 0;
    }
    AMLBT_DBG("%s end \n", __func__);
}

void bt_drv_hci_evt_fifo_init(void)
{
    AMLBT_DBG("%s \n", __func__);

    if (g_event_fifo == 0)
    {
        g_event_fifo = gdsl_fifo_init(WF_SRAM_EVENT_LEN, (unsigned char *)(WF_SRAM_EVENT_Q_ADDR));
        g_auc_hif_ops.bt_hi_write_word(WF_SRAM_EVT_FIFO_R_ADDR, (unsigned int)(unsigned long)g_event_fifo->r);
        AMLBT_DBG("event fifo init r: %#lx\n", (unsigned long)g_event_fifo->r);
        g_auc_hif_ops.bt_hi_write_word(WF_SRAM_EVT_FIFO_W_ADDR, (unsigned int)(unsigned long)g_event_fifo->w);
        AMLBT_DBG("event fifo init w : %#lx\n", (unsigned long)g_event_fifo->w);
        g_event_fifo->r = (unsigned char *)(unsigned long)(WF_SRAM_EVENT_Q_ADDR);
    }
    AMLBT_DBG("%s end \n", __func__);
}

void bt_drv_hci_evt_fifo_deinit(void)
{
    AMLBT_DBG("%s \n", __func__);
    if (g_event_fifo != 0)
    {
        g_auc_hif_ops.bt_hi_write_word(WF_SRAM_EVT_FIFO_R_ADDR, 0);
        g_auc_hif_ops.bt_hi_write_word(WF_SRAM_EVT_FIFO_W_ADDR, 0);
        gdsl_fifo_deinit(g_event_fifo);
        g_event_fifo = 0;
    }
    AMLBT_DBG("%s end \n", __func__);
}

void bt_drv_fw_recv_fifo_init(void)
{
    AMLBT_DBG("%s \n", __func__);

    if (g_rx_fifo == 0)
    {
        g_rx_fifo = gdsl_fifo_init(WF_SRAM_RX_FIFO_LEN, (unsigned char *)(WF_SRAM_RX_Q_FIFO_ADDR));

        g_auc_hif_ops.bt_hi_write_word(WF_SRAM_RX_FIFO_R_ADDR, (unsigned int)(unsigned long)g_rx_fifo->r);
        AMLBT_DBG("recv fifo init r: %#lx\n", (unsigned long)g_rx_fifo->r);
        g_auc_hif_ops.bt_hi_write_word(WF_SRAM_RX_FIFO_W_ADDR, (unsigned int)(unsigned long)g_rx_fifo->w);
        AMLBT_DBG("recv fifo init w : %#lx\n", (unsigned long)g_rx_fifo->w);
        g_rx_fifo->r = (unsigned char *)(unsigned long)(WF_SRAM_RX_Q_FIFO_ADDR);
    }
    AMLBT_DBG("%s end \n", __func__);
}

void bt_drv_fw_recv_fifo_deinit(void)
{
    AMLBT_DBG("%s \n", __func__);
    if (g_rx_fifo != 0)
    {
        g_auc_hif_ops.bt_hi_write_word(WF_SRAM_RX_FIFO_R_ADDR, 0);
        g_auc_hif_ops.bt_hi_write_word(WF_SRAM_RX_FIFO_W_ADDR, 0);
        gdsl_fifo_deinit(g_rx_fifo);
        g_rx_fifo = 0;
    }
}

static void bt_usb_send_hci_cmd(unsigned char *data, unsigned int len)
{
    unsigned int i = 0;
    AMLBT_DBG("%s, len %d \n", __func__, len);

    if (g_cmd_fifo == NULL)
    {
        printk("%s: bt_usb_hci_cmd_fifo NULL!!!!\n", __func__);
        return ;
    }

    len = ((len + 3) & 0xFFFFFFFC);//Keep 4 bytes aligned

    AMLBT_DBG("%s, Actual length %d \n", __func__, len);
    // step 1: Update the command FIFO read pointer
    g_cmd_fifo->r = (unsigned char *)(unsigned long)g_auc_hif_ops.bt_hi_read_word(WF_SRAM_CMD_FIFO_R_ADDR);
    g_cmd_fifo->r += WF_SRAM_CMD_Q_ADDR;
    // step 2: Check the command FIFO space
    printk("C:%#x\n", len);
    for (i = 0; i < (len >= 10 ? 10 : len); i++)
    {
        printk("%#x|", data[i]);
    }
    printk("\n");
    //step 3: Write HCI commands to WiFi SRAM
    gdsl_fifo_write_data(g_cmd_fifo, data, len);
    //step 4: Update the write pointer and write to WiFi SRAM

    //	AMLBT_DBG("before write:r:%#lx, w:%#lx\n", r, w);

    g_auc_hif_ops.bt_hi_write_word(WF_SRAM_CMD_FIFO_W_ADDR, ((unsigned long)g_cmd_fifo->w - WF_SRAM_CMD_Q_ADDR) & 0x7ff);

    AMLBT_DBG("len %#x:w %#lx, r %#lx\n", len, (unsigned long)g_cmd_fifo->w, (unsigned long)g_cmd_fifo->r);
    AMLBT_DBG("w:r:%#lx, w:%#lx\n", (unsigned long)g_cmd_fifo->r, (unsigned long)g_cmd_fifo->w);

    //	AMLBT_DBG("after write: r:%#lx, w:%#lx\n", (unsigned long)g_cmd_fifo->r, (unsigned long)g_cmd_fifo->w);
}

static void bt_usb_send_hci_data(unsigned char *data, unsigned int len)
{
    unsigned int i = 0;
    unsigned int acl_handle = (((data[1] << 8) | data[0]) & 0xfff);
    unsigned int prio = 0;
    //unsigned int tx_q_info[3 * WF_SRAM_TX_Q_NUM] = {0};
    //unsigned int tx_q_status[WF_SRAM_TX_Q_NUM] = {0};
    //	AMLBT_DBG("s d r:%#x, w:%#x\n", g_tx_fifo->r, g_tx_fifo->w);
    AMLBT_DBG("%s, len:%d\n", __func__, len);

    bt_usb_update_tx_q(g_tx_q);

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

    prio = bt_usb_get_tx_prio(g_tx_q, acl_handle);

    g_tx_q[i].tx_q_prio = (++prio & 7);
    g_tx_q[i].tx_q_dev_index = acl_handle;
    g_tx_q[i].tx_q_status = GDSL_TX_Q_USED;

    AMLBT_DBG("D(%#x):%#x,%#x,%#x\n", i, (unsigned long)g_tx_q[i].tx_q_dev_index,
              (unsigned long)g_tx_q[i].tx_q_prio, len);

    g_auc_hif_ops.bt_hi_write_sram(data, g_tx_q[i].tx_q_addr, len);

#if 0
    g_auc_hif_ops.bt_hi_write_sram((unsigned char *)&g_tx_q[i].tx_q_dev_index,
                                   (unsigned char *)g_tx_q[i].tx_q_dev_index_addr, sizeof(g_tx_q[i].tx_q_dev_index));
    g_auc_hif_ops.bt_hi_write_sram((unsigned char *)&g_tx_q[i].tx_q_prio,
                                   (unsigned char *)g_tx_q[i].tx_q_prio_addr, sizeof(g_tx_q[i].tx_q_prio));
    g_auc_hif_ops.bt_hi_write_sram((unsigned char *)&g_tx_q[i].tx_q_status,
                                   (unsigned char *)g_tx_q[i].tx_q_status_addr, sizeof(g_tx_q[i].tx_q_status));
#else
    g_auc_hif_ops.bt_hi_write_word((unsigned int)(unsigned long)g_tx_q[i].tx_q_dev_index_addr, g_tx_q[i].tx_q_dev_index);
    g_auc_hif_ops.bt_hi_write_word((unsigned int)(unsigned long)g_tx_q[i].tx_q_prio_addr, g_tx_q[i].tx_q_prio);
    g_auc_hif_ops.bt_hi_write_word((unsigned int)(unsigned long)g_tx_q[i].tx_q_status_addr, g_tx_q[i].tx_q_status);

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

#if 0
static unsigned int bt_usb_recv_hci_data(unsigned char *buff, unsigned int index)
{
    //unsigned int len = 0;

    AMLBT_DBG("%s, index %d\n", __func__, index);
    AMLBT_DBG("d index %#x addr %#x\n", index, (WF_SRAM_RX_Q_ADDR + index * RX_Q_LEN));
    g_auc_hif_ops.bt_hi_read_sram(buff, (unsigned char *)(unsigned long)(WF_SRAM_RX_Q_ADDR + index * RX_Q_LEN), RX_Q_LEN);
    //AMLBT_DBG("d len %#x\n", len);
    AMLBT_DBG("%s len %d\n", __func__, len);

    return 0;
}
#endif

static unsigned int bt_usb_recv_hci_event(unsigned char *buff, unsigned int cnt)
{
    unsigned int len = 0;
    unsigned int i = 0;

    g_event_fifo->w = (unsigned char *)(unsigned long)g_auc_hif_ops.bt_hi_read_word(WF_SRAM_EVT_FIFO_W_ADDR);
    g_event_fifo->w += (WF_SRAM_EVENT_Q_ADDR);

    AMLBT_DBG("%s\n", __func__);

    AMLBT_DBG("r:%#lx,w:%#lx\n", (unsigned long)g_event_fifo->r, (unsigned long)g_event_fifo->w);

    len = gdsl_read_data(g_event_fifo, buff, cnt);

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

void aml_bt_usb_fifo_init(void)
{
    unsigned int st_reg = 0;

    AMLBT_DBG("%s\n", __func__);

    //g_auc_hif_ops.bt_hi_write_word((unsigned int)0x00a0d0e4, 0x8000007f);

    st_reg = g_auc_hif_ops.bt_hi_read_word(WF_SRAM_FW_DRIVER_STATUS_ADDR);
    st_reg |= WF_SRAM_FD_INIT_FLAG;
    g_auc_hif_ops.bt_hi_write_word(WF_SRAM_FW_DRIVER_STATUS_ADDR, st_reg);

    bt_drv_rx_type_fifo_init();
    bt_drv_hci_cmd_fifo_init();
    bt_drv_hci_tx_data_init();
    bt_drv_hci_evt_fifo_init();
    bt_drv_fw_recv_fifo_init();
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
    g_auc_hif_ops.bt_hi_write_word(WF_SRAM_FW_DRIVER_STATUS_ADDR, st_reg);
}

void aml_bt_usb_fifo_deinit(void)
{
    unsigned int st_reg = 0;

    st_reg = g_auc_hif_ops.bt_hi_read_word(WF_SRAM_FW_DRIVER_STATUS_ADDR);
    st_reg |= WF_SRAM_FD_INIT_FLAG;
    g_auc_hif_ops.bt_hi_write_word(WF_SRAM_FW_DRIVER_STATUS_ADDR, st_reg);

    bt_drv_rx_type_fifo_deinit();
    bt_drv_hci_cmd_fifo_deinit();
    bt_drv_hci_tx_data_deinit();
    bt_drv_hci_evt_fifo_deinit();
    bt_drv_fw_recv_fifo_deinit();
    gdsl_fifo_deinit(g_fw_data_fifo);
    gdsl_fifo_deinit(g_fw_evt_fifo);
    gdsl_fifo_deinit(g_fw_type_fifo);
    g_fw_data_fifo = 0;
    g_fw_evt_fifo = 0;
    g_fw_type_fifo = 0;
    st_reg &= ~(WF_SRAM_FD_INIT_FLAG);
    g_auc_hif_ops.bt_hi_write_word(WF_SRAM_FW_DRIVER_STATUS_ADDR, st_reg);
}

static void aml_usb_init(void)
{
    AMLBT_DBG("%s\n", __func__);
    aml_bt_usb_fifo_init();
    AMLBT_DBG("%s end\n", __func__);
}

static void aml_usb_deinit(void)
{
    AMLBT_DBG("%s\n", __func__);
    aml_bt_usb_fifo_deinit();
    AMLBT_DBG("%s end\n", __func__);
}

static void bt_drv_reset(void)
{
    memset(p_acl_buf, 0, sizeof(p_acl_buf));
    memset(type, 0, sizeof(type));
    total_evt_len = 0;
    read_evt_len = 0;
    read_index = 0;
}

void bt_fw_check(void)
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
        st_reg = g_auc_hif_ops.bt_hi_read_word(iccm_base_addr);
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
        st_reg = g_auc_hif_ops.bt_hi_read_word(dccm_base_addr);
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

void bt_write_firmware(unsigned char *buf, unsigned int len, unsigned int addr)
{
    unsigned int st_reg = 0;

    g_auc_hif_ops.bt_hi_write_sram(buf, (unsigned char *)(unsigned long)(WF_SRAM_RFU_ADDR), len);
    st_reg = g_auc_hif_ops.bt_hi_read_word(WF_SRAM_FW_DRIVER_STATUS_ADDR);
    st_reg |= WF_SRAM_FD_DOWNLOAD_W;
    g_auc_hif_ops.bt_hi_write_word(WF_SRAM_FW_DRIVER_STATUS_ADDR, st_reg);
    auc_send_cmd(addr, len);
    while (WF_SRAM_FD_DOWNLOAD_W & g_auc_hif_ops.bt_hi_read_word(WF_SRAM_FW_DRIVER_STATUS_ADDR))
    {

    }
}

void bt_drv_download_firmware(void)
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
    //g_auc_hif_ops.bt_hi_write_word(0xf03050, 1);    //ram power down
    //g_auc_hif_ops.bt_hi_write_word(0xf03058, 0);    //pmu down
    g_auc_hif_ops.bt_hi_write_word(0xf03058, 0x007);    //pmu up
    //g_auc_hif_ops.bt_hi_write_word(0xf03050, 0);    //ram power up
    remain_len = (download_size - offset);

    while (offset < download_size)
    {
        if (remain_len < WF_SRAM_FW_DOWNLOAD_SIZE)
        {
            bt_write_firmware((unsigned char *)&fw_iccmBuf[offset], remain_len, iccm_base_addr);
            offset += remain_len;
            iccm_base_addr += remain_len;
            AMLBT_DBG("bt_usb_download_firmware iccm1 offset %#x, write_len %#x\n", offset, write_len);
        }
        else
        {
            bt_write_firmware((unsigned char *)&fw_iccmBuf[offset], WF_SRAM_FW_DOWNLOAD_SIZE, iccm_base_addr);
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
            bt_write_firmware((unsigned char *)&fw_dccmBuf[offset], remain_len, dccm_base_addr);
            offset += remain_len;
            dccm_base_addr += remain_len;
            AMLBT_DBG("bt_usb_download_firmware dccm1 offset %#x, write_len %#x\n", offset, write_len);
        }
        else
        {
            bt_write_firmware((unsigned char *)&fw_dccmBuf[offset], WF_SRAM_FW_DOWNLOAD_SIZE, dccm_base_addr);
            offset += WF_SRAM_FW_DOWNLOAD_SIZE;
            remain_len -= WF_SRAM_FW_DOWNLOAD_SIZE;
            dccm_base_addr += WF_SRAM_FW_DOWNLOAD_SIZE;
            AMLBT_DBG("bt_usb_download_firmware dccm2 offset %#x, write_len %#x\n", offset, write_len);
        }

        AMLBT_DBG("bt_usb_download_firmware dccm remain_len %#x \n", remain_len);
    }
    //bt_fw_check();
    g_auc_hif_ops.bt_hi_write_word(0xf03058, 0x700);
    printk("read bt reg : %#x \n", g_auc_hif_ops.bt_hi_read_word(0x230128));
    printk("%s end 15:51,bt reg : %#x\n", __func__, g_auc_hif_ops.bt_hi_read_word(0xf03058));
}


static int aml_usb_char_open(struct inode *inode_p, struct file *file_p)
{
    printk("%s, %#x\n", __func__, AML_BT_USB_VERSION);
    if (download_fw)
    {
#ifndef AML_BT_PRODUCTION_TOOLS
        download_fw = 0;
#else
        bt_drv_reset();
        aml_usb_init();
        download_fw = 1;
        download_end = 0;
        download_flag = 1;
        fw_cmd_w = 0;
        fw_cmd_r = 0;
#endif
    }
    close_state = 0;
    init_completion(&usb_completion);
    return nonseekable_open(inode_p, file_p);
}

static int aml_usb_char_close(struct inode *inode_p, struct file *file_p)
{
    printk("%s $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ %#x \n", __func__, download_fw);

    if (g_event_fifo != 0)
    {
        printk("event w:%p,r:%p\n", g_event_fifo->w, g_event_fifo->r);
    }

    if (download_fw)
    {
        aml_usb_deinit();
    }

    return 0;
}

static ssize_t aml_usb_char_read_fw(struct file *file_p,
                                    char __user *buf_p,
                                    size_t count,
                                    loff_t *pos_p)
{
//    unsigned int size = sizeof(cmd) / sizeof(cmd[0]);
    unsigned char cmd_opcode[2] = {0};
    //unsigned int n = 0;

    //printk("R_FW:%#x,%#x,%#x\n", (unsigned int)count,
    //    (unsigned int)(unsigned long)g_lib_cmd_fifo->w,
    //    (unsigned int)(unsigned long)g_lib_cmd_fifo->r);
    //printk("R_FW:%#x \n", (unsigned int)count);
    while (g_lib_cmd_fifo->w == g_lib_cmd_fifo->r)
    {
        wait_for_completion(&usb_completion);
    }

    gdsl_fifo_get_data(g_lib_cmd_fifo, cmd_opcode, 2);
    cmd_cpt[4] = cmd_opcode[0];
    cmd_cpt[5] = cmd_opcode[1];

    //printk("RE:\n");
    //for (n = 0; n < 7; n++)
    {
        //printk("%#x|", cmd_cpt[n]);
    }
    //printk("\n");

    if (copy_to_user(buf_p, &cmd_cpt, 7))
    {
        return -EFAULT;
    }

/*
    if (download_end)
    {
        bt_drv_reset();
        aml_usb_init();
        download_fw = 1;
        download_end = 0;
        download_flag = 1;
        printk("read bt reg : %#x \n", g_auc_hif_ops.bt_hi_read_word(0x230128));
        printk("download_end 15:53,bt reg : %#x\n", g_auc_hif_ops.bt_hi_read_word(0xf03058));
    }
    */
    //printk("R_FW:END \n");
    //printk("11\n");
    fw_cmd_r++;
    return 7;
}

static ssize_t aml_usb_read_fw_data(void)
{
    unsigned int i = 0;
    unsigned int read_len = 0;
    unsigned int data_index = 0;
    unsigned long tmp = 0;
    unsigned int type_size = 0;
    int cnt = 0;
    g_rx_type_fifo->w = (unsigned char *)(unsigned long)g_auc_hif_ops.bt_hi_read_word(WF_SRAM_RX_TYPE_FIFO_W_ADDR);
    g_rx_type_fifo->w += (WF_SRAM_RX_TYPE_FIFO_ADDR);
    memset(read_buff, 0, sizeof(read_buff));
    memset(type_buff, 0, sizeof(type_buff));
    read_index = 0;
    while (g_rx_type_fifo->w == g_rx_type_fifo->r)
    {
        if (gdsl_fifo_used_size(g_fw_type_fifo))
        {
            AMLBT_DBG("[%#x]\n", (unsigned int)(unsigned long)g_fw_type_fifo);
            AMLBT_DBG("[%#x,%#x]\n", (unsigned int)(unsigned long)g_fw_type_fifo->w, (unsigned int)(unsigned long)g_fw_type_fifo->r);
            return 0;
        }

        if (close_state)
        {
            printk("R CLOSE\n");
            close_state = 0;
            return -EFAULT;
        }
        //wait_for_completion(&usb_completion);
        g_rx_type_fifo->w = (unsigned char *)(unsigned long)g_auc_hif_ops.bt_hi_read_word(WF_SRAM_RX_TYPE_FIFO_W_ADDR);
        g_rx_type_fifo->w += (WF_SRAM_RX_TYPE_FIFO_ADDR);
        //AMLBT_DBG("(%#x,%#x)\n", g_rx_type_fifo->w, g_rx_type_fifo->r);

        //If the sleep time of the read thread is short, thread scheduling will be increased,
        //CPU resources will be occupied, and the data sending thread will be affected.
        //Therefore, when sending data(outstanding_ptks>0), increase the sleep time.
        if (outstanding_ptks == 0)
            usleep_range(1000, 1000);
        else if (cnt % 250 == 0)
        {
            usleep_range(5000, 5000);
            cnt = 0;
        }
        cnt++;
    }

    type_size = gdsl_read_data(g_rx_type_fifo, type_buff, sizeof(type_buff));
    AMLBT_DBG("R SIZE:%d\n", type_size);

    if (type_size == 0)
    {
        printk("read type fifo err!!\n");
        return -EFAULT;
    }
    gdsl_fifo_copy_data(g_fw_type_fifo, type_buff, type_size);
    AMLBT_DBG("TYPE:[%#x,%#x,%#x,%#x]\n", type_buff[0],
              type_buff[4], type_buff[8], type_buff[12]);
    g_auc_hif_ops.bt_hi_write_word(WF_SRAM_RX_TYPE_FIFO_R_ADDR,
                                   ((unsigned int)(unsigned long)g_rx_type_fifo->r - WF_SRAM_RX_TYPE_FIFO_ADDR) & 0x1fff);

    for (i = 0; i < (type_size / 4); i++)
    {
        memset(read_buff, 0, sizeof(read_buff));
        if (type_buff[i * 4] == HCI_EVENT_PKT)
        {
            bt_usb_recv_hci_event(read_buff, 4);
            AMLBT_DBG("HEAD:[%#x,%#x,%#x,%#x]\n", read_buff[0],
                      read_buff[1], read_buff[2], read_buff[3]);
            if (read_buff[1] == 0x13)
                outstanding_ptks--;
            read_len = read_buff[2];
            read_len -= 1;
            read_len = ((read_len + 3) & 0xFFFFFFFC);
            bt_usb_recv_hci_event(&read_buff[4], read_len);
            gdsl_fifo_copy_data(g_fw_evt_fifo, read_buff, read_buff[2] + 3);
            //printk("read 1 r:%#x, w:%#x\n", (unsigned int)g_fw_evt_fifo->r, (unsigned int)g_fw_evt_fifo->w);
            //printk("{1 %#x|%#x|%#x|%#x}\n", g_fw_evt_fifo->r[0],g_fw_evt_fifo->r[1],g_fw_evt_fifo->r[2],g_fw_evt_fifo->r[3]);
            tmp = (unsigned long)g_event_fifo->r;
            tmp = ((tmp + 3) & 0xFFFFFFFC);
            g_event_fifo->r = (unsigned char *)tmp;
            g_auc_hif_ops.bt_hi_write_word(WF_SRAM_EVT_FIFO_R_ADDR,
                                           ((unsigned int)(unsigned long)g_event_fifo->r - WF_SRAM_EVENT_Q_ADDR) & 0x7ff);
        }
        else if (type_buff[i * 4] == HCI_ACLDATA_PKT)
        {
            g_rx_fifo->w = (unsigned char *)(unsigned long)g_auc_hif_ops.bt_hi_read_word(WF_SRAM_RX_FIFO_W_ADDR);
            g_rx_fifo->w += (WF_SRAM_RX_Q_FIFO_ADDR);
            while (g_rx_fifo->r == g_rx_fifo->w)
            {
                if (close_state)
                {
                    printk("R CLOSE 2\n");
                    close_state = 0;
                    return -EFAULT;
                }
                g_rx_fifo->w = (unsigned char *)(unsigned long)g_auc_hif_ops.bt_hi_read_word(WF_SRAM_RX_FIFO_W_ADDR);
                g_rx_fifo->w += (WF_SRAM_RX_Q_FIFO_ADDR);
                AMLBT_DBG("rf2 r %#x, w %#x\n", (unsigned long)g_rx_fifo->r, (unsigned long)g_rx_fifo->w);
            }
            gdsl_read_data(g_rx_fifo, (unsigned char *)&data_index, 4);
            g_auc_hif_ops.bt_hi_write_word(WF_SRAM_RX_FIFO_R_ADDR,
                                           ((unsigned int)(unsigned long)g_rx_fifo->r - WF_SRAM_RX_Q_FIFO_ADDR) & 0x1f);
            g_auc_hif_ops.bt_hi_read_sram(&read_buff[0],
                                          (unsigned char *)(unsigned long)(WF_SRAM_RX_Q_ADDR + data_index * RX_Q_LEN), 8);

            read_len = ((read_buff[7] << 8) | (read_buff[6]));
            read_len = ((read_len + 3) & 0xFFFFFFFC);

            g_auc_hif_ops.bt_hi_read_sram(&read_buff[8],
                                          (unsigned char *)(unsigned long)(WF_SRAM_RX_Q_ADDR + data_index * RX_Q_LEN + 8), read_len);

            gdsl_fifo_copy_data(g_fw_data_fifo, read_buff, (((read_buff[7] << 8) | (read_buff[6])) + 8));
            AMLBT_DBG("HEAD1:[%#x,%#x,%#x,%#x]\n", read_buff[0],
                      read_buff[1], read_buff[2], read_buff[3]);
            AMLBT_DBG("HEAD2:[%#x,%#x,%#x,%#x]\n", read_buff[4],
                      read_buff[5], read_buff[6], read_buff[7]);
            AMLBT_DBG("HEAD3:[%#x,%#x,%#x,%#x]\n", read_buff[8],
                      read_buff[9], read_buff[10], read_buff[11]);
            AMLBT_DBG("HEAD4:[%#x,%#x,%#x,%#x]\n", read_buff[12],
                      read_buff[13], read_buff[14], read_buff[15]);
        }
        else
        {
            printk("type error!\n");
        }
    }
    return 0;
}

static ssize_t aml_usb_char_read(struct file *file_p,
                                 char __user *buf_p,
                                 size_t count,
                                 loff_t *pos_p)
{
    ssize_t ret = 0;
    unsigned int bt_type = 0;
    unsigned int read_len = 0;
    unsigned int len = 0;
    unsigned char data_head[8] = {0};
    //AMLBT_DBG("R:%#x, r %#x, w %#x\n", count, (unsigned long)g_host_tx_fifo->r, (unsigned long)g_host_tx_fifo->w);

    AMLBT_DBG("%s start, count : %ld\n", __func__, count);
    AMLBT_DBG("fw recv fifo r:%#lx, w:%#lx\n", (unsigned long)g_rx_fifo->r, (unsigned long)g_rx_fifo->w);
    AMLBT_DBG("%s, data : %d\n", __func__, data);

    if (!download_fw || (fw_cmd_r < fw_cmd_w))
    {
        //printk("1:%#x\n",fw_cmd_r);
        return aml_usb_char_read_fw(file_p, buf_p, count, pos_p);
    }
    //printk("R:%#x\n", (unsigned int)count);
    ret = aml_usb_read_fw_data();
    if (ret < 0)
    {
        return ret;
    }

    //printk("recv(%#x,%#x) w:%#x,r:%#x\n", type, evt_state,
    //	(unsigned long)g_rx_fifo->w, (unsigned long)g_rx_fifo->r);

    //printk("{%#x, %#x}\n", (unsigned int)(unsigned long)g_fw_type_fifo->w,
    //    (unsigned int)(unsigned long)g_fw_type_fifo->r);

    gdsl_fifo_get_data(g_fw_type_fifo, (unsigned char *)&bt_type, sizeof(bt_type));

    if (bt_type == HCI_EVENT_PKT)
    {
        gdsl_fifo_get_data(g_fw_evt_fifo, read_buff, 4);    //read head;
        len = read_buff[2];
        len -= 1;
        //len = ((len + 3) & 0xFFFFFFFC);
        read_len = gdsl_fifo_get_data(g_fw_evt_fifo, &read_buff[4], len);    //read payload;
        read_len += 4;
        if (copy_to_user(buf_p, read_buff, read_len))
        {
            printk("%s, copy_to_user error \n", __func__);
            return -EFAULT;
        }
        AMLBT_DBG("EV:");
        for (ret = 0; ret < 12; ret++)
        {
            AMLBT_DBG("%#x|", read_buff[ret]);
        }
        AMLBT_DBG("\n");
    }
    else if (bt_type == HCI_ACLDATA_PKT)
    {
        gdsl_fifo_get_data(g_fw_data_fifo, data_head, 8);   //read header
        read_buff[0] = data_head[0];
        read_buff[1] = data_head[4];
        read_buff[2] = data_head[5];
        read_buff[3] = data_head[6];
        read_buff[4] = data_head[7];
        len = ((data_head[7] << 8) | (data_head[6]));
        //len = ((read_len + 3) & 0xFFFFFFFC);
        read_len = gdsl_fifo_get_data(g_fw_data_fifo, &read_buff[5], len); //read payload
        read_len += 5;
        if (copy_to_user(buf_p, read_buff, read_len))
        {
            printk("%s, copy_to_user error \n", __func__);
            return -EFAULT;
        }
        AMLBT_DBG("DA:");
        for (ret = 0; ret < 12; ret++)
        {
            AMLBT_DBG("%#x|", read_buff[ret]);
        }
        AMLBT_DBG("\n");
    }

    //printk("R END %#x\n", read_len);

    return read_len;
}

static ssize_t aml_usb_char_write_fw(struct file *file_p,
                                     const char __user *buf_p,
                                     size_t count,
                                     loff_t *pos_p)
{
    unsigned int n = 0;
    unsigned int size = sizeof(cmd) / sizeof(cmd[0]);
    unsigned char len  = 0;
    unsigned int offset = 0;
    unsigned int reg_value = 0;

    //printk("W_FW:%#x\n", (unsigned int)count);

    //if (count == 1)
    //{
        //return count;
    //}

    memset(p_acl_buf, 0, HCI_MAX_FRAME_SIZE);

    if (copy_from_user(p_acl_buf, buf_p, count))
    {
        AMLBT_DBG("%s: Failed to get data from user space\n", __func__);
        return -EFAULT;
    }

    for (n = 0; n < size; n++)
    {
        if (!memcmp(&p_acl_buf[1], cmd[n], 2))
        {
            cmd_index = n;
            break;
        }
    }
    //AMLRW_DBG("CMD_I:%#x\n", cmd_index);

    if (n == size)
    {
        for (n = 0; n < 7; n++)
        {
            printk("%#x|", p_acl_buf[n]);
        }
        printk("\n");
        printk("---------------cmd error!------------");
        return -EINVAL;
    }

    //for (n = 0; n < 12; n++)
    {
        //printk("%#x|", p_acl_buf[n]);
    }
    //printk("\n");

    gdsl_fifo_copy_data(g_lib_cmd_fifo, &p_acl_buf[1], 2);

    if (p_acl_buf[1] == 0xf3 && p_acl_buf[2] == 0xfe)   //download fw
    {
        len = count - 8;
        offset = ((p_acl_buf[7] << 24) | (p_acl_buf[6] << 16) | (p_acl_buf[5] << 8) | p_acl_buf[4]);
        //printk("%#x,%#x,%#x\n", len, offset, dw_state);
        AMLRW_DBG("%#x,%#x,%#x,%#x\n", p_acl_buf[8], p_acl_buf[9], p_acl_buf[10], p_acl_buf[11]);
        if (offset == BT_ICCM_ROM_LEN)
        {
            //printk("W S\n");
            if (!download_flag)
            {
                g_auc_hif_ops.bt_hi_write_word(0xf03050, 0);
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
            //bt_write_firmware(&p_acl_buf[7], len, iccm_base_addr);
            //printk("W S 1\n");
            memcpy(&BT_fwICCM[iccm_base_addr], &p_acl_buf[8], len);
            //printk("W S 2\n");
            iccm_base_addr += len;
        }
        else
        {
            //bt_write_firmware(&p_acl_buf[7], len, dccm_base_addr);
            memcpy(&BT_fwDCCM[dccm_base_addr], &p_acl_buf[8], len);
            dccm_base_addr += len;
        }
    }
    else if (p_acl_buf[1] == 0xf1 && p_acl_buf[2] == 0xfe)
    {
        offset = ((p_acl_buf[7] << 24) | (p_acl_buf[6] << 16) | (p_acl_buf[5] << 8) | p_acl_buf[4]);
        reg_value = ((p_acl_buf[11] << 24) | (p_acl_buf[10] << 16) | (p_acl_buf[9] << 8) | p_acl_buf[8]);
        printk("WR:%#x,%#x\n", offset, reg_value);
        if (offset == 0xa70014 && !(reg_value & (1 << 24))) //rf calibration
        {
            //bt_fw_check();
            if (!download_flag)
            {
                bt_drv_download_firmware();
                //g_auc_hif_ops.bt_hi_write_word(0xf03058, 0x700);
                printk("0x2fe008:%#x\n", g_auc_hif_ops.bt_hi_read_word(0x2fe008));
            }
            printk("W E %#x,%#x\n", iccm_base_addr, dccm_base_addr);
        }
        else if (offset == 0xf03040)
        {
            bt_drv_reset();
            aml_usb_init();
            download_fw = 1;
            download_end = 0;
            download_flag = 1;
            printk("end read bt reg : %#x \n", g_auc_hif_ops.bt_hi_read_word(0x230128));
            printk("end download_end 15:53,bt reg : %#x\n", g_auc_hif_ops.bt_hi_read_word(0xf03058));
        }
    }
    //printk("W_FW END\n");
    //printk("22\n");
    complete(&usb_completion);
    return count;
}

static ssize_t aml_usb_char_write(struct file *file_p,
                                  const char __user *buf_p,
                                  size_t count,
                                  loff_t *pos_p)
{
    unsigned int i = 0;
    unsigned int w_type = 0;

    AMLBT_DBG("%s, count:%ld\n", __func__, count);

    if (count > HCI_MAX_FRAME_SIZE)
    {
        return -EINVAL;
    }

    if (!download_fw)
    {
        //printk("2:%#x\n",fw_cmd_w);
        fw_cmd_w++;
        return aml_usb_char_write_fw(file_p, buf_p, count, pos_p);
    }

    //printk("W:%#x\n", (unsigned int)count);

    memset(p_acl_buf, 0, HCI_MAX_FRAME_SIZE);

    if (copy_from_user(p_acl_buf, buf_p, count))
    {
        AMLBT_DBG("%s: Failed to get data from user space\n", __func__);
        return -EFAULT;
    }
    w_type = p_acl_buf[0];
    AMLBT_DBG("%s:get hci raw data:\n", __func__);

    if (w_type == HCI_COMMAND_PKT)
    {
        printk("W:%#x\n", (unsigned int)count);
        for (i = 0; i < (count >= 10 ? 10 : count); i++)
        {
            printk("%#x|", p_acl_buf[i]);
        }
        printk("\n");
    }

    AMLBT_DBG("\n");

    if (w_type == HCI_COMMAND_PKT)
    {
        if (count == 0x10 && p_acl_buf[1] == 0x66 && p_acl_buf[2] == 0xfc)   //close
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
        bt_usb_send_hci_cmd(&p_acl_buf[1], count - 1);
    }
    else if (w_type == HCI_ACLDATA_PKT)
    {
        bt_usb_send_hci_data(&p_acl_buf[1], count - 1);
        outstanding_ptks++;
    }
    //complete(&usb_completion);
    //printk("W END\n");

    return count;
}


static struct file_operations bt_chrdev_ops  =
{
    .open = aml_usb_char_open,
    .release = aml_usb_char_close,
    .read = aml_usb_char_read,
    .write = aml_usb_char_write,
    //.poll = btchr_poll,
    //.unlocked_ioctl = btchr_ioctl,
};


static int aml_usb_char_init(void)
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

    cdev_init(&bt_char_dev, &bt_chrdev_ops);
    res = cdev_add(&bt_char_dev, bt_devid, 1);
    if (res < 0)
    {
        AMLBT_DBG("%s:Failed to add bt char device\n", __func__);
        goto err_add;
    }
    g_cdev = dev;
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

static void aml_usb_char_deinit(void)
{
    printk("%s \n", __func__);
    device_destroy(bt_char_class, bt_devid);
    class_destroy(bt_char_class);
    cdev_del(&bt_char_dev);
    unregister_chrdev_region(bt_devid, 1);
}

static int __init btusb_init(void)
{
    int err = 0;

    AMLBT_DBG("%s", __func__);
    //g_auc_hif_ops.bt_hi_write_word((unsigned int)0x00a0d0e4, 0x8000007f);
    printk("%s, %#x", __func__, g_auc_hif_ops.bt_hi_read_word(0x00a0d0e4));
#if CONFIG_BLUEDROID
    err = aml_usb_char_init();
    if (err < 0)
    {
        /* usb register will go on, even bt char register failed */
        AMLBT_DBG("%s:Failed to register usb char device interfaces\n", __func__);
    }
#endif
    download_fw = 0;
    memset(g_lib_cmd_buff, 0, sizeof(g_lib_cmd_buff));
    g_lib_cmd_fifo = gdsl_fifo_init(sizeof(g_lib_cmd_buff), g_lib_cmd_buff);
    g_lib_cmd_fifo->w = g_lib_cmd_buff;
    g_lib_cmd_fifo->r = g_lib_cmd_buff;

    BT_fwICCM = vmalloc(BT_CCM_SIZE);
    BT_fwDCCM = vmalloc(BT_CCM_SIZE);
    return err;
}

static void __exit btusb_exit(void)
{
    printk("%s\n", __func__);
#if CONFIG_BLUEDROID
    aml_usb_char_deinit();
#endif
    gdsl_fifo_deinit(g_lib_cmd_fifo);
    g_lib_cmd_fifo = 0;
    vfree(BT_fwICCM);
    vfree(BT_fwDCCM);
    printk("%s end\n", __func__);
}

module_init(btusb_init);
module_exit(btusb_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("2022-01-18");

