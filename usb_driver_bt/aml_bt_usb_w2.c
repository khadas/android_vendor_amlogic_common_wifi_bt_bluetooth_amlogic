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

#include "bt_fucode_w2.h"
#include "aml_bt_usb_w2.h"

#define BT_USB_MAX_PRIO                        0xFFFFFFFF

//static int reg_config_complete = 0;
static int amlbt_sdio_major;
static struct cdev amlbt_sdio_cdev;
static int amlbt_sdio_devs = 1;
static struct class *amlbt_sdio_class;
static struct device *amlbt_sdio_dev;
static unsigned int rf_num = -1;

#define AML_BT_VERSION  (0x202300912)
#define AML_BT_PRODUCTION_TOOLS
#define CONFIG_BLUEDROID        1 /* bleuz 0, bluedroid 1 */
#define INDEPENDENT_USB			0

#define AML_BT_ROM_CHECK        0
#define REG_DEV_RESET           0xf03058
#define REG_PMU_POWER_CFG       0xf03040
#define	REG_RAM_PD_SHUTDWONW_SW 0xf03050

#define BIT_PHY                 1
#define BIT_MAC                 (1 << 1)
#define BIT_CPU                 (1 << 2)
#define DEV_RESET_SW            16
#define DEV_RESET_HW            0
#define BIT_RF_NUM              28
#define AML_ADDR_AON            1

#define FAMILY_TYPE_IS_W1(x)        ((AMLBT_PD_ID_FAMILY & x) == AMLBT_FAMILY_W1)
#define FAMILY_TYPE_IS_W1U(x)       ((AMLBT_PD_ID_FAMILY & x) == AMLBT_FAMILY_W1U)
#define FAMILY_TYPE_IS_W2(x)        ((AMLBT_PD_ID_FAMILY & x) == AMLBT_FAMILY_W2)
#define INTF_TYPE_IS_SDIO(x)        ((AMLBT_PD_ID_INTF & x) == AMLBT_INTF_SDIO)
#define INTF_TYPE_IS_USB(x)         ((AMLBT_PD_ID_INTF & x) == AMLBT_INTF_USB)
#define INTF_TYPE_IS_PCIE(x)        ((AMLBT_PD_ID_INTF & x) == AMLBT_INTF_PCIE)

#define TYPE_RETURN_STR(type) \
    case type:                \
    return #type;


const char *amlbt_family_intf(int type)
{
    switch (type)
    {
            TYPE_RETURN_STR(AMLBT_FAMILY_W1)
            TYPE_RETURN_STR(AMLBT_FAMILY_W1U)
            TYPE_RETURN_STR(AMLBT_FAMILY_W2)
            TYPE_RETURN_STR(AMLBT_INTF_SDIO)
            TYPE_RETURN_STR(AMLBT_INTF_USB)
            TYPE_RETURN_STR(AMLBT_INTF_PCIE)
        default:
            break;
    }

    return "unknown type";
}

static unsigned int amlbt_if_type = AMLBT_TRANS_UNKNOWN;
static unsigned char *usb_buf = NULL;
static unsigned char *sdio_buf = NULL;
static unsigned char *bt_usb_data_buf = NULL;
static int evt_state = 0;
static unsigned char *type = NULL;
static unsigned char *p_acl_buf = NULL;
static unsigned char download_fw = 0;
static unsigned char download_flag = 0;
static unsigned char download_end = 0;
static unsigned int iccm_base_addr = 0;
static unsigned int dccm_base_addr = 0;
static unsigned int close_state = 0;
static struct task_struct *check_fw_rx_stask = NULL;

static unsigned char cmd[4][2] = {{0xf2, 0xfe}, {0xf1, 0xfe}, {0xf0, 0xfe}, {0xf3, 0xfe}};
static unsigned char cmd_cpt[64] = {0x04, 0x0e, 0x04, 0x01, 0x98, 0xfc, 0x00};
static volatile unsigned char cmd_index = 0xff;
static unsigned char dw_state = 0;
//static unsigned char *type_buff = NULL;

extern struct auc_hif_ops g_auc_hif_ops;
extern struct aml_hif_sdio_ops g_hif_sdio_ops;

extern struct usb_device *g_udev;
extern int auc_send_cmd_ep1(unsigned int addr, unsigned int len);
extern uint32_t aml_pci_read_for_bt(int base, u32 offset);
extern void aml_pci_write_for_bt(u32 val, int base, u32 offset);
int amlbt_usb_check_fw_rx(void *data);

struct completion usb_completion;
struct completion data_completion;

static unsigned int fw_cmd_w = 0;
static unsigned int fw_cmd_r = 0;

#define CMD_DOWNLOAD _IO('b', 0)
#define CMD_USB_CRASH _IO('c', 0)

#define ICCM_RAM_BASE           (0x000000)
#define DCCM_RAM_BASE           (0xd00000)

#define CMD_FIFO_SIZE           128
#define DATA_FIFO_SIZE          16*1024
#define DATA_INDEX_SIZE          128
#define EVT_FIFO_SIZE           64*1024
#define TYPE_FIFO_SIZE          1024

//static struct task_struct *check_fw_rx_stask = NULL;

static dev_t bt_devid; /* bt char device number */
static struct cdev bt_char_dev; /* bt character device structure */
static struct class *bt_char_class; /* device class for usb char driver */

static gdsl_fifo_t *g_cmd_fifo = 0;
static gdsl_fifo_t *g_event_fifo = 0;
static gdsl_fifo_t *g_rx_fifo = 0;
static gdsl_tx_q_t *g_tx_q = 0;
static gdsl_fifo_t *g_rx_type_fifo = 0;
static gdsl_fifo_t *g_fw_data_fifo = 0;
//static gdsl_fifo_t *g_fw_data_index_fifo = 0;
static gdsl_fifo_t *g_fw_evt_fifo = 0;
static gdsl_fifo_t *g_fw_type_fifo = 0;
static gdsl_fifo_t *g_lib_cmd_fifo = 0;

static unsigned char *g_lib_cmd_buff = NULL;
static unsigned char *g_fw_data_buf = NULL;
static unsigned char *g_fw_evt_buf  = NULL;
//static unsigned char *g_fw_data_index_buf  = NULL;

static unsigned int gdsl_fifo_remain(gdsl_fifo_t *p_fifo);
static unsigned suspend_value = 0;
static unsigned shutdown_value = 0;
//DECLARE_WAIT_QUEUE_HEAD(poll_amlbt_queue);
//DEFINE_MUTEX(usb_rw_lock);

enum
{
    LOG_LEVEL_FATAL,
    LOG_LEVEL_ERROR,
    LOG_LEVEL_WARN,
    LOG_LEVEL_INFO,
    LOG_LEVEL_DEBUG,
    LOG_LEVEL_ALL,
};

static int g_dbg_level = LOG_LEVEL_INFO;

#define BTA(fmt, arg...) if (g_dbg_level >= LOG_LEVEL_ALL) printk(KERN_INFO "BTA:" fmt, ## arg)
#define BTD(fmt, arg...) if (g_dbg_level >= LOG_LEVEL_DEBUG) printk(KERN_INFO "BTD:" fmt, ## arg)
#define BTI(fmt, arg...) if (g_dbg_level >= LOG_LEVEL_INFO) printk(KERN_INFO "BTI:" fmt, ## arg)
#define BTW(fmt, arg...) if (g_dbg_level >= LOG_LEVEL_WARN) printk(KERN_INFO "BTW:" fmt, ## arg)
#define BTE(fmt, arg...) if (g_dbg_level >= LOG_LEVEL_ERROR) printk(KERN_INFO "BTE:" fmt, ## arg)
#define BTF(fmt, arg...) if (g_dbg_level >= LOG_LEVEL_FATAL) printk(KERN_INFO "BTF:" fmt, ## arg)

void amlbt_buff_init(void)
{
    unsigned int i = 0;
    unsigned int buff_size = 0;
    unsigned int index = 0;
    unsigned int buf_size[] =
    {
        TYPE_FIFO_SIZE,             //type
        HCI_MAX_FRAME_SIZE,         //p_acl_buf
        //HI_USB_RX_TYPE_FIFO_LEN,    //type_buff
        DATA_FIFO_SIZE,             //hci data buff
        EVT_FIFO_SIZE,              //hci event buff
        CMD_FIFO_SIZE,              //g_lib_cmd_buff
        //DATA_INDEX_SIZE,            //g_fw_data_index_buf
    };

    for (i = 0; i < sizeof(buf_size)/sizeof(buf_size[0]); i++)
    {
        buff_size += buf_size[i];
    }
    BTI("%s size %d, usb_buf %#x", __func__, buff_size, (unsigned long)usb_buf);
    if (usb_buf == NULL)
    {
        usb_buf = kzalloc(buff_size, GFP_DMA|GFP_ATOMIC);
        if (usb_buf == NULL)
        {
            BTF("%s kzalloc falied!", __func__);
            return ;
        }
        type        = &usb_buf[index];
        index       += buf_size[0];
        p_acl_buf   = &usb_buf[index];
        index       += buf_size[1];
        //type_buff   = &usb_buf[index];
        //index       += buf_size[2];
        g_fw_data_buf = &usb_buf[index];
        index       += buf_size[2];
        g_fw_evt_buf = &usb_buf[index];
        index       += buf_size[3];
        g_lib_cmd_buff = &usb_buf[index];
        //index       += buf_size[4];
        //g_fw_data_index_buf = &usb_buf[index];
    }
    if (bt_usb_data_buf == NULL)
    {
        bt_usb_data_buf = kzalloc(HI_USB_BT_TOTAL_DATA_LENGTH, GFP_DMA|GFP_ATOMIC);
        if (bt_usb_data_buf == NULL)
        {
            BTF("%s bt_usb_data_buf kzalloc falied! \n", __func__);
            kfree(usb_buf);
            usb_buf = NULL;
            return ;
        }
    }
}

void amlbt_buff_deinit(void)
{
    kfree(usb_buf);
    kfree(bt_usb_data_buf);
    usb_buf         = NULL;
    bt_usb_data_buf = NULL;
    type            = NULL;
    p_acl_buf       = NULL;
    //type_buff       = NULL;
    g_fw_data_buf   = NULL;
    g_fw_evt_buf    = NULL;
    g_lib_cmd_buff  = NULL;
    //g_fw_data_index_buf = NULL;
}

static gdsl_fifo_t *gdsl_fifo_init(unsigned int len, unsigned char *base_addr)
{
    gdsl_fifo_t *p_fifo = (gdsl_fifo_t *)kzalloc(sizeof(gdsl_fifo_t), GFP_DMA|GFP_ATOMIC);

    BTA("%s \n", __func__);

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

unsigned int gdsl_fifo_used(gdsl_fifo_t *p_fifo)
{
    if (p_fifo->r <= p_fifo->w)
        return (p_fifo->w - p_fifo->r);

    return (p_fifo->size + p_fifo->w - p_fifo->r);
}

unsigned int gdsl_fifo_remain(gdsl_fifo_t *p_fifo)
{
    unsigned int used = gdsl_fifo_used(p_fifo);

    return p_fifo->size - used - 4;
}


unsigned int gdsl_fifo_copy_data(gdsl_fifo_t *p_fifo, unsigned char *buff, unsigned int len)
{
    unsigned int i = 0;
    BTA("copy d %d, %#x, %#x\n", len, p_fifo->base_addr, p_fifo->w);

    if (gdsl_fifo_remain(p_fifo) < len)
    {
        BTF("gdsl_fifo_copy_data no space!!\n");
        return 0;
    }

    while (i < len)
    {
        *(unsigned char *)((unsigned long)p_fifo->w + (unsigned long)p_fifo->base_addr) = buff[i];
        p_fifo->w = (unsigned char *)(((unsigned long)p_fifo->w + 1) % p_fifo->size);
        i++;
    }

    BTA("actual len %#x \n", __func__, i);

    return i;
}

unsigned int gdsl_fifo_get_data(gdsl_fifo_t *p_fifo, unsigned char *buff, unsigned int len)
{
    unsigned int used = gdsl_fifo_used(p_fifo);
    unsigned int i = 0;
    unsigned int get_len = (len >= used ? used : len);

    BTA("get d %d, %#x, %#x\n", get_len, p_fifo->base_addr, p_fifo->w);
    if (used == 0)
    {
        return 0;
    }

    while (i < get_len)
    {
        buff[i] = *(unsigned char *)((unsigned long)p_fifo->r + (unsigned long)p_fifo->base_addr);
        p_fifo->r = (unsigned char *)(((unsigned long)p_fifo->r + 1) % p_fifo->size);
        i++;
    }

    BTA("actual len %#x \n", __func__, i);

    return i;
}


unsigned int gdsl_write_data_by_ep(gdsl_fifo_t *p_fifo, unsigned char *data, unsigned int len, unsigned int ep)
{
    unsigned int index = 0;
    unsigned char *w = p_fifo->w;
    unsigned int i = 0;

    BTA("%s len:%d\n", __func__, len);

    len = ((len + 3) & 0xFFFFFFFC);
    if (gdsl_fifo_remain(p_fifo) < len)
    {
        BTF("write data no space!!\n");
        return 0;
    }

    if (w == 0)
    {
        BTA("w ep %#x\n", (unsigned long)p_fifo->base_addr);
        g_auc_hif_ops.hi_write_sram_for_bt(data, p_fifo->base_addr, len, ep);
        p_fifo->w = (unsigned char *)(unsigned long)(len % p_fifo->size);
        return len;
    }

    while (i < len)
    {
        w = (unsigned char *)(((unsigned long)w + 1) % p_fifo->size);
        i++;
        if (w == 0)
        {
            BTA("w ep2 %#x\n", (unsigned long)p_fifo->w);
            g_auc_hif_ops.hi_write_sram_for_bt(data,
                (unsigned char *)((unsigned long)p_fifo->w + (unsigned long)p_fifo->base_addr), i, ep);
            p_fifo->w = 0;
            index = i;
        }
    }
    if (index < len)
    {
        BTA("w ep3 %#x\n", (unsigned long)p_fifo->w);
        g_auc_hif_ops.hi_write_sram_for_bt(&data[index],
            (unsigned char *)((unsigned long)p_fifo->w + (unsigned long)p_fifo->base_addr), len - index, ep);
    }
    p_fifo->w = w;
    return len;
}

#if 0
unsigned int gdsl_read_data_by_ep(gdsl_fifo_t *p_fifo, unsigned char *data, unsigned int len, unsigned int ep)
{
    unsigned int index = 0;
    unsigned char *r = p_fifo->r;
    unsigned int i = 0;
    unsigned int used = gdsl_fifo_used(p_fifo);
    unsigned int read_len = (used >= len ? len : used);

    BTD("%s len:%d\n", __func__, len);
    BTD("rd %#x, %#x, %d, %d\n", (unsigned long)p_fifo->r, (unsigned long)p_fifo->w, len, read_len);
    if (r == 0)
    {
        g_auc_hif_ops.hi_read_sram_for_bt(data, p_fifo->base_addr, read_len, ep);
        p_fifo->r = (unsigned char *)(unsigned long)(read_len % p_fifo->size);
        return read_len;
    }

    while (i < read_len)
    {
        r = (unsigned char *)(((unsigned long)r + 1) % p_fifo->size);
        i++;
        if (r == 0)
        {
            g_auc_hif_ops.hi_read_sram_for_bt(data,
                (unsigned char *)((unsigned long)p_fifo->r + (unsigned long)p_fifo->base_addr), i, ep);
            p_fifo->r = 0;
            index = i;
        }
    }
    if (index < read_len)
    {
        g_auc_hif_ops.hi_read_sram_for_bt(&data[index],
            (unsigned char *)((unsigned long)p_fifo->r + (unsigned long)p_fifo->base_addr), read_len - index, ep);
    }
    p_fifo->r = r;
    BTD("rde %#x, %#x, %d\n", (unsigned long)p_fifo->r, (unsigned long)p_fifo->w, index);
    return read_len;
}
#endif

unsigned int amlbt_usb_get_tx_prio(gdsl_tx_q_t *p_fifo, unsigned int acl_handle)
{
    unsigned int prio = 0;
    unsigned int i = 0;
    unsigned int find = 0;

    for (i = 0; i < WF_SRAM_TX_Q_NUM; i++)
    {
        if (p_fifo[i].tx_q_dev_index == acl_handle/* && p_fifo[i].tx_q_status == GDSL_TX_Q_USED*/)
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
        prio = BT_USB_MAX_PRIO;
    }

    return prio;
}

unsigned int amlbt_usb_get_tx_q(gdsl_tx_q_t *p_fifo, unsigned int acl_handle)
{
    unsigned int prio = BT_USB_MAX_PRIO;
    unsigned int i = 0;
    unsigned int index = WF_SRAM_TX_Q_NUM;

    for (i = 0; i < WF_SRAM_TX_Q_NUM; i++)
    {
        if (p_fifo[i].tx_q_dev_index == acl_handle && p_fifo[i].tx_q_status == GDSL_TX_Q_USED)
        {
            if (prio == BT_USB_MAX_PRIO)
            {
                prio = p_fifo[i].tx_q_prio;
                index = i;
            }
            else if (p_fifo[i].tx_q_prio < prio)
            {
                prio = p_fifo[i].tx_q_prio;
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

        if (FAMILY_TYPE_IS_W2(amlbt_if_type))
        {
            //update read pointer
            g_auc_hif_ops.hi_write_word_for_bt(WF_SRAM_RX_TYPE_FIFO_R_ADDR, (unsigned int)(unsigned long)g_rx_type_fifo->r,
                                               USB_EP2);
            //update write pointer
            g_auc_hif_ops.hi_write_word_for_bt(WF_SRAM_RX_TYPE_FIFO_W_ADDR, (unsigned int)(unsigned long)g_rx_type_fifo->w,
                                               USB_EP2);
        }
        //g_rx_type_fifo->r = (unsigned char *)(unsigned long)(WF_SRAM_RX_TYPE_FIFO_ADDR);
    }
}

void amlbt_usb_rx_type_fifo_deinit(void)
{
    if (g_rx_type_fifo)
    {
        if (FAMILY_TYPE_IS_W2(amlbt_if_type))
        {
            g_auc_hif_ops.hi_write_word_for_bt(WF_SRAM_RX_TYPE_FIFO_R_ADDR, 0, USB_EP2);
            g_auc_hif_ops.hi_write_word_for_bt(WF_SRAM_RX_TYPE_FIFO_W_ADDR, 0, USB_EP2);
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

        if (FAMILY_TYPE_IS_W2(amlbt_if_type))
        {
            g_auc_hif_ops.hi_write_word_for_bt(WF_SRAM_CMD_FIFO_R_ADDR, (unsigned int)(unsigned long)g_cmd_fifo->r, USB_EP2);
            BTA("cmd fifo init r: %#lx\n", (unsigned long)g_cmd_fifo->r);
            //update write pointer
            g_auc_hif_ops.hi_write_word_for_bt(WF_SRAM_CMD_FIFO_W_ADDR, (unsigned int)(unsigned long)g_cmd_fifo->w, USB_EP2);
        }
        BTA("cmd fifo init w : %#lx\n", (unsigned long)g_cmd_fifo->w);
        //g_cmd_fifo->w = (unsigned char *)(unsigned long)(WF_SRAM_CMD_Q_ADDR);
    }
}

void amlbt_usb_hci_cmd_fifo_deinit(void)
{
    BTD("%s \n", __func__);
    if (g_cmd_fifo != 0)
    {
        if (FAMILY_TYPE_IS_W2(amlbt_if_type))
        {
            g_auc_hif_ops.hi_write_word_for_bt(WF_SRAM_CMD_FIFO_R_ADDR, 0, USB_EP2);
            g_auc_hif_ops.hi_write_word_for_bt(WF_SRAM_CMD_FIFO_W_ADDR, 0, USB_EP2);
        }
        gdsl_fifo_deinit(g_cmd_fifo);
        g_cmd_fifo = 0;
    }
}

void amlbt_usb_hci_tx_data_init(void)
{
    unsigned int i = 0;
    unsigned int tx_info[WF_SRAM_TX_Q_NUM * 4] = {0};

    if (g_tx_q == 0)
    {
        g_tx_q = (gdsl_tx_q_t *)kzalloc(sizeof(gdsl_tx_q_t) * WF_SRAM_TX_Q_NUM, GFP_KERNEL);
    }

    for (i = 0; i < WF_SRAM_TX_Q_NUM; i++)
    {
        g_tx_q[i].tx_q_addr = (unsigned char *)(unsigned long)(WF_SRAM_TX_Q_ADDR + i * TX_Q_LEN);

        g_tx_q[i].tx_q_prio_addr = (unsigned int *)(unsigned long)(WF_SRAM_TX_Q_PRIO_ADDR + i * 16);
        g_tx_q[i].tx_q_dev_index_addr = (unsigned int *)((unsigned long)g_tx_q[i].tx_q_prio_addr + 4);
        g_tx_q[i].tx_q_status_addr = (unsigned int *)((unsigned long)g_tx_q[i].tx_q_dev_index_addr + 4);

        g_tx_q[i].tx_q_dev_index = 0;
        g_tx_q[i].tx_q_prio = BT_USB_MAX_PRIO;
        g_tx_q[i].tx_q_status = GDSL_TX_Q_UNUSED;
        tx_info[i*4] = g_tx_q[i].tx_q_prio;
        tx_info[i*4+1] = g_tx_q[i].tx_q_dev_index;
        tx_info[i*4+2] = g_tx_q[i].tx_q_status;
        BTA("tx_addr:%#x,%#x,%#x\n", (unsigned long)g_tx_q[i].tx_q_prio_addr,
            (unsigned long)g_tx_q[i].tx_q_dev_index_addr,
            (unsigned long)g_tx_q[i].tx_q_status_addr);
    }
    //if (FAMILY_TYPE_IS_W2(amlbt_if_type))
    {
        g_auc_hif_ops.hi_write_sram_for_bt((unsigned char *)tx_info,
                                           (unsigned char *)WF_SRAM_TX_Q_PRIO_ADDR, sizeof(tx_info),
                                           USB_EP2);
    }
}

void amlbt_usb_hci_tx_data_deinit(void)
{
    unsigned int tx_info[WF_SRAM_TX_Q_NUM * 4] = {0};

    BTD("%s \n", __func__);

    if (g_tx_q)
    {
        if (FAMILY_TYPE_IS_W2(amlbt_if_type))
        {
            g_auc_hif_ops.hi_write_sram_for_bt((unsigned char *)tx_info,
                                               (unsigned char *)WF_SRAM_TX_Q_PRIO_ADDR, sizeof(tx_info),
                                               USB_EP2);
        }
        kfree(g_tx_q);
        g_tx_q = 0;
    }
    BTD("%s end \n", __func__);
}

void amlbt_usb_hci_evt_fifo_init(void)
{
    if (g_event_fifo == 0)
    {
        g_event_fifo = gdsl_fifo_init(WF_SRAM_EVENT_LEN, (unsigned char *)(WF_SRAM_EVENT_Q_ADDR));
        if (FAMILY_TYPE_IS_W2(amlbt_if_type))
        {
            g_auc_hif_ops.hi_write_word_for_bt(WF_SRAM_EVT_FIFO_R_ADDR, (unsigned int)(unsigned long)g_event_fifo->r, USB_EP2);
            BTA("event fifo init r: %#lx\n", (unsigned long)g_event_fifo->r);
            g_auc_hif_ops.hi_write_word_for_bt(WF_SRAM_EVT_FIFO_W_ADDR, (unsigned int)(unsigned long)g_event_fifo->w, USB_EP2);
        }
        BTA("event fifo init w : %#lx\n", (unsigned long)g_event_fifo->w);
        //g_event_fifo->r = (unsigned char *)(unsigned long)(WF_SRAM_EVENT_Q_ADDR);
    }
}

void amlbt_usb_hci_evt_fifo_deinit(void)
{
    BTD("%s \n", __func__);
    if (g_event_fifo != 0)
    {
        if (FAMILY_TYPE_IS_W2(amlbt_if_type))
        {
            g_auc_hif_ops.hi_write_word_for_bt(WF_SRAM_EVT_FIFO_R_ADDR, 0, USB_EP2);
            g_auc_hif_ops.hi_write_word_for_bt(WF_SRAM_EVT_FIFO_W_ADDR, 0, USB_EP2);
        }
        gdsl_fifo_deinit(g_event_fifo);
        g_event_fifo = 0;
    }
    BTD("%s end \n", __func__);
}

void amlbt_usb_fw_recv_fifo_init(void)
{
    if (g_rx_fifo == 0)
    {
        g_rx_fifo = gdsl_fifo_init(WF_SRAM_RX_FIFO_LEN, (unsigned char *)(WF_SRAM_RX_Q_FIFO_ADDR));
        if (FAMILY_TYPE_IS_W2(amlbt_if_type))
        {
            g_auc_hif_ops.hi_write_word_for_bt(WF_SRAM_RX_FIFO_R_ADDR, (unsigned int)(unsigned long)g_rx_fifo->r, USB_EP2);
            BTA("recv fifo init r: %#lx\n", (unsigned long)g_rx_fifo->r);
            g_auc_hif_ops.hi_write_word_for_bt(WF_SRAM_RX_FIFO_W_ADDR, (unsigned int)(unsigned long)g_rx_fifo->w, USB_EP2);
        }
        BTA("recv fifo init w : %#lx\n", (unsigned long)g_rx_fifo->w);
        //g_rx_fifo->r = (unsigned char *)(unsigned long)(WF_SRAM_RX_Q_FIFO_ADDR);
    }
}

void amlbt_usb_fw_recv_fifo_deinit(void)
{
    BTD("%s \n", __func__);
    if (g_rx_fifo != 0)
    {
        if (FAMILY_TYPE_IS_W2(amlbt_if_type))
        {
            g_auc_hif_ops.hi_write_word_for_bt(WF_SRAM_RX_FIFO_R_ADDR, 0, USB_EP2);
            g_auc_hif_ops.hi_write_word_for_bt(WF_SRAM_RX_FIFO_W_ADDR, 0, USB_EP2);
        }
        gdsl_fifo_deinit(g_rx_fifo);
        g_rx_fifo = 0;
    }
}

static void amlbt_usb_send_hci_cmd(unsigned char *data, unsigned int len)
{
    BTA("%s, len %d \n", __func__, len);

    if (g_cmd_fifo == NULL)
    {
        BTF("%s: bt_usb_hci_cmd_fifo NULL!!!!\n", __func__);
        return ;
    }

    len = ((len + 3) & 0xFFFFFFFC);//Keep 4 bytes aligned

    BTA("%s, Actual length %d \n", __func__, len);
    //step 1: Update the command FIFO read pointer
    if (FAMILY_TYPE_IS_W2(amlbt_if_type))
    {
        g_cmd_fifo->r = (unsigned char *)(unsigned long)g_auc_hif_ops.hi_read_word_for_bt(WF_SRAM_CMD_FIFO_R_ADDR, USB_EP2);
    }
    BTA("cmd r %#x\n", (unsigned long)g_cmd_fifo->r);
    //step 2: Check the command FIFO space

    //step 3: Write HCI commands to WiFi SRAM
    if (FAMILY_TYPE_IS_W2(amlbt_if_type))
    {
        gdsl_write_data_by_ep(g_cmd_fifo, data, len, USB_EP2);
    }
    //step 4: Update the write pointer and write to WiFi SRAM

    BTA("before write:r:%#lx, w:%#lx\n", (unsigned long)g_cmd_fifo->r, (unsigned long)g_cmd_fifo->w);

    if (FAMILY_TYPE_IS_W2(amlbt_if_type))
    {
        g_auc_hif_ops.hi_write_word_for_bt(WF_SRAM_CMD_FIFO_W_ADDR,
                                           (unsigned long)g_cmd_fifo->w & 0xfff, USB_EP2);
    }
    BTA("len %#x:w %#lx, r %#lx\n", len, (unsigned long)g_cmd_fifo->w, (unsigned long)g_cmd_fifo->r);
}

static void amlbt_usb_send_hci_data(unsigned char *data, unsigned int len)
{
    unsigned int i = 0;
    unsigned int acl_handle = (((data[1] << 8) | data[0]) & 0xfff);
    unsigned int prio = 0;
    unsigned int tx_q_prio[WF_SRAM_TX_Q_NUM] = {0};
    unsigned int tx_q_index[WF_SRAM_TX_Q_NUM] = {0};
    unsigned int tx_q_status[WF_SRAM_TX_Q_NUM] = {0};
    unsigned int tx_buff[WF_SRAM_TX_Q_NUM * 4] = {0};//prio, index, status
    BTA("%s, len:%d\n", __func__, len);

    g_auc_hif_ops.hi_read_sram_for_bt((unsigned char *)tx_buff, (unsigned char *)g_tx_q[0].tx_q_prio_addr,
                                          sizeof(tx_buff), USB_EP2);
    for (i = 0; i < WF_SRAM_TX_Q_NUM; i++)
    {
        tx_q_prio[i] = tx_buff[i*4];
        tx_q_index[i]  = tx_buff[i*4+1];
        tx_q_status[i]   = tx_buff[i*4+2];
    }

    for (i = 0; i < WF_SRAM_TX_Q_NUM; i++)
    {
        if (tx_q_status[i] == GDSL_TX_Q_COMPLETE/* && tx_q_prio[i] == 0*/)
        {
            //acl_handle = tx_q_index[i];
            tx_q_index[i] = 0;
            tx_q_status[i] = GDSL_TX_Q_UNUSED;
            tx_q_prio[i] = BT_USB_MAX_PRIO;
        }
    }

    for (i = 0; i < WF_SRAM_TX_Q_NUM; i++)
    {
        g_tx_q[i].tx_q_dev_index = tx_q_index[i];
        g_tx_q[i].tx_q_status = tx_q_status[i];
        g_tx_q[i].tx_q_prio = tx_q_prio[i];
    }

    for (i = 0; i < WF_SRAM_TX_Q_NUM; i++)
    {
        if (g_tx_q[i].tx_q_status == GDSL_TX_Q_UNUSED)
        {
            break;
        }
    }

    if (i == WF_SRAM_TX_Q_NUM)
    {
        BTF("%s: hci data space invalid!!!! \n", __func__);
        for (i = 0; i < WF_SRAM_TX_Q_NUM; i++)
        {
            BTF("[%#x,%#x,%#x]", (unsigned int)g_tx_q[i].tx_q_prio,
                    (unsigned int)g_tx_q[i].tx_q_dev_index,
                    (unsigned int)g_tx_q[i].tx_q_status);
            BTF("{%#x,%#x,%#x}", tx_q_prio[i], tx_q_index[i],tx_q_status[i]);
        }
        return ;
    }

    BTD("%s idle queue index : %d, handle:%#x\n", __func__, i, acl_handle);

    prio = amlbt_usb_get_tx_prio(g_tx_q, acl_handle);

    g_tx_q[i].tx_q_prio = (++prio & BT_USB_MAX_PRIO);
    g_tx_q[i].tx_q_dev_index = acl_handle;
    g_tx_q[i].tx_q_status = GDSL_TX_Q_USED;


    if (FAMILY_TYPE_IS_W2(amlbt_if_type))
    {
        g_auc_hif_ops.hi_write_sram_for_bt(data, g_tx_q[i].tx_q_addr, len, USB_EP2);
    }

    tx_buff[0] = g_tx_q[i].tx_q_prio;
    tx_buff[1] = g_tx_q[i].tx_q_dev_index;
    tx_buff[2] = g_tx_q[i].tx_q_status;
    tx_buff[3] = 0;
    g_auc_hif_ops.hi_write_sram_for_bt((unsigned char *)&tx_buff[0], (unsigned char *)g_tx_q[i].tx_q_prio_addr,
                                      sizeof(unsigned int)*4, USB_EP2);

    BTA("%s, Actual length:%d\n", __func__, len);
}

#if 0
static unsigned int amlbt_usb_recv_hci_event(unsigned char *buff, unsigned int cnt)
{
    unsigned int len = 0;
    //unsigned int i = 0;

    //g_event_fifo->w = (unsigned char *)(unsigned long)g_auc_hif_ops.hi_read_word_for_bt(WF_SRAM_EVT_FIFO_W_ADDR, USB_EP2);
    //g_event_fifo->w += (WF_SRAM_EVENT_Q_ADDR);

    BTD("%s\n", __func__);
#if AML_FW_POINTER
    printk("evt r:%#lx,w:%#lx\n", (unsigned long)g_event_fifo->r - WF_SRAM_EVENT_Q_ADDR, (unsigned long)g_event_fifo->w - WF_SRAM_EVENT_Q_ADDR);
#endif
    if (g_event_fifo->w != g_event_fifo->r)
    {
        len = gdsl_read_data_by_ep(g_event_fifo, buff, cnt, USB_EP2);
    }
#if AML_FW_POINTER
    printk("read event fifo len %d\n", len);
    if (len)
    {
        printk("event data:\n");
        for (; i < len; i++)
        {
            printk(KERN_CONT "%#x|", buff[i]);
        }
    }
#endif
    return len;
}

static unsigned int amlbt_usb_recv_rx_type(unsigned char *buff, unsigned int cnt)
{
    unsigned int len = 0;

    //g_rx_type_fifo->w = (unsigned char *)(unsigned long)g_auc_hif_ops.hi_read_word_for_bt(WF_SRAM_RX_TYPE_FIFO_W_ADDR, USB_EP2);
    //g_rx_type_fifo->w += (WF_SRAM_RX_TYPE_FIFO_ADDR);

    BTD("%s\n", __func__);

    if (g_rx_type_fifo->w != g_rx_type_fifo->r)
    {
        len = gdsl_read_data_by_ep(g_rx_type_fifo, buff, cnt, USB_EP2);
    }
#if AML_FW_POINTER
    printk("read rx type fifo len %d\n", len);
    if (len)
    {
        printk("rx type data:\n");
        for (; i < len; i++)
        {
            printk(KERN_CONT "%#x|", buff[i]);
        }
    }
#endif
    return len;
}

static unsigned int amlbt_usb_recv_data_index(unsigned char *buff, unsigned int cnt)
{
    unsigned int len = 0;

    //g_rx_fifo->w = (unsigned char *)(unsigned long)g_auc_hif_ops.hi_read_word_for_bt(WF_SRAM_RX_FIFO_W_ADDR, USB_EP2);
    //g_rx_fifo->w += (WF_SRAM_RX_Q_FIFO_ADDR);

    BTD("%s\n", __func__);

    if (g_rx_fifo->w != g_rx_fifo->r)
    {
        len = gdsl_read_data_by_ep(g_rx_fifo, buff, cnt, USB_EP2);
    }
#if AML_FW_POINTER
    printk("read rx data index len %d\n", len);
    if (len)
    {
        printk("rx data index data:\n");
        for (; i < len; i++)
        {
            printk(KERN_CONT "%#x|", buff[i]);
        }
    }
#endif
    return len;
}
#endif


void amlbt_usb_fifo_init(void)
{
    unsigned int st_reg = 0;

    //BTD("%s\n", __func__);

    if (FAMILY_TYPE_IS_W2(amlbt_if_type))
    {
        st_reg = g_auc_hif_ops.hi_read_word_for_bt(WF_SRAM_FW_DRIVER_STATUS_ADDR, USB_EP2);
        st_reg |= WF_SRAM_FD_INIT_FLAG;
        g_auc_hif_ops.hi_write_word_for_bt(WF_SRAM_FW_DRIVER_STATUS_ADDR, st_reg, USB_EP2);
    }
    amlbt_buff_init();
    amlbt_usb_rx_type_fifo_init();
    amlbt_usb_hci_cmd_fifo_init();
    amlbt_usb_hci_tx_data_init();
    amlbt_usb_hci_evt_fifo_init();
    amlbt_usb_fw_recv_fifo_init();
    //memset(BT_fwICCM, 0, BT_ICCM_SIZE);
    //memset(BT_fwDCCM, 0, BT_DCCM_SIZE);
    memset(type, 0, TYPE_FIFO_SIZE);

    g_fw_data_fifo = gdsl_fifo_init(DATA_FIFO_SIZE, g_fw_data_buf);
    //g_fw_data_fifo->w = g_fw_data_buf;
    //g_fw_data_fifo->r = g_fw_data_buf;
    g_fw_evt_fifo = gdsl_fifo_init(EVT_FIFO_SIZE, g_fw_evt_buf);
    //g_fw_evt_fifo->w = g_fw_evt_buf;
    //g_fw_evt_fifo->r = g_fw_evt_buf;
    //g_fw_data_index_fifo = gdsl_fifo_init(DATA_INDEX_SIZE, g_fw_data_index_buf);
    //g_fw_data_index_fifo->w = g_fw_data_index_buf;
    //g_fw_data_index_fifo->r = g_fw_data_index_buf;
    g_fw_type_fifo = gdsl_fifo_init(TYPE_FIFO_SIZE, type);
    //g_fw_type_fifo->w = type;
    //g_fw_type_fifo->r = type;

    st_reg &= ~(WF_SRAM_FD_INIT_FLAG);
    if (FAMILY_TYPE_IS_W2(amlbt_if_type))
    {
        g_auc_hif_ops.hi_write_word_for_bt(WF_SRAM_FW_DRIVER_STATUS_ADDR, st_reg, USB_EP2);
    }
}

void amlbt_usb_fifo_deinit(void)
{
    unsigned int st_reg = 0;

    if (FAMILY_TYPE_IS_W2(amlbt_if_type))
    {
        st_reg = g_auc_hif_ops.hi_read_word_for_bt(WF_SRAM_FW_DRIVER_STATUS_ADDR, USB_EP2);
        st_reg |= WF_SRAM_FD_INIT_FLAG;
        g_auc_hif_ops.hi_write_word_for_bt(WF_SRAM_FW_DRIVER_STATUS_ADDR, st_reg, USB_EP2);
    }
    amlbt_usb_rx_type_fifo_deinit();
    amlbt_usb_hci_cmd_fifo_deinit();
    amlbt_usb_hci_tx_data_deinit();
    amlbt_usb_hci_evt_fifo_deinit();
    amlbt_usb_fw_recv_fifo_deinit();
    gdsl_fifo_deinit(g_fw_data_fifo);
    gdsl_fifo_deinit(g_fw_evt_fifo);
    gdsl_fifo_deinit(g_fw_type_fifo);
    //gdsl_fifo_deinit(g_fw_data_index_fifo);
    g_fw_data_fifo = 0;
    g_fw_evt_fifo = 0;
    g_fw_type_fifo = 0;
    //g_fw_data_index_fifo = 0;

    st_reg &= ~(WF_SRAM_FD_INIT_FLAG);
    if (FAMILY_TYPE_IS_W2(amlbt_if_type))
    {
        g_auc_hif_ops.hi_write_word_for_bt(WF_SRAM_FW_DRIVER_STATUS_ADDR, st_reg, USB_EP2);
    }
}

static void amlbt_usb_init(void)
{
    BTD("%s\n", __func__);
    if (!download_fw)
    {
        amlbt_usb_fifo_init();
    }
    check_fw_rx_stask = kthread_run(amlbt_usb_check_fw_rx, NULL, "check_fw_rx_thread");
    BTD("%s amlbt_usb_check_fw_rx start\n");
}

static void amlbt_usb_deinit(void)
{
    BTD("%s\n", __func__);
    if (check_fw_rx_stask)
    {
        kthread_stop(check_fw_rx_stask);
        check_fw_rx_stask = NULL;
    }
    BTD("%s end\n", __func__);
}

static void amlbt_usb_reset(void)
{
    memset(p_acl_buf, 0, HCI_MAX_FRAME_SIZE);
    memset(type, 0, TYPE_FIFO_SIZE);
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

    fw_iccmLen = BT_ICCM_SIZE / 256;
    fw_iccmBuf = BT_fwICCM;
    fw_dccmLen = BT_DCCM_SIZE / 256;
    fw_dccmBuf = BT_fwDCCM;

    BTD("iccm check:\n");

    iccm_base_addr = BT_ICCM_AHB_BASE + BT_ICCM_ROM_LEN;
    p_check = (unsigned int *)fw_iccmBuf;
    for (offset = 0; offset < (fw_iccmLen / 4); offset++)
    {
        BTD("%s -s check %#x\n", __func__, iccm_base_addr);
        if (FAMILY_TYPE_IS_W2(amlbt_if_type))
        {
            st_reg = g_auc_hif_ops.hi_read_word_for_bt(iccm_base_addr, USB_EP2);
        }
        iccm_base_addr += 4;
        if (st_reg != *p_check)
        {
            BTF("iccm download data:%#x, raw data:%#x\n", st_reg, *p_check);
            BTF("iccm no match, offset = %#x!\n", offset);
            break;
        }
        p_check++;
    }
    BTD("iccm check size : %#x\n", offset);

    if (offset == (fw_iccmLen / 4))
    {
        BTD("iccm check pass\n");
    }

    BTD("dccm check:\n");

    dccm_base_addr = BT_DCCM_AHB_BASE;
    p_check = (unsigned int *)fw_dccmBuf;
    for (offset = 0; offset < fw_dccmLen / 4; offset++)
    {
        if (amlbt_if_type == AMLBT_TRANS_W2_USB)
        {
            st_reg = g_auc_hif_ops.hi_read_word_for_bt(dccm_base_addr, USB_EP2);
        }
        dccm_base_addr += 4;
        if (st_reg != *p_check)
        {
            BTF("dccm download data:%#x, raw data:%#x\n", st_reg, *p_check);
            BTF("dccm no match!\n");
            break;
        }
        p_check++;
    }
    BTD("dccm check size : %#x\n", offset);
    if (offset == fw_dccmLen / 4)
    {
        BTD("dccm check pass\n");
    }
}

#if AML_BT_ROM_CHECK
void amlbt_usb_rom_check(void)
{
    unsigned int offset = 0;
    unsigned long addr = 0;

    for (offset = 0; offset < 256 * 1024; offset += 512)
    {
        addr = (BT_ICCM_AHB_BASE + offset);
        g_auc_hif_ops.hi_read_sram_for_bt(fw_read_buff, (unsigned char *)addr,
                                          512, USB_EP2);
        if (memcmp(fw_read_buff, &bt_rom_code[offset], 512))
        {
            BTD("amlbt_usb_rom_check fail,%#x \n", offset);
            BTD("[%#x,%#x,%#x,%#x] \n", fw_read_buff[0], fw_read_buff[1], fw_read_buff[2], fw_read_buff[3]);
            BTD("[%#x,%#x,%#x,%#x] \n", bt_rom_code[offset], bt_rom_code[offset + 1],
                   bt_rom_code[offset + 2], bt_rom_code[offset + 3]);
            return ;
        }
    }

    BTD("amlbt_usb_rom_check pass,%#x \n", offset);
}
#endif

void amlbt_usb_write_firmware(unsigned char *buf, unsigned int len, unsigned int addr)
{
    if (FAMILY_TYPE_IS_W2(amlbt_if_type))
    {
        g_auc_hif_ops.hi_write_sram_for_bt(buf, (unsigned char *)(unsigned long)(addr), len, USB_EP2);
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

    fw_iccmLen = BT_ICCM_SIZE;
    fw_iccmBuf = BT_fwICCM;
    fw_dccmLen = BT_DCCM_SIZE;
    fw_dccmBuf = BT_fwDCCM;

    download_size = fw_iccmLen;

    //to do download bt fw
    BTI("bt_usb_download_firmware:iccm size %#x\n", download_size);
    //g_auc_hif_ops.bt_hi_write_word(0xf03050, 1);    //ram power down rg_ram_pd_shutdown_sw
    //g_auc_hif_ops.bt_hi_write_word(REG_DEV_RESET, 0);    //pmu down

    //if (amlbt_if_type == AMLBT_TRANS_W2_USB)
    //{
    //    g_auc_hif_ops.hi_write_word_for_bt(REG_DEV_RESET, 0x007, USB_EP2);    //pmu up
    //}

    //g_auc_hif_ops.bt_hi_write_word(0xf03050, 0);    //ram power up
    remain_len = (download_size - offset);
    //printk("bt_usb_download_firmware:remain_len %#x\n", remain_len);
    while (offset < download_size)
    {
        if (remain_len < WF_SRAM_FW_DOWNLOAD_SIZE)
        {
            //printk("bt_usb_download_firmware iccm1 offset %#x\n", offset);
            amlbt_usb_write_firmware((unsigned char *)&fw_iccmBuf[offset], remain_len, iccm_base_addr);
            offset += remain_len;
            iccm_base_addr += remain_len;
            BTA("bt_usb_download_firmware iccm1 offset %#x, write_len %#x\n", offset, remain_len);
        }
        else
        {
            amlbt_usb_write_firmware((unsigned char *)&fw_iccmBuf[offset], WF_SRAM_FW_DOWNLOAD_SIZE, iccm_base_addr);
            offset += WF_SRAM_FW_DOWNLOAD_SIZE;
            remain_len -= WF_SRAM_FW_DOWNLOAD_SIZE;
            iccm_base_addr += WF_SRAM_FW_DOWNLOAD_SIZE;
            BTA("bt_usb_download_firmware iccm2 offset %#x, write_len %#x\n", offset, WF_SRAM_FW_DOWNLOAD_SIZE);
        }
        //printk("bt_usb_download_firmware iccm remain_len %#x\n", remain_len);
    }

    download_size = fw_dccmLen;

    //to do download bt fw
    //printk("bt_usb_download_firmware:dccm size %#x\n", download_size);
    offset = 0;
    remain_len = download_size;
    while (offset < download_size)
    {
        if (remain_len < WF_SRAM_FW_DOWNLOAD_SIZE)
        {
            amlbt_usb_write_firmware((unsigned char *)&fw_dccmBuf[offset], remain_len, dccm_base_addr);
            offset += remain_len;
            dccm_base_addr += remain_len;
        }
        else
        {
            amlbt_usb_write_firmware((unsigned char *)&fw_dccmBuf[offset], WF_SRAM_FW_DOWNLOAD_SIZE, dccm_base_addr);
            offset += WF_SRAM_FW_DOWNLOAD_SIZE;
            remain_len -= WF_SRAM_FW_DOWNLOAD_SIZE;
            dccm_base_addr += WF_SRAM_FW_DOWNLOAD_SIZE;
        }

        BTI("bt_usb_download_firmware dccm remain_len %#x \n", remain_len);
    }
    //amlbt_usb_firmware_check();
}

static int amlbt_usb_char_open(struct inode *inode_p, struct file *file_p)
{
    int rf_num= 0;
    BTI("%s, %#x \n", __func__, AML_BT_VERSION);
    close_state = 0;
    if (download_fw)
    {
#ifndef AML_BT_PRODUCTION_TOOLS
        download_fw = 0;
        BTI("%s, download_fw %d\n", __func__, download_fw);
#else
        amlbt_usb_reset();
        amlbt_usb_init();
        download_fw = 1;
        download_end = 0;
        download_flag = 1;
        fw_cmd_w = 0;
        fw_cmd_r = 0;
        rf_num = ((g_auc_hif_ops.hi_read_word_for_bt(REG_PMU_POWER_CFG, USB_EP2) >> BIT_RF_NUM) & 0x03);
        BTI("%s set rf num %#x", __func__, rf_num);
#endif
    }
    init_completion(&usb_completion);
    init_completion(&data_completion);
    return nonseekable_open(inode_p, file_p);
}
static void amlbt_usb_char_deinit(void);

static int amlbt_usb_char_close(struct inode *inode_p, struct file *file_p)
{
    BTI("%s $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ %#x\n", __func__, download_fw);

    if (g_event_fifo != 0)
    {
        BTI("event w:%p,r:%p\n", g_event_fifo->w, g_event_fifo->r);
    }

    if (download_fw)
    {
        amlbt_usb_deinit();
    }

    /*if (close_state == 2)
    {
        amlbt_usb_fifo_deinit();
        printk("remove driver clear reg\n");
    }*/
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

    BTA("R FW:%#x", count);

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
                download_fw = 1;
                download_end = 0;
                download_flag = 1;
                fw_cmd_r = 0;
                fw_cmd_w = 0;
                BTI("%s end \n", __func__);
            }
        }
        break;
    }

    return count;
}

/*
    rx type read reg    ->0x00  [0,1,2,3]
    event read reg      ->0x04  [4,5,6,7]
    rx fifo read reg    ->0x08  [8,9,10,11]
    dummy               ->0x0c  [12,13,14,15]
    dummy               ->0x10  [16,17,18,19]
    dummy               ->0x14  [20,21,22,23]
    dummy               ->0x18  [24,25,26,27]
    rx fifo write reg   ->0x1c  [28,29,30,31]
    rx type write reg   ->0x20  [32,33,34,35]
    event write reg     ->0x24  [36,37,38,39]
*/

static int amlbt_usb_get_data(void)
{
    unsigned int read_len = 0;
    unsigned char data_index[WF_SRAM_RX_FIFO_LEN] = {0};
    unsigned int type_size = 0;
    unsigned int evt_size = 0;
    unsigned int data_size = 0;
    unsigned long w_point = 0;
    static unsigned char fw_read_buff[WF_SRAM_EVENT_LEN] = {0};
    static unsigned char type_buff[RX_TYPE_FIFO_LEN] = {0};
    unsigned int i = 0;
    unsigned int reg = 0;
    unsigned char read_reg[16] = {0};
    gdsl_fifo_t read_fifo = {0};


    g_auc_hif_ops.hi_read_sram_for_bt(&bt_usb_data_buf[0], (unsigned char *)HI_USB_RX_TYPE_R_OFF, HI_USB_BT_TOTAL_DATA_LENGTH, USB_EP2);
    //1. process type
    w_point = ((bt_usb_data_buf[35]<<24)|(bt_usb_data_buf[34]<<16)|(bt_usb_data_buf[33]<<8)|bt_usb_data_buf[32]);
    g_rx_type_fifo->w = (unsigned char *)w_point;
    if (g_rx_type_fifo->w == g_rx_type_fifo->r)
    {
        return -EFAULT;
    }
    read_fifo.base_addr = &bt_usb_data_buf[HI_USB_RX_TYPE_FIFO_ADDR - HI_USB_RX_TYPE_R_OFF];
    read_fifo.r = g_rx_type_fifo->r;
    read_fifo.w = g_rx_type_fifo->w;
    read_fifo.size = HI_USB_RX_TYPE_FIFO_LEN;

    type_size = gdsl_fifo_get_data(&read_fifo, type_buff, sizeof(type_buff));

    if (type_size)
    {
        g_rx_type_fifo->r = read_fifo.r;
    }
    else
    {
        return -EFAULT;
    }

    //2. process event
    g_event_fifo->w = (unsigned char *)(unsigned long)((bt_usb_data_buf[39]<<24)|(bt_usb_data_buf[38]<<16)|(bt_usb_data_buf[37]<<8)|bt_usb_data_buf[36]);

    if (g_event_fifo->w != g_event_fifo->r)
    {
        read_fifo.base_addr = &bt_usb_data_buf[HI_USB_EVENT_Q_ADDR - HI_USB_RX_TYPE_R_OFF];
        read_fifo.r = g_event_fifo->r;
        read_fifo.w = g_event_fifo->w;
        read_fifo.size = HI_USB_EVENT_Q_LEN;
        evt_size = gdsl_fifo_get_data(&read_fifo, fw_read_buff, sizeof(fw_read_buff));
    }

    if (evt_size)
    {
        g_event_fifo->r = read_fifo.r;
        gdsl_fifo_copy_data(g_fw_evt_fifo, fw_read_buff, evt_size);
    }
    reg = (((unsigned int)(unsigned long)g_event_fifo->r) & 0x1fff);
    read_reg[4] = (reg & 0xff);
    read_reg[5] = ((reg >> 8) & 0xff);
    read_reg[6] = ((reg >> 16) & 0xff);
    read_reg[7] = ((reg >> 24) & 0xff);

    //3. process data
    g_rx_fifo->w = (unsigned char *)(unsigned long)((bt_usb_data_buf[31]<<24)|(bt_usb_data_buf[30]<<16)|(bt_usb_data_buf[29]<<8)|bt_usb_data_buf[28]);
    read_fifo.base_addr = &bt_usb_data_buf[HI_USB_RX_Q_FIFO_ADDR - HI_USB_RX_TYPE_R_OFF];
    read_fifo.r = g_rx_fifo->r;
    read_fifo.w = g_rx_fifo->w;
    read_fifo.size = HI_USB_RX_Q_FIFO_LEN;
    data_size = gdsl_fifo_get_data(&read_fifo, data_index, sizeof(data_index));
    if (data_size)
    {
        g_rx_fifo->r = read_fifo.r;
    }
    //data_size = amlbt_usb_recv_data_index(data_index, sizeof(data_index));

    reg = (((unsigned int)(unsigned long)g_rx_fifo->r) & 0x1f);
    read_reg[8] = (reg & 0xff);
    read_reg[9] = ((reg >> 8) & 0xff);
    read_reg[10] = ((reg >> 16) & 0xff);
    read_reg[11] = ((reg >> 24) & 0xff);

    for (i = 0; i < data_size; i+=4)
    {
        if (FAMILY_TYPE_IS_W2(amlbt_if_type))
        {
            g_auc_hif_ops.hi_read_sram_for_bt(&fw_read_buff[0],
                                              (unsigned char *)(unsigned long)(WF_SRAM_RX_Q_ADDR + data_index[i] * RX_Q_LEN), HI_USB_RX_Q_LEN,
                                              USB_EP2);
            read_len = ((fw_read_buff[7] << 8) | (fw_read_buff[6]));
            read_len = ((read_len + 3) & 0xFFFFFFFC);
            BTD("!%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x!\n", fw_read_buff[0],
                   fw_read_buff[1], fw_read_buff[2], fw_read_buff[3], fw_read_buff[4], fw_read_buff[5],
                   fw_read_buff[6], fw_read_buff[7]);
            BTD("r dh %#x\n", read_len);
        }
        gdsl_fifo_copy_data(g_fw_data_fifo, fw_read_buff, (read_len + 8));
        //printk("d:%#x,%#x\n", (unsigned long)g_fw_data_fifo->r, (unsigned long)g_fw_data_fifo->w);
        BTD("HEAD1:[%#x,%#x,%#x,%#x]\n", fw_read_buff[0],
               fw_read_buff[1], fw_read_buff[2], fw_read_buff[3]);
        BTD("HEAD2:[%#x,%#x,%#x,%#x]\n", fw_read_buff[4],
               fw_read_buff[5], fw_read_buff[6], fw_read_buff[7]);
        BTD("HEAD3:[%#x,%#x,%#x,%#x]\n", fw_read_buff[8],
               fw_read_buff[9], fw_read_buff[10], fw_read_buff[11]);
        BTD("HEAD4:[%#x,%#x,%#x,%#x]\n", fw_read_buff[12],
               fw_read_buff[13], fw_read_buff[14], fw_read_buff[15]);
    }

    BTD("%s TYPE:[%#x,%#x,%#x,%#x]\n", __func__, type_buff[0],
           type_buff[4], type_buff[8], type_buff[12]);

    reg = (((unsigned int)(unsigned long)g_rx_type_fifo->r) & 0x1fff);
    read_reg[0] = (reg & 0xff);
    read_reg[1] = ((reg >> 8) & 0xff);
    read_reg[2] = ((reg >> 16) & 0xff);
    read_reg[3] = ((reg >> 24) & 0xff);
    g_auc_hif_ops.hi_write_sram_for_bt(&read_reg[0],
                                      (unsigned char *)(unsigned long)(HI_USB_RX_TYPE_R_OFF), 16,
                                      USB_EP2);
    gdsl_fifo_copy_data(g_fw_type_fifo, type_buff, type_size);
    complete(&data_completion);
    return 0;
}

int amlbt_usb_check_fw_rx(void *data)
{
    while (!kthread_should_stop())
    {
        while (suspend_value || shutdown_value)
        {
            usleep_range(20000, 20000);
            if (close_state)
            {
                BTI("%s R1 CLOSE\n", __func__);
                check_fw_rx_stask = NULL;
                break;
            }
        }
        if (!suspend_value)
        {
            amlbt_usb_get_data();
            if (!gdsl_fifo_used(g_fw_type_fifo))
            {
                usleep_range(1000, 1000);
            }
            if (close_state)
            {
                BTI("%s R CLOSE\n", __func__);
                check_fw_rx_stask = NULL;
                break;
            }
        }
    }
    complete(&data_completion);
    BTI("%s exit read fw rx thread\n", __func__);
    return 0;
}

static ssize_t amlbt_usb_char_read(struct file *file_p,
                                   char __user *buf_p,
                                   size_t count,
                                   loff_t *pos_p)
{
    unsigned char close_evt[7] = {0x04, 0x0e, 0x04, 0x01, 0x27, 0xfc, 0x00};
    static unsigned char host_read_buff[TX_Q_LEN] = {0};
    static unsigned char bt_type[4] = {0};
    unsigned int read_len = 0;

    BTA("%s start, count : %ld\n", __func__, count);
    BTA("fw recv fifo r:%#lx, w:%#lx\n", (unsigned long)g_rx_fifo->r, (unsigned long)g_rx_fifo->w);

    if (!download_fw || (fw_cmd_r < fw_cmd_w))
    {
        return amlbt_usb_char_read_fw(file_p, buf_p, count, pos_p);
    }

    BTD("rv(%#x, %d)\n", evt_state, count);

    switch (evt_state)
    {
        case 0:					//read type
            evt_state = 1;
            memset(bt_type, 0, sizeof(bt_type));
            while (suspend_value || shutdown_value)
            {
                usleep_range(20000, 20000);
                if (close_state)
                {
                    BTI("%s R1 CLOSE\n", __func__);
                    return 0;
                }
            }
            while (!gdsl_fifo_used(g_fw_type_fifo))
            {
                if (close_state)
                {
                    BTI("%s R2 CLOSE\n", __func__);
                    if (copy_to_user(buf_p, close_evt, count))
                    {
                        BTF("%s, copy_to_user error \n", __func__);
                        return -EFAULT;
                    }
                    return count;
                }
                wait_for_completion(&data_completion);
            }
            gdsl_fifo_get_data(g_fw_type_fifo, bt_type, sizeof(bt_type));
            BTA("tp(%#x,%#x,%#x,%#x)\n", bt_type[0],bt_type[1],bt_type[2],bt_type[3]);
            if (copy_to_user(buf_p, bt_type, count))
            {
                BTF("%s, copy_to_user error \n", __func__);
                return -EFAULT;
            }
        break;
        case 1:                 // read header
            evt_state = 2;
            if (bt_type[0] == HCI_EVENT_PKT)
            {
                if (close_state)
                {
                    BTI("%s R3 CLOSE\n", __func__);
                    if (copy_to_user(buf_p, &close_evt[1], count))
                    {
                        BTF("%s, copy_to_user error \n", __func__);
                        return -EFAULT;
                    }
                    return count;
                }
                gdsl_fifo_get_data(g_fw_evt_fifo, host_read_buff, 4);
                read_len = host_read_buff[2];
                read_len -= 1;
                read_len = ((read_len + 3) & 0xFFFFFFFC);
                gdsl_fifo_get_data(g_fw_evt_fifo, &host_read_buff[4], read_len);
                if (copy_to_user(buf_p, &host_read_buff[1], count))
                {
                    BTF("%s, copy_to_user error \n", __func__);
                    return -EFAULT;
                }
                BTD("E:%#x|%#x|%#x|%#x\n", host_read_buff[0], host_read_buff[1],
                       host_read_buff[2], host_read_buff[3]);
            }
            else if (bt_type[0] == HCI_ACLDATA_PKT)
            {
                gdsl_fifo_get_data(g_fw_data_fifo, host_read_buff, 8);
                read_len = ((host_read_buff[7] << 8) | (host_read_buff[6]));
                read_len = ((read_len + 3) & 0xFFFFFFFC);
                gdsl_fifo_get_data(g_fw_data_fifo, &host_read_buff[8], read_len);
                BTD("D:%#x|%#x|%#x|%#x\n", host_read_buff[0], host_read_buff[1],
                       host_read_buff[2], host_read_buff[3]);
                if (copy_to_user(buf_p, &host_read_buff[4], count))
                {
                    BTF("%s, copy_to_user error \n", __func__);
                    return -EFAULT;
                }
            }
        break;
        case 2:					//read payload
            evt_state = 0;
            if (bt_type[0] == HCI_EVENT_PKT)
            {
                BTD("E:%#x|%#x|%#x|%#x\n", host_read_buff[4], host_read_buff[5],
                       host_read_buff[6], host_read_buff[7]);
                if (close_state)
                {
                    BTI("%s R4 CLOSE\n", __func__);
                    if (copy_to_user(buf_p, &close_evt[3], count))
                    {
                        BTF("%s, copy_to_user error \n", __func__);
                        return -EFAULT;
                    }
                    return count;
                }
                if (copy_to_user(buf_p, &host_read_buff[3], count))
                {
                    BTF("%s, copy_to_user error \n", __func__);
                    return -EFAULT;
                }
            }
            else if (bt_type[0] == HCI_ACLDATA_PKT)
            {
                BTD("D:%#x|%#x|%#x|%#x|%#x|%#x|%#x|%#x\n", host_read_buff[4], host_read_buff[5],
                       host_read_buff[6], host_read_buff[7], host_read_buff[8],
                       host_read_buff[9], host_read_buff[10], host_read_buff[11]);
                if (copy_to_user(buf_p, &host_read_buff[8], count))
                {
                    BTF("%s, copy_to_user error \n", __func__);
                    return -EFAULT;
                }
            }
            break;
        default:
            BTF("%s, evt_state error!!\n", __func__);
            break;
    }
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


    BTA("W_FW:%#x\n", count);

    if (count == 1)
    {
        return count;
    }

    memset(p_acl_buf, 0, HCI_MAX_FRAME_SIZE);

    if (copy_from_user(p_acl_buf, buf_p, count))
    {
        BTF("%s: Failed to get data from user space\n", __func__);
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
        BTF("CMD_I:%#x\n", cmd_index);
        for (n = 0; n < 11; n++)
        {
            BTF("%#x|", p_acl_buf[n]);
        }
        BTF("---------------cmd error!------------");
        return -EINVAL;
    }

    gdsl_fifo_copy_data(g_lib_cmd_fifo, p_acl_buf, 2);

    if (p_acl_buf[0] == 0xf3 && p_acl_buf[1] == 0xfe)   //download fw
    {
        len = count - 7;
        offset = ((p_acl_buf[6] << 24) | (p_acl_buf[5] << 16) | (p_acl_buf[4] << 8) | p_acl_buf[3]);
        BTA("%#x,%#x,%#x\n", len, offset, dw_state);
        BTA("%#x,%#x,%#x,%#x\n", p_acl_buf[7], p_acl_buf[8], p_acl_buf[9], p_acl_buf[10]);
        if (offset == BT_ICCM_ROM_LEN)
        {
            if (!download_flag)
            {
                if (FAMILY_TYPE_IS_W2(amlbt_if_type))
                {
                    //printk("w2 usb write first reg\n");
                    g_auc_hif_ops.hi_write_word_for_bt(0xf03050, 0, USB_EP2);
                    //printk("w2 usb write first reg end\n");
                }
            }
            iccm_base_addr = 0;
            dccm_base_addr = 0;
            dw_state = 0;
            if (BT_fwICCM == NULL)
            {
                BT_fwICCM = vmalloc(BT_ICCM_SIZE);
            }
            if (BT_fwDCCM == NULL)
            {
                BT_fwDCCM = vmalloc(BT_DCCM_SIZE);
            }

            if (BT_fwICCM == NULL || BT_fwDCCM == NULL)
            {
                BTF("amlbt_usb_char_write_fw vmalloc err!!\n");
                if (BT_fwICCM != NULL)
                {
                    vfree(BT_fwICCM);
                    BT_fwICCM = NULL;
                }
                if (BT_fwDCCM != NULL)
                {
                    vfree(BT_fwDCCM);
                    BT_fwDCCM = NULL;
                }
                return -EINVAL;
            }
        }

        if (offset == DCCM_RAM_BASE)
        {
            BTD("DCCM_RAM_BASE have address!\n");
            dw_state = 1;
        }

        if (dw_state == 0)
        {
            memcpy(&BT_fwICCM[iccm_base_addr], &p_acl_buf[7], len);
            iccm_base_addr += len;
        }
        else
        {
            memcpy(&BT_fwDCCM[dccm_base_addr], &p_acl_buf[7], len);
            dccm_base_addr += len;
        }
    }
    else if (p_acl_buf[0] == 0xf1 && p_acl_buf[1] == 0xfe)
    {
        offset = ((p_acl_buf[6] << 24) | (p_acl_buf[5] << 16) | (p_acl_buf[4] << 8) | p_acl_buf[3]);
        reg_value = ((p_acl_buf[10] << 24) | (p_acl_buf[9] << 16) | (p_acl_buf[8] << 8) | p_acl_buf[7]);
        BTI("WR:%#x,%#x\n", offset, reg_value);
        if (offset == 0xa70014 && !(reg_value & (1 << 24))) //rf calibration
        {
            //amlbt_usb_firmware_check();
            if (!download_flag)
            {
                amlbt_usb_download_firmware();
                BTI("download finished!\n");
                vfree(BT_fwICCM);
                vfree(BT_fwDCCM);
                BT_fwICCM = NULL;
                BT_fwDCCM = NULL;
            }
            BTI("W E %#x,%#x\n", iccm_base_addr, dccm_base_addr);
        }
        else if (offset == REG_PMU_POWER_CFG)
        {
            if (FAMILY_TYPE_IS_W2(amlbt_if_type))
            {
                g_auc_hif_ops.hi_write_word_for_bt(REG_PMU_POWER_CFG, reg_value, USB_EP2); // rg_pmu_power_cfg
            }
        }
        else if (offset == REG_DEV_RESET)
        {
            download_end = 1;
            amlbt_usb_reset();
            amlbt_usb_init();
            if (FAMILY_TYPE_IS_W2(amlbt_if_type))
            {
                g_auc_hif_ops.hi_write_word_for_bt(REG_DEV_RESET, (unsigned int)((BIT_CPU | BIT_MAC | BIT_PHY) << DEV_RESET_SW),
                                                   USB_EP2);
                BTI("%s end 19:30,bt reg : %#x\n", __func__, g_auc_hif_ops.hi_read_word_for_bt(REG_DEV_RESET, USB_EP2));
            }
        }
    }
    else if (p_acl_buf[0] == 0xf2 && p_acl_buf[1] == 0xfe)
    {
        offset = ((p_acl_buf[6] << 24) | (p_acl_buf[5] << 16) | (p_acl_buf[4] << 8) | p_acl_buf[3]);
        reg_value = ((p_acl_buf[10] << 24) | (p_acl_buf[9] << 16) | (p_acl_buf[8] << 8) | p_acl_buf[7]);
        BTD("WC:%#x,%#x\n", offset, reg_value);
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

    BTA("%s, count:%ld\n", __func__, count);

    if (count > HCI_MAX_FRAME_SIZE)
    {
        BTF("count > HCI_MAX_FRAME_SIZE \n");
        return -EINVAL;
    }

    if (!download_fw)
    {
        return amlbt_usb_char_write_fw(file_p, buf_p, count, pos_p);
    }
    if (amlbt_if_type == AMLBT_TRANS_W2_USB)
    {
        BTD("W:%#x, %#x,%#x,%#x\n", count, download_fw, fw_cmd_r, fw_cmd_w);
    }
    if (count == 1)	//host write hci type
    {
        get_user(w_type, buf_p);
        return count;
    }

    if (copy_from_user(p_acl_buf, buf_p, count))
    {
        BTF("%s: Failed to get data from user space\n", __func__);
        //mutex_unlock(&usb_rw_lock);
        return -EFAULT;
    }

    if (count > 1 && w_type == HCI_COMMAND_PKT)
    {
        BTD("hci cmd:");
        for (i = 0; i < 8; i++)
        {
            BTD("%#x|", p_acl_buf[i]);
        }
    }

    BTD("s: %#x\n", w_type);
    if (w_type == HCI_COMMAND_PKT)
    {
        if (count == 0x0f && p_acl_buf[0] == 0x27 && p_acl_buf[1] == 0xfc)   //close
        {
            for (i = 0; i < 8; i++)
            {
                BTI("%#x|", p_acl_buf[i]);
            }
            BTI("W CLOSE\n");
            //mutex_unlock(&usb_rw_lock);
            close_state = 1;
            return count;
        }
        if (p_acl_buf[0] == 0x05 && p_acl_buf[1] == 0x14)     // read rssi
        {
            BTI("rs\n");
        }
        amlbt_usb_send_hci_cmd(p_acl_buf, count);
    }
    else if (w_type == HCI_ACLDATA_PKT)
    {
        amlbt_usb_send_hci_data(p_acl_buf, count);
    }

    return count;
}

unsigned int btchr_poll(struct file *file, poll_table *wait)
{
    //int mask = 0;
#if 0
    if (!download_fw)
    {
        return POLLIN | POLLRDNORM;
    }
    //printk("ps %#x\n", evt_state);
    if (evt_state)
    {
        return POLLIN | POLLRDNORM;
    }

    if (gdsl_fifo_used_size(g_fw_type_fifo))
    {
        return POLLIN | POLLRDNORM;
    }

    poll_wait(file, &poll_amlbt_queue, wait);
#endif
    return POLLIN | POLLRDNORM;
}

static long btchr_ioctl(struct file* filp, unsigned int cmd, unsigned long arg)
{
    long ret = download_fw;
    BTA("arg value %ld", arg);
    BTA("cmd type=%c\t nr=%d\t dir=%d\t size=%d\n", _IOC_TYPE(cmd), _IOC_NR(cmd), _IOC_DIR(cmd), _IOC_SIZE(cmd));
    BTA("cmd value %ld", cmd);
    switch (cmd)
    {
        case CMD_DOWNLOAD:
            BTI("CMD_DOWNLOAD Success\n");
            return ret;
            break;
        case CMD_USB_CRASH:
            BTI("CMD_USB_CRASH Success\n");
            download_fw = 0;
            download_end = 0;
            download_flag = 0;
            amlbt_usb_fifo_deinit();
            amlbt_buff_init();
            memset(g_lib_cmd_buff, 0, CMD_FIFO_SIZE);
            g_lib_cmd_fifo = gdsl_fifo_init(CMD_FIFO_SIZE, g_lib_cmd_buff);
            return 1;
            break;

        default:
            break;
    }
    return 0;
}


static struct file_operations amlbt_usb_fops  =
{
    .open = amlbt_usb_char_open,
    .release = amlbt_usb_char_close,
    .read = amlbt_usb_char_read,
    .write = amlbt_usb_char_write,
    //.poll = btchr_poll,
    //.unlocked_ioctl = btchr_ioctl,
    .compat_ioctl = btchr_ioctl,
};

static int amlbt_usb_char_init(void)
{
    int res = 0;
    struct device *dev;

    BTI("%s\n", __func__);

    //	init_timer(&r_timer);

    bt_char_class = class_create(THIS_MODULE, AML_BT_CHAR_DEVICE_NAME);
    if (IS_ERR(bt_char_class))
    {
        BTF("%s:Failed to create bt char class\n", __func__);
        return PTR_ERR(bt_char_class);
    }

    res = alloc_chrdev_region(&bt_devid, 0, 1, AML_BT_CHAR_DEVICE_NAME);
    if (res < 0)
    {
        BTF("%s:Failed to allocate bt char device\n", __func__);
        goto err_alloc;
    }

    dev = device_create(bt_char_class, NULL, bt_devid, NULL, AML_BT_CHAR_DEVICE_NAME);
    if (IS_ERR(dev))
    {
        BTF("%s:Failed to create bt char device\n", __func__);
        res = PTR_ERR(dev);
        goto err_create;
    }

    cdev_init(&bt_char_dev, &amlbt_usb_fops);
    res = cdev_add(&bt_char_dev, bt_devid, 1);
    if (res < 0)
    {
        BTF("%s:Failed to add bt char device\n", __func__);
        goto err_add;
    }
    //g_cdev = dev;
    BTD("%s end", __func__);
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
    BTD("%s \n", __func__);
    device_destroy(bt_char_class, bt_devid);
    class_destroy(bt_char_class);
    cdev_del(&bt_char_dev);
    unregister_chrdev_region(bt_devid, 1);
}

static void amlbt_dev_release(struct device *dev)
{
    return;
}
#if 0
static void amlbt_cmd_test(void)
{
    memset(BT_fwICCM, 0x55, BT_ICCM_SIZE);
    memset(BT_fwDCCM, 0xaa, BT_DCCM_SIZE);
    //amlbt_usb_download_firmware();
    g_auc_hif_ops.hi_write_sram_for_bt(BT_fwICCM, (unsigned char *)(unsigned long)(0x00300000 + 256 * 1024), 20 * 1024,
                                       USB_EP2);
    g_auc_hif_ops.hi_read_sram_for_bt(BT_fwDCCM, (unsigned char *)(unsigned long)(0x00300000 + 256 * 1024), 20 * 1024,
                                      USB_EP2);

    if (!memcmp(BT_fwICCM, BT_fwDCCM, 20 * 1024))
    {
        printk("check ok!\n");
    }
    else
    {
        printk("check err : %#x,%#x,%#x,%#x \n", BT_fwDCCM[0], BT_fwDCCM[1], BT_fwDCCM[2], BT_fwDCCM[3]);
    }
    memset(BT_fwDCCM, 0xaa, BT_DCCM_SIZE);
    g_auc_hif_ops.hi_write_sram_for_bt(BT_fwICCM, (unsigned char *)(unsigned long)(0x00400000), 20 * 1024, USB_EP2);
    g_auc_hif_ops.hi_read_sram_for_bt(BT_fwDCCM, (unsigned char *)(unsigned long)(0x00400000), 20 * 1024, USB_EP2);
    if (!memcmp(BT_fwICCM, BT_fwDCCM, 20 * 1024))
    {
        printk("check2 ok!\n");
    }
    else
    {
        printk("check2 err : %#x,%#x,%#x,%#x \n", BT_fwDCCM[0], BT_fwDCCM[1], BT_fwDCCM[2], BT_fwDCCM[3]);
    }
}

#endif
static int amlbt_sdio_fops_open(struct inode *inode, struct file *file)
{
    BTI("%s bt opened rf num:%#x\n", __func__, rf_num);
    if (sdio_buf == NULL)
    {
        sdio_buf = kzalloc(HCI_MAX_FRAME_SIZE, GFP_DMA|GFP_ATOMIC);
    }
    return 0;
}

static int amlbt_sdio_fops_close(struct inode *inode, struct file *file)
{
    BTI("%s BT closed\n", __func__);
    return 0;
}

static ssize_t amlbt_sdio_char_write(struct file *file_p,
                                     const char __user *buf_p,
                                     size_t count,
                                     loff_t *pos_p)
{
    unsigned int offset = 0;
    unsigned int reg_value = 0;
    if (copy_from_user(sdio_buf, buf_p, count))
    {
        BTF("%s: Failed to get data from user space\n", __func__);
        return -EFAULT;
    }

    offset = ((sdio_buf[3] << 24) | (sdio_buf[2] << 16) | (sdio_buf[1] << 8) | sdio_buf[0]);
    reg_value = ((sdio_buf[7] << 24) | (sdio_buf[6] << 16) | (sdio_buf[5] << 8) | sdio_buf[4]);
    BTD("WR:%#x,%#x\n", offset, reg_value);
    if (offset == REG_PMU_POWER_CFG)
    {
        if (FAMILY_TYPE_IS_W2(amlbt_if_type))
        {
            //g_hif_sdio_ops.bt_hi_write_word(REG_PMU_POWER_CFG, reg_value);
            //printk("%s REG_PMU_POWER_CFG:%#x\n", __func__, g_hif_sdio_ops.bt_hi_read_word(REG_PMU_POWER_CFG));
            rf_num = reg_value >> BIT_RF_NUM;
            BTI("BT set rf succeed!\n");
            BTI("%s set rf num %#x", __func__, rf_num);
        }
    }
    return count;
}

const struct file_operations amlbt_sdio_fops =
{
    .open       = amlbt_sdio_fops_open,
    .release    = amlbt_sdio_fops_close,
    .write      = amlbt_sdio_char_write,
    .poll       = NULL,
    .unlocked_ioctl = NULL,
    .fasync     = NULL
};

static int amlbt_sdio_init(void)
{
    int ret = 0;
    int cdevErr = 0;

    dev_t devID = MKDEV(amlbt_sdio_major, 0);
    BTD("amlbt_sdio_init\n");

    ret = alloc_chrdev_region(&devID, 0, 1, AML_BT_NOTE);
    if (ret)
    {
        BTF("fail to allocate chrdev\n");
        return ret;
    }

    amlbt_sdio_major = MAJOR(devID);
    BTI("major number:%d\n", amlbt_sdio_major);
    cdev_init(&amlbt_sdio_cdev, &amlbt_sdio_fops);
    amlbt_sdio_cdev.owner = THIS_MODULE;

    cdevErr = cdev_add(&amlbt_sdio_cdev, devID, amlbt_sdio_devs);
    if (cdevErr)
        goto error;

    BTI("%s driver(major %d) installed.\n",
           "BT_sdiodev", amlbt_sdio_major);

    amlbt_sdio_class = class_create(THIS_MODULE, AML_BT_NOTE);
    if (IS_ERR(amlbt_sdio_class))
    {
        BTF("class create fail, error code(%ld)\n",
               PTR_ERR(amlbt_sdio_class));
        goto err1;
    }

    amlbt_sdio_dev = device_create(amlbt_sdio_class, NULL, devID, NULL, AML_BT_NOTE);
    if (IS_ERR(amlbt_sdio_dev))
    {
        BTF("device create fail, error code(%ld)\n",
               PTR_ERR(amlbt_sdio_dev));
        goto err2;
    }

    BTI("%s: BT_major %d\n", __func__, amlbt_sdio_major);
    BTI("%s: devID %d\n", __func__, devID);

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
    BTI("BTAML version:%#x\n", AML_BT_VERSION);
    BTI("++++++sdio bt driver insmod start.++++++\n");
    ret = amlbt_sdio_init();
    if (ret)
    {
        BTF("%s: amlbt_sdio_init failed!\n", __func__);
        return ret;
    }
    BTI("------sdio bt driver insmod end.------\n");
    return ret;
}

static int amlbt_sdio_exit(void)
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

    BTI("%s driver removed.\n", AML_BT_NOTE);
    return 0;
}

static void amlbt_sdio_rmmod(void)
{
    BTI("++++++sdio bt driver rmmod start++++++\n");
    amlbt_sdio_exit();
    BTI("------sdio bt driver rmmod end------\n");
}

static int amlbt_sdio_probe(struct platform_device *dev)
{
    BTI("%s \n", __func__);
    amlbt_sdio_insmod();
    return 0;
}

static int amlbt_sdio_remove(struct platform_device *dev)
{
    BTI("%s \n", __func__);

    amlbt_sdio_rmmod();
    kfree(sdio_buf);
    sdio_buf = NULL;
    return 0;
}
static int amlbt_sdio_suspend(struct platform_device *dev, pm_message_t state)
{
    unsigned int reg_value = 0;
    if (FAMILY_TYPE_IS_W2(amlbt_if_type) && INTF_TYPE_IS_SDIO(amlbt_if_type))
    {
        reg_value = g_hif_sdio_ops.bt_hi_read_word(RG_AON_A52);
        BTI("%s RG_AON_A52:%#x\n", __func__, reg_value);
        reg_value |= (1 << 26);
        g_hif_sdio_ops.bt_hi_write_word(RG_AON_A52, reg_value);
        BTI("RG_AON_A52:%#x", g_hif_sdio_ops.bt_hi_read_word(RG_AON_A52));
    }
    if (FAMILY_TYPE_IS_W2(amlbt_if_type) && INTF_TYPE_IS_PCIE(amlbt_if_type))
    {
        reg_value = aml_pci_read_for_bt(AML_ADDR_AON, RG_AON_A52);
        BTI("%s RG_AON_A52:%#x\n", __func__, reg_value);
        reg_value |= (1 << 26);
        aml_pci_write_for_bt(reg_value, AML_ADDR_AON, RG_AON_A52);
        BTI("RG_AON_A52:%#x", aml_pci_read_for_bt(AML_ADDR_AON, RG_AON_A52));
    }
    BTI("%s \n", __func__);
    return 0;
}

static int amlbt_sdio_resume(struct platform_device *dev)
{
    unsigned int reg_value = 0;
    if (FAMILY_TYPE_IS_W2(amlbt_if_type) && INTF_TYPE_IS_SDIO(amlbt_if_type))
    {
        reg_value = g_hif_sdio_ops.bt_hi_read_word(RG_AON_A52);
        BTI("%s RG_AON_A52:%#x\n", __func__, reg_value);
        reg_value &= ~(1 << 26);
        g_hif_sdio_ops.bt_hi_write_word(RG_AON_A52, reg_value);
        BTI("RG_AON_A52:%#x", g_hif_sdio_ops.bt_hi_read_word(RG_AON_A52));
    }
    if (FAMILY_TYPE_IS_W2(amlbt_if_type) && INTF_TYPE_IS_PCIE(amlbt_if_type))
    {
        reg_value = aml_pci_read_for_bt(AML_ADDR_AON, RG_AON_A52);
        BTI("%s RG_AON_A52:%#x\n", __func__, reg_value);
        reg_value &= ~(1 << 26);
        aml_pci_write_for_bt(reg_value, AML_ADDR_AON, RG_AON_A52);
        BTI("RG_AON_A52:%#x", aml_pci_read_for_bt(AML_ADDR_AON, RG_AON_A52));
    }
    BTI("%s \n", __func__);
    return 0;
}

static void amlbt_sdio_shutdown(struct platform_device *dev)
{
    BTI("%s \n", __func__);
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

static void amlbt_usb_insmod(void)
{
    // int ret = 0;
    BTI("BTAML version:%#x\n", AML_BT_VERSION);
    BTI("++++++usb bt driver insmod start.++++++\n");
    BTI("------usb bt driver insmod end.------\n");

    //return ret;
}

static void amlbt_usb_rmmod(void)
{
    //if (amlbt_poweron == AML_SDIO_EN)
    {
        BTI("++++++usb bt driver rmmod start++++++\n");
        BTI("------usb bt driver rmmod end------\n");
    }
}

static int amlbt_usb_probe(struct platform_device *dev)
{
    int err = 0;
    amlbt_usb_insmod();   //load
    //g_auc_hif_ops.bt_hi_write_word((unsigned int)0x00a0d0e4, 0x8000007f);
    //printk("%s, %#x", __func__, g_auc_hif_ops.bt_hi_read_word(0x00a0d0e4));
    BTI("%s \n", __func__);
    err = amlbt_usb_char_init();
    if (err < 0)
    {
        /* usb register will go on, even bt char register failed */
        BTD("%s:Failed to register usb char device interfaces\n", __func__);
    }
    download_fw = 0;
    amlbt_buff_init();

    //amlbt_cmd_test();
    memset(g_lib_cmd_buff, 0, CMD_FIFO_SIZE);
    g_lib_cmd_fifo = gdsl_fifo_init(CMD_FIFO_SIZE, g_lib_cmd_buff);
    //g_lib_cmd_fifo->w = g_lib_cmd_buff;
    //g_lib_cmd_fifo->r = g_lib_cmd_buff;
#if AML_BT_ROM_CHECK
    amlbt_usb_rom_check();
#endif
    BTI("%s, end \n", __func__);
    return err;
}

static int amlbt_usb_remove(struct platform_device *dev)
{
    BTI("%s\n", __func__);

    amlbt_usb_rmmod();    //unload

    close_state = 2;
    if (g_fw_data_fifo == 0)
    {
        amlbt_usb_char_deinit();
    }
    gdsl_fifo_deinit(g_lib_cmd_fifo);
    g_lib_cmd_fifo = 0;
    amlbt_buff_deinit();
    msleep(500);
    BTI("%s end\n", __func__);
    return 0;
}

static int amlbt_usb_suspend(struct platform_device *dev, pm_message_t state)
{
    suspend_value = 1;
    if (FAMILY_TYPE_IS_W2(amlbt_if_type))
    {
        unsigned int reg_value = g_auc_hif_ops.hi_read_word_for_bt(RG_AON_A52, USB_EP2);
        BTI("%s RG_AON_A52:%#x\n", __func__, reg_value);
        reg_value |= (1 << 26);
        g_auc_hif_ops.hi_write_word_for_bt(RG_AON_A52, reg_value, USB_EP2);
        BTI("RG_AON_A52:%#x", g_auc_hif_ops.hi_read_word_for_bt(RG_AON_A52, USB_EP2));
    }
    BTI("%s \n", __func__);
    return 0;
}

static int amlbt_usb_resume(struct platform_device *dev)
{
    BTI("%s\n", __func__);
    msleep(1500);
    if (FAMILY_TYPE_IS_W2(amlbt_if_type))
    {
        unsigned int reg_value = g_auc_hif_ops.hi_read_word_for_bt(RG_AON_A52, USB_EP2);
        BTI("%s RG_AON_A52:%#x\n", __func__, reg_value);
        reg_value &= ~(1 << 26);
        g_auc_hif_ops.hi_write_word_for_bt(RG_AON_A52, reg_value, USB_EP2);
        BTI("RG_AON_A52:%#x", g_auc_hif_ops.hi_read_word_for_bt(RG_AON_A52, USB_EP2));
    }
    suspend_value = 0;
    BTI("%s \n", __func__);
    return 0;
}

static void amlbt_usb_shutdown(struct platform_device *dev)
{
    shutdown_value = 1;
    if (FAMILY_TYPE_IS_W2(amlbt_if_type))
    {
        unsigned int reg_value = g_auc_hif_ops.hi_read_word_for_bt(RG_AON_A52, USB_EP2);
        BTI("%s RG_AON_A52:%#x\n", __func__, reg_value);
        reg_value |= (1 << 27);
        g_auc_hif_ops.hi_write_word_for_bt(RG_AON_A52, reg_value, USB_EP2);
        BTI("%s RG_AON_A52:%#x", __func__, g_auc_hif_ops.hi_read_word_for_bt(RG_AON_A52, USB_EP2));
    }
    BTI("%s \n", __func__);
}

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

    BTI("%s, type:%d \n", __func__, amlbt_if_type);

    BTI("%s, type:%#x inter %s, family %s\n", __func__, amlbt_if_type,
           amlbt_family_intf((AMLBT_PD_ID_INTF & amlbt_if_type)),
           amlbt_family_intf(AMLBT_PD_ID_FAMILY & amlbt_if_type));

    if (INTF_TYPE_IS_USB(amlbt_if_type))        //usb interface
    {
        p_device = &amlbt_usb_device;
        p_driver = &amlbt_usb_driver;
    }
    else if (INTF_TYPE_IS_SDIO(amlbt_if_type) || INTF_TYPE_IS_PCIE(amlbt_if_type))  //sdio interface
    {
        p_device = &amlbt_sdio_device;
        p_driver = &amlbt_sdio_driver;
    }
    else
    {
        BTF("%s amlbt_if_type invalid!! \n", __func__);
        return ret;
    }

    BTD("%s 1 \n", __func__);

    ret = platform_device_register(p_device);
    if (ret)
    {
        dev_err(&p_device->dev, "platform_device_register failed!\n");
        return ret;
    }
    BTD("%s 2 \n", __func__);
    ret = platform_driver_register(p_driver);
    if (ret)
    {
        dev_err(&p_device->dev, "platform_driver_register failed!\n");
        return ret;
    }

    dev_info(&p_device->dev, "Init %d OK!\n", amlbt_if_type);

    return ret;
}

static void amlbt_exit(void)
{
    struct platform_device *p_device = NULL;
    struct platform_driver *p_driver = NULL;

    BTI("%s, type:%d \n", __func__, amlbt_if_type);

    if (INTF_TYPE_IS_USB(amlbt_if_type))        //usb interface
    {
        p_device = &amlbt_usb_device;
        p_driver = &amlbt_usb_driver;
    }
    else if (INTF_TYPE_IS_SDIO(amlbt_if_type) || INTF_TYPE_IS_PCIE(amlbt_if_type))  //sdio interface
    {
        p_device = &amlbt_sdio_device;
        p_driver = &amlbt_sdio_driver;
    }
    platform_driver_unregister(p_driver);
    platform_device_unregister(p_device);
}


module_param(amlbt_if_type, uint, S_IRUGO);
module_init(amlbt_init);
module_exit(amlbt_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("2022-09-06");

