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
#include <linux/ktime.h>
#include <linux/kthread.h>
#include <linux/semaphore.h>
#include <linux/poll.h>
#include <linux/platform_device.h>

#include "amlbt.h"
#include <linux/amlogic/pm.h>

//#define BT_USB_DBG
#define BT_USB_MAX_PRIO                        0xFFFFFFFF

static struct early_suspend amlbt_early_suspend;
static int amlbt_sdio_major;
static struct cdev amlbt_sdio_cdev;
static int amlbt_sdio_devs = 1;
static struct class *amlbt_sdio_class;
static struct device *amlbt_sdio_dev;
static unsigned int rf_num = -1;

#ifdef BT_USB_DBG
static unsigned int dbg_handle = 0;
static unsigned int dbg_credit = 8;
static unsigned int dbg_cnt = 0;
#endif

#define AML_BT_VERSION  (0x20240204)

#define BT_EP           (USB_EP2)

#define CONFIG_BLUEDROID        1 /* bluez 0, bluedroid 1 */
#define INDEPENDENT_USB         0

#define AML_BT_ROM_CHECK        0
#define REG_DEV_RESET           0xf03058
#define REG_PMU_POWER_CFG       0xf03040
#define REG_RAM_PD_SHUTDWONW_SW 0xf03050
#define REG_FW_MODE             0x00f000e0

#define BIT_PHY                 1
#define BIT_MAC                 (1 << 1)
#define BIT_CPU                 (1 << 2)
#define DEV_RESET_SW            16
#define DEV_RESET_HW            0
#define BIT_RF_NUM              28
#define AML_ADDR_AON            1
#define AML_ADDR_MAC            2
#define RESUME_RTC_S            3
#define RESUME_RTC_C            7

#define FAMILY_TYPE_IS_W1(x)        ((AMLBT_PD_ID_FAMILY & x) == AMLBT_FAMILY_W1)
#define FAMILY_TYPE_IS_W1U(x)       ((AMLBT_PD_ID_FAMILY & x) == AMLBT_FAMILY_W1U)
#define FAMILY_TYPE_IS_W2(x)        ((AMLBT_PD_ID_FAMILY & x) == AMLBT_FAMILY_W2)
#define FAMILY_TYPE_IS_W2L(x)       ((AMLBT_PD_ID_FAMILY & x) == AMLBT_FAMILY_W2L)

#define INTF_TYPE_IS_SDIO(x)        ((AMLBT_PD_ID_INTF & x) == AMLBT_INTF_SDIO)
#define INTF_TYPE_IS_USB(x)         ((AMLBT_PD_ID_INTF & x) == AMLBT_INTF_USB)
#define INTF_TYPE_IS_PCIE(x)        ((AMLBT_PD_ID_INTF & x) == AMLBT_INTF_PCIE)

#define TYPE_RETURN_STR(type) \
    case type:                \
    return #type;

static unsigned char *BT_fwICCM = 0;
static unsigned char *BT_fwDCCM = 0;

static unsigned int bt_iccm_size = 0;
static unsigned int bt_dccm_size = 0;
static unsigned int bt_iccm_rom_size = 0;

static unsigned long hci_cmd_queue_addr = 0;
static unsigned int hci_cmd_queue_size = 0;
static unsigned int hci_cmd_rd_ptr = 0;
static unsigned int hci_cmd_wr_ptr = 0;

static unsigned long hci_rx_type_fifo_addr = 0;
static unsigned int hci_rx_type_fifo_size = 0;
static unsigned int hci_rx_type_rd_ptr = 0;
static unsigned int hci_rx_type_wr_ptr = 0;

static unsigned int hci_rx_index_fifo_rd_ptr = 0;
static unsigned int hci_rx_index_fifo_wr_ptr = 0;

static unsigned long hci_evt_fifo_addr = 0;
static unsigned int hci_evt_fifo_rd_ptr = 0;
static unsigned int hci_evt_fifo_wr_ptr = 0;

static unsigned long hci_data_tx_queue_addr = 0;
static unsigned long hci_data_tx_prio_addr = 0;

static unsigned long hci_data_rx_queue_addr = 0;
static unsigned long hci_data_rx_index_fifo_addr = 0;

static unsigned int usb_driver_fw_status_ptr = 0;

static unsigned long _15p4_rx_queue_addr = 0;
static unsigned long _15p4_tx_queue_addr = 0;
static unsigned int _15p4_rx_queue_size = 0;
static unsigned int _15p4_tx_queue_size = 0;

static unsigned int _15p4_rx_wr_ptr = 0;
static unsigned int _15p4_rx_rd_ptr = 0;
static unsigned int _15p4_tx_wr_ptr = 0;
static unsigned int _15p4_tx_rd_ptr = 0;

const char *amlbt_family_intf(int type)
{
    switch (type)
    {
            TYPE_RETURN_STR(AMLBT_FAMILY_W1)
            TYPE_RETURN_STR(AMLBT_FAMILY_W1U)
            TYPE_RETURN_STR(AMLBT_FAMILY_W2)
            TYPE_RETURN_STR(AMLBT_FAMILY_W2L)
            TYPE_RETURN_STR(AMLBT_INTF_SDIO)
            TYPE_RETURN_STR(AMLBT_INTF_USB)
            TYPE_RETURN_STR(AMLBT_INTF_PCIE)
        default:
            break;
    }

    return "unknown type";
}

static unsigned int amlbt_if_type = AMLBT_TRANS_UNKNOWN;
//static unsigned char *usb_buf = NULL;
static unsigned char *sdio_buf = NULL;
static unsigned char *bt_usb_data_buf = NULL;
static int evt_state = 0;
static unsigned char *type = NULL;
static unsigned char *_15p4_buf = NULL;
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
extern struct aml_bus_state_detect bus_state_detect;

extern struct usb_device *g_udev;
extern int auc_send_cmd_ep1(unsigned int addr, unsigned int len);
extern uint32_t aml_pci_read_for_bt(int base, u32 offset);
extern void aml_pci_write_for_bt(u32 val, int base, u32 offset);
extern unsigned int get_resume_method(void);
extern int bt_wt_ptr;
extern int bt_rd_ptr;
static unsigned long bt_wt_ptr_local;

int amlbt_usb_check_fw_rx(void *data);
DECLARE_WAIT_QUEUE_HEAD(poll_amlbt_queue);
struct completion usb_completion;
//struct completion data_completion;

static unsigned int fw_cmd_w = 0;
static unsigned int fw_cmd_r = 0;

#define BTUSB_IOC_MAGIC 'x'

#define IOCTL_GET_BT_DOWNLOAD_STATUS    _IOR(BTUSB_IOC_MAGIC, 0, int)
#define IOCTL_SET_BT_RESET              _IOW(BTUSB_IOC_MAGIC, 1, int)
#define IOCTL_SET_WIFI_COEX_TIME        _IOW(BTUSB_IOC_MAGIC, 10, int)
#define IOCTL_SET_BT_COEX_TIME          _IOW(BTUSB_IOC_MAGIC, 11, int)
#define IOCTL_SET_WIFI_MAX_DURATION     _IOW(BTUSB_IOC_MAGIC, 12, int)
#define IOCTL_GET_WIFI_COEX_TIME        _IOR(BTUSB_IOC_MAGIC, 13, int)
#define IOCTL_GET_BT_COEX_TIME          _IOR(BTUSB_IOC_MAGIC, 14, int)
#define IOCTL_SET_BT_TX_POWER           _IOW(BTUSB_IOC_MAGIC, 15, int)
#define IOCTL_GET_RF_ANTENNA            _IOR(BTUSB_IOC_MAGIC, 16, int)
#define IOCTL_GET_BT_TX_POWER           _IOR(BTUSB_IOC_MAGIC, 17, int)
#define IOCTL_GET_BT_RSSI               _IOR(BTUSB_IOC_MAGIC, 18, int)
#define IOCTL_GET_BT_TIME_PERCENT       _IOR(BTUSB_IOC_MAGIC, 19, int)
#define IOCTL_GET_WF_TIME_PERCENT       _IOR(BTUSB_IOC_MAGIC, 20, int)
#define IOCTL_EXIT_USER                 _IOW(BTUSB_IOC_MAGIC, 21, int)

#define ICCM_RAM_BASE           (0x000000)
#define DCCM_RAM_BASE           (0xd00000)

#define CMD_FIFO_SIZE           128
#define DATA_FIFO_SIZE          16*1024
#define DATA_INDEX_SIZE          128
#define EVT_FIFO_SIZE           8*1024
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
static gdsl_fifo_t *g_15p4_rx_local_fifo = 0;
static gdsl_fifo_t *g_15p4_tx_fifo = 0;
static gdsl_fifo_t *g_15p4_rx_fifo = 0;


static unsigned char *g_lib_cmd_buff = NULL;
static unsigned char *g_fw_data_buf = NULL;
static unsigned char *g_fw_evt_buf  = NULL;
//static unsigned char *g_fw_data_index_buf  = NULL;

static unsigned int gdsl_fifo_remain(gdsl_fifo_t *p_fifo);
static unsigned int suspend_value = 0;
static unsigned int shutdown_value = 0;
static unsigned int bt_recovery = 0;
static unsigned int fwdead_value = 0;

static struct completion download_completion;
static unsigned char sdio_cmd_buff[128] = {0};

//DECLARE_WAIT_QUEUE_HEAD(poll_amlbt_queue);
//DEFINE_MUTEX(usb_rw_lock);
extern struct mutex auc_usb_mutex;

//#define BT_LOCK

#ifdef BT_LOCK

#define USB_BEGIN_LOCK() do {\
    mutex_lock(&auc_usb_mutex);\
} while (0)

#define USB_END_LOCK() do {\
    mutex_unlock(&auc_usb_mutex);\
} while (0)
#else
#define USB_BEGIN_LOCK() do {\
    \
} while (0)

#define USB_END_LOCK() do {\
    \
} while (0)

#endif

enum
{
    LOG_LEVEL_FATAL,
    LOG_LEVEL_ERROR,
    LOG_LEVEL_WARN,
    LOG_LEVEL_INFO,
    LOG_LEVEL_DEBUG,
    LOG_LEVEL_ALL,
};

//static int g_dbg_level = LOG_LEVEL_INFO;
/*
#define BTA(fmt, arg...) if (g_dbg_level >= LOG_LEVEL_ALL) printk(KERN_INFO "BTA:" fmt, ## arg)
#define BTD(fmt, arg...) if (g_dbg_level >= LOG_LEVEL_DEBUG) printk(KERN_INFO "BTD:" fmt, ## arg)
#define BTI(fmt, arg...) if (g_dbg_level >= LOG_LEVEL_INFO) printk(KERN_INFO "BTI:" fmt, ## arg)
#define BTW(fmt, arg...) if (g_dbg_level >= LOG_LEVEL_WARN) printk(KERN_INFO "BTW:" fmt, ## arg)
#define BTE(fmt, arg...) if (g_dbg_level >= LOG_LEVEL_ERROR) printk(KERN_INFO "BTE:" fmt, ## arg)
#define BTF(fmt, arg...) if (g_dbg_level >= LOG_LEVEL_FATAL) printk(KERN_INFO "BTF:" fmt, ## arg)
*/
#define BTA(fmt, arg...)
#define BTD(fmt, arg...)
#define BTI(fmt, arg...)
#define BTW(fmt, arg...)
#define BTE(fmt, arg...)
#define BTF(fmt, arg...)

/*-----------------------------------------------W1U--------------------------------------------
Tx Queue,     start:0x00938000, end:0x0093a01c, length:8224 bytes

Rx Queue,     start:0x0093a020, end:0x0093b01c, length:4096 bytes

Event FIFO,   start:0x0093b020, end:0x0093c01c, length:4096 bytes
    Rx Type FIFO Read Pointer     : 0x0093b020  [0,1,2,3]
    HCI Event FIFO Read Pointer   : 0x0093b024  [4,5,6,7]
    Rx Index FIFO Read Pointer    : 0x0093b028  [8,9,10,11]
    dummy                         : 0x0093b02c  [12,13,14,15]
    dummy                         : 0x0093b030  [16,17,18,19]
    dummy                         : 0x0093b034  [20,21,22,23]
    dummy                         : 0x0093b038  [24,25,26,27]
    Rx Index FIFO Write Pointer   : 0x0093b03c  [28,29,30,31]
    Rx Type FIFO Write Pointer    : 0x0093b040  [32,33,34,35]
    HCI Event FIFO Write Pointer  : 0x0093b044  [36,37,38,39]
    Rx Index FIFO                 : 0x0093b048  [20 bytes]
    Rx Type FIFO                  : 0x0093b04c  [256 bytes]
    HCI Event FIFO                : 0x0093b15c  [1024 bytes]

Command FIFO, start:0x0093c020, end:0x0093c81c, length:2048 bytes

Register RAM, start:0x0093c820, length:? bytes
    HCI Command FIFO Read Pointer : 0x0093c820
    HCI Command FIFO Write Pointer: 0x0093c824

    Tx Queue Prio Pointer         : 0x0093c834
    Tx Queue Acl Handle Pointer   : 0x0093c838
    Tx Queue Status Pointer       : 0x0093c83c
    Dummy                         : 0x0093c840

    Driver Firmware Status Pointer: 0x0093d7fc
-----------------------------------------------W2--------------------------------------------
Event FIFO,   start:0x00500000, end:0x005011fc, length:4604 bytes
    Rx Type FIFO Read Pointer     : 0x00500000  [0,1,2,3]
    HCI Event FIFO Read Pointer   : 0x00500004  [4,5,6,7]
    Rx Index FIFO Read Pointer    : 0x00500008  [8,9,10,11]
    dummy                         : 0x0050000c  [12,13,14,15]
    dummy                         : 0x00500010  [16,17,18,19]
    dummy                         : 0x00500014  [20,21,22,23]
    dummy                         : 0x00500018  [24,25,26,27]
    Rx Index FIFO Write Pointer   : 0x0050001c  [28,29,30,31]
    Rx Type FIFO Write Pointer    : 0x00500020  [32,33,34,35]
    HCI Event FIFO Write Pointer  : 0x00500024  [36,37,38,39]
    Rx Index FIFO                 : 0x00500028 [20 bytes]
    Rx Type FIFO                  : 0x0050003c [256 bytes]
    HCI Event FIFO                : 0x0050013c [1024 bytes]

Tx Queue,     start:0x00508000, end:0x0050a3fc, length:9212 bytes

Register RAM, start:0x00510000, end:0x00510200, length:512 bytes
    HCI Command FIFO Read Pointer : 0x00510000
    HCI Command FIFO Write Pointer: 0x00510004

    Tx Queue Prio Pointer         : 0x00510018
    Tx Queue Acl Handle Pointer   : 0x0051001c
    Tx Queue Status Pointer       : 0x00510020
    Dummy                         : 0x00510024

    Driver Firmware Status Pointer: 0x005101fc

Rx Queue,     start:0x00514000, end:0x00515000, length:4096 bytes

Command FIFO, start:0x00518000, end:0x00519000, length:4096 bytes
-----------------------------------------------W2L--------------------------------------------
Event FIFO,   start:0x00500000, end:0x005011fc, length:4604 bytes
    Rx Type FIFO Read Pointer     : 0x00500000  [0,1,2,3]
    HCI Event FIFO Read Pointer   : 0x00500004  [4,5,6,7]
    Rx Index FIFO Read Pointer    : 0x00500008  [8,9,10,11]
    dummy                         : 0x0050000c  [12,13,14,15]
    dummy                         : 0x00500010  [16,17,18,19]
    dummy                         : 0x00500014  [20,21,22,23]
    dummy                         : 0x00500018  [24,25,26,27]
    Rx Index FIFO Write Pointer   : 0x0050001c  [28,29,30,31]
    Rx Type FIFO Write Pointer    : 0x00500020  [32,33,34,35]
    HCI Event FIFO Write Pointer  : 0x00500024  [36,37,38,39]
    Rx Index FIFO                 : 0x00500028  [20 bytes]
    Rx Type FIFO                  : 0x0050003c  [256 bytes]
    HCI Event FIFO                : 0x0050013c  [1024 bytes]

    15.4 Rx Queue WPoint          : 0x0050053c  [0x53c,0x53d,0x53e,0x53f]
    15.4 Rx Queue RPoint          : 0x00500540  [0x540,0x541,0x542,0x543]
    15.4 Tx Queue WPoint          : 0x00500544  [0x544,0x545,0x546,0x547]
    15.4 Tx Queue RPoint          : 0x00500548  [0x548,0x549,0x54a,0x54b]

Tx Queue,     start:0x00508000, end:0x0050a3fc, length:9212 bytes

Register RAM, start:0x00510000, end:0x00510200, length:512 bytes
    HCI Command FIFO Read Pointer : 0x00510000
    HCI Command FIFO Write Pointer: 0x00510004

    Tx Queue Prio Pointer         : 0x00510018
    Tx Queue Acl Handle Pointer   : 0x0051001c
    Tx Queue Status Pointer       : 0x00510020
    Dummy                         : 0x00510024

    Driver Firmware Status Pointer: 0x005101fc

Rx Queue,     start:0x00514000, end:0x00515000, length:4096 bytes

Command FIFO, start:0x00518000, end:0x00519000, length:4096 bytes
---------------------------------------------------------------------------------------------*/
static void amlbt_usb_ram_init(void)
{
    if (FAMILY_TYPE_IS_W1U(amlbt_if_type))
    {
        bt_iccm_size = 0x10000;
        bt_dccm_size = 0x10000;
        bt_iccm_rom_size = 256*1024;
        hci_cmd_queue_addr = 0x0093c000;
        hci_cmd_queue_size = 2048;
        hci_cmd_rd_ptr = 0x0093c800;
        hci_cmd_wr_ptr = 0x0093c804;

        hci_data_tx_queue_addr = 0x00938000;
        hci_data_tx_prio_addr = 0x0093c814;

        usb_driver_fw_status_ptr = 0x0093d7fc;
    }
    else if (FAMILY_TYPE_IS_W2(amlbt_if_type))
    {
        bt_iccm_size = 0x40000;
        bt_dccm_size = 0x20000;
        bt_iccm_rom_size = 256*1024;

        hci_cmd_queue_addr = 0x00518000;
        hci_cmd_queue_size = 4096;
        hci_cmd_rd_ptr = 0x00510000;
        hci_cmd_wr_ptr = 0x00510004;

        hci_rx_type_fifo_addr = 0x0050003c;
        hci_rx_type_fifo_size = 256;
        hci_rx_type_rd_ptr = 0x00500000;
        hci_rx_type_wr_ptr = 0x00500020;

        hci_rx_index_fifo_rd_ptr = 0x00500008;
        hci_rx_index_fifo_wr_ptr = 0x0050001c;

        hci_evt_fifo_addr = 0x0050013c;
        hci_evt_fifo_rd_ptr = 0x00500004;
        hci_evt_fifo_wr_ptr = 0x00500024;

        hci_data_tx_queue_addr = 0x00508000;
        hci_data_tx_prio_addr = 0x00510018;

        hci_data_rx_queue_addr = 0x00514000;
        hci_data_rx_index_fifo_addr = 0x00500028;

        usb_driver_fw_status_ptr = 0x005101fc;
    }
    else if (FAMILY_TYPE_IS_W2L(amlbt_if_type))
    {
        bt_iccm_size = 0x38000;
        bt_dccm_size = 0x20000;
        bt_iccm_rom_size = 384*1024;

        hci_cmd_queue_addr = 0x00518000;
        hci_cmd_queue_size = 4096;
        hci_cmd_rd_ptr = 0x00510000;
        hci_cmd_wr_ptr = 0x00510004;

        hci_rx_type_fifo_addr = 0x0050003c;
        hci_rx_type_fifo_size = 256;
        hci_rx_type_rd_ptr = 0x00500000;
        hci_rx_type_wr_ptr = 0x00500020;

        hci_rx_index_fifo_rd_ptr = 0x00500008;
        hci_rx_index_fifo_wr_ptr = 0x0050001c;

        hci_evt_fifo_addr = 0x0050013c;
        hci_evt_fifo_rd_ptr = 0x00500004;
        hci_evt_fifo_wr_ptr = 0x00500024;

        hci_data_tx_queue_addr = 0x00508000;
        hci_data_tx_prio_addr = 0x00510018;

        hci_data_rx_queue_addr = 0x00514000;
        hci_data_rx_index_fifo_addr = 0x00500028;

        usb_driver_fw_status_ptr = 0x005101fc;

        _15p4_rx_queue_addr = 0x00702000;
        _15p4_tx_queue_addr = 0x00703000;
        _15p4_rx_queue_size = 0x800;
        _15p4_tx_queue_size = 0x800;

        _15p4_rx_wr_ptr = 0x0050053c;
        _15p4_rx_rd_ptr = 0x00500540;
        _15p4_tx_wr_ptr = 0x00500544;
        _15p4_tx_rd_ptr = 0x00500548;
    }
}

static void amlbt_usb_write_word(unsigned int addr,unsigned int data, unsigned int ep)
{
    if (g_auc_hif_ops.hi_write_word_for_bt == NULL)
    {
        printk("amlbt_usb_write_word NULL");
        return ;
    }
    while (bus_state_detect.bus_err || bus_state_detect.bus_reset_ongoing)
    {
        printk("WW");
        while (g_auc_hif_ops.hi_read_word(0xa10004, USB_EP4) != 0x1b8e)
        {
            usleep_range(20000, 20000);
        }
        bt_recovery = 1;
        printk("WWE");
    }
    USB_BEGIN_LOCK();
    g_auc_hif_ops.hi_write_word_for_bt(addr, data, ep);
    USB_END_LOCK();
}
/*
static void amlbt_usb_write_word_ext(unsigned int addr,unsigned int data, unsigned int ep)
{
    while (bus_state_detect.bus_err || bus_state_detect.bus_reset_ongoing)
    {
        printk("WW");
        while (g_auc_hif_ops.hi_read_word(0xa10004, USB_EP4) != 0x1b8e)
        {
            usleep_range(20000, 20000);
        }
        bt_recovery = 1;
        printk("WWE");
    }
    g_auc_hif_ops.hi_write_word_for_bt(addr, data, ep);
}
*/
static unsigned int amlbt_usb_read_word(unsigned int addr, unsigned int ep)
{
    unsigned int value = 0;
    if (g_auc_hif_ops.hi_read_word_for_bt == NULL)
    {
        printk("amlbt_usb_read_word NULL");
        return 0;
    }
    while (bus_state_detect.bus_err || bus_state_detect.bus_reset_ongoing)
    {
        printk("RW");
        while (g_auc_hif_ops.hi_read_word(0xa10004, USB_EP4) != 0x1b8e)
        {
            usleep_range(20000, 20000);
        }
        bt_recovery = 1;
        printk("RWE");
    }
    USB_BEGIN_LOCK();
    value = g_auc_hif_ops.hi_read_word_for_bt(addr, ep);
    USB_END_LOCK();
    return value;
}
static void amlbt_usb_write_sram(unsigned char* buf, unsigned char* addr, unsigned int len, unsigned int ep)
{
    if (g_auc_hif_ops.hi_write_sram_for_bt == NULL)
    {
        printk("amlbt_usb_write_sram NULL");
        return;
    }
    while (bus_state_detect.bus_err || bus_state_detect.bus_reset_ongoing)
    {
        printk("WS");
        while (g_auc_hif_ops.hi_read_word(0xa10004, USB_EP4) != 0x1b8e)
        {
            usleep_range(20000, 20000);
        }
        bt_recovery = 1;
        printk("WSE");
    }
    USB_BEGIN_LOCK();
    g_auc_hif_ops.hi_write_sram_for_bt(buf, addr, len, ep);
    USB_END_LOCK();
}
#ifdef BT_USB_DBG
static void amlbt_usb_read_sram(unsigned char* buf, unsigned char* addr, unsigned int len, unsigned int ep)
{
    if (g_auc_hif_ops.hi_read_sram_for_bt == NULL)
    {
        printk("hi_read_sram_for_bt NULL");
        return;
    }
    while (bus_state_detect.bus_err || bus_state_detect.bus_reset_ongoing)
    {
        printk("RS");
        while (g_auc_hif_ops.hi_read_word(0xa10004, USB_EP4) != 0x1b8e)
        {
            usleep_range(20000, 20000);
        }
        bt_recovery = 1;
        printk("RSE");
    }
    USB_BEGIN_LOCK();
    g_auc_hif_ops.hi_read_sram_for_bt(buf, addr, len, ep);
    USB_END_LOCK();
}
#endif

static void amlbt_usb_read_sram_ext(unsigned char* buf, unsigned char* addr, unsigned int len, unsigned int ep)
{
    if (g_auc_hif_ops.hi_read_sram_for_bt == NULL)
    {
        printk("hi_read_sram_for_bt NULL");
        return;
    }
    while (bus_state_detect.bus_err || bus_state_detect.bus_reset_ongoing)
    {
        printk("RS");
        while (g_auc_hif_ops.hi_read_word(0xa10004, USB_EP4) != 0x1b8e)
        {
            usleep_range(20000, 20000);
        }
        bt_recovery = 1;
        printk("RSE");
    }
    g_auc_hif_ops.hi_read_sram_for_bt(buf, addr, len, ep);
}

static void amlbt_usb_write_sram_ext(unsigned char* buf, unsigned char* addr, unsigned int len, unsigned int ep)
{
    if (g_auc_hif_ops.hi_write_sram_for_bt == NULL)
    {
        printk("hi_write_sram_for_bt NULL");
        return;
    }
    while (bus_state_detect.bus_err || bus_state_detect.bus_reset_ongoing)
    {
        printk("WS");
        while (g_auc_hif_ops.hi_read_word(0xa10004, USB_EP4) != 0x1b8e)
        {
            usleep_range(20000, 20000);
        }
        bt_recovery = 1;
        printk("WSE");
    }
    g_auc_hif_ops.hi_write_sram_for_bt(buf, addr, len, ep);
}


#ifdef BT_USB_DBG

static void rssi_dbg(void)
{
    unsigned int tx_q_prio[USB_TX_Q_NUM] = {0};
    unsigned int tx_q_index[USB_TX_Q_NUM] = {0};
    unsigned int tx_q_status[USB_TX_Q_NUM] = {0};
    unsigned int tx_buff[USB_TX_Q_NUM * 4] = {0};//prio, index, status
    unsigned int i = 0;
    unsigned long w_point = 0;

    amlbt_usb_read_sram((unsigned char *)tx_buff, (unsigned char *)g_tx_q[0].tx_q_prio_addr,
                                          sizeof(tx_buff), BT_EP);

    for (i = 0; i < USB_TX_Q_NUM; i++)
    {
        tx_q_prio[i] = tx_buff[i*4];
        tx_q_index[i]  = tx_buff[i*4+1];
        tx_q_status[i]   = tx_buff[i*4+2];
    }
    BTA("P %#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x\n", tx_q_prio[0],tx_q_prio[1],tx_q_prio[2],tx_q_prio[3],
        tx_q_prio[4],tx_q_prio[5],tx_q_prio[6],tx_q_prio[7]);
    BTA("A %#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x\n", tx_q_index[0],tx_q_index[1],tx_q_index[2],tx_q_index[3],
        tx_q_index[4],tx_q_index[5],tx_q_index[6],tx_q_index[7]);
    printk("S %#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x\n", tx_q_status[0],tx_q_status[1],tx_q_status[2],tx_q_status[3],
        tx_q_status[4],tx_q_status[5],tx_q_status[6],tx_q_status[7]);

    amlbt_usb_read_sram(&bt_usb_data_buf[0], (unsigned char *)hci_rx_type_rd_ptr, USB_POLL_TOTAL_LEN, BT_EP);
    //1. process type
    w_point = ((bt_usb_data_buf[35]<<24)|(bt_usb_data_buf[34]<<16)|(bt_usb_data_buf[33]<<8)|bt_usb_data_buf[32]);
    g_rx_type_fifo->w = (unsigned char *)w_point;
    BTA("T %#x,%#x", (unsigned long)g_rx_type_fifo->w, (unsigned long)g_rx_type_fifo->r);
    BTA("LT %#x,%#x", (unsigned long)g_fw_type_fifo->w, (unsigned long)g_fw_type_fifo->r);

    //2. process event
    g_event_fifo->w = (unsigned char *)(unsigned long)((bt_usb_data_buf[39]<<24)|(bt_usb_data_buf[38]<<16)|(bt_usb_data_buf[37]<<8)|bt_usb_data_buf[36]);
    BTA("E %#x,%#x", (unsigned long)g_event_fifo->w, (unsigned long)g_event_fifo->r);
    BTA("LE %#x,%#x", (unsigned long)g_fw_evt_fifo->w, (unsigned long)g_fw_evt_fifo->r);
}
#endif

static void aml_15p4_buff_init(void)
{
    if (_15p4_buf == NULL)
    {
        _15p4_buf = kzalloc(_15p4_rx_queue_size, GFP_DMA|GFP_ATOMIC);
        printk("aml_15p4_buff_init:size %#x, b %#lx,\n", _15p4_rx_queue_size, (unsigned long)_15p4_buf);
        if (_15p4_buf == NULL)
        {
            printk("%s p_acl_buf kzalloc falied! \n", __func__);
            return ;
        }
    }
}

void amlbt_buff_init(void)
{
    if (type == NULL)
    {
        type = kzalloc(TYPE_FIFO_SIZE, GFP_DMA|GFP_ATOMIC);
        if (type == NULL)
        {
            printk("%s type kzalloc falied! \n", __func__);
            return ;
        }
    }
    if (p_acl_buf == NULL)
    {
        p_acl_buf = kzalloc(HCI_MAX_FRAME_SIZE, GFP_DMA|GFP_ATOMIC);
        if (p_acl_buf == NULL)
        {
            printk("%s p_acl_buf kzalloc falied! \n", __func__);
            kfree(type);
            type = NULL;
            return ;
        }
    }
    if (g_fw_data_buf == NULL)
    {
        g_fw_data_buf = kzalloc(DATA_FIFO_SIZE, GFP_DMA|GFP_ATOMIC);
        if (g_fw_data_buf == NULL)
        {
            printk("%s g_fw_data_buf kzalloc falied! \n", __func__);
            kfree(type);
            kfree(p_acl_buf);
            type = NULL;
            p_acl_buf = NULL;
            return ;
        }
    }
    if (g_fw_evt_buf == NULL)
    {
        g_fw_evt_buf = kzalloc(EVT_FIFO_SIZE, GFP_DMA|GFP_ATOMIC);
        if (g_fw_evt_buf == NULL)
        {
            printk("%s g_fw_evt_buf kzalloc falied! \n", __func__);
            kfree(type);
            kfree(p_acl_buf);
            kfree(g_fw_data_buf);
            type = NULL;
            p_acl_buf = NULL;
            g_fw_data_buf = NULL;
            return ;
        }
    }
    if (g_lib_cmd_buff == NULL)
    {
        g_lib_cmd_buff = kzalloc(CMD_FIFO_SIZE, GFP_DMA|GFP_ATOMIC);
        if (g_lib_cmd_buff == NULL)
        {
            printk("%s g_lib_cmd_buff kzalloc falied! \n", __func__);
            kfree(type);
            kfree(p_acl_buf);
            kfree(g_fw_data_buf);
            kfree(g_fw_evt_buf);
            type = NULL;
            p_acl_buf = NULL;
            g_fw_data_buf = NULL;
            g_fw_evt_buf = NULL;
            return ;
        }
    }
    if (bt_usb_data_buf == NULL)
    {
        if (FAMILY_TYPE_IS_W2L(amlbt_if_type))
        {
            bt_usb_data_buf = kzalloc(USB_POLL_TOTAL_LEN + 16, GFP_DMA|GFP_ATOMIC);
        }
        else
        {
            bt_usb_data_buf = kzalloc(USB_POLL_TOTAL_LEN, GFP_DMA|GFP_ATOMIC);
        }

        if (bt_usb_data_buf == NULL)
        {
            printk("%s bt_usb_data_buf kzalloc falied! \n", __func__);
            kfree(type);
            kfree(p_acl_buf);
            kfree(g_fw_data_buf);
            kfree(g_fw_evt_buf);
            kfree(g_lib_cmd_buff);
            type = NULL;
            p_acl_buf = NULL;
            g_fw_data_buf = NULL;
            g_fw_evt_buf = NULL;
            g_lib_cmd_buff = NULL;
            return ;
        }
    }
}

static void aml_15p4_buff_deinit(void)
{
    kfree(_15p4_buf);
    _15p4_buf = NULL;
}

void amlbt_buff_deinit(void)
{
    kfree(type);
    kfree(p_acl_buf);
    kfree(g_fw_data_buf);
    kfree(g_fw_evt_buf);
    kfree(g_lib_cmd_buff);
    kfree(bt_usb_data_buf);
    bt_usb_data_buf = NULL;
    type            = NULL;
    p_acl_buf       = NULL;
    g_fw_data_buf   = NULL;
    g_fw_evt_buf    = NULL;
    g_lib_cmd_buff  = NULL;
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
        printk("gdsl_fifo_copy_data no space!!\n");
        printk("fifo->base_addr %#x, fifo->size %#x\n", (unsigned long)p_fifo->base_addr, p_fifo->size);
        printk("fifo->w %#x, fifo->r %#x\n", (unsigned long)p_fifo->w, (unsigned long)p_fifo->r);
        printk("remain %#x, len %#x\n", gdsl_fifo_remain(p_fifo), len);
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

unsigned int gdsl_fifo_calc_r(gdsl_fifo_t *p_fifo, unsigned char *buff, unsigned int len)
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
        p_fifo->r = (unsigned char *)(((unsigned long)p_fifo->r + 1) % p_fifo->size);
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
        printk("write data no space!!\n");
        return 0;
    }

    if (w == 0)
    {
        BTA("w ep %#x\n", (unsigned long)p_fifo->base_addr);
        amlbt_usb_write_sram(data, p_fifo->base_addr, len, ep);
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
            amlbt_usb_write_sram(data,
                (unsigned char *)((unsigned long)p_fifo->w + (unsigned long)p_fifo->base_addr), i, ep);
            p_fifo->w = 0;
            index = i;
        }
    }
    if (index < len)
    {
        BTA("w ep3 %#x\n", (unsigned long)p_fifo->w);
        amlbt_usb_write_sram(&data[index],
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
        amlbt_usb_read_sram(data, p_fifo->base_addr, read_len, ep);
        p_fifo->r = (unsigned char *)(unsigned long)(read_len % p_fifo->size);
        return read_len;
    }

    while (i < read_len)
    {
        r = (unsigned char *)(((unsigned long)r + 1) % p_fifo->size);
        i++;
        if (r == 0)
        {
            amlbt_usb_read_sram(data,
                (unsigned char *)((unsigned long)p_fifo->r + (unsigned long)p_fifo->base_addr), i, ep);
            p_fifo->r = 0;
            index = i;
        }
    }
    if (index < read_len)
    {
        amlbt_usb_read_sram(&data[index],
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

    for (i = 0; i < USB_TX_Q_NUM; i++)
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
    unsigned int index = USB_TX_Q_NUM;

    for (i = 0; i < USB_TX_Q_NUM; i++)
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
        g_rx_type_fifo = gdsl_fifo_init(USB_RX_TYPE_FIFO_LEN, (unsigned char *)hci_rx_type_fifo_addr);

        if (FAMILY_TYPE_IS_W2(amlbt_if_type) || FAMILY_TYPE_IS_W2L(amlbt_if_type))
        {
            //update read pointer
            amlbt_usb_write_word(hci_rx_type_rd_ptr, (unsigned int)(unsigned long)g_rx_type_fifo->r,
                                               BT_EP);
            //update write pointer
            amlbt_usb_write_word(hci_rx_type_wr_ptr, (unsigned int)(unsigned long)g_rx_type_fifo->w,
                                               BT_EP);
        }
        //g_rx_type_fifo->r = (unsigned char *)(unsigned long)(hci_rx_type_fifo_addr);
    }
}

void amlbt_usb_rx_type_fifo_deinit(void)
{
    if (g_rx_type_fifo)
    {
        if (FAMILY_TYPE_IS_W2(amlbt_if_type) || FAMILY_TYPE_IS_W2L(amlbt_if_type))
        {
            amlbt_usb_write_word(hci_rx_type_rd_ptr, 0, BT_EP);
            amlbt_usb_write_word(hci_rx_type_wr_ptr, 0, BT_EP);
        }
        gdsl_fifo_deinit(g_rx_type_fifo);
        g_rx_type_fifo = 0;
    }
}

void amlbt_usb_hci_cmd_fifo_init(void)
{
    if (g_cmd_fifo == 0)
    {
        g_cmd_fifo = gdsl_fifo_init(hci_cmd_queue_size, (unsigned char *)(hci_cmd_queue_addr));
        amlbt_usb_write_word(hci_cmd_rd_ptr, (unsigned int)(unsigned long)g_cmd_fifo->r, BT_EP);
        BTA("cmd fifo init r: %#lx\n", (unsigned long)g_cmd_fifo->r);
        //update write pointer
        amlbt_usb_write_word(hci_cmd_wr_ptr, (unsigned int)(unsigned long)g_cmd_fifo->w, BT_EP);
        BTA("cmd fifo init w : %#lx\n", (unsigned long)g_cmd_fifo->w);
    }
}

void amlbt_usb_15p4_tx_fifo_init(void)
{
    if (g_15p4_tx_fifo == 0)
    {
        g_15p4_tx_fifo = gdsl_fifo_init(_15p4_tx_queue_size, (unsigned char *)(_15p4_tx_queue_addr));
        amlbt_usb_write_word(_15p4_tx_rd_ptr, (unsigned int)(unsigned long)g_15p4_tx_fifo->r, BT_EP);
        printk("15.4 tx fifo init r: %#lx\n", (unsigned long)g_15p4_tx_fifo->r);
        //update write pointer
        amlbt_usb_write_word(_15p4_tx_wr_ptr, (unsigned int)(unsigned long)g_15p4_tx_fifo->w, BT_EP);
        printk("15.4 tx fifo init w : %#lx\n", (unsigned long)g_15p4_tx_fifo->w);
    }
}

void amlbt_usb_15p4_rx_fifo_init(void)
{
    if (g_15p4_rx_fifo == 0)
    {
        g_15p4_rx_fifo = gdsl_fifo_init(_15p4_rx_queue_size, (unsigned char *)(_15p4_rx_queue_addr));
        amlbt_usb_write_word(_15p4_rx_rd_ptr, (unsigned int)(unsigned long)g_15p4_rx_fifo->r, BT_EP);
        printk("15.4 rx fifo init r: %#lx\n", (unsigned long)g_15p4_rx_fifo->r);
        //update write pointer
        amlbt_usb_write_word(_15p4_rx_wr_ptr, (unsigned int)(unsigned long)g_15p4_rx_fifo->w, BT_EP);
        printk("15.4 rx fifo init w : %#lx\n", (unsigned long)g_15p4_rx_fifo->w);
    }
}

void amlbt_usb_hci_cmd_fifo_deinit(void)
{
    BTD("%s \n", __func__);
    if (g_cmd_fifo != 0)
    {
        amlbt_usb_write_word(hci_cmd_rd_ptr, 0, BT_EP);
        amlbt_usb_write_word(hci_cmd_wr_ptr, 0, BT_EP);
        gdsl_fifo_deinit(g_cmd_fifo);
        g_cmd_fifo = 0;
    }
}

void amlbt_usb_hci_tx_data_init(void)
{
    unsigned int i = 0;
    unsigned int tx_info[USB_TX_Q_NUM * 4] = {0};

    if (g_tx_q == 0)
    {
        g_tx_q = (gdsl_tx_q_t *)kzalloc(sizeof(gdsl_tx_q_t) * USB_TX_Q_NUM, GFP_KERNEL);
    }

    for (i = 0; i < USB_TX_Q_NUM; i++)
    {
        g_tx_q[i].tx_q_addr = (unsigned char *)(unsigned long)(hci_data_tx_queue_addr + i * USB_TX_Q_LEN);

        g_tx_q[i].tx_q_prio_addr = (unsigned int *)(unsigned long)(hci_data_tx_prio_addr + i * 16);
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
        amlbt_usb_write_sram((unsigned char *)tx_info,
                                           (unsigned char *)hci_data_tx_prio_addr, sizeof(tx_info),
                                           BT_EP);
    }
}

void amlbt_usb_hci_tx_data_deinit(void)
{
    unsigned int tx_info[USB_TX_Q_NUM * 4] = {0};

    BTD("%s \n", __func__);

    if (g_tx_q)
    {
        if (FAMILY_TYPE_IS_W2(amlbt_if_type) || FAMILY_TYPE_IS_W2L(amlbt_if_type))
        {
            amlbt_usb_write_sram((unsigned char *)tx_info,
                                               (unsigned char *)hci_data_tx_prio_addr, sizeof(tx_info),
                                               BT_EP);
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
        g_event_fifo = gdsl_fifo_init(USB_EVENT_Q_LEN, (unsigned char *)(hci_evt_fifo_addr));
        if (FAMILY_TYPE_IS_W2(amlbt_if_type) || FAMILY_TYPE_IS_W2L(amlbt_if_type))
        {
            amlbt_usb_write_word(hci_evt_fifo_rd_ptr, (unsigned int)(unsigned long)g_event_fifo->r, BT_EP);
            BTA("event fifo init r: %#lx\n", (unsigned long)g_event_fifo->r);
            amlbt_usb_write_word(hci_evt_fifo_wr_ptr, (unsigned int)(unsigned long)g_event_fifo->w, BT_EP);
        }
        BTA("event fifo init w : %#lx\n", (unsigned long)g_event_fifo->w);
    }
}

void amlbt_usb_hci_evt_fifo_deinit(void)
{
    BTD("%s \n", __func__);
    if (g_event_fifo != 0)
    {
        if (FAMILY_TYPE_IS_W2(amlbt_if_type) || FAMILY_TYPE_IS_W2L(amlbt_if_type))
        {
            amlbt_usb_write_word(hci_evt_fifo_rd_ptr, 0, BT_EP);
            amlbt_usb_write_word(hci_evt_fifo_wr_ptr, 0, BT_EP);
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
        g_rx_fifo = gdsl_fifo_init(USB_RX_INDEX_FIFO_LEN, (unsigned char *)(hci_data_rx_index_fifo_addr));
        if (FAMILY_TYPE_IS_W2(amlbt_if_type) || FAMILY_TYPE_IS_W2L(amlbt_if_type))
        {
            amlbt_usb_write_word(hci_rx_index_fifo_rd_ptr, (unsigned int)(unsigned long)g_rx_fifo->r, BT_EP);
            BTA("recv fifo init r: %#lx\n", (unsigned long)g_rx_fifo->r);
            amlbt_usb_write_word(hci_rx_index_fifo_wr_ptr, (unsigned int)(unsigned long)g_rx_fifo->w, BT_EP);
        }
        BTA("recv fifo init w : %#lx\n", (unsigned long)g_rx_fifo->w);
    }
}

void amlbt_usb_fw_recv_fifo_deinit(void)
{
    BTD("%s \n", __func__);
    if (g_rx_fifo != 0)
    {
        if (FAMILY_TYPE_IS_W2(amlbt_if_type) || FAMILY_TYPE_IS_W2L(amlbt_if_type))
        {
            amlbt_usb_write_word(hci_rx_index_fifo_rd_ptr, 0, BT_EP);
            amlbt_usb_write_word(hci_rx_index_fifo_wr_ptr, 0, BT_EP);
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
        printk("%s: bt_usb_hci_cmd_fifo NULL!!!!\n", __func__);
        return ;
    }

    len = ((len + 3) & 0xFFFFFFFC);//Keep 4 bytes aligned

    BTA("%s, Actual length %d \n", __func__, len);
    //step 1: Update the command FIFO read pointer
    g_cmd_fifo->r = (unsigned char *)(unsigned long)amlbt_usb_read_word(hci_cmd_rd_ptr, BT_EP);
    BTA("cmd r %#x\n", (unsigned long)g_cmd_fifo->r);
    //step 2: Check the command FIFO space

    //step 3: Write HCI commands to WiFi SRAM
    gdsl_write_data_by_ep(g_cmd_fifo, data, len, BT_EP);
    //step 4: Update the write pointer and write to WiFi SRAM

    BTA("before write:r:%#lx, w:%#lx\n", (unsigned long)g_cmd_fifo->r, (unsigned long)g_cmd_fifo->w);
    amlbt_usb_write_word(hci_cmd_wr_ptr, (unsigned long)g_cmd_fifo->w & 0xfff, BT_EP);
    BTA("len %#x:w %#lx, r %#lx\n", len, (unsigned long)g_cmd_fifo->w, (unsigned long)g_cmd_fifo->r);
}

static void amlbt_usb_send_hci_data(unsigned char *data, unsigned int len)
{
    unsigned int i = 0;
    unsigned int acl_handle = (((data[1] << 8) | data[0]) & 0xfff);
    unsigned int prio = 0;
    unsigned int tx_q_prio[USB_TX_Q_NUM] = {0};
    unsigned int tx_q_index[USB_TX_Q_NUM] = {0};
    unsigned int tx_q_status[USB_TX_Q_NUM] = {0};
    unsigned int tx_buff[USB_TX_Q_NUM * 4] = {0};//prio, index, status
    BTA("%s, len:%d\n", __func__, len);

    USB_BEGIN_LOCK();
#ifdef BT_USB_DBG
    if (acl_handle == 0x33)
    {
        dbg_handle = acl_handle;
        dbg_credit--;
        if (len >= 300)
        {
            dbg_cnt++;
        }
        if (dbg_credit <= 5)
        {
            rssi_dbg();
        }
    }

    printk("1:%#x,%#x,%d,%#x,%#x\n", acl_handle,len,dbg_credit,dbg_cnt,amlbt_usb_read_word(0x2f61c8, BT_EP));
#endif
    amlbt_usb_read_sram_ext((unsigned char *)tx_buff, (unsigned char *)g_tx_q[0].tx_q_prio_addr,
                                          sizeof(tx_buff), BT_EP);
    if (bt_recovery)
    {
        USB_END_LOCK();
        return ;
    }

    for (i = 0; i < USB_TX_Q_NUM; i++)
    {
        tx_q_prio[i] = tx_buff[i*4];
        tx_q_index[i]  = tx_buff[i*4+1];
        tx_q_status[i]   = tx_buff[i*4+2];
    }
    BTA("P %#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x\n", tx_q_prio[0],tx_q_prio[1],tx_q_prio[2],tx_q_prio[3],
        tx_q_prio[4],tx_q_prio[5],tx_q_prio[6],tx_q_prio[7]);
    BTA("A %#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x\n", tx_q_index[0],tx_q_index[1],tx_q_index[2],tx_q_index[3],
        tx_q_index[4],tx_q_index[5],tx_q_index[6],tx_q_index[7]);
    BTA("S %#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x\n", tx_q_status[0],tx_q_status[1],tx_q_status[2],tx_q_status[3],
        tx_q_status[4],tx_q_status[5],tx_q_status[6],tx_q_status[7]);
    for (i = 0; i < USB_TX_Q_NUM; i++)
    {
        if (tx_q_status[i] == GDSL_TX_Q_COMPLETE/* && tx_q_prio[i] == 0*/)
        {
            //acl_handle = tx_q_index[i];
            tx_q_index[i] = 0;
            tx_q_status[i] = GDSL_TX_Q_UNUSED;
            tx_q_prio[i] = BT_USB_MAX_PRIO;
        }
    }

    for (i = 0; i < USB_TX_Q_NUM; i++)
    {
        g_tx_q[i].tx_q_dev_index = tx_q_index[i];
        g_tx_q[i].tx_q_status = tx_q_status[i];
        g_tx_q[i].tx_q_prio = tx_q_prio[i];
    }

    for (i = 0; i < USB_TX_Q_NUM; i++)
    {
        if (g_tx_q[i].tx_q_status == GDSL_TX_Q_UNUSED)
        {
            break;
        }
    }

    if (i == USB_TX_Q_NUM)
    {
        printk("%s: hci data space invalid!!!! \n", __func__);
        for (i = 0; i < USB_TX_Q_NUM; i++)
        {
            printk("[%#x,%#x,%#x]", (unsigned int)g_tx_q[i].tx_q_prio,
                    (unsigned int)g_tx_q[i].tx_q_dev_index,
                    (unsigned int)g_tx_q[i].tx_q_status);
            printk("{%#x,%#x,%#x}", tx_q_prio[i], tx_q_index[i],tx_q_status[i]);
        }
        USB_END_LOCK();
        return ;
    }

    prio = amlbt_usb_get_tx_prio(g_tx_q, acl_handle);

    g_tx_q[i].tx_q_prio = (++prio & BT_USB_MAX_PRIO);
    g_tx_q[i].tx_q_dev_index = acl_handle;
    g_tx_q[i].tx_q_status = GDSL_TX_Q_USED;


    len = (len + 3) & ~3;
    amlbt_usb_write_sram_ext(data, g_tx_q[i].tx_q_addr, len, BT_EP);

    tx_buff[0] = g_tx_q[i].tx_q_prio;
    tx_buff[1] = g_tx_q[i].tx_q_dev_index;
    tx_buff[2] = g_tx_q[i].tx_q_status;
    tx_buff[3] = 0;

    BTA("%d,%d,%#x,%#x\n", i, len, acl_handle, g_tx_q[i].tx_q_prio);
    amlbt_usb_write_sram_ext((unsigned char *)&tx_buff[0], (unsigned char *)g_tx_q[i].tx_q_prio_addr,
                                      sizeof(unsigned int)*4, BT_EP);
#ifdef BT_USB_DBG
    printk("6:%d,%#x,%#x\n", i, g_tx_q[i].tx_q_prio, amlbt_usb_read_word(0x2f61c8, BT_EP));
#endif
    BTA("%s, Actual length:%d\n", __func__, len);
    USB_END_LOCK();
}

static void amlbt_usb_send_15p4_data(unsigned char *data, unsigned int len)
{
    printk("%s, len %d \n", __func__, len);

    if (g_15p4_tx_fifo == NULL)
    {
        printk("%s: g_15p4_tx_fifo NULL!!!!\n", __func__);
        return ;
    }

    len = ((len + 3) & 0xFFFFFFFC);//Keep 4 bytes aligned

    BTA("%s, Actual length %d \n", __func__, len);
    //step 1: Update the tx FIFO read pointer
    g_15p4_tx_fifo->r = (unsigned char *)(unsigned long)amlbt_usb_read_word(_15p4_tx_rd_ptr, BT_EP);

    printk("15p4 tx r %#x\n", (unsigned long)g_15p4_tx_fifo->r);
    //step 2: Check the command FIFO space

    //step 3: Write HCI commands to WiFi SRAM
    gdsl_write_data_by_ep(g_15p4_tx_fifo, data, len, BT_EP);

    //step 4: Update the write pointer and write to WiFi SRAM
    printk("15p4 before write:r:%#lx, w:%#lx\n", (unsigned long)g_15p4_tx_fifo->r, (unsigned long)g_15p4_tx_fifo->w);

    amlbt_usb_write_word(_15p4_tx_wr_ptr, (unsigned long)g_15p4_tx_fifo->w & 0x7ff, BT_EP);
    printk("15p4 len %#x:w %#lx, r %#lx\n", len, (unsigned long)g_15p4_tx_fifo->w, (unsigned long)g_15p4_tx_fifo->r);
}


#if 0
static unsigned int amlbt_usb_recv_hci_event(unsigned char *buff, unsigned int cnt)
{
    unsigned int len = 0;
    //unsigned int i = 0;

    //g_event_fifo->w = (unsigned char *)(unsigned long)g_auc_hif_ops.hi_read_word_for_bt(hci_evt_fifo_wr_ptr, BT_EP);
    //g_event_fifo->w += (hci_evt_fifo_addr);

    BTD("%s\n", __func__);
#if AML_FW_POINTER
    printk("evt r:%#lx,w:%#lx\n", (unsigned long)g_event_fifo->r - hci_evt_fifo_addr, (unsigned long)g_event_fifo->w - hci_evt_fifo_addr);
#endif
    if (g_event_fifo->w != g_event_fifo->r)
    {
        len = gdsl_read_data_by_ep(g_event_fifo, buff, cnt, BT_EP);
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

    //g_rx_type_fifo->w = (unsigned char *)(unsigned long)g_auc_hif_ops.hi_read_word_for_bt(hci_rx_type_wr_ptr, BT_EP);
    //g_rx_type_fifo->w += (hci_rx_type_fifo_addr);

    BTD("%s\n", __func__);

    if (g_rx_type_fifo->w != g_rx_type_fifo->r)
    {
        len = gdsl_read_data_by_ep(g_rx_type_fifo, buff, cnt, BT_EP);
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

    //g_rx_fifo->w = (unsigned char *)(unsigned long)g_auc_hif_ops.hi_read_word_for_bt(hci_rx_index_fifo_wr_ptr, BT_EP);
    //g_rx_fifo->w += (hci_data_rx_index_fifo_addr);

    BTD("%s\n", __func__);

    if (g_rx_fifo->w != g_rx_fifo->r)
    {
        len = gdsl_read_data_by_ep(g_rx_fifo, buff, cnt, BT_EP);
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

    BTD("%s\n", __func__);

    if (FAMILY_TYPE_IS_W2(amlbt_if_type) || FAMILY_TYPE_IS_W2L(amlbt_if_type))
    {
        st_reg = amlbt_usb_read_word(usb_driver_fw_status_ptr, BT_EP);
        st_reg |= WF_SRAM_FD_INIT_FLAG;
        amlbt_usb_write_word(usb_driver_fw_status_ptr, st_reg, BT_EP);
    }
    amlbt_buff_init();
    if (FAMILY_TYPE_IS_W2L(amlbt_if_type))
    {
        aml_15p4_buff_init();
    }
    amlbt_usb_rx_type_fifo_init();
    amlbt_usb_hci_cmd_fifo_init();
    amlbt_usb_hci_tx_data_init();
    amlbt_usb_hci_evt_fifo_init();
    amlbt_usb_fw_recv_fifo_init();
    if (FAMILY_TYPE_IS_W2L(amlbt_if_type))
    {
        amlbt_usb_15p4_tx_fifo_init();
        amlbt_usb_15p4_rx_fifo_init();
    }
    memset(type, 0, TYPE_FIFO_SIZE);

    g_fw_data_fifo = gdsl_fifo_init(DATA_FIFO_SIZE, g_fw_data_buf);
    g_fw_evt_fifo = gdsl_fifo_init(EVT_FIFO_SIZE, g_fw_evt_buf);
    g_fw_type_fifo = gdsl_fifo_init(TYPE_FIFO_SIZE, type);
    if (FAMILY_TYPE_IS_W2L(amlbt_if_type))
    {
        g_15p4_rx_local_fifo = gdsl_fifo_init(_15p4_rx_queue_size, _15p4_buf);
        printk("amlbt_usb_fifo_init1:w %#lx, r %#lx, b %#lx,\n", (unsigned long)g_15p4_rx_local_fifo->w,
                    (unsigned long)g_15p4_rx_local_fifo->r, (unsigned long)_15p4_buf);
    }
    st_reg &= ~(WF_SRAM_FD_INIT_FLAG);
    if (FAMILY_TYPE_IS_W2(amlbt_if_type) || FAMILY_TYPE_IS_W2L(amlbt_if_type))
    {
        amlbt_usb_write_word(usb_driver_fw_status_ptr, st_reg, BT_EP);
    }

    if (FAMILY_TYPE_IS_W2L(amlbt_if_type))
    {
        printk("amlbt_usb_fifo_init:w %#lx, r %#lx, b %#lx,\n", (unsigned long)g_15p4_rx_local_fifo->w,
                    (unsigned long)g_15p4_rx_local_fifo->r, (unsigned long)g_15p4_rx_local_fifo->base_addr);
    }
}

void amlbt_usb_fifo_deinit(void)
{
    unsigned int st_reg = 0;

    if (FAMILY_TYPE_IS_W2(amlbt_if_type) || FAMILY_TYPE_IS_W2L(amlbt_if_type))
    {
        st_reg = amlbt_usb_read_word(usb_driver_fw_status_ptr, BT_EP);
        st_reg |= WF_SRAM_FD_INIT_FLAG;
        amlbt_usb_write_word(usb_driver_fw_status_ptr, st_reg, BT_EP);
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
    if (FAMILY_TYPE_IS_W2(amlbt_if_type) || FAMILY_TYPE_IS_W2L(amlbt_if_type))
    {
        amlbt_usb_write_word(usb_driver_fw_status_ptr, st_reg, BT_EP);
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
    unsigned int iccm_base_addr = BT_ICCM_AHB_BASE + bt_iccm_rom_size;
    unsigned int dccm_base_addr = BT_DCCM_AHB_BASE;
    unsigned int *p_check = NULL;
    uint32_t fw_iccmLen = 0;
    uint8_t *fw_iccmBuf = NULL;
    uint32_t fw_dccmLen = 0;
    uint8_t *fw_dccmBuf = NULL;

    fw_iccmLen = bt_iccm_size / 256;
    fw_iccmBuf = BT_fwICCM;
    fw_dccmLen = bt_dccm_size / 256;
    fw_dccmBuf = BT_fwDCCM;

    BTD("iccm check:\n");

    iccm_base_addr = BT_ICCM_AHB_BASE + bt_iccm_rom_size;
    p_check = (unsigned int *)fw_iccmBuf;
    for (offset = 0; offset < (fw_iccmLen / 4); offset++)
    {
        BTD("%s -s check %#x\n", __func__, iccm_base_addr);
        if (FAMILY_TYPE_IS_W2(amlbt_if_type) || FAMILY_TYPE_IS_W2L(amlbt_if_type))
        {
            st_reg = amlbt_usb_read_word(iccm_base_addr, BT_EP);
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
            st_reg = amlbt_usb_read_word(dccm_base_addr, BT_EP);
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
        amlbt_usb_read_sram(fw_read_buff, (unsigned char *)addr,
                                          512, BT_EP);
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
    if (FAMILY_TYPE_IS_W2(amlbt_if_type) || FAMILY_TYPE_IS_W2L(amlbt_if_type))
    {
        amlbt_usb_write_sram(buf, (unsigned char *)(unsigned long)(addr), len, BT_EP);
    }
}

void amlbt_usb_download_firmware(void)
{
    unsigned int offset = 0;
    unsigned int remain_len = 0;
    unsigned int iccm_base_addr = BT_ICCM_AHB_BASE + bt_iccm_rom_size;
    unsigned int dccm_base_addr = BT_DCCM_AHB_BASE;
    unsigned int download_size = 0;
    uint32_t fw_iccmLen = 0;
    uint8_t *fw_iccmBuf = NULL;
    uint32_t fw_dccmLen = 0;
    uint8_t *fw_dccmBuf = NULL;

    fw_iccmLen = bt_iccm_size;
    fw_iccmBuf = BT_fwICCM;
    fw_dccmLen = bt_dccm_size;
    fw_dccmBuf = BT_fwDCCM;

    download_size = fw_iccmLen;

    //to do download bt fw
    printk("bt_usb_download_firmware:iccm size %#x\n", download_size);
    //g_auc_hif_ops.bt_hi_write_word(0xf03050, 1);    //ram power down rg_ram_pd_shutdown_sw
    //g_auc_hif_ops.bt_hi_write_word(REG_DEV_RESET, 0);    //pmu down

    if (FAMILY_TYPE_IS_W1U(amlbt_if_type))
    {
        amlbt_usb_write_word(REG_DEV_RESET, (unsigned int)((BIT_CPU|BIT_MAC|BIT_PHY) << DEV_RESET_HW), BT_EP);    //pmu up
    }

    remain_len = (download_size - offset);
    while (offset < download_size)
    {
        if (remain_len < USB_DOWNLOAD_LEN)
        {
            amlbt_usb_write_firmware((unsigned char *)&fw_iccmBuf[offset], remain_len, iccm_base_addr);
            offset += remain_len;
            iccm_base_addr += remain_len;
            BTA("bt_usb_download_firmware iccm1 offset %#x, write_len %#x\n", offset, remain_len);
        }
        else
        {
            amlbt_usb_write_firmware((unsigned char *)&fw_iccmBuf[offset], USB_DOWNLOAD_LEN, iccm_base_addr);
            offset += USB_DOWNLOAD_LEN;
            remain_len -= USB_DOWNLOAD_LEN;
            iccm_base_addr += USB_DOWNLOAD_LEN;
            BTA("bt_usb_download_firmware iccm2 offset %#x, write_len %#x\n", offset, USB_DOWNLOAD_LEN);
        }
    }

    download_size = fw_dccmLen;

    //to do download bt fw
    //printk("bt_usb_download_firmware:dccm size %#x\n", download_size);
    offset = 0;
    remain_len = download_size;
    while (offset < download_size)
    {
        if (remain_len < USB_DOWNLOAD_LEN)
        {
            amlbt_usb_write_firmware((unsigned char *)&fw_dccmBuf[offset], remain_len, dccm_base_addr);
            offset += remain_len;
            dccm_base_addr += remain_len;
        }
        else
        {
            amlbt_usb_write_firmware((unsigned char *)&fw_dccmBuf[offset], USB_DOWNLOAD_LEN, dccm_base_addr);
            offset += USB_DOWNLOAD_LEN;
            remain_len -= USB_DOWNLOAD_LEN;
            dccm_base_addr += USB_DOWNLOAD_LEN;
        }

        printk("bt_usb_download_firmware dccm remain_len %#x \n", remain_len);
    }
    //amlbt_usb_firmware_check();
}

static int amlbt_usb_char_open(struct inode *inode_p, struct file *file_p)
{
    int rf_num= 0;
    printk("%s, %#x, %#x, %#x\n", __func__, download_fw, bt_recovery, AML_BT_VERSION);
    close_state = 0;

    amlbt_usb_ram_init();

    if (download_fw)
    {
        amlbt_usb_reset();
        amlbt_usb_init();
        download_fw = 1;
        download_end = 0;
        download_flag = 1;
        fw_cmd_w = 0;
        fw_cmd_r = 0;
    }
    else
    {
        download_fw = 0;
        download_end = 0;
        download_flag = 0;
        fw_cmd_w = 0;
        fw_cmd_r = 0;
        amlbt_buff_init();
        if (FAMILY_TYPE_IS_W2L(amlbt_if_type))
        {
            aml_15p4_buff_init();
        }
        if (g_lib_cmd_fifo == NULL)
        {
            memset(g_lib_cmd_buff, 0, CMD_FIFO_SIZE);
            g_lib_cmd_fifo = gdsl_fifo_init(CMD_FIFO_SIZE, g_lib_cmd_buff);
        }
    }
    rf_num = ((amlbt_usb_read_word(REG_PMU_POWER_CFG, BT_EP) >> BIT_RF_NUM) & 0x03);
    printk("%s set rf num %#x", __func__, rf_num);
    init_completion(&usb_completion);
    return nonseekable_open(inode_p, file_p);
}
static void amlbt_usb_char_deinit(void);

static int amlbt_usb_char_close(struct inode *inode_p, struct file *file_p)
{
    printk("%s $$$$$$$$$$$$$$$$$$$$$$$$$$$$ %#x, %#x\n", __func__, download_fw, bt_recovery);

    if (g_event_fifo != 0)
    {
        printk("event w:%p,r:%p\n", g_event_fifo->w, g_event_fifo->r);
    }

    if (download_fw)
    {
        amlbt_usb_deinit();
        if (bt_recovery || fwdead_value)
        {
            amlbt_usb_fifo_deinit();
        }
    }

    if (bt_recovery || fwdead_value)
    {
        download_fw = 0;
        gdsl_fifo_deinit(g_lib_cmd_fifo);
        g_lib_cmd_fifo = 0;
        amlbt_buff_deinit();
        if (FAMILY_TYPE_IS_W2L(amlbt_if_type))
        {
            aml_15p4_buff_deinit();
        }
        download_end = 0;
        download_flag = 0;
        bt_recovery = 0;
        fwdead_value = 0;
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
                printk("%s end \n", __func__);
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
    ...
    15.4 Rx Queue WPoint->0x53c  [0x53c,0x53d,0x53e,0x53f]
    15.4 Rx Queue RPoint->0x540  [0x540,0x541,0x542,0x543]
    15.4 Tx Queue WPoint->0x544  [0x544,0x545,0x546,0x547]
    15.4 Tx Queue RPoint->0x548  [0x548,0x549,0x54a,0x54b]
*/

static int amlbt_usb_get_data(void)
{
    unsigned int read_len = 0;
    unsigned char data_index[USB_RX_INDEX_FIFO_LEN] = {0};
    unsigned int type_size = 0;
    unsigned int _15p4_size = 0;
    unsigned int evt_size = 0;
    unsigned int data_size = 0;
    unsigned long w_point = 0;
    static unsigned char fw_read_buff[USB_EVENT_Q_LEN*4] = {0};
    static unsigned char type_buff[USB_RX_TYPE_FIFO_LEN] = {0};
    unsigned int i = 0;
    unsigned int reg = 0;
    unsigned char read_reg[16] = {0};
    gdsl_fifo_t read_fifo = {0};
    unsigned char *p_data = NULL;

    USB_BEGIN_LOCK();
    if (FAMILY_TYPE_IS_W2L(amlbt_if_type))
    {
        amlbt_usb_read_sram_ext(&bt_usb_data_buf[0], (unsigned char *)(unsigned long)hci_rx_type_rd_ptr, USB_POLL_TOTAL_LEN + 16, BT_EP);
    }
    else
    {
        amlbt_usb_read_sram_ext(&bt_usb_data_buf[0], (unsigned char *)(unsigned long)hci_rx_type_rd_ptr, USB_POLL_TOTAL_LEN, BT_EP);
    }

    if (bt_recovery)
    {
        USB_END_LOCK();
        return -EFAULT;
    }
    //1. process type
    w_point = ((bt_usb_data_buf[35]<<24)|(bt_usb_data_buf[34]<<16)|(bt_usb_data_buf[33]<<8)|bt_usb_data_buf[32]);
    g_event_fifo->w = (unsigned char *)(unsigned long)((bt_usb_data_buf[39]<<24)|(bt_usb_data_buf[38]<<16)|(bt_usb_data_buf[37]<<8)|bt_usb_data_buf[36]);
    g_rx_fifo->w = (unsigned char *)(unsigned long)((bt_usb_data_buf[31]<<24)|(bt_usb_data_buf[30]<<16)|(bt_usb_data_buf[29]<<8)|bt_usb_data_buf[28]);
    g_rx_type_fifo->w = (unsigned char *)w_point;
    if (((unsigned long)g_rx_type_fifo->w > USB_RX_TYPE_FIFO_LEN) ||
            ((unsigned long)g_event_fifo->w > USB_POLL_TOTAL_LEN) ||
                ((unsigned long)g_rx_fifo->w > USB_RX_Q_LEN*4))
    {
        fwdead_value = 1;
        return -EFAULT;
    }

    if (g_rx_type_fifo->w == g_rx_type_fifo->r)
    {
        USB_END_LOCK();
        return -EFAULT;
    }
    read_fifo.base_addr = &bt_usb_data_buf[hci_rx_type_fifo_addr - hci_rx_type_rd_ptr];
    read_fifo.r = g_rx_type_fifo->r;
    read_fifo.w = g_rx_type_fifo->w;
    read_fifo.size = hci_rx_type_fifo_size;

    type_size = gdsl_fifo_calc_r(&read_fifo, type_buff, sizeof(type_buff));
    if (!type_size)
    {
        USB_END_LOCK();
        return -EFAULT;
    }
    reg = (((unsigned int)(unsigned long)read_fifo.r) & 0xff);
    read_reg[0] = (reg & 0xff);
    read_reg[1] = ((reg >> 8) & 0xff);
    read_reg[2] = ((reg >> 16) & 0xff);
    read_reg[3] = ((reg >> 24) & 0xff);
    g_event_fifo->w = (unsigned char *)(unsigned long)((bt_usb_data_buf[39]<<24)|(bt_usb_data_buf[38]<<16)|(bt_usb_data_buf[37]<<8)|bt_usb_data_buf[36]);
    read_fifo.r = g_event_fifo->r;
    if (g_event_fifo->w != g_event_fifo->r)
    {
        read_fifo.base_addr = &bt_usb_data_buf[hci_evt_fifo_addr - hci_rx_type_rd_ptr];
        read_fifo.r = g_event_fifo->r;
        read_fifo.w = g_event_fifo->w;
        read_fifo.size = USB_EVENT_Q_LEN;
        gdsl_fifo_calc_r(&read_fifo, fw_read_buff, sizeof(fw_read_buff));
    }
    reg = (((unsigned int)(unsigned long)read_fifo.r) & 0x1fff);
    read_reg[4] = (reg & 0xff);
    read_reg[5] = ((reg >> 8) & 0xff);
    read_reg[6] = ((reg >> 16) & 0xff);
    read_reg[7] = ((reg >> 24) & 0xff);
    //3. process data
    g_rx_fifo->w = (unsigned char *)(unsigned long)((bt_usb_data_buf[31]<<24)|(bt_usb_data_buf[30]<<16)|(bt_usb_data_buf[29]<<8)|bt_usb_data_buf[28]);
    read_fifo.r = g_rx_fifo->r;
    if (g_rx_fifo->w != g_rx_fifo->r)
    {
        amlbt_usb_read_sram_ext(&fw_read_buff[0],
                                              (unsigned char *)(unsigned long)(hci_data_rx_queue_addr), USB_RX_Q_LEN*4,
                                              BT_EP);
        if (bt_recovery)
        {
            USB_END_LOCK();
            return -EFAULT;
        }
        read_fifo.base_addr = &bt_usb_data_buf[hci_data_rx_index_fifo_addr - hci_rx_type_rd_ptr];
        read_fifo.r = g_rx_fifo->r;
        read_fifo.w = g_rx_fifo->w;
        read_fifo.size = USB_RX_INDEX_FIFO_LEN;
        gdsl_fifo_calc_r(&read_fifo, data_index, sizeof(data_index));
    }
    reg = (((unsigned int)(unsigned long)read_fifo.r) & 0x1f);
    read_reg[8] = (reg & 0xff);
    read_reg[9] = ((reg >> 8) & 0xff);
    read_reg[10] = ((reg >> 16) & 0xff);
    read_reg[11] = ((reg >> 24) & 0xff);
    amlbt_usb_write_sram_ext(&read_reg[0],
                                      (unsigned char *)(unsigned long)(hci_rx_type_rd_ptr), 16,
                                      BT_EP);
    if (bt_recovery)
    {
        USB_END_LOCK();
        return -EFAULT;
    }
    USB_END_LOCK();

    //1. process type
    w_point = ((bt_usb_data_buf[35]<<24)|(bt_usb_data_buf[34]<<16)|(bt_usb_data_buf[33]<<8)|bt_usb_data_buf[32]);
    g_rx_type_fifo->w = (unsigned char *)w_point;
    read_fifo.base_addr = &bt_usb_data_buf[hci_rx_type_fifo_addr - hci_rx_type_rd_ptr];
    read_fifo.r = g_rx_type_fifo->r;
    read_fifo.w = g_rx_type_fifo->w;
    read_fifo.size = hci_rx_type_fifo_size;

    type_size = gdsl_fifo_get_data(&read_fifo, type_buff, sizeof(type_buff));
    if (type_buff[0] != 0x2 && type_buff[0] != 0x4 && type_buff[0] != 0x10)
    {
        printk("%s g_rx_type_fifo->w:%#x g_rx_type_fifo->r:%#x", __func__, g_rx_type_fifo->w, g_rx_type_fifo->r);
        g_event_fifo->w = (unsigned char *)(unsigned long)((bt_usb_data_buf[39]<<24)|(bt_usb_data_buf[38]<<16)|(bt_usb_data_buf[37]<<8)|bt_usb_data_buf[36]);
        printk("%s g_event_fifo->w:%#x g_event_fifo->r:%#x", __func__, g_event_fifo->w, g_event_fifo->r);
        g_rx_fifo->w = (unsigned char *)(unsigned long)((bt_usb_data_buf[31]<<24)|(bt_usb_data_buf[30]<<16)|(bt_usb_data_buf[29]<<8)|bt_usb_data_buf[28]);
        printk("%s g_rx_fifo->w:%#x g_rx_fifo->r:%#x", __func__, g_rx_fifo->w, g_rx_fifo->r);
    }
    BTD("type fifo:w %#lx, r %#lx\n", (unsigned long)g_rx_type_fifo->w, (unsigned long)g_rx_type_fifo->r);
    if (type_size)
    {
        g_rx_type_fifo->r = read_fifo.r;
        BTD("TYPE:%d [%#x,%#x,%#x,%#x]\n", type_size, type_buff[0],
           type_buff[4], type_buff[8], type_buff[12]);
    }
    else
    {
        return -EFAULT;
    }
    //2. process data
    g_rx_fifo->w = (unsigned char *)(unsigned long)((bt_usb_data_buf[31]<<24)|(bt_usb_data_buf[30]<<16)|(bt_usb_data_buf[29]<<8)|bt_usb_data_buf[28]);
    read_fifo.base_addr = &bt_usb_data_buf[hci_data_rx_index_fifo_addr - hci_rx_type_rd_ptr];
    read_fifo.r = g_rx_fifo->r;
    read_fifo.w = g_rx_fifo->w;
    read_fifo.size = USB_RX_INDEX_FIFO_LEN;
    data_size = gdsl_fifo_get_data(&read_fifo, data_index, sizeof(data_index));
    if (data_size)
    {
        g_rx_fifo->r = read_fifo.r;
    }

    for (i = 0; i < data_size; i+=4)
    {
        if (FAMILY_TYPE_IS_W2(amlbt_if_type) || FAMILY_TYPE_IS_W2L(amlbt_if_type))
        {
            p_data = &fw_read_buff[data_index[i] * USB_RX_Q_LEN];
            read_len = ((p_data[7] << 8) | (p_data[6]));
            read_len = ((read_len + 3) & 0xFFFFFFFC);
            BTD("!%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x!\n", p_data[0],
                   p_data[1], p_data[2], p_data[3], p_data[4], p_data[5],
                   p_data[6], p_data[7]);
            BTD("r dh %#x\n", read_len);
        }
        gdsl_fifo_copy_data(g_fw_data_fifo, p_data, (read_len + 8));
        BTD("HEAD1:[%#x,%#x,%#x,%#x]\n", p_data[0],
               p_data[1], p_data[2], p_data[3]);
        BTD("HEAD2:[%#x,%#x,%#x,%#x]\n", p_data[4],
               p_data[5], p_data[6], p_data[7]);
        BTD("HEAD3:[%#x,%#x,%#x,%#x]\n", p_data[8],
               p_data[9], p_data[10], p_data[11]);
        BTD("HEAD4:[%#x,%#x,%#x,%#x]\n", p_data[12],
               p_data[13], p_data[14], p_data[15]);
    }
    //3. process event
    g_event_fifo->w = (unsigned char *)(unsigned long)((bt_usb_data_buf[39]<<24)|(bt_usb_data_buf[38]<<16)|(bt_usb_data_buf[37]<<8)|bt_usb_data_buf[36]);
    if (g_event_fifo->w != g_event_fifo->r)
    {
        read_fifo.base_addr = &bt_usb_data_buf[hci_evt_fifo_addr - hci_rx_type_rd_ptr];
        read_fifo.r = g_event_fifo->r;
        read_fifo.w = g_event_fifo->w;
        read_fifo.size = USB_EVENT_Q_LEN;
        evt_size = gdsl_fifo_get_data(&read_fifo, fw_read_buff, sizeof(fw_read_buff));
    }
    if (evt_size)
    {
        g_event_fifo->r = read_fifo.r;
        gdsl_fifo_copy_data(g_fw_evt_fifo, fw_read_buff, evt_size);
    }
    //complete(&data_completion);
    //printk("[r:%#x w:%#x]", g_rx_type_fifo->r, g_rx_type_fifo->w);

    if (FAMILY_TYPE_IS_W2L(amlbt_if_type))
    {
        //4. process 15.4
        w_point = ((bt_usb_data_buf[0x53f]<<24)|(bt_usb_data_buf[0x53e]<<16)|(bt_usb_data_buf[0x53d]<<8)|bt_usb_data_buf[0x53c]);
        g_15p4_rx_fifo->w = (unsigned char *)w_point;
        //BTI("15.4 rx:w %#lx, r %#lx\n", (unsigned long)g_15p4_rx_fifo->w, (unsigned long)g_15p4_rx_fifo->r);
        if (g_15p4_rx_fifo->w != g_15p4_rx_fifo->r)
        {
            memset(fw_read_buff, 0, sizeof(fw_read_buff));
            _15p4_size = 0;
            amlbt_usb_read_sram_ext(fw_read_buff, (unsigned char *)_15p4_rx_queue_addr, _15p4_rx_queue_size, BT_EP);
            read_fifo.base_addr = fw_read_buff;
            read_fifo.r = g_15p4_rx_fifo->r;
            read_fifo.w = g_15p4_rx_fifo->w;
            read_fifo.size = _15p4_rx_queue_size;
            _15p4_size = gdsl_fifo_get_data(&read_fifo, &fw_read_buff[_15p4_rx_queue_size], _15p4_rx_queue_size);
            printk("15.4 get size %d\n", _15p4_size);
            printk("15.4 rx:w %#lx, r %#lx\n", (unsigned long)g_15p4_rx_fifo->w, (unsigned long)g_15p4_rx_fifo->r);
            if (_15p4_size)
            {
                g_15p4_rx_fifo->r = read_fifo.r;
                printk("15.4 copy:w %#lx, r %#lx, b %#lx,\n", (unsigned long)g_15p4_rx_local_fifo->w,
                    (unsigned long)g_15p4_rx_local_fifo->r, (unsigned long)g_15p4_rx_local_fifo->base_addr);
                gdsl_fifo_copy_data(g_15p4_rx_local_fifo, &fw_read_buff[_15p4_rx_queue_size], _15p4_size);
                reg = (((unsigned int)(unsigned long)read_fifo.r) & 0x7ff);
                amlbt_usb_write_word(_15p4_rx_rd_ptr, reg, BT_EP);
                printk("15.4 update rx rd %#x\n", amlbt_usb_read_word(_15p4_rx_rd_ptr, BT_EP));
            }
        }
    }
    gdsl_fifo_copy_data(g_fw_type_fifo, type_buff, type_size);
    bt_wt_ptr_local = (unsigned long)g_rx_type_fifo->w;
    return 0;
}

int amlbt_usb_check_fw_rx(void *data)
{
    bool fw_type;

    s64 last = ktime_to_ns(ktime_get_real());
    s64 now = 0;

    while (!kthread_should_stop())
    {
        if (close_state)
        {
            printk("%s R CLOSE\n", __func__);
            check_fw_rx_stask = NULL;
            break;
        }
        if (bt_recovery)
        {
            wake_up_interruptible(&poll_amlbt_queue);
            printk("%s R CLOSE\n", __func__);
            check_fw_rx_stask = NULL;
            break;
        }
        while (suspend_value || shutdown_value)
        {
            usleep_range(20000, 20000);
            if (close_state)
            {
                printk("%s R1 CLOSE\n", __func__);
                check_fw_rx_stask = NULL;
                break;
            }
        }

        fw_type = gdsl_fifo_used(g_fw_type_fifo);
        if (fw_type)
        {
            BTD("%s fw_type1 %#x\n", __func__, fw_type);
            wake_up_interruptible(&poll_amlbt_queue);
            continue;
        }
        if (!suspend_value)
        {
            now = ktime_to_ns(ktime_get_real());

            if ((abs(now - last) >= 5500000) || (((bt_wt_ptr & 0x1fff) != (bt_rd_ptr & 0x1fff)) && (bt_wt_ptr_local < bt_wt_ptr)))
            //if ((abs(now - last) >= 12000000))
            {
                last = now;
                amlbt_usb_get_data();
                if (fwdead_value)
                {
                    check_fw_rx_stask = NULL;
                    break;
                }
                fw_type = gdsl_fifo_used(g_fw_type_fifo);
                //printk("[%#x, %#x, %#x, %#x]\n", bt_rd_ptr, bt_wt_ptr, now, last);
                if (fw_type)
                {
                    BTD("%s fw_type %#x\n", __func__, fw_type);
                    wake_up_interruptible(&poll_amlbt_queue);
                    //while (down_interruptible(&read_rx_sem) !=0);
                    if ((unsigned long)g_rx_type_fifo->w > USB_RX_TYPE_FIFO_LEN)
                    {
                        check_fw_rx_stask = NULL;
                        fwdead_value = 1;
                        break;
                    }
                }
            }
            else
            {
                usleep_range(5000, 5000);
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
    unsigned char close_evt[7] = {0x04, 0x0e, 0x04, 0x01, 0x27, 0xfc, 0x00};
    static unsigned char host_read_buff[USB_TX_Q_LEN] = {0};
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
            if (bt_recovery)
            {
                return -EFAULT;
            }
            gdsl_fifo_get_data(g_fw_type_fifo, bt_type, sizeof(bt_type));
            BTD("tp(%#x,%#x,%#x,%#x)\n", bt_type[0],bt_type[1],bt_type[2],bt_type[3]);
            if (bt_type[0] != 0x4 && bt_type[0] != 0x2 && bt_type[0] != 0x10)
            {
                printk("TYPE ERROR(%#x,%#x,%#x,%#x)\n", bt_type[0],bt_type[1],bt_type[2],bt_type[3]);
                printk("g_fw_type_fifo->w:%#x g_fw_type_fifo->r:%#x\n", g_fw_type_fifo->w, g_fw_type_fifo->r);
                if (FAMILY_TYPE_IS_W2L(amlbt_if_type))
                {
                    printk("g_15p4_rx_local_fifo->w:%#x g_15p4_rx_local_fifo->r:%#x\n", g_15p4_rx_local_fifo->w, g_15p4_rx_local_fifo->r);
                }
                printk("g_fw_evt_fifo->w:%#x g_fw_evt_fifo->r:%#x\n", g_fw_evt_fifo->w, g_fw_evt_fifo->r);
                printk("g_fw_data_fifo->w:%#x g_fw_data_fifo->r:%#x\n", g_fw_data_fifo->w, g_fw_data_fifo->r);
                return -EFAULT;
            }
            if (copy_to_user(buf_p, bt_type, count))
            {
                printk("%s, copy_to_user error \n", __func__);
                return -EFAULT;
            }
        break;
        case 1:                 // read header
            evt_state = 2;
            if (bt_type[0] == HCI_EVENT_PKT)
            {
                if (close_state)
                {
                    printk("%s R3 CLOSE\n", __func__);
                    if (copy_to_user(buf_p, &close_evt[1], count))
                    {
                        printk("%s, copy_to_user error \n", __func__);
                        return -EFAULT;
                    }
                    return count;
                }
                gdsl_fifo_get_data(g_fw_evt_fifo, host_read_buff, 4);
                if (copy_to_user(buf_p, &host_read_buff[1], count))
                {
                    printk("%s, copy_to_user error \n", __func__);
                    return -EFAULT;
                }
                BTD("E:%#x|%#x|%#x|%#x\n", host_read_buff[0], host_read_buff[1],
                       host_read_buff[2], host_read_buff[3]);
            }
            else if (bt_type[0] == HCI_ACLDATA_PKT)
            {
                gdsl_fifo_get_data(g_fw_data_fifo, host_read_buff, 8);
                BTD("D:%#x|%#x|%#x|%#x\n", host_read_buff[0], host_read_buff[1],
                       host_read_buff[2], host_read_buff[3]);
                if (copy_to_user(buf_p, &host_read_buff[4], count))
                {
                    printk("%s, copy_to_user error \n", __func__);
                    return -EFAULT;
                }
            }
            else if (FAMILY_TYPE_IS_W2L(amlbt_if_type) && bt_type[0] == HCI_15P4_PKT)
            {
                gdsl_fifo_get_data(g_15p4_rx_local_fifo, host_read_buff, 5);
                read_len = ((7 + ((host_read_buff[4] << 8) | (host_read_buff[3])) + 3) & 0xFFFFFFFC);
                gdsl_fifo_get_data(g_15p4_rx_local_fifo, &host_read_buff[5], read_len - 5);
                printk("15.4 read len %d\n", read_len);
                printk("15.4 header:%#x|%#x|%#x|%#x|%#x\n", host_read_buff[0], host_read_buff[1],
                       host_read_buff[2], host_read_buff[3], host_read_buff[4]);
                printk("15.4 local:w %#lx, r %#lx\n", (unsigned long)g_15p4_rx_local_fifo->w, (unsigned long)g_15p4_rx_local_fifo->r);
                if (copy_to_user(buf_p, &host_read_buff[1], count))
                {
                    printk("%s, copy_to_user error \n", __func__);
                    return -EFAULT;
                }
            }
        break;
        case 2:					//read payload
            evt_state = 0;
            if (bt_type[0] == HCI_EVENT_PKT)
            {
                read_len = host_read_buff[2];
                read_len -= 1;
                read_len = ((read_len + 3) & 0xFFFFFFFC);
                gdsl_fifo_get_data(g_fw_evt_fifo, &host_read_buff[4], read_len);
                BTD("E:[%#x|%#x|%#x|%#x|%#x|%#x|%#x|%#x]\n", host_read_buff[0], host_read_buff[1],
                       host_read_buff[2], host_read_buff[3],host_read_buff[4], host_read_buff[5],
                       host_read_buff[6], host_read_buff[7]);
                if (close_state)
                {
                    printk("%s R4 CLOSE\n", __func__);
                    if (copy_to_user(buf_p, &close_evt[3], count))
                    {
                        printk("%s, copy_to_user error \n", __func__);
                        return -EFAULT;
                    }
                    return count;
                }
                if (copy_to_user(buf_p, &host_read_buff[3], count))
                {
                    printk("%s, copy_to_user error \n", __func__);
                    return -EFAULT;
                }
#ifdef BT_USB_DBG
                if (host_read_buff[1] == 0x13 && host_read_buff[4] == dbg_handle)
                {
                    dbg_credit++;
                    printk("N:%d,%#x\n", dbg_credit,amlbt_usb_read_word(0x2f61c8, BT_EP));
                }
#endif
            }
            else if (bt_type[0] == HCI_ACLDATA_PKT)
            {
                read_len = ((host_read_buff[7] << 8) | (host_read_buff[6]));
                read_len = ((read_len + 3) & 0xFFFFFFFC);
                gdsl_fifo_get_data(g_fw_data_fifo, &host_read_buff[8], read_len);
                BTD("D:%#x|%#x|%#x|%#x|%#x|%#x|%#x|%#x\n", host_read_buff[4], host_read_buff[5],
                       host_read_buff[6], host_read_buff[7], host_read_buff[8],
                       host_read_buff[9], host_read_buff[10], host_read_buff[11]);
                if (copy_to_user(buf_p, &host_read_buff[8], count))
                {
                    printk("%s, copy_to_user error \n", __func__);
                    return -EFAULT;
                }
            }
            else if (FAMILY_TYPE_IS_W2L(amlbt_if_type) && bt_type[0] == HCI_15P4_PKT)
            {
                //read_len = ((host_read_buff[4] << 8) | (host_read_buff[3])) + 2;
                //read_len -= 1;
                //read_len = ((read_len + 3) & 0xFFFFFFFC);
                printk("15.4 read payload len %d\n", count);
                //gdsl_fifo_get_data(g_15p4_rx_local_fifo, &host_read_buff[5], count);

                printk("15.4 payload:[%#x|%#x|%#x|%#x|%#x|%#x|%#x|%#x]\n", host_read_buff[5], host_read_buff[6],
                       host_read_buff[7], host_read_buff[8]);

                printk("15.4 get:w %#lx, r %#lx\n", (unsigned long)g_15p4_rx_local_fifo->w, (unsigned long)g_15p4_rx_local_fifo->r);
                printk("type:w %#lx, r %#lx\n", (unsigned long)g_fw_type_fifo->w, (unsigned long)g_fw_type_fifo->r);

                if (copy_to_user(buf_p, &host_read_buff[5], count))
                {
                    printk("%s, copy_to_user error \n", __func__);
                    return -EFAULT;
                }
            }
            break;
        default:
            printk("%s, evt_state error!!\n", __func__);
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
        printk("%s: Failed to get data from user space\n", __func__);
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
        printk("---------------cmd error!------------");
        return -EINVAL;
    }

    gdsl_fifo_copy_data(g_lib_cmd_fifo, p_acl_buf, 2);

    if (p_acl_buf[0] == 0xf3 && p_acl_buf[1] == 0xfe)   //download fw
    {
        len = count - 7;
        offset = ((p_acl_buf[6] << 24) | (p_acl_buf[5] << 16) | (p_acl_buf[4] << 8) | p_acl_buf[3]);
        BTA("%#x,%#x,%#x\n", len, offset, dw_state);
        BTA("%#x,%#x,%#x,%#x\n", p_acl_buf[7], p_acl_buf[8], p_acl_buf[9], p_acl_buf[10]);
        if (offset == bt_iccm_rom_size)
        {
            if (!download_flag)
            {
                //Generic Process
                amlbt_usb_write_word(REG_RAM_PD_SHUTDWONW_SW, 0, BT_EP);
            }
            iccm_base_addr = 0;
            dccm_base_addr = 0;
            dw_state = 0;
            if (BT_fwICCM == NULL)
            {
                BT_fwICCM = vmalloc(bt_iccm_size);
            }
            if (BT_fwDCCM == NULL)
            {
                BT_fwDCCM = vmalloc(bt_dccm_size);
            }

            if (BT_fwICCM == NULL || BT_fwDCCM == NULL)
            {
                printk("amlbt_usb_char_write_fw vmalloc err!!\n");
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
        printk("WR:%#x,%#x\n", offset, reg_value);
        if (offset == 0xa70014 && !(reg_value & (1 << 24))) //rf calibration
        {
            //amlbt_usb_firmware_check();
            if (!download_flag)
            {
                amlbt_usb_download_firmware();
                printk("download finished!\n");
                vfree(BT_fwICCM);
                vfree(BT_fwDCCM);
                BT_fwICCM = NULL;
                BT_fwDCCM = NULL;
            }
            printk("W E %#x,%#x\n", iccm_base_addr, dccm_base_addr);
        }
        else if (offset == REG_PMU_POWER_CFG)
        {
            //Generic Process
            amlbt_usb_write_word(REG_PMU_POWER_CFG, reg_value, BT_EP); // rg_pmu_power_cfg
        }
        else if (offset == REG_FW_MODE)
        {
            amlbt_usb_write_word(REG_FW_MODE, reg_value, BT_EP); //W2L USED
        }
        else if (offset == REG_DEV_RESET)
        {
            download_end = 1;
            if ((g_auc_hif_ops.hi_write_word_for_bt == NULL) || (g_auc_hif_ops.hi_read_word_for_bt == NULL) ||
            (g_auc_hif_ops.hi_write_sram_for_bt == NULL) || (g_auc_hif_ops.hi_read_sram_for_bt == NULL))
            {
                printk("USB Interface NULL");
            }
            else
            {
                amlbt_usb_reset();
                amlbt_usb_init();
                //Generic Process
                amlbt_usb_write_word(REG_DEV_RESET, (unsigned int)((BIT_CPU | BIT_MAC | BIT_PHY) << DEV_RESET_SW),
                                               BT_EP);
                printk("%s end 19:30,bt reg : %#x\n", __func__, amlbt_usb_read_word(REG_DEV_RESET, BT_EP));
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
        printk("count > HCI_MAX_FRAME_SIZE %d, %d\n", count, HCI_MAX_FRAME_SIZE);
        return -EINVAL;
    }

    if (!download_fw)
    {
        return amlbt_usb_char_write_fw(file_p, buf_p, count, pos_p);
    }

    if (count == 1)	//host write hci type
    {
        get_user(w_type, buf_p);
        return count;
    }

    if (copy_from_user(p_acl_buf, buf_p, count))
    {
        printk("%s: Failed to get data from user space\n", __func__);
        //mutex_unlock(&usb_rw_lock);
        return -EFAULT;
    }

    if (count > 1 && w_type == HCI_COMMAND_PKT)
    {
        BTD("hci cmd:[%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x]",
            p_acl_buf[0],p_acl_buf[1],p_acl_buf[2],p_acl_buf[3],
            p_acl_buf[4],p_acl_buf[5],p_acl_buf[6],p_acl_buf[7]);
    }

    BTD("s: %#x\n", w_type);
    if (w_type == HCI_COMMAND_PKT)
    {
        if (count == 0x0f && p_acl_buf[0] == 0x27 && p_acl_buf[1] == 0xfc)   //close
        {
            for (i = 0; i < 8; i++)
            {
                printk("%#x|", p_acl_buf[i]);
            }
            printk("W CLOSE\n");
            //mutex_unlock(&usb_rw_lock);
            close_state = 1;
            return count;
        }
        if (p_acl_buf[0] == 0x05 && p_acl_buf[1] == 0x14)     // read rssi
        {
            printk("rs\n");
#ifdef BT_USB_DBG
            rssi_dbg();
#endif
        }
        amlbt_usb_send_hci_cmd(p_acl_buf, count);
    }
    else if (w_type == HCI_ACLDATA_PKT)
    {
        amlbt_usb_send_hci_data(p_acl_buf, count);
    }
    else if (FAMILY_TYPE_IS_W2L(amlbt_if_type) && w_type == HCI_15P4_PKT)
    {
        amlbt_usb_send_15p4_data(p_acl_buf, count);
    }

    return count;
}

unsigned int btchr_poll(struct file *file, poll_table *wait)
{
    int mask = 0;

    if (!download_fw)
    {
        return POLLIN | POLLRDNORM;
    }
    BTD("poll \n");
    poll_wait(file, &poll_amlbt_queue, wait);
    if ((evt_state))
    {
        mask |= POLLIN | POLLRDNORM;
    }
    if ((g_auc_hif_ops.hi_write_word_for_bt == NULL) || (g_auc_hif_ops.hi_read_word_for_bt == NULL) ||
    (g_auc_hif_ops.hi_write_sram_for_bt == NULL) || (g_auc_hif_ops.hi_read_sram_for_bt == NULL))
    {
        return mask;
    }
    else if(g_fw_type_fifo->r != g_fw_type_fifo->w)
    {
        if (FAMILY_TYPE_IS_W2L(amlbt_if_type) && (g_15p4_rx_local_fifo->r != g_15p4_rx_local_fifo->w))
        {
            mask |= POLLIN | POLLRDNORM;
        }
        else if (g_fw_evt_fifo->r != g_fw_evt_fifo->w)
        {
            mask |= POLLIN | POLLRDNORM;
        }
        else if (g_fw_data_fifo->r != g_fw_data_fifo->w)
        {
            mask |= POLLIN | POLLRDNORM;
        }
    }

    if (bt_recovery || fwdead_value)
    {
        mask |= POLLIN | POLLRDNORM;
    }
    return mask;
}

static long btuartchr_ioctl(struct file* filp, unsigned int cmd, unsigned long arg)
{
    unsigned long coex_time = 0;
    unsigned int reg_value = 0;
    BTD("arg value %ld", arg);
    BTD("cmd type=%c\t nr=%d\t dir=%d\t size=%d\n", _IOC_TYPE(cmd), _IOC_NR(cmd), _IOC_DIR(cmd), _IOC_SIZE(cmd));
    BTD("cmd value %ld", cmd);
    switch (cmd)
    {
        case IOCTL_SET_BT_COEX_TIME:
        {
            if (copy_from_user(&coex_time, (int __user *)arg, sizeof(int)) != 0)
            {
                printk("IOCTL_SET_BT_COEX_TIME copy error\n");
                return -EFAULT;
            }
            printk("IOCTL_SET_BT_COEX_TIME %#x\n", coex_time);
            if (INTF_TYPE_IS_SDIO(amlbt_if_type))
            {
                reg_value = g_hif_sdio_ops.bt_hi_read_word(RG_AON_A57);
                printk("w RG_AON_A57 %#x %#x", reg_value, coex_time);
                g_hif_sdio_ops.bt_hi_write_word(RG_AON_A57, coex_time);
            }
            if (INTF_TYPE_IS_PCIE(amlbt_if_type))
            {
                reg_value = aml_pci_read_for_bt(AML_ADDR_AON, RG_AON_A57);
                printk("w RG_AON_A57 %#x %#x", reg_value, coex_time);
                aml_pci_write_for_bt(coex_time, AML_ADDR_AON, RG_AON_A57);
            }
        }
        break;
        case IOCTL_SET_WIFI_COEX_TIME:
        {
            if (copy_from_user(&coex_time, (int __user *)arg, sizeof(int)) != 0)
            {
                printk("IOCTL_SET_WIFI_COEX_TIME copy error\n");
                return -EFAULT;
            }
            printk("IOCTL_SET_WIFI_COEX_TIME %#x\n", coex_time);
            if (INTF_TYPE_IS_SDIO(amlbt_if_type))
            {
                reg_value = g_hif_sdio_ops.bt_hi_read_word(RG_AON_A58);
                printk("w RG_AON_A58 %#x %#x", reg_value, coex_time);
                g_hif_sdio_ops.bt_hi_write_word(RG_AON_A58, coex_time);
            }
            if (INTF_TYPE_IS_PCIE(amlbt_if_type))
            {
                reg_value = aml_pci_read_for_bt(AML_ADDR_AON, RG_AON_A58);
                printk("w RG_AON_A58 %#x %#x", reg_value, coex_time);
                aml_pci_write_for_bt(coex_time, AML_ADDR_AON, RG_AON_A58);
            }
        }
        break;
        case IOCTL_SET_WIFI_MAX_DURATION:
        {
            if (copy_from_user(&coex_time, (int __user *)arg, sizeof(int)) != 0)
            {
                printk("IOCTL_SET_WIFI_MAX_DURATION copy error\n");
                return -EFAULT;
            }
            printk("IOCTL_SET_WIFI_MAX_DURATION %#x\n", coex_time);
            if (INTF_TYPE_IS_SDIO(amlbt_if_type))
            {
                reg_value = g_hif_sdio_ops.bt_hi_read_word(RG_AON_A59);
                reg_value = (coex_time & 0xffffff) | (reg_value & 0xff000000);
                printk("w RG_AON_A59 %#x %#x", reg_value, coex_time);
                g_hif_sdio_ops.bt_hi_write_word(RG_AON_A59, reg_value);
            }
            if (INTF_TYPE_IS_PCIE(amlbt_if_type))
            {
                reg_value = aml_pci_read_for_bt(AML_ADDR_AON, RG_AON_A59);
                reg_value = (coex_time & 0xffffff) | (reg_value & 0xff000000);
                printk("w RG_AON_A59 %#x %#x", reg_value, coex_time);
                aml_pci_write_for_bt(reg_value, AML_ADDR_AON, RG_AON_A59);
            }
        }
        break;
        case IOCTL_GET_BT_COEX_TIME:
        {
            if (INTF_TYPE_IS_SDIO(amlbt_if_type))
            {
                coex_time = g_hif_sdio_ops.bt_hi_read_word(RG_AON_A60);
            }
            if (INTF_TYPE_IS_PCIE(amlbt_if_type))
            {
                coex_time = aml_pci_read_for_bt(AML_ADDR_AON, RG_AON_A60);
            }
            if (copy_to_user((int __user *)arg, &coex_time, sizeof(int)) != 0)
            {
                printk("IOCTL_GET_BT_COEX_TIME copy error\n");
                return -EFAULT;
            }
            printk("IOCTL_GET_BT_COEX_TIME %#x\n", coex_time);
        }
        break;
        case IOCTL_GET_WIFI_COEX_TIME:
        {
            if (INTF_TYPE_IS_SDIO(amlbt_if_type))
            {
                coex_time = g_hif_sdio_ops.bt_hi_read_word(RG_AON_A61);
            }
            if (INTF_TYPE_IS_PCIE(amlbt_if_type))
            {
                coex_time = aml_pci_read_for_bt(AML_ADDR_AON, RG_AON_A61);
            }
            if (copy_to_user((int __user *)arg, &coex_time, sizeof(int)) != 0)
            {
                printk("IOCTL_GET_WIFI_COEX_TIME copy error\n");
                return -EFAULT;
            }
            printk("IOCTL_GET_WIFI_COEX_TIME %#x\n", coex_time);
        }
        break;
        case IOCTL_SET_BT_TX_POWER:
        {
            if (copy_from_user(&coex_time, (int __user *)arg, sizeof(int)) != 0)
            {
                printk("IOCTL_SET_BT_TX_POWER copy error\n");
                return -EFAULT;
            }
            printk("IOCTL_SET_BT_TX_POWER %#x\n", coex_time);
            if (INTF_TYPE_IS_SDIO(amlbt_if_type))
            {
                if (coex_time >= 0 && coex_time <= 106)
                {
                    reg_value = g_hif_sdio_ops.bt_hi_read_word(RG_TX_AGAIN);
                    BTD("r RG_TX_AGAIN %#x", reg_value);
                    reg_value = (again[0] | (reg_value & 0xfffff000));
                    printk("w RG_TX_AGAIN %#x %#x", reg_value, again[0]);
                    g_hif_sdio_ops.bt_hi_write_word(RG_TX_AGAIN, reg_value);
                }
                else
                {
                    reg_value = g_hif_sdio_ops.bt_hi_read_word(RG_TX_AGAIN);
                    BTD("r RG_TX_AGAIN %#x", reg_value);
                    reg_value = (again[1] | (reg_value & 0xfffff000));
                    printk("w RG_TX_AGAIN %#x %#x", reg_value, again[1]);
                    g_hif_sdio_ops.bt_hi_write_word(RG_TX_AGAIN, reg_value);
                }
            }
            if (INTF_TYPE_IS_PCIE(amlbt_if_type))
            {
                if (coex_time >= 0 && coex_time <= 106)
                {
                    reg_value = aml_pci_read_for_bt(AML_ADDR_MAC, RG_TX_AGAIN);
                    BTD("r RG_TX_AGAIN %#x", reg_value);
                    reg_value = (again[0] | (reg_value & 0xfffff000));
                    printk("w RG_TX_AGAIN %#x %#x", reg_value, again[0]);
                    aml_pci_write_for_bt(reg_value, AML_ADDR_MAC, RG_TX_AGAIN);
                }
                else
                {
                    reg_value = aml_pci_read_for_bt(AML_ADDR_MAC, RG_TX_AGAIN);
                    BTD("r RG_TX_AGAIN %#x", reg_value);
                    reg_value = (again[1] | (reg_value & 0xfffff000));
                    printk("w RG_TX_AGAIN %#x %#x", reg_value, again[1]);
                    aml_pci_write_for_bt(reg_value, AML_ADDR_MAC, RG_TX_AGAIN);
                }
            }
            if (INTF_TYPE_IS_SDIO(amlbt_if_type))
            {
                reg_value = g_hif_sdio_ops.bt_hi_read_word(RG_TX_DGAIN);
                BTD("r RG_TX_DGAIN %#x", reg_value);
                reg_value = (dgain[coex_time] << 24) | (reg_value & 0x00ffffff);
                printk("w RG_TX_DGAIN %#x %#x", reg_value, dgain[coex_time]);
                g_hif_sdio_ops.bt_hi_write_word(RG_TX_DGAIN, reg_value);

                reg_value = g_hif_sdio_ops.bt_hi_read_word(RG_AON_A59);
                reg_value |= (1 << 31);
                printk("w RG_TX_DGAIN %#x", reg_value);
                g_hif_sdio_ops.bt_hi_write_word(RG_AON_A59, reg_value);
            }
            if (INTF_TYPE_IS_PCIE(amlbt_if_type))
            {
                reg_value = aml_pci_read_for_bt(AML_ADDR_MAC, RG_TX_DGAIN);
                BTD("r RG_TX_DGAIN %#x", reg_value);
                reg_value = (dgain[coex_time] << 24) | (reg_value & 0x00ffffff);
                printk("w RG_TX_DGAIN %#x %#x", reg_value, dgain[coex_time]);
                aml_pci_write_for_bt(reg_value, AML_ADDR_MAC, RG_TX_DGAIN);

                reg_value = aml_pci_read_for_bt(AML_ADDR_AON, RG_AON_A59);
                reg_value |= (1 << 31);
                printk("w RG_TX_DGAIN %#x", reg_value);
                aml_pci_write_for_bt(reg_value, AML_ADDR_AON, RG_AON_A59);
            }
        }
        break;
        case IOCTL_GET_RF_ANTENNA:
        {
            if (INTF_TYPE_IS_SDIO(amlbt_if_type))
            {
                reg_value = g_hif_sdio_ops.bt_hi_read_word(REG_PMU_POWER_CFG);
                coex_time = ((reg_value >> BIT_RF_NUM) & 0x03);
            }
            if (INTF_TYPE_IS_PCIE(amlbt_if_type))
            {
                reg_value = aml_pci_read_for_bt(AML_ADDR_AON, REG_PMU_POWER_CFG);
                coex_time = ((reg_value >> BIT_RF_NUM) & 0x03);
            }
            if (copy_to_user((int __user *)arg, &coex_time, sizeof(int)) != 0)
            {
                printk("IOCTL_GET_RF_ANTENNA copy error\n");
                return -EFAULT;
            }
            printk("IOCTL_GET_RF_ANTENNA %#x\n", coex_time);
        }
        break;
        case IOCTL_GET_BT_RSSI:
        {
            if (INTF_TYPE_IS_SDIO(amlbt_if_type))
            {
                reg_value = g_hif_sdio_ops.bt_hi_read_word(RG_AON_A59);
            }
            if (INTF_TYPE_IS_PCIE(amlbt_if_type))
            {
                reg_value = aml_pci_read_for_bt(AML_ADDR_AON, RG_AON_A59);
            }
            coex_time = ((reg_value >> 24) & 0x7f);
            if (copy_to_user((int __user *)arg, &coex_time, sizeof(int)) != 0)
            {
                printk("IOCTL_GET_BT_RSSI copy error\n");
                return -EFAULT;
            }
            printk("IOCTL_GET_BT_RSSI %#x\n", coex_time);
        }
        break;
        case IOCTL_GET_BT_TIME_PERCENT:
        {
            int bt_time = 0;
            int wf_time = 0;
            int bt_percent = 0;
            if (INTF_TYPE_IS_SDIO(amlbt_if_type))
            {
                bt_time = g_hif_sdio_ops.bt_hi_read_word(RG_AON_A60);
                wf_time = g_hif_sdio_ops.bt_hi_read_word(RG_AON_A61);
            }
            if (INTF_TYPE_IS_PCIE(amlbt_if_type))
            {
                bt_time = aml_pci_read_for_bt(AML_ADDR_AON, RG_AON_A60);
                wf_time = aml_pci_read_for_bt(AML_ADDR_AON, RG_AON_A61);
            }
            bt_percent = (bt_time * 100) / (bt_time + wf_time);
            if (copy_to_user((int __user *)arg, &bt_percent, sizeof(int)) != 0)
            {
                printk("IOCTL_GET_BT_TIME_PERCENT copy error\n");
                return -EFAULT;
            }
            printk("IOCTL_GET_BT_TIME_PERCENT %d\n", bt_percent);
        }
        break;
        case IOCTL_GET_WF_TIME_PERCENT:
        {
            int bt_time = 0;
            int wf_time = 0;
            int wf_percent = 0;
            if (INTF_TYPE_IS_SDIO(amlbt_if_type))
            {
                bt_time = g_hif_sdio_ops.bt_hi_read_word(RG_AON_A60);
                wf_time = g_hif_sdio_ops.bt_hi_read_word(RG_AON_A61);
            }
            if (INTF_TYPE_IS_PCIE(amlbt_if_type))
            {
                bt_time = aml_pci_read_for_bt(AML_ADDR_AON, RG_AON_A60);
                wf_time = aml_pci_read_for_bt(AML_ADDR_AON, RG_AON_A61);
            }
            wf_percent = (100 * wf_time) / (bt_time + wf_time);
            if (copy_to_user((int __user *)arg, &wf_percent, sizeof(int)) != 0)
            {
                printk("IOCTL_GET_WF_TIME_PERCENT copy error\n");
                return -EFAULT;
            }
            printk("IOCTL_GET_WF_TIME_PERCENT %d\n", wf_percent);
        }
        break;
        case IOCTL_EXIT_USER:
        {
            if (copy_from_user(&coex_time, (int __user *)arg, sizeof(int)) != 0)
            {
                printk("IOCTL_EXIT_USER copy error\n");
                return -EFAULT;
            }
            printk("IOCTL_EXIT_USER %#x\n", coex_time);
            if (INTF_TYPE_IS_SDIO(amlbt_if_type))
            {
                reg_value = g_hif_sdio_ops.bt_hi_read_word(RG_AON_A59);
                //printk("r RG_AON_A59 %#x", reg_value);
                reg_value &= (coex_time << 31);
                printk("w RG_AON_A59 %#x %#x", reg_value, coex_time);
                g_hif_sdio_ops.bt_hi_write_word(RG_AON_A59, reg_value);
            }
            if (INTF_TYPE_IS_PCIE(amlbt_if_type))
            {
                reg_value = aml_pci_read_for_bt(AML_ADDR_AON, RG_AON_A59);
                //printk("r RG_AON_A59 %#x", reg_value);
                reg_value &= (coex_time << 31);
                printk("w RG_AON_A59 %#x %#x", reg_value, coex_time);
                aml_pci_write_for_bt(reg_value, AML_ADDR_AON, RG_AON_A59);
            }

        }
        default:
            break;
    }
    return 0;
}

static long btusbchr_ioctl(struct file* filp, unsigned int cmd, unsigned long arg)
{
    unsigned long coex_time = 0;
    unsigned int reg_value = 0;
    BTD("arg value %ld", arg);
    BTD("cmd type=%c\t nr=%d\t dir=%d\t size=%d\n", _IOC_TYPE(cmd), _IOC_NR(cmd), _IOC_DIR(cmd), _IOC_SIZE(cmd));
    BTD("cmd value %ld", cmd);
    switch (cmd)
    {
        case IOCTL_GET_BT_DOWNLOAD_STATUS:
        {
            if (copy_to_user((unsigned char __user *)arg, &download_fw, sizeof(unsigned char)) != 0)
            {
                printk("IOCTL_GET_BT_DOWNLOAD_STATUS copy error\n");
                return -EFAULT;
            }
            printk("IOCTL_GET_BT_DOWNLOAD_STATUS %#x\n", coex_time);
        }
        break;
        case IOCTL_SET_BT_RESET:
        {
            printk("IOCTL_SET_BT_RESET\n");
            download_fw = 0;
            download_end = 0;
            download_flag = 0;
            amlbt_usb_fifo_deinit();
            amlbt_buff_init();
            if (FAMILY_TYPE_IS_W2L(amlbt_if_type))
            {
                aml_15p4_buff_init();
            }
            memset(g_lib_cmd_buff, 0, CMD_FIFO_SIZE);
            g_lib_cmd_fifo = gdsl_fifo_init(CMD_FIFO_SIZE, g_lib_cmd_buff);
        }
        break;
        case IOCTL_SET_BT_COEX_TIME:
        {
            if (!download_fw)
            {
                printk("IOCTL_SET_BT_COEX_TIME bt firmware state error\n");
                return -EFAULT;
            }
            if (copy_from_user(&coex_time, (int __user *)arg, sizeof(int)) != 0)
            {
                printk("IOCTL_SET_BT_COEX_TIME copy error\n");
                return -EFAULT;
            }
            printk("IOCTL_SET_BT_COEX_TIME %#x\n", coex_time);
            reg_value = amlbt_usb_read_word(RG_AON_A57, BT_EP);
            printk("w RG_AON_A57 %#x %#x", reg_value, coex_time);
            amlbt_usb_write_word(RG_AON_A57, coex_time, BT_EP);
        }
        break;
        case IOCTL_SET_WIFI_COEX_TIME:
        {
            if (!download_fw)
            {
                printk("IOCTL_SET_WIFI_COEX_TIME bt firmware state error\n");
                return -EFAULT;
            }
            if (copy_from_user(&coex_time, (int __user *)arg, sizeof(int)) != 0)
            {
                printk("IOCTL_SET_WIFI_COEX_TIME copy error\n");
                return -EFAULT;
            }
            printk("IOCTL_SET_WIFI_COEX_TIME %#x\n", coex_time);
            reg_value = amlbt_usb_read_word(RG_AON_A58, BT_EP);
            printk("w RG_AON_A58 %#x %#x", reg_value, coex_time);
            amlbt_usb_write_word(RG_AON_A58, coex_time, BT_EP);
        }
        break;
        case IOCTL_SET_WIFI_MAX_DURATION:
        {
            if (!download_fw)
            {
                printk("IOCTL_SET_WIFI_MAX_DURATION bt firmware state error\n");
                return -EFAULT;
            }
            if (copy_from_user(&coex_time, (int __user *)arg, sizeof(int)) != 0)
            {
                printk("IOCTL_SET_WIFI_MAX_DURATION copy error\n");
                return -EFAULT;
            }
            printk("IOCTL_SET_WIFI_MAX_DURATION %#x\n", coex_time);
            reg_value = amlbt_usb_read_word(RG_AON_A59, BT_EP);
            BTD("r RG_AON_A59 %#x", reg_value);
            reg_value = (coex_time & 0xffffff) | (reg_value & 0xff000000);
            printk("w RG_AON_A59 %#x %#x", reg_value, coex_time);
            amlbt_usb_write_word(RG_AON_A59, reg_value, BT_EP);
        }
        break;
        case IOCTL_GET_BT_COEX_TIME:
        {
            if (!download_fw)
            {
                printk("IOCTL_GET_BT_COEX_TIME bt firmware state error\n");
                return -EFAULT;
            }
            coex_time = amlbt_usb_read_word(RG_AON_A60, BT_EP);
            if (copy_to_user((int __user *)arg, &coex_time, sizeof(int)) != 0)
            {
                printk("IOCTL_GET_BT_COEX_TIME copy error\n");
                return -EFAULT;
            }
            printk("IOCTL_GET_BT_COEX_TIME %#x\n", coex_time);
        }
        break;
        case IOCTL_GET_WIFI_COEX_TIME:
        {
            if (!download_fw)
            {
                printk("IOCTL_GET_WIFI_COEX_TIME bt firmware state error\n");
                return -EFAULT;
            }
            coex_time = amlbt_usb_read_word(RG_AON_A61, BT_EP);
            if (copy_to_user((int __user *)arg, &coex_time, sizeof(int)) != 0)
            {
                printk("IOCTL_GET_WIFI_COEX_TIME copy error\n");
                return -EFAULT;
            }
            printk("IOCTL_GET_WIFI_COEX_TIME %#x\n", coex_time);
        }
        break;
        case IOCTL_SET_BT_TX_POWER:
        {
            if (!download_fw)
            {
                printk("IOCTL_SET_BT_TX_POWER bt firmware state error\n");
                return -EFAULT;
            }
            if (copy_from_user(&coex_time, (int __user *)arg, sizeof(int)) != 0)
            {
                printk("IOCTL_SET_BT_TX_POWER copy error\n");
                return -EFAULT;
            }
            printk("IOCTL_SET_BT_TX_POWER %#x\n", coex_time);
            if (coex_time >= 0 && coex_time <= 106)
            {
                reg_value = amlbt_usb_read_word(RG_TX_AGAIN, BT_EP);
                BTD("r RG_TX_AGAIN %#x", reg_value);
                reg_value = (again[0] | (reg_value & 0xfffff000));
                printk("w RG_TX_AGAIN %#x %#x", reg_value, again[0]);
                amlbt_usb_write_word(RG_TX_AGAIN, reg_value, BT_EP);
            }
            else
            {
                reg_value = amlbt_usb_read_word(RG_TX_AGAIN, BT_EP);
                BTD("r RG_TX_AGAIN %#x", reg_value);
                reg_value = (again[1] | (reg_value & 0xfffff000));
                printk("w RG_TX_AGAIN %#x %#x", reg_value, again[1]);
                amlbt_usb_write_word(RG_TX_AGAIN, reg_value, BT_EP);
            }
            reg_value = amlbt_usb_read_word(RG_TX_DGAIN, BT_EP);
            BTD("r RG_TX_AGAIN %#x", reg_value);
            reg_value = (dgain[coex_time] << 24) | (reg_value & 0x00ffffff);
            printk("w RG_TX_AGAIN %#x %#x", reg_value, dgain[coex_time]);
            amlbt_usb_write_word(RG_TX_DGAIN, reg_value, BT_EP);

            reg_value = amlbt_usb_read_word(RG_AON_A59, BT_EP);
            reg_value |= (1 << 31);
            printk("w RG_TX_DGAIN %#x", reg_value);
            amlbt_usb_write_word(RG_AON_A59, reg_value, BT_EP);
        }
        break;
        case IOCTL_GET_RF_ANTENNA:
        {
            if (!download_fw)
            {
                printk("IOCTL_GET_RF_ANTENNA bt firmware state error\n");
                return -EFAULT;
            }
            reg_value = amlbt_usb_read_word(REG_PMU_POWER_CFG, BT_EP);
            coex_time = ((reg_value >> BIT_RF_NUM) & 0x03);
            if (copy_to_user((int __user *)arg, &coex_time, sizeof(int)) != 0)
            {
                printk("IOCTL_GET_RF_ANTENNA copy error\n");
                return -EFAULT;
            }
            printk("IOCTL_GET_RF_ANTENNA %#x\n", coex_time);
        }
        break;
        case IOCTL_GET_BT_RSSI:
        {
            if (!download_fw)
            {
                printk("IOCTL_GET_BT_RSSI bt firmware state error\n");
                return -EFAULT;
            }
            reg_value = amlbt_usb_read_word(RG_AON_A59, BT_EP);
            coex_time = ((reg_value >> 24) & 0x7f);
            if (copy_to_user((int __user *)arg, &coex_time, sizeof(int)) != 0)
            {
                printk("IOCTL_GET_BT_RSSI copy error\n");
                return -EFAULT;
            }
            printk("IOCTL_GET_BT_RSSI %#x\n", coex_time);
        }
        break;
        case IOCTL_GET_BT_TIME_PERCENT:
        {
            int bt_time = 0;
            int wf_time = 0;
            int bt_percent = 0;
            bt_time = amlbt_usb_read_word(RG_AON_A60, BT_EP);
            wf_time = amlbt_usb_read_word(RG_AON_A61, BT_EP);
            bt_percent = (bt_time * 100) / (bt_time + wf_time);
            if (copy_to_user((int __user *)arg, &bt_percent, sizeof(int)) != 0)
            {
                printk("IOCTL_GET_BT_TIME_PERCENT copy error\n");
                return -EFAULT;
            }
            printk("IOCTL_GET_BT_TIME_PERCENT %d\n", bt_percent);
        }
        break;
        case IOCTL_GET_WF_TIME_PERCENT:
        {
            int bt_time = 0;
            int wf_time = 0;
            int wf_percent = 0;
            bt_time = amlbt_usb_read_word(RG_AON_A60, BT_EP);
            wf_time = amlbt_usb_read_word(RG_AON_A61, BT_EP);
            wf_percent = (100 * wf_time) / (bt_time + wf_time);
            if (copy_to_user((int __user *)arg, &wf_percent, sizeof(int)) != 0)
            {
                printk("IOCTL_GET_WF_TIME_PERCENT copy error\n");
                return -EFAULT;
            }
            printk("IOCTL_GET_WF_TIME_PERCENT %d\n", wf_percent);
        }
        break;
        case IOCTL_EXIT_USER:
        {
            if (copy_from_user(&coex_time, (int __user *)arg, sizeof(int)) != 0)
            {
                printk("IOCTL_EXIT_USER copy error\n");
                return -EFAULT;
            }
            printk("IOCTL_EXIT_USER %#x\n", coex_time);
            reg_value = amlbt_usb_read_word(RG_AON_A59, BT_EP);
            //printk("r RG_AON_A59 %#x", reg_value);
            reg_value &= (coex_time << 31);
            printk("w RG_AON_A59 %#x %#x", reg_value, coex_time);
            amlbt_usb_write_word(RG_AON_A59, reg_value, BT_EP);
        }
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
    .poll = btchr_poll,
    //.unlocked_ioctl = btchr_ioctl,
    .compat_ioctl = btusbchr_ioctl,
};

static int amlbt_usb_char_init(void)
{
    int res = 0;
    struct device *dev;

    printk("%s\n", __func__);

    //	init_timer(&r_timer);

    bt_char_class = class_create(THIS_MODULE, AML_BT_CHAR_DEVICE_NAME);
    if (IS_ERR(bt_char_class))
    {
        printk("%s:Failed to create bt char class\n", __func__);
        return PTR_ERR(bt_char_class);
    }

    res = alloc_chrdev_region(&bt_devid, 0, 1, AML_BT_CHAR_DEVICE_NAME);
    if (res < 0)
    {
        printk("%s:Failed to allocate bt char device\n", __func__);
        goto err_alloc;
    }

    dev = device_create(bt_char_class, NULL, bt_devid, NULL, AML_BT_CHAR_DEVICE_NAME);
    if (IS_ERR(dev))
    {
        printk("%s:Failed to create bt char device\n", __func__);
        res = PTR_ERR(dev);
        goto err_create;
    }

    cdev_init(&bt_char_dev, &amlbt_usb_fops);
    res = cdev_add(&bt_char_dev, bt_devid, 1);
    if (res < 0)
    {
        printk("%s:Failed to add bt char device\n", __func__);
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
    memset(BT_fwICCM, 0x55, bt_iccm_size);
    memset(BT_fwDCCM, 0xaa, bt_dccm_size);
    //amlbt_usb_download_firmware();
    amlbt_usb_write_sram(BT_fwICCM, (unsigned char *)(unsigned long)(0x00300000 + 256 * 1024), 20 * 1024,
                                       BT_EP);
    amlbt_usb_read_sram(BT_fwDCCM, (unsigned char *)(unsigned long)(0x00300000 + 256 * 1024), 20 * 1024,
                                      BT_EP);

    if (!memcmp(BT_fwICCM, BT_fwDCCM, 20 * 1024))
    {
        printk("check ok!\n");
    }
    else
    {
        printk("check err : %#x,%#x,%#x,%#x \n", BT_fwDCCM[0], BT_fwDCCM[1], BT_fwDCCM[2], BT_fwDCCM[3]);
    }
    memset(BT_fwDCCM, 0xaa, bt_dccm_size);
    amlbt_usb_write_sram(BT_fwICCM, (unsigned char *)(unsigned long)(0x00400000), 20 * 1024, BT_EP);
    amlbt_usb_read_sram(BT_fwDCCM, (unsigned char *)(unsigned long)(0x00400000), 20 * 1024, BT_EP);
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

void amlbt_sdio_download_firmware(void)
{
    unsigned int offset = 0;
    unsigned int remain_len = 0;
    unsigned int iccm_base_addr = BT_ICCM_AHB_BASE + bt_iccm_rom_size;
    unsigned int dccm_base_addr = BT_DCCM_AHB_BASE;
    unsigned int download_size = 0;
    unsigned int write_size = 4096;
    uint32_t fw_iccmLen = 0;
    uint8_t *fw_iccmBuf = NULL;
    uint32_t fw_dccmLen = 0;
    uint8_t *fw_dccmBuf = NULL;
    //uint8_t check_buf[512] = {0};

    fw_iccmLen = bt_iccm_size;
    fw_iccmBuf = BT_fwICCM;
    fw_dccmLen = bt_dccm_size;
    fw_dccmBuf = BT_fwDCCM;

    download_size = fw_iccmLen;

    //to do download bt fw
    BTD("amlbt_sdio_download_firmware:iccm size %#x\n", download_size);

    remain_len = (download_size - offset);

    while (offset < download_size)
    {
        if (remain_len < write_size)
        {
            BTD("bt_usb_download_firmware iccm1 offset %#x, addr %#x\n", offset, iccm_base_addr);
            g_hif_sdio_ops.hi_random_ram_write((unsigned char *)&fw_iccmBuf[offset], (unsigned char *)(unsigned long)iccm_base_addr, remain_len);
            //g_hif_sdio_ops.hi_random_ram_read(check_buf, (unsigned char *)(unsigned long)iccm_base_addr, remain_len);
            //if (memcmp(check_buf, &fw_iccmBuf[offset], remain_len))
            //{
                //BTI("Firmware iccm check2 error! offset %#x\n", offset);
            //}
            offset += remain_len;
            iccm_base_addr += remain_len;
            BTD("amlbt_sdio_download_firmware iccm1 offset %#x, write_len %#x\n", offset, remain_len);
        }
        else
        {
            BTD("amlbt_sdio_download_firmware iccm2 offset %#x, write_len %#x, addr %#x\n", offset, write_size, iccm_base_addr);
            g_hif_sdio_ops.hi_random_ram_write((unsigned char *)&fw_iccmBuf[offset], (unsigned char *)(unsigned long)iccm_base_addr, write_size);
            //g_hif_sdio_ops.hi_random_ram_read(check_buf, (unsigned char *)(unsigned long)iccm_base_addr, write_size);
            //if (memcmp(check_buf, &fw_iccmBuf[offset], write_size))
            //{
            //    BTI("Firmware iccm check error! offset %#x\n", offset);
            //}
            offset += write_size;
            remain_len -= write_size;
            iccm_base_addr += write_size;
        }
        BTD("amlbt_sdio_download_firmware iccm remain_len %#x\n", remain_len);
    }

    download_size = fw_dccmLen;

    //to do download bt fw
    BTD("amlbt_sdio_download_firmware:dccm size %#x\n", download_size);
    offset = 0;
    remain_len = download_size;
    while (offset < download_size)
    {
        if (remain_len < write_size)
        {
            BTD("bt_usb_download_firmware dccm1 offset %#x, addr %#x\n", offset, dccm_base_addr);
            g_hif_sdio_ops.hi_random_ram_write((unsigned char *)&fw_dccmBuf[offset], (unsigned char *)(unsigned long)dccm_base_addr, remain_len);
            //g_hif_sdio_ops.hi_random_ram_read(check_buf, (unsigned char *)(unsigned long)dccm_base_addr, remain_len);
            //if (memcmp(check_buf, &fw_dccmBuf[offset], remain_len))
            //{
            //    BTI("Firmware dccm check2 error! offset %#x\n", offset);
            //}
            offset += remain_len;
            dccm_base_addr += remain_len;
            BTD("amlbt_sdio_download_firmware dccm1 offset %#x, write_len %#x\n", offset, remain_len);
        }
        else
        {
            BTD("amlbt_sdio_download_firmware dccm2 offset %#x, write_len %#x, addr%#x\n", offset, write_size, dccm_base_addr);
            g_hif_sdio_ops.hi_random_ram_write((unsigned char *)&fw_dccmBuf[offset], (unsigned char *)(unsigned long)dccm_base_addr, write_size);
            //g_hif_sdio_ops.hi_random_ram_read(check_buf, (unsigned char *)(unsigned long)dccm_base_addr, write_size);
            //if (memcmp(check_buf, &fw_dccmBuf[offset], write_size))
            //{
            //    BTI("Firmware dccm check error! offset %#x\n", offset);
            //}
            offset += write_size;
            remain_len -= write_size;
            dccm_base_addr += write_size;
        }

        BTD("amlbt_sdio_download_firmware dccm remain_len %#x \n", remain_len);
    }

}


static int amlbt_sdio_fops_open(struct inode *inode, struct file *file)
{
    printk("%s bt opened rf num:%#x\n", __func__, rf_num);
    if (sdio_buf == NULL)
    {
        sdio_buf = kzalloc(HCI_MAX_FRAME_SIZE, GFP_DMA|GFP_ATOMIC);
    }

    printk("%s, %#x \n", __func__, AML_BT_VERSION);
    close_state = 0;
    download_fw = 0;
    download_end = 0;
    download_flag = 0;
    fw_cmd_w = 0;
    fw_cmd_r = 0;

    amlbt_usb_ram_init();

    g_lib_cmd_fifo = gdsl_fifo_init(sizeof(sdio_cmd_buff), sdio_cmd_buff);
    //rf_num = ((g_hif_sdio_ops.bt_hi_read_word(REG_PMU_POWER_CFG) >> BIT_RF_NUM) & 0x03);
    //printk("%s set rf num %#x", __func__, rf_num);
    init_completion(&download_completion);
    return nonseekable_open(inode, file);
}

static int amlbt_sdio_fops_close(struct inode *inode, struct file *file)
{
    printk("%s BT closed\n", __func__);
    download_fw = 0;
    gdsl_fifo_deinit(g_lib_cmd_fifo);
    g_lib_cmd_fifo = 0;
    download_end = 0;
    download_flag = 0;
    close_state = 0;
    if (BT_fwDCCM)
    {
        vfree(BT_fwDCCM);
        BT_fwDCCM = NULL;
    }
    if (BT_fwICCM)
    {
        vfree(BT_fwICCM);
        BT_fwICCM = NULL;
    }

    return 0;
}

static ssize_t amlbt_sdio_char_write(struct file *file_p,
                                     const char __user *buf_p,
                                     size_t count,
                                     loff_t *pos_p)
{
    unsigned int n = 0;
    unsigned int size = sizeof(cmd) / sizeof(cmd[0]);
    unsigned char len  = 0;
    unsigned int offset = 0;
    unsigned int reg_value = 0;
    //unsigned char buf[16] = {0};

    BTD("W_FW:%#x\n", count);

    memset(sdio_buf, 0, HCI_MAX_FRAME_SIZE);

    if (copy_from_user(sdio_buf, buf_p, count))
    {
        printk("%s: Failed to get data from user space\n", __func__);
        return -EFAULT;
    }

    offset = ((sdio_buf[3] << 24) | (sdio_buf[2] << 16) | (sdio_buf[1] << 8) | sdio_buf[0]);
    reg_value = ((sdio_buf[7] << 24) | (sdio_buf[6] << 16) | (sdio_buf[5] << 8) | sdio_buf[4]);

    if (offset == REG_PMU_POWER_CFG)
    {
        if (FAMILY_TYPE_IS_W2(amlbt_if_type) || FAMILY_TYPE_IS_W2L(amlbt_if_type))
        {
            rf_num = reg_value >> BIT_RF_NUM;
            printk("BT set rf succeed!\n");
            printk("%s set rf num %#x", __func__, rf_num);
        }
        return count;
    }

    for (n = 0; n < size; n++)
    {
        if (!memcmp(sdio_buf, cmd[n], 2))
        {
            cmd_index = n;
            break;
        }
    }

    if (n == size)
    {
        printk("CMD_I:%#x, %d\n", cmd_index, count);
        for (n = 0; n < 11; n++)
        {
            printk("%#x|", sdio_buf[n]);
        }
        printk("---------------cmd error!------------");
        return -EINVAL;
    }

    gdsl_fifo_copy_data(g_lib_cmd_fifo, sdio_buf, 2);

    if (sdio_buf[0] == 0xf3 && sdio_buf[1] == 0xfe)   //download fw
    {
        len = count - 7;
        offset = ((sdio_buf[6] << 24) | (sdio_buf[5] << 16) | (sdio_buf[4] << 8) | sdio_buf[3]);
        //printk("%#x,%#x,%#x\n", len, offset, dw_state);
        //printk("%#x,%#x,%#x,%#x\n", sdio_buf[8], sdio_buf[9], sdio_buf[10], sdio_buf[11]);
        if (offset == bt_iccm_rom_size)
        {
            printk("iccm start dw_state %#x\n", dw_state);
#if 0
            if (!download_flag)
            {
                if (FAMILY_TYPE_IS_W2(amlbt_if_type))
                {
                    //printk("w2 usb write first reg\n");
                    g_hif_sdio_ops.bt_hi_write_word(0xf03050, 0);
                    //printk("w2 usb write first reg end\n");
                }
            }
#endif
            iccm_base_addr = 0;
            dccm_base_addr = 0;
            dw_state = 0;
            if (BT_fwICCM == NULL)
            {
                BT_fwICCM = vmalloc(bt_iccm_size);
            }
            if (BT_fwDCCM == NULL)
            {
                BT_fwDCCM = vmalloc(bt_dccm_size);
            }
            printk("amlbt_sdio_char_write BT_fwICCM %#x, BT_fwDCCM %#x \n", (unsigned long)BT_fwICCM, (unsigned long)BT_fwDCCM);
            if (BT_fwICCM == NULL || BT_fwDCCM == NULL)
            {
                printk("amlbt_usb_char_write_fw vmalloc err!!\n");
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
            printk("dccm start\n");
            dw_state = 1;
        }

        if (dw_state == 0)
        {
            BTD("len %#x, %#x \n", len, iccm_base_addr);
            memcpy(&BT_fwICCM[iccm_base_addr], &sdio_buf[7], len);
            //g_hif_sdio_ops.bt_hi_write_sram((unsigned char *)&sdio_buf[8], (unsigned char *)(unsigned long)(BT_ICCM_AHB_BASE+offset), len);
            //memset(buf, 0, sizeof(buf));
            //g_hif_sdio_ops.bt_hi_read_sram(buf, (unsigned char *)(unsigned long)(BT_ICCM_AHB_BASE+offset), sizeof(buf));
            //memcpy(buf, &sdio_buf[8], sizeof(buf));

           // BTD("ICCM:%#x|%#x|%#x|%#x|%#x|%#x|%#x|%#x|%#x|%#x|%#x|%#x|%#x|%#x|%#x|%#x \n", buf[0],
             //   buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7],buf[8],buf[9],buf[10],buf[11],buf[12],
               // buf[13],buf[14],buf[15]);
            iccm_base_addr += len;
            //printk("%#x \n", offset);
        }
        else
        {
            BTD("dccm_base_addr %#x \n", dccm_base_addr);
            memcpy(&BT_fwDCCM[dccm_base_addr], &sdio_buf[7], len);
            //g_hif_sdio_ops.bt_hi_write_sram((unsigned char *)&sdio_buf[8],
            //    (unsigned char *)(unsigned long)(BT_DCCM_AHB_BASE+offset-DCCM_RAM_BASE),
            //    len);
            //memset(buf, 0, sizeof(buf));
            //g_hif_sdio_ops.bt_hi_read_sram(buf, (unsigned char *)(unsigned long)(BT_DCCM_AHB_BASE+offset-DCCM_RAM_BASE), sizeof(buf));
            //memcpy(buf, &sdio_buf[8], sizeof(buf));
            BTD("len %#x, %#x \n", len, dccm_base_addr);
            //BTD("DCCM:%#x|%#x|%#x|%#x|%#x|%#x|%#x|%#x|%#x|%#x|%#x|%#x|%#x|%#x|%#x|%#x \n", buf[0],
              //  buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7],buf[8],buf[9],buf[10],buf[11],buf[12],
               // buf[13],buf[14],buf[15]);
            dccm_base_addr += len;

            if (dccm_base_addr >= 0x20000)
            {
                amlbt_sdio_download_firmware();
                printk("download finished!\n");
                vfree(BT_fwICCM);
                vfree(BT_fwDCCM);
                BT_fwICCM = NULL;
                BT_fwDCCM = NULL;
                printk("W E %#x,%#x\n", iccm_base_addr, dccm_base_addr);
                download_end = 1;
            }
        }
    }

    fw_cmd_w++;
    complete(&download_completion);
    return count;
}


static ssize_t amlbt_sdio_char_read(struct file *file_p,
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
            wait_for_completion(&download_completion);
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
                printk("%s end \n", __func__);
            }
        }
        break;
    }
    return count;
}

const struct file_operations amlbt_sdio_fops =
{
    .open       = amlbt_sdio_fops_open,
    .release    = amlbt_sdio_fops_close,
    .write      = amlbt_sdio_char_write,
    .read      = amlbt_sdio_char_read,
    .compat_ioctl = btuartchr_ioctl,
    .poll       = NULL,
    //.unlocked_ioctl = NULL,
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
        printk("fail to allocate chrdev\n");
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
        printk("class create fail, error code(%ld)\n",
               PTR_ERR(amlbt_sdio_class));
        goto err1;
    }

    amlbt_sdio_dev = device_create(amlbt_sdio_class, NULL, devID, NULL, AML_BT_NOTE);
    if (IS_ERR(amlbt_sdio_dev))
    {
        printk("device create fail, error code(%ld)\n",
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
    printk("++++++sdio bt driver insmod start.++++++\n");
    ret = amlbt_sdio_init();
    if (ret)
    {
        printk("%s: amlbt_sdio_init failed!\n", __func__);
        return ret;
    }
    printk("------sdio bt driver insmod end.------\n");
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

    printk("%s driver removed.\n", AML_BT_NOTE);
    return 0;
}

static void amlbt_sdio_rmmod(void)
{
    printk("++++++sdio bt driver rmmod start++++++\n");
    amlbt_sdio_exit();
    printk("------sdio bt driver rmmod end------\n");
}

static void bt_earlysuspend(struct early_suspend *h)
{
    printk("%s \n", __func__);
}

static void bt_lateresume(struct early_suspend *h)
{
    unsigned int reg_value = 0;

    if ((FAMILY_TYPE_IS_W2(amlbt_if_type) || FAMILY_TYPE_IS_W2L(amlbt_if_type)) && INTF_TYPE_IS_SDIO(amlbt_if_type))
    {
        reg_value = g_hif_sdio_ops.bt_hi_read_word(RG_AON_A52);
        printk("%s RG_AON_A52:%#x\n", __func__, reg_value);
        reg_value &= ~(1 << 26);
        g_hif_sdio_ops.bt_hi_write_word(RG_AON_A52, reg_value);
        printk("RG_AON_A52:%#x", g_hif_sdio_ops.bt_hi_read_word(RG_AON_A52));
    }
    if ((FAMILY_TYPE_IS_W2(amlbt_if_type) || FAMILY_TYPE_IS_W2L(amlbt_if_type)) && INTF_TYPE_IS_PCIE(amlbt_if_type))
    {
        reg_value = aml_pci_read_for_bt(AML_ADDR_AON, RG_AON_A52);
        printk("%s RG_AON_A52:%#x\n", __func__, reg_value);
        reg_value &= ~(1 << 26);
        aml_pci_write_for_bt(reg_value, AML_ADDR_AON, RG_AON_A52);
        printk("RG_AON_A52:%#x", aml_pci_read_for_bt(AML_ADDR_AON, RG_AON_A52));
    }
}

static int amlbt_sdio_probe(struct platform_device *dev)
{
    printk("%s \n", __func__);
    amlbt_early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
    amlbt_early_suspend.suspend = bt_earlysuspend;
    amlbt_early_suspend.resume = bt_lateresume;
    amlbt_early_suspend.param = dev;
    register_early_suspend(&amlbt_early_suspend);
    amlbt_sdio_insmod();
    return 0;
}

static int amlbt_sdio_remove(struct platform_device *dev)
{
    printk("%s \n", __func__);
    unregister_early_suspend(&amlbt_early_suspend);
    amlbt_sdio_rmmod();
    kfree(sdio_buf);
    sdio_buf = NULL;
    return 0;
}

static int amlbt_sdio_suspend(struct platform_device *dev, pm_message_t state)
{
    unsigned int reg_value = 0;
    if ((FAMILY_TYPE_IS_W2(amlbt_if_type) || FAMILY_TYPE_IS_W2L(amlbt_if_type)) && INTF_TYPE_IS_SDIO(amlbt_if_type))
    {
        reg_value = g_hif_sdio_ops.bt_hi_read_word(RG_AON_A52);
        printk("%s RG_AON_A52:%#x\n", __func__, reg_value);
        reg_value |= (1 << 26);
        g_hif_sdio_ops.bt_hi_write_word(RG_AON_A52, reg_value);
        printk("RG_AON_A52:%#x", g_hif_sdio_ops.bt_hi_read_word(RG_AON_A52));
    }
    if ((FAMILY_TYPE_IS_W2(amlbt_if_type) || FAMILY_TYPE_IS_W2L(amlbt_if_type)) && INTF_TYPE_IS_PCIE(amlbt_if_type))
    {
        reg_value = aml_pci_read_for_bt(AML_ADDR_AON, RG_AON_A52);
        printk("%s RG_AON_A52:%#x\n", __func__, reg_value);
        reg_value |= (1 << 26);
        aml_pci_write_for_bt(reg_value, AML_ADDR_AON, RG_AON_A52);
        printk("RG_AON_A52:%#x", aml_pci_read_for_bt(AML_ADDR_AON, RG_AON_A52));
    }
    printk("%s \n", __func__);
    return 0;
}

static int amlbt_sdio_resume(struct platform_device *dev)
{
#if 0
    unsigned int reg_value = 0;
    if ((get_resume_method() != RESUME_RTC_S) && (get_resume_method() != RESUME_RTC_C))
    {
        if (FAMILY_TYPE_IS_W2(amlbt_if_type) && INTF_TYPE_IS_SDIO(amlbt_if_type))
        {
            reg_value = g_hif_sdio_ops.bt_hi_read_word(RG_AON_A52);
            printk("%s RG_AON_A52:%#x\n", __func__, reg_value);
            reg_value &= ~(1 << 26);
            g_hif_sdio_ops.bt_hi_write_word(RG_AON_A52, reg_value);
            printk("RG_AON_A52:%#x", g_hif_sdio_ops.bt_hi_read_word(RG_AON_A52));
        }
        if (FAMILY_TYPE_IS_W2(amlbt_if_type) && INTF_TYPE_IS_PCIE(amlbt_if_type))
        {
            reg_value = aml_pci_read_for_bt(AML_ADDR_AON, RG_AON_A52);
            printk("%s RG_AON_A52:%#x\n", __func__, reg_value);
            reg_value &= ~(1 << 26);
            aml_pci_write_for_bt(reg_value, AML_ADDR_AON, RG_AON_A52);
            printk("RG_AON_A52:%#x", aml_pci_read_for_bt(AML_ADDR_AON, RG_AON_A52));
        }
    }
#endif
    printk("%s \n", __func__);
    return 0;
}

static void amlbt_sdio_shutdown(struct platform_device *dev)
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

static void amlbt_usb_insmod(void)
{
    // int ret = 0;
    printk("BTAML version:%#x\n", AML_BT_VERSION);
    printk("++++++usb bt driver insmod start.++++++\n");
    printk("------usb bt driver insmod end.------\n");

    //return ret;
}

static void amlbt_usb_rmmod(void)
{
    //if (amlbt_poweron == AML_SDIO_EN)
    {
        printk("++++++usb bt driver rmmod start++++++\n");
        printk("------usb bt driver rmmod end------\n");
    }
}
static void btusb_earlysuspend(struct early_suspend *h)
{
    printk("%s \n", __func__);
}

static void btusb_lateresume(struct early_suspend *h)
{
    if (FAMILY_TYPE_IS_W2(amlbt_if_type) || FAMILY_TYPE_IS_W2L(amlbt_if_type))
    {
        unsigned int reg_value = amlbt_usb_read_word(RG_AON_A52, BT_EP);
        printk("%s RG_AON_A52:%#x\n", __func__, reg_value);
        reg_value &= ~(1 << 26);
        amlbt_usb_write_word(RG_AON_A52, reg_value, BT_EP);
        printk("RG_AON_A52:%#x\n", amlbt_usb_read_word(RG_AON_A52, BT_EP));
    }
    suspend_value = 0;
}

static int amlbt_usb_probe(struct platform_device *dev)
{
    int err = 0;
    amlbt_usb_insmod();   //load
    //g_auc_hif_ops.bt_hi_write_word((unsigned int)0x00a0d0e4, 0x8000007f);
    //printk("%s, %#x", __func__, g_auc_hif_ops.bt_hi_read_word(0x00a0d0e4));
    printk("%s \n", __func__);
    amlbt_early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
    amlbt_early_suspend.suspend = btusb_earlysuspend;
    amlbt_early_suspend.resume = btusb_lateresume;
    amlbt_early_suspend.param = dev;
    register_early_suspend(&amlbt_early_suspend);
    err = amlbt_usb_char_init();
    if (err < 0)
    {
        /* usb register will go on, even bt char register failed */
        BTD("%s:Failed to register usb char device interfaces\n", __func__);
    }
    amlbt_usb_ram_init();
    download_fw = 0;
    download_end = 0;
    download_flag = 0;
    fw_cmd_w = 0;
    fw_cmd_r = 0;
    amlbt_buff_init();
    if (FAMILY_TYPE_IS_W2L(amlbt_if_type))
    {
        aml_15p4_buff_init();
    }
    memset(g_lib_cmd_buff, 0, CMD_FIFO_SIZE);
    g_lib_cmd_fifo = gdsl_fifo_init(CMD_FIFO_SIZE, g_lib_cmd_buff);

#if AML_BT_ROM_CHECK
    amlbt_usb_rom_check();
#endif
    printk("%s, end \n", __func__);
    return err;
}

static int amlbt_usb_remove(struct platform_device *dev)
{
    printk("%s\n", __func__);

    amlbt_usb_rmmod();    //unload

    close_state = 2;
    if (g_fw_data_fifo == 0)
    {
        amlbt_usb_char_deinit();
    }
    gdsl_fifo_deinit(g_lib_cmd_fifo);
    g_lib_cmd_fifo = 0;
    amlbt_buff_deinit();
    if (FAMILY_TYPE_IS_W2L(amlbt_if_type))
    {
        aml_15p4_buff_deinit();
    }
    download_fw = 0;
    download_end = 0;
    download_flag = 0;
    msleep(500);
    printk("%s end\n", __func__);
    return 0;
}

static int amlbt_usb_suspend(struct platform_device *dev, pm_message_t state)
{
    suspend_value = 1;
    if (FAMILY_TYPE_IS_W2(amlbt_if_type) || FAMILY_TYPE_IS_W2L(amlbt_if_type))
    {
        unsigned int reg_value = amlbt_usb_read_word(RG_AON_A52, BT_EP);
        printk("%s RG_AON_A52:%#x\n", __func__, reg_value);
        reg_value |= (1 << 26);
        amlbt_usb_write_word(RG_AON_A52, reg_value, BT_EP);
        printk("RG_AON_A52:%#x", amlbt_usb_read_word(RG_AON_A52, BT_EP));
    }
    printk("%s \n", __func__);
    return 0;
}

static int amlbt_usb_resume(struct platform_device *dev)
{
    printk("%s\n", __func__);
    //msleep(1500);
#if 0
    if ((get_resume_method() != RESUME_RTC_S) && (get_resume_method() != RESUME_RTC_C))
    {
        if (FAMILY_TYPE_IS_W2(amlbt_if_type))
        {
            unsigned int reg_value = amlbt_usb_read_word(RG_AON_A52, BT_EP);
            printk("%s RG_AON_A52:%#x\n", __func__, reg_value);
            reg_value &= ~(1 << 26);
            amlbt_usb_write_word(RG_AON_A52, reg_value, BT_EP);
            printk("RG_AON_A52:%#x", amlbt_usb_read_word(RG_AON_A52, BT_EP));
        }
    }
#endif
    return 0;
}

static void amlbt_usb_shutdown(struct platform_device *dev)
{
    shutdown_value = 1;
    if ((FAMILY_TYPE_IS_W2(amlbt_if_type) || FAMILY_TYPE_IS_W2L(amlbt_if_type))
        && (get_resume_method() != RESUME_RTC_S) && (get_resume_method() != RESUME_RTC_C))
    {
        unsigned int reg_value = amlbt_usb_read_word(RG_AON_A52, BT_EP);
        printk("%s RG_AON_A52:%#x\n", __func__, reg_value);
        reg_value |= (1 << 27);
        amlbt_usb_write_word(RG_AON_A52, reg_value, BT_EP);
        printk("%s RG_AON_A52:%#x", __func__, amlbt_usb_read_word(RG_AON_A52, BT_EP));
    }
    printk("%s \n", __func__);
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

    printk("%s, type:%d \n", __func__, amlbt_if_type);

    printk("%s, type:%#x inter %s, family %s\n", __func__, amlbt_if_type,
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
        printk("%s amlbt_if_type invalid!! \n", __func__);
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

    printk("%s, type:%d \n", __func__, amlbt_if_type);

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
MODULE_DESCRIPTION("2024-02-04-1655");

