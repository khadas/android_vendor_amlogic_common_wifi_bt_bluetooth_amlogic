#ifndef __AML_BT_USB_T920_H__
#define __AML_BT_USB_T920_H__

typedef unsigned long SYS_TYPE;

#define W1u_VENDOR  0x414D

#define AML_BT_CHAR_DEVICE_NAME "aml_btusb"



#define BT_ICCM_AHB_BASE	0x00300000
#define BT_ICCM_ROM_LEN		256*1024
#define BT_DCCM_AHB_BASE	0x00400000

#define WF_SRAM_BASE_ADDR	0x00900000
#define WF_SRAM_RFU_ADDR	(WF_SRAM_BASE_ADDR + (224*1024))	//0x938000
#define WF_SRAM_RFU_LEN		(22*1024)
#define WF_SRAM_CMD_LEN		(2*1024)
#define WF_SRAM_EVENT_LEN	(4*1024)
#define WF_SRAM_TX_FIFO_LEN	(32+4)
#define WF_SRAM_RX_FIFO_LEN	(16+4)
#define WF_SRAM_TX_Q_NUM	(8)

#define HOST_TX_FIFO_LEN	(128)
#define TX_Q_LEN			(1024)
#define RX_Q_LEN			(1024)
#define RX_TYPE_FIFO_LEN	(1024)

#define WF_SRAM_TX_Q_ADDR		(WF_SRAM_RFU_ADDR + 0x0)			//8k	0x938000
#define WF_SRAM_RX_Q_ADDR		(WF_SRAM_TX_Q_ADDR + TX_Q_LEN*8)	//4k	0x93a000
#define WF_SRAM_EVENT_Q_ADDR	(WF_SRAM_RX_Q_ADDR + RX_Q_LEN*4) 	//4k	0x93b000
#define WF_SRAM_CMD_Q_ADDR		(WF_SRAM_EVENT_Q_ADDR + WF_SRAM_EVENT_LEN) 	//2k	0x93c000

#define WF_SRAM_REG_BASE_ADDR	(WF_SRAM_CMD_Q_ADDR + WF_SRAM_CMD_LEN)

#define WF_SRAM_CMD_FIFO_R_ADDR		(WF_SRAM_REG_BASE_ADDR + 0)
#define WF_SRAM_CMD_FIFO_W_ADDR    	(WF_SRAM_REG_BASE_ADDR + 0x04)

#define WF_SRAM_EVT_FIFO_R_ADDR		(WF_SRAM_REG_BASE_ADDR + 0x08)
#define WF_SRAM_EVT_FIFO_W_ADDR  	(WF_SRAM_REG_BASE_ADDR + 0x0c)
#define WF_SRAM_EVENT_LOCAL_WRITE_OFFSET  (WF_SRAM_REG_BASE_ADDR + 0x10)

#define WF_SRAM_TX_Q_STATUS_ADDR	(WF_SRAM_REG_BASE_ADDR + 0x14)	//32 bytes
#define WF_SRAM_TX_Q_INDEX_ADDR		(WF_SRAM_REG_BASE_ADDR + 0x34)	//32 bytes
#define WF_SRAM_TX_Q_PRIO_ADDR		(WF_SRAM_REG_BASE_ADDR + 0x54)	//32 bytes

//#define WF_SRAM_RX_Q_STATUS_ADDR	(WF_SRAM_RFU_ADDR + 0x4074)	//16 bytes

#define WF_SRAM_TX_Q_FIFO_ADDR		(WF_SRAM_REG_BASE_ADDR + 0x84)	//32 bytes
#define WF_SRAM_RX_Q_FIFO_ADDR		(WF_SRAM_REG_BASE_ADDR + 0xa4)	//16 bytes

//#define WF_SRAM_CMD_FIFO_R_ADDR		(0x40c4)	//4 bytes
//#define WF_SRAM_CMD_FIFO_W_ADDR		(0x40c8)	//4 bytes
#define WF_SRAM_TX_FIFO_R_ADDR		(WF_SRAM_REG_BASE_ADDR + 0xcc)	//4 bytes
#define WF_SRAM_TX_FIFO_W_ADDR		(WF_SRAM_REG_BASE_ADDR + 0xd0)	//4 bytes
#define WF_SRAM_RX_FIFO_R_ADDR		(WF_SRAM_REG_BASE_ADDR + 0xd4)	//4 bytes
#define WF_SRAM_RX_FIFO_W_ADDR		(WF_SRAM_REG_BASE_ADDR + 0xd8)	//4 bytes


#define WF_SRAM_RX_TYPE_FIFO_R_ADDR		(WF_SRAM_REG_BASE_ADDR + 0xdc)	//4 bytes
#define WF_SRAM_RX_TYPE_FIFO_W_ADDR		(WF_SRAM_REG_BASE_ADDR + 0xe0)	//4 bytes

//#define WF_SRAM_FW_DOWNLOAD_STATUS		(WF_SRAM_REG_BASE_ADDR + 0xe4)	//4 bytes


#define WF_SRAM_RX_TYPE_FIFO_ADDR		(WF_SRAM_RFU_ADDR + WF_SRAM_RFU_LEN - 2048)

//#define WF_SRAM_EVT_FIFO_R_ADDR		(0x40dc)	//4 bytes
//#define WF_SRAM_EVT_FIFO_W_ADDR		(0x40e0)	//4 bytes
//#define WF_SRAM_FW_RECV_R_ADDR		(WF_SRAM_RFU_ADDR + 0x40e4) //4 bytes
//#define WF_SRAM_FW_RECV_W_ADDR		(WF_SRAM_RFU_ADDR + 0x40e8)	//4 bytes

//#define WF_SRAM_FW_RECV_FIFO_ADDR	(WF_SRAM_RFU_ADDR + 0x40ec) //128 bytes

#define WF_SRAM_FW_DOWNLOAD_SIZE	(WF_SRAM_RFU_LEN - 2048)	//20k
//#define WF_SRAM_FW_DRIVER_REG_ADDR	(WF_SRAM_RFU_ADDR + 0x57f0)	//22k - 16
#define WF_SRAM_FW_DRIVER_STATUS_ADDR	(WF_SRAM_RFU_ADDR + WF_SRAM_RFU_LEN - 4)	//0x93d7fc
//#define WF_SRAM_BT_FW_LEN_ADDR		(WF_SRAM_RFU_ADDR + 0x57f4)	//4 bytes


//#define WF_SRAM_FW_RECV_DATA		1
//#define WF_SRAM_FW_RECV_EVENT		2

#define WF_SRAM_FD_DOWNLOAD_W		(1 << 0)	//firmware download write notify
#define WF_SRAM_FD_INIT_FLAG		(1 << 1)	//driver init flag
//#define WF_SRAM_FD_FW_CMD_COMPLETE	(1 << 2)	//firmware rx hci cmd complete
//#define WF_SRAM_FD_LOCK				(1 << 31)


#define HCI_MAX_ACL_SIZE    1020
#define HCI_MAX_SCO_SIZE    255
#define HCI_MAX_EVENT_SIZE    260
#define HCI_MAX_FRAME_SIZE    (HCI_MAX_ACL_SIZE + 4)

#define HCI_COMMAND_PKT        0x01
#define HCI_ACLDATA_PKT        0x02
#define HCI_SCODATA_PKT        0x03
#define HCI_EVENT_PKT        0x04
#define HCI_NO_TYPE			0xfe
#define HCI_VENDOR_PKT        0xff


#define GDSL_FIFO_MAX	(2*1024)

#define GDSL_TX_Q_MAX		(8)
#define GDSL_TX_Q_USED		(1)
#define GDSL_TX_Q_COMPLETE	(2)
#define GDSL_TX_Q_UNUSED	(0)

#define GDSL_RX_Q_MAX		(4)
#define GDSL_RX_Q_USED		(1)
#define GDSL_RX_Q_UNUSED	(0)


struct _gdsl_fifo_t;
typedef struct _gdsl_fifo_t gdsl_fifo_t;

typedef enum
{
    GDSL_ERR_SUCCESS,
    GDSL_ERR_NULL_POINTER,
    GDSL_ERR_NOT_FULL,
    GDSL_ERR_FULL,
    GDSL_ERR_NOT_EMPTY,
    GDSL_ERR_EMPTY,
    GDSL_ERR_SPACE_INVALID,
    GDSL_ERR_SPACE_VALID,
} gdsl_err_t;

struct _gdsl_fifo_t
{
    //unsigned char *fifo;
    unsigned char *r;
    unsigned char *w;
    unsigned char *base_addr;
    unsigned int size;
};

typedef struct
{
    unsigned char *tx_q_addr;
    unsigned int *tx_q_status_addr;
    unsigned int *tx_q_prio_addr;
    unsigned int *tx_q_dev_index_addr;

    unsigned int tx_q_status;
    unsigned int tx_q_prio;
    unsigned int tx_q_dev_index;
} gdsl_tx_q_t;

struct auc_hif_ops
{
    int (*hi_bottom_write8)(unsigned char  func_num, int addr, unsigned char data);
    unsigned char (*hi_bottom_read8)(unsigned char  func_num, int addr);
    int (*hi_bottom_read)(unsigned char func_num, int addr, void *buf, size_t len, int incr_addr);
    int (*hi_bottom_write)(unsigned char func_num, int addr, void *buf, size_t len, int incr_addr);

    unsigned char (*hi_read8_func0)(unsigned long sram_addr);
    void (*hi_write8_func0)(unsigned long sram_addr, unsigned char sramdata);

    unsigned long (*hi_read_reg8)(unsigned long sram_addr);
    void (*hi_write_reg8)(unsigned long sram_addr, unsigned long sramdata);
    unsigned long (*hi_read_reg32)(unsigned long sram_addr);
    int (*hi_write_reg32)(unsigned long sram_addr, unsigned long sramdata);

    void (*hi_write_cmd)(unsigned long sram_addr, unsigned long sramdata);
    void (*hi_write_sram)(unsigned char *buf, unsigned char *addr, SYS_TYPE len);
    void (*hi_read_sram)(unsigned char *buf, unsigned char *addr, SYS_TYPE len);
    void (*hi_write_word)(unsigned int addr, unsigned int data);
    unsigned int (*hi_read_word)(unsigned int addr);

    void (*hi_rcv_frame)(unsigned char *buf, unsigned char *addr, SYS_TYPE len);

    int (*hi_enable_scat)(void);
    void (*hi_cleanup_scat)(void);
    struct amlw_hif_scatter_req *(*hi_get_scatreq)(void);
    int (*hi_scat_rw)(struct scatterlist *sg_list, unsigned int sg_num, unsigned int blkcnt,
                      unsigned char func_num, unsigned int addr, unsigned char write);
    int (*hi_send_frame)(struct amlw_hif_scatter_req *scat_req);

    /*bt use*/
    void (*bt_hi_write_sram)(unsigned char *buf, unsigned char *addr, SYS_TYPE len);
    void (*bt_hi_read_sram)(unsigned char *buf, unsigned char *addr, SYS_TYPE len);
    void (*bt_hi_write_word)(unsigned int addr, unsigned int data);
    unsigned int (*bt_hi_read_word)(unsigned int addr);

    void (*hif_get_sts)(unsigned int op_code, unsigned int ctrl_code);
    void (*hif_pt_rx_start)(unsigned int qos);
    void (*hif_pt_rx_stop)(void);

    int (*hif_suspend)(unsigned int suspend_enable);
};


#endif

