#ifndef __AML_BT_USB_W2_H__
#define __AML_BT_USB_W2_H__

typedef unsigned long SYS_TYPE;

#define W1u_VENDOR  0x414D

#define AML_BT_CHAR_DEVICE_NAME "aml_btusb"
#define AML_BT_NOTE "stpbt"


/*-----W2-----*/
#define HI_USB_EVENT_Q_ADDR     (0x005001f4) //start:0x00500000, end:0x005011fc, length:4604 bytes
#define HI_USB_TX_Q_ADDR        (0x00508000) //start:0x00508000, end:0x0050a3fc, length:9212 bytes
#define HI_USB_MEM_ADDR         (0x00510000) //start:0x00510000, end:0x00510200, length:512 bytes
#define HI_USB_RX_Q_ADDR        (0x00514000) //start:0x00514000, end:0x00515000, length:4096 bytes
#define HI_USB_CMD_Q_ADDR       (0x00518000) //start:0x00518000, end:0x00519000, length:4096 bytes

#define HI_USB_TX_Q_LEN         (1024)
#define HI_USB_RX_Q_LEN         (1024)
#define BT_EVENT_QUEUE_LEN      (2048)
#define BT_CMD_QUEUE_LEN        (2048)
#define HI_USB_MEM_LEN          (128 * 4)
#define HI_USB_EVENT_Q_LEN      (4096)
#define HI_USB_CMD_Q_LEN        (4096)

#define HI_USB_TX_Q_TOTAL_LEN         (1024 * 8)
#define HI_USB_RX_Q_TOTAL_LEN         (1024 * 4)

#define HI_USB_TX_Q_FIFO_LEN           (32 + 4)
#define HI_USB_RX_Q_FIFO_LEN           (16 + 4)
#define HI_USB_RX_TYPE_FIFO_LEN        (256)        //1024

#define HI_USB_CMD_READ_OFF             (HI_USB_MEM_ADDR + 0x0)
#define HI_USB_CMD_WRITE_OFF            (HI_USB_MEM_ADDR + 0x4)



//#define HI_USB_TX_Q_STATUS_ADDR         (HI_USB_MEM_ADDR + 0x14)                        //32 bytes // 4*8
//#define HI_USB_TX_Q_HADNLE_ADDR         (HI_USB_MEM_ADDR + 0x34)                        //32 bytes // 4*8
//#define HI_USB_TX_Q_PRIO_ADDR           (HI_USB_MEM_ADDR + 0x54)                        //32 bytes // 4*8

#define HI_USB_TX_Q_PRIO_ADDR         (HI_USB_MEM_ADDR + 0x18)                        //32 bytes // 4*8
//#define HI_USB_TX_Q_HADNLE_ADDR       (HI_USB_MEM_ADDR + 0x34)                        //32 bytes // 4*8
//#define HI_USB_TX_Q_STATUS_ADDR       (HI_USB_MEM_ADDR + 0x54)                        //32 bytes // 4*8


//#define HI_USB_TX_Q_FIFO_ADDR           (HI_USB_MEM_ADDR + 0x74)                        //32 bytes + 4 byte     0x74--0x98
//#define HI_USB_RX_Q_FIFO_ADDR           (HI_USB_MEM_ADDR + 0x98)                        //16 bytes + 4 byte	 0x98--0xac

#define HI_USB_TX_FIFO_R_OFF            (HI_USB_MEM_ADDR + 0x08)                        //4 bytes
#define HI_USB_TX_FIFO_W_OFF            (HI_USB_MEM_ADDR + 0x0c)                        //4 bytes

//#define HI_USB_RX_TYPE_R_OFF            (HI_USB_MEM_ADDR + 0xb0)                        //4 bytes
//#define HI_USB_EVENT_READ_OFF           (HI_USB_MEM_ADDR + 0xb4)
//#define HI_USB_RX_FIFO_R_OFF            (HI_USB_MEM_ADDR + 0xb8)                        //4 bytes

//#define HI_USB_RX_FIFO_W_OFF            (HI_USB_MEM_ADDR + 0xbc)                        //4 bytes
//#define HI_USB_RX_TYPE_W_OFF            (HI_USB_MEM_ADDR + 0xc0)
//#define HI_USB_EVENT_WRITE_OFF          (HI_USB_MEM_ADDR + 0xc4)

#define HI_USB_RX_TYPE_R_OFF            (0x00500000)                        //4 bytes
#define HI_USB_EVENT_READ_OFF           (0x00500004)                        //4 bytes
#define HI_USB_RX_FIFO_R_OFF            (0x00500008)                        //4 bytes

#define HI_USB_RX_FIFO_W_OFF            (0x0050001c)                        //4 bytes
#define HI_USB_RX_TYPE_W_OFF            (0x00500020)                        //4 bytes
#define HI_USB_EVENT_WRITE_OFF          (0x00500024)                        //4 bytes

#define HI_USB_RX_Q_FIFO_ADDR           (0x00500028)                        //20 bytes

#define HI_USB_RX_TYPE_FIFO_ADDR        (0x00500080)                        //256 bytes  0xd0--0x1d0
// used on wf fw, for bt download.
#define HI_USB_FW_DRIVER_STATUS_ADDR    (HI_USB_MEM_ADDR + HI_USB_MEM_LEN - 4)          //0x1fc(0x200-4)
#define HI_USB_BT_TOTAL_DATA_LENGTH     (HI_USB_EVENT_Q_ADDR + 4096 - HI_USB_RX_TYPE_R_OFF + 4)
/*-----W2-----*/


#define BT_ICCM_AHB_BASE	0x00300000
#define BT_ICCM_ROM_LEN		256*1024
#define BT_DCCM_AHB_BASE	0x00400000

#define WF_SRAM_BASE_ADDR	0x00b00000
#define WF_SRAM_RFU_ADDR	(WF_SRAM_BASE_ADDR + (224*1024))	//0x938000
#define WF_SRAM_RFU_LEN		(22*1024)
#define WF_SRAM_CMD_LEN		(HI_USB_CMD_Q_LEN)
#define WF_SRAM_EVENT_LEN	(HI_USB_EVENT_Q_LEN)
#define WF_SRAM_TX_FIFO_LEN	(HI_USB_TX_Q_FIFO_LEN)
#define WF_SRAM_RX_FIFO_LEN	(HI_USB_RX_Q_FIFO_LEN)
#define WF_SRAM_TX_Q_NUM	(8)

#define HOST_TX_FIFO_LEN	(128)
#define TX_Q_LEN			(HI_USB_TX_Q_LEN)
#define RX_Q_LEN			(HI_USB_RX_Q_LEN)
#define RX_TYPE_FIFO_LEN	(HI_USB_RX_TYPE_FIFO_LEN)


#define WF_SRAM_TX_Q_ADDR		(HI_USB_TX_Q_ADDR)
#define WF_SRAM_RX_Q_ADDR		(HI_USB_RX_Q_ADDR)
#define WF_SRAM_EVENT_Q_ADDR	(HI_USB_EVENT_Q_ADDR)
#define WF_SRAM_CMD_Q_ADDR		(HI_USB_CMD_Q_ADDR)

#define WF_SRAM_REG_BASE_ADDR	(HI_USB_MEM_ADDR)

#define WF_SRAM_CMD_FIFO_R_ADDR		(HI_USB_CMD_READ_OFF)
#define WF_SRAM_CMD_FIFO_W_ADDR    	(HI_USB_CMD_WRITE_OFF)

#define WF_SRAM_EVT_FIFO_R_ADDR		(HI_USB_EVENT_READ_OFF)
#define WF_SRAM_EVT_FIFO_W_ADDR  	(HI_USB_EVENT_WRITE_OFF)
#define WF_SRAM_EVENT_LOCAL_WRITE_OFFSET  (WF_SRAM_REG_BASE_ADDR + 0x10)

//#define WF_SRAM_TX_Q_STATUS_ADDR	(HI_USB_TX_Q_STATUS_ADDR)	//32 bytes
//#define WF_SRAM_TX_Q_INDEX_ADDR		(HI_USB_TX_Q_HADNLE_ADDR)	//32 bytes
#define WF_SRAM_TX_Q_PRIO_ADDR		(HI_USB_TX_Q_PRIO_ADDR)	//32 bytes

//#define WF_SRAM_RX_Q_STATUS_ADDR	(WF_SRAM_RFU_ADDR + 0x4074)	//16 bytes

//#define WF_SRAM_TX_Q_FIFO_ADDR		(HI_USB_TX_Q_FIFO_ADDR)	//32 bytes
#define WF_SRAM_RX_Q_FIFO_ADDR		(HI_USB_RX_Q_FIFO_ADDR)	//16 bytes

//#define WF_SRAM_CMD_FIFO_R_ADDR		(0x40c4)	//4 bytes
//#define WF_SRAM_CMD_FIFO_W_ADDR		(0x40c8)	//4 bytes
#define WF_SRAM_TX_FIFO_R_ADDR		(HI_USB_TX_FIFO_R_OFF)	//4 bytes
#define WF_SRAM_TX_FIFO_W_ADDR		(HI_USB_TX_FIFO_W_OFF)	//4 bytes
#define WF_SRAM_RX_FIFO_R_ADDR		(HI_USB_RX_FIFO_R_OFF)	//4 bytes
#define WF_SRAM_RX_FIFO_W_ADDR		(HI_USB_RX_FIFO_W_OFF)	//4 bytes


#define WF_SRAM_RX_TYPE_FIFO_R_ADDR		(HI_USB_RX_TYPE_R_OFF)	//4 bytes
#define WF_SRAM_RX_TYPE_FIFO_W_ADDR		(HI_USB_RX_TYPE_W_OFF)	//4 bytes

//#define WF_SRAM_FW_DOWNLOAD_STATUS		(WF_SRAM_REG_BASE_ADDR + 0xe4)	//4 bytes


#define WF_SRAM_RX_TYPE_FIFO_ADDR		(HI_USB_RX_TYPE_FIFO_ADDR)

//#define WF_SRAM_EVT_FIFO_R_ADDR		(0x40dc)	//4 bytes
//#define WF_SRAM_EVT_FIFO_W_ADDR		(0x40e0)	//4 bytes
//#define WF_SRAM_FW_RECV_R_ADDR		(WF_SRAM_RFU_ADDR + 0x40e4) //4 bytes
//#define WF_SRAM_FW_RECV_W_ADDR		(WF_SRAM_RFU_ADDR + 0x40e8)	//4 bytes

//#define WF_SRAM_FW_RECV_FIFO_ADDR	(WF_SRAM_RFU_ADDR + 0x40ec) //128 bytes

#define WF_SRAM_FW_DOWNLOAD_SIZE	(WF_SRAM_RFU_LEN - 2048)	//20k
//#define WF_SRAM_FW_DRIVER_REG_ADDR	(WF_SRAM_RFU_ADDR + 0x57f0)	//22k - 16
#define WF_SRAM_FW_DRIVER_STATUS_ADDR	(HI_USB_FW_DRIVER_STATUS_ADDR)	//0x93d7fc
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

enum SDIO_STD_FUNNUM {
    SDIO_FUNC0=0,
    SDIO_FUNC1,
    SDIO_FUNC2,
    SDIO_FUNC3,
    SDIO_FUNC4,
    SDIO_FUNC5,
    SDIO_FUNC6,
    SDIO_FUNC7,
};

//sdio struct use
#define FUNCNUM_SDIO_LAST SDIO_FUNC7
#define SDIO_FUNCNUM_MAX (FUNCNUM_SDIO_LAST+1)
#define OS_LOCK spinlock_t

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


struct aml_hwif_sdio {
    struct sdio_func * sdio_func_if[SDIO_FUNCNUM_MAX];
    bool scatter_enabled;

    /* protects access to scat_req */
    OS_LOCK scat_lock;

    /* scatter request list head */
    struct amlw_hif_scatter_req *scat_req;
};

struct aml_hif_sdio_ops {
    //sdio func1 for self define domain, cmd52
    int (*hi_self_define_domain_write8)(int addr, unsigned char data);
    unsigned char (*hi_self_define_domain_read8)(int addr);
    int (*hi_self_define_domain_write32)(unsigned long sram_addr, unsigned long sramdata);
    unsigned long (*hi_self_define_domain_read32)(unsigned long sram_addr);

    //sdio func2 for random ram
    void (*hi_random_word_write)(unsigned int addr, unsigned int data);
    unsigned int (*hi_random_word_read)(unsigned int addr);
    void (*hi_random_ram_write)(unsigned char *buf, unsigned char *addr, size_t len);
    void (*hi_random_ram_read)(unsigned char *buf, unsigned char *addr, size_t len);

    //sdio func3 for sram
    void (*hi_sram_word_write)(unsigned int addr, unsigned int data);
    unsigned int (*hi_sram_word_read)(unsigned int addr);
    void (*hi_sram_write)(unsigned char *buf, unsigned char *addr, size_t len);
    void (*hi_sram_read)(unsigned char *buf, unsigned char *addr, size_t len);

    //sdio func4 for tx buffer
    void (*hi_tx_buffer_write)(unsigned char *buf, unsigned char *addr, size_t len);
    void (*hi_tx_buffer_read)(unsigned char *buf, unsigned char *addr, size_t len);

    //sdio func5 for rxdesc
    void (*hi_desc_read)(unsigned char *buf, unsigned char *addr, size_t len);

    //sdio func6 for rx buffer
    void (*hi_rx_buffer_read)(unsigned char* buf, unsigned char* addr, size_t len, unsigned char scat_use);

    //scatter list operation
    int (*hi_enable_scat)(struct aml_hwif_sdio *hif_sdio);
    void (*hi_cleanup_scat)(struct aml_hwif_sdio *hif_sdio);
    struct amlw_hif_scatter_req * (*hi_get_scatreq)(struct aml_hwif_sdio *hif_sdio);
    int (*hi_scat_rw)(struct scatterlist *sg_list, unsigned int sg_num, unsigned int blkcnt,
        unsigned char func_num, unsigned int addr, unsigned char write);
    int (*hi_send_frame)(struct amlw_hif_scatter_req *scat_req);
    int (*hi_recv_frame)(struct amlw_hif_scatter_req *scat_req);

    //sdio func7 for bt
    void (*bt_hi_write_sram)(unsigned char* buf, unsigned char* addr, SYS_TYPE len);
    void (*bt_hi_read_sram)(unsigned char* buf, unsigned char* addr, SYS_TYPE len);
    void (*bt_hi_write_word)(unsigned int addr,unsigned int data);
    unsigned int (*bt_hi_read_word)(unsigned int addr);

    //suspend & resume
    int (*hif_suspend)(unsigned int suspend_enable);
};

struct auc_hif_ops {
    int (*hi_send_cmd)(unsigned int addr, unsigned int len);
    void (*hi_write_word)(unsigned int addr,unsigned int data, unsigned int ep);
    unsigned int (*hi_read_word)(unsigned int addr, unsigned int ep);
    void (*hi_write_sram)(unsigned char* buf, unsigned char* addr, unsigned int len, unsigned int ep);
    void (*hi_read_sram)(unsigned char* buf, unsigned char* addr, unsigned int len, unsigned int ep);

    void (*hi_rx_buffer_read)(unsigned char* buf, unsigned char* addr, unsigned int len, unsigned int ep);

    /*bt use*/
    void (*hi_write_word_for_bt)(unsigned int addr,unsigned int data, unsigned int ep);
    unsigned int (*hi_read_word_for_bt)(unsigned int addr, unsigned int ep);
    void (*hi_write_sram_for_bt)(unsigned char* buf, unsigned char* addr, unsigned int len, unsigned int ep);
    void (*hi_read_sram_for_bt)(unsigned char* buf, unsigned char* addr, unsigned int len, unsigned int ep);

    int (*hi_enable_scat)(void);
    void (*hi_cleanup_scat)(void);
    struct amlw_usb_hif_scatter_req * (*hi_get_scatreq)(void);
    int (*hi_scat_rw)(struct scatterlist *sg_list, unsigned int sg_num, unsigned int blkcnt,
        unsigned char func_num, unsigned int addr, unsigned char write);

    int (*hi_send_frame)(struct amlw_usb_hif_scatter_req *scat_req);
    void (*hi_rcv_frame)(unsigned char* buf, unsigned char* addr, unsigned long len);
};
#if 0
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

    void (*bt_hi_write_sram_by_ep)(unsigned char* buf, unsigned char* addr, SYS_TYPE len, unsigned int ep);
    void (*bt_hi_read_sram_by_ep)(unsigned char* buf, unsigned char* addr, SYS_TYPE len, unsigned int ep);
    void (*bt_hi_write_word_by_ep)(unsigned int addr,unsigned int data, unsigned int ep);
    unsigned int (*bt_hi_read_word_by_ep)(unsigned int addr, unsigned int ep);

    void (*hif_get_sts)(unsigned int op_code, unsigned int ctrl_code);
    void (*hif_pt_rx_start)(unsigned int qos);
    void (*hif_pt_rx_stop)(void);

    int (*hif_suspend)(unsigned int suspend_enable);
};
#endif
struct amlw1_hif_ops {
	int				(*hi_bottom_write8)(unsigned char func_num, int addr, unsigned char data);
	unsigned char			(*hi_bottom_read8)(unsigned char func_num, int addr);
	int				(*hi_bottom_read)(unsigned char func_num, int addr, void *buf, size_t len, int incr_addr);
	int				(*hi_bottom_write)(unsigned char func_num, int addr, void *buf, size_t len, int incr_addr);

	unsigned char			(*hi_read8_func0)(unsigned long sram_addr);
	void				(*hi_write8_func0)(unsigned long sram_addr, unsigned char sramdata);

	unsigned long			(*hi_read_reg8)(unsigned long sram_addr);
	void				(*hi_write_reg8)(unsigned long sram_addr, unsigned long sramdata);
	unsigned long			(*hi_read_reg32)(unsigned long sram_addr);
	int				(*hi_write_reg32)(unsigned long sram_addr, unsigned long sramdata);

	void				(*hi_write_cmd)(unsigned long sram_addr, unsigned long sramdata);
	void				(*hi_write_sram)(unsigned char *buf, unsigned char *addr, SYS_TYPE len);
	void				(*hi_read_sram)(unsigned char *buf, unsigned char *addr, SYS_TYPE len);
	void				(*hi_write_word)(unsigned int addr, unsigned int data);
	unsigned int			(*hi_read_word)(unsigned int addr);

	void				(*hi_rcv_frame)(unsigned char *buf, unsigned char *addr, SYS_TYPE len);

	int				(*hi_enable_scat)(void);
	void				(*hi_cleanup_scat)(void);
	struct amlw_hif_scatter_req *	(*hi_get_scatreq)(void);
	int				(*hi_scat_rw)(struct scatterlist *sg_list, unsigned int sg_num, unsigned int blkcnt, unsigned char func_num, unsigned int addr, unsigned char write);
	int				(*hi_send_frame)(struct amlw_hif_scatter_req *scat_req);

	/*bt use*/
	void				(*bt_hi_write_sram)(unsigned char *buf, unsigned char *addr, SYS_TYPE len);
	void				(*bt_hi_read_sram)(unsigned char *buf, unsigned char *addr, SYS_TYPE len);
	void				(*bt_hi_write_word)(unsigned int addr, unsigned int data);
	unsigned int			(*bt_hi_read_word)(unsigned int addr);

    void				(*hif_get_sts)(unsigned int op_code, unsigned int ctrl_code);
	void				(*hif_pt_rx_start)(unsigned int qos);
	void				(*hif_pt_rx_stop)(void);

	int				(*hif_suspend)(unsigned int suspend_enable);
};

#define BT_BIT  0               //means bt
#define REGISTER_BT_SDIO 1      //used to mark register bt sdio ko
#define UNREGISTER_BT_SDIO 0
#define REGISTER_BT_USB 1       //used to mark register bt USB ko
#define UNREGISTER_BT_USB 0
#define BT_PWR_ON 1
#define BT_PWR_OFF 0

#define SDIO_FUNC1 1

#define RG_BT_SDIO_PMU_HOST_REQ         0x231
#define RG_SDIO_PMU_STATUS                      0x23c

#define CHIP_BT_PMU_REG_BASE               (0xf03000)
#define RG_BT_PMU_A11                             (CHIP_BT_PMU_REG_BASE + 0x2c)
#define RG_BT_PMU_A12                             (CHIP_BT_PMU_REG_BASE + 0x30)
#define RG_BT_PMU_A13                             (CHIP_BT_PMU_REG_BASE + 0x34)
#define RG_BT_PMU_A14                             (CHIP_BT_PMU_REG_BASE + 0x38)
#define RG_BT_PMU_A15                             (CHIP_BT_PMU_REG_BASE + 0x3c)
#define RG_BT_PMU_A16                             (CHIP_BT_PMU_REG_BASE + 0x40)
#define RG_BT_PMU_A17                             (CHIP_BT_PMU_REG_BASE + 0x44)
#define RG_BT_PMU_A18                             (CHIP_BT_PMU_REG_BASE + 0x48)
#define RG_BT_PMU_A20                             (CHIP_BT_PMU_REG_BASE + 0x50)
#define RG_BT_PMU_A22                             (CHIP_BT_PMU_REG_BASE + 0x58)

#define CHIP_INTF_REG_BASE               (0xf00000)
#define RG_AON_A15                                (CHIP_INTF_REG_BASE + 0x3c)
#define RG_AON_A30                                (CHIP_INTF_REG_BASE + 0x78)
#define RG_AON_A52                                (CHIP_INTF_REG_BASE + 0xd0)


// pmu status
#define PMU_PWR_OFF       0x0
#define PMU_PWR_XOSC      0x1
#define PMU_XOSC_WAIT     0x2
#define PMU_XOSC_DPLL     0x3
#define PMU_DPLL_WAIT     0x4
#define PMU_DPLL_ACT      0x5
#define PMU_ACT_MODE      0x6
#define PMU_ACT_SLEEP     0x7
#define PMU_SLEEP_MODE    0x8
#define PMU_SLEEP_WAKE    0x9
#define PMU_WAKE_WAIT     0xa
#define PMU_WAKE_XOSC     0xb

#define	AMLBT_PD_ID_INTF            0x7
#define AMLBT_PD_ID_FAMILY_VER      (0x7<<6)
#define AMLBT_PD_ID_FAMILY          (0x1f<<9)

#define AMLBT_INTF_SDIO             0x0
#define AMLBT_INTF_USB              0x01
#define AMLBT_INTF_PCIE             0x02

#define AMLBT_FAMILY_W1             (0x01<<9)
#define AMLBT_FAMILY_W1U            (0x02<<9)
#define AMLBT_FAMILY_W2             (0x03<<9)
#define AMLBT_TRANS_UNKNOWN          0x00

#define AMLBT_TRANS_W1_UART         0x01
#define AMLBT_TRANS_W1U_UART        0x02
#define AMLBT_TRANS_W2_UART         0x03
#define AMLBT_TRANS_W1U_USB         0x04
#define AMLBT_TRANS_W2_USB          0x05


enum usb_endpoint_num{
    USB_EP0 = 0x0,
    USB_EP1,
    USB_EP2,
    USB_EP3,
    USB_EP4,
    USB_EP5,
    USB_EP6,
    USB_EP7,
};


#endif

