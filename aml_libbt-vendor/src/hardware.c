/******************************************************************************
*
*  Copyright (C) 2019-2021 Amlogic Corporation
*
*  Licensed under the Apache License, Version 2.0 (the "License");
*  you may not use this file except in compliance with the License.
*  You may obtain a copy of the License at:
*
*  http://www.apache.org/licenses/LICENSE-2.0
*
*  Unless required by applicable law or agreed to in writing, software
*  distributed under the License is distributed on an "AS IS" BASIS,
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*  See the License for the specific language governing permissions and
*  limitations under the License.
*
******************************************************************************/

/******************************************************************************
*
*  Filename:      hardware.c
*
*  Description:   Contains controller-specific functions, like
*                      firmware patch download
*                      low power mode operations
*
******************************************************************************/

#define LOG_TAG "bt_hwcfg"

#include <unistd.h>
#include <utils/Log.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <signal.h>
#include <time.h>
#include <errno.h>
#include <fcntl.h>
#include <dirent.h>
#include <ctype.h>
#include <cutils/properties.h>
#include <stdlib.h>
#include <string.h>
#include "bt_hci_bdroid.h"
#include "bt_vendor_aml.h"
#ifdef O_AMLOGIC
    #include "esco_parameters.h"
#else
    #include "hci_audio.h"
#endif
#include "userial.h"
#include "userial_vendor.h"
#include "upio.h"

void aml_15p4_tx(unsigned char *data, unsigned short len);
extern void aml_15p4_data_cb(unsigned char *p_mem);

/**********bt related path in the file system**********/
#ifndef AML_BT_FS_PATH
    //#define AML_BT_FS_PATH "/vendor/etc/bluetooth/amlbt"
    #define AML_BT_FS_PATH "/vendor/firmware"
#endif

/**********bt config**********/
#define AML_BT_CONFIG_RF_FILE AML_BT_FS_PATH"/aml_bt.conf"

/**********bt rom check************/
//#define AML_BT_ROM_CHECK_ENABLE

/**********bt fw**********/
#define AML_W1_BT_FW_UART_FILE  AML_BT_FS_PATH"/w1_bt_fw_uart.bin"
#define AML_W1U_BT_FW_UART_FILE AML_BT_FS_PATH"/w1u_bt_fw_uart.bin"
#define AML_W2_BT_FW_UART_FILE AML_BT_FS_PATH"/w2_bt_fw_uart.bin"
#define AML_W1U_BT_FW_USB_FILE  AML_BT_FS_PATH"/w1u_bt_fw_usb.bin"
#define AML_W2_BT_FW_USB_FILE  AML_BT_FS_PATH"/w2_bt_fw_usb.bin"

#define AML_BT_CHIP_TYPE        5
#define AML_BT_INTF_TYPE        3
#define REG_DEV_RESET           0xf03058
#define REG_PMU_POWER_CFG       0xf03040
#define REG_RAM_PD_SHUTDWONW_SW 0xf03050
#define REG_FW_MODE             0x00f000e0
#define BIT_PHY                 1
#define BIT_MAC                 (1 << 1)
#define BIT_CPU                 (1 << 2)
#define DEV_RESET_SW            16
#define BIT_RF_NUM              28
#define BT_SINK_MODE            25


extern int amlbt_btrecovery;
extern int bt_sdio_fd;
extern unsigned int amlbt_rftype;
extern unsigned int amlbt_btsink;
extern unsigned int amlbt_fw_mode;

extern unsigned int hw_state;
unsigned int state = 0;
const char *amlbt_file_path[] = {
	"/vendor/firmware",
	NULL
};

char *amlbt_fw_bin[AML_BT_CHIP_TYPE][AML_BT_INTF_TYPE] =
{
    {
        NULL,
        NULL,
        NULL,
    },
    {
        "w1_bt_fw_uart.bin",     //w1 sdio
        NULL,                                               //w1 usb
        NULL,                                               //w1 pcie
    },
    {
        "w1u_bt_fw_uart.bin",    //w1u sdio
        "w1u_bt_fw_usb.bin",     //w1u usb
        "w1u_bt_fw_uart.bin",    //w1u pcie
    },
    {
        "w2_bt_fw_uart.bin",     //w2 sdio
        "w2_bt_fw_usb.bin",      //w2 usb
        "w2_bt_fw_uart.bin",     //w2 pcie
    },
    {
        "w2l_bt_15p4_fw_uart.bin",     //w2l sdio
        "w2l_bt_15p4_fw_usb.bin",      //w2 usb
        "w2l_bt_15p4_fw_uart.bin",     //w2 pcie
    }
};

/*baudrate define FPGA*/
//#define FPGA_ENABLE
//#define UART_2M
#define UART_4M


#define BTM_SCO_CODEC_CVSD 0x0001

#define AML_DOWNLOADFW_UART
#define AML_FW_BIN
//#define AML_FW_FILE

#ifdef AML_DOWNLOADFW_UART
    #ifdef AML_FW_FILE
        #include "bt_fucode.h"
    #endif
#endif
/******************************************************************************
**  Constants & Macros
******************************************************************************/

#ifndef BTHW_DBG
    #define BTHW_DBG TRUE
#endif

#if (BTHW_DBG == TRUE)
    #define BTHWDBG(param, ...) { ALOGD(param, ## __VA_ARGS__); }
#else
    #define BTHWDBG(param, ...) {}
#endif

#define FW_PATCHFILE_EXTENSION      ".hcd"
#define FW_PATCHFILE_EXTENSION_LEN  4
#define FW_PATCHFILE_PATH_MAXLEN    248 /* Local_Name length of return of
					 * HCI_Read_Local_Name */

static const char VENDOR_AMLBTVER_PROPERTY[] = "vendor.sys.amlbtversion";
static const char AML_ROM_CHECK_PROPERTY[] = "vendor.sys.amlbt_rom_check";
static const char PWR_PROP_NAME[] = "sys.shutdown.requested";

#define HCI_CMD_MAX_LEN                         258

#define HCI_READ_LOCAL_INFO                     0x1001
#define HCI_RESET                               0x0C03
#define HCI_VSC_WRITE_UART_CLOCK_SETTING        0xFC45
#define HCI_VSC_UPDATE_BAUDRATE                 0xFC18
#define HCI_READ_LOCAL_NAME                     0x0C14
#define HCI_VSC_DOWNLOAD_MINIDRV                0xFC2E
#define HCI_VSC_WRITE_BD_ADDR                   0xFC1A
#define HCI_VSC_WRITE_PARAMS                    0xFC20
#define HCI_VSC_WRITE_SLEEP_MODE                0xFC27
#define HCI_VSC_WRITE_SCO_PCM_INT_PARAM         0xFC1C
#define HCI_VSC_WRITE_PCM_DATA_FORMAT_PARAM     0xFC1E
#define HCI_VSC_WRITE_I2SPCM_INTERFACE_PARAM    0xFC6D
#define HCI_VSC_ENABLE_WBS                      0xFC7E
#define HCI_VSC_LAUNCH_RAM                      0xFC4E
#define HCI_READ_LOCAL_BDADDR                   0x1009
#define HCI_RECOVERY_CMD                        0xFF98
#define HCI_CHECK_CMD                           0xfC51
#define HCI_VSC_WAKE_WRITE_DATA                 0xFC22
#define HCI_HOST_SLEEP_VSC                      0xfc21
#define HCI_FW_CLEAR_LIST                       0xfc55

#define HCI_AML_15P4_CMD                        0xFF9A

#ifdef AML_DOWNLOADFW_UART
    int len_iccm = 0, offset_iccm = 0;
    unsigned int cmd_len_iccm = 0;
    unsigned int data_len_iccm = 0;
    int len_dccm = 0, offset_dccm = 0;
    unsigned int cmd_len_dccm = 0;
    unsigned int data_len_dccm = 0;
    int cnt = 1;
    int iccm_read_off = 0;
    unsigned int j = 0;
    int cmp_data = 0;
    int reg_data = 0;
    static int bt_start_config;

    #ifdef AML_FW_BIN
        unsigned char *p_iccm_buf = NULL;
        unsigned char *p_dccm_buf = NULL;
        unsigned int iccm_size = 0;
        unsigned int dccm_size = 0;
        unsigned int rom_size = 256 * 1024;
        unsigned int check_size = 4 * 1024;
        unsigned int check_start = 0;
        int fw_fd = -1;
    #endif

    #define ICCM_RAM_BASE           (0x000000)
    #define DCCM_RAM_BASE           (0xd00000)
    #define RW_OPERATION_SIZE        (248)
    #define TCI_READ_REG                            0xfef0
    #define TCI_WRITE_REG                           0xfef1
    #define TCI_UPDATE_UART_BAUDRATE                0xfef2
    #define TCI_DOWNLOAD_BT_FW                      0xfef3
#endif


#define HCI_EVT_CMD_CMPL_STATUS_RET_BYTE        5
#define HCI_EVT_CMD_CMPL_LOCAL_NAME_STRING      6
#define HCI_EVT_CMD_CMPL_LOCAL_BDADDR_ARRAY     6
#define HCI_EVT_CMD_CMPL_LOCAL_FW_VERSION       6
#define HCI_EVT_CMD_CMPL_OPCODE                 3
#define LPM_CMD_PARAM_SIZE                      12
#define UPDATE_BAUDRATE_CMD_PARAM_SIZE          6
#define HCI_CMD_PREAMBLE_SIZE                   3
#define HCD_REC_PAYLOAD_LEN_BYTE                2
#define BD_ADDR_LEN                             6
#define LOCAL_BDADDR_PATH_BUFFER_LEN            256

#define STREAM_TO_UINT16(u16, p) { u16 = ((uint16_t)(*(p)) + (((uint16_t)(*((p) + 1))) << 8)); (p) += 2; }
#define UINT8_TO_STREAM(p, u8)   { *(p)++ = (uint8_t)(u8); }
#define UINT16_TO_STREAM(p, u16) { *(p)++ = (uint8_t)(u16); *(p)++ = (uint8_t)((u16) >> 8); }
#define UINT32_TO_STREAM(p, u32) { *(p)++ = (uint8_t)(u32); *(p)++ = (uint8_t)((u32) >> 8); *(p)++ = (uint8_t)((u32) >> 16); *(p)++ = (uint8_t)((u32) >> 24); }

#define SCO_INTERFACE_PCM  0
#define SCO_INTERFACE_I2S  1

#define RG_AON_A                (0x00f00094)
#define W1U_RG_AON_A            (0x1)
#define W1_RG_AON_A             (0x0)

/* one byte is for enable/disable
 *    next 2 bytes are for codec type */
#define SCO_CODEC_PARAM_SIZE                    3

/******************************************************************************
**  Local type definitions
******************************************************************************/
/* Power off send 6 cmd*/
enum
{
    HW_RESET_CLOSE,
    HW_DISBT_CONFIGURE,
    HW_REG_PUM_CLEAR,
    HW_CLEAR_LIST,
};

/* Hardware Configuration State */
enum
{
    HW_CFG_AML_WRITE_FIRMWARE_MODE = 0,
    HW_CFG_AML_UPDATE_BAUDRATE_UART = 1,    //1
    HW_CFG_AML_DOWNLOAD_FIRMWARE_TEST_3_UART,   //2
    HW_CFG_AML_DOWNLOAD_FIRMWARE_TEST_2_UART,  //3
    HW_CFG_AML_DOWNLOAD_FIRMWARE_ICCM_UART, //4
    HW_CFG_AML_DOWNLOAD_FIRMWARE_DCCM_UART, //5
    HW_CFG_AML_DOWNLOAD_FIRMWARE_CLOSE_EVENT,   //6
    HW_CFG_AML_CONFIG_RF_CALIBRATION,   //7
    HW_CFG_AML_DOWNLOAD_FIRMWARE_STRAT_CPU_UART_BEFORE, //8
    HW_CFG_AML_DOWNLOAD_FIRMWARE_STRAT_CPU_UART,    //9
    HW_CFG_SET_PARAMS,      //a
    HW_CFG_SET_BD_ADDR,     //b
    HW_CFG_AML_POWER_END,   //c
    HW_CFG_SET_MANU_DATA,   //d
    HW_CFG_START,       //e
    HW_CFG_SET_UART_CLOCK,  //f
    HW_CFG_SET_UART_BAUD_1, //g
    HW_CFG_READ_LOCAL_NAME, //h
    //HW_CFG_DL_MINIDRIVER,   //5
    HW_CFG_DL_FW_PATCH,     //i
    HW_CFG_SET_UART_BAUD_2, //j
    HW_CFG_AML_UPDATE_BAUDRATE, //k
    HW_CFG_GET_REG, //l
    HW_CFG_AML_DOWNLOAD_FIRMWARE_CHANGE_BAUDRATE_UART,  //m
    HW_CFG_AML_DOWNLOAD_FIRMWARE_FINISH_UART,   //n
    HW_CFG_AML_DOWNLOAD_FIRMWARE_TEST_UART, //o
    HW_CFG_AML_SET_BAUDRATE, //p
#if (USE_CONTROLLER_BDADDR == TRUE)
    HW_CFG_READ_BD_ADDR,   //q
#endif
};

/* low power mode parameters */
typedef struct
{
    uint8_t sleep_mode;                     /* 0(disable),1(UART),9(H5) */
    uint8_t host_stack_idle_threshold;      /* Unit scale 300ms/25ms */
    uint8_t host_controller_idle_threshold; /* Unit scale 300ms/25ms */
    uint8_t bt_wake_polarity;               /* 0=Active Low, 1= Active High */
    uint8_t host_wake_polarity;             /* 0=Active Low, 1= Active High */
    uint8_t allow_host_sleep_during_sco;
    uint8_t combine_sleep_mode_and_lpm;
    uint8_t enable_uart_txd_tri_state;      /* UART_TXD Tri-State */
    uint8_t sleep_guard_time;               /* sleep guard time in 12.5ms */
    uint8_t wakeup_guard_time;              /* wakeup guard time in 12.5ms */
    uint8_t txd_config;                     /* TXD is high in sleep state */
    uint8_t pulsed_host_wake;               /* pulsed host wake if mode = 1 */
} bt_lpm_param_t;

/* Firmware re-launch settlement time */
typedef struct
{
    const char 	*chipset_name;
    const uint32_t	delay_time;
} fw_settlement_entry_t;

uint8_t hw_cfg_write_fw_mode(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p);
uint8_t hw_cfg_update_baudrate_uart(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p);
uint8_t hw_cfg_download_firmware_test_3_uart(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p);
uint8_t hw_cfg_download_firmware_test_2_uart(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p);
uint8_t hw_cfg_download_firmware_iccm_uart(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p);
uint8_t hw_cfg_download_firmware_dccm_uart(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p);
uint8_t hw_cfg_download_firmware_close_event(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p);
uint8_t hw_cfg_rf_calibration(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p);
uint8_t hw_cfg_download_firmware_start_cpu_uart_before(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p);
uint8_t hw_cfg_download_firmware_start_cpu_uart(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p);
uint8_t hw_cfg_set_params(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p);
uint8_t hw_cfg_set_bd_addr(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p);
uint8_t hw_cfg_bt_power_end(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p);
uint8_t wole_config_write_manufacture(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p);
uint8_t hw_cfg_start(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p);
uint8_t hw_cfg_set_uart_clock(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p);
uint8_t hw_cfg_set_uart_baud1(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p);
uint8_t hw_cfg_read_local_name(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p);
//uint8_t hw_cfg_dl_minidriver(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p);
uint8_t hw_cfc_null_2(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p);
uint8_t hw_cfg_set_uart_baud_2(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p);
uint8_t hw_cfg_update_baudrate(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p);
uint8_t hw_cfg_get_reg(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p);
uint8_t hw_cfg_download_firmware_change_baudrate_uart(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p);
uint8_t hw_cfg_download_firmware_finish_uart(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p);
uint8_t hw_cfg_download_firmware_test_uart(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p);
uint8_t hw_cfg_set_baudrate(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p);
uint8_t hw_cfg_read_bd_addr(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p);
uint8_t hw_cfg_rom_check(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p);

uint8_t (*hw_config_func[])(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p) =
{
    hw_cfg_write_fw_mode,    //0
    hw_cfg_update_baudrate_uart,    //1
    hw_cfg_download_firmware_test_3_uart,   //2
    hw_cfg_download_firmware_test_2_uart,  //3
    hw_cfg_download_firmware_iccm_uart, //4
    hw_cfg_download_firmware_dccm_uart, //5
    hw_cfg_download_firmware_close_event,   //6
    hw_cfg_rf_calibration,   //7
    hw_cfg_download_firmware_start_cpu_uart_before, //8
    hw_cfg_download_firmware_start_cpu_uart,    //9
    hw_cfg_set_params,      //a
    hw_cfg_set_bd_addr,     //b
    hw_cfg_bt_power_end,   //c
    wole_config_write_manufacture,    //d
    hw_cfg_start,       //e
    hw_cfg_set_uart_clock,  //f
    hw_cfg_set_uart_baud1, //g
    hw_cfg_read_local_name, //h
    //hw_cfg_dl_minidriver,   //
    hw_cfc_null_2,     //i
    hw_cfg_set_uart_baud_2, //j
    hw_cfg_update_baudrate, //k
    hw_cfg_get_reg, //l
    hw_cfg_download_firmware_change_baudrate_uart,  //m
    hw_cfg_download_firmware_finish_uart,   //n
    hw_cfg_download_firmware_test_uart, //o
    hw_cfg_set_baudrate, //p
#if (USE_CONTROLLER_BDADDR == TRUE)
    hw_cfg_read_bd_addr,   //q
#endif
};


/******************************************************************************
**  Externs
******************************************************************************/

void hw_config_cback(void *p_evt_buf);
extern uint8_t vnd_local_bd_addr[BD_ADDR_LEN];
extern unsigned int amlbt_rftype;
extern unsigned char bt_power;
extern unsigned int download_hw;

/******************************************************************************
**  Static variables
******************************************************************************/

static char fw_patchfile_path[256] = FW_PATCHFILE_LOCATION;
static char fw_patchfile_name[128] = { 0 };
#if (VENDOR_LIB_RUNTIME_TUNING_ENABLED == TRUE)
    static int fw_patch_settlement_delay = -1;
#endif

static int wbs_sample_rate = SCO_WBS_SAMPLE_RATE;
extern bt_hw_cfg_cb_t hw_cfg_cb;
static timer_t bt_recovery_timer = 0;

void hw_read_type(HC_BT_HDR *p_buf);
void hw_read_type_cback(void *p_mem);

static bt_lpm_param_t lpm_param =
{
    LPM_SLEEP_MODE,
    LPM_IDLE_THRESHOLD,
    LPM_HC_IDLE_THRESHOLD,
    LPM_BT_WAKE_POLARITY,
    LPM_HOST_WAKE_POLARITY,
    LPM_ALLOW_HOST_SLEEP_DURING_SCO,
    LPM_COMBINE_SLEEP_MODE_AND_LPM,
    LPM_ENABLE_UART_TXD_TRI_STATE,
    0,      /* not applicable */
    0,      /* not applicable */
    0,      /* not applicable */
    LPM_PULSED_HOST_WAKE
};

#if (SCO_CFG_INCLUDED == TRUE)

/* need to update the bt_sco_i2spcm_param as well
 * bt_sco_i2spcm_param will be used for WBS setting
 * update the bt_sco_param and bt_sco_i2spcm_param */
static uint8_t bt_sco_param[SCO_PCM_PARAM_SIZE] =
{
    SCO_PCM_ROUTING,
    SCO_PCM_IF_CLOCK_RATE,
    SCO_PCM_IF_FRAME_TYPE,
    SCO_PCM_IF_SYNC_MODE,
    SCO_PCM_IF_CLOCK_MODE
};

static uint8_t bt_pcm_data_fmt_param[PCM_DATA_FORMAT_PARAM_SIZE] =
{
    PCM_DATA_FMT_SHIFT_MODE,
    PCM_DATA_FMT_FILL_BITS,
    PCM_DATA_FMT_FILL_METHOD,
    PCM_DATA_FMT_FILL_NUM,
    PCM_DATA_FMT_JUSTIFY_MODE
};

static uint8_t bt_sco_i2spcm_param[SCO_I2SPCM_PARAM_SIZE] =
{
    SCO_I2SPCM_IF_MODE,
    SCO_I2SPCM_IF_ROLE,
    SCO_I2SPCM_IF_SAMPLE_RATE,
    SCO_I2SPCM_IF_CLOCK_RATE
};
#endif
/*
 * The look-up table of recommended firmware settlement delay (milliseconds) on
 * known chipsets.
 */
static const fw_settlement_entry_t fw_settlement_table[] =
{
    { "BCM43241",	      200 },
    { "BCM43341",	      100 },
    { (const char *)NULL, 100 }// Giving the generic fw settlement delay setting.
};


/*
 * NOTICE:
 *     If the platform plans to run I2S interface bus over I2S/PCM port of the
 *     BT Controller with the Host AP, explicitly set "SCO_USE_I2S_INTERFACE = TRUE"
 *     in the corresponding include/vnd_<target>.txt file.
 *     Otherwise, leave SCO_USE_I2S_INTERFACE undefined in the vnd_<target>.txt file.
 *     And, PCM interface will be set as the default bus format running over I2S/PCM
 *     port.
 */
#if (defined(SCO_USE_I2S_INTERFACE) && SCO_USE_I2S_INTERFACE == TRUE)
    static uint8_t sco_bus_interface = SCO_INTERFACE_I2S;
#else
    static uint8_t sco_bus_interface = SCO_INTERFACE_PCM;
#endif

#define INVALID_SCO_CLOCK_RATE  0xFF
static uint8_t sco_bus_clock_rate = INVALID_SCO_CLOCK_RATE;
static uint8_t sco_bus_wbs_clock_rate = INVALID_SCO_CLOCK_RATE;

/******************************************************************************
**  Static functions
******************************************************************************/
static void hw_sco_i2spcm_config(void *p_mem, uint16_t codec);

static int open_file(const char *filename, int flags)
{
    int fd = -1;
    char debug_path[PROPERTY_VALUE_MAX] = {'\0'};
    char buf[256];
    const char *f = NULL;
    int i = 0;

    memset(debug_path, 0, sizeof(debug_path));
    f = strrchr(filename, '/');

    /* Note: you need disable selinux and gives chmod permission for
    ** driver files when specify the path of driver.
    ** e.g. setprop persist.vendor.wifibt_drv_path "/data/vendor"
    */
    if (property_get("persist.vendor.wifibt_drv_path", debug_path, NULL)) {
        memset(buf, 0, sizeof(buf));
        if (f)
            snprintf(buf, sizeof(buf), "%s%s", debug_path, f);
        else
            snprintf(buf, sizeof(buf), "%s/%s", debug_path, filename);
        ALOGD("open file: %s\n", buf);
        if ((fd = open(buf, flags)) < 0) {
            ALOGD("open: %s failed!\n", buf);
        } else {
            ALOGD("open: %s successful!\n", buf);
            return fd;
        }
    }

    if (!f) {
        for (i = 0; amlbt_file_path[i] != NULL; i++) {
            memset(buf, 0, sizeof(buf));
            snprintf(buf, sizeof(buf), "%s/%s", amlbt_file_path[i], filename);
            ALOGD("open file: %s\n", buf);
            if ((fd = open(buf, flags)) < 0) {
                ALOGD("open file: %s failed!\n", buf);
            } else {
                ALOGD("open file: %s successful!\n", buf);
                return fd;
            }
        }
    } else {
        ALOGD("open file: %s\n", filename);
        if ((fd = open(filename, flags)) < 0) {
            ALOGD("open: %s failed!\n", filename);
        } else {
            ALOGD("open: %s successful!\n", filename);
            return fd;
        }
    }

    return fd;
}

/******************************************************************************
**  Controller Initialization Static Functions
******************************************************************************/

/*******************************************************************************
**
** Function        look_up_fw_settlement_delay
**
** Description     If FW_PATCH_SETTLEMENT_DELAY_MS has not been explicitly
**                 re-defined in the platform specific build-time configuration
**                 file, we will search into the look-up table for a
**                 recommended firmware settlement delay value.
**
**                 Although the settlement time might be also related to board
**                 configurations such as the crystal clocking speed.
**
** Returns         Firmware settlement delay
**
*******************************************************************************/
uint32_t look_up_fw_settlement_delay(void)
{
    uint32_t ret_value;
    fw_settlement_entry_t *p_entry;

    if (FW_PATCH_SETTLEMENT_DELAY_MS > 0)
    {
        ret_value = FW_PATCH_SETTLEMENT_DELAY_MS;
    }
#if (VENDOR_LIB_RUNTIME_TUNING_ENABLED == TRUE)
    else if (fw_patch_settlement_delay >= 0)
    {
        ret_value = fw_patch_settlement_delay;
    }
#endif
    else
    {
        p_entry = (fw_settlement_entry_t *)fw_settlement_table;

        while (p_entry->chipset_name != NULL)
        {
            if (strstr(hw_cfg_cb.local_chip_name, p_entry->chipset_name) != NULL)
            {
                break;
            }

            p_entry++;
        }

        ret_value = p_entry->delay_time;
    }

    BTHWDBG("Settlement delay -- %d ms", ret_value);

    return (ret_value);
}

/*******************************************************************************
**
** Function        ms_delay
**
** Description     sleep unconditionally for timeout milliseconds
**
** Returns         None
**
*******************************************************************************/
void ms_delay(uint32_t timeout)
{
    struct timespec delay;
    int err;

    if (timeout == 0)
        return;

    delay.tv_sec = timeout / 1000;
    delay.tv_nsec = 1000 * 1000 * (timeout % 1000);

    /* [u]sleep can't be used because it uses SIGALRM */
    do
    {
        err = nanosleep(&delay, &delay);
    }
    while (err < 0 && errno == EINTR);
}

/*******************************************************************************
**
** Function        line_speed_to_userial_baud
**
** Description     helper function converts line speed number into USERIAL baud
**                 rate symbol
**
** Returns         unit8_t (USERIAL baud symbol)
**
*******************************************************************************/
uint8_t line_speed_to_userial_baud(uint32_t line_speed)
{
    uint8_t baud;

    if (line_speed == 4000000)
        baud = USERIAL_BAUD_4M;
    else if (line_speed == 3000000)
        baud = USERIAL_BAUD_3M;
    else if (line_speed == 2000000)
        baud = USERIAL_BAUD_2M;
    else if (line_speed == 1000000)
        baud = USERIAL_BAUD_1M;
    else if (line_speed == 921600)
        baud = USERIAL_BAUD_921600;
    else if (line_speed == 460800)
        baud = USERIAL_BAUD_460800;
    else if (line_speed == 230400)
        baud = USERIAL_BAUD_230400;
    else if (line_speed == 115200)
        baud = USERIAL_BAUD_115200;
    else if (line_speed == 57600)
        baud = USERIAL_BAUD_57600;
    else if (line_speed == 19200)
        baud = USERIAL_BAUD_19200;
    else if (line_speed == 9600)
        baud = USERIAL_BAUD_9600;
    else if (line_speed == 1200)
        baud = USERIAL_BAUD_1200;
    else if (line_speed == 600)
        baud = USERIAL_BAUD_600;
    else
    {
        ALOGE("userial vendor: unsupported baud speed %d", line_speed);
        baud = USERIAL_BAUD_115200;
    }
    ALOGE("userial vendor: set baud speed %d", line_speed);
    return baud;
}


/*******************************************************************************
**
** Function         hw_strncmp
**
** Description      Used to compare two strings in caseless
**
** Returns          0: match, otherwise: not match
**
*******************************************************************************/
static int hw_strncmp(const char *p_str1, const char *p_str2, const int len)
{
    int i;

    if (!p_str1 || !p_str2)
        return (1);

    for (i = 0; i < len; i++)
    {
        if (toupper(p_str1[i]) != toupper(p_str2[i]))
            return (i + 1);
    }

    return 0;
}

/*******************************************************************************
**
** Function         hw_config_findpatch
**
** Description      Search for a proper firmware patch file
**                  The selected firmware patch file name with full path
**                  will be stored in the input string parameter, i.e.
**                  p_chip_id_str, when returns.
**
** Returns          TRUE when found the target patch file, otherwise FALSE
**
*******************************************************************************/
static uint8_t hw_config_findpatch(char *p_chip_id_str)
{
    DIR *dirp;
    struct dirent *dp;
    int filenamelen;
    uint8_t retval = FALSE;

    BTHWDBG("Target name = [%s]", p_chip_id_str);

    if (strlen(fw_patchfile_name) > 0)
    {
        /* If specific filepath and filename have been given in run-time
         * configuration /etc/bluetooth/bt_vendor.conf file, we will use them
         * to concatenate the filename to open rather than searching a file
         * matching to chipset name in the fw_patchfile_path folder.
         */
        sprintf(p_chip_id_str, "%s", fw_patchfile_path);
        if (fw_patchfile_path[strlen(fw_patchfile_path) - 1] != '/')
        {
            strcat(p_chip_id_str, "/");
        }
        strcat(p_chip_id_str, fw_patchfile_name);

        ALOGI("FW patchfile: %s", p_chip_id_str);
        return TRUE;
    }

    if ((dirp = opendir(fw_patchfile_path)) != NULL)
    {
        /* Fetch next filename in patchfile directory */
        while ((dp = readdir(dirp)) != NULL)
        {
            /* Check if filename starts with chip-id name */
            if ((hw_strncmp(dp->d_name, p_chip_id_str, strlen(p_chip_id_str)) \
                ) == 0)
            {
                /* Check if it has .hcd extension */
                filenamelen = strlen(dp->d_name);
                if ((filenamelen >= FW_PATCHFILE_EXTENSION_LEN) &&
                        ((hw_strncmp(
                              &dp->d_name[filenamelen - FW_PATCHFILE_EXTENSION_LEN], \
                              FW_PATCHFILE_EXTENSION, \
                              FW_PATCHFILE_EXTENSION_LEN) \
                         ) == 0))
                {
                    ALOGI("Found patchfile: %s/%s", \
                          fw_patchfile_path, dp->d_name);

                    /* Make sure length does not exceed maximum */
                    if ((filenamelen + strlen(fw_patchfile_path)) > \
                            FW_PATCHFILE_PATH_MAXLEN)
                    {
                        ALOGE("Invalid patchfile name (too long)");
                    }
                    else
                    {
                        memset(p_chip_id_str, 0, FW_PATCHFILE_PATH_MAXLEN);
                        /* Found patchfile. Store location and name */
                        strcpy(p_chip_id_str, fw_patchfile_path);
                        if (fw_patchfile_path[ \
                                               strlen(fw_patchfile_path) - 1 \
                                             ] != '/')
                        {
                            strcat(p_chip_id_str, "/");
                        }
                        strcat(p_chip_id_str, dp->d_name);
                        retval = TRUE;
                    }
                    break;
                }
            }
        }

        closedir(dirp);

        if (retval == FALSE)
        {
            /* Try again chip name without revision info */

            int len = strlen(p_chip_id_str);
            char *p = p_chip_id_str + len - 1;

            /* Scan backward and look for the first alphabet
             * which is not M or m
             */
            while (len > 3) // BCM****
            {
                if ((isdigit(*p) == 0) && (*p != 'M') && (*p != 'm'))
                    break;

                p--;
                len--;
            }

            if (len > 3)
            {
                *p = 0;
                retval = hw_config_findpatch(p_chip_id_str);
            }
        }
    }
    else
    {
        ALOGE("Could not open %s", fw_patchfile_path);
    }

    return (retval);
}

/*******************************************************************************
**
** Function         hw_config_set_bdaddr
**
** Description      Program controller's Bluetooth Device Address
**
** Returns          TRUE, if valid address is sent
**                  FALSE, otherwise
**
*******************************************************************************/
static uint8_t hw_config_set_bdaddr(HC_BT_HDR *p_buf)
{
    uint8_t retval = FALSE;
    uint8_t *p = (uint8_t *)(p_buf + 1);

    BTHWDBG("Setting local bd addr to %02X:%02X:%02X:%02X:%02X:%02X",
            vnd_local_bd_addr[0], vnd_local_bd_addr[1], vnd_local_bd_addr[2],
            vnd_local_bd_addr[3], vnd_local_bd_addr[4], vnd_local_bd_addr[5]);

    UINT16_TO_STREAM(p, HCI_VSC_WRITE_BD_ADDR);
    *p++ = BD_ADDR_LEN; /* parameter length */
    *p++ = vnd_local_bd_addr[5];
    *p++ = vnd_local_bd_addr[4];
    *p++ = vnd_local_bd_addr[3];
    *p++ = vnd_local_bd_addr[2];
    *p++ = vnd_local_bd_addr[1];
    *p = vnd_local_bd_addr[0];

    p_buf->len = HCI_CMD_PREAMBLE_SIZE + BD_ADDR_LEN;
    hw_cfg_cb.state = HW_CFG_SET_BD_ADDR;

    retval = bt_vendor_cbacks->xmit_cb(HCI_VSC_WRITE_BD_ADDR, p_buf, \
                                       hw_config_cback);

    return (retval);
}

#if (USE_CONTROLLER_BDADDR == TRUE)
/*******************************************************************************
**
** Function         hw_config_read_bdaddr
**
** Description      Read controller's Bluetooth Device Address
**
** Returns          TRUE, if valid address is sent
**                  FALSE, otherwise
**
*******************************************************************************/
static uint8_t hw_config_read_bdaddr(HC_BT_HDR *p_buf)
{
    uint8_t retval = FALSE;
    uint8_t *p = (uint8_t *)(p_buf + 1);

    UINT16_TO_STREAM(p, HCI_READ_LOCAL_BDADDR);
    *p = 0; /* parameter length */

    p_buf->len = HCI_CMD_PREAMBLE_SIZE;
    hw_cfg_cb.state = HW_CFG_READ_BD_ADDR;

    retval = bt_vendor_cbacks->xmit_cb(HCI_READ_LOCAL_BDADDR, p_buf, \
                                       hw_config_cback);

    return (retval);
}
#endif // (USE_CONTROLLER_BDADDR == TRUE)


/*******************************************************************************
**
** Function         hw_config_set_rf_params
**
** Description      Config rf parameters to controller
**
** Returns
**
**
*******************************************************************************/
static uint8_t hw_config_set_rf_params(HC_BT_HDR *p_buf)
{
    uint8_t retval = FALSE;
    uint8_t *p = (uint8_t *)(p_buf + 1);
    uint8_t set_rf[8] = { 0 };
    int size = 0;
    uint8_t *q;
    //antenna_num = amlbt_rftype;

    BTHWDBG("antenna number=%d sink mode=%d", amlbt_rftype, amlbt_btsink);

    UINT16_TO_STREAM(p, TCI_WRITE_REG);
    *p++ = 8;                       /* parameter length */
    UINT32_TO_STREAM(p, REG_PMU_POWER_CFG);  /* addr */
    if (amlbt_rftype == AML_SINGLE_ANTENNA)
    {
        UINT32_TO_STREAM(p, (unsigned int)((0x1 << BIT_RF_NUM) | (amlbt_btsink << BT_SINK_MODE)));
    }
    else if (amlbt_rftype == AML_DOUBLE_ANTENNA)
    {
        UINT32_TO_STREAM(p, (unsigned int)((0x2 << BIT_RF_NUM) | (amlbt_btsink << BT_SINK_MODE)));
    }

    p_buf->len = HCI_CMD_PREAMBLE_SIZE + 8;

    hw_cfg_cb.state = HW_CFG_AML_DOWNLOAD_FIRMWARE_STRAT_CPU_UART_BEFORE;

    retval = bt_vendor_cbacks->xmit_cb(TCI_WRITE_REG, \
                                       p_buf, hw_config_cback);

    if ((amlbt_transtype.family_id >= AML_W1U) &&
            (amlbt_transtype.interface != AML_INTF_USB))
    {
        q = set_rf;
        UINT32_TO_STREAM(q, REG_PMU_POWER_CFG);  /* addr */
        if (amlbt_rftype == AML_SINGLE_ANTENNA)
        {
            UINT32_TO_STREAM(q, (unsigned int)((0x1 << BIT_RF_NUM) | (amlbt_btsink << BT_SINK_MODE)));
        }
        else if (amlbt_rftype == AML_DOUBLE_ANTENNA)
        {
            UINT32_TO_STREAM(q, (unsigned int)((0x2 << BIT_RF_NUM) | (amlbt_btsink << BT_SINK_MODE)));
        }
        if (amlbt_transtype.interface == AML_INTF_PCIE \
                                         || (amlbt_transtype.interface == AML_INTF_SDIO && amlbt_transtype.family_id == AML_W1U))
        {
            bt_sdio_fd = userial_vendor_uart_open();
            if (bt_sdio_fd < 0)
            {
                BTHWDBG("hw_config_set_rf_params open failed!");
            }
        }
        size = write(bt_sdio_fd, set_rf, sizeof(set_rf));
        if (size < 0)
        {
            BTHWDBG("write failed!");
        }
        if (bt_sdio_fd)
        {
            close(bt_sdio_fd);
            bt_sdio_fd = 0;
        }
    }

    return (retval);
}

static int hw_config_get_iccm_size(void)
{
    int fd = 0;
    unsigned int iccm_size = 0;
    int size = 0;
    char *file = NULL;

    BTHWDBG("hw_config_get_iccm_size chip(%d:%d:%d)\n", amlbt_transtype.family_id,
            amlbt_transtype.family_rev, amlbt_transtype.interface);

    file = amlbt_fw_bin[amlbt_transtype.family_id][amlbt_transtype.interface];
    if (file == NULL)
    {
        BTHWDBG("hw_config_get_iccm_size get fw bin error!\n");
        return 0;
    }

    if ((fd = open_file(file, O_RDONLY)) < 0)
        return 0;
    size = read(fd, &iccm_size, 4);
    if (size < 0)
    {
        BTHWDBG("---------hw_config_get_iccm_size read error!---------");
        close(fd);
        return 0;
    }
    close(fd);

    BTHWDBG("---------hw_config_get_iccm_size iccm_size %d---------\n", iccm_size);
    return iccm_size;
}

static int hw_config_get_dccm_size(void)
{
    int fd = 0;
    unsigned int dccm_size = 0;
    int size = 0;
    char *file = NULL;

    BTHWDBG("hw_config_get_dccm_size chip(%d:%d:%d)\n", amlbt_transtype.family_id,
            amlbt_transtype.family_rev, amlbt_transtype.interface);

    file = amlbt_fw_bin[amlbt_transtype.family_id][amlbt_transtype.interface];
    if (file == NULL)
    {
        BTHWDBG("hw_config_get_dccm_size get fw bin error!\n");
        return 0;
    }

    if ((fd = open_file(file, O_RDONLY)) < 0)
        return 0;
    /*skip 4 bytes iccm len*/
    size = read(fd, &dccm_size, 4);
    if (size < 0)
    {
        BTHWDBG("---------hw_config_get_dccm_size read error!---------");
        close(fd);
        return 0;
    }
    size = read(fd, &dccm_size, 4);
    if (size < 0)
    {
        BTHWDBG("---------hw_config_get_dccm_size read error!---------");
        close(fd);
        return 0;
    }
    close(fd);

    BTHWDBG("---------hw_config_get_dccm_size dccm_size %d---------\n", dccm_size);
    return dccm_size;
}

#ifdef AML_DOWNLOADFW_UART
static void hw_get_bin_size(void)
{
#ifdef AML_FW_FILE
    len_iccm = sizeof(BT_fwICCM);
#endif
#ifdef AML_FW_BIN
    len_iccm = hw_config_get_iccm_size();
    iccm_size = len_iccm;
#endif
    if (amlbt_transtype.family_id == AML_W2L)
    {
        len_iccm -= 384 * 1024;
        offset_iccm = 384 * 1024;
    }
    else
    {
        len_iccm -= 256 * 1024;
        offset_iccm = 256 * 1024;
    }
#ifdef AML_FW_FILE
    len_dccm = sizeof(BT_fwDCCM);
#endif
#ifdef AML_FW_BIN
    len_dccm = hw_config_get_dccm_size();
    dccm_size = len_dccm;
#endif
    offset_dccm = 0;
#ifdef AML_FW_BIN
    p_iccm_buf = malloc(iccm_size);
    p_dccm_buf = malloc(dccm_size);
#endif
}
#endif

timer_t timerid;
static unsigned int fw_alive = 1;
static void hw_start_recovery(void);

void hw_detect_fw_cback(void *p_mem)
{
    HC_BT_HDR *p_evt_buf = (HC_BT_HDR *)p_mem;
    //uint8_t *p, status;
    //uint16_t opcode;

    fw_alive = 1;
    if (bt_vendor_cbacks)
    {
        bt_vendor_cbacks->dealloc(p_evt_buf);
    }
    //status = *((uint8_t *)(p_evt_buf + 1) + HCI_EVT_CMD_CMPL_STATUS_RET_BYTE);
    //p = (uint8_t *)(p_evt_buf + 1) + HCI_EVT_CMD_CMPL_OPCODE;
    //STREAM_TO_UINT16(opcode, p);
    //BTHWDBG("---------hw_detect_fw status %#x, %#x--------", status, opcode);
}

static void hw_detect_fw(void)
{
    HC_BT_HDR *p_buf = NULL;
    uint8_t *p;

    if (bt_vendor_cbacks)
    {
        p_buf = (HC_BT_HDR *)bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + HCI_CMD_PREAMBLE_SIZE);
        if (p_buf)
        {
            p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
            p_buf->offset = 0;
            p_buf->layer_specific = 0;
            p = (uint8_t *)(p_buf + 1);
            UINT16_TO_STREAM(p, HCI_CHECK_CMD);
            *p = 0;
            p_buf->len = HCI_CMD_PREAMBLE_SIZE;

            bt_vendor_cbacks->xmit_cb(HCI_CHECK_CMD, \
                                      p_buf, hw_detect_fw_cback);
            fw_alive = 0;
        }
    }
    else
    {
        BTHWDBG("---------hw_detect_fw error--------");
    }
}

void hw_stop_recovery(void)
{
    if (bt_recovery_timer)
    {
        timer_delete(bt_recovery_timer);
        bt_recovery_timer = 0;
    }
}

static void hw_start_recovery_handle(int param)
{
    BTHWDBG("---------hw_start_recovery_handle %d--------", fw_alive);

    if (fw_alive)
    {
        bt_power = 1;
        hw_detect_fw();
    }
    else
    {
        BTHWDBG("---------hw recovery--------");
        bt_start_config = 0;
        bt_power = 0;
        hw_stop_recovery();
        if (bt_vendor_cbacks && bt_vendor_cbacks->xmit_cb)
        {
            bt_vendor_cbacks->xmit_cb(HCI_RECOVERY_CMD, NULL, NULL);
        }
        else
        {
            BTHWDBG("---------bt_vendor_cbacks NULL--------");
        }
    }
}

static void hw_start_recovery(void)
{
#if 1
    struct sigevent evp;
    struct itimerspec ts;
    timer_t timer;
    int ret;

    memset(&evp, 0, sizeof(evp));
    evp.sigev_value.sival_ptr = &timer;
    evp.sigev_notify = SIGEV_THREAD;
    evp.sigev_notify_function = hw_start_recovery_handle;
    evp.sigev_value.sival_int = 5;

    ret = timer_create(CLOCK_REALTIME, &evp, &timer);
    if (!ret)
    {
        ts.it_interval.tv_sec = 1;
        ts.it_interval.tv_nsec = 0;
        ts.it_value.tv_sec = 3;
        ts.it_value.tv_nsec = 0;
        ret = timer_settime(timer, 0, &ts, NULL);
        bt_recovery_timer = timer;
    }
    else
    {
        BTHWDBG("---------timer_create error!--------");
    }
    BTHWDBG("---------hw_start_recovery--------");
#endif
}

uint8_t hw_cfg_write_fw_mode(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p)
{
    uint8_t is_proceeding = FALSE;

#ifdef UART_4M
    userial_vendor_set_baud(\
                            line_speed_to_userial_baud(4000000) \
                           );
#endif
#ifdef UART_2M
    userial_vendor_set_baud(\
                            line_speed_to_userial_baud(2000000) \
                           );
#endif

    UINT16_TO_STREAM(p, TCI_WRITE_REG);
    *p++ = 8;

    UINT32_TO_STREAM(p, REG_FW_MODE);
    UINT32_TO_STREAM(p, amlbt_fw_mode); /*enable download event */

    p_buf->len = HCI_CMD_PREAMBLE_SIZE + \
                 8;
    hw_cfg_cb.state = HW_CFG_AML_UPDATE_BAUDRATE_UART;

    is_proceeding = bt_vendor_cbacks->xmit_cb(TCI_WRITE_REG, \
                    p_buf, hw_config_cback);
    return is_proceeding;
}

uint8_t hw_cfc_null_2(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p)
{
    BTHWDBG("---------hw_cfc_null 2--------");
    return FALSE;
}

uint8_t hw_cfg_start(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p)
{
    uint8_t is_proceeding = FALSE;

    if (amlbt_transtype.interface == AML_INTF_USB)
    {
        is_proceeding = hw_config_set_bdaddr(p_buf);
    }
    else
    {
        ms_delay(350);
        p_buf->len = HCI_CMD_PREAMBLE_SIZE;
        UINT16_TO_STREAM(p, HCI_RESET);
        *p = 0; /* parameter length */
        hw_cfg_cb.state = HW_CFG_AML_UPDATE_BAUDRATE;
        is_proceeding = bt_vendor_cbacks->xmit_cb(HCI_RESET, p_buf, hw_config_cback);
    }

    return is_proceeding;
}

uint8_t hw_cfg_set_uart_clock(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p)
{
    uint8_t is_proceeding = FALSE;

    UINT16_TO_STREAM(p, HCI_VSC_UPDATE_BAUDRATE);
    *p++ = UPDATE_BAUDRATE_CMD_PARAM_SIZE;
    *p++ = 0;
    *p++ = 0;
    UINT32_TO_STREAM(p, UART_TARGET_BAUD_RATE);

    p_buf->len = HCI_CMD_PREAMBLE_SIZE + \
                 UPDATE_BAUDRATE_CMD_PARAM_SIZE;
    hw_cfg_cb.state = (hw_cfg_cb.f_set_baud_2) ? \
                      HW_CFG_SET_UART_BAUD_2 : HW_CFG_SET_UART_BAUD_1;

    is_proceeding = bt_vendor_cbacks->xmit_cb(HCI_VSC_UPDATE_BAUDRATE, \
                    p_buf, hw_config_cback);

    return is_proceeding;
}

uint8_t hw_cfg_set_uart_baud1(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p)
{
    uint8_t is_proceeding = FALSE;

    /* update baud rate of host's UART port */
    BTHWDBG("bt vendor lib: set UART baud %i", UART_TARGET_BAUD_RATE);
    userial_vendor_set_baud(\
                            line_speed_to_userial_baud(UART_TARGET_BAUD_RATE) \
                           );

    /* read local name */
    UINT16_TO_STREAM(p, HCI_READ_LOCAL_NAME);
    *p = 0; /* parameter length */

    p_buf->len = HCI_CMD_PREAMBLE_SIZE;
    hw_cfg_cb.state = HW_CFG_READ_LOCAL_NAME;

    is_proceeding = bt_vendor_cbacks->xmit_cb(HCI_READ_LOCAL_NAME, \
                    p_buf, hw_config_cback);

    return is_proceeding;
}

uint8_t hw_cfg_read_local_name(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p)
{
    int i;
    uint8_t is_proceeding = FALSE;
    char *p_name, *p_tmp;
    HC_BT_HDR *p_evt_buf = (HC_BT_HDR *)p_mem;

    p_tmp = p_name = (char *)(p_evt_buf + 1) + \
                     HCI_EVT_CMD_CMPL_LOCAL_NAME_STRING;

    for (i = 0; (i < LOCAL_NAME_BUFFER_LEN) || (*(p_name + i) != 0); i++)
        *(p_name + i) = toupper(*(p_name + i));

    if ((p_name = strstr(p_name, "BCM")) != NULL)
    {
        strncpy(hw_cfg_cb.local_chip_name, p_name, \
                LOCAL_NAME_BUFFER_LEN - 1);
    }
    else
    {
        strncpy(hw_cfg_cb.local_chip_name, "UNKNOWN", \
                LOCAL_NAME_BUFFER_LEN - 1);
        p_name = p_tmp;
    }

    hw_cfg_cb.local_chip_name[LOCAL_NAME_BUFFER_LEN - 1] = 0;

    BTHWDBG("Chipset %s", hw_cfg_cb.local_chip_name);
#ifdef AML_DOWNLOADFW_UART
    if (hw_config_findpatch(p_name))
#endif
    {
        if ((hw_cfg_cb.fw_fd = open(p_name, O_RDONLY)) == -1)
        {
            ALOGE("vendor lib preload failed to open [%s]", p_name);
        }
        else
        {
            /* vsc_download_minidriver */
            UINT16_TO_STREAM(p, HCI_VSC_DOWNLOAD_MINIDRV);
            *p = 0; /* parameter length */

            p_buf->len = HCI_CMD_PREAMBLE_SIZE;
            //hw_cfg_cb.state = HW_CFG_DL_MINIDRIVER;

            is_proceeding = bt_vendor_cbacks->xmit_cb(\
                            HCI_VSC_DOWNLOAD_MINIDRV, p_buf, \
                            hw_config_cback);
        }
    }
    else
    {
        ALOGE(\
              "vendor lib preload failed to locate firmware patch file" \
             );
    }

    if (is_proceeding == FALSE)
    {
        is_proceeding = hw_config_set_bdaddr(p_buf);
    }

    return is_proceeding;
}
#if 0
uint8_t hw_cfg_dl_minidriver(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p)
{
    uint8_t is_proceeding = FALSE;
    int delay = 100;
    uint16_t opcode = 0;

    /* give time for placing firmware in download mode */
    ms_delay(50);
    hw_cfg_cb.state = HW_CFG_DL_FW_PATCH;

    /* fall through intentionally */
    p_buf->len = read(hw_cfg_cb.fw_fd, p, HCI_CMD_PREAMBLE_SIZE);
    if (p_buf->len > 0)
    {
        if ((p_buf->len < HCI_CMD_PREAMBLE_SIZE) || \
                (opcode == HCI_VSC_LAUNCH_RAM))
        {
            ALOGW("firmware patch file might be altered!");
        }
        else
        {
            p_buf->len += read(hw_cfg_cb.fw_fd, \
                               p + HCI_CMD_PREAMBLE_SIZE, \
                               * (p + HCD_REC_PAYLOAD_LEN_BYTE));
            STREAM_TO_UINT16(opcode, p);
            is_proceeding = bt_vendor_cbacks->xmit_cb(opcode, \
                            p_buf, hw_config_cback);
            return is_proceeding;
        }
    }

    close(hw_cfg_cb.fw_fd);
    hw_cfg_cb.fw_fd = -1;

    /* Normally the firmware patch configuration file
     * sets the new starting baud rate at 115200.
     * So, we need update host's baud rate accordingly.
     */
    userial_vendor_set_baud(USERIAL_BAUD_115200);

    /* Next, we would like to boost baud rate up again
     * to desired working speed.
     */
    hw_cfg_cb.f_set_baud_2 = TRUE;

    /* Check if we need to pause a few hundred milliseconds
     * before sending down any HCI command.
     */
    delay = look_up_fw_settlement_delay();
    BTHWDBG("Setting fw settlement delay to %d ", delay);
    ms_delay(delay);

    p_buf->len = HCI_CMD_PREAMBLE_SIZE;
    UINT16_TO_STREAM(p, HCI_RESET);
    *p = 0; /* parameter length */
    hw_cfg_cb.state = HW_CFG_START;
    is_proceeding = bt_vendor_cbacks->xmit_cb(HCI_RESET, p_buf, hw_config_cback);

    return is_proceeding;
}
#endif
uint8_t hw_cfg_set_uart_baud_2(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p)
{
    uint8_t is_proceeding = FALSE;

    userial_vendor_set_baud(\
                            line_speed_to_userial_baud(UART_TARGET_BAUD_RATE) \
                           );
    ms_delay(10);
#if (USE_CONTROLLER_BDADDR == TRUE)
    is_proceeding = hw_config_read_bdaddr(p_buf);
#else
    is_proceeding = hw_config_set_bdaddr(p_buf);
#endif

    return is_proceeding;
}

uint8_t hw_cfg_update_baudrate(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p)
{
    uint8_t is_proceeding = FALSE;

    UINT16_TO_STREAM(p, HCI_VSC_UPDATE_BAUDRATE);
    *p++ = UPDATE_BAUDRATE_CMD_PARAM_SIZE;
    *p++ = 0;
    *p++ = 0;
    UINT32_TO_STREAM(p, UART_TARGET_BAUD_RATE);

    p_buf->len = HCI_CMD_PREAMBLE_SIZE + \
                 UPDATE_BAUDRATE_CMD_PARAM_SIZE;
    hw_cfg_cb.state = HW_CFG_AML_SET_BAUDRATE;

    is_proceeding = bt_vendor_cbacks->xmit_cb(HCI_VSC_UPDATE_BAUDRATE, \
                    p_buf, hw_config_cback);

    return is_proceeding;
}

uint8_t hw_cfg_update_baudrate_uart(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p)
{
    char *file = NULL;
    uint8_t is_proceeding = FALSE;
    char tempBuf[8];
    int size = 0;

    if (amlbt_transtype.family_id != AML_W2L)
    {
#ifdef UART_4M
        userial_vendor_set_baud(\
                                line_speed_to_userial_baud(4000000) \
                               );
#endif
#ifdef UART_2M
        userial_vendor_set_baud(\
                                line_speed_to_userial_baud(2000000) \
                               );
#endif
    }

    if (amlbt_transtype.family_id == AML_UNKNOWN)
    {
        hw_read_type(p_buf);
        is_proceeding = TRUE;
    }
    else
    {
#ifdef AML_DOWNLOADFW_UART
        hw_get_bin_size();
#endif
        UINT16_TO_STREAM(p, TCI_WRITE_REG);
        *p++ = 8;

        UINT32_TO_STREAM(p, 0xa70014);
        UINT32_TO_STREAM(p, 0x1000000); /*enable download event */

        p_buf->len = HCI_CMD_PREAMBLE_SIZE + \
                     8;
        hw_cfg_cb.state = HW_CFG_AML_DOWNLOAD_FIRMWARE_TEST_3_UART;

        is_proceeding = bt_vendor_cbacks->xmit_cb(TCI_WRITE_REG, \
                        p_buf, hw_config_cback);

        BTHWDBG("uart_target_baud_rate = %d", UART_TARGET_BAUD_RATE);
        file = amlbt_fw_bin[amlbt_transtype.family_id][amlbt_transtype.interface];

        if ((fw_fd = open_file(file, O_RDONLY)) > 0)
        {
            size = read(fw_fd, tempBuf, 8);
            if (size < 0)
            {
                ALOGE("In %s, Read head failed:%s", __FUNCTION__, strerror(errno));
                close(fw_fd);
                return FALSE;
            }
            size = read(fw_fd, p_iccm_buf, iccm_size);
            if (size < 0)
            {
                ALOGE("In %s, Read iccm failed:%s", __FUNCTION__, strerror(errno));
                close(fw_fd);
                return FALSE;
            }
            size = read(fw_fd, p_dccm_buf, dccm_size);
            if (size < 0)
            {
                ALOGE("In %s, Read dccm failed:%s", __FUNCTION__, strerror(errno));
                close(fw_fd);
                return FALSE;
            }
        }
    }

    return is_proceeding;
}

uint8_t hw_cfg_get_reg(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p)
{
    uint8_t is_proceeding = FALSE;
    char *file = NULL;
    char tempBuf[8];
    int size = 0;

    hw_read_type_cback(p_mem);
#ifdef AML_DOWNLOADFW_UART
    hw_get_bin_size();
#endif
    UINT16_TO_STREAM(p, TCI_WRITE_REG);
    *p++ = 8;

    UINT32_TO_STREAM(p, 0xa70014);
    UINT32_TO_STREAM(p, 0x1000000); /*enable download event */

    p_buf->len = HCI_CMD_PREAMBLE_SIZE + \
                 8;
    hw_cfg_cb.state = HW_CFG_AML_DOWNLOAD_FIRMWARE_TEST_3_UART;

    is_proceeding = bt_vendor_cbacks->xmit_cb(TCI_WRITE_REG, \
                    p_buf, hw_config_cback);

    BTHWDBG("uart_target_baud_rate = %d", UART_TARGET_BAUD_RATE);
    file = amlbt_fw_bin[amlbt_transtype.family_id][amlbt_transtype.interface];

    if ((fw_fd = open_file(file, O_RDONLY)) > 0)
    {
        size = read(fw_fd, tempBuf, 8);
        if (size < 0)
        {
            ALOGE("In %s, Read head failed:%s", __FUNCTION__, strerror(errno));
            close(fw_fd);
            return FALSE;
        }
        size = read(fw_fd, p_iccm_buf, iccm_size);
        if (size < 0)
        {
            ALOGE("In %s, Read iccm failed:%s", __FUNCTION__, strerror(errno));
            close(fw_fd);
            return FALSE;
        }
        size = read(fw_fd, p_dccm_buf, dccm_size);
        if (size < 0)
        {
            ALOGE("In %s, Read dccm failed:%s", __FUNCTION__, strerror(errno));
            close(fw_fd);
            return FALSE;
        }
    }

    return is_proceeding;
}

uint8_t hw_cfg_download_firmware_test_3_uart(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p)
{
    uint8_t is_proceeding = FALSE;

    UINT16_TO_STREAM(p, TCI_WRITE_REG);
    *p++ = 8;						/* parameter length */
    UINT32_TO_STREAM(p, REG_RAM_PD_SHUTDWONW_SW);	/* addr */
    UINT32_TO_STREAM(p, 0x0);		/* data 4M */

    p_buf->len = HCI_CMD_PREAMBLE_SIZE + \
                 8;
    hw_cfg_cb.state = HW_CFG_AML_DOWNLOAD_FIRMWARE_TEST_2_UART;

    is_proceeding = bt_vendor_cbacks->xmit_cb(TCI_WRITE_REG, \
                    p_buf, hw_config_cback);

    return is_proceeding;
}

uint8_t hw_cfg_download_firmware_test_2_uart(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p)
{
    uint8_t is_proceeding = FALSE;

    UINT16_TO_STREAM(p, TCI_READ_REG);
    *p++ = 4;                       /* parameter length */
    UINT32_TO_STREAM(p, REG_RAM_PD_SHUTDWONW_SW);  /* addr */

    p_buf->len = HCI_CMD_PREAMBLE_SIZE + \
                 4;
    hw_cfg_cb.state = HW_CFG_AML_DOWNLOAD_FIRMWARE_ICCM_UART;

    is_proceeding = bt_vendor_cbacks->xmit_cb(TCI_READ_REG, \
                    p_buf, hw_config_cback);

    return is_proceeding;
}


uint8_t hw_cfg_download_firmware_iccm_uart(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p)
{
    uint8_t is_proceeding = FALSE;
    int i;
    unsigned char rsp[260];
    unsigned char download_cmd[255] = {0};
    uint8_t *p_dn = 0;

    if (amlbt_transtype.family_id > AML_W1U && amlbt_transtype.interface == AML_INTF_SDIO)
    {
        bt_sdio_fd = userial_vendor_uart_open();
        if (bt_sdio_fd < 0)
        {
            BTHWDBG("hw_cfg_download_firmware_iccm_uart open failed!");
            return FALSE;
        }

        while (len_iccm)
        {
            p_dn = &download_cmd[0];
            data_len_iccm = (len_iccm > RW_OPERATION_SIZE) ? RW_OPERATION_SIZE : len_iccm;
            cmd_len_iccm = data_len_iccm + 4;		  // addr
            UINT16_TO_STREAM(p_dn, TCI_DOWNLOAD_BT_FW);
            *p_dn++ = cmd_len_iccm; /* parameter length */
            UINT32_TO_STREAM(p_dn, ICCM_RAM_BASE + offset_iccm);
            for (i = 0; i < (int)data_len_iccm; i += 1)
            {
                //p_dn[i] = *(p_iccm_buf + offset_iccm + i);/* data */
                download_cmd[7 + i] = *(p_iccm_buf + offset_iccm + i);
            }
            aml_hci_send_cmd_download(bt_sdio_fd, (unsigned char *)download_cmd, data_len_iccm + 7, (unsigned char *)rsp);
            cnt++;
            offset_iccm += data_len_iccm;
            len_iccm -= data_len_iccm;

        if (len_iccm <= 0)
        {
            BTHWDBG("iccm write over successfully %#x", cnt);
            cnt = 0;
        }
    }

    while (len_dccm)
    {
        p_dn = &download_cmd[0];
        data_len_dccm = (len_dccm > RW_OPERATION_SIZE) ? RW_OPERATION_SIZE : len_dccm;
        cmd_len_dccm = data_len_dccm + 4;
        UINT16_TO_STREAM(p_dn, TCI_DOWNLOAD_BT_FW);
        *p_dn++ = cmd_len_dccm;
        UINT32_TO_STREAM(p_dn, DCCM_RAM_BASE + offset_dccm);
        for (i = 0; i < (int)data_len_dccm; i += 1)
        {
            download_cmd[7 + i] = *(p_dccm_buf + offset_dccm + i);
            //p_dn[i] = *(p_dccm_buf + offset_dccm + i);
        }
        aml_hci_send_cmd_download(bt_sdio_fd, (unsigned char *)download_cmd, data_len_dccm + 7, (unsigned char *)rsp);
        cnt++;
        offset_dccm += data_len_dccm;
        len_dccm -= data_len_dccm;

        if (len_dccm <= 0)
        {
            BTHWDBG("dccm write over successfully %#x", cnt);
            cnt = 0;
        }
    }

    UINT16_TO_STREAM(p, TCI_WRITE_REG);
    *p++ = 8;
    UINT32_TO_STREAM(p, 0xa70014);
    UINT32_TO_STREAM(p, 0x0000000);

        p_buf->len = HCI_CMD_PREAMBLE_SIZE + \
                     8;
        hw_cfg_cb.state = HW_CFG_AML_CONFIG_RF_CALIBRATION;
        free(p_iccm_buf);
        free(p_dccm_buf);
        if (fw_fd != -1)
        {
            close(fw_fd);
            fw_fd = -1;
        }
        is_proceeding = bt_vendor_cbacks->xmit_cb(TCI_WRITE_REG, \
                        p_buf, hw_config_cback);
        BTHWDBG("HW_CFG_AML_DOWNLOAD_FIRMWARE_CLOSE_EVENT");
    }
    else
    {
        if (len_iccm <= 0)
        {
            ALOGW(" iccm write over ");
            hw_cfg_cb.state = HW_CFG_AML_DOWNLOAD_FIRMWARE_DCCM_UART;
            cnt = 0;
            return is_proceeding;
        }
        data_len_iccm = (len_iccm > RW_OPERATION_SIZE) ? RW_OPERATION_SIZE : len_iccm;
        cmd_len_iccm = data_len_iccm + 4;		  // addr


        UINT16_TO_STREAM(p, TCI_DOWNLOAD_BT_FW);
        *p++ = cmd_len_iccm; /* parameter length */
        UINT32_TO_STREAM(p, ICCM_RAM_BASE + offset_iccm);
        for (i = 0; i < (int)data_len_iccm; i += 1)
        {
            p[i] = *(p_iccm_buf + offset_iccm + i);/* data */
        }

        p_buf->len = HCI_CMD_PREAMBLE_SIZE + \
                     cmd_len_iccm;

        is_proceeding = bt_vendor_cbacks->xmit_cb(TCI_DOWNLOAD_BT_FW, \
                        p_buf, hw_config_cback);
        cnt++;
        offset_iccm += data_len_iccm;
        len_iccm -= data_len_iccm;

        if (len_iccm <= 0)
        {
            BTHWDBG("iccm write over successfully ");
            hw_cfg_cb.state = HW_CFG_AML_DOWNLOAD_FIRMWARE_DCCM_UART;
            cnt = 0;
        }
    }

    return is_proceeding;
}


uint8_t hw_cfg_download_firmware_dccm_uart(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p)
{
    uint8_t is_proceeding = FALSE;
    int i;

    if (len_dccm <= 0)
    {
        ALOGW("dccm write over ");
        hw_cfg_cb.state = HW_CFG_AML_DOWNLOAD_FIRMWARE_CLOSE_EVENT;
        return is_proceeding;
    }
    data_len_dccm = (len_dccm > RW_OPERATION_SIZE) ? RW_OPERATION_SIZE : len_dccm;
    cmd_len_dccm = data_len_dccm + 4;


    UINT16_TO_STREAM(p, TCI_DOWNLOAD_BT_FW);
    *p++ = cmd_len_dccm;
    UINT32_TO_STREAM(p, DCCM_RAM_BASE + offset_dccm);
    for (i = 0; i < (int)data_len_dccm; i += 1)
    {
        p[i] = *(p_dccm_buf + offset_dccm + i);
    }

    p_buf->len = HCI_CMD_PREAMBLE_SIZE + \
                 cmd_len_dccm;

    is_proceeding = bt_vendor_cbacks->xmit_cb(TCI_DOWNLOAD_BT_FW, \
                    p_buf, hw_config_cback);

    cnt++;
    offset_dccm += data_len_dccm;
    len_dccm -= data_len_dccm;

    if (len_dccm <= 0)
    {
        ALOGI("dccm write over successfully. ");
        hw_cfg_cb.state = HW_CFG_AML_DOWNLOAD_FIRMWARE_CLOSE_EVENT;
    }

    return is_proceeding;
}

uint8_t hw_cfg_download_firmware_change_baudrate_uart(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p)
{
    uint8_t is_proceeding = FALSE;

    UINT16_TO_STREAM(p, TCI_UPDATE_UART_BAUDRATE);
    *p++ = 8;
    UINT32_TO_STREAM(p, 0xa30128);
    UINT32_TO_STREAM(p, 0x70cf);

    p_buf->len = HCI_CMD_PREAMBLE_SIZE + \
                 8;
    hw_cfg_cb.state = HW_CFG_AML_DOWNLOAD_FIRMWARE_CLOSE_EVENT;
    ALOGI("rom check finished\n");
    is_proceeding = bt_vendor_cbacks->xmit_cb(TCI_UPDATE_UART_BAUDRATE, \
                    p_buf, hw_config_cback);

    return is_proceeding;
}

uint8_t hw_cfg_download_firmware_close_event(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p)
{
    uint8_t is_proceeding = FALSE;

    UINT16_TO_STREAM(p, TCI_WRITE_REG);
    *p++ = 8;
    UINT32_TO_STREAM(p, 0xa70014);
    UINT32_TO_STREAM(p, 0x0000000);

    p_buf->len = HCI_CMD_PREAMBLE_SIZE + \
                 8;
    hw_cfg_cb.state = HW_CFG_AML_CONFIG_RF_CALIBRATION;
    free(p_iccm_buf);
    free(p_dccm_buf);
    if (fw_fd != -1)
    {
        close(fw_fd);
        fw_fd = -1;
    }
    is_proceeding = bt_vendor_cbacks->xmit_cb(TCI_WRITE_REG, \
                    p_buf, hw_config_cback);
    BTHWDBG("HW_CFG_AML_DOWNLOAD_FIRMWARE_CLOSE_EVENT");

    return is_proceeding;
}

uint8_t hw_cfg_rf_calibration(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p)
{
    uint8_t is_proceeding = FALSE;

    is_proceeding = hw_config_set_rf_params(p_buf);

    if (is_proceeding == FALSE)
    {
        if (hw_cfg_cb.fw_fd != -1)
        {
            close(hw_cfg_cb.fw_fd);
            hw_cfg_cb.fw_fd = -1;
        }
        ALOGE("config rf parameters failed!!!");
    }

    return is_proceeding;
}

uint8_t hw_cfg_download_firmware_start_cpu_uart_before(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p)
{
    uint8_t is_proceeding = FALSE;

    UINT16_TO_STREAM(p, TCI_WRITE_REG);
    *p++ = 8;
    UINT32_TO_STREAM(p, 0xa7000c);
    UINT32_TO_STREAM(p, 0x8000000);

    p_buf->len = HCI_CMD_PREAMBLE_SIZE + \
                 8;
    hw_cfg_cb.state = HW_CFG_AML_DOWNLOAD_FIRMWARE_STRAT_CPU_UART;
    is_proceeding = bt_vendor_cbacks->xmit_cb(TCI_WRITE_REG, \
                    p_buf, hw_config_cback);
    BTHWDBG("HW_CFG_AML_DOWNLOAD_FIRMWARE_STRAT_CPU_UART_BEFORE");

    return is_proceeding;
}

uint8_t hw_cfg_download_firmware_start_cpu_uart(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p)
{
    uint8_t is_proceeding = FALSE;

    UINT16_TO_STREAM(p, TCI_WRITE_REG);
    *p++ = 8;
    UINT32_TO_STREAM(p, REG_DEV_RESET);
    UINT32_TO_STREAM(p, (unsigned int)((BIT_CPU | BIT_MAC | BIT_PHY) << DEV_RESET_SW));

    p_buf->len = HCI_CMD_PREAMBLE_SIZE + \
                 8;
    hw_cfg_cb.state = HW_CFG_SET_PARAMS;

    is_proceeding = bt_vendor_cbacks->xmit_cb(TCI_WRITE_REG, \
                    p_buf, hw_config_cback);
    BTHWDBG("HW_CFG_AML_DOWNLOAD_FIRMWARE_STRAT_CPU_UART");

    return is_proceeding;
}

uint8_t hw_cfg_download_firmware_finish_uart(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p)
{
    uint8_t is_proceeding = FALSE;

    is_proceeding = TRUE;
    usleep(200 * 1000);
    bt_vendor_cbacks->dealloc(p_buf);
    bt_vendor_cbacks->fwcfg_cb(BT_VND_OP_RESULT_SUCCESS);

    hw_cfg_cb.state = 0;
    if (hw_cfg_cb.fw_fd != -1)
    {
        close(hw_cfg_cb.fw_fd);
        hw_cfg_cb.fw_fd = -1;
    }
    BTHWDBG("HW_CFG_AML_DOWNLOAD_FIRMWARE_FINISH_UART");

    return is_proceeding;
}

int hw_cfg_get_rom_offset(void)
{
    char local_ver[128] = {0};
    int offset = 0;

    if (property_get(AML_ROM_CHECK_PROPERTY, local_ver, NULL))
    {
        ALOGE("%s:Failed to set amlbt version in %s", __func__, VENDOR_AMLBTVER_PROPERTY);
    }
    offset = atoi(local_ver);
    ALOGD("%s offset:%#x", __func__, offset);

    return offset;
}

void hw_cfg_set_rom_offset(int offset)
{
    char local_ver[64] = {0};

    ALOGD("%s offset:%#x", __func__, offset);

    sprintf(local_ver, "%d", offset);
    if (property_set(AML_ROM_CHECK_PROPERTY, (char *)local_ver) < 0)
    {
        ALOGE("%s:Failed to set rom offset", __func__);
    }
}

#if 0
int hw_cfg_rom_check_restore(void)
{
    int fd = 0;
    int size = 0;
    int offset = 0;
    char buffer[255] = {0};
    char *p_check = "RomCheck=\0";
    char *ptr;

    if ((fd = open_file(AML_BT_CONFIG_RF_FILE, O_RDONLY)) < 0)
    {
        ALOGE("In %s, Open failed:%s", __FUNCTION__, strerror(errno));
        return -1;
    }

    size = read(fd, buffer, sizeof(buffer));

    if (size < 0)
    {
        ALOGE("In %s, Read failed:%s", __FUNCTION__, strerror(errno));
        close(fd);
        return -1;
    }
    ptr = strstr(buffer, p_check);
    if (ptr == NULL)
    {
        ALOGE("In %s, ptr is NULL", __FUNCTION__);
        close(fd);
        return -1;
    }
    ptr++;
    offset = atoi(ptr);
    ALOGD("%s rom check offset %d", __FUNCTION__, offset);
    close(fd);
    return offset;
}

void hw_cfg_rom_check_save(unsigned int offset)
{
    int fd = 0;
    int size = 0;
    char buffer[255] = {0};
    char *p_check = "RomCheck=\0";
    char *ptr;

    if ((fd = open_file(AML_BT_CONFIG_RF_FILE, O_RDWR)) < 0)
    {
        ALOGE("In %s, Open failed:%s", __FUNCTION__, strerror(errno));
        return;
    }

    size = read(fd, buffer, sizeof(buffer));

    if (size < 0)
    {
        ALOGE("In %s, Read failed:%s", __FUNCTION__, strerror(errno));
        close(fd);
        return;
    }
    ptr = strstr(buffer, p_check);
    if (ptr == NULL)
    {
        ALOGE("In %s, ptr is NULL", __FUNCTION__);
        close(fd);
        return ;
    }
    ptr++;
    sprintf(ptr, "%d", offset);
    size = write(fd, buffer, sizeof(buffer));

    if (size < 0)
    {
        ALOGE("In %s, Write failed:%s", __FUNCTION__, strerror(errno));
        close(fd);
        return ;
    }
    ALOGD("%s rom check save offset %u", __FUNCTION__, offset);
    close(fd);
}
#endif
uint8_t hw_cfg_download_firmware_test_uart(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p)
{
    uint8_t is_proceeding = FALSE;
    HC_BT_HDR *p_evt_buf = (HC_BT_HDR *)p_mem;
    char *p_name, *p_tmp;

    p_tmp = p_name = (char *)(p_evt_buf + 1) + \
                     HCI_EVT_CMD_CMPL_LOCAL_NAME_STRING;

    if (check_start)
    {
        reg_data = *p_tmp + ((*(p_tmp + 1)) << 8) + ((*(p_tmp + 2)) << 16) + ((*(p_tmp + 3)) << 24);
        cmp_data = (p_iccm_buf[iccm_read_off]) + (p_iccm_buf[iccm_read_off + 1] << 8)
                   + (p_iccm_buf[iccm_read_off + 2] << 16) + (p_iccm_buf[iccm_read_off + 3] << 24);
        //ALOGI("[rsj]dbg99,reg_data = %x,cmp_data = %x\n", reg_data, cmp_data);
        if (cmp_data != reg_data)
        {
            ALOGI("read iccm Fail\n");
            ALOGI("[rsj]dbg99,reg_data = %#x,cmp_data = %#x\n", reg_data, cmp_data);
            ALOGI("[rsj]dbg99,j = %d,rom_size = %d\n", j, rom_size);
        }
        iccm_read_off += 4;
        //iccm_read_off = j;
    }
    else
    {
        userial_vendor_set_baud(\
                                line_speed_to_userial_baud(4000000) \
                               );
        check_start = 1;
    }
    UINT16_TO_STREAM(p, TCI_READ_REG);
    *p++ = 4;                               /* parameter length */
    UINT32_TO_STREAM(p, iccm_read_off);     /* addr */

    p_buf->len = HCI_CMD_PREAMBLE_SIZE + \
                 4;
    hw_cfg_cb.state = HW_CFG_AML_DOWNLOAD_FIRMWARE_TEST_UART;

    is_proceeding = bt_vendor_cbacks->xmit_cb(TCI_READ_REG, \
                    p_buf, hw_config_cback);
    if (j >= check_size || iccm_read_off >= rom_size)
        //if (j >= rom_size)
    {
        j = 0;
        hw_cfg_cb.state = HW_CFG_AML_DOWNLOAD_FIRMWARE_CHANGE_BAUDRATE_UART;

        hw_cfg_set_rom_offset(iccm_read_off);
        if (iccm_read_off >= rom_size)
        {
            hw_cfg_set_rom_offset(0);
        }
    }
    j = j + 4;
    ALOGI("rom check iccm_read_off = %d\n", iccm_read_off);
    return is_proceeding;
}

uint8_t hw_cfg_set_baudrate(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p)
{
    return TRUE;
}

uint8_t hw_cfg_read_bd_addr(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p)
{
    uint8_t is_proceeding = FALSE;
    HC_BT_HDR *p_evt_buf = (HC_BT_HDR *)p_mem;
    char *p_name, *p_tmp;
    const uint8_t null_bdaddr[BD_ADDR_LEN] = { 0, 0, 0, 0, 0, 0 };

    p_tmp = (char *)(p_evt_buf + 1) + \
            HCI_EVT_CMD_CMPL_LOCAL_BDADDR_ARRAY;

    if (memcmp(p_tmp, null_bdaddr, BD_ADDR_LEN) == 0)
    {
        // Controller does not have a valid OTP BDADDR!
        // Set the BTIF initial BDADDR instead.
        is_proceeding = hw_config_set_bdaddr(p_buf);
        if (is_proceeding == TRUE)
        {
            return is_proceeding;
        }
    }
    else
    {
        BTHWDBG("Controller OTP bdaddr %02X:%02X:%02X:%02X:%02X:%02X",
                *(p_tmp + 5), *(p_tmp + 4), *(p_tmp + 3),
                *(p_tmp + 2), *(p_tmp + 1), *p_tmp);
    }

    ALOGI("vendor lib fwcfg completed");
    bt_vendor_cbacks->dealloc(p_buf);
    bt_vendor_cbacks->fwcfg_cb(BT_VND_OP_RESULT_SUCCESS);

    hw_cfg_cb.state = 0;

    if (hw_cfg_cb.fw_fd != -1)
    {
        close(hw_cfg_cb.fw_fd);
        hw_cfg_cb.fw_fd = -1;
    }

    is_proceeding = TRUE;
    return is_proceeding;
}

void hw_cfg_read_rf_param(void)
{
    int bt_fd = -1;
    unsigned char cnt = 0;
    int size = 0;

      if (amlbt_transtype.interface == AML_INTF_SDIO || amlbt_transtype.interface == AML_INTF_PCIE)
      {
          bt_fd = open("/dev/stpbt", O_RDWR | O_NOCTTY | O_NONBLOCK);
      }
      else
      {
          bt_fd = open("/dev/aml_btusb", O_RDWR | O_NOCTTY | O_NONBLOCK);
      }

    ALOGD("hw_cfg_read_rf_param bt_fd %#x", bt_fd);
    if (bt_fd >= 0)
    {
        size = read(bt_fd, &cnt, sizeof(cnt));
        if (size < 0)
        {
            ALOGE("In %s, Read failed:%s", __FUNCTION__, strerror(errno));
        }
        close(bt_fd);
    }
}


uint8_t hw_cfg_bt_power_end(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p)
{
    HC_BT_HDR *p_evt_buf = (HC_BT_HDR *)p_mem;
    char local_ver[kverLength + 1];
    char *p_tmp = (char *)(p_evt_buf + 1) + \
                  HCI_EVT_CMD_CMPL_LOCAL_FW_VERSION;

    BTHWDBG("hw_cfg_bt_power_end \n");

    /*
        if (amlbt_transtype.interface != AML_INTF_USB
            && amlbt_transtype.family_id != AML_W1)
        {
            hw_cfg_read_rf_param();
        }
    */
    bt_vendor_cbacks->dealloc(p_buf);
    bt_vendor_cbacks->fwcfg_cb(BT_VND_OP_RESULT_SUCCESS);
    if (amlbt_btrecovery)
    {
        hw_start_recovery();
    }
    hw_cfg_cb.state = 0;
    bt_power = 1;
    if (hw_cfg_cb.fw_fd != -1)
    {
        close(hw_cfg_cb.fw_fd);
        hw_cfg_cb.fw_fd = -1;
    }

    return TRUE;
}


uint8_t hw_cfg_set_params(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p)
{
    uint8_t is_proceeding = FALSE;

    if (amlbt_transtype.family_id == AML_W1U || amlbt_transtype.family_id == AML_W1)
    {
        ms_delay(300);
    }
    is_proceeding = hw_config_set_bdaddr(p_buf);

    if (is_proceeding == FALSE)
    {
        BTHWDBG("HW_CFG_SET_PARAMS ERROR");
        if (hw_cfg_cb.fw_fd != -1)
        {
            close(hw_cfg_cb.fw_fd);
            hw_cfg_cb.fw_fd = -1;
        }
    }
    return is_proceeding;
}

uint8_t hw_cfg_set_bd_addr(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p)
{
    HC_BT_HDR *p_evt_buf = (HC_BT_HDR *)p_mem;
    char local_ver[128];
    char chip_name[128] = {0};
    char *p_tmp = (char *)(p_evt_buf + 1) + \
                  HCI_EVT_CMD_CMPL_LOCAL_FW_VERSION;
    int i;
    uint8_t is_proceeding = FALSE;

    uint8_t APCF_config_manf_data[] = {0x05, 0x19, 0xff,0x01, 0x0a,0xb};

    property_get("persist.vendor.bt_name", chip_name, "unknown");
    int year = 2020 + (*(p_tmp + 1) >> 4)%16;
    int month= (*(p_tmp + 1) & 0x0F)%16;

    BTHWDBG("BT Controller model=%s,version = %04d.%02d.%02x,number = 0x%02x%02x", chip_name,year,month, *p_tmp, *(p_tmp + 3), *(p_tmp + 2));
    sprintf(local_ver, "model=%s,version = %04d.%02d.%02x,number = 0x%02x%02x",chip_name, year,month, *p_tmp, *(p_tmp + 3), *(p_tmp + 2));

    if (property_set(VENDOR_AMLBTVER_PROPERTY, (char *)local_ver) < 0)
    {
        ALOGE("%s:Failed to set amlbt version in %s", __func__, VENDOR_AMLBTVER_PROPERTY);
    }
    ALOGD("vendor lib fwcfg completed");
    ALOGD("vendor lib config manf data");
    UINT16_TO_STREAM(p, HCI_VSC_WAKE_WRITE_DATA);
    *p++ = APCF_config_manf_data[0];
    for (i = 1; i < (int)sizeof(APCF_config_manf_data); i++)
        *p++ = APCF_config_manf_data[i];
    p_buf->len = HCI_CMD_PREAMBLE_SIZE + APCF_config_manf_data[0];
    hw_cfg_cb.state = HW_CFG_SET_MANU_DATA;
    //Set ADV parameter of remote controller using to wake up device when suspend
    is_proceeding = bt_vendor_cbacks->xmit_cb(HCI_VSC_WAKE_WRITE_DATA, p_buf, hw_config_cback);

    return is_proceeding;
}

uint8_t wole_config_write_manufacture(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p)
{
    /*
    if (amlbt_transtype.interface != AML_INTF_USB
        && amlbt_transtype.family_id != AML_W1)
    {
        hw_cfg_read_rf_param();
    }
    */
    bt_vendor_cbacks->dealloc(p_buf);
    bt_vendor_cbacks->fwcfg_cb(BT_VND_OP_RESULT_SUCCESS);
    if (amlbt_btrecovery)
    {
        hw_start_recovery();
    }
    hw_cfg_cb.state = 0;
    bt_power = 1;
    if (hw_cfg_cb.fw_fd != -1)
    {
        close(hw_cfg_cb.fw_fd);
        hw_cfg_cb.fw_fd = -1;
    }
    return TRUE;
}


/*******************************************************************************
**
** Function         hw_config_cback
**
** Description      Callback function for controller configuration
**
** Returns          None
**
*******************************************************************************/
void hw_config_cback(void *p_mem)
{
    HC_BT_HDR *p_evt_buf = (HC_BT_HDR *)p_mem;
    char *p_name, *p_tmp;
    uint8_t *p, status;
    uint16_t opcode;
    HC_BT_HDR *p_buf = NULL;
    uint8_t is_proceeding = FALSE;
    int i;
    int delay = 100;
    char tempBuf[8];
    char *file = NULL;

#if (USE_CONTROLLER_BDADDR == TRUE)
    const uint8_t null_bdaddr[BD_ADDR_LEN] = { 0, 0, 0, 0, 0, 0 };
#endif
    char local_ver[kverLength + 1];

    status = *((uint8_t *)(p_evt_buf + 1) + HCI_EVT_CMD_CMPL_STATUS_RET_BYTE);
    p = (uint8_t *)(p_evt_buf + 1) + HCI_EVT_CMD_CMPL_OPCODE;
    STREAM_TO_UINT16(opcode, p);

    /* Ask a new buffer big enough to hold any HCI commands sent in here */
    if ((status == 0) && bt_vendor_cbacks)
        p_buf = (HC_BT_HDR *)bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + \
                HCI_CMD_MAX_LEN);

    //BTHWDBG("-------------hw_config_cback %#x", hw_cfg_cb.state);
    if (p_buf != NULL)
    {
        p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
        p_buf->offset = 0;
        p_buf->len = 0;
        p_buf->layer_specific = 0;

        p = (uint8_t *)(p_buf + 1);

        is_proceeding = hw_config_func[hw_cfg_cb.state](p_mem, p_buf, p);
    }

    /* Free the RX event buffer */
    if (bt_vendor_cbacks)
        bt_vendor_cbacks->dealloc(p_evt_buf);

    if (is_proceeding == FALSE)
    {
        ALOGE("vendor lib fwcfg aborted!!!");
        if (bt_vendor_cbacks)
        {
            if (p_buf != NULL)
                bt_vendor_cbacks->dealloc(p_buf);

            bt_vendor_cbacks->fwcfg_cb(BT_VND_OP_RESULT_FAIL);
        }

        if (hw_cfg_cb.fw_fd != -1)
        {
            close(hw_cfg_cb.fw_fd);
            hw_cfg_cb.fw_fd = -1;
        }

        hw_cfg_cb.state = 0;
    }
}

/******************************************************************************
**   LPM Static Functions
******************************************************************************/

/*******************************************************************************
**
** Function         hw_lpm_ctrl_cback
**
** Description      Callback function for lpm enable/disable request
**
** Returns          None
**
*******************************************************************************/
void hw_lpm_ctrl_cback(void *p_mem)
{
    HC_BT_HDR *p_evt_buf = (HC_BT_HDR *)p_mem;
    bt_vendor_op_result_t status = BT_VND_OP_RESULT_FAIL;

    if (*((uint8_t *)(p_evt_buf + 1) + HCI_EVT_CMD_CMPL_STATUS_RET_BYTE) == 0)
    {
        status = BT_VND_OP_RESULT_SUCCESS;
    }

    if (bt_vendor_cbacks)
    {
        bt_vendor_cbacks->lpm_cb(status);
        bt_vendor_cbacks->dealloc(p_evt_buf);
    }
}


#if (SCO_CFG_INCLUDED == TRUE)
/*****************************************************************************
**   SCO Configuration Static Functions
*****************************************************************************/

/*******************************************************************************
**
** Function         hw_sco_i2spcm_cfg_cback
**
** Description      Callback function for SCO I2S/PCM configuration request
**
** Returns          None
**
*******************************************************************************/
static void hw_sco_i2spcm_cfg_cback(void *p_mem)
{
    HC_BT_HDR *p_evt_buf = (HC_BT_HDR *)p_mem;
    uint8_t *p;
    uint16_t opcode;
    HC_BT_HDR *p_buf = NULL;
    bt_vendor_op_result_t status = BT_VND_OP_RESULT_FAIL;

    p = (uint8_t *)(p_evt_buf + 1) + HCI_EVT_CMD_CMPL_OPCODE;
    STREAM_TO_UINT16(opcode, p);

    if (*((uint8_t *)(p_evt_buf + 1) + HCI_EVT_CMD_CMPL_STATUS_RET_BYTE) == 0)
    {
        status = BT_VND_OP_RESULT_SUCCESS;
    }

    /* Free the RX event buffer */
    if (bt_vendor_cbacks)
        bt_vendor_cbacks->dealloc(p_evt_buf);

    if (status == BT_VND_OP_RESULT_SUCCESS)
    {
        if ((opcode == HCI_VSC_WRITE_I2SPCM_INTERFACE_PARAM) &&
                (SCO_INTERFACE_PCM == sco_bus_interface))
        {
            uint8_t ret = FALSE;

            /* Ask a new buffer to hold WRITE_SCO_PCM_INT_PARAM command */
            if (bt_vendor_cbacks)
                p_buf = (HC_BT_HDR *)bt_vendor_cbacks->alloc(
                            BT_HC_HDR_SIZE + HCI_CMD_PREAMBLE_SIZE + SCO_PCM_PARAM_SIZE);
            if (p_buf)
            {
                p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
                p_buf->offset = 0;
                p_buf->layer_specific = 0;
                p_buf->len = HCI_CMD_PREAMBLE_SIZE + SCO_PCM_PARAM_SIZE;
                p = (uint8_t *)(p_buf + 1);

                /* do we need this VSC for I2S??? */
                UINT16_TO_STREAM(p, HCI_VSC_WRITE_SCO_PCM_INT_PARAM);
                *p++ = SCO_PCM_PARAM_SIZE;
                memcpy(p, &bt_sco_param, SCO_PCM_PARAM_SIZE);
                ALOGI("SCO PCM configure {0x%x, 0x%x, 0x%x, 0x%x, 0x%x}",
                      bt_sco_param[0], bt_sco_param[1], bt_sco_param[2], bt_sco_param[3],
                      bt_sco_param[4]);
                if ((ret = bt_vendor_cbacks->xmit_cb(HCI_VSC_WRITE_SCO_PCM_INT_PARAM, p_buf,
                                                     hw_sco_i2spcm_cfg_cback)) == FALSE)
                {
                    bt_vendor_cbacks->dealloc(p_buf);
                }
                else
                    return;
            }
            status = BT_VND_OP_RESULT_FAIL;
        }
        else if ((opcode == HCI_VSC_WRITE_SCO_PCM_INT_PARAM) &&
                 (SCO_INTERFACE_PCM == sco_bus_interface))
        {
            uint8_t ret = FALSE;

            /* Ask a new buffer to hold WRITE_PCM_DATA_FORMAT_PARAM command */
            if (bt_vendor_cbacks)
                p_buf = (HC_BT_HDR *)bt_vendor_cbacks->alloc(
                            BT_HC_HDR_SIZE + HCI_CMD_PREAMBLE_SIZE + PCM_DATA_FORMAT_PARAM_SIZE);
            if (p_buf)
            {
                p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
                p_buf->offset = 0;
                p_buf->layer_specific = 0;
                p_buf->len = HCI_CMD_PREAMBLE_SIZE + PCM_DATA_FORMAT_PARAM_SIZE;

                p = (uint8_t *)(p_buf + 1);
                UINT16_TO_STREAM(p, HCI_VSC_WRITE_PCM_DATA_FORMAT_PARAM);
                *p++ = PCM_DATA_FORMAT_PARAM_SIZE;
                memcpy(p, &bt_pcm_data_fmt_param, PCM_DATA_FORMAT_PARAM_SIZE);

                ALOGI("SCO PCM data format {0x%x, 0x%x, 0x%x, 0x%x, 0x%x}",
                      bt_pcm_data_fmt_param[0], bt_pcm_data_fmt_param[1],
                      bt_pcm_data_fmt_param[2], bt_pcm_data_fmt_param[3],
                      bt_pcm_data_fmt_param[4]);

                if ((ret = bt_vendor_cbacks->xmit_cb(HCI_VSC_WRITE_PCM_DATA_FORMAT_PARAM,
                                                     p_buf, hw_sco_i2spcm_cfg_cback)) == FALSE)
                {
                    bt_vendor_cbacks->dealloc(p_buf);
                }
                else
                    return;
            }
            status = BT_VND_OP_RESULT_FAIL;
        }
    }

    ALOGI("sco I2S/PCM config result %d [0-Success, 1-Fail]", status);
    if (bt_vendor_cbacks)
    {
        bt_vendor_cbacks->audio_state_cb(status);
    }
}

/*******************************************************************************
**
** Function         hw_set_MSBC_codec_cback
**
** Description      Callback function for setting WBS codec
**
** Returns          None
**
*******************************************************************************/
static void hw_set_MSBC_codec_cback(void *p_mem)
{
    /* whenever update the codec enable/disable, need to update I2SPCM */
    ALOGI("SCO I2S interface change the sample rate to 16K");
    hw_sco_i2spcm_config(p_mem, SCO_CODEC_MSBC);
}

/*******************************************************************************
**
** Function         hw_set_CVSD_codec_cback
**
** Description      Callback function for setting NBS codec
**
** Returns          None
**
*******************************************************************************/
static void hw_set_CVSD_codec_cback(void *p_mem)
{
    /* whenever update the codec enable/disable, need to update I2SPCM */
    ALOGI("SCO I2S interface change the sample rate to 8K");
    hw_sco_i2spcm_config(p_mem, SCO_CODEC_CVSD);
}

#endif // SCO_CFG_INCLUDED
void Insert32_Uint32(uint8_t *p_buffer, uint32_t data_32_bit)
{
    p_buffer[0] = data_32_bit & 0xFF;
    p_buffer[1] = ((data_32_bit >> 8) & 0xFF);
    p_buffer[2] = ((data_32_bit >> 16) & 0xFF);
    p_buffer[3] = ((data_32_bit >> 24) & 0xFF);
}

/*****************************************************************************
**   Hardware Configuration Interface Functions
*****************************************************************************/


/*******************************************************************************
**
** Function        hw_config_start
**
** Description     Kick off controller initialization process
**
** Returns         None
**
*******************************************************************************/
void hw_config_start(void)
{
    HC_BT_HDR *p_buf = NULL;
    uint8_t *p;
    uint8_t is_proceeding = FALSE;
    static int num;
    char *file = NULL;
    char tempBuf[8];
    num = 50;
    hw_cfg_cb.state = 0;
    hw_cfg_cb.fw_fd = -1;
    hw_cfg_cb.f_set_baud_2 = FALSE;
    int size = 0;
    bt_start_config = 0;

    ALOGD("hw_config_start-------------\n");

    if (bt_vendor_cbacks)
    {
        p_buf = (HC_BT_HDR *)bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + \
                HCI_CMD_PREAMBLE_SIZE + 8);
    }

    if (p_buf)
    {
        //if (bt_start_config == 0)
        if (((amlbt_transtype.interface == AML_INTF_USB && amlbt_transtype.family_id >= AML_W2)
            && !download_hw) || (!bt_power || hw_state == 1))
        {

            ALOGD("hw_config_start uart-------------\n");
            userial_vendor_set_baud(line_speed_to_userial_baud(115200));
            p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
            p_buf->offset = 0;
            p_buf->layer_specific = 0;
            p = (uint8_t *)(p_buf + 1);
            UINT16_TO_STREAM(p, TCI_UPDATE_UART_BAUDRATE);
            *p++ = 8;
            UINT32_TO_STREAM(p, 0xa30128);
#ifdef UART_4M
#ifdef FPGA_ENABLE
            UINT32_TO_STREAM(p, 0x7005);//FPGA 4M
#else
            UINT32_TO_STREAM(p, 0x7009);//ARM 4M
#endif
#endif
#ifdef UART_2M
#ifdef FPGA_ENABLE
            UINT32_TO_STREAM(p, 0x700b);//FPGA 2M
#endif
#endif

            p_buf->len = HCI_CMD_PREAMBLE_SIZE + 8;
            if (amlbt_transtype.family_id == AML_W2L)
            {
                hw_cfg_cb.state = HW_CFG_AML_WRITE_FIRMWARE_MODE;
            }
            else
            {
                hw_cfg_cb.state = HW_CFG_AML_UPDATE_BAUDRATE_UART;
            }
            is_proceeding = bt_vendor_cbacks->xmit_cb(TCI_UPDATE_UART_BAUDRATE,
                            p_buf, hw_config_cback);
        }
        else
        {
            ms_delay(1000);
#ifdef UART_4M
            userial_vendor_set_baud(\
                                    line_speed_to_userial_baud(4000000) \
                                   );
#endif
#ifdef UART_2M
            userial_vendor_set_baud(\
                                    line_speed_to_userial_baud(2000000) \
                                   );
#endif
            if (amlbt_transtype.interface != AML_INTF_USB && amlbt_transtype.family_id == AML_W2)
            {
                if (hw_state > HW_CFG_AML_DOWNLOAD_FIRMWARE_STRAT_CPU_UART || hw_state == 0) //reset
                {
                    p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
                    p_buf->offset = 0;
                    p_buf->layer_specific = 0;
                    p_buf->len = HCI_CMD_PREAMBLE_SIZE;

                    p = (uint8_t *)(p_buf + 1);
                    UINT16_TO_STREAM(p, HCI_RESET);
                    *p = 0;
                    //hw_cfg_cb.state = HW_CFG_START;
                    hw_cfg_cb.state = HW_CFG_AML_POWER_END;

                    BTHWDBG(" hw_config_start reset\n");
                    bt_vendor_cbacks->xmit_cb(HCI_RESET, p_buf, hw_config_cback);
                }
                else if (hw_state <= HW_CFG_AML_DOWNLOAD_FIRMWARE_ICCM_UART
                        && hw_state > HW_CFG_AML_UPDATE_BAUDRATE_UART) //download iccm
                {
                    hw_get_bin_size();

                    p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
                    p_buf->offset = 0;
                    p_buf->layer_specific = 0;
                    p_buf->len = HCI_CMD_PREAMBLE_SIZE;
                    p = (uint8_t *)(p_buf + 1);

                    UINT16_TO_STREAM(p, TCI_WRITE_REG);
                    *p++ = 8;

                    UINT32_TO_STREAM(p, 0xa70014);
                    UINT32_TO_STREAM(p, 0x1000000); /*enable download event */

                    p_buf->len = HCI_CMD_PREAMBLE_SIZE + \
                                 8;
                    hw_cfg_cb.state = HW_CFG_AML_DOWNLOAD_FIRMWARE_ICCM_UART;

                    is_proceeding = bt_vendor_cbacks->xmit_cb(TCI_WRITE_REG, \
                                    p_buf, hw_config_cback);

                    BTHWDBG("uart_target_baud_rate = %d", UART_TARGET_BAUD_RATE);
                    file = amlbt_fw_bin[amlbt_transtype.family_id][amlbt_transtype.interface];
                    if ((fw_fd = open_file(file, O_RDONLY)) > 0)
                    {
                        size = read(fw_fd, tempBuf, 8);
                        if (size < 0)
                        {
                            ALOGE("In %s, Read head failed:%s", __FUNCTION__, strerror(errno));
                            close(fw_fd);
                            return ;
                        }
                        size = read(fw_fd, p_iccm_buf, iccm_size);
                        if (size < 0)
                        {
                            ALOGE("In %s, Read iccm failed:%s", __FUNCTION__, strerror(errno));
                            close(fw_fd);
                            return ;
                        }
                        size = read(fw_fd, p_dccm_buf, dccm_size);
                        if (size < 0)
                        {
                            ALOGE("In %s, Read dccm failed:%s", __FUNCTION__, strerror(errno));
                            close(fw_fd);
                            return ;
                        }
                    }
                }
                else if (hw_state == HW_CFG_AML_DOWNLOAD_FIRMWARE_DCCM_UART) //download dccm
                {
                    hw_get_bin_size();

                    p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
                    p_buf->offset = 0;
                    p_buf->layer_specific = 0;
                    p_buf->len = HCI_CMD_PREAMBLE_SIZE;
                    p = (uint8_t *)(p_buf + 1);

                    UINT16_TO_STREAM(p, TCI_WRITE_REG);
                    *p++ = 8;

                    UINT32_TO_STREAM(p, 0xa70014);
                    UINT32_TO_STREAM(p, 0x1000000); /*enable download event */

                    p_buf->len = HCI_CMD_PREAMBLE_SIZE + \
                                 8;
                    hw_cfg_cb.state = HW_CFG_AML_DOWNLOAD_FIRMWARE_DCCM_UART;

                    is_proceeding = bt_vendor_cbacks->xmit_cb(TCI_WRITE_REG, \
                                    p_buf, hw_config_cback);

                    BTHWDBG("uart_target_baud_rate = %d", UART_TARGET_BAUD_RATE);
                    file = amlbt_fw_bin[amlbt_transtype.family_id][amlbt_transtype.interface];
                    if ((fw_fd = open_file(file, O_RDONLY)) > 0)
                    {
                        size = read(fw_fd, tempBuf, 8);
                        if (size < 0)
                        {
                            ALOGE("In %s, Read head failed:%s", __FUNCTION__, strerror(errno));
                            close(fw_fd);
                            return ;
                        }
                        size = read(fw_fd, p_iccm_buf, iccm_size);
                        if (size < 0)
                        {
                            ALOGE("In %s, Read iccm failed:%s", __FUNCTION__, strerror(errno));
                            close(fw_fd);
                            return ;
                        }
                        size = read(fw_fd, p_dccm_buf, dccm_size);
                        if (size < 0)
                        {
                            ALOGE("In %s, Read dccm failed:%s", __FUNCTION__, strerror(errno));
                            close(fw_fd);
                            return ;
                        }
                    }
                }
                else if (hw_state > HW_CFG_AML_DOWNLOAD_FIRMWARE_DCCM_UART
                        && hw_state <= HW_CFG_AML_DOWNLOAD_FIRMWARE_STRAT_CPU_UART) //start cpu
                {
                    p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
                    p_buf->offset = 0;
                    p_buf->layer_specific = 0;
                    p_buf->len = HCI_CMD_PREAMBLE_SIZE;
                    p = (uint8_t *)(p_buf + 1);

                    UINT16_TO_STREAM(p, TCI_WRITE_REG);
                    *p++ = 8;
                    UINT32_TO_STREAM(p, 0xa70014);
                    UINT32_TO_STREAM(p, 0x0000000);

                    p_buf->len = HCI_CMD_PREAMBLE_SIZE + \
                                 8;
                    hw_cfg_cb.state = HW_CFG_AML_CONFIG_RF_CALIBRATION;

                    is_proceeding = bt_vendor_cbacks->xmit_cb(TCI_WRITE_REG, \
                                    p_buf, hw_config_cback);
                    BTHWDBG("HW_CFG_AML_DOWNLOAD_FIRMWARE_CLOSE_EVENT");
                }
            }
            else
            {
                p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
                p_buf->offset = 0;
                p_buf->layer_specific = 0;
                p_buf->len = HCI_CMD_PREAMBLE_SIZE;

                p = (uint8_t *)(p_buf + 1);
                UINT16_TO_STREAM(p, HCI_RESET);
                *p = 0;

                //hw_cfg_cb.state = HW_CFG_START;
                hw_cfg_cb.state = HW_CFG_AML_POWER_END;

                BTHWDBG(" hw_config_start reset\n");
                bt_vendor_cbacks->xmit_cb(HCI_RESET, p_buf, hw_config_cback);
            }
        }
    }
    else
    {
        if (bt_vendor_cbacks)
        {
            ALOGE("vendor lib fw conf aborted [no buffer]");
            bt_vendor_cbacks->fwcfg_cb(BT_VND_OP_RESULT_FAIL);
        }
    }
}

/*******************************************************************************
**
** Function        hw_lpm_enable
**
** Description     Enalbe/Disable LPM
**
** Returns         TRUE/FALSE
**
*******************************************************************************/
uint8_t hw_lpm_enable(uint8_t turn_on)
{
    HC_BT_HDR *p_buf = NULL;
    uint8_t *p;
    uint8_t ret = TRUE;

    if (bt_vendor_cbacks)
        p_buf = (HC_BT_HDR *)bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + \
                HCI_CMD_PREAMBLE_SIZE + \
                LPM_CMD_PARAM_SIZE);

    if (p_buf)
    {
        p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
        p_buf->offset = 0;
        p_buf->layer_specific = 0;
        p_buf->len = HCI_CMD_PREAMBLE_SIZE + LPM_CMD_PARAM_SIZE;

        p = (uint8_t *)(p_buf + 1);
        UINT16_TO_STREAM(p, HCI_VSC_WRITE_SLEEP_MODE);
        *p++ = LPM_CMD_PARAM_SIZE; /* parameter length */

        if (turn_on)
        {
            memcpy(p, &lpm_param, LPM_CMD_PARAM_SIZE);
            upio_set(UPIO_LPM_MODE, UPIO_ASSERT, 0);
        }
        else
        {
            memset(p, 0, LPM_CMD_PARAM_SIZE);
            upio_set(UPIO_LPM_MODE, UPIO_DEASSERT, 0);
        }

        if (0 == *p)
        {
            ALOGE("LPM disabled!!");
        }
        else
        {
            ALOGD("LPM enabled!!");
        }
        if (!turn_on)
        {
            if (amlbt_transtype.interface == AML_INTF_USB)
            {
                bt_vendor_cbacks->xmit_cb(HCI_VSC_WRITE_SLEEP_MODE,
                                          p_buf, NULL);
            }
        }
        if (amlbt_transtype.interface == AML_INTF_PCIE || amlbt_transtype.interface == AML_INTF_SDIO)
        {
            bt_vendor_cbacks->dealloc(p_buf);
        }
    }
/*
    if ((ret == FALSE) && bt_vendor_cbacks)
        bt_vendor_cbacks->lpm_cb(BT_VND_OP_RESULT_FAIL);
*/
    return ret;
}


/*******************************************************************************
**
** Function        hw_lpm_get_idle_timeout
**
** Description     Calculate idle time based on host stack idle threshold
**
** Returns         idle timeout value
**
*******************************************************************************/
uint32_t hw_lpm_get_idle_timeout(void)
{
    return 3000;
#if 0
    uint32_t timeout_ms;

    /* set idle time to be LPM_IDLE_TIMEOUT_MULTIPLE times of
     * host stack idle threshold (in 300ms/25ms)
     */
    timeout_ms = (uint32_t)lpm_param.host_stack_idle_threshold \
                 * LPM_IDLE_TIMEOUT_MULTIPLE;

    if (strstr(hw_cfg_cb.local_chip_name, "BCM4325") != NULL)
        timeout_ms *= 25; // 12.5 or 25 ?
    else
        timeout_ms *= 300;

    return timeout_ms;
#endif
}

/*******************************************************************************
**
** Function        hw_lpm_set_wake_state
**
** Description     Assert/Deassert BT_WAKE
**
** Returns         None
**
*******************************************************************************/
void hw_lpm_set_wake_state(uint8_t wake_assert)
{
    uint8_t state = (wake_assert) ? UPIO_ASSERT : UPIO_DEASSERT;

    upio_set(UPIO_BT_WAKE, state, lpm_param.bt_wake_polarity);
}

#if (SCO_CFG_INCLUDED == TRUE)
/*******************************************************************************
**
** Function         hw_sco_config
**
** Description      Configure SCO related hardware settings
**
** Returns          None
**
*******************************************************************************/
static int hw_set_SCO_codec(uint16_t codec);
void hw_sco_config(void)
{
    if (SCO_INTERFACE_I2S == sco_bus_interface)
    {
        /* 'Enable' I2S mode */
        bt_sco_i2spcm_param[0] = 1;

        /* set nbs clock rate as the value in SCO_I2SPCM_IF_CLOCK_RATE field */
        sco_bus_clock_rate = bt_sco_i2spcm_param[3];
    }
    else
    {
        /* 'Disable' I2S mode */
        bt_sco_i2spcm_param[0] = 0;

        /* set nbs clock rate as the value in SCO_PCM_IF_CLOCK_RATE field */
        sco_bus_clock_rate = bt_sco_param[1];

        /* sync up clock mode setting */
        bt_sco_i2spcm_param[1] = bt_sco_param[4];
    }

    if (sco_bus_wbs_clock_rate == INVALID_SCO_CLOCK_RATE)
    {
        /* set default wbs clock rate */
        sco_bus_wbs_clock_rate = SCO_I2SPCM_IF_CLOCK_RATE4WBS;

        if (sco_bus_wbs_clock_rate < sco_bus_clock_rate)
            sco_bus_wbs_clock_rate = sco_bus_clock_rate;
    }

    /*
     *  To support I2S/PCM port multiplexing signals for sharing Bluetooth audio
     *  and FM on the same PCM pins, we defer Bluetooth audio (SCO/eSCO)
     *  configuration till SCO/eSCO is being established;
     *  i.e. in hw_set_audio_state() call.
     */

    hw_set_SCO_codec(BTM_SCO_CODEC_CVSD);

    if (bt_vendor_cbacks)
    {
        bt_vendor_cbacks->scocfg_cb(BT_VND_OP_RESULT_SUCCESS);
    }
}

/*******************************************************************************
**
** Function         hw_sco_i2spcm_config
**
** Description      Configure SCO over I2S or PCM
**
** Returns          None
**
*******************************************************************************/
static void hw_sco_i2spcm_config(void *p_mem, uint16_t codec)
{
    HC_BT_HDR *p_evt_buf = (HC_BT_HDR *)p_mem;
    bt_vendor_op_result_t status = BT_VND_OP_RESULT_FAIL;

    if (*((uint8_t *)(p_evt_buf + 1) + HCI_EVT_CMD_CMPL_STATUS_RET_BYTE) == 0)
    {
        status = BT_VND_OP_RESULT_SUCCESS;
    }

    /* Free the RX event buffer */
    if (bt_vendor_cbacks)
        bt_vendor_cbacks->dealloc(p_evt_buf);

    if (status == BT_VND_OP_RESULT_SUCCESS)
    {
        HC_BT_HDR *p_buf = NULL;
        uint8_t *p, ret;
        uint16_t cmd_u16 = HCI_CMD_PREAMBLE_SIZE + SCO_I2SPCM_PARAM_SIZE;

        if (bt_vendor_cbacks)
            p_buf = (HC_BT_HDR *)bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + cmd_u16);

        if (p_buf)
        {
            p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
            p_buf->offset = 0;
            p_buf->layer_specific = 0;
            p_buf->len = cmd_u16;

            p = (uint8_t *)(p_buf + 1);

            UINT16_TO_STREAM(p, HCI_VSC_WRITE_I2SPCM_INTERFACE_PARAM);
            *p++ = SCO_I2SPCM_PARAM_SIZE;
            if (codec == SCO_CODEC_CVSD)
            {
                bt_sco_i2spcm_param[2] = 0; /* SCO_I2SPCM_IF_SAMPLE_RATE  8k */
                bt_sco_i2spcm_param[3] = bt_sco_param[1] = sco_bus_clock_rate;
            }
            else if (codec == SCO_CODEC_MSBC)
            {
                bt_sco_i2spcm_param[2] = wbs_sample_rate; /* SCO_I2SPCM_IF_SAMPLE_RATE 16K */
                bt_sco_i2spcm_param[3] = bt_sco_param[1] = sco_bus_wbs_clock_rate;
            }
            else
            {
                bt_sco_i2spcm_param[2] = 0; /* SCO_I2SPCM_IF_SAMPLE_RATE  8k */
                bt_sco_i2spcm_param[3] = bt_sco_param[1] = sco_bus_clock_rate;
                ALOGE("wrong codec is use in hw_sco_i2spcm_config, goes default NBS");
            }
            memcpy(p, &bt_sco_i2spcm_param, SCO_I2SPCM_PARAM_SIZE);
            cmd_u16 = HCI_VSC_WRITE_I2SPCM_INTERFACE_PARAM;
            ALOGI("I2SPCM config {0x%x, 0x%x, 0x%x, 0x%x}",
                  bt_sco_i2spcm_param[0], bt_sco_i2spcm_param[1],
                  bt_sco_i2spcm_param[2], bt_sco_i2spcm_param[3]);

            if ((ret = bt_vendor_cbacks->xmit_cb(cmd_u16, p_buf, hw_sco_i2spcm_cfg_cback)) == FALSE)
            {
                bt_vendor_cbacks->dealloc(p_buf);
            }
            else
                return;
        }
        status = BT_VND_OP_RESULT_FAIL;
    }

    if (bt_vendor_cbacks)
    {
        bt_vendor_cbacks->audio_state_cb(status);
    }
}

/*******************************************************************************
**
** Function         hw_set_SCO_codec
**
** Description      This functgion sends command to the controller to setup
**                              WBS/NBS codec for the upcoming eSCO connection.
**
** Returns          -1 : Failed to send VSC
**                   0 : Success
**
*******************************************************************************/
static int hw_set_SCO_codec(uint16_t codec)
{
    HC_BT_HDR *p_buf = NULL;
    uint8_t *p;
    uint8_t ret;
    int ret_val = 0;
    tINT_CMD_CBACK p_set_SCO_codec_cback;

    BTHWDBG("hw_set_SCO_codec 0x%x", codec);

    if (bt_vendor_cbacks)
        p_buf = (HC_BT_HDR *)bt_vendor_cbacks->alloc(
                    BT_HC_HDR_SIZE + HCI_CMD_PREAMBLE_SIZE + SCO_CODEC_PARAM_SIZE);

    if (p_buf)
    {
        p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
        p_buf->offset = 0;
        p_buf->layer_specific = 0;
        p = (uint8_t *)(p_buf + 1);

        UINT16_TO_STREAM(p, HCI_VSC_ENABLE_WBS);

        if (codec == SCO_CODEC_MSBC)
        {
            /* Enable mSBC */
            *p++ = SCO_CODEC_PARAM_SIZE;    /* set the parameter size */
            UINT8_TO_STREAM(p, 1);          /* enable */
            UINT16_TO_STREAM(p, codec);

            /* set the totall size of this packet */
            p_buf->len = HCI_CMD_PREAMBLE_SIZE + SCO_CODEC_PARAM_SIZE;

            p_set_SCO_codec_cback = hw_set_MSBC_codec_cback;
        }
        else
        {
            /* Disable mSBC */
            *p++ = (SCO_CODEC_PARAM_SIZE);  /* set the parameter size */
            UINT8_TO_STREAM(p, 0);          /* disable */
            UINT16_TO_STREAM(p, codec);

            /* set the totall size of this packet */
            p_buf->len = HCI_CMD_PREAMBLE_SIZE + SCO_CODEC_PARAM_SIZE;

            p_set_SCO_codec_cback = hw_set_CVSD_codec_cback;
            if ((codec != SCO_CODEC_CVSD) && (codec != SCO_CODEC_NONE))
            {
                ALOGW("SCO codec setting is wrong: codec: 0x%x", codec);
            }
        }

        if ((ret = bt_vendor_cbacks->xmit_cb(HCI_VSC_ENABLE_WBS, p_buf, p_set_SCO_codec_cback)) \
                == FALSE)
        {
            bt_vendor_cbacks->dealloc(p_buf);
            ret_val = -1;
        }
    }
    else
    {
        ret_val = -1;
    }

    return ret_val;
}

/*******************************************************************************
**
** Function         hw_set_audio_state
**
** Description      This function configures audio base on provided audio state
**
** Parameters        pointer to audio state structure
**
** Returns          0: ok, -1: error
**
*******************************************************************************/
int hw_set_audio_state(bt_vendor_op_audio_state_t *p_state)
{
    int ret_val = -1;

    if (!bt_vendor_cbacks)
        return ret_val;

    ret_val = hw_set_SCO_codec(p_state->peer_codec);
    return ret_val;
}

#else  // SCO_CFG_INCLUDED
int hw_set_audio_state(bt_vendor_op_audio_state_t *p_state)
{
    int ret_val = 0;
    ret_val = p_state->state;
    return -256;
}
#endif
/*******************************************************************************
**
** Function        hw_set_patch_file_path
**
** Description     Set the location of firmware patch file
**
** Returns         0 : Success
**                 Otherwise : Fail
**
*******************************************************************************/
int hw_set_patch_file_path(char *p_conf_name __unused, char *p_conf_value __unused, int param __unused)
{
#ifdef AML_DOWNLOADFW_UART
    strcpy(fw_patchfile_path, "/etc/bluetooth/");
#else
    strcpy(fw_patchfile_path, p_conf_value);
#endif
    return 0;
}

/*******************************************************************************
**
** Function        hw_set_patch_file_name
**
** Description     Give the specific firmware patch filename
**
** Returns         0 : Success
**                 Otherwise : Fail
**
*******************************************************************************/
int hw_set_patch_file_name(char *p_conf_name __unused, char *p_conf_value __unused, int param __unused)
{
#ifdef AML_DOWNLOADFW_UART
    strcpy(fw_patchfile_name, "bt_fucode.h");
#else
    strcpy(fw_patchfile_name, p_conf_value);
#endif
    return 0;
}

#if (VENDOR_LIB_RUNTIME_TUNING_ENABLED == TRUE)
/*******************************************************************************
**
** Function        hw_set_patch_settlement_delay
**
** Description     Give the specific firmware patch settlement time in milliseconds
**
** Returns         0 : Success
**                 Otherwise : Fail
**
*******************************************************************************/
int hw_set_patch_settlement_delay(char *p_conf_name, char *p_conf_value, int param)
{
    fw_patch_settlement_delay = atoi(p_conf_value);

    return 0;
}
#endif  //VENDOR_LIB_RUNTIME_TUNING_ENABLED

/*****************************************************************************
**   Sample Codes Section
*****************************************************************************/

#if (HW_END_WITH_HCI_RESET == TRUE)
/*******************************************************************************
**
** Function         hw_epilog_cback
**
** Description      Callback function for Command Complete Events from HCI
**                  commands sent in epilog process.
**
** Returns          None
**
*******************************************************************************/
void hw_epilog_cback(void *p_mem)
{
    HC_BT_HDR *p_evt_buf = (HC_BT_HDR *)p_mem;
    uint8_t *p, status;
    uint16_t opcode;

    status = *((uint8_t *)(p_evt_buf + 1) + HCI_EVT_CMD_CMPL_STATUS_RET_BYTE);
    p = (uint8_t *)(p_evt_buf + 1) + HCI_EVT_CMD_CMPL_OPCODE;
    STREAM_TO_UINT16(opcode, p);

    BTHWDBG("%s Opcode:0x%04X Status: %d", __FUNCTION__, opcode, status);

    if (bt_vendor_cbacks)
    {
        /* Must free the RX event buffer */
        bt_vendor_cbacks->dealloc(p_evt_buf);

        /* Once epilog process is done, must call epilog_cb callback
         * to notify caller */
        bt_vendor_cbacks->epilog_cb(BT_VND_OP_RESULT_SUCCESS);
    }
}

/*******************************************************************************
**
** Function         hw_epilog_process
**
** Description      Sample implementation of epilog process
**
** Returns          None
**
*******************************************************************************/
void hw_epilog_process(void)
{
    HC_BT_HDR *p_buf = NULL;
    uint8_t *p;

    BTHWDBG("hw_epilog_process");

    /* Sending a HCI_RESET */
    if (bt_vendor_cbacks)
    {
        /* Must allocate command buffer via HC's alloc API */
        p_buf = (HC_BT_HDR *)bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + \
                HCI_CMD_PREAMBLE_SIZE);
    }

    if (p_buf)
    {
        p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
        p_buf->offset = 0;
        p_buf->layer_specific = 0;
        p_buf->len = HCI_CMD_PREAMBLE_SIZE;

        p = (uint8_t *)(p_buf + 1);
        UINT16_TO_STREAM(p, HCI_RESET);
        *p = 0; /* parameter length */

        /* Send command via HC's xmit_cb API */
        bt_vendor_cbacks->xmit_cb(HCI_RESET, p_buf, hw_epilog_cback);
    }
    else
    {
        if (bt_vendor_cbacks)
        {
            ALOGE("vendor lib epilog process aborted [no buffer]");
            bt_vendor_cbacks->epilog_cb(BT_VND_OP_RESULT_FAIL);
        }
    }
}
#endif // (HW_END_WITH_HCI_RESET == TRUE)


void hw_read_type_cback(void *p_mem)
{
    HC_BT_HDR *p_evt_buf = (HC_BT_HDR *)p_mem;
    unsigned int reg_value = 0;

    char *p_tmp;
    p_tmp = (char *)(p_evt_buf + 1) + \
            HCI_EVT_CMD_CMPL_LOCAL_NAME_STRING;
    reg_value = *p_tmp + ((*(p_tmp + 1)) << 8) + ((*(p_tmp + 2)) << 16) + ((*(p_tmp + 3)) << 24);

    ALOGD("[AML_USB] %s, %#x", __FUNCTION__, reg_value);

    if (reg_value == W1_RG_AON_A)
    {
        amlbt_transtype.family_id = AML_W1;
        amlbt_transtype.interface = AML_INTF_SDIO;
    }
    else if (reg_value == W1U_RG_AON_A)
    {
        amlbt_transtype.family_id = AML_W1U;
        amlbt_transtype.interface = AML_INTF_SDIO;
    }

    ALOGD("[AML_USB] %s amlbt_transtype(%d:%d:%d)\n", __FUNCTION__, amlbt_transtype.family_id,
          amlbt_transtype.family_rev, amlbt_transtype.interface);
}

void hw_read_type(HC_BT_HDR *p_buf)
{
    uint8_t *p;

    ALOGD("[AML_USB] %s,%p 1", __FUNCTION__, p_buf);
    ALOGD("[AML_USB] %s 2", __FUNCTION__);
    p = (uint8_t *)(p_buf + 1);

    UINT16_TO_STREAM(p, TCI_READ_REG);
    *p++ = 4;                               /* parameter length */
    UINT32_TO_STREAM(p, RG_AON_A);     /* addr */
    ALOGD("[AML_USB] %s 3", __FUNCTION__);
    p_buf->len = HCI_CMD_PREAMBLE_SIZE + \
                 4;
    bt_vendor_cbacks->xmit_cb(TCI_READ_REG, \
                              p_buf, hw_config_cback);
    ALOGD("[AML_USB] %s end", __FUNCTION__);
    hw_cfg_cb.state = HW_CFG_GET_REG;
}

void hw_crash(int sig)
{
    HC_BT_HDR *p_buf = NULL;
    uint8_t *p;
    ALOGI("received user signal to trigger fw crash\n");
    if (bt_vendor_cbacks)
    {
        p_buf = (HC_BT_HDR *)bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + HCI_CMD_MAX_LEN);
        if (p_buf)
        {
            p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
            p_buf->offset = 0;
            p_buf->layer_specific = 0;
            p = (uint8_t *)(p_buf + 1);
            UINT16_TO_STREAM(p, HCI_CHECK_CMD);
            //Send a format incorrect command to trigger fw crash.
            *p = 1; /* parameter length */
            p_buf->len = HCI_CMD_PREAMBLE_SIZE;

            bt_vendor_cbacks->xmit_cb(HCI_CHECK_CMD, \
                                      p_buf, hw_detect_fw_cback);
            BTHWDBG("---------trigger bt hw crash--------");
            fw_alive = 0;
        }
    }
    else
    {
        BTHWDBG("---------failed to trigger BT hw crash--------");
    }
}
void hw_reset_close_cback(void *p_mem)
{
    HC_BT_HDR *p_evt_buf = (HC_BT_HDR *)p_mem;
    uint8_t *rsp = (uint8_t *)p_evt_buf->data;
    BTHWDBG("evt read: %#x, %#x, %#x, %#x, %#x, %#x, %#x \n", rsp[0], rsp[1], rsp[2], rsp[3], rsp[4], rsp[5], rsp[6]);
    state = HW_DISBT_CONFIGURE;
    if (rsp[0] == 0x0e && rsp[5] == 0x00)
    {
        BTHWDBG("reset cback success!\n");
    }
    else
    {
        BTHWDBG("reset cback failed!\n");
    }
    if (bt_vendor_cbacks)
    {
        bt_vendor_cbacks->dealloc(p_evt_buf);
    }
}

void hw_reset_close()
{
    HC_BT_HDR *p_buf = NULL;
    uint8_t *p;
    /* Sending a HCI_RESET */
    if (bt_vendor_cbacks)
    {
        /* Must allocate command buffer via HC's alloc API */
        p_buf = (HC_BT_HDR *)bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + \
                HCI_CMD_PREAMBLE_SIZE);
    }

    if (p_buf)
    {
        p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
        p_buf->offset = 0;
        p_buf->layer_specific = 0;
        p_buf->len = HCI_CMD_PREAMBLE_SIZE;

        p = (uint8_t *)(p_buf + 1);
        UINT16_TO_STREAM(p, HCI_RESET);
        *p = 0; /* parameter length */

        /* Send command via HC's xmit_cb API */
        bt_vendor_cbacks->xmit_cb(HCI_RESET, \
                                    p_buf, hw_reset_close_cback);
        BTHWDBG("reset send success!\n");
    }
}

void hw_reset_shutdown_cback(void *p_mem)
{
    HC_BT_HDR *p_evt_buf = (HC_BT_HDR *)p_mem;
    uint8_t *rsp = (uint8_t *)p_evt_buf->data;
    BTHWDBG("evt read: %#x, %#x, %#x, %#x, %#x, %#x, %#x \n", rsp[0], rsp[1], rsp[2], rsp[3], rsp[4], rsp[5], rsp[6]);
    //state = HW_HOST_SLEEP_VSC;

    if (rsp[0] == 0x0e && rsp[5] == 0x00)
    {
        BTHWDBG("reset cback success!\n");
    }
    else
    {
        BTHWDBG("reset cback failed!\n");
    }
    if (bt_vendor_cbacks)
    {
        bt_vendor_cbacks->dealloc(p_evt_buf);
    }
}

void hw_reset_shutdown()
{
    HC_BT_HDR *p_buf = NULL;
    uint8_t *p;
    /* Sending a HCI_RESET */
    if (bt_vendor_cbacks)
    {
        /* Must allocate command buffer via HC's alloc API */
        p_buf = (HC_BT_HDR *)bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + \
                HCI_CMD_PREAMBLE_SIZE);
    }

    if (p_buf)
    {
        p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
        p_buf->offset = 0;
        p_buf->layer_specific = 0;
        p_buf->len = HCI_CMD_PREAMBLE_SIZE;

        p = (uint8_t *)(p_buf + 1);
        UINT16_TO_STREAM(p, HCI_RESET);
        *p = 0; /* parameter length */

        /* Send command via HC's xmit_cb API */
        bt_vendor_cbacks->xmit_cb(HCI_RESET, \
                                    p_buf, hw_reset_shutdown_cback);
        BTHWDBG("reset send success!\n");
    }
}
void host_sleep_VSC_cback(void *p_mem)
{
    HC_BT_HDR *p_evt_buf = (HC_BT_HDR *)p_mem;
    uint8_t *rsp = (uint8_t *)p_evt_buf->data;
    BTHWDBG("read: %#x, %#x, %#x, %#x, %#x, %#x, %#x \n", rsp[0], rsp[1], rsp[2], rsp[3], rsp[4], rsp[5], rsp[6]);
    //state = HW_APCE_CONFIG_MANF_DATA;

    if (rsp[0] == 0x0e && rsp[5] == 0x00)
    {
        BTHWDBG("%s success!\n", __func__);
    }
    else
    {
        BTHWDBG("%s failed!\n", __func__);
    }
    if (bt_vendor_cbacks)
    {
        bt_vendor_cbacks->dealloc(p_evt_buf);
    }
}

void hw_host_sleep_VSC()
{
    HC_BT_HDR *p_buf = NULL;
    uint8_t *p;
    if (bt_vendor_cbacks)
    {
        /* Must allocate command buffer via HC's alloc API */
        p_buf = (HC_BT_HDR *)bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + \
                HCI_CMD_PREAMBLE_SIZE + 1);
    }

    if (p_buf)
    {
        p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
        p_buf->offset = 0;
        p_buf->layer_specific = 0;
        p_buf->len = HCI_CMD_PREAMBLE_SIZE;

        p = (uint8_t *)(p_buf + 1);
        UINT16_TO_STREAM(p, HCI_HOST_SLEEP_VSC);
        *p++ = 1; /* parameter length */
        UINT8_TO_STREAM(p, 0x01);
        p_buf->len = HCI_CMD_PREAMBLE_SIZE + 1;

        /* Send command via HC's xmit_cb API */
        bt_vendor_cbacks->xmit_cb(HCI_HOST_SLEEP_VSC, \
                                    p_buf, host_sleep_VSC_cback);
        BTHWDBG("%s send success!\n", __func__);
    }
}
void APCF_config_manf_data_cback(void *p_mem)
{
    HC_BT_HDR *p_evt_buf = (HC_BT_HDR *)p_mem;
    uint8_t *rsp = (uint8_t *)p_evt_buf->data;
    BTHWDBG("read: %#x, %#x, %#x, %#x, %#x, %#x, %#x \n", rsp[0], rsp[1], rsp[2], rsp[3], rsp[4], rsp[5], rsp[6]);
    //state = HW_LE_SET_EVT_MASK;

    if (rsp[0] == 0x0e && rsp[5] == 0x00)
    {
        BTHWDBG("%s success!\n", __func__);
    }
    else
    {
        BTHWDBG("%s failed!\n", __func__);
    }

    if (bt_vendor_cbacks)
    {
        bt_vendor_cbacks->dealloc(p_evt_buf);
    }
}

void hw_APCF_config_manf_data()
{
    HC_BT_HDR *p_buf = NULL;
    uint8_t *p;
    int i = 0;
    unsigned char APCF_data[] = {0x19, 0xff, 0x01, 0x0a, 0xb};
    if (bt_vendor_cbacks)
    {
        /* Must allocate command buffer via HC's alloc API */
        p_buf = (HC_BT_HDR *)bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + \
                HCI_CMD_PREAMBLE_SIZE + 5);
    }

    if (p_buf)
    {
        p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
        p_buf->offset = 0;
        p_buf->layer_specific = 0;
        p_buf->len = HCI_CMD_PREAMBLE_SIZE;

        p = (uint8_t *)(p_buf + 1);
        UINT16_TO_STREAM(p, HCI_VSC_WAKE_WRITE_DATA);
        *p++ = 5; /* parameter length */
        for (; i < sizeof(APCF_data); i++)
        {
            UINT8_TO_STREAM(p, APCF_data[i]);
        }
        p_buf->len = HCI_CMD_PREAMBLE_SIZE + 5;

        /* Send command via HC's xmit_cb API */
        bt_vendor_cbacks->xmit_cb(HCI_VSC_WAKE_WRITE_DATA, \
                                    p_buf, APCF_config_manf_data_cback);
        BTHWDBG("%s send success!\n", __func__);
    }
}
void le_set_evt_mask_cback(void *p_mem)
{
    HC_BT_HDR *p_evt_buf = (HC_BT_HDR *)p_mem;
    uint8_t *rsp = (uint8_t *)p_evt_buf->data;
    BTHWDBG("read: %#x, %#x, %#x, %#x, %#x, %#x, %#x \n", rsp[0], rsp[1], rsp[2], rsp[3], rsp[4], rsp[5], rsp[6]);
    //state = HW_LE_SCAN_PARAM_SETTING;

    if (rsp[0] == 0x0e && rsp[5] == 0x00)
    {
        BTHWDBG("%s success!\n", __func__);
    }
    else
    {
        BTHWDBG("%s failed!\n", __func__);
    }
    if (bt_vendor_cbacks)
    {
        bt_vendor_cbacks->dealloc(p_evt_buf);
    }
}

void hw_le_set_evt_mask()
{
    HC_BT_HDR *p_buf = NULL;
    uint8_t *p;
    int i = 0;
    unsigned char le_set[] = {0x7F, 0x1A, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00};
    if (bt_vendor_cbacks)
    {
        /* Must allocate command buffer via HC's alloc API */
        p_buf = (HC_BT_HDR *)bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + \
                HCI_CMD_PREAMBLE_SIZE + 8);
    }

    if (p_buf)
    {
        p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
        p_buf->offset = 0;
        p_buf->layer_specific = 0;
        p_buf->len = HCI_CMD_PREAMBLE_SIZE;

        p = (uint8_t *)(p_buf + 1);
        UINT16_TO_STREAM(p, 0x2001);
        *p++ = 8; /* parameter length */
        for (; i < sizeof(le_set); i++)
        {
            UINT8_TO_STREAM(p, le_set[i]);
        }
        p_buf->len = HCI_CMD_PREAMBLE_SIZE + 8;

        /* Send command via HC's xmit_cb API */
        bt_vendor_cbacks->xmit_cb(0x2001, \
                                    p_buf, le_set_evt_mask_cback);
        BTHWDBG("%s send success!\n", __func__);
    }
}
void le_scan_param_setting_cback(void *p_mem)
{
    HC_BT_HDR *p_evt_buf = (HC_BT_HDR *)p_mem;
    uint8_t *rsp = (uint8_t *)p_evt_buf->data;
    BTHWDBG("read: %#x, %#x, %#x, %#x, %#x, %#x, %#x \n", rsp[0], rsp[1], rsp[2], rsp[3], rsp[4], rsp[5], rsp[6]);
    //state = HW_LE_SCAN_ENABLE;
    BTHWDBG("%s state %d\n", __func__, state);
    if (rsp[0] == 0x0e && rsp[5] == 0x00)
    {
        BTHWDBG("%s success!\n", __func__);
    }
    else
    {
        BTHWDBG("%s failed!\n", __func__);
    }
    if (bt_vendor_cbacks)
    {
        bt_vendor_cbacks->dealloc(p_evt_buf);
    }
}

void hw_le_scan_param_setting()
{
    HC_BT_HDR *p_buf = NULL;
    uint8_t *p;
    int i = 0;
    unsigned char le_scan[] = {0x00, 0x10, 0x00, 0x10, 0x00, 0x00, 0x00};
    if (bt_vendor_cbacks)
    {
        /* Must allocate command buffer via HC's alloc API */
        p_buf = (HC_BT_HDR *)bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + \
                HCI_CMD_PREAMBLE_SIZE + 7);
    }

    if (p_buf)
    {
        p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
        p_buf->offset = 0;
        p_buf->layer_specific = 0;
        p_buf->len = HCI_CMD_PREAMBLE_SIZE;

        p = (uint8_t *)(p_buf + 1);
        UINT16_TO_STREAM(p, 0x200b);
        *p++ = 7; /* parameter length */
        for (; i < sizeof(le_scan); i++)
        {
            UINT8_TO_STREAM(p, le_scan[i]);
        }
        p_buf->len = HCI_CMD_PREAMBLE_SIZE + 7;

        /* Send command via HC's xmit_cb API */
        bt_vendor_cbacks->xmit_cb(0x200b, \
                                    p_buf, le_scan_param_setting_cback);
        BTHWDBG("%s send success!\n", __func__);
    }
}
void le_scan_enable_cback(void *p_mem)
{
    HC_BT_HDR *p_evt_buf = (HC_BT_HDR *)p_mem;
    uint8_t *rsp = (uint8_t *)p_evt_buf->data;
    BTHWDBG("read: %#x, %#x, %#x, %#x, %#x, %#x, %#x \n", rsp[0], rsp[1], rsp[2], rsp[3], rsp[4], rsp[5], rsp[6]);
    if (rsp[0] == 0x0e && rsp[5] == 0x00)
    {
        BTHWDBG("%s success!\n", __func__);
    }
    else
    {
        BTHWDBG("%s failed!\n", __func__);
    }
    if (bt_vendor_cbacks)
    {
        bt_vendor_cbacks->dealloc(p_evt_buf);
    }
    //state = 0;
}

void hw_le_scan_enable()
{
    HC_BT_HDR *p_buf = NULL;
    uint8_t *p;
    if (bt_vendor_cbacks)
    {
        /* Must allocate command buffer via HC's alloc API */
        p_buf = (HC_BT_HDR *)bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + \
                HCI_CMD_PREAMBLE_SIZE + 2);
    }

    if (p_buf)
    {
        p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
        p_buf->offset = 0;
        p_buf->layer_specific = 0;
        p_buf->len = HCI_CMD_PREAMBLE_SIZE;

        p = (uint8_t *)(p_buf + 1);
        UINT16_TO_STREAM(p, 0x200c);
        *p++ = 2; /* parameter length */
        UINT8_TO_STREAM(p, 0x01);
        UINT8_TO_STREAM(p, 0x00);
        p_buf->len = HCI_CMD_PREAMBLE_SIZE + 2;

        /* Send command via HC's xmit_cb API */
        bt_vendor_cbacks->xmit_cb(0x200c, \
                                    p_buf, le_scan_enable_cback);
        BTHWDBG("%s send success!\n", __func__);
    }
}
void hw_disbt_configure_cback(void *p_mem)
{
    char shutdwon_status[PROPERTY_VALUE_MAX];
    HC_BT_HDR *p_evt_buf = (HC_BT_HDR *)p_mem;
    uint8_t *rsp = (uint8_t *)p_evt_buf->data;
    BTHWDBG("read: %#x, %#x, %#x, %#x, %#x, %#x, %#x \n", rsp[0], rsp[1], rsp[2], rsp[3], rsp[4], rsp[5], rsp[6]);
    property_get(PWR_PROP_NAME, shutdwon_status, "unknown");
    if (strstr(shutdwon_status, "0userrequested") == NULL)
    {
        state = HW_REG_PUM_CLEAR;
    }
    else
    {
        state = HW_CLEAR_LIST;
    }

    if (rsp[0] == 0x0e && rsp[1] == 0x04)
    {
        BTHWDBG("%s success!\n", __func__);
    }
    else
    {
        BTHWDBG("%s failed!\n", __func__);
    }
    if (bt_vendor_cbacks)
    {
        bt_vendor_cbacks->dealloc(p_evt_buf);
    }
}

void hw_disbt_configure()
{
    HC_BT_HDR *p_buf = NULL;
    uint8_t *p;
    int i = 0;
    unsigned char reset_A15_cmd[] = {0x3c, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00};
    if (bt_vendor_cbacks)
    {
        /* Must allocate command buffer via HC's alloc API */
        p_buf = (HC_BT_HDR *)bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + \
                HCI_CMD_PREAMBLE_SIZE + 8);
    }

    if (p_buf)
    {
        p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
        p_buf->offset = 0;
        p_buf->layer_specific = 0;
        p_buf->len = HCI_CMD_PREAMBLE_SIZE;

        p = (uint8_t *)(p_buf + 1);
        UINT16_TO_STREAM(p, 0xfcf1);
        *p++ = 8; /* parameter length */
        for (; i < sizeof(reset_A15_cmd); i++)
        {
            UINT8_TO_STREAM(p, reset_A15_cmd[i]);
        }
        p_buf->len = HCI_CMD_PREAMBLE_SIZE + 8;

        /* Send command via HC's xmit_cb API */
        bt_vendor_cbacks->xmit_cb(0xfcf1, \
                                    p_buf, hw_disbt_configure_cback);
        BTHWDBG("%s send success!\n", __func__);
    }
}
void hw_reg_pum_power_cfg_clear_cback(void *p_mem)
{
    HC_BT_HDR *p_evt_buf = (HC_BT_HDR *)p_mem;
    uint8_t *rsp = (uint8_t *)p_evt_buf->data;
    BTHWDBG("read: %#x, %#x, %#x, %#x, %#x, %#x, %#x \n", rsp[0], rsp[1], rsp[2], rsp[3], rsp[4], rsp[5], rsp[6]);
    state = HW_RESET_CLOSE;
    if (rsp[0] == 0x0e && rsp[1] == 0x04)
    {
        BTHWDBG("%s success!\n", __func__);
    }
    else
    {
        BTHWDBG("%s failed!\n", __func__);
    }
    if (bt_vendor_cbacks)
    {
        bt_vendor_cbacks->dealloc(p_evt_buf);
    }
}

void hw_reg_pum_power_cfg_clear()
{
    HC_BT_HDR *p_buf = NULL;
    uint8_t *p;
    int i = 0;
    unsigned char clear_pmu_cmd[] = {0x40, 0x30, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00};
    if (bt_vendor_cbacks)
    {
        /* Must allocate command buffer via HC's alloc API */
        p_buf = (HC_BT_HDR *)bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + \
                HCI_CMD_PREAMBLE_SIZE + 8);
    }

    if (p_buf)
    {
        p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
        p_buf->offset = 0;
        p_buf->layer_specific = 0;
        p_buf->len = HCI_CMD_PREAMBLE_SIZE;

        p = (uint8_t *)(p_buf + 1);
        UINT16_TO_STREAM(p, 0xfcf1);
        *p++ = 8; /* parameter length */
        for (; i < sizeof(clear_pmu_cmd); i++)
        {
            UINT8_TO_STREAM(p, clear_pmu_cmd[i]);
        }
        p_buf->len = HCI_CMD_PREAMBLE_SIZE + 8;

        /* Send command via HC's xmit_cb API */
        bt_vendor_cbacks->xmit_cb(0xfcf1, \
                                    p_buf, hw_reg_pum_power_cfg_clear_cback);
        BTHWDBG("%s send success!\n", __func__);
    }
}

void wifi_recovery_to_host()
{
    HC_BT_HDR *p_buf = NULL;
    uint8_t *p;

    if (bt_vendor_cbacks)
    {
        p_buf = (HC_BT_HDR *)bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + HCI_CMD_PREAMBLE_SIZE);
    }
    if (p_buf)
    {
        p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
        p_buf->offset = 0;
        p_buf->layer_specific = 0;
        p = (uint8_t *)(p_buf + 1);
        UINT16_TO_STREAM(p, HCI_RECOVERY_CMD);
        *p = 0;
        p_buf->len = HCI_CMD_PREAMBLE_SIZE;

        bt_vendor_cbacks->xmit_cb(HCI_RECOVERY_CMD, p_buf, NULL);
    }
}

void aml_15p4_tx(unsigned char *data, unsigned short len)
{
    HC_BT_HDR *p_buf = NULL;
    uint8_t *p;
    uint8_t is_proceeding = FALSE;
    static int num;
    char *file = NULL;
    hw_cfg_cb.state = 0;
    hw_cfg_cb.fw_fd = -1;
    hw_cfg_cb.f_set_baud_2 = FALSE;

    if (bt_vendor_cbacks)
    {
        p_buf = (HC_BT_HDR *)bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + AML_15P4_CMD_BUF_SIZE);
        if (p_buf)
        {
            p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
            p_buf->offset = 0;
            p_buf->layer_specific = 0;
            p = (uint8_t *)(p_buf + 1);
            memcpy(p, data, len);
            p_buf->len = len;
            bt_vendor_cbacks->xmit_cb(HCI_AML_15P4_CMD, p_buf, aml_15p4_data_cb);
        }
    }
}
void hw_poweroff_clear_list_cback(void *p_mem)
{
    HC_BT_HDR *p_evt_buf = (HC_BT_HDR *)p_mem;
    uint8_t *rsp = (uint8_t *)p_evt_buf->data;
    BTHWDBG("evt read: %#x, %#x, %#x, %#x, %#x, %#x, %#x \n", rsp[0], rsp[1], rsp[2], rsp[3], rsp[4], rsp[5], rsp[6]);
    state = HW_RESET_CLOSE;
    if (rsp[0] == 0x0e && rsp[5] == 0x00)
    {
        BTHWDBG("clear list cback success!\n");
    }
    else
    {
        BTHWDBG("clear list cback failed!\n");
    }
    if (bt_vendor_cbacks)
    {
        bt_vendor_cbacks->dealloc(p_evt_buf);
    }
}

void hw_poweroff_clear_list()
{
    HC_BT_HDR *p_buf = NULL;
    uint8_t *p;

    if (bt_vendor_cbacks)
    {
        /* Must allocate command buffer via HC's alloc API */
        p_buf = (HC_BT_HDR *)bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + \
                HCI_CMD_PREAMBLE_SIZE);
    }

    if (p_buf)
    {
        p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
        p_buf->offset = 0;
        p_buf->layer_specific = 0;
        p_buf->len = HCI_CMD_PREAMBLE_SIZE;

        p = (uint8_t *)(p_buf + 1);
        UINT16_TO_STREAM(p, HCI_FW_CLEAR_LIST);
        *p = 0; /* parameter length */

        /* Send command via HC's xmit_cb API */
        bt_vendor_cbacks->xmit_cb(HCI_FW_CLEAR_LIST, \
                                    p_buf, hw_poweroff_clear_list_cback);
        BTHWDBG("clear list send success!\n");
    }
}


