/******************************************************************************
*
*  Copyright (C) 2029-2021 Amlogic Corporation
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
*  Filename:      bt_vendor_aml.c
*
*  Description:   Amlogic vendor specific library implementation
*
******************************************************************************/

#define LOG_TAG "bt_vendor"

#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <unistd.h>
#include <ctype.h>
#include <utils/Log.h>
#include <cutils/properties.h>
#include <sys/syscall.h>
#include <signal.h>
#include <errno.h>
#include <pthread.h>

#include "bt_vendor_aml.h"
#include "upio.h"
#include "userial_vendor.h"

#include <sys/select.h>
#include <sys/time.h>
#include <assert.h>

#define USB_DEVICE_DIR          "/sys/bus/usb/devices"
#define SDIO_DEVICE_DIR         "/sys/class/mmc_host"

#ifndef BTVND_DBG
    #define BTVND_DBG TRUE
#endif

#if (BTVND_DBG == TRUE)
    #define BTVNDDBG(param, ...) { ALOGD(param, ## __VA_ARGS__); }
#else
    #define BTVNDDBG(param, ...) {}
#endif

#ifndef PROPERTY_VALUE_MAX
    #define PROPERTY_VALUE_MAX 92
#endif

#define W1U_VENDOR  0x414D
#define AML_VENDOR  0x1B8E

#define BTUSB_IOC_MAGIC 'x'

#define IOCTL_GET_BT_DOWNLOAD_STATUS    _IOR(BTUSB_IOC_MAGIC, 0, int)
#define IOCTL_SET_BT_RESET              _IOR(BTUSB_IOC_MAGIC, 1, int)

#define TIMEOUT_TRYSUM    6
#define W1_PID      0x8888

#define finit_module(fd, opts, flags) syscall(SYS_finit_module, fd, opts, flags)
int delete_module(const char *, unsigned int);

/******************************************************************************
**  Externs
******************************************************************************/

void hw_config_start(void);
void hw_crash(int sig);
uint8_t hw_lpm_enable(uint8_t turn_on);
uint32_t hw_lpm_get_idle_timeout(void);
void hw_lpm_set_wake_state(uint8_t wake_assert);
#if (SCO_CFG_INCLUDED == TRUE)
    void hw_sco_config(void);
#endif
void vnd_load_conf(const char *p_path);
#if (HW_END_WITH_HCI_RESET == TRUE)
    void hw_epilog_process(void);
#endif
extern void ms_delay(uint32_t timeout);
extern unsigned int state;
unsigned int hw_state = 0;
void aml_15p4_deinit(void);
void aml_15p4_handle(void);
extern void aml_15p4_tx(unsigned char *data, unsigned short len);
static bool exit_thread;
pthread_t aml_15p4_handle_thread;
#define rev_15p4_cmd_fifo "/data/vendor/bluetooth/fifo_cmd_out"
#define rsp_15p4_rst_fifo "/data/vendor/bluetooth/fifo_cmd_in"

static int S15P4_RF;
static int S15P4_WF;

enum
{
    HW_RESET_CLOSE,
    HW_DISBT_CONFIGURE,
    HW_REG_PUM_CLEAR,
    HW_CLEAR_LIST,
};

enum
{
    hw_state_success = 0,
    hw_state_buadrate = 1,
    hw_state_iccm = 4,
    hw_state_dccm = 5,
    hw_state_cpu = 9,
    hw_state_reset = 10,
};

enum
{
    FW_MODE_COEX        = 1,
    FW_MODE_BT_ONLY     = 2,
    FW_MODE_15P4_ONLY   = 3,
};

/******************************************************************************
**  Variables
******************************************************************************/

bt_vendor_callbacks_t *bt_vendor_cbacks = NULL;
uint8_t vnd_local_bd_addr[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static int g_userial_fd = -1;
int bt_sdio_fd = -1;
aml_chip_type amlbt_transtype = {0};
extern unsigned int amlbt_poweron;
extern unsigned int amlbt_fw_mode;
unsigned char bt_power = 0;
unsigned int download_hw = 0;
static unsigned int recovery_flag = 0;
bt_hw_cfg_cb_t hw_cfg_cb;

pthread_t amlbt_thread_recovery;

/******************************************************************************
**  Local type definitions
******************************************************************************/
#define HCI_MAX_EVENT_SIZE     260

/******************************************************************************
**  Static Variables
******************************************************************************/
static const char DRIVER_PROP_NAME[] = "vendor.sys.amlbtsdiodriver";
static const char W1U_DRIVER_PROP_NAME[] = "vendor.sys.amlbt_w1u";
static const char PWR_PROP_NAME[] = "sys.shutdown.requested";
static const char CHIP_TYPE[] = "persist.vendor.bt_name";
const char *str[6] = {"HW_STATE_SUCCESS", "HW_STATE_BUADRATE",
                       "HW_STATE_ICCM", "HW_STATE_DCCM",
                       "HW_STATE_CPU", "HW_STATE_RESET"};

static const tUSERIAL_CFG userial_init_cfg =
{
    (USERIAL_DATABITS_8 | USERIAL_PARITY_NONE | USERIAL_STOPBITS_1),
    USERIAL_BAUD_115200
};

static unsigned int is_libbt_load_driver(void)
{
    char value[PROPERTY_VALUE_MAX] = {'\0'};

    memset(value, 0, sizeof(value));
    if (property_get("persist.vendor.amllibbt.loaddrv", value, "true"))
    {
        if (!strcmp(value, "true"))
            return 1;
    }
    return 0;
}

static int driver_check(const char *modname, int timeout_ms)
{
    FILE *modules;
    char line[256];
    char *module;
    int count = 1;

    if ((modules = fopen("/proc/modules", "r")) == NULL)
    {
        ALOGW("open /proc/modules failed! err=%s\n", strerror(errno));
        return 0;
    }

    do
    {
        while ((fgets(line, sizeof(line), modules)) != NULL)
        {
            module = strtok(line, " ");
            if (module == NULL)
            {
                fclose(modules);
                ALOGE("%s: module is NULL", __FUNCTION__);
                return 0;
            }
            if (!strcmp(module, modname))
            {
                fclose(modules);
                ALOGW("driver %s is detected! count=%d\n", modname, count);
                return 1;
            }
        }
        ALOGW("driver %s is not detected! count=%d, usleep(20ms)...\n", modname, count);
        count++;
        usleep(20 * 1000);
        timeout_ms -= 20;
    }
    while (timeout_ms > 0);

    fclose(modules);
    return 0;
}
static int insmod_check(const char *modname)
{
    FILE *modules;
    char line[256];
    char *module;

    if ((modules = fopen("/proc/modules", "r")) == NULL)
    {
        ALOGW("open /proc/modules failed! err=%s\n", strerror(errno));
        return 0;
    }
    if ((fgets(line, sizeof(line), modules)) != NULL)
    {
        module = strtok(line, " ");
        if (module == NULL)
        {
            fclose(modules);
            ALOGE("%s: module is NULL", __FUNCTION__);
            return 0;
        }
        if (!strcmp(module, modname))
        {
            fclose(modules);
            ALOGW("driver %s is detected!\n", modname);
            return 1;
        }
    }
    else
    {
        ALOGW("driver %s is not detected!,\n", modname);
    }
    fclose(modules);
    return 0;
}


static int insmod(const char *filename, const char *args,
                  const char *modname, int timeout_ms)
{
    int fd = -1;
    int ret;
    char value[PROPERTY_VALUE_MAX] = {'\0'};
    char buf[256];
    const char *p = NULL;

    if (insmod_check(modname))
    {
        ALOGD("[insmod]driver has already insmod: %s\n", buf);
        return 0;
    }
    memset(value, 0, sizeof(value));
    /* Note: you need disable selinux and gives chmod permission for
    ** driver files when specify the path of driver.
    ** e.g. setprop persist.vendor.wifibt_drv_path "/data/vendor"
    */
    if (property_get("persist.vendor.wifibt_drv_path", value, NULL))
    {
        if ((p = strrchr(filename, '/')))
        {
            memset(buf, 0, sizeof(buf));
            snprintf(buf, sizeof(buf), "%s%s", value, p);
            ALOGD("[insmod]driver: %s\n", buf);
            if ((fd = open(buf, O_RDONLY)) < 0)
            {
                ALOGE("[insmod]open: %s failed! %s\n", buf, strerror(errno));
            }
        }
    }
    if (fd < 0)
    {
        ALOGD("[insmod]driver: %s\n", filename);
        if ((fd = open(filename, O_RDONLY)) < 0)
        {
            ALOGE("[insmod]open: %s failed! %s \n", filename, strerror(errno));
            return -1;
        }
    }
    ret = finit_module(fd, args, 0);
    close(fd);
    if (ret < 0)
    {
        ALOGE("[insmod]finit_module failed! %s\n", strerror(errno));
    }
    if (!driver_check(modname, timeout_ms))
    {
        return -1;
    }
    return 0;
}

static int rmmod(const char *modname, int timeout_ms)
{
    int ret = -1;
    int count = 1;

    do
    {
        ret = delete_module(modname, O_NONBLOCK | O_EXCL);
        if (ret < 0 && errno == EAGAIN)
        {
            usleep(20 * 1000);
        }
        else
        {
            if (!driver_check(modname, 20))
            {
                ret = 0;
                break;
            }
        }
        timeout_ms -= 20;
    }
    while (timeout_ms > 0);

    if (ret != 0)
        ALOGE("[rmmod]Unable to unload driver module %s!\n", modname);
    return ret;
}

static int check_key_value(char *path, char *key, int value)
{
    FILE *fp;
    char newpath[100];
    char string_get[12];
    int value_int = 0;
    memset(newpath, 0, 100);
    sprintf(newpath, "%s/%s", path, key);

    if ((fp = fopen(newpath, "r")) != NULL)
    {
        ALOGD("check_key_value %s \n", newpath);
        memset(string_get, 0, sizeof(string_get));
        if (fgets(string_get, sizeof(string_get) - 1, fp) != NULL)
            ALOGE("string_get %s =%s\n", key, string_get);
        fclose(fp);
        value_int = strtol(string_get, NULL, 16);
        ALOGD("check_key_value value_int %#x, value %#x\n", value_int, value);
        if (value_int == value)
            return 1;
    }
    return 0;
}

static int get_key_value(char *path, char *key)
{
    FILE *fp;
    char newpath[100];
    char string_get[12];
    int value_int = 0;
    memset(newpath, 0, 100);
    sprintf(newpath, "%s/%s", path, key);
    if ((fp = fopen(newpath, "r")) != NULL)
    {
        ALOGD("get_key_value %s \n", newpath);
        memset(string_get, 0, sizeof(string_get));
        if (fgets(string_get, sizeof(string_get) - 1, fp) != NULL)
            ALOGE("string_get %s =%s\n", key, string_get);
        fclose(fp);
        value_int = strtol(string_get, NULL, 16);
        ALOGD("check_key_value value_int %#x\n", value_int);
        return value_int;
    }
    return 0;
}

static void scan_aml_usb_devices(char *path)
{
    char newpath[100];
    DIR *pdir;
    struct dirent *ptr;
    struct stat filestat;
    unsigned int w2 = 0;
    unsigned int pid = 0;

    if (stat(path, &filestat) != 0)
    {
        ALOGE("The file or path(%s) can not be get stat!\n", newpath);
        return ;
    }
    if ((filestat.st_mode & S_IFDIR) != S_IFDIR)
    {
        ALOGE("(%s) is not be a path!\n", path);
        return;
    }
    pdir = opendir(path);
    /*enter sub direc*/
    while ((ptr = readdir(pdir)) != NULL)
    {
        if (strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name, "..") == 0)
            continue;
        memset(newpath, 0, sizeof(newpath));
        sprintf(newpath, "%s/%s", path, ptr->d_name);
        ALOGD("[AML_USB] The file or path(%s)\n", newpath);
        if (stat(newpath, &filestat) != 0)
        {
            ALOGE("The file or path(%s) can not be get stat!\n", newpath);
            continue;
        }
        /* Check if it is path. */
        if ((filestat.st_mode & S_IFDIR) == S_IFDIR)
        {
            if (check_key_value(newpath, "idVendor", W1U_VENDOR))
            {
                amlbt_transtype.interface = AML_INTF_USB;
                amlbt_transtype.wireless = 0;
                amlbt_transtype.family_rev = AML_REV_E;
                amlbt_transtype.family_id = AML_W1U;
                amlbt_transtype.reserved = 0;
                closedir(pdir);
                ALOGD("[AML_USB] Chip type(%d:%d:%d)\n", amlbt_transtype.family_id,
                      amlbt_transtype.family_rev, amlbt_transtype.interface);
                return ;
            }
            else if (check_key_value(newpath, "idVendor", AML_VENDOR))
            {
                pid = get_key_value(newpath, "idProduct");
                amlbt_transtype.interface = pid & 0x07;
                amlbt_transtype.wireless = 0;
                amlbt_transtype.family_rev = (pid >> 7) & 0x03;
                amlbt_transtype.family_id = (pid >> 9) & 0x1f;
                amlbt_transtype.reserved = 0;
                closedir(pdir);
                ALOGD("[AML_USB] Chip type(%d:%d:%d)\n", amlbt_transtype.family_id,
                      amlbt_transtype.family_rev, amlbt_transtype.interface);
                return ;
            }
        }
    }
    closedir(pdir);
}

static int amlbt_sdio_check(char *subpathdst)
{
    unsigned int aml = 0;
    unsigned int pid = 0;
    unsigned int chip_type = 0;

    if (check_key_value(subpathdst, "vendor", AML_VENDOR))
    {
        pid = get_key_value(subpathdst, "device");
        amlbt_transtype.interface = pid & 0x07;
        amlbt_transtype.wireless = 0;
        amlbt_transtype.family_rev = (pid >> 6) & 0x07;
        amlbt_transtype.family_id = (pid >> 9) & 0x1f;
        amlbt_transtype.reserved = 0;
        ALOGD("[AML_SDIO] Chip type(%d:%d:%d)\n", amlbt_transtype.family_id,
              amlbt_transtype.family_rev, amlbt_transtype.interface);
        return 1;
    }
    else if (check_key_value(subpathdst, "vendor", W1_PID))
    {
        amlbt_transtype.interface = AML_INTF_SDIO;
        amlbt_transtype.wireless = 0;
        amlbt_transtype.family_rev = AML_REV_C;
        amlbt_transtype.family_id = AML_W1;
        amlbt_transtype.reserved = 0;
        ALOGD("[AML_SDIO] W1 Chip type(%d:%d:%d)\n", amlbt_transtype.family_id,
              amlbt_transtype.family_rev, amlbt_transtype.interface);
        return 1;
    }

    return 0;
}

static int scan_file_sys(char *path, int level)
{
    char newpath[100];
    DIR *pdir;
    struct dirent *ptr;
    int chip_find = 0;
    struct stat filestat;

    pdir = opendir(path);
    /*enter sub direc*/
    while ((ptr = readdir(pdir)) != NULL)
    {
        if (strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name, "..") == 0)
            continue;
        memset(newpath, 0, sizeof(newpath));
        sprintf(newpath, "%s/%s", path, ptr->d_name);
        ALOGD("[AML_SDIO] The file or path(%d:%s)\n", level, newpath);
        if (stat(newpath, &filestat) != 0)
        {
            ALOGE("The file 1 or path(%s) can not be get stat!\n", newpath);
            perror("1:");
            continue;
        }
        /* Check if it is path. */

        if (level > 0)
        {
            if ((filestat.st_mode & S_IFDIR) == S_IFDIR)
            {
                chip_find = scan_file_sys(newpath, level - 1);
                if (chip_find)
                {
                    closedir(pdir);
                    return chip_find;
                }
            }
        }
        else
        {
            chip_find = amlbt_sdio_check(newpath);
            if (!chip_find)
            {
                ALOGE("chip_find %d!\n", chip_find);
            }
            else
            {
                closedir(pdir);
                return chip_find;
            }
        }
    }
    closedir(pdir);
    return chip_find;
}

static void scan_aml_sdio_devices(char *path)
{
    scan_file_sys(path, 2);
}

static void amlbt_transtype_check_by_persist(void)
{
    char property[PROPERTY_VALUE_MAX] = {0};

    property_get(CHIP_TYPE, property, NULL);

    ALOGD("[AML_BT] %s %s", __FUNCTION__, property);
    if (strcmp("aml_w1", property) == 0)
    {
        amlbt_transtype.interface = AML_INTF_SDIO;
        amlbt_transtype.wireless = 0;
        amlbt_transtype.family_rev = AML_REV_C;
        amlbt_transtype.family_id = AML_W1;
        amlbt_transtype.reserved = 0;
        ALOGD("[AML_BT] W1 Chip type(%d:%d:%d)\n", amlbt_transtype.family_id,
              amlbt_transtype.family_rev, amlbt_transtype.interface);
    }
    else if (strcmp("aml_w1u_s", property) == 0)
    {
        amlbt_transtype.interface = AML_INTF_SDIO;
        amlbt_transtype.wireless = 0;
        amlbt_transtype.family_rev = AML_REV_E;
        amlbt_transtype.family_id = AML_W1U;
        amlbt_transtype.reserved = 0;
        ALOGD("[AML_BT] W1US Chip type(%d:%d:%d)\n", amlbt_transtype.family_id,
              amlbt_transtype.family_rev, amlbt_transtype.interface);
    }
    else if (strcmp("aml_w1u", property) == 0)
    {
        amlbt_transtype.interface = AML_INTF_USB;
        amlbt_transtype.wireless = 0;
        amlbt_transtype.family_rev = AML_REV_E;
        amlbt_transtype.family_id = AML_W1U;
        amlbt_transtype.reserved = 0;
        ALOGD("[AML_BT] W1UU Chip type(%d:%d:%d)\n", amlbt_transtype.family_id,
              amlbt_transtype.family_rev, amlbt_transtype.interface);
    }
    else if (strcmp("aml_w2_s", property) == 0)
    {
        amlbt_transtype.interface = AML_INTF_SDIO;
        amlbt_transtype.wireless = 0;
        amlbt_transtype.family_rev = AML_REV_A;
        amlbt_transtype.family_id = AML_W2;
        amlbt_transtype.reserved = 0;
        ALOGD("[AML_BT] W2S Chip type(%d:%d:%d)\n", amlbt_transtype.family_id,
              amlbt_transtype.family_rev, amlbt_transtype.interface);
    }
    else if (strcmp("aml_w2_p", property) == 0)
    {
        amlbt_transtype.interface = AML_INTF_PCIE;
        amlbt_transtype.wireless = 0;
        amlbt_transtype.family_rev = AML_REV_A;
        amlbt_transtype.family_id = AML_W2;
        amlbt_transtype.reserved = 0;
        ALOGD("[AML_BT] W2P Chip type(%d:%d:%d)\n", amlbt_transtype.family_id,
              amlbt_transtype.family_rev, amlbt_transtype.interface);
    }
    else if (strcmp("aml_w2_u", property) == 0)
    {
        amlbt_transtype.interface = AML_INTF_USB;
        amlbt_transtype.wireless = 0;
        amlbt_transtype.family_rev = AML_REV_A;
        amlbt_transtype.family_id = AML_W2;
        amlbt_transtype.reserved = 0;
        ALOGD("[AML_BT] W2U Chip type(%d:%d:%d)\n", amlbt_transtype.family_id,
              amlbt_transtype.family_rev, amlbt_transtype.interface);
    }
    else if (strcmp("aml_w2l_u", property) == 0)
    {
        amlbt_transtype.interface = AML_INTF_USB;
        amlbt_transtype.wireless = 0;
        amlbt_transtype.family_rev = AML_REV_A;
        amlbt_transtype.family_id = AML_W2L;
        amlbt_transtype.reserved = 0;
        ALOGD("[AML_BT] W2LU Chip type(%d:%d:%d)\n", amlbt_transtype.family_id,
              amlbt_transtype.family_rev, amlbt_transtype.interface);
    }
    else if (strcmp("aml_w2l_s", property) == 0)
    {
        amlbt_transtype.interface = AML_INTF_SDIO;
        amlbt_transtype.wireless = 0;
        amlbt_transtype.family_rev = AML_REV_A;
        amlbt_transtype.family_id = AML_W2L;
        amlbt_transtype.reserved = 0;
        ALOGD("[AML_BT] W2LS Chip type(%d:%d:%d)\n", amlbt_transtype.family_id,
              amlbt_transtype.family_rev, amlbt_transtype.interface);
    }
    else
    {
        ALOGD("[AML_BT] No Chip persist\n");
    }
}


static void amlbt_transtype_check(void)
{
    scan_aml_usb_devices(USB_DEVICE_DIR);

    if (amlbt_transtype.family_id == AML_UNKNOWN)
    {
        scan_aml_sdio_devices(SDIO_DEVICE_DIR);
    }

    ALOGD("[AML_USB] amlbt_transtype_check(%d:%d:%d)\n", amlbt_transtype.family_id,
          amlbt_transtype.family_rev, amlbt_transtype.interface);
}

static void amlbt_usb_driver_load(void)
{
    int p = 0;
    if (is_libbt_load_driver())
    {
        //char value[PROPERTY_VALUE_MAX] = {'\0'};
        char buf[256];
        char driver_pram[100] = {0};
        snprintf(driver_pram, sizeof(driver_pram), "amlbt_if_type=%u", *((unsigned short*)&amlbt_transtype));
        ALOGD("%s %s", __FUNCTION__, driver_pram);
        if (amlbt_transtype.family_id == AML_W1U
                && amlbt_transtype.interface == AML_INTF_USB)
        {
            insmod("/vendor/lib/modules/w1u_comm.ko", "bus_type=usb", "w1u_comm", 200);
            //insmod("/vendor/lib/modules/w1u.ko", "", "w1u", 200);
        }
        else if (amlbt_transtype.family_id == AML_W2
                 && amlbt_transtype.interface == AML_INTF_USB)
        {
            rmmod("wifi_comm", 400);
            insmod("/vendor/lib/modules/w2_comm.ko", "bus_type=usb", "w2_comm", 200);
            //insmod("/vendor/lib/modules/w2.ko", "", "w2", 200);
        }
        else if (amlbt_transtype.family_id == AML_W2L
                 && amlbt_transtype.interface == AML_INTF_USB)
        {
            rmmod("wifi_comm", 400);
            insmod("/vendor/lib/modules/w2l_comm.ko", "bus_type=usb", "w2l_comm", 200);
            //insmod("/vendor/lib/modules/w2.ko", "", "w2", 200);
        }
        //memset(value, 0, sizeof(value));
        /* Note: you need disable selinux and gives chmod permission for
        ** driver files when specify the path of driver.
        ** e.g. setprop persist.vendor.wifibt_drv_path "/data/vendor"
        */
        if (amlbt_transtype.interface == AML_INTF_USB)
        {
            insmod("/vendor/lib/modules/aml_bt.ko", driver_pram, "aml_bt", 200);
        }
        /*
        if (property_get("persist.vendor.wifibt_drv_path", value, NULL))
        {
            memset(buf, 0, sizeof(buf));
            snprintf(buf, sizeof(buf), "fw_path=%s", value);
            if (amlbt_transtype.family_id == AML_W1U
                    && amlbt_transtype.interface == AML_INTF_USB)
            {
                insmod("/vendor/lib/modules/w1u_bt.ko", driver_pram, "w1u_bt", 200);
            }
            else if (amlbt_transtype.family_id == AML_W2
                     && amlbt_transtype.interface == AML_INTF_USB)
            {
                insmod("/vendor/lib/modules/w2_bt.ko", driver_pram, "w2_bt", 200);
            }
            else if (amlbt_transtype.family_id == AML_W2L
                     && amlbt_transtype.interface == AML_INTF_USB)
            {
                insmod("/vendor/lib/modules/w2l_bt.ko", driver_pram, "w2l_bt", 200);
            }
        }
        else
        {
            if (amlbt_transtype.family_id == AML_W1U
                    && amlbt_transtype.interface == AML_INTF_USB)
            {
                insmod("/vendor/lib/modules/w1u_bt.ko", driver_pram, "w1u_bt", 200);
            }
            else if (amlbt_transtype.family_id == AML_W2
                     && amlbt_transtype.interface == AML_INTF_USB)
            {
                insmod("/vendor/lib/modules/w2_bt.ko", driver_pram, "w2_bt", 200);
            }
            else if (amlbt_transtype.family_id == AML_W2L
                     && amlbt_transtype.interface == AML_INTF_USB)
            {
                insmod("/vendor/lib/modules/w2l_bt.ko", driver_pram, "w2l_bt", 200);
            }
        }*/
    }
    else
    {
        char driver_status[PROPERTY_VALUE_MAX];

        property_get(W1U_DRIVER_PROP_NAME, driver_status, "amldriverunkown");
        ALOGD("%s: driver_status = %s ", __FUNCTION__, driver_status);
        if (strcmp("true", driver_status) == 0)
        {
            ALOGW("%s: amlbt usb is already insmod!", __FUNCTION__);
            return;
        }
        ALOGD("%s: set %s true\n", __FUNCTION__, W1U_DRIVER_PROP_NAME);
        p = property_set(W1U_DRIVER_PROP_NAME, "true");
        if (p < 0)
        {
            ALOGW("%s: property_set failed!", __FUNCTION__);
            return;
        }
    }
    return;
}

static void amlbt_usb_driver_unload(void)
{
    int p = 0;
    if (is_libbt_load_driver())
    {
        char value[PROPERTY_VALUE_MAX] = {'\0'};
        char buf[256];

        memset(value, 0, sizeof(value));
        /* Note: you need disable selinux and gives chmod permission for
        ** driver files when specify the path of driver.
        ** e.g. setprop persist.vendor.wifibt_drv_path "/data/vendor"
        */
        if (property_get("persist.vendor.wifibt_drv_path", value, NULL))
        {
            memset(buf, 0, sizeof(buf));
            snprintf(buf, sizeof(buf), "fw_path=%s", value);
            rmmod("aml_bt", 200);
        }
        else
            rmmod("aml_bt", 200);
    }
    else
    {
        char driver_status[PROPERTY_VALUE_MAX];

        property_get(W1U_DRIVER_PROP_NAME, driver_status, "amldriverunkown");
        ALOGD("%s: driver_status = %s ", __FUNCTION__, driver_status);
        if (strcmp("false", driver_status) == 0)
        {
            ALOGW("%s: amlbt usb is already rmmod!", __FUNCTION__);
            return;
        }
        ALOGD("%s: set %s false\n", __FUNCTION__, W1U_DRIVER_PROP_NAME);
        p = property_set(W1U_DRIVER_PROP_NAME, "false");
        if (p < 0)
        {
            ALOGW("%s: property_set failed!", __FUNCTION__);
            return;
        }
    }
    return;
}

static void property_set_state(void)
{
    int p = 0;
    if (hw_cfg_cb.state == hw_state_success)
    {
        p = property_set(DRIVER_PROP_NAME, "HW_STATE_SUCCESS");
        ALOGD("%s: %s ", __FUNCTION__, str[0]);
    }
    else if (hw_cfg_cb.state == hw_state_buadrate)
    {
        p = property_set(DRIVER_PROP_NAME, "HW_STATE_BUADRATE");
        ALOGD("%s: %s ", __FUNCTION__, str[1]);
    }
    else if (hw_cfg_cb.state > hw_state_buadrate && hw_cfg_cb.state <= hw_state_iccm)
    {
        p = property_set(DRIVER_PROP_NAME, "HW_STATE_ICCM");
        ALOGD("%s: %s ", __FUNCTION__, str[2]);
    }
    else if (hw_cfg_cb.state == hw_state_dccm)
    {
        p = property_set(DRIVER_PROP_NAME, "HW_STATE_DCCM");
        ALOGD("%s: %s ", __FUNCTION__, str[3]);
    }
    else if (hw_cfg_cb.state > hw_state_dccm && hw_cfg_cb.state <= hw_state_cpu)
    {
        p = property_set(DRIVER_PROP_NAME, "HW_STATE_CPU");
        ALOGD("%s: %s ", __FUNCTION__, str[4]);
    }
    else if (hw_cfg_cb.state > hw_state_cpu)
    {
        p = property_set(DRIVER_PROP_NAME, "HW_STATE_RESET");
        ALOGD("%s: %s ", __FUNCTION__, str[5]);
    }
    if (p < 0)
    {
        ALOGE("%s: property_set failed!", __FUNCTION__);
        return;
    }
}

static void property_get_state(void)
{
    char value[PROPERTY_VALUE_MAX] = {'\0'};
    int num[6] = {hw_state_success, hw_state_buadrate,
                    hw_state_iccm, hw_state_dccm,
                    hw_state_cpu, hw_state_reset};

    property_get(DRIVER_PROP_NAME, value, "0");
    ALOGD("%s: value %s ", __FUNCTION__, value);

    for (int i = 0; i < (sizeof(str)/sizeof(str[0])); i++)
    {
        if (!strcmp(value, str[i]))
        {
           hw_state = num[i];
           break;
        }
    }
    //ALOGD("%s: hw_state %d ", __FUNCTION__, hw_state);
}

/******************************************************************************
**  Functions
******************************************************************************/

/*****************************************************************************
**
**   BLUETOOTH VENDOR INTERFACE LIBRARY FUNCTIONS
**
*****************************************************************************/
static int init(const bt_vendor_callbacks_t *p_cb, unsigned char *local_bdaddr)
{
    ALOGI("amlbt init 0x2024-0402-2020\n");
    ALOGI("Ib9f3195d65a91ae945df4bd28f4f4a7dae2948da\n");

    if (p_cb == NULL)
    {
        ALOGE("init failed with no user callbacks!");
        return -1;
    }

    load_aml_stack_conf();
    if (amlbt_fw_mode == FW_MODE_15P4_ONLY)
    {
        ALOGE("firmware 15.4 only!\n");
        return -1;
    }
#if (VENDOR_LIB_RUNTIME_TUNING_ENABLED == TRUE)
    ALOGW("*****************************************************************");
    ALOGW("*****************************************************************");
    ALOGW("** Warning - BT Vendor Lib is loaded in debug tuning mode!");
    ALOGW("**");
    ALOGW("** If this is not intentional, rebuild libbt-vendor.so ");
    ALOGW("** with VENDOR_LIB_RUNTIME_TUNING_ENABLED=FALSE and ");
    ALOGW("** check if any run-time tuning parameters needed to be");
    ALOGW("** carried to the build-time configuration accordingly.");
    ALOGW("*****************************************************************");
    ALOGW("*****************************************************************");
#endif
    signal(SIGUSR1, hw_crash);
    amlbt_transtype_check_by_persist();
    if (amlbt_transtype.family_id == AML_UNKNOWN)
    {
        amlbt_transtype_check();
    }

    hw_cfg_cb.state = 0;
    if (amlbt_transtype.family_id == AML_W1)
    {
        bt_power = driver_check("sdio_bt", 20);
    }
    else if (amlbt_transtype.interface == AML_INTF_USB)
    {
        bt_power = driver_check("aml_bt", 20);
    }
    /*
    else if (amlbt_transtype.family_id == AML_W1U && amlbt_transtype.interface == AML_INTF_USB)
    {
        bt_power = driver_check("w1u_bt", 20);
    }
    else if (amlbt_transtype.family_id == AML_W2 && amlbt_transtype.interface == AML_INTF_USB)
    {
        bt_power = driver_check("w2_bt", 20);
    }
    else if (amlbt_transtype.family_id == AML_W2L && amlbt_transtype.interface == AML_INTF_USB)
    {
        bt_power = driver_check("w2l_bt", 20);
    }
    */
    if (bt_power == 0 && amlbt_transtype.interface == AML_INTF_USB)
    {
        amlbt_usb_driver_load();
    }
    userial_vendor_init();
    upio_init();
    if (amlbt_transtype.family_id >= AML_W2 && amlbt_transtype.interface != AML_INTF_USB)
    {
        property_get_state();
    }

    vnd_load_conf(VENDOR_LIB_CONF_FILE);


    /* store reference to user callbacks */
    bt_vendor_cbacks = (bt_vendor_callbacks_t *)p_cb;

    /* This is handed over from the stack */
    memcpy(vnd_local_bd_addr, local_bdaddr, 6);

    return 0;
}

int sdio_bt_completed()
{
    int retry_cnt = 1;
    int bt_fd = -1;
    int ret = -1;

    while (retry_cnt < 200)
    {
        usleep(20000);
        if (amlbt_transtype.interface == AML_INTF_SDIO || amlbt_transtype.interface == AML_INTF_PCIE)
        {
            bt_fd = open("/dev/stpbt", O_RDWR | O_NOCTTY | O_NONBLOCK);
        }
        else
        {
            bt_fd = open("/dev/aml_btusb", O_RDWR | O_NOCTTY | O_NONBLOCK);
        }

        if (bt_fd >= 0)
            break;
        else
            ALOGE("%s: Can't open bt: %s. retry_cnt=%d\n", __FUNCTION__, strerror(errno), retry_cnt);

        retry_cnt++;
    }

    if (bt_fd >= 0)
    {
        ALOGD("%s: open bt successfully.[%d]...\n", __FUNCTION__, bt_fd);
        close(bt_fd);
        usleep(10000);
        return bt_fd;
    }
    else
    {
        ALOGE("%s: Can't open bt: %s.\n", __FUNCTION__, strerror(errno));
        return -1;
    }

    return ret;
}


int do_write(int fd, unsigned char *buf, int len)
{
    int ret = 0;
    int write_offset = 0;
    int write_len = len;
    do
    {
        ret = write(fd, buf + write_offset, write_len);
        if (ret < 0)
        {
            ALOGE("%s, write failed ret = %d err = %s", __func__, ret, strerror(errno));
            return -1;
        }
        else if (ret == 0)
        {
            ALOGE("%s, write failed with ret 0 err = %s", __func__, strerror(errno));
            return 0;
        }
        else
        {
            if (ret < write_len)
            {
                //ALOGD("%s, Write pending,do write ret = %d err = %s", __func__, ret,
                //      strerror(errno));
                write_len = write_len - ret;
                write_offset = ret;
            }
            else
            {
                //ALOGD("Write successful");
                break;
            }
        }
    }
    while (1);

    return len;
}

/*******************************************************************************
**
** Function        read_hci_event
**
** Description     Read HCI event during vendor initialization
**
** Returns         int: size to read
**
*******************************************************************************/
int read_hci_event(int fd, unsigned char *buf, int size)
{
    int remain, r;
    int count = 0;
    int retry_cnt = 0;

    if (size <= 0)
    {
        ALOGE("Invalid size argument!");
        return -1;
    }

    //ALOGI("%s: Wait for Command Compete Event from SOC", __FUNCTION__);

    /* The first byte identifies the packet type. For HCI event packets, it
     * should be 0x04, so we read until we get to the 0x04. */
    while (1)
    {
        r = read(fd, buf, 1);
        /*if (r <= 0)
        {
            ALOGE("read_hci_event err: %s \n", strerror(errno));
            return -1;
        }*/

        if (buf[0] == 0x04)
        {
            break;
        }
        usleep(5000);
        if (retry_cnt > TIMEOUT_TRYSUM*5)
        {
            ALOGE("Rx Hci command pkt typetimeout!\n");
            return -1;
        }
        retry_cnt++;
        //ALOGD("TYPE %d", retry_cnt);
    }
    count++;

    /* The next two bytes are the event code and parameter total length. */
    while (count < 3)
    {
        r = read(fd, buf + count, 3 - count);
        if (r <= 0)
            return -1;
        count += r;
        usleep(5000);
        if (retry_cnt > TIMEOUT_TRYSUM*10)
        {
            ALOGE("Rx Hci command pkt headtimeout!\n");
            return -1;
        }
        retry_cnt++;
        //ALOGD("head %d", retry_cnt);
    }
    /* Now we read the parameters. */
    if (buf[2] < (size - 3))
        remain = buf[2];
    else
        remain = size - 3;
    while ((count - 3) < remain)
    {
        r = read(fd, buf + count, remain - (count - 3));
        if (r <= 0)
            return -1;
        count += r;
        usleep(5000);
        if (retry_cnt > TIMEOUT_TRYSUM*15)
        {
            ALOGE("Rx Hci command pkt playloadtimeout!\n");
            return -1;
        }
        retry_cnt++;
        //ALOGD("playload %d", retry_cnt);
    }
    return count;
}
#if 0
int read_hci_event_close(int fd, unsigned char *buf, int size)
{
    int remain, r;
    int count = 0;
    int timeout = 0;

    if (size <= 0)
    {
        ALOGE("Invalid size argument!");
        return -1;
    }

    //ALOGI("%s: Wait for Command Compete Event from SOC", __FUNCTION__);

    /* The first byte identifies the packet type. For HCI event packets, it
     * should be 0x04, so we read until we get to the 0x04. */
    while (1)
    {
        r = read(fd, buf, 1);
        if (r <= 0)
        {
            ALOGE("read_hci_event err: %s \n", strerror(errno));
            return -1;
        }

        if (buf[0] == 0x04)
        {
            break;
        }
    }
    count++;

    /* The next two bytes are the event code and parameter total length. */
    while (count < 3)
    {
        r = read(fd, buf + count, 3 - count);
        if (r <= 0)
            return -1;
        count += r;
    }

    /* Now we read the parameters. */
    if (buf[2] < (size - 3))
        remain = buf[2];
    else
        remain = size - 3;
    while ((count - 3) < remain)
    {
        r = read(fd, buf + count, remain - (count - 3));
        if (r <= 0)
            return -1;
        count += r;
    }
    return count;
}
#endif
int aml_hci_send_cmd(int fd, unsigned char *cmd, int cmdsize, unsigned char *rsp)
{
    int err = 0;

    //ALOGD("%s [abner test]: ", __FUNCTION__);
    err = do_write(fd, cmd, cmdsize);
    if (err != cmdsize)
    {
        ALOGE("%s: Send failed with ret value: %d", __FUNCTION__, err);
        err = -1;
        goto error;
    }

    memset(rsp, 0, HCI_MAX_EVENT_SIZE);

    /* Wait for command complete event */
    while (1)
    {
        //Wait for command complete event
        err = read_hci_event(fd, rsp, HCI_MAX_EVENT_SIZE);
        if (err < 0)
        {
            ALOGE("%s: Failed to set patch info on Controller", __FUNCTION__);
            goto error;
        }
        /*ALOGD("aml_hci_send_cmd read rsp [%#x, %#x, %#x, %#x, %#x, %#x, %#x, %#x, %#x, %#x, %#x, %#x]",
            rsp[0], rsp[1], rsp[2], rsp[3], rsp[4], rsp[5], rsp[6], rsp[7], rsp[8], rsp[9], rsp[10], rsp[11]);*/
        if (rsp[0] == 0x04 && (rsp[1] == 0x0e || rsp[1] == 0x19))
        {
            break;
        }
    }
error:
    return err;
}
int do_write_download(int fd, unsigned char *buf, int len)
{
    int ret = 0;
    int write_offset = 0;
    int write_len = len;
    do
    {
        ret = write(fd, buf + write_offset, write_len);
        if (ret < 0)
        {
            ALOGE("%s, write failed ret = %d err = %s", __func__, ret, strerror(errno));
            return -1;
        }
        else if (ret == 0)
        {
            ALOGE("%s, write failed with ret 0 err = %s", __func__, strerror(errno));
            return 0;
        }
        else
        {
            if (ret < write_len)
            {
                //ALOGD("%s, Write pending,do write ret = %d err = %s", __func__, ret,
                //      strerror(errno));
                write_len = write_len - ret;
                write_offset = ret;
            }
            else
            {
                //ALOGD("Write successful");
                break;
            }
        }
    }
    while (1);

    return len;
}
int read_hci_event_download(int fd, unsigned char *buf, int size)
{
    int remain, r;
    int count = 0;

    if (size <= 0)
    {
        ALOGE("Invalid size argument!");
        return -1;
    }

    //ALOGI("%s: Wait for Command Compete Event from SOC", __FUNCTION__);

    /* The first byte identifies the packet type. For HCI event packets, it
     * should be 0x04, so we read until we get to the 0x04. */
    while (1)
    {
        r = read(fd, buf, 1);
        if (r <= 0)
        {
            ALOGE("read_hci_event err: %s \n", strerror(errno));
            return -1;
        }

        if (buf[0] == 0x04)
        {
            break;
        }
    }
    count++;

    /* The next two bytes are the event code and parameter total length. */
    while (count < 3)
    {
        r = read(fd, buf + count, 3 - count);
        if (r <= 0)
            return -1;
        count += r;
    }
    /* Now we read the parameters. */
    if (buf[2] < (size - 3))
        remain = buf[2];
    else
        remain = size - 3;
    while ((count - 3) < remain)
    {
        r = read(fd, buf + count, remain - (count - 3));
        if (r <= 0)
            return -1;
        count += r;
    }
    return count;
}

int aml_hci_send_cmd_download(int fd, unsigned char *cmd, int cmdsize, unsigned char *rsp)
{
    int err = 0;

    //ALOGD("%s [abner test]: ", __FUNCTION__);
    err = do_write_download(fd, cmd, cmdsize);
    if (err != cmdsize)
    {
        ALOGE("%s: Send failed with ret value: %d", __FUNCTION__, err);
        err = -1;
        goto error;
    }

    memset(rsp, 0, HCI_MAX_EVENT_SIZE);

    /* Wait for command complete event */
    while (1)
    {
        //Wait for command complete event
        err = read_hci_event_download(fd, rsp, HCI_MAX_EVENT_SIZE);
        if (err < 0)
        {
            ALOGE("%s: Failed to set patch info on Controller", __FUNCTION__);
            goto error;
        }
        /*ALOGD("aml_hci_send_cmd read rsp [%#x, %#x, %#x, %#x, %#x, %#x, %#x, %#x, %#x, %#x, %#x, %#x]",
            rsp[0], rsp[1], rsp[2], rsp[3], rsp[4], rsp[5], rsp[6], rsp[7], rsp[8], rsp[9], rsp[10], rsp[11]);*/
        if (rsp[0] == 0x04 && (rsp[1] == 0x0e || rsp[1] == 0x19))
        {
            break;
        }
    }
error:
    return err;
}

int aml_woble_configure(int fd)
{
    unsigned char rsp[HCI_MAX_EVENT_SIZE];
    unsigned char reset_cmd[] = {0x01, 0x03, 0x0C, 0x00};
    unsigned char read_BD_ADDR[] = {0x01, 0x09, 0x10, 0x00};
    unsigned char APCF_config_manf_data[] = {0x01, 0x22, 0xFC, 0x05, 0x19, 0xff, 0x01, 0x0a, 0xb};

    unsigned char APCF_enable[] = {0x01, 0x57, 0xFD, 0x02, 0x00, 0x01};
    unsigned char le_set_evt_mask[] = {0x01, 0x01, 0x20, 0x08, 0x7F, 0x1A, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00};
    unsigned char le_scan_param_setting[] = {0x01, 0x0b, 0x20, 0x07, 0x00, 0x10, 0x00, 0x10, 0x00, 0x00, 0x00};
    unsigned char le_scan_enable[] = {0x01, 0x0c, 0x20, 0x02, 0x01, 0x00};
    unsigned char host_sleep_VSC[] = {0x01, 0x21, 0xfc, 0x01, 0x01};
#if 0
    if (amlbt_transtype.interface == AML_INTF_USB)
    {
        hw_reset_shutdown();
        do
        {
            usleep(5000);
        }while (state == HW_RESET_SHUTDOWN);

        hw_host_sleep_VSC();
        do
        {
            usleep(5000);
        }while (state == HW_HOST_SLEEP_VSC);

        hw_APCF_config_manf_data();
        do
        {
            usleep(5000);
        }while (state == HW_APCE_CONFIG_MANF_DATA);

        hw_le_set_evt_mask();
        do
        {
            usleep(5000);
        }while (state == HW_LE_SET_EVT_MASK);

        hw_le_scan_param_setting();
        do
        {
            usleep(5000);
        }while (state == HW_LE_SCAN_PARAM_SETTING);
        //ALOGD("state %d", state);
        hw_le_scan_enable();
        //ALOGD("hw_le_scan_enable");
    }
#endif
    {
        aml_hci_send_cmd(fd, (unsigned char *)reset_cmd, sizeof(reset_cmd), (unsigned char *)rsp);
        aml_hci_send_cmd(fd, (unsigned char *)host_sleep_VSC, sizeof(host_sleep_VSC), (unsigned char *)rsp);
        aml_hci_send_cmd(fd, (unsigned char *)APCF_config_manf_data, sizeof(APCF_config_manf_data), (unsigned char *)rsp);
        aml_hci_send_cmd(fd, (unsigned char *)le_set_evt_mask, sizeof(le_set_evt_mask), (unsigned char *)rsp);
        aml_hci_send_cmd(fd, (unsigned char *)le_scan_param_setting, sizeof(le_scan_param_setting), (unsigned char *)rsp);
        aml_hci_send_cmd(fd, (unsigned char *)le_scan_enable, sizeof(le_scan_enable), (unsigned char *)rsp);
    }
    return 0;
}

void aml_reg_pum_power_cfg_clear(int fd)
{
    unsigned char rsp[HCI_MAX_EVENT_SIZE];
    unsigned char clear_pmu_cmd[] = {0x01, 0xf1, 0xfc, 0x08, 0x40, 0x30, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00};

    ALOGD("hw_reg_pum_power_cfg_clear start");
    aml_hci_send_cmd(fd, (unsigned char *)clear_pmu_cmd, sizeof(clear_pmu_cmd), (unsigned char *)rsp);
    ALOGD("hw_reg_pum_power_cfg_clear end");
}

int aml_disbt_configure(int fd)
{
    unsigned char rsp[HCI_MAX_EVENT_SIZE];
    unsigned char read_a15_cmd[] = {0x01, 0xf0, 0xfc, 0x04, 0x3c, 0x00, 0xf0, 0x00};
    //set the bit 31 bit 30 of RG_AON_A15(0x00f0003c) t o0 when disable bt;svc bluetooth disable
    unsigned char reset_A15_cmd[] = {0x01, 0xf1, 0xfc, 0x08, 0x3c, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00};

    aml_hci_send_cmd(fd, (unsigned char *)read_a15_cmd, sizeof(read_a15_cmd), (unsigned char *)rsp);
    ALOGD("aml_disbt_configure read rsp [%#x, %#x, %#x, %#x, %#x, %#x, %#x, %#x, %#x, %#x, %#x, %#x]",
        rsp[0], rsp[1], rsp[2], rsp[3], rsp[4], rsp[5], rsp[6], rsp[7], rsp[8], rsp[9], rsp[10], rsp[11]);

    rsp[10] &= 0x3f;
    reset_A15_cmd[11] = rsp[10];
    reset_A15_cmd[10] = rsp[9];
    reset_A15_cmd[9] = rsp[8];
    reset_A15_cmd[8] = rsp[7];

    aml_hci_send_cmd(fd, (unsigned char *)reset_A15_cmd, sizeof(reset_A15_cmd), (unsigned char *)rsp);
    ALOGD("aml_disbt_configure end");
    return 0;
}

void aml_poweroff_bt(int fd)
{
    unsigned char poweroff_cmd[] = {0x01, 0x55, 0xfc, 0x00};
    unsigned char rsp[HCI_MAX_EVENT_SIZE];

    ALOGD("aml_poweroff_bt \n");
    aml_hci_send_cmd(fd, (unsigned char *)poweroff_cmd, sizeof(poweroff_cmd), (unsigned char *)rsp);
    ALOGD("aml_poweroff_bt end\n");
}

void aml_reset_bt(int fd)
{
    unsigned char reset_cmd[] = {0x01, 0x03, 0x0C, 0x00};
    unsigned char rsp[HCI_MAX_EVENT_SIZE];

    ALOGD("aml_reset_bt \n");
    aml_hci_send_cmd(fd, (unsigned char *)reset_cmd, sizeof(reset_cmd), (unsigned char *)rsp);
    ALOGD("aml_reset_bt end\n");
}
void download_hw_crash_ioctl()
{
    if ((amlbt_transtype.family_id == AML_W2 && amlbt_transtype.interface == AML_INTF_USB) && hw_cfg_cb.state != 0)
    {
        long revcrash = 0;
        revcrash = ioctl(g_userial_fd, IOCTL_SET_BT_RESET);
        if (revcrash < 0)
        {
            ALOGD("ioctl send failed!");
        }
        else
        {
           ALOGD("receive usb crash flag=%ld\n", revcrash);
        }
    }
}

void amlbt_recovery_init(void)
{
    int size;
    int rval;
    fd_set read_fds;
    struct timeval timeout = {0, 500000};
    unsigned char recovery_buf[7] = {0};
    FD_ZERO(&read_fds);
    FD_SET(bt_sdio_fd, &read_fds);
    while (1)
    {
        rval = select(bt_sdio_fd + 1, &read_fds, NULL, NULL, &timeout);
        if (rval > 0)
        {
            if (FD_ISSET(bt_sdio_fd, &read_fds))
            {
                size = read(bt_sdio_fd, recovery_buf, sizeof(recovery_buf));
                ALOGI("read size %d select %d", size, rval);
                /*
                for (int i = 0; i < size; i++)
                {
                    ALOGI("%d %x", i, recovery_buf[i]);
                }*/
                if (size == -1)
                {
                    ALOGE("[reovery] read failed %s",strerror(errno));
                    break;
                }
                if (recovery_buf[0] == 0x04 && recovery_buf[1] == 0x0e)
                {
                    ALOGD("%s close bt", __func__);
                    recovery_flag = 1;
                    wifi_recovery_to_host();
                    break;
                }
            }
        }
        timeout.tv_sec = 0;
        timeout.tv_usec = 500000;
        FD_ZERO(&read_fds);
        FD_SET(bt_sdio_fd, &read_fds);
    }
}

void sigpipeHandler(int signum) {
    printf("Caught SIGPIPE signal (%d)\n", signum);
}


extern void hw_stop_recovery(void);
/** Requested operations */
static int op(bt_vendor_opcode_t opcode, void *param)
{
    int retval = 0;
    char shutdwon_status[PROPERTY_VALUE_MAX];
    char driver_pram[100] = {0};
    ALOGD("op for %d", opcode);
    switch (opcode)
    {
        case BT_VND_OP_POWER_CTRL:	//0
        {
            //if (amlbt_transtype.family_id != AML_W2)
            {
                hw_stop_recovery();
            }
            int *state = (int *)param;
            if (*state == BT_VND_PWR_OFF)
            {
                ALOGD("=== power off BT ===");
                if (hw_cfg_cb.state != 0)
                {
                    ALOGD("bt config failed set bt_power to 0");
                    bt_power = 0;
                    download_hw = 0;
                }
                if (amlbt_transtype.family_id >= AML_W1U && amlbt_transtype.interface != AML_INTF_USB)
                {
                    property_get(PWR_PROP_NAME, shutdwon_status, "unknown");
                    if (strstr(shutdwon_status, "0userrequested") == NULL)
                    {
                        bt_power = 0;
                        upio_set_bluetooth_power(UPIO_BT_POWER_OFF);

                        if (amlbt_transtype.family_id == AML_W2L &&
                         amlbt_transtype.interface == AML_INTF_SDIO)
                        {
                            rmmod("aml_bt", 400);
                            rmmod("w2l_comm", 400);
                        }
                    }
                }
                ALOGD("bt_power %d", bt_power);
                if ((amlbt_fw_mode == FW_MODE_COEX) && (amlbt_transtype.family_id == AML_W2L))
                {
                    exit_thread = true;
                    aml_15p4_deinit();
                }
                /*if (!bt_power)
                {
                    if (amlbt_transtype.family_id == AML_W1)
                    {
                        rmmod("sdio_bt", 200);
                        //usleep(500000);
                    }
                    else if (amlbt_transtype.family_id == AML_W1U)
                    {
                        rmmod("w1u_bt", 200);
                        //usleep(500000);
                    }
                    else if (amlbt_transtype.family_id == AML_W2 &&
                            (amlbt_transtype.interface == AML_INTF_PCIE || amlbt_transtype.interface == AML_INTF_SDIO))
                    {
                        rmmod("w2_bt", 200);
                        //usleep(500000);
                    }
                    ALOGD("BT_VND_PWR_OFF rmmod end");
                    upio_set_bluetooth_power(UPIO_BT_POWER_OFF);
                }*/
            }
            else if (*state == BT_VND_PWR_ON)
            {
                ALOGD("=== power on BT ===");
                ALOGD("bt_power %d", bt_power);
                if (upio_power_get() == 0)
                {
                    ALOGD("Upper level upio_power_get power failed");
                    upio_set_bluetooth_power(UPIO_BT_POWER_ON);
                    ALOGD("libbt set_bluetooth_power");
                }
                rmmod("wifi_comm", 100);
                if (bt_power == 0)
                {
                    snprintf(driver_pram, sizeof(driver_pram), "amlbt_if_type=%u", *((unsigned short*)&amlbt_transtype));
                    ALOGD("%s %s", __FUNCTION__, driver_pram);
                    if (amlbt_transtype.family_id == AML_W1)
                    {
                        insmod("/vendor/lib/modules/aml_sdio.ko", "", "aml_sdio", 200);
                        insmod("/vendor/lib/modules/sdio_bt.ko", "aml_w1", "sdio_bt", 200);
                        if (sdio_bt_completed() >= 0)
                        {
                            ALOGD("%s: insmod sdio_bt.ko successfully!", __FUNCTION__);
                        }
                        else
                        {
                            ALOGE("%s: insmod sdio_bt.ko failed!!!!!!!!!!!!!", __FUNCTION__);
                        }
                    }
                    else if (amlbt_transtype.family_id == AML_W1U &&
                             amlbt_transtype.interface == AML_INTF_SDIO)
                    {
                        insmod("/vendor/lib/modules/w1u_comm.ko", "bus_type=sdio", "w1u_comm", 200);
                        //insmod("/vendor/lib/modules/w1u.ko", "", "w1u", 200);
                        insmod("/vendor/lib/modules/aml_bt.ko", driver_pram, "aml_bt", 200);
                    }
                    else if (amlbt_transtype.family_id == AML_W2 &&
                             amlbt_transtype.interface == AML_INTF_PCIE)
                     {
                        //rmmod("wifi_comm", 400);
                        insmod("/vendor/lib/modules/w2_comm.ko", "bus_type=pci", "w2_comm", 200);
                        //insmod("/vendor/lib/modules/w2.ko", "", "w2", 200);
                        insmod("/vendor/lib/modules/aml_bt.ko", driver_pram, "aml_bt", 200);
                    }
                    else if (amlbt_transtype.family_id == AML_W2 &&
                             amlbt_transtype.interface == AML_INTF_SDIO)
                    {
                        //rmmod("wifi_comm", 400);
                        insmod("/vendor/lib/modules/w2_comm.ko", "bus_type=sdio", "w2_comm", 200);
                        //insmod("/vendor/lib/modules/w2.ko", "", "w2", 200);
                        insmod("/vendor/lib/modules/aml_bt.ko", driver_pram, "aml_bt", 200);
                    }
                    else if (amlbt_transtype.family_id == AML_W2L &&
                             amlbt_transtype.interface == AML_INTF_SDIO)
                    {
                        //rmmod("wifi_comm", 400);
                        insmod("/vendor/lib/modules/w2l_comm.ko", "bus_type=sdio", "w2l_comm", 200);
                        //insmod("/vendor/lib/modules/w2.ko", "", "w2", 200);
                        insmod("/vendor/lib/modules/aml_bt.ko", driver_pram, "aml_bt", 200);
                    }
                    else if (amlbt_transtype.family_id == AML_W2L &&
                             amlbt_transtype.interface == AML_INTF_PCIE)
                    {
                        //rmmod("wifi_comm", 400);
                        insmod("/vendor/lib/modules/w2l_comm.ko", "bus_type=pci", "w2l_comm", 200);
                        //insmod("/vendor/lib/modules/w2.ko", "", "w2", 200);
                        insmod("/vendor/lib/modules/aml_bt.ko", driver_pram, "aml_bt", 200);
                    }
                    else if (amlbt_transtype.family_id == AML_W2L &&
                             amlbt_transtype.interface == AML_INTF_USB)
                    {
                        //rmmod("wifi_comm", 400);
                        insmod("/vendor/lib/modules/w2l_comm.ko", "bus_type=usb", "w2l_comm", 200);
                        //insmod("/vendor/lib/modules/w2.ko", "", "w2", 200);
                        insmod("/vendor/lib/modules/aml_bt.ko", driver_pram, "aml_bt", 200);
                    }
                }
                if ((amlbt_fw_mode == FW_MODE_COEX) && (amlbt_transtype.family_id == AML_W2L))
                {
                    exit_thread = false;
                    if (signal(SIGPIPE, sigpipeHandler) == SIG_ERR) {
                        perror("Unable to register SIGPIPE handler");
                    }
                    pthread_create(&aml_15p4_handle_thread, NULL, aml_15p4_handle, NULL);
                }
            }
        }
        break;

        case BT_VND_OP_FW_CFG:	//1
        {
            if (amlbt_transtype.family_id >= AML_W2 && amlbt_transtype.interface == AML_INTF_USB)
            {
                long revData = 0;

                if (ioctl(g_userial_fd, IOCTL_GET_BT_DOWNLOAD_STATUS, &revData) != 0)
                {
                    ALOGD("ioctl send failed!");
                }
                else
                {
                   ALOGD("receive fw flag=%ld\n", revData);
                }
                download_hw = revData;
            }

            hw_config_start();
        }
        break;

        case BT_VND_OP_SCO_CFG:	//2
        {
#if (SCO_CFG_INCLUDED == TRUE)
            hw_sco_config();
#else
            retval = -1;
#endif
        }
        break;

        case BT_VND_OP_USERIAL_OPEN:	//3
        {
            int (*fd_array)[] = (int (*)[])param;
            int fd, idx;

            if (amlbt_transtype.interface == AML_INTF_USB)
            {
                fd = userial_vendor_usb_open();
            }
            else
            {
                fd = userial_vendor_open((tUSERIAL_CFG *)&userial_init_cfg);
            }
            g_userial_fd = fd;
            if (fd != -1)
            {
                for (idx = 0; idx < CH_MAX; idx++)
                    (*fd_array)[idx] = fd;

                retval = 1;
            }
        }
        break;

        case BT_VND_OP_USERIAL_CLOSE: //4
        {
            if (amlbt_transtype.family_id >= AML_W2 && amlbt_transtype.interface != AML_INTF_USB)
            {
                property_set_state();
            }
            ALOGD("%s: shutdwon_status = %s ", __FUNCTION__, shutdwon_status);
            if (amlbt_transtype.family_id == AML_W1 && hw_cfg_cb.state == 0)
            {
                property_get(PWR_PROP_NAME, shutdwon_status, "unknown");
                if (strstr(shutdwon_status, "0userrequested") != NULL)
                {
                    ALOGD("w1 amlbt shutdown");
                    aml_woble_configure(g_userial_fd);
                }
            }
            if (!recovery_flag)
            {
                if ((amlbt_transtype.family_id >= AML_W1U && amlbt_transtype.interface != AML_INTF_USB)
                    && hw_cfg_cb.state == 0)
                {
                    property_get(PWR_PROP_NAME, shutdwon_status, "unknown");
                    if (strstr(shutdwon_status, "0userrequested") == NULL)
                    {
                        //ALOGD("amlbt uart shutdown");
                        aml_reg_pum_power_cfg_clear(g_userial_fd);
                        //aml_woble_configure(g_userial_fd);
                    }
                    else
                    {
                        aml_poweroff_bt(g_userial_fd);
                    }

                    usleep(100000);
                    aml_reset_bt(g_userial_fd);
                    usleep(100000);

                    aml_disbt_configure(g_userial_fd);
                }
            }
            download_hw_crash_ioctl();
            userial_vendor_close();
            if (bt_sdio_fd != -1)
            {
                close(bt_sdio_fd);
            }
        }
        break;

        case BT_VND_OP_GET_LPM_IDLE_TIMEOUT:  //5
        {
            uint32_t *timeout_ms = (uint32_t *)param;
            *timeout_ms = hw_lpm_get_idle_timeout();
        }
        break;

        case BT_VND_OP_LPM_SET_MODE:  //6
        {
            uint8_t *mode = (uint8_t *)param;
            int err = -1;
            int retry_cnt = 0;

            if (amlbt_transtype.family_id > AML_W1U && amlbt_transtype.interface == AML_INTF_USB)
            {
                if (*mode == BT_VND_LPM_DISABLE)
                {
                    hw_reset_close();
                    do
                    {
                        usleep(5000);
                        if (retry_cnt > TIMEOUT_TRYSUM)
                        {
                            state = HW_DISBT_CONFIGURE;
                            break;
                        }
                        retry_cnt++;
                    }while (state == HW_RESET_CLOSE);

                    hw_disbt_configure();
                    do
                    {
                        usleep(5000);
                        if (retry_cnt > 2*TIMEOUT_TRYSUM)
                        {
                            property_get(PWR_PROP_NAME, shutdwon_status, "unknown");
                            if (strstr(shutdwon_status, "0userrequested") == NULL)
                            {
                                state = HW_REG_PUM_CLEAR;
                            }
                            else
                            {
                                state = HW_CLEAR_LIST;
                            }
                            break;
                        }
                        retry_cnt++;
                    }while (state == HW_DISBT_CONFIGURE);

                    property_get(PWR_PROP_NAME, shutdwon_status, "unknown");
                    if (strstr(shutdwon_status, "0userrequested") == NULL)
                    {
                        hw_reg_pum_power_cfg_clear();
                        do
                        {
                            usleep(5000);
                            if (retry_cnt > 3*TIMEOUT_TRYSUM)
                            {
                                state = HW_RESET_CLOSE;
                                break;
                            }
                            retry_cnt++;
                        }while (state == HW_REG_PUM_CLEAR);
                    }
                    else
                    {
                        hw_poweroff_clear_list();
                        do
                        {
                            usleep(5000);
                            if (retry_cnt > 3*TIMEOUT_TRYSUM)
                            {
                                state = HW_RESET_CLOSE;
                                break;
                            }
                            retry_cnt++;
                        }while (state == HW_CLEAR_LIST);
                    }
                }
            }
            if (amlbt_transtype.family_id == AML_W1U && amlbt_transtype.interface == AML_INTF_USB)
            {
                if (*mode == BT_VND_LPM_DISABLE)
                {
                    hw_reset_close();
                }
            }
            /*if (amlbt_transtype.family_id == AML_W1U && amlbt_transtype.interface == AML_INTF_SDIO)
            {
                err = pthread_create(&amlbt_thread_recovery, NULL, amlbt_recovery_init, NULL);
                if (err != 0)
                {
                    ALOGE("can't create thread");
                }
            }*/
            retval = hw_lpm_enable(*mode);
        }
        break;

        case BT_VND_OP_LPM_WAKE_SET_STATE:  //7
        {
            uint8_t *state = (uint8_t *)param;
            uint8_t wake_assert = (*state == BT_VND_LPM_WAKE_ASSERT) ? \
                                  TRUE : FALSE;

            hw_lpm_set_wake_state(wake_assert);
        }
        break;

        case BT_VND_OP_SET_AUDIO_STATE:	//8
        {
            retval = hw_set_audio_state((bt_vendor_op_audio_state_t *)param);
        }
        break;

        case BT_VND_OP_EPILOG:
        {
#if (HW_END_WITH_HCI_RESET == FALSE)
            if (bt_vendor_cbacks)
            {
                bt_vendor_cbacks->epilog_cb(BT_VND_OP_RESULT_SUCCESS);
            }
#else
            hw_epilog_process();
#endif
        }
        break;

        case BT_VND_OP_A2DP_OFFLOAD_START:
        case BT_VND_OP_A2DP_OFFLOAD_STOP:
        default:
            break;
    }

    return retval;
}

/** Closes the interface */
static void cleanup(void)
{
    BTVNDDBG("cleanup");

    upio_cleanup();

    bt_vendor_cbacks = NULL;
}

/* | HCI_ZIGBEE_FLAG | MHDL |           MID        | BODY LENGTH | checksum |
*  |   0x10 1 Byts   | 0xFA | command id 1 Bytes   |    2 Bytes  |  2 Bytes |
*/
void aml_15p4_data_cb(unsigned char *p_mem)
{
    unsigned short len = ((p_mem[3] << 8) | p_mem[2]);
    int size;
    size = write(S15P4_WF, p_mem, len + 4 + 2);
    if (size != len + 4 + 2)
    {
        ALOGI("%s packed error exit\n", __func__);
        //exit(-1);
    }
}

void aml_15p4_deinit(void)
{
    ALOGI("%s", __func__);
    if (S15P4_WF != -1)
    {
        close(S15P4_WF);
    }
    if (S15P4_RF != -1)
    {
        close(S15P4_RF);
    }
}

void aml_15p4_handle(void)
{
    int data_len;
    unsigned int read_len = 0;
    fd_set read_fd;
    fd_set error_fd;
    struct timeval timeout = {0, 500000};
    int rval;
    char thread_buf[AML_15P4_CMD_BUF_SIZE] = {0};
    if (mkfifo(rev_15p4_cmd_fifo, S_IFIFO | 0775) < 0 && errno != EEXIST)
    {
        ALOGI("%s mkfifo cmd error %s", __func__, strerror(errno));
        return;
    }
    if (mkfifo(rsp_15p4_rst_fifo, S_IFIFO | 0775) < 0 && errno != EEXIST)
    {
        ALOGI("%s mkfifo rst error %s", __func__, strerror(errno));
        return;
    }
    S15P4_RF = open(rev_15p4_cmd_fifo, O_RDONLY);
    if (S15P4_RF == -1)
    {
        ALOGI("%s can not open zigbee tx pipe error %s", __func__, strerror(errno));
        return;
    }
    S15P4_WF = open(rsp_15p4_rst_fifo, O_WRONLY);
    if (S15P4_WF == -1)
    {
        ALOGI("%s can not open zigbee rx pipe error %s", __func__, strerror(errno));
        return;
    }

    FD_ZERO(&read_fd);
    FD_SET(S15P4_RF, &read_fd);

    while (exit_thread == false)
    {
        rval = select(S15P4_RF + 1, &read_fd, NULL, NULL, &timeout);
        if (rval > 0)
        {
            if (FD_ISSET(S15P4_RF, &error_fd))
            {
                ALOGI("%s thread read file error exit\n", __func__);
                exit(-1);
            }
            else if (FD_ISSET(S15P4_RF, &read_fd))
            {
                memset(thread_buf, 0, sizeof(thread_buf));
                data_len = read(S15P4_RF, thread_buf, 5);
                if (data_len < 0)
                {
                    ALOGI("%s read error exit %s\n", __func__, strerror(errno));
                    exit(-1);
                }
                while (data_len && data_len < 5)
                {
                  read_len = read(S15P4_RF, &thread_buf[data_len], 5 - data_len);
                  data_len += read_len;
                }
                if ((*(unsigned short*)&thread_buf[0]) == 0xfa10)
                {
                    ALOGD("15.4 head:len %d,[%#x,%#x,%#x,%#x,%#x]",data_len, thread_buf[0], thread_buf[1],
                        thread_buf[2], thread_buf[3], thread_buf[4]);
                    read_len = *(unsigned short*)&thread_buf[3];
                    ALOGD("15p4 payload len = %d ", read_len);
                    data_len = read(S15P4_RF, &thread_buf[5], read_len + 2);
                    if (data_len > 0)
                    {
                        ALOGD("15p4 data len = %d ", read_len + 6);
                        aml_15p4_tx(&thread_buf[1], read_len + 6);
                    }
                }
            }
        }
        timeout.tv_sec = 0;
        timeout.tv_usec = 500000;
        FD_ZERO(&read_fd);
        FD_SET(S15P4_RF, &read_fd);
    }
    ALOGI("exit aml_thread thread");
}

// Entry point of DLib
const bt_vendor_interface_t BLUETOOTH_VENDOR_LIB_INTERFACE =
{
    sizeof(bt_vendor_interface_t),
    init,
    op,
    cleanup
};
