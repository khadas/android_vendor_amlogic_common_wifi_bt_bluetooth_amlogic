#include "bt_hal_plateform.h"
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/platform_device.h>


extern unsigned char (*host_wake_w1_req)(void);
static int reg_config_complete = 0;
static struct cdev BTAML_cdev;
static int BTAML_major;
static int BTAML_devs = 1;
static struct class *pBTClass;
static struct device *pBTDev;
#define BT_NODE "stpbt"
#define BT_DRIVER_NAME "sdio_bt"
#define BT_AML_SDIOBT_VERSION "v2.0.2_20230913_sdiobt"

static char *chip_name = "aml_w1";
//extern unsigned int amlbt_poweron;
extern void aml_wifi_sdio_power_lock(void);
extern void aml_wifi_sdio_power_unlock(void);

static int bt_aml_insmod(void);
static void bt_aml_rmmod(void);
extern unsigned int get_resume_method(void);

static int config_bt_pmu_reg(bool is_power_on)
{
    unsigned int value_pmu_A12 = 0, value_pmu_A13 = 0, value_pmu_A14 = 0, value_pmu_A15 = 0,
                 value_pmu_A16 = 0, value_pmu_A17 = 0, value_pmu_A18 = 0, value_pmu_A20 = 0,
                 value_pmu_A22 = 0, bt_pmu_status = 0, host_req_status = 0, value_aon_a15 = 0;
    int ret = 0;
    unsigned char pmu_fsm = 0;
    unsigned int reg_addr_mapping_form_pmu_fsm = 0;
    unsigned int reg_data = 0;
    unsigned int bt_pmu_status_check = 0;
    unsigned int wait_count = 0;

    /* set wifi keep alive, BIT(5)*/
    if (g_w1_hif_ops.hi_bottom_read8 == NULL)
    {
        PRINT("config_bt_pmu_reg(): can't get g_w1_hif_ops interface!!!!!!!!!!\n");
        return (-1);
    }
    host_req_status = g_w1_hif_ops.hi_bottom_read8(SDIO_FUNC1, RG_BT_SDIO_PMU_HOST_REQ);
    PRINT("host_req_status = 0x%x\n", host_req_status);
    host_req_status |= BIT(5);
    g_w1_hif_ops.hi_bottom_write8(SDIO_FUNC1, RG_BT_SDIO_PMU_HOST_REQ, host_req_status);

    /* wake wifi fw firstly */
    if (host_wake_w1_req != NULL)
    {
        PRINT("BT lock\n");
        aml_wifi_sdio_power_lock();
        while (host_wake_w1_req() == 0)
        {
            msleep(10);
            PRINT("BT insmod, wake wifi failed\n");
        }
        aml_wifi_sdio_power_unlock();
        PRINT("BT unlock\n");
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
        PRINT("BT power on: before write A12=0x%x, A13=0x%x, A14=0x%x, A15=0x%x, A16=0x%x, A17=0x%x, A18=0x%x, A20=0x%x, A22=0x%x\n",
              value_pmu_A12, value_pmu_A13, value_pmu_A14, value_pmu_A15, value_pmu_A16, value_pmu_A17, value_pmu_A18, value_pmu_A20,
              value_pmu_A22);

        /* set bt work flag && xosc/bbpll/ao_iso/pmu mask*/
        g_w1_hif_ops.bt_hi_write_word(RG_BT_PMU_A14, 0x1f);
        PRINT("BT power on:RG_BT_PMU_A14 = 0x%x\n", g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A14));

        /* release pmu fsm : bit0 */
        host_req_status = g_w1_hif_ops.hi_bottom_read8(SDIO_FUNC1, RG_BT_SDIO_PMU_HOST_REQ);
        host_req_status &= ~BIT(0);
        g_w1_hif_ops.hi_bottom_write8(SDIO_FUNC1, RG_BT_SDIO_PMU_HOST_REQ, host_req_status);
        msleep(10);

        /* reset bt, then release bt */
        g_w1_hif_ops.bt_hi_write_word(RG_BT_PMU_A17, 0x700);
        msleep(1);
        PRINT("BT power on:RG_BT_PMU_A17 = 0x%x\n", g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A17));
        g_w1_hif_ops.bt_hi_write_word(RG_BT_PMU_A20, 0x0);
        msleep(1);
        PRINT("BT power on:RG_BT_PMU_A20 = 0x%x\n", g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A20));
        //g_w1_hif_ops.bt_hi_write_word(RG_BT_PMU_A12, value_pmu_A12|0xc0);
        //msleep(1);
        //PRINT("BT power on:RG_BT_PMU_A12 = 0x%x\n", g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A12));
        g_w1_hif_ops.bt_hi_write_word(RG_BT_PMU_A18, 0x1700);
        PRINT("BT power on: %s, line=%d\n", __func__, __LINE__);
        msleep(1);
        PRINT("BT power on:RG_BT_PMU_A18 = 0x%x\n", g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A18));
        g_w1_hif_ops.bt_hi_write_word(RG_BT_PMU_A22, 0x704);
        msleep(10);
        PRINT("BT power on:RG_BT_PMU_A22 = 0x%x\n", g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A22));

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
        PRINT("BT power on:RG_BT_PMU_A12 = 0x%x\n", g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A12));

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
        PRINT("%s bt_pmu_status:0x%x\n", __func__, bt_pmu_status);

        /* wait bt pmu fsm to PMU_ACT_MODE*/
        while ((bt_pmu_status & 0xF) != PMU_ACT_MODE)
        {
            msleep(5);
            PRINT("%s bt_pmu_status:0x%x\n", __func__, bt_pmu_status);
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
        PRINT("BT power on: after write A12=0x%x, A13=0x%x, A14=0x%x, A15=0x%x, A16=0x%x, A17=0x%x, A18=0x%x, A20=0x%x, A22=0x%x\n",
              value_pmu_A12, value_pmu_A13, value_pmu_A14, value_pmu_A15, value_pmu_A16, value_pmu_A17, value_pmu_A18, value_pmu_A20,
              value_pmu_A22);
    }
    else    // turn off bt
    {
        value_pmu_A16 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A16);

        value_pmu_A16 |= (0x1 << 30); //bit30 = 1, make sure can't enter power down when turn off BT
        g_w1_hif_ops.bt_hi_write_word(RG_BT_PMU_A16, value_pmu_A16);
        PRINT("Enable RG_BT_PMU_A16 BIT30, FW can't enter power down!\n");

        value_pmu_A12 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A12);
        value_pmu_A13 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A13);
        value_pmu_A14 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A14);
        value_pmu_A15 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A15);

        value_pmu_A17 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A17);
        value_pmu_A18 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A18);
        value_pmu_A20 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A20);
        value_pmu_A22 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A22);
        value_aon_a15 = g_w1_hif_ops.bt_hi_read_word(RG_AON_A15);

        PRINT("BT power off: before write A12=0x%x, A13=0x%x, A14=0x%x, A15=0x%x, A16=0x%x, A17=0x%x, A18=0x%x, A20=0x%x, A22=0x%x, aon_a15=0x%x\n",
              value_pmu_A12, value_pmu_A13, value_pmu_A14, value_pmu_A15, value_pmu_A16, value_pmu_A17, value_pmu_A18, value_pmu_A20,
              value_pmu_A22, value_aon_a15);

        //if (value_pmu_A15 == 8)
        {
            value_pmu_A16 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A16);
            value_pmu_A16 &= 0xFffffffE; //bit0 =0
            g_w1_hif_ops.bt_hi_write_word(RG_BT_PMU_A16, value_pmu_A16);
            msleep(10);

            value_pmu_A16 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A16);
            PRINT("RG_BT_PMU_A16_0 = 0x%x\n", value_pmu_A16);

            value_pmu_A16 &= 0xFffffffD; //bit1 =0
            g_w1_hif_ops.bt_hi_write_word(RG_BT_PMU_A16, value_pmu_A16);
            msleep(10);

            if (value_pmu_A15 == 8)
            {
                PRINT("Is sleep mode, wakeup first!\n");
                value_pmu_A16 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A16);
                PRINT("RG_BT_PMU_A16_1 = 0x%x\n", value_pmu_A16);
                value_pmu_A16 |= 0x2; //bit1 = 1
                g_w1_hif_ops.bt_hi_write_word(RG_BT_PMU_A16, value_pmu_A16);
                msleep(10);
            }

            value_pmu_A16 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A16);
            PRINT("RG_BT_PMU_A16_2 = 0x%x\n", value_pmu_A16);
        }

        value_pmu_A16 &= 0x7fffffff;
        g_w1_hif_ops.bt_hi_write_word(RG_BT_PMU_A16, value_pmu_A16);
        msleep(1);
        PRINT("BT power off:RG_BT_PMU_A16 = 0x%x\n", g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A16));

        PRINT("Check whether is active mode\n");
        bt_pmu_status = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A15);
        bt_pmu_status_check = bt_pmu_status;
        //if (!strcmp(chip_name, "aml_w1u_s"))
        bt_pmu_status_check = bt_pmu_status & 0x0f;
        while (bt_pmu_status_check != PMU_ACT_MODE)
        {
            msleep(10);
            bt_pmu_status = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A15);
            PRINT("wait wakeup, RG_BT_PMU_A15 = 0x%x\n", bt_pmu_status);
            wait_count++;

            if (wait_count > 50)    //
            {
#if 0
                /* set bt pmu fsm to PMU_SLEEP_MODE */
                PRINT("Force BT power off\n");
                host_req_status = g_w1_hif_ops.hi_bottom_read8(SDIO_FUNC1, RG_BT_SDIO_PMU_HOST_REQ);
                host_req_status |= (PMU_SLEEP_MODE << 1) | BIT(0);
                g_w1_hif_ops.hi_bottom_write8(SDIO_FUNC1, RG_BT_SDIO_PMU_HOST_REQ, host_req_status);

                PRINT("Force BT power on\n");
                /* release pmu fsm : bit0 */
                host_req_status = g_w1_hif_ops.hi_bottom_read8(SDIO_FUNC1, RG_BT_SDIO_PMU_HOST_REQ);
                host_req_status &= ~BIT(0);
                g_w1_hif_ops.hi_bottom_write8(SDIO_FUNC1, RG_BT_SDIO_PMU_HOST_REQ, host_req_status);
                msleep(10);
#else
                // add workaroud for pmu lock, when pmu lock trig the bnd let pmu fsm switch to next state.
                pmu_fsm = (g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A15) & 0xf);

                //if (pmu_fsm == PMU_SLEEP_MODE)
                //{
                //	value_pmu_A16 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A16);
                //	value_pmu_A16 |= 0x2;
                //	g_w1_hif_ops.bt_hi_write_word(RG_BT_PMU_A16, value_pmu_A16);
                //}
                //else
                {
                    PRINT("trig pmu bnd to unlock pmu fsm, pmu_state=0x%x\n", pmu_fsm);
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
        g_w1_hif_ops.bt_hi_write_word(RG_BT_PMU_A12, value_pmu_A12 | 0xc0);
        msleep(1);

        /* reset bt */
        g_w1_hif_ops.bt_hi_write_word(RG_BT_PMU_A22, 0x707);
        msleep(1);
        PRINT("BT power off:RG_BT_PMU_A22 = 0x%x\n", g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A22));
        g_w1_hif_ops.bt_hi_write_word(RG_BT_PMU_A18, 0x1787);
        msleep(1);
        PRINT("BT power off:RG_BT_PMU_A18 = 0x%x\n", g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A18));
        g_w1_hif_ops.bt_hi_write_word(RG_BT_PMU_A20, 0x0);      //0x1f703007
        msleep(1);
        PRINT("BT power off:RG_BT_PMU_A20 = 0x%x\n", g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A20));
        g_w1_hif_ops.bt_hi_write_word(RG_BT_PMU_A17, 0x707);
        msleep(1);
        PRINT("BT power off:RG_BT_PMU_A17 = 0x%x\n", g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A17));

        /* clear bt work flag for coex */
        //value_pmu_A16 &= 0x7fffffff;
        //g_w1_hif_ops.bt_hi_write_word(RG_BT_PMU_A16, value_pmu_A16);
        //PRINT("BT power off:RG_BT_PMU_A16 = 0x%x\n", g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A16));

        value_pmu_A12 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A12);
        value_pmu_A13 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A13);
        value_pmu_A14 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A14);
        value_pmu_A15 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A15);
        value_pmu_A16 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A16);
        value_pmu_A17 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A17);
        value_pmu_A18 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A18);
        value_pmu_A20 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A20);
        value_pmu_A22 = g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A22);
        PRINT("BT power off: after write A12=0x%x, A13=0x%x, A14=0x%x, A15=0x%x, A16=0x%x, A17=0x%x, A18=0x%x, A20=0x%x, A22=0x%x\n",
              value_pmu_A12, value_pmu_A13, value_pmu_A14, value_pmu_A15, value_pmu_A16, value_pmu_A17, value_pmu_A18, value_pmu_A20,
              value_pmu_A22);

        /* clear bt work flag && mask */
        g_w1_hif_ops.bt_hi_write_word(RG_BT_PMU_A14, 0x0);
        PRINT("BT power off: %s, line=%d\n", __func__, __LINE__);
        //PRINT("BT power off:RG_BT_PMU_A14 = 0x%x\n", g_w1_hif_ops.bt_hi_read_word(RG_BT_PMU_A14));

        /* set bt pmu fsm to PMU_SLEEP_MODE */
        host_req_status = g_w1_hif_ops.hi_bottom_read8(SDIO_FUNC1, RG_BT_SDIO_PMU_HOST_REQ);
        host_req_status |= (PMU_SLEEP_MODE << 1) | BIT(0);
        g_w1_hif_ops.hi_bottom_write8(SDIO_FUNC1, RG_BT_SDIO_PMU_HOST_REQ, host_req_status);
        PRINT("BT power off: %s, line=%d\n", __func__, __LINE__);
    }

    /* clear wifi keep alive, BIT(5)*/
    host_req_status = g_w1_hif_ops.hi_bottom_read8(SDIO_FUNC1, RG_BT_SDIO_PMU_HOST_REQ);
    host_req_status &= ~BIT(5);
    g_w1_hif_ops.hi_bottom_write8(SDIO_FUNC1, RG_BT_SDIO_PMU_HOST_REQ, host_req_status);

    reg_config_complete = 1;

    return ret;
}

extern unsigned char wifi_in_insmod;
int  bt_aml_sdio_init(void)
{
    int ret = 0;
    int cnt = 0;

    //amlwifi_set_sdio_host_clk(50000000);

    if (!w1_sdio_driver_insmoded)
    {
        PRINT("===start register sdio common driver through bt sdio driver===\n");
        aml_w1_sdio_init();
        msleep(10);

        while (!w1_sdio_after_porbe)
        {
            PRINT("waiting sdio common driver probe complete\n");
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
            PRINT("WIFI in insmod\n");
            msleep(10);
        }
    }

    ret = config_bt_pmu_reg(BT_PWR_ON);

    return ret;
}


static int btaml_fops_open(struct inode *inode, struct file *file)
{
    if (!reg_config_complete)
    {
        PRINT("%s reg_config_complete is %d return\n",
              __func__, reg_config_complete);
        return -EFAULT;
    }

    PRINT("%s reg_config_complete is 1\n", __func__);
    printk("\n");
    printk("\n");
    printk("\n");

    return 0;
}

static int amlbt_sdio_probe(struct platform_device *dev)
{
    int ret = bt_aml_insmod();

    unsigned int reg_value = g_w1_hif_ops.bt_hi_read_word(RG_AON_A15);

    printk("%s RG_AON_A15:%#x\n", __func__, reg_value);

    reg_value &= ~(1<<31);
    reg_value &= ~(1<<30);
    g_w1_hif_ops.bt_hi_write_word(RG_AON_A15, reg_value);
    printk("RG_AON_A15:%#x", g_w1_hif_ops.bt_hi_read_word(RG_AON_A15));
    return ret;
}

static int amlbt_sdio_remove(struct platform_device *dev)
{
    printk("%s \n", __func__);

    bt_aml_rmmod();

    return 0;
}

static int amlbt_sdio_suspend(struct platform_device *dev, pm_message_t state)
{
    unsigned int reg_value = g_w1_hif_ops.bt_hi_read_word(RG_AON_A15);
    printk("%s RG_AON_A15:%#x\n", __func__, reg_value);
    reg_value |= (1<<31);
    g_w1_hif_ops.bt_hi_write_word(RG_AON_A15, reg_value);
    printk("RG_AON_A15:%#x", g_w1_hif_ops.bt_hi_read_word(RG_AON_A15));
    return 0;
}

static int amlbt_sdio_resume(struct platform_device *dev)
{
    if ((get_resume_method() != 3) && (get_resume_method() != 7))
    {
        unsigned int reg_value = g_w1_hif_ops.bt_hi_read_word(RG_AON_A15);
        printk("%s RG_AON_A15:%#x\n", __func__, reg_value);
        reg_value &= ~(1<<31);
        g_w1_hif_ops.bt_hi_write_word(RG_AON_A15, reg_value);
        printk("RG_AON_A15:%#x", g_w1_hif_ops.bt_hi_read_word(RG_AON_A15));
    }
    return 0;
}

static void amlbt_sdio_shutdown(struct platform_device *dev)
{
    printk("%s \n", __func__);
}

const struct file_operations BTAML_fops =
{
    .open       = btaml_fops_open,
    .release    = NULL,
    .read       = NULL,
    .write      = NULL,
    .poll       = NULL,
    .unlocked_ioctl = NULL,
    .fasync     = NULL
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

static void amlbt_dev_release(struct device* dev)
{
    return;
}


static struct platform_device amlbt_sdio_device = {
    .name    = "sdio_bt",
    .id      = -1,
    .dev     = {
        .release = &amlbt_dev_release,
    }
};


static int BTAML_init(void)
{
    int ret = 0;
    int cdevErr = 0;

    dev_t devID = MKDEV(BTAML_major, 0);
    PRINT("BTAML_init\n");

    ret = alloc_chrdev_region(&devID, 0, 1, "BT_sdiodev");
    if (ret)
    {
        pr_err("fail to allocate chrdev\n");
        return ret;
    }

    BTAML_major = MAJOR(devID);
    PRINT("major number:%d\n", BTAML_major);
    cdev_init(&BTAML_cdev, &BTAML_fops);
    BTAML_cdev.owner = THIS_MODULE;

    cdevErr = cdev_add(&BTAML_cdev, devID, BTAML_devs);
    if (cdevErr)
        goto error;

    PRINT("%s driver(major %d) installed.\n",
          "BT_sdiodev", BTAML_major);

    pBTClass = class_create(THIS_MODULE, "BT_sdiodev");
    if (IS_ERR(pBTClass))
    {
        pr_err("class create fail, error code(%ld)\n",
               PTR_ERR(pBTClass));
        goto err1;
    }

    pBTDev = device_create(pBTClass, NULL, devID, NULL, BT_NODE);
    if (IS_ERR(pBTDev))
    {
        pr_err("device create fail, error code(%ld)\n",
               PTR_ERR(pBTDev));
        goto err2;
    }

    PRINT("%s: BT_major %d\n", __func__, BTAML_major);
    PRINT("%s: devID %d\n", __func__, devID);

    return 0;

err2:
    if (pBTClass)
    {
        class_destroy(pBTClass);
        pBTClass = NULL;
    }

err1:

error:
    if (cdevErr == 0)
        cdev_del(&BTAML_cdev);

    if (ret == 0)
        unregister_chrdev_region(devID, BTAML_devs);

    return -1;
}


static void BTAML_exit(void)
{
    dev_t dev = MKDEV(BTAML_major, 0);

    PRINT("%s 1\n", __func__);
    if (pBTDev)
    {
        device_destroy(pBTClass, dev);
        pBTDev = NULL;
    }
    PRINT("%s 2\n", __func__);
    if (pBTClass)
    {
        class_destroy(pBTClass);
        pBTClass = NULL;
    }
    PRINT("%s 3\n", __func__);
    cdev_del(&BTAML_cdev);

    PRINT("%s 4\n", __func__);
    unregister_chrdev_region(dev, 1);

    PRINT("%s driver removed.\n", BT_DRIVER_NAME);
}


static int bt_aml_insmod(void)
{
    int ret = 0;

    //if (amlbt_poweron == AML_SDIO_EN)
    {
        PRINT("BTAML SDIOBT version:%s\n", BT_AML_SDIOBT_VERSION);
        PRINT("++++++sdio bt driver insmod start.++++++\n");
        reg_config_complete = 0;

        set_wifi_bt_sdio_driver_bit(REGISTER_BT_SDIO, BT_BIT);
        ret = bt_aml_sdio_init();
        if (ret)
        {
            pr_err("%s: bt sdio driver init failed!\n", __func__);
            return ret;
        }
        ret = BTAML_init();
        if (ret)
        {
            pr_err("%s: BTAML_init failed!\n", __func__);
            return ret;
        }
    }

    PRINT("------sdio bt driver insmod end.------\n");

    return ret;
}

static void bt_aml_rmmod(void)
{
    //if (amlbt_poweron == AML_SDIO_EN)
    {
        printk("\n");
        printk("\n");
        printk("\n");
        PRINT("++++++sdio bt driver rmmod start.++++++\n");
        while (wifi_in_insmod)
        {
            PRINT("WIFI in insmod\n");
            msleep(10);
        }
        config_bt_pmu_reg(BT_PWR_OFF);
        set_wifi_bt_sdio_driver_bit(UNREGISTER_BT_SDIO, BT_BIT);
        BTAML_exit();
        PRINT("------sdio bt driver rmmod end.------\n");
        printk("\n");
        printk("\n");
        printk("\n");
    }
}

static int __init amlbt_sdio_init(void)
{
    int ret = 0;

    ret = platform_device_register(&amlbt_sdio_device);
    if (ret)
    {
        dev_err(&amlbt_sdio_device.dev, "platform_device_register failed!\n");
        return ret;
    }

    ret = platform_driver_register(&amlbt_sdio_driver);
    if (ret)
    {
        dev_err(&amlbt_sdio_device.dev, "platform_driver_register failed!\n");
        return ret;
    }

    dev_info(&amlbt_sdio_device.dev, "Init %s OK!\n", "sdio_bt");

    return ret;
}

static void __exit amlbt_sdio_exit(void)
{
    platform_driver_unregister(&amlbt_sdio_driver);
    platform_device_unregister(&amlbt_sdio_device);
}

module_param(chip_name, charp, S_IRUGO);
module_init(amlbt_sdio_init);
module_exit(amlbt_sdio_exit);
MODULE_LICENSE("GPL");
