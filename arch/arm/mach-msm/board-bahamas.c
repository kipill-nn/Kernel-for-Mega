/* arch/arm/mach-msm/board-bahamas.c
 * Copyright (C) 2007-2009 HTC Corporation.
 * Author: Tony Liu <tony_liu@htc.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/leds.h>
#include <linux/switch.h>
#include <linux/synaptics_i2c_rmi.h>
#include <linux/akm8973.h>
#include <linux/bma150.h>
#include <linux/sysdev.h>
#include <linux/android_pmem.h>

#include <linux/delay.h>

#include <asm/gpio.h>
#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>
#include <asm/system.h>
#include <mach/system.h>
#include <mach/vreg.h>
#include <mach/h2w.h>
#include <mach/audio_jack.h>

#include <asm/io.h>
#include <asm/delay.h>
#include <asm/setup.h>

#include <linux/gpio_event.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>

#include <asm/mach/mmc.h>
#include <linux/mmc/sdio_ids.h>

#include "board-bahamas.h"
#include "proc_comm.h"
#include "gpio_chip.h"

#include <mach/board.h>
#include <mach/board_htc.h>
#include <mach/msm_serial_hs.h>

#include "devices.h"

#include <mach/microp_i2c.h>
#include <mach/msm_tssc.h>
#include <linux/keyreset.h>

#include <mach/htc_pwrsink.h>
#include <mach/perflock.h>
#include <mach/drv_callback.h>
#include <mach/camera.h>  /*CC090518*/

void msm_init_irq(void);
void msm_init_gpio(void);
void msm_init_pmic_vibrator(void);
void config_bahamas_camera_on_gpios(void);
void config_bahamas_camera_off_gpios(void);

extern int bahamas_init_mmc(unsigned int);
/*CC090319*/
static unsigned int hwid;
static unsigned int skuid;
static unsigned engineerid;
/*~CC090319*/

enum {
	PANEL_HITACHI = 0,
	PANEL_WINTEK,
	PANEL_SAMSUNG,
};

#define LCM_ID0 57
#define LCM_ID1 58

static int panel_detect(void)
{
	int panel_id = -1 ;

	panel_id = ((gpio_get_value(LCM_ID1) << 1) | gpio_get_value(LCM_ID0));

	switch(panel_id) {
	case PANEL_HITACHI:
		break ;
	case PANEL_WINTEK:
		break ;
	case PANEL_SAMSUNG:
		break ;
	default:
		printk("%s: Invalid panel id !\n", __FUNCTION__ ) ;
		break ;
	}

	return panel_id ;
}

//Samsung panel
static struct microp_pin_config microp_pins_0[] = {
	{
		.name   = "green",
		.pin    = 3,
		.config = MICROP_PIN_CONFIG_GPO_INV,
	},
	{
		.name   = "amber",
		.pin    = 5,
		.config = MICROP_PIN_CONFIG_GPO_INV,
	},
	{
		.name   = "lcd-backlight",
		.pin    = 6,
		.config = MICROP_PIN_CONFIG_PWM,
		.freq   = MICROP_PIN_PWM_FREQ_HZ_15600,
		.levels = { 3, 31, 59, 87, 115, 143, 171, 199, 227, 255 },
		.dutys	= { 10, 16, 39, 74, 118, 154, 180, 207, 235, 255 },
	},
	{
		.name	= "button-backlight",
		.pin	= 10,
		.config = MICROP_PIN_CONFIG_GPO,
	},
	{
		.name   = "adc",
		.pin    = 24,
		.config = MICROP_PIN_CONFIG_ADC,
		.levels = { 0, 0, 0, 6, 51, 319, 425, 497, 569, 638 },
	}
};

// Wintek panel
static struct microp_pin_config microp_pins_0_wint[] = {
	{
		.name   = "green",
		.pin    = 3,
		.config = MICROP_PIN_CONFIG_GPO_INV,
	},
	{
		.name   = "amber",
		.pin    = 5,
		.config = MICROP_PIN_CONFIG_GPO_INV,
	},
	{
		.name   = "lcd-backlight",
		.pin    = 6,
		.config = MICROP_PIN_CONFIG_PWM,
		.freq   = MICROP_PIN_PWM_FREQ_HZ_15600,
		.levels = { 3, 31, 59, 87, 115, 143, 171, 199, 227, 255 },
		.dutys	= { 10, 16, 39, 74, 118, 154, 176, 201, 226, 240 },
	},
	{
		.name	= "button-backlight",
		.pin	= 10,
		.config = MICROP_PIN_CONFIG_GPO,
	},
	{
		.name   = "adc",
		.pin    = 24,
		.config = MICROP_PIN_CONFIG_ADC,
		.levels = { 0, 0, 0, 6, 51, 319, 425, 497, 569, 638 },
	}
};

// XD after, Samsung panel
static struct microp_pin_config microp_pins_1[] = {
	{	.name = "microp-pullup",
		.pin = 23,
		.config = MICROP_PIN_CONFIG_PULL_UP1,
		.mask = { 0x00, 0x00, 0x02 },
	},
	MICROP_PIN(0, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(1, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(2, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(4, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(7, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(8, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(9, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(11, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(12, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(13, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(14, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(15, MICROP_PIN_CONFIG_GPO),
	{
		.name   = "green",
		.pin    = 3,
		.config = MICROP_PIN_CONFIG_GPO_INV,
	},
	{
		.name   = "amber",
		.pin    = 5,
		.config = MICROP_PIN_CONFIG_GPO_INV,
	},
	{
		.name   = "lcd-backlight",
		.pin    = 6,
		.config = MICROP_PIN_CONFIG_PWM,
		.freq   = MICROP_PIN_PWM_FREQ_HZ_15600,
		.levels = { 3, 31, 59, 87, 115, 143, 171, 199, 227, 255 },
		.dutys	= { 10, 16, 39, 74, 118, 154, 180, 207, 235, 255 },
	},
	{
		.name	= "button-backlight",
		.pin	= 10,
		.config = MICROP_PIN_CONFIG_GPO,
	},
	{
		.name = "microp_11pin_mic",
		.pin = 16,
		.config = MICROP_PIN_CONFIG_MIC,
		.init_value = 1,
	},
	{
		.name	= "microp_intrrupt",
		.pin	= 17,
		.config  = MICROP_PIN_CONFIG_INTR_ALL,
		.mask	 = { 0x00, 0x00, 0x00 },
	},
	{
		.name   = "audio_remote_sensor",
		.pin    = 25,
		.adc_pin = 7,
		.config = MICROP_PIN_CONFIG_ADC_READ,
	}
};

// XD after, Wintek panel
static struct microp_pin_config microp_pins_1_wint[] = {
	{	.name = "microp-pullup",
		.pin = 23,
		.config = MICROP_PIN_CONFIG_PULL_UP1,
		.mask = { 0x00, 0x00, 0x02 },
	},
	MICROP_PIN(0, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(1, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(2, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(4, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(7, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(8, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(9, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(11, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(12, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(13, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(14, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(15, MICROP_PIN_CONFIG_GPO),
	{
		.name   = "green",
		.pin    = 3,
		.config = MICROP_PIN_CONFIG_GPO_INV,
	},
	{
		.name   = "amber",
		.pin    = 5,
		.config = MICROP_PIN_CONFIG_GPO_INV,
	},
	{
		.name   = "lcd-backlight",
		.pin    = 6,
		.config = MICROP_PIN_CONFIG_PWM,
		.freq   = MICROP_PIN_PWM_FREQ_HZ_15600,
		.levels = { 3, 31, 59, 87, 115, 143, 171, 199, 227, 255 },
		.dutys	= { 10, 16, 39, 74, 118, 154, 176, 201, 226, 240 },
	},
	{
		.name	= "button-backlight",
		.pin	= 10,
		.config = MICROP_PIN_CONFIG_GPO,
	},
	{
		.name = "microp_11pin_mic",
		.pin = 16,
		.config = MICROP_PIN_CONFIG_MIC,
		.init_value = 1,
	},
	{
		.name	= "microp_intrrupt",
		.pin	= 17,
		.config  = MICROP_PIN_CONFIG_INTR_ALL,
		.mask	 = { 0x00, 0x00, 0x00 },
	},
	{
		.name   = "audio_remote_sensor",
		.pin    = 25,
		.adc_pin = 7,
		.config = MICROP_PIN_CONFIG_ADC_READ,
	}
};


static struct microp_i2c_platform_data microp_data = {
	.num_pins   = ARRAY_SIZE(microp_pins_1),
	.pin_config = microp_pins_1,
	.microp_enable_early_suspend = 1,
	.gpio_reset = BAHAMAS_GPIO_UP_RESET_N,
	.microp_enable_reset_button = 1,
};

static struct i2c_board_info i2c_microp_devices = {
	I2C_BOARD_INFO(MICROP_I2C_NAME, 0xCC >> 1),
	.platform_data = &microp_data,
	.irq = MSM_GPIO_TO_INT(BAHAMAS_XD_GPIO_UP_INT),
};

#if 0
static struct gpio_switch_platform_data sd_door_switch_data = {
	.name		= "sd-door",
	.gpio		= TROUT_GPIO_SD_DOOR_N,
	.state_on	= "open",
	.state_off	= "closed",
};

static struct platform_device sd_door_switch = {
	.name		= "switch-gpio",
	.id		= -1,
	.dev		= {
		.platform_data	= &sd_door_switch_data,
	},
};
#endif

#define MSM_PROBE_INIT(name) name##_probe_init   /*CC090518*/
static struct msm_camera_sensor_info msm_camera_sensor = {
	.sensor_reset	= 118,
	//.sensor_pwd	  = 107,
	//.vcm_pwd	= 117,     /* FIXME: by Bahamas GPIO table,*/
	.sensor_name  = "s5k4b2fx",
//	},
};
/*CC090519*/
int s5k4b2fx_probe_init(void *dev, void *ctrl)
{
	return -1;
}
/*CC090519~*/

/*CC090319*/
static struct msm_camera_sensor_info msm_camera_sensor_s5k4b2fx = {
	.sensor_reset	= 118,
	.sensor_name  = "s5k4b2fx",
	.sensor_probe = MSM_PROBE_INIT(s5k4b2fx),   /*CC090519*/
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9t013 = {
	.sensor_reset	= 118,  /*CC090319*/
	.sensor_pwd	  = BAHAMAS_MT9T013_CAM_PWDN,  /*CC090320, set CAM POWER DOWN GPIO=091*/
	.sensor_name  = "mt9t013",
	.sensor_probe = MSM_PROBE_INIT(mt9t013),   /*CC090518*/
};
/*~CC090319*/
#undef MSM_PROBE_INIT    /*CC090518*/

static struct msm_camera_platform_data msm_camera_device_data = {
	.camera_gpio_on  = config_bahamas_camera_on_gpios,
	.camera_gpio_off = config_bahamas_camera_off_gpios,
	.snum = 1,
	.sinfo = &msm_camera_sensor,
	/*CC090414*/
	.ioext.mdcphy = MSM_MDC_PHYS,
	.ioext.mdcsz  = MSM_MDC_SIZE,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
};

static struct akm8973_platform_data compass_platform_data = {
	.reset = BAHAMA_GPIO_COMPASS_RST_N,
	.intr = BAHAMA_GPIO_COMPASS_INT_N,
};

static struct bma150_platform_data gsensor_platform_data = {
	.intr = BAHAMA_GPIO_GSENSOR_INT_N,
};

/*CC090505*/
static struct i2c_board_info i2c_devices[] = {
	{
		/*CC090519*/
		/*I2C_BOARD_INFO("s5k4b2fx", 0x22 >> 1),*/
		I2C_BOARD_INFO("s5k4b2fx", 0x22),
		/*CC090519~*/
		/* .irq = TROUT_GPIO_TO_INT(TROUT_GPIO_CAM_BTN_STEP1_N), */
	},
	/*CC090414*/
	{
		I2C_BOARD_INFO("mt9t013", 0x6C),   /*3M bayer sensor driver*/    /*CC090518*/
		.platform_data = &msm_camera_device_data, /*CC090505*/
	},
};
/*CC090505~*/
static struct i2c_board_info i2c_sensor[] = {
	{
		I2C_BOARD_INFO(AKM8973_I2C_NAME, 0x1C),
		.platform_data = &compass_platform_data,
		.irq = MSM_GPIO_TO_INT(BAHAMA_GPIO_COMPASS_INT_N),
	},
	{
		I2C_BOARD_INFO(BMA150_G_SENSOR_NAME, 0x38),
		.platform_data = &gsensor_platform_data,
		.irq = MSM_GPIO_TO_INT(BAHAMA_GPIO_GSENSOR_INT_N),
	},
};


static struct platform_device msm_camera_device = {
	.name		= "msm_camera",
	//.id		= 0,
	.id		= -1,
	.dev		= {
		.platform_data = &msm_camera_device_data,
	},
};

static struct platform_device trout_camera = {
	.name		= "camera",
	.dev		= { 
		.platform_data = &msm_camera_device,
	},
};

/* Switch between UART3 and GPIO */
static uint32_t uart3_on_gpio_table[] = {
	/* RX */
	PCOM_GPIO_CFG(BAHAMAS_GPIO_UART3_RX, 1, GPIO_INPUT, GPIO_NO_PULL, 0),
	/* TX */
	PCOM_GPIO_CFG(BAHAMAS_GPIO_UART3_TX, 1, GPIO_OUTPUT, GPIO_NO_PULL, 0),
};

/* Set TX,RX to GPI */
static uint32_t uart3_off_gpi_table[] = {
	PCOM_GPIO_CFG(BAHAMAS_GPIO_H2W_DATA, 0,
		      GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA), /* RX, H2W DATA */
	PCOM_GPIO_CFG(BAHAMAS_GPIO_H2W_CLK, 0,
		      GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA), /* TX, H2W CLK */
};

static int bahamas_h2w_path = H2W_GPIO;

static void h2w_configure(int route)
{
	printk(KERN_INFO "H2W route = %s \n",
	       route == H2W_UART3 ? "UART3" : "GPIO");
	switch (route) {
	case H2W_UART3:
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX,
			      uart3_on_gpio_table+0, 0);
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX,
			      uart3_on_gpio_table+1, 0);
		bahamas_h2w_path = H2W_UART3;
		break;
	case H2W_GPIO:
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX,
			      uart3_off_gpi_table+0, 0);
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX,
			      uart3_off_gpi_table+1, 0);
		bahamas_h2w_path = H2W_GPIO;
		break;
	}
}

static void h2w_defconfig(void)
{
	h2w_configure(H2W_GPIO);
}

static void set_h2w_dat(int n)
{
	gpio_set_value(BAHAMAS_GPIO_H2W_DATA, n);
}

static void set_h2w_clk(int n)
{
	gpio_set_value(BAHAMAS_GPIO_H2W_CLK, n);
}

static void set_h2w_dat_dir(int n)
{
	if (n == 0) /* input */
		gpio_direction_input(BAHAMAS_GPIO_H2W_DATA);
	else
		gpio_configure(BAHAMAS_GPIO_H2W_DATA, GPIOF_DRIVE_OUTPUT);
}

static void set_h2w_clk_dir(int n)
{
	if (n == 0) /* input */
		gpio_direction_input(BAHAMAS_GPIO_H2W_CLK);
	else
		gpio_configure(BAHAMAS_GPIO_H2W_CLK, GPIOF_DRIVE_OUTPUT);
}

static int get_h2w_dat(void)
{
	return gpio_get_value(BAHAMAS_GPIO_H2W_DATA);
}

static int get_h2w_clk(void)
{
	return gpio_get_value(BAHAMAS_GPIO_H2W_CLK);
}

#ifdef CONFIG_HTC_HEADSET
static int set_h2w_path(const char *val, struct kernel_param *kp)
{
	int ret = -EINVAL;
	int enable;
	int current_h2w_path = bahamas_h2w_path;

	ret = param_set_int(val, kp);
	if (ret)
		return ret;
	
	if (bahamas_h2w_path == current_h2w_path) {
		printk(KERN_INFO "%s: H2W path has been set to %s\n", __func__,
			(bahamas_h2w_path == H2W_UART3) ? "UART3" : "GPIO");
		return ret;
	}

	switch (bahamas_h2w_path) {
	case H2W_GPIO:
		enable = 1;
		cnf_driver_event("H2W_enable_irq", &enable);
		break;
	case H2W_UART3:
		enable = 0;
		cnf_driver_event("H2W_enable_irq", &enable);
		break;
	default:
		bahamas_h2w_path = -1;
		return -EINVAL;
	}
	
	h2w_configure(bahamas_h2w_path);
	return ret;
}

module_param_call(h2w_path, set_h2w_path, param_get_int,
		&bahamas_h2w_path, S_IWUSR | S_IRUGO);
#endif

static struct h2w_platform_data bahamas_h2w_data = {
	.h2w_power		= BAHAMAS_GPIO_H2W_POWER,
	.cable_in1		= BAHAMAS_GPIO_CABLE_IN1,
	.cable_in2		= BAHAMAS_GPIO_CABLE_IN2,
	.h2w_clk		= BAHAMAS_GPIO_H2W_CLK,
	.h2w_data		= BAHAMAS_GPIO_H2W_DATA,
	.headset_mic_35mm	= BAHAMAS_GPIO_HEADSET_MIC,
	.ext_mic_sel		= BAHAMAS_GPIO_AUD_EXTMIC_SEL,
	.wfm_ant_sw		= BAHAMAS_GPIO_WFM_ANT_SW,
	.debug_uart 		= H2W_UART3,
	.config 		= h2w_configure,
	.defconfig 		= h2w_defconfig,
	.set_dat		= set_h2w_dat,
	.set_clk		= set_h2w_clk,
	.set_dat_dir		= set_h2w_dat_dir,
	.set_clk_dir		= set_h2w_clk_dir,
	.get_dat		= get_h2w_dat,
	.get_clk		= get_h2w_clk,
};

static struct platform_device bahamas_h2w = {
	.name		= "h2w",
	.id			= -1,
	.dev		= {
		.platform_data	= &bahamas_h2w_data,
	},
};

static struct audio_jack_platform_data bahamas_jack_data = {
	.gpio	= BAHAMAS_GPIO_35MM_HEADSET_DET,
};

static struct platform_device bahamas_audio_jack = {
	.name		= "audio-jack",
	.id			= -1,
	.dev		= {
		.platform_data	= &bahamas_jack_data,
	},
};

static struct pwr_sink bahamas_pwrsink_table[] = {
	{
		.id     = PWRSINK_AUDIO,
		.ua_max = 100000,
	},
	{
		.id     = PWRSINK_BACKLIGHT,
		.ua_max = 125000,
	},
	{
		.id     = PWRSINK_LED_BUTTON,
		.ua_max = 0,
	},
	{
		.id     = PWRSINK_LED_KEYBOARD,
		.ua_max = 0,
	},
	{
		.id     = PWRSINK_GP_CLK,
		.ua_max = 0,
	},
	{
		.id     = PWRSINK_BLUETOOTH,
		.ua_max = 15000,
	},
	{
		.id     = PWRSINK_CAMERA,
		.ua_max = 0,
	},
	{
		.id     = PWRSINK_SDCARD,
		.ua_max = 0,
	},
	{
		.id     = PWRSINK_VIDEO,
		.ua_max = 0,
	},
	{
		.id     = PWRSINK_WIFI,
		.ua_max = 200000,
	},
	{
		.id     = PWRSINK_SYSTEM_LOAD,
		.ua_max = 100000,
		.percent_util = 38,
	},
};

static int bahamas_pwrsink_resume_early(struct platform_device *pdev)
{
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 7);
	return 0;
}

static void bahamas_pwrsink_resume_late(struct early_suspend *h)
{
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 38);
}

static void bahamas_pwrsink_suspend_early(struct early_suspend *h)
{
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 7);
}

static int bahamas_pwrsink_suspend_late(struct platform_device *pdev, pm_message_t state)
{
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 1);
	return 0;
}

static struct pwr_sink_platform_data bahamas_pwrsink_data = {
	.num_sinks      = ARRAY_SIZE(bahamas_pwrsink_table),
	.sinks          = bahamas_pwrsink_table,
	.suspend_late	= bahamas_pwrsink_suspend_late,
	.resume_early	= bahamas_pwrsink_resume_early,
	.suspend_early	= bahamas_pwrsink_suspend_early,
	.resume_late	= bahamas_pwrsink_resume_late,
};

static struct platform_device bahamas_pwr_sink = {
	.name = "htc_pwrsink",
	.id = -1,
	.dev    = {
		.platform_data = &bahamas_pwrsink_data,
	},
};

static struct msm_pmem_setting pmem_setting_monodie = {
	.pmem_start = MSM_PMEM_MDP_BASE,
	.pmem_size = MSM_PMEM_MDP_SIZE,
	.pmem_adsp_start = MSM_PMEM_ADSP_BASE_MONODIE,
	.pmem_adsp_size = MSM_PMEM_ADSP_SIZE,
	.pmem_camera_start = MSM_PMEM_CAMERA_BASE_MONODIE,
	.pmem_camera_size = MSM_PMEM_CAMERA_SIZE,
	.ram_console_start = MSM_RAM_CONSOLE_BASE,
	.ram_console_size = MSM_RAM_CONSOLE_SIZE,
};

static struct msm_pmem_setting pmem_setting_dualdie = {
	.pmem_start = MSM_PMEM_MDP_BASE,
	.pmem_size = MSM_PMEM_MDP_SIZE,
	.pmem_adsp_start = MSM_PMEM_ADSP_BASE_DUALDIE,
	.pmem_adsp_size = MSM_PMEM_ADSP_SIZE,
	.pmem_camera_start = MSM_PMEM_CAMERA_BASE_DUALDIE,
	.pmem_camera_size = MSM_PMEM_CAMERA_SIZE,
	.ram_console_start = MSM_RAM_CONSOLE_BASE,
	.ram_console_size = MSM_RAM_CONSOLE_SIZE,
};
static struct tssc_ts_platform_data tssc_ts_device_data = {
	.swapped = 1,
	.x_reversed = 1,
	.y_reversed = 1,
	.x_min = 129,
	.x_max = 903,
	.y_min = 113,
	.y_max = 887,
};

static struct platform_device tssc_ts_device = {
	.name	= "tssc-manager",
	.id	= -1,
	.dev	= {
		.platform_data	= &tssc_ts_device_data,
	},
};

static int bahamas_reset_keys_up[] = {
	BTN_MOUSE,
	0
};

static struct keyreset_platform_data bahamas_reset_keys_pdata0 = {
	.keys_up = bahamas_reset_keys_up,
	.keys_down = {
		KEY_HOME,
		KEY_MENU,
		KEY_END,
		0
	},
};

static struct keyreset_platform_data bahamas_reset_keys_pdata1 = {
	.keys_up = bahamas_reset_keys_up,
	.keys_down = {
		KEY_SEND,
		KEY_MENU,
		KEY_END,
		0
	},
};

struct platform_device bahamas_reset_keys_device = {
	.name = KEYRESET_NAME,
	.dev.platform_data = &bahamas_reset_keys_pdata1,
};


static struct platform_device bahamas_rfkill = {
	.name = "bahamas_rfkill",
	.id = -1,
};

static struct platform_device *devices[] __initdata = {
	&msm_device_smd,
	&msm_device_nand,
	&msm_device_i2c,
	//&sd_door_switch,
	&trout_camera,
	&bahamas_h2w,
	&bahamas_audio_jack,
	&tssc_ts_device,
	&msm_camera_device,
	&bahamas_rfkill,
	&bahamas_reset_keys_device,
#ifdef CONFIG_HTC_PWRSINK
	&bahamas_pwr_sink,
#endif
};

extern struct sys_timer msm_timer;

static void __init bahamas_init_irq(void)
{
	printk("bahamas_init_irq()\n");
	msm_init_irq();
}

static uint cpld_iset;
static uint cpld_charger_en;
static uint cpld_usb_h2w_sw;
static uint opt_disable_uart3;
static char *keycaps = "";

module_param_named(iset, cpld_iset, uint, 0);
module_param_named(charger_en, cpld_charger_en, uint, 0);
module_param_named(usb_h2w_sw, cpld_usb_h2w_sw, uint, 0);
module_param_named(disable_uart3, opt_disable_uart3, uint, 0);
module_param_named(keycaps, keycaps, charp, 0);

extern int bahamas_bt_fastclock_power(int on);

static char bt_chip_id[10] = "brfxxxx";
module_param_string(bt_chip_id, bt_chip_id, sizeof(bt_chip_id), S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(bt_chip_id, "BT's chip id");

static char bt_fw_version[10] = "v2.0.38";
module_param_string(bt_fw_version, bt_fw_version, sizeof(bt_fw_version), S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(bt_fw_version, "BT's fw version");

static void bahamas_reset(void)
{
	gpio_set_value(BAHAMAS_GPIO_PS_HOLD, 0);
}

static uint32_t gpio_table[] = {
	/* BLUETOOTH */
	#ifdef CONFIG_SERIAL_MSM_HS		//allenou, uart hs test, 2008/11/18
	PCOM_GPIO_CFG(43, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* RTS */
	PCOM_GPIO_CFG(44, 2, GPIO_INPUT, GPIO_PULL_UP, GPIO_8MA), /* CTS */
	PCOM_GPIO_CFG(45, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* RX */
	PCOM_GPIO_CFG(46, 3, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* TX */
	#else
	PCOM_GPIO_CFG(43, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* BT_RTS */
	PCOM_GPIO_CFG(44, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_4MA), /* BT_CTS */
	PCOM_GPIO_CFG(45, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_4MA), /* BT_RX */
	PCOM_GPIO_CFG(46, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* BT_TX */
	#endif
};

static uint32_t camera_off_gpio_table[] = {
	/* CAMERA */
	PCOM_GPIO_CFG(2, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT2 */
	PCOM_GPIO_CFG(3, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT3 */
	PCOM_GPIO_CFG(4, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT4 */
	PCOM_GPIO_CFG(5, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT5 */
	PCOM_GPIO_CFG(6, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT6 */
	PCOM_GPIO_CFG(7, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT7 */
	PCOM_GPIO_CFG(8, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT8 */
	PCOM_GPIO_CFG(9, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT9 */
	PCOM_GPIO_CFG(10, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT10 */
	PCOM_GPIO_CFG(11, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT11 */
	PCOM_GPIO_CFG(12, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* PCLK */
	PCOM_GPIO_CFG(13, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* HSYNC_IN */
	PCOM_GPIO_CFG(14, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* VSYNC_IN */
	PCOM_GPIO_CFG(15, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* MCLK */
};

static uint32_t camera_on_gpio_table[] = {
	/* CAMERA */
	PCOM_GPIO_CFG(2, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT2 */
	PCOM_GPIO_CFG(3, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT3 */
	PCOM_GPIO_CFG(4, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT4 */
	PCOM_GPIO_CFG(5, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT5 */
	PCOM_GPIO_CFG(6, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT6 */
	PCOM_GPIO_CFG(7, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT7 */
	PCOM_GPIO_CFG(8, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT8 */
	PCOM_GPIO_CFG(9, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT9 */
	PCOM_GPIO_CFG(10, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT10 */
	PCOM_GPIO_CFG(11, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT11 */
	PCOM_GPIO_CFG(12, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_16MA), /* PCLK */
	PCOM_GPIO_CFG(13, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* HSYNC_IN */
	PCOM_GPIO_CFG(14, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* VSYNC_IN */
	PCOM_GPIO_CFG(15, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA), /* MCLK */
};

static void config_gpio_table(uint32_t *table, int len)
{
	int n;
	unsigned id;
	for(n = 0; n < len; n++) {
		id = table[n];
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
	}
}

void config_bahamas_camera_on_gpios(void)
{
	config_gpio_table(camera_on_gpio_table,
		ARRAY_SIZE(camera_on_gpio_table));
}

void config_bahamas_camera_off_gpios(void)
{
	config_gpio_table(camera_off_gpio_table,
		ARRAY_SIZE(camera_off_gpio_table));
}

static void __init config_gpios(void)
{
	config_gpio_table(gpio_table, ARRAY_SIZE(gpio_table));
	config_bahamas_camera_off_gpios();
}

static struct msm_acpu_clock_platform_data bahamas_clock_data = {
	.acpu_switch_time_us = 50,
	.max_speed_delta_khz = 256000,
	.vdd_switch_time_us = 62,
	.power_collapse_khz = 19200000,
#if defined(CONFIG_TURBO_MODE)
	.wait_for_irq_khz = 176000000,
#else
	.wait_for_irq_khz = 128000000,
#endif
};

static unsigned bahamas_perf_acpu_table[] = {
	245760000,
	480000000,
	528000000,
};

static struct perflock_platform_data bahamas_perflock_data = {
	.perf_acpu_table = bahamas_perf_acpu_table,
	.table_size = ARRAY_SIZE(bahamas_perf_acpu_table),
};

void msm_serial_debug_init(unsigned int base, int irq,
			   struct device *clk_device, int signal_irq);

#ifdef CONFIG_SERIAL_MSM_HS
static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
	.wakeup_irq = MSM_GPIO_TO_INT(45),
	.inject_rx_on_wakeup = 1,
	.rx_to_inject = 0x32,
	.cpu_lock_supported = 1,
};
#endif

/*Farmer:For H2W power*/
static struct vreg *vreg_h2w;
static int h2w_power_configure(struct gpio_chip *chip,
			       unsigned int gpio,
			       unsigned long flags)
{
	if ((flags & GPIOF_DRIVE_OUTPUT) && !vreg_h2w)
		vreg_h2w = vreg_get(0, BAHAMAS_H2W_POWER_NAME);

	if ((flags & GPIOF_OUTPUT_HIGH) && vreg_h2w) {
		vreg_enable(vreg_h2w);
		vreg_set_level(vreg_h2w, 3000);
	} else if ((flags & GPIOF_OUTPUT_LOW) && vreg_h2w)
		vreg_disable(vreg_h2w);

	return 0;
}

static int h2w_power_get_irq_num(struct gpio_chip *chip,
				 unsigned int gpio,
				 unsigned int *irqp,
				 unsigned long *irqnumflagsp)
{
	return -1;
}

static int h2w_power_read(struct gpio_chip *chip, unsigned n)
{
	return -1;
}
static int h2w_power_write(struct gpio_chip *chip, unsigned n, unsigned on)
{
	if (!vreg_h2w)
		return -1;

	if (on) {
		vreg_enable(vreg_h2w);
		vreg_set_level(vreg_h2w, 3000);
	} else
		vreg_disable(vreg_h2w);
	return 0;
}

static struct gpio_chip bahamas_h2w_gpio_chip = {
	.start = BAHAMAS_GPIO_H2W_POWER,
	.end = BAHAMAS_GPIO_H2W_POWER,
	.configure = h2w_power_configure,
	.get_irq_num = h2w_power_get_irq_num,
	.read = h2w_power_read,
	.write = h2w_power_write,
};

void bahamas_init_h2w_power_gpio(void)
{
	register_gpio_chip(&bahamas_h2w_gpio_chip);
}

static void __init bahamas_init(void)
{
	int rc;
	printk("bahamas_init() revision=%d\n", system_rev);

	/*
	 * Setup common MSM GPIOS
	 */
	config_gpios();

	msm_hw_reset_hook = bahamas_reset;

	msm_acpu_clock_init(&bahamas_clock_data);
	perflock_init(&bahamas_perflock_data);
	/* adjust GPIOs based on bootloader request */
	/* XXX: on Memphis,
	 *      GPIO#86 is H2W DAT / UART RX for HTC 11-Pin
	 *      GPIO#87 is H2W CLK / UART TX for HTC 11-Pin
	 *      We would need to use UART3 as debug port
	 *
	 * TODO: switch UART3 and H2W (for headset detect)
	 *       based on bootloader request.
	 */

#if defined(CONFIG_MSM_SERIAL_DEBUGGER)
	if (!opt_disable_uart3)
		msm_serial_debug_init(MSM_UART3_PHYS, INT_UART3,
				      &msm_device_uart3.dev, 1);
#endif

	/* touchscreen must be powered before we enable i2c pullup */
#if 0 /* TODO: do some equivalent operations here */
	udelay(50);
	trout_gpio_write(NULL, TROUT_GPIO_I2C_PULL, 1);
#endif

	/* put the AF VCM in powerdown mode to avoid noise */
#if 0
	trout_gpio_write(NULL, TROUT_GPIO_VCM_PWDN, 1);
	mdelay(100);
	trout_i2c_sysdev_resume(NULL);
#endif

#if 0 /* TODO: do some equivalent operations here */
	if(sysdev_class_register(&trout_i2c_sysdev_class) == 0)
		sysdev_register(&trout_i2c_sys_device);
#endif

	#ifdef CONFIG_SERIAL_MSM_HS
	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;
	msm_add_serial_devices(3);
	#else
	msm_add_serial_devices(0);
	#endif

	msm_add_serial_devices(2);

	msm_add_usb_devices(NULL,NULL);

	//msm_change_usb_id(0x0bb4, 0x0c06);
	if (board_mcp_monodie())
		msm_add_mem_devices(&pmem_setting_monodie);
	else
		msm_add_mem_devices(&pmem_setting_dualdie);
	msm_init_pmic_vibrator();

	bahamas_init_h2w_power_gpio();
#if 1
	rc = bahamas_init_mmc(system_rev);
	if (rc)
		printk(KERN_CRIT "%s: MMC init failure (%d)\n", __func__, rc);
#endif
	if(!system_rev)
		bahamas_reset_keys_device.dev.platform_data = &bahamas_reset_keys_pdata0;
	/*CC090319*/
	if  (bahamas_is_3M_camera())   {
		msm_camera_device_data.sinfo = &msm_camera_sensor_mt9t013;
		}
	else  {
		msm_camera_device_data.sinfo = &msm_camera_sensor_s5k4b2fx;
	}

	if(system_rev < 3) {
		if (panel_detect() == PANEL_WINTEK) {
			microp_data.num_pins   = ARRAY_SIZE(microp_pins_0_wint);
			microp_data.pin_config = microp_pins_0_wint;
		} else {
			microp_data.num_pins   = ARRAY_SIZE(microp_pins_0);
			microp_data.pin_config = microp_pins_0;
		}
		i2c_microp_devices.irq = 0;
	} else if (panel_detect() == PANEL_WINTEK) {
		microp_data.num_pins   = ARRAY_SIZE(microp_pins_1_wint);
		microp_data.pin_config = microp_pins_1_wint;
	}

/*~CC090319*/
	platform_add_devices(devices, ARRAY_SIZE(devices));

	/* Read Config 8 200 (Full Speed USB Mode) */
	if (readl(MSM_SHARED_RAM_BASE + 0xFC054) & 0x200)
		bahamas_h2w_data.flags |= FULL_SPEED_USB;

	if (system_rev >= 3)
		bahamas_h2w_data.flags |= _35MM_MIC_DET_L2H;

	if (system_rev >= 5)
		i2c_register_board_info(0, i2c_sensor, ARRAY_SIZE(i2c_sensor));

	i2c_register_board_info(0, i2c_devices, ARRAY_SIZE(i2c_devices));
	i2c_register_board_info(0 ,&i2c_microp_devices, 1);

	/* SD card door should wake the device */
	//trout_gpio_irq_set_wake(TROUT_GPIO_TO_INT(TROUT_GPIO_SD_DOOR_N), 1);
}
/*CC090319*/
unsigned int bahamas_get_hwid(void)
{
	printk("bahamas_get_hwid=0x%x\r\n", hwid);
	return hwid;
}

unsigned int bahamas_get_skuid(void)
{
	printk("bahamas_get_skuid=0x%x\r\n", skuid);
	return skuid;
}

unsigned bahamas_engineerid(void)
{
	printk("bahamas_engineerid=0x%x\r\n", engineerid);
	return engineerid;
}

int bahamas_is_3M_camera(void)
{
	int ret  = 0;

	printk("bahamas_is_3M_camera, PCBID=0x%x\r\n", system_rev);

	if (system_rev > 1)
		ret  = 1;    /*CC09031, system_rev==PCBID */

	return ret;
}
/*~CC090319*/

static void __init bahamas_fixup(struct machine_desc *desc, struct tag *tags,
                               char **cmdline, struct meminfo *mi)
{
	/*CC090319*/

	hwid = 0;
	skuid = 0;
	engineerid = (0x01 << 1);

	hwid = parse_tag_hwid((const struct tag *)tags);
	printk("bahamas_fixup:hwid=0x%x\n", hwid);
	skuid = parse_tag_skuid((const struct tag *)tags);
	printk("bahamas_fixup:skuid=0x%x\n", skuid);
	engineerid = parse_tag_engineerid((const struct tag *)tags);
	printk("bahamas_fixup:engineerid=0x%x\n", engineerid);
	parse_tag_monodie((const struct tag *)tags);
	/*~CC090319*/

	if (board_mcp_monodie()) {
		mi->nr_banks=1;
		mi->bank[0].start = MSM_LINUX_BASE1;
		mi->bank[0].node = PHYS_TO_NID(MSM_LINUX_BASE1);
		mi->bank[0].size = MSM_LINUX_SIZE1+MSM_LINUX_SIZE2;
	}
	else {
		mi->nr_banks=2;
		mi->bank[0].start = MSM_LINUX_BASE1;
		mi->bank[0].node = PHYS_TO_NID(MSM_LINUX_BASE1);
		mi->bank[0].size = MSM_LINUX_SIZE1;
		mi->bank[1].start = MSM_LINUX_BASE2_DUALDIE;
		mi->bank[1].node = PHYS_TO_NID(MSM_LINUX_BASE2_DUALDIE);
		mi->bank[1].size = MSM_LINUX_SIZE2;
	}
}

static void __init bahamas_map_io(void)
{
	printk("bahamas_init_map_io()\n");
	msm_map_common_io();
	msm_clock_init();
}

MACHINE_START(BAHAMAS, "bahamas")
/* Maintainer: Tony Liu <Tony_Liu@htc.com> */
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io        = MSM_DEBUG_UART_PHYS,
	.io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params    = 0x02E00100,
	.fixup          = bahamas_fixup,
	.map_io         = bahamas_map_io,
	.init_irq       = bahamas_init_irq,
	.init_machine   = bahamas_init,
	.timer          = &msm_timer,
MACHINE_END
