/*****************************************************************************
* Filename: tc358765xbg_dsi2lvds.c
* Author: Innocomm
****************************************************************************/

#include <linux/types.h>
#include <linux/string.h>
#include <linux/i2c.h>
#include <linux/gpio.h>

#include "ddp_dsi.h"
#include "ddp_hal.h"
#include "lcm_drv.h"
#include "tc358765xbg_dsi2lvds.h"

/* --------------------------------------------------------------------------- */
/* Local Constants */
/* --------------------------------------------------------------------------- */

/* Debug Flag */
//#define TC358765XBG_DEBUG

#define FRAME_WIDTH		(1920)
#define FRAME_HEIGHT	(1080)

/* --------------------------------------------------------------------------- */
/* GPIO settings */
/* --------------------------------------------------------------------------- */

#define LCM_LOG_DEBUG(fmt, ...)		printk(KERN_DEBUG "[Kernel/LCM]" LCM_DRIVER_NAME fmt, ##__VA_ARGS__)
#define LCM_LOG_INFO(fmt, ...)		printk(KERN_INFO "[Kernel/LCM]" LCM_DRIVER_NAME fmt, ##__VA_ARGS__)
#define LCM_LOG_ERR(fmt, ...)		printk(KERN_ERROR "[Kernel/LCM]" LCM_DRIVER_NAME fmt, ##__VA_ARGS__)

/* --------------------------------------------------------------------------- */
/* Local Variables */
/* --------------------------------------------------------------------------- */
static LCM_UTIL_FUNCS lcm_util = {
	.set_reset_pin	= NULL,
	.udelay 		= NULL,
	.mdelay 		= NULL,
};

//#define SET_RESET_PIN(v)			(mt_set_reset_pin((v)))
#define SET_RESET_PIN(v)			(lcm_util.set_reset_pin((v)))
#define UDELAY(n)					(lcm_util.udelay(n))
#define MDELAY(n)					(lcm_util.mdelay(n))
//#define REGFLAG_DELAY				0xAB

#ifndef kal_uint8
typedef unsigned char kal_uint8;
#endif

#ifndef kal_uint16
typedef unsigned short kal_uint16;
#endif

#ifndef kal_uint32
typedef unsigned int kal_uint32;
#endif

typedef struct {
	const kal_uint8 data[6];
} tc358765xbg_setting_table;

/* --------------------------------------------------------------------------- */
/* Local Functions */
/* --------------------------------------------------------------------------- */
extern void DSI_clk_HS_mode(DISP_MODULE_ENUM module, void *cmdq, bool enter);
struct i2c_client *tc358765xbg_i2c_client;
static DEFINE_MUTEX(tc358765xbg_i2c_access);

/**********************************************************
 *
 *   [I2C Function For Read/Write tc358765xbg]
 *
 *********************************************************/
kal_uint32 tc358765xbg_write_byte(const kal_uint8* value, int len)
{
	kal_uint32 ret = 0;
	int i = 0;
	int retry_count = 5;

	for (i = 1; i <= retry_count; i++) {
		mutex_lock(&tc358765xbg_i2c_access);
		//tc358765xbg_i2c_client->ext_flag = ((tc358765xbg_i2c_client->ext_flag) & I2C_MASK_FLAG) | I2C_DIRECTION_FLAG;
		ret = i2c_master_send(tc358765xbg_i2c_client, value, len);
		//tc358765xbg_i2c_client->ext_flag = 0;
		mutex_unlock(&tc358765xbg_i2c_access);

#ifdef TC358765XBG_DEBUG
		LCM_LOG_INFO("%s write [%#X,%#X,%#X,%#X,%#X,%#X], ret=[%d]\n", __func__, value[0], value[1], value[2], value[3], value[4], value[5], ret);
#endif // TC358765XBG_DEBUG

		if (ret < 0) {
#ifdef TC358765XBG_DEBUG
			LCM_LOG_INFO("%s write [%#X,%#X] failed retry count=[%d]\n", __func__, value[0], value[1], i);
#endif // TC358765XBG_DEBUG
		} else {
			/* Data is transfered successfully */
			ret = 0;
			break;
		}
	}

	return ret;
}

kal_uint32 tc358765xbg_read_byte(const kal_uint8 addr, kal_uint8 *dataBuffer)
{
	kal_uint32 ret = 0;
	kal_uint16 len;

	dataBuffer[0] = addr;

	len = 1;

	mutex_lock(&tc358765xbg_i2c_access);
	//tc358765xbg_i2c_client->ext_flag = ((tc358765xbg_i2c_client->ext_flag) & I2C_MASK_FLAG) | I2C_WR_FLAG | I2C_DIRECTION_FLAG;
	ret = i2c_master_send(tc358765xbg_i2c_client, &addr, len);

	if (ret < 0) {
		goto exit;
	}

	ret = i2c_master_recv(tc358765xbg_i2c_client, dataBuffer, len);

	if (ret == len) {
		ret = 0;
	}
exit:
	//tc358765xbg_i2c_client->ext_flag = 0;
	mutex_unlock(&tc358765xbg_i2c_access);

	return ret;
}

kal_uint32 tc358765xbg_read_reg(const kal_uint8* addr, kal_uint16 addr_size, kal_uint8 *dataBuffer, kal_uint16 dataBuffer_size)
{
	kal_uint32 ret = 0;

	if ((dataBuffer_size != 4) && (addr_size != 2)) {
		return -22;
	}

	dataBuffer[0] = addr[0];
	dataBuffer[1] = addr[1];

#ifdef TC358765XBG_DEBUG
	LCM_LOG_INFO("%s cmd = [%#X,%#X]\n", __func__, dataBuffer[0], dataBuffer[1]);
#endif // TC358765XBG_DEBUG

	mutex_lock(&tc358765xbg_i2c_access);
	//tc358765xbg_i2c_client->ext_flag = ((tc358765xbg_i2c_client->ext_flag) & I2C_MASK_FLAG) | I2C_WR_FLAG | I2C_DIRECTION_FLAG;
	ret = i2c_master_send(tc358765xbg_i2c_client, dataBuffer, addr_size);

	if (ret < 0) {
		goto exit;
	}

	ret = i2c_master_recv(tc358765xbg_i2c_client, dataBuffer, dataBuffer_size);

	if (ret == dataBuffer_size) {
		ret = 0;
	}
exit:
	//tc358765xbg_i2c_client->ext_flag = 0;
	mutex_unlock(&tc358765xbg_i2c_access);

#ifdef TC358765XBG_DEBUG
	LCM_LOG_INFO("%s read data = [%d][%#X,%#X,%#X,%#X]\n", __func__, ret, dataBuffer[0], dataBuffer[1], dataBuffer[2],dataBuffer[3]);
#endif // TC358765XBG_DEBUG

	return ret;
}

/* tc358765xbg chip init table */
static const tc358765xbg_setting_table tc358765xbg_init_table[] = {
//**************************************************
// TC358764/65XBG DSI Basic Parameters.  Following 10 setting should be pefromed in LP mode
//**************************************************
	{{0x01, 0x3C, 0x0E, 0x00, 0x0B, 0x00}},
	{{0x01, 0x14, 0x09, 0x00, 0x00, 0x00}},
	{{0x01, 0x64, 0x0B, 0x00, 0x00, 0x00}},
	{{0x01, 0x68, 0x0B, 0x00, 0x00, 0x00}},
	{{0x01, 0x6C, 0x0B, 0x00, 0x00, 0x00}},
	{{0x01, 0x70, 0x0B, 0x00, 0x00, 0x00}},
	{{0x01, 0x34, 0x1F, 0x00, 0x00, 0x00}},
	{{0x02, 0x10, 0x1F, 0x00, 0x00, 0x00}},
	{{0x01, 0x04, 0x01, 0x00, 0x00, 0x00}},
	{{0x02, 0x04, 0x01, 0x00, 0x00, 0x00}},
//**************************************************
// TC358764/65XBG Timing and mode setting
//**************************************************
	{{0x04, 0x50, 0x20, 0x01, 0x20, 0x00}},
	{{0x04, 0x54, 0x06, 0x00, 0x72, 0x00}},
	{{0x04, 0x58, 0x80, 0x07, 0x3C, 0x00}},
	{{0x04, 0x5C, 0x06, 0x00, 0x12, 0x00}},
	{{0x04, 0x60, 0x38, 0x04, 0x11, 0x00}},
	{{0x04, 0x64, 0x01, 0x00, 0x00, 0x00}},
	{{0x04, 0xA0, 0x06, 0x80, 0x44, 0x00}},
//**************************************************
// Wait more than 100us
//**************************************************
	{{0x04, 0xA0, 0x06, 0x80, 0x04, 0x00}},
	{{0x05, 0x04, 0x04, 0x00, 0x00, 0x00}},
//**************************************************
// TC358764/65XBG LVDS Color mapping setting
//**************************************************
	{{0x04, 0x80, 0x00, 0x01, 0x02, 0x03}},
	{{0x04, 0x84, 0x04, 0x07, 0x05, 0x08}},
	{{0x04, 0x88, 0x09, 0x0A, 0x0E, 0x0F}},
	{{0x04, 0x8C, 0x0B, 0x0C, 0x0D, 0x10}},
	{{0x04, 0x90, 0x16, 0x17, 0x11, 0x12}},
	{{0x04, 0x94, 0x13, 0x14, 0x15, 0x1B}},
	{{0x04, 0x98, 0x18, 0x19, 0x1A, 0x06}},
//**************************************************
// TC358764/65XBG LVDS enable
//**************************************************
	{{0x04, 0x9C, 0x33, 0x04, 0x00, 0x00}}
};

static void push_reg_table(const tc358765xbg_setting_table *table, unsigned int count)
{
	unsigned int i;

	LCM_LOG_INFO("%s enter\n", __func__);

	for (i = 0; i < count; i++) {
		tc358765xbg_write_byte(table[i].data, ARRAY_SIZE(table[i].data));

		if (table[i].data[0] == 0x04 && table[i].data[1] == 0xA0 && table[i].data[4] == 0x44) {
			LCM_LOG_INFO("PHY reset at least delay 150us\n");
			MDELAY(2);
		}
	}
}

#ifdef TC358765XBG_DEBUG
void dump_reg_table(tc358765xbg_setting_table *table, unsigned int count)
{
	kal_uint32 ret = 0;
	unsigned int i;
	unsigned char data[4];

	LCM_LOG_INFO("%s enter\n", __func__);

	for (i = 0; i < count; i++) {
		ret_code = tc358765xbg_read_reg(table[i].data, 2, data, sizeof(data));
		LCM_LOG_INFO("%s ret_code =[%d]\n", __func__, ret);
	}
}

void tc358765xbg_check_dsi_basic_reg(void)
{
	dump_reg_table(tc358765xbg_init_table, ARRAY_SIZE(tc358765xbg_init_table));
}
#endif // TC358765XBG_DEBUG

void init_tc358765xbg(void)
{
	LCM_LOG_INFO("%s enter\n", __func__);

	push_reg_table(tc358765xbg_init_table, ARRAY_SIZE(tc358765xbg_init_table));

#ifdef TC358765XBG_DEBUG
	MDELAY(10);

	dump_reg_table(tc358765xbg_init_table, ARRAY_SIZE(tc358765xbg_init_table));
#endif // TC358765XBG_DEBUG
}

/* --------------------------------------------------------------------------- */
/* LCM Driver Implementations */
/* --------------------------------------------------------------------------- */
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
	LCM_LOG_INFO("%s enter\n", __func__);

	memset(params, 0, sizeof(LCM_PARAMS));

	params->type = LCM_TYPE_DSI;
	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	params->dsi.mode = SYNC_EVENT_VDO_MODE;

	/* DSI settings */
	params->dsi.LANE_NUM = LCM_FOUR_LANE;

	/* The following defined the fomat for data coming from LCD engine. */
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;
	params->dsi.packet_size = 256;
	params->lcm_if = LCM_INTERFACE_DSI0;
	params->lcm_cmd_if = LCM_INTERFACE_DSI0;

	params->dsi.word_count = FRAME_WIDTH * 3;

	params->dsi.vertical_sync_active	= 6;
	params->dsi.vertical_backporch		= 18;
	params->dsi.vertical_frontporch		= 17;
	params->dsi.vertical_active_line	= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active	= 5;
	params->dsi.horizontal_backporch	= 133;
	params->dsi.horizontal_frontporch	= 62;
	params->dsi.horizontal_active_pixel	= FRAME_WIDTH;

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.pll_select = 0;	/* 0: MIPI_PLL; 1: LVDS_PLL */
	// (Height +VSA + VBP + VFP)*(Width +HSA + HBP +HFP)*total_bit_per_pixel*frame_per_second/total_lane_num/2
	params->dsi.PLL_CLOCK = 450; // calc 427
	params->dsi.cont_clock = 1;

	// params->dsi.LPX = 22;
	// params->dsi.CLK_ZERO = 106;
	// params->dsi.CLK_HS_PRPR = 26;
	// params->dsi.CLK_TRAIL = 26;
	// params->dsi.HS_ZERO = 42;
	// params->dsi.HS_PRPR = 30;
	// params->dsi.CLK_HS_EXIT = 58; // ??
	// params->dsi.HS_TRAIL = 45;
}

static void lcm_set_gpio_output(unsigned int GPIO, unsigned int output)
{
	if (gpio_is_valid(GPIO)) {
		gpio_set_value(GPIO, (output > 0) ? 1 : 0);
	}
}

static void lcm_suspend(void)
{
	LCM_LOG_INFO("%s enter\n", __func__);

	/* VGP3_PMU: LVDS_26MHZ */
	lcm_vgp3_supply_disable();
	MDELAY(20);

	/* VGP5_PMU: LVDS power */
	lcm_vgp5_supply_disable();
	MDELAY(20);

	/* GPIO127: LCM_RST for panel */
	lcm_set_gpio_output(gpio_lcd_rst_en, 0);

	lcm_set_gpio_output(gpio_lcd_12v_en, 0);
}

static void lcm_resume(void)
{
	LCM_LOG_INFO("%s enter\n", __func__);

	lcm_set_gpio_output(gpio_lcd_12v_en, 1);

	/* VGP5_PMU: LVDS power */
	lcm_vgp5_supply_enable();
	MDELAY(30);

	/* VGP3_PMU: LVDS_26MHZ */
	lcm_vgp3_supply_enable();
	MDELAY(30);

	/* GPIO127: LCM_RST for panel */
	lcm_set_gpio_output(gpio_lcd_rst_en, 1);
	MDELAY(10);

	/* DSI clock enter High Speed mode */
	DSI_clk_HS_mode(DISP_MODULE_DSI0, NULL, 1);

	/* Bridge IC register config via I2C command */
	init_tc358765xbg();
	MDELAY(50);
}

static unsigned int lcm_compare_id(void)
{
	LCM_LOG_INFO("%s enter\n", __func__);

	return 1;
}

LCM_DRIVER tc358765xbg_dsi2lvds_lcm_drv = {
	.name			= LCM_DRIVER_NAME,
	.set_util_funcs	= lcm_set_util_funcs,
	.get_params		= lcm_get_params,
	.suspend		= lcm_suspend,
	.resume			= lcm_resume,
	.compare_id		= lcm_compare_id,
};
