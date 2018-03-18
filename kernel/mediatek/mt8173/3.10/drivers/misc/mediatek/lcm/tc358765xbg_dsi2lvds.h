/*****************************************************************************
* Filename: tc358765xbg_dsi2lvds.h
* Author: Innocomm
****************************************************************************/
#ifndef _TC358765XBG_DSI2_LVDS_H_
#define _TC358765XBG_DSI2_LVDS_H_

extern int gpio_lcd_rst_en;
extern int gpio_lcd_12v_en;
extern struct i2c_client *tc358765xbg_i2c_client;
int lcm_vgp3_supply_enable(void);
int lcm_vgp3_supply_disable(void);
int lcm_vgp5_supply_enable(void);
int lcm_vgp5_supply_disable(void);

#define LCM_DRIVER_NAME		"tc358765xbg_dsi2lvds"

#endif
