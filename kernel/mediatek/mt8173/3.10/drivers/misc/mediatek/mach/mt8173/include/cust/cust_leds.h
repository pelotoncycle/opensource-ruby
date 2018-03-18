#ifndef _CUST_LEDS_H
#define _CUST_LEDS_H

#include <mach/mt_typedefs.h>
enum mt65xx_led_type {
	MT65XX_LED_TYPE_RED = 0,
	MT65XX_LED_TYPE_GREEN,
	MT65XX_LED_TYPE_BLUE,
	MT65XX_LED_TYPE_JOGBALL,
	MT65XX_LED_TYPE_KEYBOARD,
	MT65XX_LED_TYPE_BUTTON,
	MT65XX_LED_TYPE_LCD,
	MT65XX_LED_TYPE_TOTAL,
};

extern int disp_bls_set_backlight(int level);
//#ifndef CONFIG_TB8173_EVB
//#define BACKLIGHT_SUPPORT_LP8557
//#endif

#endif
