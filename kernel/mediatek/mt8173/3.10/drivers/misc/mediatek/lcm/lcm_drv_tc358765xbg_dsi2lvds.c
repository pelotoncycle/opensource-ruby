#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/earlysuspend.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#endif

#include "tc358765xbg_dsi2lvds.h"

#define I2C_DRIVER_NAME	"tc358765xbg"

static struct device *lcm_dev;
static struct regulator *lcm_vgp3;
static struct regulator *lcm_vgp5;
static unsigned int volt_vgp3;
static unsigned int volt_vgp5;
int gpio_lcd_rst_en;
int gpio_lcd_12v_en;
static struct pinctrl *disp_pwm0_pinctrl;
static struct pinctrl_state *pwm0_mode, *gpio_mode_out_low, *gpio_mode_out_high;
static int gpio_lcd_bl_en;
static int gpio_lcd_bl_en_level;

/* get(vgp3) LDO supply */
static int lcm_get_vgp3_supply(struct device *dev)
{
	int ret = 0;

	dev_dbg(dev, "LCM: lcm_get_vgp3_supply is going\n");

	lcm_vgp3 = devm_regulator_get(dev, "reg-lcm-5v");

	if (IS_ERR(lcm_vgp3)) {
		ret = PTR_ERR(lcm_vgp3);
		lcm_vgp3 = NULL;
		dev_err(dev, "failed to get reg-lcm-5v LDO, %d\n", ret);
		goto exit;
	}

	pr_debug("LCM: lcm get vgp3 supply ok.\n");

	/* get current voltage settings */
	volt_vgp3 = regulator_get_voltage(lcm_vgp3);
	dev_info(dev, "lcm VGP3 current voltage = %d\n", volt_vgp3);

exit:
	return ret;
}

int lcm_vgp3_supply_enable(void)
{
	int ret = 0;
	unsigned int volt = 0;

	dev_dbg(lcm_dev, "LCM: lcm_vgp3_supply_enable\n");

	if (NULL != lcm_vgp3) {
		dev_dbg(lcm_dev, "LCM: set regulator voltage lcm_vgp3 voltage to %duV\n", volt_vgp3);
		/* set(vgp3) voltage to 2.8V */
		ret = regulator_set_voltage(lcm_vgp3, volt_vgp3, volt_vgp3);
		if (ret != 0) {
			pr_err("LCM: lcm failed to set lcm_vgp3 voltage: %d\n", ret);
			goto exit;
		}

		/* get(vgp3) voltage settings again */
		volt = regulator_get_voltage(lcm_vgp3);
		if (volt == volt_vgp3)
			dev_err(lcm_dev, "LCM: check regulator voltage=%uV pass!\n", volt_vgp3);
		else
			dev_err(lcm_dev, "LCM: check regulator voltage=%uV fail! (voltage: %d)\n", volt_vgp3, volt);

		ret = regulator_enable(lcm_vgp3);

		if (ret != 0) {
			dev_err(lcm_dev, "LCM: Failed to enable lcm_vgp3: %d\n", ret);
		}
	}

exit:
	return ret;
}

int lcm_vgp3_supply_disable(void)
{
	int ret = 0;
	unsigned int isenable;

	if (NULL != lcm_vgp3) {
		/* disable regulator */
		isenable = regulator_is_enabled(lcm_vgp3);

		dev_dbg(lcm_dev, "LCM: lcm_vgp3 query regulator enable status[0x%d]\n", isenable);

		if (isenable) {
			ret = regulator_disable(lcm_vgp3);

			if (ret != 0) {
				dev_err(lcm_dev, "LCM: lcm failed to disable lcm_vgp3: %d\n", ret);
				goto exit;
			}

			/* verify */
			isenable = regulator_is_enabled(lcm_vgp3);

			if (!isenable)
				dev_err(lcm_dev, "LCM: lcm_vgp3 regulator disable pass\n");
		}
	}

exit:
	return ret;
}

/* get(vgp5) LDO supply */
static int lcm_get_vgp5_supply(struct device *dev)
{
	int ret = 0;

	dev_dbg(dev, "LCM: lcm_get_vgp5_supply is going\n");

	lcm_vgp5 = devm_regulator_get(dev, "reg-lvds-26mhz");

	if (IS_ERR(lcm_vgp5)) {
		ret = PTR_ERR(lcm_vgp5);
		lcm_vgp5 = NULL;
		dev_err(dev, "failed to get reg-lvds-26mhz LDO, %d\n", ret);
		goto exit;
	}

	dev_dbg(dev, "LCM: lcm get vgp5 supply ok.\n");

	/* get current voltage settings */
	volt_vgp5 = regulator_get_voltage(lcm_vgp5);
	dev_info(dev, "lcm VGP5 current voltage = %d\n", volt_vgp5);

exit:
	return ret;
}

int lcm_vgp5_supply_enable(void)
{
	int ret = 0;
	unsigned int volt = 0;

	dev_dbg(lcm_dev, "LCM: lcm_vgp5_supply_enable\n");

	if (NULL != lcm_vgp5) {
		dev_dbg(lcm_dev, "LCM: set regulator voltage lcm_vgp5 voltage to %duV\n", volt_vgp5);
		/* set(vgp3) voltage to 2.8V */
		ret = regulator_set_voltage(lcm_vgp5, volt_vgp5, volt_vgp5);

		if (ret != 0) {
			pr_err("LCM: lcm failed to set lcm_vgp5 voltage: %d\n", ret);
			goto exit;
		}

		/* get(vgp5) voltage settings again */
		volt = regulator_get_voltage(lcm_vgp5);

		if (volt == volt_vgp5)
			dev_err(lcm_dev, "LCM: check regulator voltage=%uV pass!\n", volt_vgp5);
		else
			dev_err(lcm_dev, "LCM: check regulator voltage=%uV fail! (voltage: %d)\n", volt_vgp5, volt);

		ret = regulator_enable(lcm_vgp5);

		if (ret != 0) {
			dev_err(lcm_dev, "LCM: Failed to enable lcm_vgp5: %d\n", ret);
		}
	}

exit:
	return ret;
}

int lcm_vgp5_supply_disable(void)
{
	int ret = 0;
	unsigned int isenable;

	if (NULL != lcm_vgp5) {
		/* disable regulator */
		isenable = regulator_is_enabled(lcm_vgp5);

		dev_dbg(lcm_dev, "LCM: lcm_vgp5 query regulator enable status[0x%d]\n", isenable);

		if (isenable) {
			ret = regulator_disable(lcm_vgp5);

			if (ret != 0) {
				dev_err(lcm_dev, "LCM: lcm failed to disable lcm_vgp5: %d\n", ret);
				goto exit;
			}

			/* verify */
			isenable = regulator_is_enabled(lcm_vgp5);

			if (!isenable)
				dev_err(lcm_dev, "LCM: lcm_vgp5 regulator disable pass\n");
		}
	}

exit:
	return ret;
}

void lcm_request_gpio_control(struct device *dev)
{
	int ret = 0;
	enum of_gpio_flags flags;

	gpio_lcd_rst_en = of_get_named_gpio(dev->of_node, "gpio_lcm_rst_en", 0);

	if (!gpio_is_valid(gpio_lcd_rst_en)) {
		gpio_lcd_rst_en = -ENODEV;
	} else {
		dev_dbg(dev, "gpio_lcd_rst_en = 0x%x\n", gpio_lcd_rst_en);

		ret = devm_gpio_request(dev, gpio_lcd_rst_en, "GPIO_LCD_RST_EN");

		if (ret < 0) {
			dev_err(dev, "request gpio_lcd_rst_en = 0x%x failed, ret=%d\n", gpio_lcd_rst_en, ret);
			gpio_lcd_rst_en = -ENODEV;
		}
	}

	gpio_lcd_12v_en = of_get_named_gpio(dev->of_node, "gpio_lcm_12v_en", 0);

	if (!gpio_is_valid(gpio_lcd_12v_en)) {
		gpio_lcd_12v_en = -ENODEV;
	} else {
		dev_dbg(dev, "gpio_lcd_12v_en = 0x%x\n", gpio_lcd_12v_en);

		ret = devm_gpio_request(dev, gpio_lcd_12v_en, "GPIO_LCD_12V_EN");

		if (ret < 0) {
			dev_err(dev, "request gpio_lcd_12v_en = 0x%x failed, ret=%d\n", gpio_lcd_12v_en, ret);
			gpio_lcd_12v_en = -ENODEV;
		}
	}

	gpio_lcd_bl_en_level = 1;
	gpio_lcd_bl_en = of_get_named_gpio_flags(dev->of_node, "gpio_lcm_bl_en", 0, &flags);

	if (!gpio_is_valid(gpio_lcd_bl_en)) {
		gpio_lcd_bl_en = -ENODEV;
	} else {
		if (flags & OF_GPIO_ACTIVE_LOW) {
			gpio_lcd_bl_en_level = 0;
		}

		dev_dbg(dev, "gpio_lcd_bl_en = 0x%x, gpio_lcd_bl_en_level=%d\n", gpio_lcd_bl_en, gpio_lcd_bl_en_level);

		ret = devm_gpio_request(dev, gpio_lcd_bl_en, "GPIO_LCD_BL_EN");

		if (ret < 0) {
			dev_err(dev, "request gpio_lcd_bl_en = 0x%x failed, ret=%d\n", gpio_lcd_bl_en, ret);
			gpio_lcd_bl_en = -ENODEV;
		}
	}

	disp_pwm0_pinctrl = devm_pinctrl_get(dev);

	if (IS_ERR(disp_pwm0_pinctrl)) {
		ret = PTR_ERR(disp_pwm0_pinctrl);
		disp_pwm0_pinctrl = NULL;
		dev_err(dev, "Cannot find pinctrl 'disp_pwm0_pinctrl'!\n");
	} else {
		pwm0_mode = pinctrl_lookup_state(disp_pwm0_pinctrl, "disp_pwm0_pwm0_mode");

		if (IS_ERR(pwm0_mode)) {
			ret = PTR_ERR(pwm0_mode);
			pwm0_mode = NULL;
			dev_err(dev, "Cannot find pinctrl state 'disp_pwm0_pwm0_mode'!\n");
		} else {
			pinctrl_select_state(disp_pwm0_pinctrl, pwm0_mode);
		}

		gpio_mode_out_low = pinctrl_lookup_state(disp_pwm0_pinctrl, "disp_pwm0_gpio_mode_out_low");

		if (IS_ERR(gpio_mode_out_low)) {
			ret = PTR_ERR(gpio_mode_out_low);
			gpio_mode_out_low = NULL;
			dev_err(dev, "Cannot find pinctrl state 'disp_pwm0_gpio_mode_out_low'!\n");
		}

		gpio_mode_out_high = pinctrl_lookup_state(disp_pwm0_pinctrl, "disp_pwm0_gpio_mode_out_high");

		if (IS_ERR(gpio_mode_out_high)) {
			ret = PTR_ERR(gpio_mode_out_high);
			gpio_mode_out_high = NULL;
			dev_err(dev, "Cannot find pinctrl state 'disp_pwm0_gpio_mode_out_high'!\n");
		}
	}
}

static void lcm_backlight_off(void)
{
	dev_err(lcm_dev, "%s\n", __func__);

	if ((disp_pwm0_pinctrl != NULL) && (gpio_mode_out_high != NULL)) {
		pinctrl_select_state(disp_pwm0_pinctrl, gpio_mode_out_high);
	}

	/* GPIOEXT14: LCM_LED for backlight panel */
	if (gpio_is_valid(gpio_lcd_bl_en)) {
		gpio_set_value(gpio_lcd_bl_en, !gpio_lcd_bl_en_level);
	}
}

static void lcm_backlight_on(void)
{
	dev_err(lcm_dev, "%s\n", __func__);

	if ((disp_pwm0_pinctrl != NULL) && (pwm0_mode != NULL)) {
		pinctrl_select_state(disp_pwm0_pinctrl, pwm0_mode);
	}

	/* GPIOEXT14: LCM_LED for backlight panel */
	if (gpio_is_valid(gpio_lcd_bl_en)) {
		gpio_set_value(gpio_lcd_bl_en, gpio_lcd_bl_en_level);
	}
}

static int lcm_probe(struct device *dev)
{
	lcm_dev = dev;
	lcm_vgp3 = lcm_vgp5 = NULL;
	gpio_lcd_rst_en = -ENODEV;
	gpio_lcd_12v_en = -ENODEV;
	disp_pwm0_pinctrl = NULL;
	gpio_lcd_bl_en = -ENODEV;
	pwm0_mode = gpio_mode_out_low = gpio_mode_out_high = NULL;

	lcm_request_gpio_control(dev);
	lcm_get_vgp3_supply(dev);
	lcm_get_vgp5_supply(dev);
	lcm_vgp5_supply_enable();
	lcm_vgp3_supply_enable();

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void lcm_bl_early_suspend(struct early_suspend *h)
{
	dev_err(lcm_dev, "%s\n", __func__);
	lcm_backlight_off();
}

static void lcm_bl_late_resume(struct early_suspend *h)
{
	dev_err(lcm_dev, "%s\n", __func__);
	lcm_backlight_on();
}

static struct early_suspend lcm_bl_early_suspend_handler = {
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
	.suspend = lcm_bl_early_suspend,
	.resume = lcm_bl_late_resume,
};
#endif

static void lcm_shutdown(struct device *dev)
{
	lcm_backlight_off();

	if (lcm_vgp3 != NULL) {
		devm_regulator_put(lcm_vgp3);
		lcm_vgp3 = NULL;
	}

	if (lcm_vgp5 != NULL) {
		devm_regulator_put(lcm_vgp5);
		lcm_vgp5 = NULL;
	}

	if (gpio_lcd_rst_en != -ENODEV) {
		devm_gpio_free(dev, gpio_lcd_rst_en);
	}

	if (disp_pwm0_pinctrl != NULL) {
		devm_pinctrl_put(disp_pwm0_pinctrl);
		disp_pwm0_pinctrl = NULL;
	}

	if (gpio_lcd_bl_en != -ENODEV) {
		devm_gpio_free(dev, gpio_lcd_bl_en);
	}

	pwm0_mode = gpio_mode_out_low = gpio_mode_out_high = NULL;

	if (gpio_lcd_12v_en != -ENODEV) {
		devm_gpio_free(dev, gpio_lcd_12v_en);
	}

	lcm_dev = NULL;
}

static const struct of_device_id lcm_of_ids[] = {
	{.compatible = "mediatek,mt8173-lcm",},
	{}
};

static struct platform_driver lcm_driver = {
	.driver = {
		.name = LCM_DRIVER_NAME,
		.owner = THIS_MODULE,
		.probe = lcm_probe,
		.shutdown = lcm_shutdown,
#ifdef CONFIG_OF
		.of_match_table = lcm_of_ids,
#endif
	},
};

static int __init lcm_init(void)
{
	pr_debug("LCM: Register panel driver for %s\n", LCM_DRIVER_NAME);

	if (platform_driver_register(&lcm_driver)) {
		pr_err("LCM: failed to register driver %s!\n", LCM_DRIVER_NAME);
		return -ENODEV;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend(&lcm_bl_early_suspend_handler);
#endif

	return 0;
}

static void __exit lcm_exit(void)
{
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&lcm_bl_early_suspend_handler);
#endif

	platform_driver_unregister(&lcm_driver);
	pr_debug("LCM: Unregister driver %s done\n", LCM_DRIVER_NAME);
}

late_initcall(lcm_init);
module_exit(lcm_exit);

static int tc358765xbg_i2c_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	dev_dbg(&client->dev, "[Kernel/LCM] %s enter, name=%s addr=0x%x\n", __func__, client->name, client->addr);

	tc358765xbg_i2c_client = client;

#ifdef TC358765XBG_DEBUG
	tc358765xbg_check_dsi_basic_reg();
#endif // TC358765XBG_DEBUG

	return 0;
}

static int tc358765xbg_i2c_driver_remove(struct i2c_client *client)
{
	dev_dbg(&client->dev, "[Kernel/LCM] %s enter\n", __func__);

	tc358765xbg_i2c_client = NULL;
	i2c_unregister_device(client);

	return 0;
}

static const struct i2c_device_id tc358765xbg_i2c_id[] = { {I2C_DRIVER_NAME, 0}, {} };

#ifdef CONFIG_OF
static const struct of_device_id tc358765xbg_id[] = {
	{.compatible = I2C_DRIVER_NAME},
	{},
};

MODULE_DEVICE_TABLE(of, tc358765xbg_id);
#endif

static struct i2c_driver tc358765xbg_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = I2C_DRIVER_NAME,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(tc358765xbg_id),
#endif
	},
	.probe = tc358765xbg_i2c_driver_probe,
	.remove = tc358765xbg_i2c_driver_remove,
	.id_table = tc358765xbg_i2c_id,
};

static int __init tc358765xbg_i2c_init(void)
{
	int ret = 0;

	pr_debug("[Kernel/LCM] %s enter\n", __func__);

	ret = i2c_add_driver(&tc358765xbg_i2c_driver);

	if (ret != 0) {
		pr_err("Failed to register tc358765xbg i2c driver, err=%d\n", ret);
	} else {
		pr_info("Successfully register tc358765xbg i2c driver.\n");
	}

	return 0;
}

static void __exit tc358765xbg_i2c_exit(void)
{
	i2c_del_driver(&tc358765xbg_i2c_driver);
}

module_init(tc358765xbg_i2c_init);
module_exit(tc358765xbg_i2c_exit);

MODULE_AUTHOR("mediatek");
MODULE_DESCRIPTION("LCM display subsystem driver");
MODULE_LICENSE("GPL");
