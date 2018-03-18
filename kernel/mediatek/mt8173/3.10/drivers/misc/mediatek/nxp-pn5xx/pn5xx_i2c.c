/*
 * Copyright (C) 2010 Trusted Logic S.A.
 * modifications copyright (C) 2015 NXP B.V.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include "pn5xx_i2c.h"
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>

#include <linux/dma-mapping.h>

//#include <cust_gpio_usage.h>
//#include <cust_eint.h>

#include <mach/mt_gpio.h>
#include <mach/eint.h>

#define MAX_BUFFER_SIZE	512

#define MODE_OFF    0
#define MODE_RUN    1
#define MODE_FW     2

/*
 * device node named /dev/pn544
 * android <-> "/dev/pn544" <-> PN5xx I2C driver
 */
#define CHIP "pn544"
#define DRIVER_CARD "PN54x NFC"
#define DRIVER_DESC "NFC driver for PN54x Family"
//#define PN54X_DEV_NAME "pn547"
#define PN54X_DEV_NAME "pn544"
#define I2C_ID_NAME PN54X_DEV_NAME
#define PN54X_CLIENT_TIMING 400  //I2C speed


#define I2C_RETRY_COUNT 10

struct pn54x_dev	{
	wait_queue_head_t read_wq;
	struct mutex read_mutex;
	struct i2c_client *client;
	struct miscdevice pn54x_device;
	int ven_gpio;
	int firm_gpio;
	int irq_gpio;
	int clkreq_gpio;
	struct regulator *pvdd_reg;
	struct regulator *vbat_reg;
	struct regulator *pmuvcc_reg;
	struct regulator *sevdd_reg;
	bool irq_enabled;
	spinlock_t irq_enabled_lock;
};

static struct pn54x_dev *gpn54x_dev;
/* For DMA */
static char *I2CDMAWriteBuf = NULL;
static dma_addr_t I2CDMAWriteBuf_pa;
static char *I2CDMAReadBuf = NULL;
static dma_addr_t I2CDMAReadBuf_pa;

static int is_debug = 0;
#define DBUF(buff,count) \
	if (is_debug) \
		for (i = 0; i < count; i++) \
			printk("[NFC] %s : [%d] = 0x%x\n", \
				__func__, i, buff[i]);

#define D(x...) if (is_debug) printk("[NFC] " x)
#define I(x...) printk("[NFC] " x)
#define E(x...) printk("[NFC][Err] " x)

/**********************************************************
 * Interrupt control and handler
 **********************************************************/
static void pn54x_disable_irq(struct pn54x_dev *pn54x_dev)
{
	unsigned long flags;

	spin_lock_irqsave(&pn54x_dev->irq_enabled_lock, flags);
	if (pn54x_dev->irq_enabled) {
		mt_eint_mask(pn54x_dev->client->irq);
		pn54x_dev->irq_enabled = false;
	}
	spin_unlock_irqrestore(&pn54x_dev->irq_enabled_lock, flags);
}

void pn54x_dev_irq_handler(void)
{
	struct pn54x_dev *pn54x_dev = gpn54x_dev;
	
	D("%s\n", __func__);

	if (NULL == pn54x_dev) {
		E("pn54x_dev NULL\n");
		return;
	}

	pn54x_disable_irq(pn54x_dev);

	/* Wake up waiting readers */
	wake_up(&pn54x_dev->read_wq);	
}

static void pn54x_firmware_update_enable(struct pn54x_dev *dev, int onoff)
{
	int ret;

#if 0
	mt_set_gpio_mode((dev->firm_gpio | 0x80000000), GPIO_NFC_FIRM_PIN_M_GPIO);
	mt_set_gpio_dir((dev->firm_gpio | 0x80000000), GPIO_DIR_OUT);
	mt_set_gpio_out((dev->firm_gpio | 0x80000000), onoff);
#else
	ret = gpio_direction_output(dev->firm_gpio, 0);
	if (ret < 0) {
		pr_err("%s : not able to set firm_gpio as output\n", __func__);
	}
	else
	{   
		msleep(10);
		if (dev->firm_gpio) {
			gpio_set_value(dev->firm_gpio, onoff);
		}
	}
#endif
}


/**********************************************************
 * private functions
 **********************************************************/
static int pn544_enable(struct pn54x_dev *dev, int mode)
{
	int ret;

	if (MODE_RUN == mode) {
		I("%s MODE_RUN, VEN HIGH\n", __func__);

		pn54x_firmware_update_enable(dev, GPIO_OUT_ZERO);
#if 0
		mt_set_gpio_mode((dev->ven_gpio | 0x80000000), GPIO_NFC_RST_PIN_M_GPIO);
		mt_set_gpio_dir((dev->ven_gpio | 0x80000000), GPIO_DIR_OUT);
		mt_set_gpio_out((dev->ven_gpio | 0x80000000), GPIO_OUT_ONE);
#else
		ret = gpio_direction_output(dev->ven_gpio, 0);
		if (ret < 0) {
			pr_err("%s : not able to set ven_gpio as output\n", __func__);
		}
                else
		{
			msleep(10);
			gpio_set_value(dev->ven_gpio, 1);
		}
#endif
		msleep(100);
	}
	else if (MODE_FW == mode) {
		/* power on with firmware download (requires hw reset)
		 */
		I("%s power on with firmware\n", __func__);

#if 0
		mt_set_gpio_mode((dev->ven_gpio | 0x80000000), GPIO_NFC_RST_PIN_M_GPIO);
		mt_set_gpio_dir((dev->ven_gpio | 0x80000000), GPIO_DIR_OUT);
		mt_set_gpio_out((dev->ven_gpio | 0x80000000), GPIO_OUT_ONE);
		msleep(20);
		pn54x_firmware_update_enable(dev, GPIO_OUT_ONE);
		msleep(20);
		mt_set_gpio_out((dev->ven_gpio | 0x80000000), GPIO_OUT_ZERO);
		msleep(100);
		mt_set_gpio_out((dev->ven_gpio | 0x80000000), GPIO_OUT_ONE);
		msleep(20);
#else
		ret = gpio_direction_output(dev->ven_gpio, 0);
		if (ret < 0) {
			pr_err("%s : not able to set ven_gpio as output\n", __func__);
		}
		else
		{
			msleep(10);
			gpio_set_value(dev->ven_gpio, 1);
			msleep(20);
			pn54x_firmware_update_enable(dev, GPIO_OUT_ONE);
			msleep(20);
			gpio_set_value(dev->ven_gpio, 0);
			msleep(100);
			gpio_set_value(dev->ven_gpio, 1);
			msleep(20);
		}
#endif
		
		return 0;
	}
	else {
		E("%s bad arg %d\n", __func__, mode);
		return -EINVAL;
	}

	return 0;
}

static void pn544_disable(struct pn54x_dev *dev)
{
	int ret;

	I("%s VEN LOW\n", __func__);
	/* power off */
	pn54x_firmware_update_enable(dev, GPIO_OUT_ZERO);
#if 0
	mt_set_gpio_mode((dev->ven_gpio | 0x80000000), GPIO_NFC_RST_PIN_M_GPIO);
	mt_set_gpio_dir((dev->ven_gpio | 0x80000000), GPIO_DIR_OUT);
	mt_set_gpio_out((dev->ven_gpio | 0x80000000), GPIO_OUT_ZERO);
#else
	ret = gpio_direction_output(dev->ven_gpio, 0);
	if (ret < 0) {
		pr_err("%s : not able to set ven_gpio as output\n", __func__);
	}
	else
	{
		msleep(10);
		gpio_set_value(dev->ven_gpio, 0);
	}
#endif
	// TODO:
	msleep(100);
}

/**********************************************************
 * driver functions
 **********************************************************/
static int pn54x_rx_data(int length)
{
	struct pn54x_dev *pn54x_dev = gpn54x_dev;
	int ret = 0;
	uint8_t loop_i;
	
	pn54x_dev->client->addr = (pn54x_dev->client->addr & I2C_MASK_FLAG);   
// 	pn54x_dev->client->ext_flag |= I2C_DMA_FLAG;
//	pn54x_dev->client->timing = PN54X_CLIENT_TIMING;

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {
		if (loop_i > 0) {
			D("%s: retry %d ........\n", __func__, loop_i);
		}
//		ret = i2c_master_recv(pn54x_dev->client, (unsigned char *)(uintptr_t)I2CDMAReadBuf_pa, length);
		ret = i2c_master_recv(pn54x_dev->client, (unsigned char *)(uintptr_t)I2CDMAReadBuf, length);
		if (ret != length) {
			D("%s : i2c_master_recv returned %d\n", __func__, ret);
		} else {
			break;
		}
		msleep(1);
	}

	if (loop_i >= I2C_RETRY_COUNT) {
		E("%s Error: retry over %d\n", __func__, I2C_RETRY_COUNT);
		return -EIO;
	} else {
		D("%s, RX done!\n", __func__);
	}

	return ret;
}

static int pn54x_tx_data(int length)
{
	struct pn54x_dev *pn54x_dev = gpn54x_dev;
	uint8_t loop_i;
	int ret = 0;

	pn54x_dev->client->addr = (pn54x_dev->client->addr & I2C_MASK_FLAG);
//	pn54x_dev->client->ext_flag |= I2C_DMA_FLAG;
//	pn54x_dev->client->timing = PN54X_CLIENT_TIMING;		

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {
		if (loop_i > 0) {
			D("%s: retry %d ........\n", __func__, loop_i);
		}
//		ret = i2c_master_send(pn54x_dev->client, (unsigned char *)(uintptr_t)I2CDMAWriteBuf_pa, length);
		ret = i2c_master_send(pn54x_dev->client, (unsigned char *)(uintptr_t)I2CDMAWriteBuf, length);
		if (ret != length) {
			D("%s : i2c_master_send returned %d\n", __func__, ret);
		} else {
			break;
		}		
		msleep(1);
	}

	if (loop_i >= I2C_RETRY_COUNT) {
		E("%s:  Error: retry over %d\n", __func__, I2C_RETRY_COUNT);
		return -EIO;
	} else {
		D("%s, TX done!\n", __func__);
	}

	return ret;
}

static ssize_t pn54x_dev_read(struct file *filp, char __user *buf,
		size_t count, loff_t *offset)
{
	struct pn54x_dev *pn54x_dev = filp->private_data;
	int ret = 0;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	D("%s : reading %zu bytes.\n", __func__, count);

	mutex_lock(&pn54x_dev->read_mutex);

	if(!mt_get_gpio_in((pn54x_dev->irq_gpio | 0x80000000))) {
		if (filp->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			E("%s : goto fail\n", __func__);
			goto fail;
		}

		while (1) {
			pn54x_dev->irq_enabled = true;
			mt_eint_unmask(pn54x_dev->client->irq); /* Enable Interrupt */
			ret = wait_event_interruptible(
					pn54x_dev->read_wq,
					//mt_get_gpio_in((pn54x_dev->irq_gpio | 0x80000000)));
					!pn54x_dev->irq_enabled);		

			pn54x_disable_irq(pn54x_dev);
			
			if (ret) {
				E("%s : wait_event_interruptiblet ret=%d........\n", __func__, ret);
				goto fail;
			}			

			if (mt_get_gpio_in((pn54x_dev->irq_gpio | 0x80000000)))
				break;

			E("%s: spurious interrupt detected\n", __func__);
		}
	}

	/* Read data */
	ret = pn54x_rx_data(count);

	mutex_unlock(&pn54x_dev->read_mutex);

	/* pn54x seems to be slow in handling I2C read requests
	 * so add 1ms delay after recv operation */
	// TODO:
	//udelay(1000);

	if (ret < 0) {
		E("%s: i2c_master_recv returned %d\n", __func__, ret);
		return ret;
	}
	if (ret > count) {
		E("%s: received too many bytes from i2c (%d)\n",
			__func__, ret);
		return -EIO;
	}
	if (copy_to_user(buf, I2CDMAReadBuf, ret)) {
		E("%s : failed to copy to user space\n", __func__);
		return -EFAULT;
	}
	return ret;

fail:
	mutex_unlock(&pn54x_dev->read_mutex);
	return ret;
}

static ssize_t pn54x_dev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *offset)
{
	struct pn54x_dev  *pn54x_dev;
	int ret;

	pn54x_dev = filp->private_data;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	if (copy_from_user(I2CDMAWriteBuf, buf, count)) {
		E("%s : failed to copy from user space\n", __func__);
		return -EFAULT;
	}

	D("%s : writing %zu bytes.\n", __func__, count);
	
	/* Write data */	
	ret = pn54x_tx_data(count);

	/* pn54x seems to be slow in handling I2C write requests
	 * so add 1ms delay after I2C send oparation */
	//udelay(1000);

	return ret;
}

static int pn54x_dev_open(struct inode *inode, struct file *filp)
{
	struct pn54x_dev *pn54x_dev = container_of(filp->private_data,
											   struct pn54x_dev,
											   pn54x_device);

	filp->private_data = pn54x_dev;

	I("%s : %d,%d\n", __func__, imajor(inode), iminor(inode));

	// pn544_enable(pn54x_dev, MODE_RUN);

	return 0;
}

static int pn54x_dev_release(struct inode *inode, struct file *filp)
{
	// struct pn54x_dev *pn54x_dev = container_of(filp->private_data,
	//										   struct pn54x_dev,
	//										   pn54x_device);

	I("%s : closing %d,%d\n", __func__, imajor(inode), iminor(inode));

	// pn544_disable(pn54x_dev);

	return 0;
}

static long  pn54x_dev_ioctl(struct file *filp, unsigned int cmd,
				unsigned long arg)
{
	struct pn54x_dev *pn54x_dev = filp->private_data;

	//I("%s, cmd=%d, arg=%lu\n", __func__, cmd, arg);
	switch (cmd) {
	case PN544_SET_PWR:
		I("%s, PN544_SET_PWR, arg=%lu\n", __func__, arg);
		if (arg == 2) {
			/* power on w/FW */
			if (GPIO_UNUSED == pn544_enable(pn54x_dev, arg)) {
				return GPIO_UNUSED;
			}
		} else if (arg == 1) {
			/* power on */
			pn544_enable(pn54x_dev, arg);
		} else  if (arg == 0) {
			/* power off */
			pn544_disable(pn54x_dev);
		} else {
			E("%s bad SET_PWR arg %lu\n", __func__, arg);
			return -EINVAL;
		}
		break;
	case PN54X_CLK_REQ:
#if 0
		if(1 == arg){
			if(gpio_is_valid(pn54x_dev->clkreq_gpio)){
				gpio_set_value(pn54x_dev->clkreq_gpio, 1);
			}
			else {
				pr_err("%s Unused Clkreq GPIO %lu\n", __func__, arg);
				return GPIO_UNUSED;
			}
		}
		else if(0 == arg) {
			if(gpio_is_valid(pn54x_dev->clkreq_gpio)){
				gpio_set_value(pn54x_dev->clkreq_gpio, 0);
			}
			else {
				pr_err("%s Unused Clkreq GPIO %lu\n", __func__, arg);
				return GPIO_UNUSED;
			}
		} else {
			pr_err("%s bad CLK_REQ arg %lu\n", __func__, arg);
			return -EINVAL;
		}
#else
		I("%s CLK_REQ needless\n", __func__);
#endif
		break;
	default:
		E("%s bad ioctl %u\n", __func__, cmd);
		return -EINVAL;
	}

	return 0;
}

static const struct file_operations pn54x_dev_fops = {
	.owner	= THIS_MODULE,
	.llseek	= no_llseek,
	.read	= pn54x_dev_read,
	.write	= pn54x_dev_write,
	.open	= pn54x_dev_open,
	.release  = pn54x_dev_release,
	.unlocked_ioctl  = pn54x_dev_ioctl,
};

/*
 * Translate OpenFirmware node properties into platform_data
 */
static int pn54x_get_pdata(struct device *dev,
							struct pn544_i2c_platform_data *pdata)
{
	struct device_node *node;
	int val;
	int ret;

	dev_warn(dev, "%s\n", __func__);

	/* make sure there is actually a device tree node */
	//node = of_find_compatible_node(NULL, NULL, "nxp,pn547");
	node = dev->of_node;
	if (!node)
		return -ENODEV;

	memset(pdata, 0, sizeof(*pdata));

	/* read the dev tree data */
	/* ven pin - enable's power to the chip - REQUIRED */
	ret = of_property_read_u32_index(node, "enable-gpios", 0, &val);
	if (ret >= 0) {
		pdata->ven_gpio = val;
		I("VEN GPIO %d\n", pdata->ven_gpio);
	}
	else {
		E("VEN GPIO error getting from OF node\n");
		return ret;
	}

	/* irq pin - data available irq - REQUIRED */
	ret = of_property_read_u32_index(node, "interrupt-gpios", 0, &val);
	if (ret >= 0) {
		pdata->irq_gpio = val;
		I("IRQ GPIO %d\n", pdata->irq_gpio);
	}
	else {
		E("IRQ GPIO error getting from OF node\n");
		return ret;
	}

	/* fw download pin - fw upgrade - REQUIRED */
	ret = of_property_read_u32_index(node, "dl-gpios", 0, &val);
	if (ret >= 0) {
		pdata->firm_gpio = val;
		I("FIRM GPIO %d\n", pdata->firm_gpio);
	}
	else {
		E("FIRM GPIO error getting from OF node\n");
		return ret;
	}

	return 0;
}

/*
 * pn54x_probe
 */
#ifdef KERNEL_3_4_AND_OLDER
 static int __devinit pn54x_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
#else
static int pn54x_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
#endif
{
	int ret;
	struct pn544_i2c_platform_data *pdata; // gpio values, from board file or DT
	struct pn544_i2c_platform_data tmp_pdata;
	struct pn54x_dev *pn54x_dev; // internal device specific data	

	I("%s\n", __func__);

	/* ---- retrieve the platform data ---- */
	/* If the dev.platform_data is NULL, then */
	/* attempt to read from the device tree */
	if(!client->dev.platform_data)
	{
		ret = pn54x_get_pdata(&(client->dev), &tmp_pdata);
		if(ret){
			return ret;
		}

		pdata = &tmp_pdata;
	}
	else
	{
		pdata = client->dev.platform_data;
	}

	if (pdata == NULL) {
		E("%s : nfc probe fail, platform_data null\n", __func__);
		return  -ENODEV;
	}

	/* validate the the adapter has basic I2C functionality */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		E("%s : need I2C_FUNC_I2C\n", __func__);
		return  -ENODEV;
	}

	/* allocate the pn54x driver information structure */
	pn54x_dev = kzalloc(sizeof(*pn54x_dev), GFP_KERNEL);
	if (pn54x_dev == NULL) {
		E("failed to allocate memory for module data\n");
		ret = -ENOMEM;
		goto err_exit;
	}

	/* store the platform data in the driver info struct */
	if (pdata) {
		pn54x_dev->irq_gpio = pdata->irq_gpio;
		pn54x_dev->ven_gpio = pdata->ven_gpio;
		pn54x_dev->firm_gpio = pdata->firm_gpio;
	}

	pn54x_dev->client = client;

#if 0
	mt_set_gpio_mode((pn54x_dev->ven_gpio | 0x80000000), GPIO_NFC_RST_PIN_M_GPIO);
	mt_set_gpio_dir((pn54x_dev->ven_gpio | 0x80000000), GPIO_DIR_OUT);
	mt_set_gpio_out((pn54x_dev->ven_gpio | 0x80000000), GPIO_OUT_ONE);

	mt_set_gpio_mode((pn54x_dev->firm_gpio | 0x80000000), GPIO_NFC_FIRM_PIN_M_GPIO);
	mt_set_gpio_dir((pn54x_dev->firm_gpio | 0x80000000), GPIO_DIR_OUT);
	mt_set_gpio_out((pn54x_dev->firm_gpio | 0x80000000), GPIO_OUT_ZERO);
	
	mt_set_gpio_mode((pn54x_dev->irq_gpio | 0x80000000), GPIO_NFC_EINT_PIN_M_EINT);
	mt_set_gpio_dir((pn54x_dev->irq_gpio | 0x80000000), GPIO_DIR_IN);
	mt_set_gpio_pull_enable((pn54x_dev->irq_gpio | 0x80000000), GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select((pn54x_dev->irq_gpio | 0x80000000), GPIO_PULL_DOWN);
#else
	ret = gpio_direction_input(pn54x_dev->irq_gpio);
	if (ret < 0) {
		pr_err("%s :not able to set irq_gpio as input\n", __func__);
	}

	ret = gpio_direction_output(pn54x_dev->ven_gpio, 0);
	if (ret < 0) {
		pr_err("%s : not able to set ven_gpio as output\n", __func__);
		gpio_free(pdata->irq_gpio);
	}

	if (pdata->firm_gpio) {
		ret = gpio_direction_output(pn54x_dev->firm_gpio, 0);
		if (ret < 0) {
			pr_err("%s : not able to set firm_gpio as output\n", __func__);
			gpio_free(pdata->irq_gpio);
			gpio_free(pdata->ven_gpio);
		}
	}
#endif

	/* init mutex and queues */
	init_waitqueue_head(&pn54x_dev->read_wq);
	mutex_init(&pn54x_dev->read_mutex);
	spin_lock_init(&pn54x_dev->irq_enabled_lock);

	/* register as a misc device - character based with one entry point */
	pn54x_dev->pn54x_device.minor = MISC_DYNAMIC_MINOR;
	pn54x_dev->pn54x_device.name = CHIP;
	pn54x_dev->pn54x_device.fops = &pn54x_dev_fops;
	ret = misc_register(&pn54x_dev->pn54x_device);
	if (ret) {
		E("%s : misc_register failed\n", __FILE__);
		goto err_misc_register;
	}

#ifdef CONFIG_64BIT    
	I2CDMAWriteBuf = (char *)dma_alloc_coherent(&client->dev, MAX_BUFFER_SIZE, (dma_addr_t *)&I2CDMAWriteBuf_pa, GFP_KERNEL);
#else
	I2CDMAWriteBuf = (char *)dma_alloc_coherent(NULL, MAX_BUFFER_SIZE, (dma_addr_t *)&I2CDMAWriteBuf_pa, GFP_KERNEL);
#endif

	if (I2CDMAWriteBuf == NULL) {
		E("%s : failed to allocate dma buffer\n", __func__);
		goto err_request_irq_failed;
	}
	
#ifdef CONFIG_64BIT 	
	I2CDMAReadBuf = (char *)dma_alloc_coherent(&client->dev, MAX_BUFFER_SIZE, (dma_addr_t *)&I2CDMAReadBuf_pa, GFP_KERNEL);
#else
	I2CDMAReadBuf = (char *)dma_alloc_coherent(NULL, MAX_BUFFER_SIZE, (dma_addr_t *)&I2CDMAReadBuf_pa, GFP_KERNEL);
#endif

	if (I2CDMAReadBuf == NULL) {
		E("%s : failed to allocate dma buffer\n", __func__);
		goto err_request_irq_failed;
	}
	//D("%s :I2CDMAWriteBuf_pa %ld, I2CDMAReadBuf_pa,%ld\n", __func__, (long int)I2CDMAWriteBuf_pa, I2CDMAReadBuf_pa);

	//I("%s, irq_gpio(%d)CUST_EINT_NFC_NUM(%d)\n", __func__, pn54x_dev->irq_gpio, CUST_EINT_NFC_NUM);
	client->irq = pn54x_dev->irq_gpio;
	pn54x_dev->irq_enabled = true;
//	mt_eint_set_hw_debounce(pn54x_dev->irq_gpio, CUST_EINT_NFC_DEBOUNCE_CN);
	mt_eint_registration(pn54x_dev->irq_gpio, CUST_EINTF_TRIGGER_HIGH, pn54x_dev_irq_handler, 0);
	pn54x_disable_irq(pn54x_dev);

	i2c_set_clientdata(client, pn54x_dev);
	gpn54x_dev = pn54x_dev;

#if 0
	/* TEST only */
	do {
		uint8_t buffer[] = {0x05, 0xF9, 0x04, 0x00, 0xC3, 0xE5};
		I("%s: Send software reset command\n", __func__);
		ret = pn54x_tx_data(buffer, 6);
		if (ret < 0) {
			E("%s, i2c Tx error!\n", __func__);
		}
	} while (0);
#endif

	return 0;

err_request_irq_failed:
	misc_deregister(&pn54x_dev->pn54x_device);
err_misc_register:
	kfree(pn54x_dev);
err_exit:
	return ret;
}

int pn54x_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct pn54x_dev *pn54x_dev;	
	pn54x_dev = i2c_get_clientdata(client);
	
	I("%s, do nothing\n", __func__);

	//mt_set_gpio_mode((pn54x_dev->ven_gpio | 0x80000000), GPIO_NFC_RST_PIN_M_GPIO);
	//mt_set_gpio_dir((pn54x_dev->ven_gpio | 0x80000000), GPIO_DIR_OUT);
	//mt_set_gpio_out((pn54x_dev->ven_gpio | 0x80000000), GPIO_OUT_ZERO);

	return 0;
}

int pn54x_resume(struct i2c_client *client)
{
	struct pn54x_dev *pn54x_dev;	
	pn54x_dev = i2c_get_clientdata(client);
	
	I("%s, do nothing\n", __func__);

	//mt_set_gpio_mode((pn54x_dev->ven_gpio | 0x80000000), GPIO_NFC_RST_PIN_M_GPIO);
	//mt_set_gpio_dir((pn54x_dev->ven_gpio | 0x80000000), GPIO_DIR_OUT);
	//mt_set_gpio_out((pn54x_dev->ven_gpio | 0x80000000), GPIO_OUT_ONE);

	return 0;
}

#ifdef KERNEL_3_4_AND_OLDER
static int __devexit pn54x_remove(struct i2c_client *client)
#else
static int pn54x_remove(struct i2c_client *client)
#endif
{
	struct pn54x_dev *pn54x_dev;
	int ret;

    if (I2CDMAWriteBuf) {
#ifdef CONFIG_64BIT
        dma_free_coherent(&client->dev, MAX_BUFFER_SIZE, I2CDMAWriteBuf,
                I2CDMAWriteBuf_pa);
#else
        dma_free_coherent(NULL, MAX_BUFFER_SIZE, I2CDMAWriteBuf,
                I2CDMAWriteBuf_pa);
#endif
        I2CDMAWriteBuf = NULL;
        I2CDMAWriteBuf_pa = 0;
    }

    if (I2CDMAReadBuf) {
#ifdef CONFIG_64BIT
        dma_free_coherent(&client->dev, MAX_BUFFER_SIZE, I2CDMAReadBuf,
                I2CDMAReadBuf_pa);
#else
        dma_free_coherent(NULL, MAX_BUFFER_SIZE, I2CDMAReadBuf,
                I2CDMAReadBuf_pa);
#endif
        I2CDMAReadBuf = NULL;
        I2CDMAReadBuf_pa = 0;
    }

	I("%s\n", __func__);

	pn54x_dev = i2c_get_clientdata(client);

#if 0
	mt_set_gpio_mode((pn54x_dev->ven_gpio | 0x80000000), GPIO_NFC_RST_PIN_M_GPIO);
	mt_set_gpio_dir((pn54x_dev->ven_gpio | 0x80000000), GPIO_DIR_OUT);
	mt_set_gpio_out((pn54x_dev->ven_gpio | 0x80000000), GPIO_OUT_ZERO);
#else
	ret = gpio_direction_output(pn54x_dev->ven_gpio, 0);
	if (ret < 0) {
		pr_err("%s : not able to set ven_gpio as output\n", __func__);
	}
	msleep(10);
	gpio_set_value(pn54x_dev->ven_gpio, 0);
#endif
	
	//free_irq(client->irq, pn54x_dev);
	misc_deregister(&pn54x_dev->pn54x_device);
	mutex_destroy(&pn54x_dev->read_mutex);

	// TODO:
	gpio_free((pn54x_dev->ven_gpio | 0x80000000));
	gpio_free((pn54x_dev->firm_gpio | 0x80000000));
	gpio_free((pn54x_dev->irq_gpio | 0x80000000));
	
	kfree(pn54x_dev);

	return 0;
}

/*
 *
 */
static struct of_device_id pn54x_dt_match[] = {
	{ .compatible = "nxp,pn544", },
//	{ .compatible = "nxp,pn547", },
	{},
};
MODULE_DEVICE_TABLE(of, pn54x_dt_match);

static const struct i2c_device_id pn54x_id[] = {
	{ PN54X_DEV_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, pn54x_id);

static struct i2c_driver pn54x_driver = {
	.id_table	= pn54x_id,
	.probe		= pn54x_probe,
	.suspend	= pn54x_suspend,
	.resume		= pn54x_resume,
#ifdef KERNEL_3_4_AND_OLDER
	.remove		= __devexit_p(pn54x_remove),
#else
	.remove		= pn54x_remove,
#endif
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= PN54X_DEV_NAME,
		.of_match_table = pn54x_dt_match,
	},
};

static int __init pn54x_dev_init(void)
{
	int ret = 0;
	
	D("%s\n", __func__);

	ret = i2c_add_driver(&pn54x_driver);
	I("%s, i2c_add_driver added(%d)\n", __func__, ret);
	
	return ret;
}

static void __exit pn54x_dev_exit(void)
{
	I("%s\n", __func__);
	i2c_del_driver(&pn54x_driver);
}

module_init(pn54x_dev_init);
module_exit(pn54x_dev_exit);

MODULE_AUTHOR("Sylvain Fonteneau");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
