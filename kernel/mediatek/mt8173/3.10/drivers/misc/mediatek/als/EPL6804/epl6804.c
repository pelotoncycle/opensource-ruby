/* /alps/mediatek/custom/common/kernel/alsps/epl6804.c - light sensors driver
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

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>

#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <asm/io.h>
//#include <cust_eint.h>
#include <cust_alsps.h>
#include <linux/hwmsen_helper.h>
#include "epl6804.h"
//#include "cust_gpio_usage.h"

#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <mach/eint.h>
#include <linux/of_gpio.h>
#include <include/linux/gpio.h>
#include <include/linux/of.h>
#include <include/linux/device.h>

#define TXBYTES 			2
#define RXBYTES 			2
#define PACKAGE_SIZE 			2
#define I2C_RETRY_COUNT 		10

#define LUX_PER_COUNT		344
// Patch changes ALS_DELAY 55 to 60, ALS_INTT 7 to 5
#define ALS_DELAY			60
#define ALS_INTT			6

#define epl6804_DEV_NAME		"EPL6804"

#define APS_TAG				"[ALS/PS] "
#define APS_FUN(f)			printk(KERN_INFO APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)		printk(KERN_ERR  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)		printk(KERN_INFO APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)		printk(KERN_INFO fmt, ##args)				 
#define FTM_CUST_ALSPS 			"/data/epl6804"

#define mt6516_I2C_DATA_PORT		((base) + 0x0000)
#define mt6516_I2C_SLAVE_ADDR	   	((base) + 0x0004)
#define mt6516_I2C_INTR_MASK		((base) + 0x0008)
#define mt6516_I2C_INTR_STAT		((base) + 0x000c)
#define mt6516_I2C_CONTROL		((base) + 0x0010)
#define mt6516_I2C_TRANSFER_LEN	 	((base) + 0x0014)
#define mt6516_I2C_TRANSAC_LEN	  	((base) + 0x0018)
#define mt6516_I2C_DELAY_LEN		((base) + 0x001c)
#define mt6516_I2C_TIMING		((base) + 0x0020)
#define mt6516_I2C_START		((base) + 0x0024)
#define mt6516_I2C_FIFO_STAT		((base) + 0x0030)
#define mt6516_I2C_FIFO_THRESH	  	((base) + 0x0034)
#define mt6516_I2C_FIFO_ADDR_CLR	((base) + 0x0038)
#define mt6516_I2C_IO_CONFIG		((base) + 0x0040)
#define mt6516_I2C_DEBUG		((base) + 0x0044)
#define mt6516_I2C_HS			((base) + 0x0048)
#define mt6516_I2C_DEBUGSTAT		((base) + 0x0064)
#define mt6516_I2C_DEBUGCTRL		((base) + 0x0068)

typedef struct _epl_raw_data
{
	u8 raw_bytes[PACKAGE_SIZE];
	u16 als_ch0_raw;
	u16 als_ch1_raw;
	u16 als_status;
} epl_raw_data;

static struct i2c_client *epl6804_i2c_client = NULL;
static const struct i2c_device_id epl6804_i2c_id[] = {
    {"epl6804",0},
    {}
};
static struct i2c_board_info __initdata i2c_epl6804={ I2C_BOARD_INFO("epl6804", (0X92>>1))};

static const struct of_device_id alsps_of_match[] = { 
	{ .compatible = "ambient,epl6804" }, 
	{ }, 
};


static int epl6804_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int epl6804_i2c_remove(struct i2c_client *client);
static int epl6804_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int epl6804_i2c_resume(struct i2c_client *client);
//static void epl6804_eint_func(void);

unsigned int alsps_int_gpio_number = 0;
static int of_get_epl6804_platform_data(struct device *dev);
static unsigned long long int_top_time;



typedef enum 
{
	CMC_TRC_ALS_DATA = 0x0001,
	//CMC_TRC_PS_DATA = 0X0002,
	CMC_TRC_EINT	= 0x0004,
	CMC_TRC_IOCTL   = 0x0008,
	CMC_TRC_I2C	 = 0x0010,
	CMC_TRC_CVT_ALS = 0x0020,
	//CMC_TRC_CVT_PS  = 0x0040,
	CMC_TRC_DEBUG   = 0x0800,
} CMC_TRC;

typedef enum 
{
	CMC_BIT_ALS	= 1,
	//CMC_BIT_PS	 = 2,
} CMC_BIT;

struct epl6804_i2c_addr {	/* define a series of i2c slave address */
	u8  write_addr;
};

struct elan_epl_data 
{
	struct alsps_hw  *hw;
	struct i2c_client *client;
	struct work_struct irq_work;

	/* i2c address group */
	struct epl6804_i2c_addr  addr;

	//int enable_pflag;
	int enable_lflag;

	/* misc */
	atomic_t trace;
	atomic_t i2c_retry;
	atomic_t als_suspend;
	atomic_t als_debounce;   /* debounce time after enabling als */
	atomic_t als_deb_on;	 /* indicates if the debounce is on */
	atomic_t als_deb_end;    /* the jiffies representing the end of debounce */

	/* data */
	u16		lux_per_count;
	u16 als;
	u8 _align;
	u8 als_thd_level;
	bool als_enable;	/* record current als status */
	ulong enable;		/* record HAL enalbe status */
	ulong pending_intr;     /* pending interrupt */

	/* data */
	u16 als_level_num;
	u16 als_value_num;
	u32 als_level[C_CUST_ALS_LEVEL-1];
	u32 als_value[C_CUST_ALS_LEVEL];
};

static struct i2c_driver epl6804_i2c_driver =
{
	.probe    = epl6804_i2c_probe,
	.remove	  = epl6804_i2c_remove,
	.suspend  = epl6804_i2c_suspend,
	.resume	  = epl6804_i2c_resume,
	.id_table = epl6804_i2c_id,
	.driver   = {
		.name = epl6804_DEV_NAME,
	},
};

static struct elan_epl_data *epl6804_obj = NULL;
static struct platform_driver epl6804_alsps_driver;
static unsigned int alsps_irq;


static epl_raw_data	gRawData;

/*
//====================I2C write operation===============//
//regaddr: ELAN epl6804 Register Address.
//bytecount: How many bytes to be written to epl6804 register via i2c bus.
//txbyte: I2C bus transmit byte(s). Single byte(0X01) transmit only slave address.
//data: setting value.
//
// Example: If you want to write single byte to 0x1D register address, show below
//		  elan_epl6804_I2C_Write(client,0x1D,0x01,0X02,0xff);
//
*/
static int elan_epl6804_I2C_Write(struct i2c_client *client, uint8_t regaddr, uint8_t bytecount, uint8_t txbyte, uint8_t data)
{
	uint8_t buffer[2];
	int ret = 0;
	int retry;
    
	buffer[0] = (regaddr<<3) | bytecount ;
	buffer[1] = data;
	APS_DBG("[ALS_PS][EPL6804] %s entry \n",__func__);

	for(retry = 0; retry < I2C_RETRY_COUNT; retry++)
	{
		ret = i2c_master_send(client, buffer, txbyte);
		if (ret == txbyte)
			break;
		msleep(10);
	}

	if(retry>=I2C_RETRY_COUNT)
	{
		APS_DBG(KERN_ERR "[ELAN epl6804 error] %s i2c write retry over %d\n",__func__, I2C_RETRY_COUNT);
		return -EINVAL;
	}
    APS_DBG("[ALS_PS][EPL6804] %s done \n",__func__);
	return ret;
}

/*
//====================I2C read operation===============//
*/
static int elan_epl6804_I2C_Read(struct i2c_client *client)
{
	uint8_t buffer[RXBYTES];
	int ret = 0, i =0;
	int retry;
    APS_DBG("[ALS_PS][EPL6804] %s entry \n",__func__);
	for(retry=0; retry<I2C_RETRY_COUNT; retry++)
	{
		ret = i2c_master_recv(client, buffer, RXBYTES);
		if (ret == RXBYTES)
			break;
		msleep(10);
	}

	if(retry>=I2C_RETRY_COUNT)
	{
		APS_ERR(KERN_ERR "[ELAN epl6804 error] %s i2c read retry over %d\n",__func__, I2C_RETRY_COUNT);
		return -EINVAL;
	}

	for(i=0; i<PACKAGE_SIZE; i++)
		gRawData.raw_bytes[i] = buffer[i];
    APS_DBG("[ALS_PS][EPL6804] %s done \n",__func__);
	return ret;
}

static int elan_epl6804_lsensor_enable(struct elan_epl_data *epl_data, int enable)
{
	int ret = 0;
	uint8_t regdata;
	struct i2c_client *client = epl_data->client;
	
	APS_DBG("[ALS_PS][EPL6804] %s entry \n",__func__);
	APS_DBG("[ALS_PS][EPL6804] %s enable = %d\n", __func__, enable);

	epl_data->enable_lflag = enable;

#if 0
	if(enable) {
		// reg7: Power Up
		elan_epl6804_I2C_Write(client,REG_7,R_SINGLE_BYTE,0x01,0x00);
		elan_epl6804_I2C_Read(client);
		regdata = gRawData.raw_bytes[0];
		regdata &= 0xFD;
		regdata |= EPL_C_P_UP;;
		elan_epl6804_I2C_Write(client,REG_7,W_SINGLE_BYTE,0X02,regdata);

		msleep(2);

		// reg0: LS mode
		elan_epl6804_I2C_Write(client,REG_0,R_SINGLE_BYTE,0x01,0x00);
		elan_epl6804_I2C_Read(client);
		regdata = gRawData.raw_bytes[0];
		regdata &= 0xF3;
		regdata |= EPL_ALS_MODE;
		elan_epl6804_I2C_Write(client,REG_0,W_SINGLE_BYTE,0X02,regdata);

		msleep(2);

		// reg1:
		elan_epl6804_I2C_Write(client,REG_1,W_SINGLE_BYTE,0x02,0x32);

		msleep(2);
	} else {
		// reg7 : Power Down
		elan_epl6804_I2C_Write(client,REG_7,R_SINGLE_BYTE,0x01,0x00);
		elan_epl6804_I2C_Read(client);
		regdata = gRawData.raw_bytes[0];
		regdata &= 0xFD;
		regdata |= EPL_C_P_DOWN;
		elan_epl6804_I2C_Write(client,REG_7,W_SINGLE_BYTE,0X02,regdata);
	}
#endif

	if(enable) {
		regdata = EPL_INT_DISABLE;
		ret = elan_epl6804_I2C_Write(client,REG_9,W_SINGLE_BYTE,0x02, regdata);

		regdata = EPL_S_SENSING_MODE | EPL_SENSING_32_TIME | EPL_ALS_MODE | EPL_L_GAIN;
		ret = elan_epl6804_I2C_Write(client,REG_0,W_SINGLE_BYTE,0X02,regdata);

		regdata = ALS_INTT<<4 | EPL_PST_1_TIME | EPL_10BIT_ADC;
		ret = elan_epl6804_I2C_Write(client,REG_1,W_SINGLE_BYTE,0X02,regdata);

		// ret = elan_epl6804_I2C_Write(client,REG_10,W_SINGLE_BYTE,0X02,EPL_GO_MID);
		// ret = elan_epl6804_I2C_Write(client,REG_11,W_SINGLE_BYTE,0x02,EPL_GO_LOW);

		ret = elan_epl6804_I2C_Write(client,REG_7,W_SINGLE_BYTE,0X02,EPL_C_RESET);
		ret = elan_epl6804_I2C_Write(client,REG_7,W_SINGLE_BYTE,0x02,EPL_C_START_RUN);
		msleep(ALS_DELAY);
	}


	if(ret<0)
		APS_ERR("[ELAN epl6804 error]%s: als_enable %d fail\n",__func__,ret);
	else
		ret = 0;
	APS_DBG("[ALS_PS][EPL6804] %s done \n",__func__);
	return ret;
}

static int epl6804_get_als_value(struct elan_epl_data *obj, u16 als)
{
	u16 curr_als = 0;
	APS_DBG("als raw: %d\n", als);
	curr_als = (als * obj->lux_per_count)/1000;
	APS_DBG("curr_als: %d\n", curr_als);
	return curr_als;

#if 0
	static u16 prev_als = 0;
	u16 curr_als = 0;

	if (als >= 10000)
		curr_als = (als/10) * 6;
	else
		curr_als = (als*6) / 10;

	if (curr_als == prev_als) {
		if (prev_als != 65535) {
			curr_als++;
		} else {
			curr_als--;
		}
	}

	prev_als = curr_als;
	return curr_als;
#endif
/*
	int idx;
	int invalid = 0;
	for(idx = 0; idx < obj->als_level_num; idx++)
	{
		if(als < obj->hw->als_level[idx])
		{
			break;
		}
	}

	if(idx >= obj->als_value_num)
	{
		APS_ERR("exceed range\n");
		idx = obj->als_value_num - 1;
	}

	if(1 == atomic_read(&obj->als_deb_on))
	{
		unsigned long endt = atomic_read(&obj->als_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->als_deb_on, 0);
		}

		if(1 == atomic_read(&obj->als_deb_on))
		{
			invalid = 1;
		}
	}

	if(!invalid)
	{
		APS_DBG("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);
		return obj->hw->als_value[idx];
	}
	else
	{
		APS_ERR("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);
		return -1;
	}
*/
}

static void epl6804_dumpReg(struct i2c_client *client)
{
	APS_LOG("chip id REG 0x00 value = %8x\n", i2c_smbus_read_byte_data(client, 0x00));
	APS_LOG("chip id REG 0x01 value = %8x\n", i2c_smbus_read_byte_data(client, 0x08));
	APS_LOG("chip id REG 0x02 value = %8x\n", i2c_smbus_read_byte_data(client, 0x10));
	APS_LOG("chip id REG 0x03 value = %8x\n", i2c_smbus_read_byte_data(client, 0x18));
	APS_LOG("chip id REG 0x04 value = %8x\n", i2c_smbus_read_byte_data(client, 0x20));
	APS_LOG("chip id REG 0x05 value = %8x\n", i2c_smbus_read_byte_data(client, 0x28));
	APS_LOG("chip id REG 0x06 value = %8x\n", i2c_smbus_read_byte_data(client, 0x30));
	APS_LOG("chip id REG 0x07 value = %8x\n", i2c_smbus_read_byte_data(client, 0x38));
	APS_LOG("chip id REG 0x09 value = %8x\n", i2c_smbus_read_byte_data(client, 0x48));
	APS_LOG("chip id REG 0x0D value = %8x\n", i2c_smbus_read_byte_data(client, 0x68));
	APS_LOG("chip id REG 0x0E value = %8x\n", i2c_smbus_read_byte_data(client, 0x70));
	APS_LOG("chip id REG 0x0F value = %8x\n", i2c_smbus_read_byte_data(client, 0x71));
	APS_LOG("chip id REG 0x10 value = %8x\n", i2c_smbus_read_byte_data(client, 0x80));
	APS_LOG("chip id REG 0x11 value = %8x\n", i2c_smbus_read_byte_data(client, 0x88));
	APS_LOG("chip id REG 0x13 value = %8x\n", i2c_smbus_read_byte_data(client, 0x98));
}

int hw8k_init_device(struct i2c_client *client)
{
	APS_DBG("[ALS_PS][EPL6804] hw8k_init_device.........\r\n");

	epl6804_i2c_client=client;
	APS_DBG("[ALS_PS][EPL6804] %s done \n",__func__);
	return 0;
}

int epl6804_get_addr(struct alsps_hw *hw, struct epl6804_i2c_addr *addr)
{
	APS_DBG("[ALS_PS][EPL6804] %s entry \n",__func__);
	if(!hw || !addr)
	{
		return -EFAULT;
	}
	addr->write_addr= hw->i2c_addr[0];
	APS_DBG("[ALS_PS][EPL6804] %s done \n",__func__);
	return 0;
}

static void epl6804_power(struct alsps_hw *hw, unsigned int on) 
{
	static unsigned int power_on = 0;
	APS_DBG("[ALS_PS][EPL6804] %s entry \n",__func__);

	APS_LOG("power %s\n", on ? "on" : "off");

	if(power_on == on)
	{
		APS_LOG("ignore power control: %d\n", on);
	}
	else if(on)
	{
		if(!hwPowerOn(hw->power_id, hw->power_vol, "epl6804")) 
		{
			APS_ERR("power on fails!!\n");
		}
	}
	else
	{
		if(!hwPowerDown(hw->power_id, "epl6804")) 
		{
			APS_ERR("power off fail!!\n");   
		}
	}
	APS_DBG("[ALS_PS][EPL6804] %s done \n",__func__);
	power_on = on;
}

int epl6804_read_als(struct i2c_client *client, u16 *data)
{
	struct elan_epl_data *obj = i2c_get_clientdata(client);

	APS_DBG("[ALS_PS][EPL6804] %s entry \n",__func__);

	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

	elan_epl6804_I2C_Write(obj->client,REG_14,R_TWO_BYTE,0x01,0x00);
	elan_epl6804_I2C_Read(obj->client);
	gRawData.als_ch0_raw = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];

	elan_epl6804_I2C_Write(obj->client,REG_16,R_TWO_BYTE,0x01,0x00);
	elan_epl6804_I2C_Read(obj->client);
	gRawData.als_ch1_raw = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];
	*data =  gRawData.als_ch1_raw;
//	APS_LOG("[epl6804_read_als] als raw data = %d\n", gRawData.als_ch1_raw);

	if(atomic_read(&obj->trace) & CMC_TRC_ALS_DATA)
	{
		APS_LOG("read als raw data = %d\n", gRawData.als_ch1_raw);
	}
	APS_DBG("[ALS_PS][EPL6804] %s done \n",__func__);
	return 0;
}

#if 0
void epl6804_eint_func(void)
{
	struct elan_epl_data *obj = epl6804_obj;
	
	APS_LOG(" interrupt fuc\n");

	if(!obj)
	{
		return;
	}

	schedule_delayed_work(&obj->eint_work, 0);
}
#endif // not used

static void epl6804_irq_work(struct work_struct *work)
{
	struct elan_epl_data *obj =
		(struct elan_epl_data *)container_of(work, struct elan_epl_data, irq_work);
	//int err;
	hwm_sensor_data sensor_data;
	
	int res = 0;

	APS_DBG("[ALS_PS][EPL6804] %s enrty \n",__func__);

	disable_irq(alsps_irq);

	epl6804_read_als(obj->client, &obj->als);

	APS_LOG("epl6804 int top half time = %lld\n", int_top_time);

	if (obj->als > 40000) {
		APS_LOG("APDS9930_irq_work ALS too large may under lighting als_ch0=%d!\n",obj->als);
			goto EXIT_ERR;
	}
	sensor_data.values[0] = epl6804_get_als_value(obj, obj->als);
	sensor_data.value_divide = 1;
	sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
#if 0
	err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data);
	if (err)
		APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", err);	
#endif
	enable_irq(alsps_irq);

	APS_DBG("[ALS_PS][EPL6804] %s done \n",__func__);
	return;

	EXIT_ERR:	
	enable_irq(alsps_irq);
	APS_ERR("i2c_transfer error = %d\n", res);
	return;

}

static irqreturn_t alsps_interrupt_handler(int irq, void *dev_id)
{
	struct elan_epl_data *obj=0;

	APS_DBG("[ALS_PS][EPL6804] %s enrty \n",__func__);
	if (!obj)
		return IRQ_HANDLED;

	int_top_time = sched_clock();
	schedule_work(&obj->irq_work);

	APS_DBG("[ALS_PS][EPL6804] %s done \n",__func__);
	return IRQ_HANDLED;
}


int epl6804_irq_registration(struct i2c_client *client)
{
	int ret = -1;
	struct elan_epl_data * g_epl6804_ptr;
	struct elan_epl_data *obj = i2c_get_clientdata(client);        

	g_epl6804_ptr = obj;

	APS_DBG("[ALS_PS][EPL6804] %s entry \n",__func__);
	
	gpio_direction_input(alsps_int_gpio_number);

	ret = request_irq(alsps_irq, alsps_interrupt_handler, IRQF_TRIGGER_FALLING, "epl6804", NULL);

	if (ret > 0) {
		APS_DBG("[ALS_PS][EPL6804] alsps request_irq IRQ LINE NOT AVAILABLE!.");
		return ret;
	}

	APS_DBG("[ALS_PS][EPL6804] %s done \n",__func__);
	return ret;
}


int epl6804_setup_eint(struct i2c_client *client)
{
	struct elan_epl_data *obj = i2c_get_clientdata(client);		

	APS_LOG("epl6804_setup_eint\n");
	

	epl6804_obj = obj;

	return 0;
}

static int epl6804_init_client(struct i2c_client *client)
{
	//struct elan_epl_data *obj = i2c_get_clientdata(client);
	int err=0;

	APS_DBG("[ALS_PS][EPL6804] %s entry \n",__func__);

	APS_DBG("[ALS_PS][EPL6804] I2C Addr==[0x%x],line=%d\n",epl6804_i2c_client->addr,__LINE__);


	if((err = hw8k_init_device(client)) != 0)
	{
		APS_DBG("[ALS_PS][EPL6804] init dev: %d\n", err);
		return err;
	}

	APS_DBG("[ALS_PS][EPL6804] %s done \n",__func__);
	return err;
}

/*
static ssize_t epl6804_show_reg(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = epl6804_obj->client;
	ssize_t len = 0;

	if(!epl6804_obj)
	{
		APS_ERR("epl6804_obj is null!!\n");
		return 0;
	}

	len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x00 value = %8x\n", i2c_smbus_read_byte_data(client, 0x00));
	len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x01 value = %8x\n", i2c_smbus_read_byte_data(client, 0x08));
	len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x02 value = %8x\n", i2c_smbus_read_byte_data(client, 0x10));
	len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x03 value = %8x\n", i2c_smbus_read_byte_data(client, 0x18));
	len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x04 value = %8x\n", i2c_smbus_read_byte_data(client, 0x20));
	len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x05 value = %8x\n", i2c_smbus_read_byte_data(client, 0x28));
	len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x06 value = %8x\n", i2c_smbus_read_byte_data(client, 0x30));
	len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x07 value = %8x\n", i2c_smbus_read_byte_data(client, 0x38));
	len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x09 value = %8x\n", i2c_smbus_read_byte_data(client, 0x48));
	len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0D value = %8x\n", i2c_smbus_read_byte_data(client, 0x68));
	len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0E value = %8x\n", i2c_smbus_read_byte_data(client, 0x70));
	len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0F value = %8x\n", i2c_smbus_read_byte_data(client, 0x71));
	len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x10 value = %8x\n", i2c_smbus_read_byte_data(client, 0x80));
	len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x11 value = %8x\n", i2c_smbus_read_byte_data(client, 0x88));
	len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x13 value = %8x\n", i2c_smbus_read_byte_data(client, 0x98));
	
	return len;
}
*/

/*
static ssize_t epl6804_show_status(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	struct elan_epl_data *epld = epl6804_obj;

	if(!epl6804_obj)
	{
		APS_ERR("epl6804_obj is null!!\n");
		return 0;
	}
	elan_epl6804_I2C_Write(epld->client,REG_7,W_SINGLE_BYTE,0x02,EPL_DATA_LOCK);  // 0x05

	elan_epl6804_I2C_Write(epld->client,REG_16,R_TWO_BYTE,0x01,0x00);
	elan_epl6804_I2C_Read(epld->client);
	gRawData.als_status = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];
	APS_LOG("ch1 raw_data = %d\n", gRawData.als_status);

	elan_epl6804_I2C_Write(epld->client,REG_7,W_SINGLE_BYTE,0x02,EPL_DATA_UNLOCK);  // 0x04
	len += snprintf(buf+len, PAGE_SIZE-len, "ch1 raw is %d\n",gRawData.als_status);
	return len;
}
*/

//static DRIVER_ATTR(status, S_IWUSR | S_IRUGO, epl6804_show_status, NULL);
//static DRIVER_ATTR(reg,	S_IWUSR | S_IRUGO, epl6804_show_reg, NULL);

/*
static struct attribute *ets_attributes[] = 
{
	&driver_attr_status.attr,
	&driver_attr_reg.attr,
};

static int epl6804_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(ets_attributes)/sizeof(ets_attributes[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, ets_attributes[idx])))
		{			
			APS_DBG("driver_create_file (%s) = %d\n", ets_attributes[idx]->name, err);
			break;
		}
	}	
	return err;
}

static int epl6804_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(ets_attributes)/sizeof(ets_attributes[0]));

	if (!driver)
	return -EINVAL;

	for (idx = 0; idx < num; idx++) 
	{
		driver_remove_file(driver, ets_attributes[idx]);
	}

	return err;
}
*/
/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static int epl6804_open(struct inode *inode, struct file *file)
{
	file->private_data = epl6804_i2c_client;

	APS_FUN();

	if (!file->private_data)
	{
		APS_DBG("[ALS_PS][EPL6804]null pointer!!\n");
		return -EINVAL;
	}

	return nonseekable_open(inode, file);
}

static int epl6804_release(struct inode *inode, struct file *file)
{
	APS_FUN();
	file->private_data = NULL;
	return 0;
}

static long epl6804_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct elan_epl_data *obj = i2c_get_clientdata(client);
	int err = 0;
	void __user *ptr = (void __user*) arg;
	int dat;
	uint32_t enable;

	APS_DBG("[ALS_PS][EPL6804] %s entry \n",__func__);

	switch (cmd)
	{
/*
		case ALSPS_SET_PS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable))) {
				err = -EFAULT;
				goto err_out;
			}
			if(enable) {
				if(isInterrupt) {
					if((err = elan_epl6881_psensor_enable(obj, 1)) != 0) {
									APS_ERR("enable ps fail: %d\n", err);
									return -1;
								}
				}
				set_bit(CMC_BIT_PS, &obj->enable);
			} else {
				if(isInterrupt) {
					if((err = elan_epl6881_psensor_enable(obj, 0)) != 0) {
						APS_ERR("disable ps fail: %d\n", err);
						return -1;
					}
				}
				clear_bit(CMC_BIT_PS, &obj->enable);
			}
			break;

		case ALSPS_GET_PS_MODE:
			enable=test_bit(CMC_BIT_PS, &obj->enable);
			if(copy_to_user(ptr, &enable, sizeof(enable))) {
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_DATA:
			if((err = elan_epl6881_psensor_enable(obj, 1)) !=0 ) {
				APS_ERR("enable ps fail: %d\n", err);
				return -1;
			}
			epl6881_read_ps(obj->client, &obj->ps);
			dat = gRawData.ps_state;
			APS_LOG("ioctl ps state value = %d \n", dat);
			if(copy_to_user(ptr, &dat, sizeof(dat))) {
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_RAW_DATA:
			if((err = elan_epl6881_psensor_enable(obj, 1)) !=0 ) {
				APS_ERR("enable ps fail: %d\n", err);
				return -1;
			}
			epl6881_read_ps(obj->client, &obj->ps);
			dat = gRawData.ps_raw;
			APS_LOG("ioctl ps raw value = %d \n", dat);
			if(copy_to_user(ptr, &dat, sizeof(dat))) {
				err = -EFAULT;
				goto err_out;
			}
			break;
*/
		//case ALSPS_SET_PS_MODE:
		case ALSPS_SET_ALS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable))) {
				err = -EFAULT;
				goto err_out;
			}
			if(enable) {
				set_bit(CMC_BIT_ALS, &obj->enable);
			} else {
				clear_bit(CMC_BIT_ALS, &obj->enable);
			}
			break;

		//case ALSPS_GET_PS_MODE:
		case ALSPS_GET_ALS_MODE:
			enable = test_bit(CMC_BIT_ALS, &obj->enable);
			if(copy_to_user(ptr, &enable, sizeof(enable))) {
				err = -EFAULT;
				goto err_out;
			}
			break;

		//case ALSPS_GET_PS_DATA:
		case ALSPS_GET_ALS_DATA:
			if((err = elan_epl6804_lsensor_enable(obj, 1)) != 0) {
				APS_ERR("disable als fail: %d\n", err);
				return -1;
			}
			epl6804_read_als(obj->client, &obj->als);
			dat = epl6804_get_als_value(obj, obj->als);
			APS_LOG("ioctl get als data = %d\n", dat);
/*
			if(obj->enable_pflag && isInterrupt) {
				if((err = elan_epl6881_psensor_enable(obj, 1)) != 0) {
					APS_ERR("disable ps fail: %d\n", err);
					return -1;
				}
			}
*/
			if(copy_to_user(ptr, &dat, sizeof(dat))) {
				err = -EFAULT;
				goto err_out;
			}
			break;

		//case ALSPS_GET_PS_RAW_DATA:
		case ALSPS_GET_ALS_RAW_DATA:
			if((err = elan_epl6804_lsensor_enable(obj, 1)) != 0) {
				APS_ERR("disable als fail: %d\n", err);
				return -1;
			}
			epl6804_read_als(obj->client, &obj->als);
			dat = obj->als;
			APS_DBG("ioctl get als raw data = %d\n", dat);
/*
			if(obj->enable_pflag && isInterrupt) {
				if((err = elan_epl6881_psensor_enable(obj, 1))!=0) {
					APS_ERR("disable ps fail: %d\n", err);
					return -1;
				}
			}
*/
			if(copy_to_user(ptr, &dat, sizeof(dat))) {
				err = -EFAULT;
				goto err_out;
			}
			break;

		default:
			APS_ERR("%s not supported = 0x%04x\n", __FUNCTION__, cmd);
			err = -ENOIOCTLCMD;
			break;
	}
	APS_DBG("[ALS_PS][EPL6804] %s done \n",__func__);

	err_out:
		return err;
}

static struct file_operations epl6804_fops =
{
	.owner = THIS_MODULE,
	.open = epl6804_open,
	.release = epl6804_release,
	.unlocked_ioctl = epl6804_unlocked_ioctl,
};

static struct miscdevice epl6804_device =
{
	.minor = MISC_DYNAMIC_MINOR,
	.name = "epl6804",
	.fops = &epl6804_fops,
};

static int epl6804_i2c_suspend(struct i2c_client *client, pm_message_t msg)
{
	struct elan_epl_data *obj = i2c_get_clientdata(client);	
	int err;
	APS_FUN();
	APS_DBG("[ALS_PS][EPL6804] %s entry \n",__func__);

	if(msg.event == PM_EVENT_SUSPEND)
	{
		if(!obj)
		{
			APS_DBG("[ALS_PS][EPL6804] null pointer!!\n");
			return -EINVAL;
		}

		atomic_set(&obj->als_suspend, 1);
		if((err = elan_epl6804_lsensor_enable(obj, 0))!=0)
		{
			APS_DBG("[ALS_PS][EPL6804] disable als: %d\n", err);
			return err;
		}

		//mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
		//mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
		//mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, GPIO_PULL_ENABLE);
		//mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_DOWN);

		gpio_direction_input(alsps_int_gpio_number);
		gpio_set_value(alsps_int_gpio_number,0);

		epl6804_power(obj->hw, 0);
	}
	APS_DBG("[ALS_PS][EPL6804] %s done \n",__func__);
	return 0;
}

static int epl6804_i2c_resume(struct i2c_client *client)
{

	struct elan_epl_data *obj = i2c_get_clientdata(client);
	int err;
	APS_FUN();

	APS_DBG("[ALS_PS][EPL6804] %s entry \n",__func__);
	if(!obj)
	{
		APS_DBG("[ALS_PS][EPL6804] null pointer!!\n");
		return -EINVAL;
	}

	epl6804_power(obj->hw, 1);

	//mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
	//mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
	//mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, GPIO_PULL_ENABLE);
	//mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP);

	gpio_direction_input(alsps_int_gpio_number);
	gpio_set_value(alsps_int_gpio_number,1);

	msleep(50);

	if((err = epl6804_init_client(client)))
	{
		APS_DBG("[ALS_PS][EPL6804] initialize client fail!!\n");
		return err;
	}

	atomic_set(&obj->als_suspend, 0);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
		if((err = elan_epl6804_lsensor_enable(obj, 1))!=0)
		{
			APS_DBG("[ALS_PS][EPL6804] enable als fail: %d\n", err);
		}
	}
	APS_DBG("[ALS_PS][EPL6804] %s done \n",__func__);
	return 0;
}

int epl6804_als_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* sensor_data;
	struct elan_epl_data *obj = (struct elan_epl_data *)self;

	APS_FUN();
	APS_DBG("[ALS_PS][EPL6804]epl6804_als_operate command = %x\n",command);

	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_DBG("[ALS_PS][EPL6804]Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_DBG("[ALS_PS][EPL6804]Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;

				if(value)
				{
					if((err = elan_epl6804_lsensor_enable(epl6804_obj, 1))!=0)
					{
						APS_DBG("[ALS_PS][EPL6804]enable als fail: %d\n", err);
						return -1;
					}
					set_bit(CMC_BIT_ALS, &obj->enable);
				}
				else
				{
					if((err = elan_epl6804_lsensor_enable(epl6804_obj, 0))!=0)
					{
						APS_DBG("[ALS_PS][EPL6804]disable als fail: %d\n", err);						
						return -1;
					}

					clear_bit(CMC_BIT_ALS, &obj->enable);
				}
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_DBG("[ALS_PS][EPL6804]get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				if((err = elan_epl6804_lsensor_enable(epl6804_obj, 1))!=0)
				{
					APS_DBG("[ALS_PS][EPL6804]disable als fail: %d\n", err);
					return -1;
				}
				epl6804_read_als(epl6804_obj->client, &epl6804_obj->als);

				
				sensor_data = (hwm_sensor_data *)buff_out;
				//sensor_data->values[0] = epl6804_obj->als;
				sensor_data->values[0] = epl6804_get_als_value(epl6804_obj, epl6804_obj->als);
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
				APS_DBG("[ALS_PS][EPL6804]get als data->values[0] = %d\n", sensor_data->values[0]);
			}
			break;

		default:
			APS_DBG("[ALS_PS][EPL6804]light sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}

	return err;
}

static int epl6804_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct elan_epl_data *obj;
	struct hwmsen_object obj_als;
	int err = 0;
	APS_FUN();

	APS_DBG("[ALS_PS][EPL6804] %s entry\n", __func__);

	epl6804_dumpReg(client);

	of_get_epl6804_platform_data(&client->dev);
	err = gpio_request_one(alsps_int_gpio_number, GPIOF_IN,
				 "alsps_int");

	if (err < 0) {
		APS_DBG("[ALS_PS][EPL6804]Unable to request gpio int_pin,alsps_int_gpio_number = %d\n",alsps_int_gpio_number);
		return -1;
	}

	APS_DBG("[ALS_PS][EPL6804] %s i2c slave addr = 0x%x, irq_num = %d \n", __func__, client->addr, alsps_irq);

/*
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		APS_DBG("[ELAN epl6804 error] i2c func error\n");
		goto exit;
	}
*/

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}

	memset(obj, 0, sizeof(*obj));

	epl6804_obj = obj;
	obj->hw = get_cust_alsps_hw();

	epl6804_get_addr(obj->hw, &obj->addr);

	epl6804_obj->als_level_num = sizeof(epl6804_obj->hw->als_level)/sizeof(epl6804_obj->hw->als_level[0]);
	epl6804_obj->als_value_num = sizeof(epl6804_obj->hw->als_value)/sizeof(epl6804_obj->hw->als_value[0]);
	BUG_ON(sizeof(epl6804_obj->als_level) != sizeof(epl6804_obj->hw->als_level));
	memcpy(epl6804_obj->als_level, epl6804_obj->hw->als_level, sizeof(epl6804_obj->als_level));
	BUG_ON(sizeof(epl6804_obj->als_value) != sizeof(epl6804_obj->hw->als_value));
	memcpy(epl6804_obj->als_value, epl6804_obj->hw->als_value, sizeof(epl6804_obj->als_value));

	INIT_WORK(&obj->irq_work, epl6804_irq_work);

	obj->client = client;

	i2c_set_clientdata(client, obj);

	obj->lux_per_count = LUX_PER_COUNT;
	obj->als_enable = 0;
	obj->enable = 0;
	obj->pending_intr = 0;
	obj->als_thd_level = 2;

	atomic_set(&obj->i2c_retry, 3);

	epl6804_i2c_client = client;

	//control initial procedure 
	err = elan_epl6804_I2C_Write(client,REG_0,W_SINGLE_BYTE,0x02,0x62);
	if(err < 0)
		APS_DBG("[ALS_PS][EPL6804] write reg_0 failed\n");
	err = elan_epl6804_I2C_Write(client,REG_1,W_SINGLE_BYTE,0x02,0x32);    
	// Disable interrupt mode, set Reg_9 (INTTY[1:0]) to 0x02
	err = elan_epl6804_I2C_Write(client,REG_9,W_SINGLE_BYTE,0x02,0x02);  	
	err = elan_epl6804_I2C_Write(client,REG_7,W_SINGLE_BYTE,0X02,EPL_C_RESET);  
	err = elan_epl6804_I2C_Write(client,REG_7,W_SINGLE_BYTE,0x02,EPL_C_START_RUN);  

	if((err = epl6804_init_client(client)))
	{
		goto exit_init_failed;
	}

	err = epl6804_irq_registration(client);
	if (err != 0) {
		APS_DBG("[ALS_PS][EPL6804] %s registration failed: %d\n", __func__, err);
		return err;
	}

	if((err = misc_register(&epl6804_device)))
	{
		APS_ERR("[ALS_PS][EPL6804]epl6804_device register failed\n");
		goto exit_misc_device_register_failed;
	}
/*
	if((err = epl6804_create_attr(&epl6804_alsps_driver.id_table)))
	{
		APS_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
*/
	obj_als.self = epl6804_obj;
	obj_als.polling = 1;
	obj_als.sensor_operate = epl6804_als_operate;

	if((err = hwmsen_attach(ID_LIGHT, &obj_als)))
	{
		APS_DBG("[ALS_PS][EPL6804]attach fail = %d\n", err);
		goto exit_create_attr_failed;
	}

	APS_DBG("[ALS_PS][EPL6804] %s: done\n", __func__);
	return 0;

exit_create_attr_failed:
	misc_deregister(&epl6804_device);
	APS_DBG("[ALS_PS][EPL6804]%s: exit_create_attr_failed err = %d\n", __func__, err);
exit_misc_device_register_failed:
exit_init_failed:
	kfree(obj);
	APS_DBG("[ALS_PS][EPL6804]%s: exit_init_failed err = %d\n", __func__, err);
exit:
	gpio_free(alsps_int_gpio_number);
	epl6804_i2c_client = NULL;
	APS_DBG("[ALS_PS][EPL6804]%s: exit err = %d\n", __func__, err);
	return err;
}

static int epl6804_i2c_remove(struct i2c_client *client)
{
	int err;
/*
	if((err = epl6804_delete_attr(&epl6804_i2c_driver.driver)))
	{
		APS_ERR("epl6804_delete_attr fail: %d\n", err);
	} 
*/
	if((err = misc_deregister(&epl6804_device)))
	{
		APS_ERR("misc_deregister fail: %d\n", err);	
	}
	
	gpio_free(alsps_int_gpio_number);
	epl6804_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	return 0;
}

static int epl6804_probe(struct platform_device *pdev) 
{
	struct alsps_hw *hw = get_cust_alsps_hw();

	APS_DBG("[ALS_PS][EPL6804] %s entry \n",__func__);

	epl6804_power(hw, 1);

	alsps_irq = platform_get_irq(pdev, 0);

	APS_DBG("[ALS_PS][EPL6804] %s alsps_irq = %d",__func__, alsps_irq);

	if (alsps_irq < 0) {
		APS_DBG("[ALS_PS][EPL6804] alsps request_irq IRQ LINE NOT AVAILABLE!.");
		return -1;
	}

	if(i2c_add_driver(&epl6804_i2c_driver))
	{
		APS_DBG("[ALS_PS][EPL6804] add driver error\n");
		return -1;
	}
	APS_DBG("[ALS_PS][EPL6804] %s done \n",__func__);

	return 0;
}

static int epl6804_remove(struct platform_device *pdev)
{
	struct alsps_hw *hw = get_cust_alsps_hw();
	APS_FUN();
	epl6804_power(hw, 0);

	APS_DBG("[ALS_PS][EPL6804]epl6804 remove \n");
	i2c_del_driver(&epl6804_i2c_driver);
	return 0;
}

static struct platform_driver epl6804_alsps_driver = {
	.probe	 = epl6804_probe,
	.remove	 = epl6804_remove,
	.driver	 = {
		.name  = "epl6804",
		.of_match_table = alsps_of_match,
	}
};

static int of_get_epl6804_platform_data(struct device *dev)
{
	struct device_node *devnode = NULL;
	APS_DBG("[ALS_PS][EPL6804] %s entry \n",__func__);

	devnode = of_find_compatible_node(NULL, NULL, "ambient,epl6804");
	if (devnode) {
		alsps_int_gpio_number = of_get_named_gpio(devnode, "int-gpio", 0);
		APS_DBG("[ALS_PS][EPL6804] %s,alsps_int_gpio_number %d\n",__func__, alsps_int_gpio_number);
	}
	APS_DBG("[ALS_PS][EPL6804] %s done \n",__func__);
	return 0;
}


static int __init epl6804_init(void)
{	
	struct alsps_hw *hw = get_cust_alsps_hw();
	struct device_node *devnode = NULL;

	APS_FUN();

	APS_DBG("[ALS_PS][EPL6804] %s entry \n",__func__);
	APS_DBG("[ALS_PS][EPL6804] %s i2c_number=%d \n",__func__,hw->i2c_num);

	i2c_register_board_info(hw->i2c_num , &i2c_epl6804, 1);

	if(platform_driver_register(&epl6804_alsps_driver))
	{
		APS_DBG("[ALS_PS][EPL6804] %s failed to register driver\n",__func__);
		return -ENODEV;
	}

	devnode = of_find_compatible_node(NULL, NULL, "ambient,epl6804");
	if (devnode) {
		alsps_int_gpio_number = of_get_named_gpio(devnode, "int-gpio", 0);
		APS_DBG("[ALS_PS][EPL6804] %s,alsps_int_gpio_number %d\n",__func__, alsps_int_gpio_number);
	}

	APS_DBG("[ALS_PS][EPL6804] %s done \n",__func__);

	return 0;
}

static void __exit epl6804_exit(void)
{
	APS_FUN();
	platform_driver_unregister(&epl6804_alsps_driver);
}

module_init(epl6804_init);
module_exit(epl6804_exit);

MODULE_DESCRIPTION("epl6804 driver");
MODULE_LICENSE("GPL");
