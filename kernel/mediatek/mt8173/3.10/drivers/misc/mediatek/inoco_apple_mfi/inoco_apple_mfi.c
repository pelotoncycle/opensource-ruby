/*****************************************************************************
* Filename: inoco_apple_mfi.c
* Author: Innocomm
****************************************************************************/
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/list.h>

#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/device.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#endif
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <asm/siginfo.h>
#include <linux/rcupdate.h>
#include <linux/sched.h>
#include <linux/signal.h>
#include <linux/wakelock.h>
#include <linux/dma-mapping.h>
#include <linux/kfifo.h>
#include <linux/poll.h>
#include <linux/kobject.h>

#include "inoco_apple_mfi.h"

static DEFINE_MUTEX(inoco_apple_mfi_i2c_access);

static struct device *apple_mfi_dev;
static struct _inoco_apple_mfi_i2c *p_inoco_apple_mfi_i2c_dev = NULL;
static struct inoco_apple_mfi_char_dev *p_char_dev = NULL;	// allocated in init_module
static struct mfi_register_table *mfi_reg_tbl;
static struct i2c_client *inoco_apple_mfi_i2c_client;
static atomic_t inoco_apple_mfi_char_available = ATOMIC_INIT(1);
//static atomic_t wait_command_ack = ATOMIC_INIT(0);
int gpio_apple_mfi_rst_en;
char g_buf_data[MFI_STRING_SIZE] = {0};

static int is_mifi_debug = 0; // 1 is enable debug log
#define MFI_LOG_D(x...) if (is_mifi_debug) printk("[Apple-MFi] " x)
#define MFI_LOG_I(x...) printk("[Apple-MFi] " x)
#define MFI_LOG_E(x...) printk("[Apple-MFi][Error] " x)

extern struct kset *devices_kset;

struct _inoco_apple_mfi_i2c {
	struct mutex mutex_wq;
	struct i2c_client *client;
	unsigned char work_state;
	unsigned int ioctl_cmd;
	int interrupt_gpio;
};

struct inoco_apple_mfi_char_dev
{
	int OpenCnts;
	struct kfifo DataKFiFo;
	unsigned char *pFiFoBuf;
	spinlock_t FiFoLock;
	struct semaphore sem;
	wait_queue_head_t fifo_inq;
};

struct mfi_register_list mfi_reg_list[MAX_MFI_REG_COUNT] = 
{
	{{"device_version", 0x00, 1}}, // 0
	{{"firmware_version", 0x01, 1}},
	{{"authentication_protocal_major_version", 0x02, 1}},
	{{"authentication_protocal_minor_version", 0x03, 1}},
	{{"device_id", 0x04, 4}},
	{{"error_code", 0x05, 1}},
	{{"authentication_control_and_status", 0x10, 1}},
	{{"challenge_response_data_length", 0x11, 2}},
	{{"challenge_response_data", 0x12, 128}},
	{{"challenge_data_length", 0x20, 2}},
	{{"challenge_data", 0x21, 128}}, // 10
	{{"accessory_certificate_data_length", 0x30, 2}},
	{{"accessory_certificate_data_part_01", 0x31, 128}},
	{{"accessory_certificate_data_part_02", 0x32, 128}},
	{{"accessory_certificate_data_part_03", 0x33, 128}},
	{{"accessory_certificate_data_part_04", 0x34, 128}},
	{{"accessory_certificate_data_part_05", 0x35, 128}},
	{{"accessory_certificate_data_part_06", 0x36, 128}},
	{{"accessory_certificate_data_part_07", 0x37, 128}},
	{{"accessory_certificate_data_part_08", 0x38, 128}},
	{{"accessory_certificate_data_part_09", 0x39, 128}}, //20
	{{"accessory_certificate_data_part_10", 0x3A, 128}},
	{{"self_test_control_and_status", 0x40, 1}},
	{{"system_control_counter", 0x4D, 1}},
	{{"accessory_certificate_serial_number", 0x4E, 31}},
	{{"apple_device_certificate_data_length", 0x50, 2}},
	{{"apple_device_certificate_data_01", 0x51, 128}},
	{{"apple_device_certificate_data_02", 0x52, 128}},
	{{"apple_device_certificate_data_03", 0x53, 128}},
	{{"apple_device_certificate_data_04", 0x54, 128}},
	{{"apple_device_certificate_data_05", 0x55, 128}}, // 30
	{{"apple_device_certificate_data_06", 0x56, 128}},
	{{"apple_device_certificate_data_07", 0x57, 128}},
	{{"apple_device_certificate_data_08", 0x58, 128}}
};

static ssize_t show_cmd_return_value(struct device *dev, struct device_attribute *attr,
						   char *buf)
{
	MFI_LOG_D("%s enter\n", __func__);

	MFI_LOG_D("%s return value: [%s]\n", __func__, g_buf_data);

	return sprintf(buf, "%s\n", g_buf_data);
}

static ssize_t store_cmd_addr_value(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	int data0;
	u8 data1 = 0;
	int ret;
	unsigned char buf_response[MFI_RESPONSE_SIZE];
	int reg_index = 0;
	char buf_data[MFI_I2C_FIFO_SIZE];
	int i = 0;
	char *p;
	char input_str[20];
	size_t str_len;

	MFI_LOG_D("%s enter\n", __func__);

	if (mfi_reg_tbl == NULL) 
	{
		sprintf(g_buf_data, "%s", "mfi register table is NULL");
		MFI_LOG_E("%s: %s\n", __func__, g_buf_data);
		goto fail_store_cmd;
	}

	memset(g_buf_data, 0, MFI_STRING_SIZE);
	MFI_LOG_D("%s, mfi_reg_tbl addr = %p\n", __func__, mfi_reg_tbl);

	if (mfi_reg_tbl->list == NULL)
	{
		sprintf(g_buf_data, "%s", "mfi register table list is NULL");
		MFI_LOG_E("%s: %s\n", __func__, g_buf_data);
		goto fail_store_cmd;
	}

	str_len = strlen(buf);
	MFI_LOG_D("%s: input buf strlen:%lu\n", __func__, str_len);

	if ((1 == sscanf(buf, "0x%02X", &data0) || 1 == sscanf(buf, "0X%02X", &data0)) && strlen(buf) == 5) // lenght 5 is '0xXX' plus '\n'
	{
		data1 = data0;
		MFI_LOG_D("%s: addr:0x%02X, register:0x%02X\n", __func__, inoco_apple_mfi_i2c_client->addr, data1);
	}
	else
	{
		input_str[sizeof(input_str) - 1] = '\0';
		strncpy(input_str, buf, sizeof(input_str) - 1);
		str_len = strlen(input_str);

		if (str_len && input_str[str_len - 1] == '\n')
			input_str[str_len - 1] = '\0';

		sprintf(g_buf_data, "invalid content [%s], please input [0xXX]", input_str);
		MFI_LOG_D("%s: %s\n", __func__, g_buf_data);
		goto fail_store_cmd;
	}

	buf_data[0] = data1;
	MFI_LOG_D("%s, buf_data=[%d]\n", __func__, buf_data[0]);

	/* find MFi register command index for mapping mfi_reg_list */
	for (i = 0; i < MAX_MFI_REG_COUNT; i++)
	{
		if (buf_data[0] == mfi_reg_tbl->list[i].cmd.cmd_addr)
		{
			reg_index = i;
			MFI_LOG_D("%s, register index is %d\n", __func__, reg_index);
			break;
		}
		reg_index = MAX_MFI_REG_COUNT;
	}

	if (reg_index == MAX_MFI_REG_COUNT)
	{
		sprintf(g_buf_data, "register [0x%02X] not found", data1);
		MFI_LOG_D("%s: %s\n", __func__, g_buf_data);
		goto fail_store_cmd;
	}

	MFI_LOG_D("%s mfi_reg_tbl->list->cmd[%d], func=%s, cmd_addr=%x, nbyte=%d\n", __func__, reg_index, mfi_reg_tbl->list[reg_index].cmd.func, mfi_reg_tbl->list[reg_index].cmd.cmd_addr, mfi_reg_tbl->list[reg_index].cmd.nbyte);

	ret = inoco_apple_mfi_tx_data(&data1, 1);
	if (ret < 0) 
	{
		sprintf(g_buf_data, "%s", "I2C Tx error!");
		MFI_LOG_E("%s: %s\n", __func__, g_buf_data);
		goto fail_store_cmd;
	}

	memset(buf_response, 0, MFI_RESPONSE_SIZE);
	ret = inoco_apple_mfi_rx_data(buf_response, mfi_reg_tbl->list[reg_index].cmd.nbyte);
	if (ret < 0) 
	{
		sprintf(g_buf_data, "%s", "I2C Rx error!");
		MFI_LOG_E("%s: %s\n", __func__, g_buf_data);
		goto fail_store_cmd;
	}

	p = g_buf_data;
	p += sprintf(p, "%s: ", mfi_reg_tbl->list[reg_index].cmd.func);
	for (i = 0; i < mfi_reg_tbl->list[reg_index].cmd.nbyte; i++)
		p += sprintf(p, "%02X", buf_response[i]);

	MFI_LOG_D("%s, g_buf_data=[%s]\n", __func__, g_buf_data);

	return size;

fail_store_cmd:
	MFI_LOG_D("%s failed!\n", __func__);
	return size;
}

// Create apple MFi device attribute node
/*----------------------------------------------------------------------------*/
static struct device_attribute attributes[] = {
	__ATTR(cmd_addr_value, 0664, show_cmd_return_value, store_cmd_addr_value),
};
static int create_sysfs_interfaces(struct device *dev)
{
	int i;

	MFI_LOG_I("%s enter\n", __func__);

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto error_create_sysfs;
	return 0;

error_create_sysfs:
	for ( ; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	MFI_LOG_E("%s, unable to create interface\n", __func__);
	return -1;
}

static int remove_sysfs_interfaces(struct device *dev)
{
	int i;

	MFI_LOG_I("%s enter\n", __func__);

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);

	return 0;
}
/*----------------------------------------------------------------------------*/

static int inoco_apple_mfi_tx_data(uint8_t *databuf, int length)
{
	uint8_t loop_i;
	int ret = 0;

	mutex_lock(&inoco_apple_mfi_i2c_access);
	inoco_apple_mfi_i2c_client->addr = (inoco_apple_mfi_i2c_client->addr & I2C_MASK_FLAG);

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++)
	{
		if (loop_i > 0)
		{
			MFI_LOG_D("%s: retry %d ........\n", __func__, loop_i);
		}

		ret = i2c_master_send(inoco_apple_mfi_i2c_client, databuf, length);
		MFI_LOG_D("%s : i2c_master_send len=%d, ret=%d, databuf=%x, addr=0x%x\n", __func__, length, ret, databuf[0], inoco_apple_mfi_i2c_client->addr);

		if (ret != length) 
		{
			MFI_LOG_D("%s : i2c_master_send returned %d\n", __func__, ret);
		}
		else
		{
			break;
		}

		msleep(1);
	}

	if (loop_i >= I2C_RETRY_COUNT) 
	{
		MFI_LOG_E("%s:  Error: retry over %d\n", __func__, I2C_RETRY_COUNT);
		mutex_unlock(&inoco_apple_mfi_i2c_access);
		return -EIO;
	}
	else
	{
		MFI_LOG_D("%s, TX done!\n", __func__);
	}

	mutex_unlock(&inoco_apple_mfi_i2c_access);
	return ret;
}

static int inoco_apple_mfi_rx_data(char *buf, int length)
{
	int ret = 0;
	uint8_t loop_i;

	mutex_lock(&inoco_apple_mfi_i2c_access);
	inoco_apple_mfi_i2c_client->addr = (inoco_apple_mfi_i2c_client->addr & I2C_MASK_FLAG);

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) 
	{
		if (loop_i > 0) 
		{
			MFI_LOG_D("%s: retry %d ........\n", __func__, loop_i);
		}

		ret = i2c_master_recv(inoco_apple_mfi_i2c_client, buf, length);

		if (ret != length) 
		{
			MFI_LOG_D("%s : i2c_master_recv returned %d\n", __func__, ret);
		}
		else 
		{
			break;
		}

		msleep(1);
	}

	if (loop_i >= I2C_RETRY_COUNT) 
	{
		MFI_LOG_E("%s:  Error: retry over %d\n", __func__, I2C_RETRY_COUNT);
		mutex_unlock(&inoco_apple_mfi_i2c_access);
		return -EIO;
	} 
	else
	{
		MFI_LOG_D("%s, RX done!\n", __func__);
	}

	mutex_unlock(&inoco_apple_mfi_i2c_access);
	return ret;
}

void inoco_apple_mfi_request_gpio_control(struct device *dev)
{
	int ret = 0;

	MFI_LOG_I("%s enter\n", __func__);

	gpio_apple_mfi_rst_en = of_get_named_gpio(dev->of_node, "inoco,apple_mfi-rst", 0);

	if (!gpio_is_valid(gpio_apple_mfi_rst_en)) 
	{
		MFI_LOG_E( "%s, gpio_apple_mfi_rst_en is not valid!!\n", __func__);
		gpio_apple_mfi_rst_en = -ENODEV;
	}
	else
	{
		MFI_LOG_D("%s, gpio_apple_mfi_rst_en = 0x%x\n", __func__, gpio_apple_mfi_rst_en);

		ret = devm_gpio_request(dev, gpio_apple_mfi_rst_en, "GPIO_APPLE_MFI_RST_EN");
		gpio_direction_output(gpio_apple_mfi_rst_en, 1);
		msleep(5);
		MFI_LOG_D("%s gpio_get_value=%d\n", __func__, gpio_get_value(gpio_apple_mfi_rst_en));

		if (ret < 0) 
		{
			MFI_LOG_E( "%s, request gpio_apple_mfi_rst_en = 0x%x failed, ret=%d\n", __func__, gpio_apple_mfi_rst_en, ret);
			gpio_apple_mfi_rst_en = -ENODEV;
		}
	}
}

void inoco_apple_mfi_dev_enable(void)
{
	MFI_LOG_D("%s enter\n", __func__);

	gpio_set_value(gpio_apple_mfi_rst_en, 1);
	msleep(5);
	MFI_LOG_D("%s power on, gpio_apple_mfi_rst_en value = %d\n", __func__, gpio_get_value(gpio_apple_mfi_rst_en));
	gpio_set_value(gpio_apple_mfi_rst_en, 0);
	msleep(15);
	gpio_set_value(gpio_apple_mfi_rst_en, 1);
	msleep(10);
	MFI_LOG_D("%s gpio_set_value, gpio_apple_mfi_rst_en value = %d\n", __func__, gpio_get_value(gpio_apple_mfi_rst_en));
}

static int inoco_apple_mfi_probe(struct device *dev)
{
	MFI_LOG_I("%s enter\n", __func__);

	apple_mfi_dev = dev;
	gpio_apple_mfi_rst_en = -ENODEV;

	inoco_apple_mfi_request_gpio_control(dev);
	inoco_apple_mfi_dev_enable();

	MFI_LOG_D("%s mfi_reg_tbl, kzalloc=%p\n", __func__, mfi_reg_tbl);
	mfi_reg_tbl->list = mfi_reg_list;

	msleep(5);
	MFI_LOG_D("%s mfi_reg_tbl->list->cmd[0], func=%s, cmd_addr=%x, nbyte=%d\n", __func__, mfi_reg_tbl->list[0].cmd.func, mfi_reg_tbl->list[0].cmd.cmd_addr, mfi_reg_tbl->list[0].cmd.nbyte);
	MFI_LOG_D("%s mfi_reg_tbl->list->cmd[1], func=%s, cmd_addr=%x, nbyte=%d\n", __func__, mfi_reg_tbl->list[1].cmd.func, mfi_reg_tbl->list[1].cmd.cmd_addr, mfi_reg_tbl->list[1].cmd.nbyte);
	MFI_LOG_D("%s mfi_reg_tbl->list->cmd[8], func=%s, cmd_addr=%x, nbyte=%d\n", __func__, mfi_reg_tbl->list[8].cmd.func, mfi_reg_tbl->list[8].cmd.cmd_addr, mfi_reg_tbl->list[8].cmd.nbyte);
	MFI_LOG_D("%s mfi_reg_tbl->list->cmd[9], func=%s, cmd_addr=%x, nbyte=%d\n", __func__, mfi_reg_tbl->list[9].cmd.func, mfi_reg_tbl->list[9].cmd.cmd_addr, mfi_reg_tbl->list[9].cmd.nbyte);

	return 0;
}

static void inoco_apple_mfi_shutdown(struct device *dev)
{
	MFI_LOG_I("%s enter\n", __func__);

	if (gpio_apple_mfi_rst_en != -ENODEV)
	{
		devm_gpio_free(dev, gpio_apple_mfi_rst_en);
	}

	apple_mfi_dev = NULL;
}

static const struct of_device_id apple_mfi_of_ids[] = {
	{.compatible = "innocomm,apple_mfi_dev",},
	{}
};

static struct platform_driver apple_mfi_driver = {
	.driver = {
		.name = APPLE_MFI_DRIVER_NAME,
		.owner = THIS_MODULE,
		.probe = inoco_apple_mfi_probe,
		.shutdown = inoco_apple_mfi_shutdown,
#ifdef CONFIG_OF
		.of_match_table = apple_mfi_of_ids,
#endif
	},
};

static int __init inoco_apple_mfi_init(void)
{
	MFI_LOG_I("%s, register Apple MFi driver for %s\n", __func__, APPLE_MFI_DRIVER_NAME);

	if (platform_driver_register(&apple_mfi_driver)) 
	{
		MFI_LOG_E("%s, failed to register driver %s!\n", __func__, APPLE_MFI_DRIVER_NAME);
		return -ENODEV;
	}

	return 0;
}

static void __exit inoco_apple_mfi_exit(void)
{
	MFI_LOG_D("%s enter\n", __func__);

	platform_driver_unregister(&apple_mfi_driver);
	MFI_LOG_E("%s, unregister driver %s done\n", __func__, APPLE_MFI_DRIVER_NAME);
}

late_initcall(inoco_apple_mfi_init);
module_exit(inoco_apple_mfi_exit);

static int inoco_apple_mfi_i2c_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct _inoco_apple_mfi_i2c *obj;
	struct i2c_client *new_client;
	struct mfi_register_table *obj_table;
	int ret = 0;

	MFI_LOG_I("%s enter, name=%s, addr=0x%x\n", __func__, client->name, client->addr);

	obj = (struct _inoco_apple_mfi_i2c *)kzalloc(sizeof(struct _inoco_apple_mfi_i2c), GFP_KERNEL);
	if (!obj)
	{
		ret = -ENOMEM;
		MFI_LOG_E("%s, p_inoco_apple_mfi_i2c_dev alloc memory failed, ret = %d\n", __func__, ret);
		goto fail_mfi_i2c;
	}

	memset(obj, 0, sizeof(struct _inoco_apple_mfi_i2c));
	p_inoco_apple_mfi_i2c_dev = obj;
	obj->client = client;
	new_client = obj->client;
	i2c_set_clientdata(new_client, obj);
	inoco_apple_mfi_i2c_client = new_client;

	/* create MFi table */
	obj_table = kzalloc(sizeof(*obj_table), GFP_KERNEL);
	if (!obj_table)
	{
		ret = -ENOMEM;
		MFI_LOG_E("%s, mfi_reg_tbl alloc memory failed, ret = %d\n", __func__, ret);
		goto fail_mfi_table;
	}

	memset(obj_table, 0, sizeof(struct mfi_register_table));
	mfi_reg_tbl = obj_table;

	/* create sysfs */
	if (create_sysfs_interfaces(&client->dev))
	{
		MFI_LOG_E("%s, create sysfs interfaces failed!!\n", __func__);
		ret = -1;
	}

	/* create symlink to dev under /sys/devices */
	if(sysfs_create_link(&devices_kset->kobj, &client->dev.kobj, "inoco_apple_mfi"))
	{
		MFI_LOG_E("%s, create symbolic link to device failed!!\n", __func__);
		goto fail_remove_sysfs;
	}

	return 0;

fail_remove_sysfs:
	remove_sysfs_interfaces(&client->dev);
fail_mfi_table:
	kfree(obj_table);
	obj_table = NULL;
fail_mfi_i2c:
	kfree(obj);
	obj = NULL;
	MFI_LOG_E("%s failed!!\n", __func__);
	return ret;
}

static int inoco_apple_mfi_i2c_driver_remove(struct i2c_client *client)
{
	struct _inoco_apple_mfi_i2c *inoco_apple_mfi_i2c = i2c_get_clientdata(client);

	MFI_LOG_I("%s enter\n", __func__);

	i2c_set_clientdata(client, NULL);
	kfree(inoco_apple_mfi_i2c);
	p_inoco_apple_mfi_i2c_dev = NULL;

	inoco_apple_mfi_i2c_client = NULL;
	i2c_unregister_device(client);

	return 0;
}

static char fifo_read_buf[MFI_MAX_I2C_LEN];
static ssize_t inoco_apple_mfi_cdev_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{
	int read_cnt, ret, fifoLen;
	struct inoco_apple_mfi_char_dev *cdev = file->private_data;

	MFI_LOG_I("%s enter\n", __func__);

	if( down_interruptible(&cdev->sem) )
		return -ERESTARTSYS;

	fifoLen = kfifo_len(&cdev->DataKFiFo);

	while( fifoLen<1 ) // nothing to read
	{
		up(&cdev->sem); // release the lock
		if( file->f_flags & O_NONBLOCK )
			return -EAGAIN;

		if( wait_event_interruptible(cdev->fifo_inq, kfifo_len( &cdev->DataKFiFo )>0) )
		{
			return -ERESTARTSYS; // signal: tell the fs layer to handle it
		}

		if( down_interruptible(&cdev->sem) )
		return -ERESTARTSYS;
	}

	if(count > MFI_MAX_I2C_LEN)
		count = MFI_MAX_I2C_LEN;

	read_cnt = kfifo_out_locked(&cdev->DataKFiFo, fifo_read_buf, count, &cdev->FiFoLock);
	MFI_LOG_D("%s, \"%s\" reading fifo data count=%d\n", __func__, current->comm, read_cnt);

	ret = copy_to_user(buf, fifo_read_buf, read_cnt)?-EFAULT:read_cnt;

	up(&cdev->sem);

	return ret;
}

static ssize_t inoco_apple_mfi_cdev_write(struct file *file, const char __user *buf, size_t count, loff_t *offset)
{
	struct inoco_apple_mfi_char_dev *cdev = file->private_data;
	int ret=0;
	char *tmp;
	uint8_t buffer[] = {0x01, 0x01, 0x01, 0x01};

	MFI_LOG_I("%s enter\n", __func__);

	if( down_interruptible(&cdev->sem) )
		return -ERESTARTSYS;

	if (count > MFI_MAX_I2C_LEN)
		count = MFI_MAX_I2C_LEN;

	tmp = kzalloc(MFI_MAX_I2C_LEN, GFP_KERNEL);
	if(tmp==NULL)
	{
		up(&cdev->sem);
		return -ENOMEM;
	}

	if(copy_from_user(tmp, buf, count))
	{
		up(&cdev->sem);
		kfree(tmp);
		return -EFAULT;
	}

	ret = inoco_apple_mfi_tx_data(buffer, 1);

	up(&cdev->sem);
	MFI_LOG_D("%s, I2C writing %zu bytes.\n", __func__, count);
	kfree(tmp);

	return (ret==0?count:-1);
}

static int inoco_apple_mfi_cdev_open(struct inode *inode, struct file *filp)
{
	MFI_LOG_I("%s enter\n", __func__);

	if( !atomic_dec_and_test(&inoco_apple_mfi_char_available) )
	{
		atomic_inc(&inoco_apple_mfi_char_available);
		return -EBUSY; // already open
	}

	p_char_dev->OpenCnts++;
	filp->private_data = p_char_dev;// Used by the read and write metheds

	MFI_LOG_I("%s, CDev open done!\n", __func__);
	try_module_get(THIS_MODULE);
	return 0;
}

static int inoco_apple_mfi_cdev_release(struct inode *inode, struct file *filp)
{
	struct inoco_apple_mfi_char_dev *cdev = filp->private_data;

	MFI_LOG_I("%s enter\n", __func__);

	atomic_inc(&inoco_apple_mfi_char_available); // release the device

	cdev->OpenCnts--;

	kfifo_reset( &cdev->DataKFiFo );

	MFI_LOG_I("%s, CDev release done!\n", __func__);
	module_put(THIS_MODULE);
	return 0;
}

static unsigned int inoco_apple_mfi_cdev_poll(struct file *filp, struct poll_table_struct *wait)
{
	struct inoco_apple_mfi_char_dev *cdev = filp->private_data;
	unsigned int mask = 0;
	int fifoLen;

	MFI_LOG_I("%s enter\n", __func__);

	down(&cdev->sem);
	poll_wait(filp, &cdev->fifo_inq, wait);

	fifoLen = kfifo_len(&cdev->DataKFiFo);

	if( fifoLen > 0 )
	mask |= POLLIN | POLLRDNORM;	/* readable */
	if( (MFI_FIFO_SIZE - fifoLen) > MFI_MAX_I2C_LEN )
	mask |= POLLOUT | POLLWRNORM;   /* writable */

	up(&cdev->sem);
	return mask;
}

static struct inoco_apple_mfi_char_dev* setup_chardev(void)
{
	struct inoco_apple_mfi_char_dev *pCharDev;

	MFI_LOG_I("%s enter\n", __func__);

	pCharDev = kzalloc(1*sizeof(struct inoco_apple_mfi_char_dev), GFP_KERNEL);
	if(!pCharDev)
	goto fail_mdi_cdev;

	spin_lock_init( &pCharDev->FiFoLock );
	pCharDev->pFiFoBuf = kzalloc(sizeof(unsigned char)*MFI_FIFO_SIZE, GFP_KERNEL);
	if(!pCharDev->pFiFoBuf)
	goto fail_mfi_cdev_fifobuf;

	kfifo_init(&pCharDev->DataKFiFo, pCharDev->pFiFoBuf, MFI_FIFO_SIZE);
	if( !kfifo_initialized(&pCharDev->DataKFiFo) )
	goto fail_mfi_cdev_kfifo;

	pCharDev->OpenCnts = 0;
	sema_init(&pCharDev->sem, 1);
	init_waitqueue_head(&pCharDev->fifo_inq);

	return pCharDev;

fail_mfi_cdev_kfifo:
	kfree(pCharDev->pFiFoBuf);
fail_mfi_cdev_fifobuf:
	kfree(pCharDev);
fail_mdi_cdev:
	MFI_LOG_E("%s failed!\n", __func__);
	return NULL;
}

static const struct i2c_device_id inoco_apple_mfi_i2c_id[] = { {I2C_DRIVER_NAME, 0}, {} };

#ifdef CONFIG_OF
static const struct of_device_id inoco_apple_mfi_id[] = {
	{.compatible = "inoco,apple_mfi"},
	{},
};
#endif
MODULE_DEVICE_TABLE(of, inoco_apple_mfi_id);

static struct i2c_driver inoco_apple_mfi_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = I2C_DRIVER_NAME,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(inoco_apple_mfi_id),
#endif
	},
	.probe = inoco_apple_mfi_i2c_driver_probe,
	.remove = inoco_apple_mfi_i2c_driver_remove,
	.id_table = inoco_apple_mfi_i2c_id,
};

static const struct file_operations inoco_apple_mfi_cdev_fops = {
	.owner	= THIS_MODULE,
	.read = inoco_apple_mfi_cdev_read,
	.write = inoco_apple_mfi_cdev_write,
	.open = inoco_apple_mfi_cdev_open,
	.release = inoco_apple_mfi_cdev_release,
	.poll = inoco_apple_mfi_cdev_poll,
};

static struct miscdevice inoco_apple_mfi_misc_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "inoco_apple_mfi",
	.fops = &inoco_apple_mfi_cdev_fops,
};

void inoco_apple_mfi_i2c_exit_handle(void)
{
	MFI_LOG_I("%s enter\n", __func__);

	if (p_char_dev)
	{
		if( p_char_dev->pFiFoBuf )
		kfree(p_char_dev->pFiFoBuf);

		kfree(p_char_dev);
		p_char_dev = NULL;
	}

	misc_deregister(&inoco_apple_mfi_misc_dev);

	i2c_del_driver(&inoco_apple_mfi_i2c_driver);
}

static void __exit inoco_apple_mfi_i2c_exit(void)
{
	MFI_LOG_I("%s enter\n", __func__);

	inoco_apple_mfi_i2c_exit_handle();
}

static int __init inoco_apple_mfi_i2c_init(void)
{
	int ret = 0;

	MFI_LOG_I("%s enter\n", __func__);

	ret = misc_register(&inoco_apple_mfi_misc_dev);
	if (ret)
	{
		MFI_LOG_E("%s, inoco_apple_mfi_misc_dev register failed, ret = %d\n", __func__, ret);
		goto fail_mfi_i2c_init;
	}

	p_char_dev = setup_chardev(); // allocate the character device
	if(!p_char_dev)
	{
		MFI_LOG_E("%s, failed to setup_chardev\n", __func__);
		ret = -ENOMEM;
		goto fail_mfi_i2c_init;
	}

	ret = i2c_add_driver(&inoco_apple_mfi_i2c_driver);

	if (ret != 0)
	{
		MFI_LOG_E("%s, failed to register Apple MFi i2c driver, err=%d\n", __func__, ret);
		goto fail_mfi_i2c_init;
	}
	else
	{
		MFI_LOG_I("%s, successfully register Apple MFi i2c driver.\n", __func__);
	}

	return 0;

fail_mfi_i2c_init:
	inoco_apple_mfi_i2c_exit_handle();
	return ret;
}

module_init(inoco_apple_mfi_i2c_init);
module_exit(inoco_apple_mfi_i2c_exit);

MODULE_AUTHOR("innocomm@innocomm.com");
MODULE_DESCRIPTION("Innocomm Apple-MFi driver");
MODULE_LICENSE("GPL");
