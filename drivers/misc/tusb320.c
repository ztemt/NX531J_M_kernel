/*
 * tusb320.c (v1.1) -- tusb320 USB TYPE-C Controller device driver 
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */


#include <linux/module.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/switch.h>
#include <linux/input.h>
#include <linux/timer.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>


#include "tusb320.h"

#define LOG_TAG "TI-TUSB320"
#define DEBUG_ON //DEBUG SWITCH


#define USB_LOG_ERROR(fmt, args...) printk(KERN_ERR "[%s] [%s:%d] " fmt,\
					LOG_TAG, __FUNCTION__, __LINE__, ##args)
#define USB_LOG_INFO(fmt, args...)  printk(KERN_INFO "[%s] [%s:%d] "  fmt,\
					LOG_TAG, __FUNCTION__, __LINE__, ##args)
#ifdef  DEBUG_ON
#define USB_LOG_DEBUG(fmt, args...) printk(KERN_DEBUG "[%s] [%s:%d] "  fmt,\
					LOG_TAG, __FUNCTION__, __LINE__, ##args)
#else
#define USB_LOG_DEBUG(fmt, args...)
#endif

/******************************************************************************
* Register addresses	
******************************************************************************/
#define REG_DEV_ID_0		0x00
#define REG_MODE			0x0A

#define TYPE_MODE_SEL_SHIFT		4
#define TYPE_MODE_SEL				(0x03 << TYPE_MODE_SEL_SHIFT)


/******************************************************************************
* Register bits	
******************************************************************************/
#define TUSB320_ID					0x30

#define INT_INDEX	0
#define CC_EN		1
#define NUM_GPIO	2

#define GPIO_CC_INT	106
#define GPIO_CC_ENABLE		52

/******************************************************************************/

struct tusb320_info {
	struct i2c_client		*i2c;
	struct device *dev_t;
	struct tusb320_platform_data	 *platform_data;
	struct mutex		mutex;
	struct class *pusb_class;
	int irq;
	enum tusb320_type pusb_type;
};

struct gpio ti_typec_gpio[NUM_GPIO] = {
	{GPIO_CC_INT, GPIOF_IN, "CC_intr"},
	{GPIO_CC_ENABLE, GPIOF_OUT_INIT_HIGH, "CC_En"}
};


static int tusb320_read_reg(struct i2c_client *i2c, BYTE reg, BYTE *dest)
{
	int ret;

	ret = i2c_smbus_read_byte_data(i2c, reg);
	if (ret < 0) {
		USB_LOG_ERROR("failed to read reg(0x%x), ret(%d)\n",reg, ret);
		return ret;
	}

	ret &= 0xff;
	*dest = ret;
	return 0;
}
/*
static int tusb320_write_reg(struct i2c_client *i2c, BYTE reg, BYTE value)
{
	int ret;

	ret = i2c_smbus_write_byte_data(i2c, reg, value);
	if (ret < 0)
		USB_LOG_ERROR("failed to write reg(0x%x), ret(%d)\n",reg, ret);

	return ret;
}
*/
static void tusb320_source_cb(bool attach, int bc_lvl)
{
	USB_LOG_INFO("attached ->%d, BC_LVL ->%d\n",attach, bc_lvl);
}

static void tusb320_sink_cb(bool attach)
{
	USB_LOG_INFO("attached -> %d\n",attach);
    // VBUS switch control
}

static void tusb320_check_type(struct tusb320_info *info, BYTE type)
{
    const char *string;

        if((type & TYPE_MODE_SEL) == TUSB320_TYPE_DEFAULT)
    {
        info->pusb_type = TUSB320_TYPE_DEFAULT;
        string = "DEFAULT";
    }
    else if((type & TYPE_MODE_SEL) == TUSB320_TYPE_UFP)
    {
        info->pusb_type = TUSB320_TYPE_UFP;
        string = "UFP";
    }
	else if((type & TYPE_MODE_SEL) == TUSB320_TYPE_DFP)
	{
	    info->pusb_type = TUSB320_TYPE_DFP;
            string = "DFP";
	}
	else if((type & TYPE_MODE_SEL) == TUSB320_TYPE_DRP)
	{
	    info->pusb_type = TUSB320_TYPE_DRP;
            string = "DRP";
	}
	else
	{
	    USB_LOG_INFO("The mode reg is %x\n",type);
	    printk("%s: No device type!\n", __func__);
            return;
	}
	
       printk("%s: Attached TYPE is %s\n", __func__, string);
}

static ssize_t show_type(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	BYTE rdata;
 	struct tusb320_info *info = dev_get_drvdata(dev);

	tusb320_read_reg(info->i2c, REG_MODE, &rdata);
	tusb320_check_type(info, rdata);

	switch(info->pusb_type){
		case TUSB320_TYPE_DEFAULT:
			return sprintf(buf, "TUSB320_TYPE_DEFAULT\n");
		case TUSB320_TYPE_UFP:
			return sprintf(buf, "TUSB320_TYPE_UFP\n");
		case TUSB320_TYPE_DFP:
			return sprintf(buf, "TUSB320_TYPE_DFP\n");
		case TUSB320_TYPE_DRP:
			return sprintf(buf, "TUSB320_TYPE_DRP\n");		
		default:
			return sprintf(buf, "TYPE ERROR\n");
	}
	
}

static DEVICE_ATTR(type, 0444, show_type, NULL);

static int tusb320_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	int value;

	value = of_get_named_gpio_flags(np, "ti,int-gpio", 0, NULL);
	if (value >= 0)
		ti_typec_gpio[INT_INDEX].gpio = value;
	else
		return -ENODEV;
	value = of_get_named_gpio_flags(np, "ti,enable-gpio", 0, NULL);
	if (value >= 0)
		ti_typec_gpio[CC_EN].gpio = value;
	else
		return -ENODEV;

	USB_LOG_INFO("Interrupt GPIO = %d\n",ti_typec_gpio[INT_INDEX].gpio);
	USB_LOG_INFO("CC_ENABLE GPIO = %d\n", ti_typec_gpio[CC_EN].gpio);
	return 0;
}

static int tusb320_read_device_id(struct tusb320_info *info)
{
	int retry_times = 3;
	u8 device_id;
	int res = -1;

	USB_LOG_INFO("start to read device id\n");
	while(retry_times--) {
		res = tusb320_read_reg(info->i2c, REG_DEV_ID_0, &device_id);
		if (res >= 0) {
			USB_LOG_INFO("device_id = %x\n", device_id);
			if (device_id == TUSB320_ID) {
				USB_LOG_INFO("read device id success!typeC IC is TI TUSB320!\n");
				return 0;
			}
		}
	}
	USB_LOG_ERROR("read device id failed\n");
	return res;

}


static int tusb320_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	struct tusb320_info *info=NULL;
	struct tusb320_platform_data *pdata=client->dev.platform_data;
	int ret = 0;


	USB_LOG_DEBUG("probe start!\n");
	info = kzalloc(sizeof(struct tusb320_info), GFP_KERNEL);
	if (!info) {
		USB_LOG_ERROR("kzalloc tusb320_info failed\n");
		ret= -ENOMEM;
		goto exit;	
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
	USB_LOG_ERROR("i2c_check_functionality error");
	ret = -ENODEV;
	goto exit;
	}
	
	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
				sizeof(struct tusb320_platform_data),
				GFP_KERNEL);
		if (!pdata) {
			USB_LOG_ERROR("Failed to allocate memory\n");
			ret = -ENOMEM;
			goto exit_platform_failed;
		}

		client->dev.platform_data = pdata;
		ret = tusb320_parse_dt(&client->dev);
		if (ret){
			USB_LOG_ERROR("tusb320_parse_dt() err\n");
			goto exit_platform_failed;
		}
	} else{
		pdata = client->dev.platform_data;
		if (!pdata) {
			USB_LOG_ERROR("No platform data\n"); 
			ret = -ENODEV;
			goto exit_platform_failed;
		}
	}

	USB_LOG_INFO("parse device tree success!\n");
	pdata->source_cb = tusb320_source_cb;
	pdata->sink_cb = tusb320_sink_cb;

	ret = gpio_request_array(ti_typec_gpio, ARRAY_SIZE(ti_typec_gpio));
	if (ret < 0) {
		USB_LOG_ERROR("gpio_request_array failed\n");
		goto exit_gpio_request_failed;
	}
       
       info->i2c = client;
	i2c_set_clientdata(client, info);
	info->platform_data = pdata;
	info->irq = gpio_to_irq(ti_typec_gpio[INT_INDEX].gpio);
	
	mutex_init(&info->mutex);

	ret = tusb320_read_device_id(info);
	if(ret < 0){
		USB_LOG_ERROR("tusb320_read_device_id fail!\n");
		goto exit_read_id_failed;
	}


	info->pusb_class = class_create(THIS_MODULE, "tusb320-type-c");
	info->dev_t = device_create(info->pusb_class, NULL, 0, NULL, "tusb320");
	if (IS_ERR(info->dev_t)) {
		ret = PTR_ERR(info->dev_t);
		USB_LOG_ERROR("device_create  failed\n");
		goto create_dev_failed;
	}
	ret = device_create_file(info->dev_t, &dev_attr_type);
	if (ret < 0) {
		USB_LOG_ERROR("device_create_file  failed\n");
		goto create_dev_failed;
	}
	dev_set_drvdata(info->dev_t, info);

	return 0;

create_dev_failed:
	device_destroy(info->pusb_class, 0);
	class_destroy(info->pusb_class);
exit_read_id_failed:
exit_gpio_request_failed:
	gpio_free_array(ti_typec_gpio,
	ARRAY_SIZE(ti_typec_gpio));
exit_platform_failed:
	kfree(info);
exit:
	return ret;
}


static int tusb320_remove(struct i2c_client *client)
{
    struct tusb320_info *info = i2c_get_clientdata(client);

	USB_LOG_INFO("\nGPIO free Array\n");
	gpio_free_array(ti_typec_gpio,
		ARRAY_SIZE(ti_typec_gpio));
	
       device_remove_file(info->dev_t, &dev_attr_type);
	device_destroy(info->pusb_class, 0);
	class_destroy(info->pusb_class);	
	mutex_destroy(&info->mutex);
	i2c_set_clientdata(client, NULL);
	
	kfree(info);
	
	return 0;
}


static int  tusb320_suspend(struct i2c_client *client, pm_message_t message)
{
	return 0;
}

static int  tusb320_resume(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id tusb320_i2c_id[] = {
	{ "tusb320", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, tusb320_i2c_id);

#ifdef CONFIG_OF
static struct of_device_id tusb320_usbtypec_match_table[] = {
        { .compatible = "ti,usbtypec",},
        {},
};
#else
#define  tusb320_usbtypec_match_table NULL
#endif
 
static struct i2c_driver tusb320_i2c_driver = {
	.driver = {
		.name = "tusb320",
		.owner = THIS_MODULE,
		.of_match_table = tusb320_usbtypec_match_table,
	},
	.probe    = tusb320_probe,
	.remove   = tusb320_remove,
	.suspend  = tusb320_suspend,
	.resume	  = tusb320_resume,
	.id_table = tusb320_i2c_id,
};

static __init int tusb320_i2c_init(void)
{

	return i2c_add_driver(&tusb320_i2c_driver);
}

static __exit void tusb320_i2c_exit(void)
{
	i2c_del_driver(&tusb320_i2c_driver);
}

late_initcall(tusb320_i2c_init);
module_exit(tusb320_i2c_exit);

MODULE_AUTHOR("shuchao gao<gao.shuchao123@zte.com.cn>");
MODULE_DESCRIPTION("I2C bus driver for tusb320 USB Type-C");
MODULE_LICENSE("GPL v2");
