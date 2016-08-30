/*
 * fusb301.c (v1.1) -- FUSB301 USB TYPE-C Controller device driver 
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

#include "fusb301.h"

#define LOG_TAG "FCHILD-FUSB301"
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
#define REG_DEV_ID		0x01
#define REG_MOD			0x02
#define REG_CON			0x03
#define REG_MAN			0x04
#define REG_RST			0x05
#define REG_MSK			0x10
#define REG_STAT		0x11
#define REG_TYPE		0x12
#define REG_INT			0x13

/******************************************************************************
* Register bits	
******************************************************************************/
/*	  REG_DEV_ID (0x01)    */
#define ID_REV					0x0F
#define ID_VER_SHIFT			4
#define ID_VER					(0x0F << ID_VER_SHIFT)
#define FUSB301_ID				0x12

/*    REG_MOD (0x02)    */
#define MOD_SRC 				0x01
#define MOD_SRC_ACC_SHIFT		1
#define MOD_SRC_ACC 			(0x01 << MOD_SRC_ACC_SHIFT)
#define MOD_SNK_SHIFT			2
#define MOD_SNK 				(0x01 << MOD_SNK_SHIFT)
#define MOD_SNK_ACC_SHIFT 		3
#define MOD_SNK_ACC 			(0x01 << MOD_SNK_ACC_SHIFT)
#define MOD_DRP_SHIFT			4
#define MOD_DRP 				(0x01 << MOD_DRP_SHIFT)
#define MOD_DRP_ACC_SHIFT 		5
#define MOD_DRP_ACC 			(0x01 << MOD_DRP_ACC_SHIFT)

/*    REG_CON (0x03)    */
#define CON_INT_MSK				0x01
#define CON_HOST_CUR_SHIFT		1
#define CON_HOST_CUR 			(0x03 << CON_HOST_CUR_SHIFT)
#define CON_DRP_TGL_SHIFT		4
#define CON_DRP_TGL 			(0x03 << CON_DRP_TGL_SHIFT)

/*    REG_MAN (0x04)    */
#define MAN_ERR_REC				0x01
#define MAN_DIS_SHIFT			1
#define MAN_DIS 				(0x01 << MAN_DIS_SHIFT)
#define MAN_UNATT_SRC_SHIFT		2
#define MAN_UNATT_SRC 			(0x01 << MAN_UNATT_SRC_SHIFT)
#define MAN_UNATT_SNK_SHIFT		3
#define MAN_UNATT_SNK 			(0x01 << MAN_UNATT_SNK_SHIFT)

/*    REG_RST (0x05)    */
#define RST_SW					0x01

/*    REG_MSK (0x10)    */
#define MSK_ATTACH 				0x01
#define MSK_DETACH_SHIFT		1
#define MSK_DETACH 				(0x01 << MSK_DETACH_SHIFT)
#define MSK_BC_LVL_SHIFT		2
#define MSK_BC_LVL 				(0x01 << MSK_BC_LVL_SHIFT)
#define MSK_ACC_CHG_SHIFT		3
#define MSK_ACC_CHG 			(0x01 << MSK_ACC_CHG_SHIFT)

/*    REG_STAT (0x11)    */
#define STAT_ATTACH				0x01 
#define STAT_BC_LVL_SHIFT		1
#define STAT_BC_LVL 			(0x03 << STAT_BC_LVL_SHIFT)
#define STAT_VBUS_OK_SHIFT		3
#define STAT_VBUS_OK 			(0x01 << STAT_VBUS_OK_SHIFT)
#define STAT_ORIENT_SHIFT		3
#define STAT_ORIENT 			(0x03 << STAT_ORIENT_SHIFT)

/*    REG_TYPE (0x12)    */
#define TYPE_AUDIO_ACC			0x01
#define TYPE_DBG_ACC_SHIFT		1
#define TYPE_DBG_ACC			(0x01 << TYPE_DBG_ACC_SHIFT)
#define TYPE_PWR_ACC_SHIFT		2
#define TYPE_PWR_ACC 			(0x01 << TYPE_PWR_ACC_SHIFT)
//#define TYPE_SNK_SHIFT			3
//#define TYPE_SNK 				(0x01 << TYPE_SNK_SHIFT)
//#define TYPE_SRC_SHIFT 			4
//#define TYPE_SRC 				(0x01 << TYPE_SRC_SHIFT)
#define TYPE_SRC_SHIFT			3
#define TYPE_SRC 				(0x01 << TYPE_SRC_SHIFT)
#define TYPE_SNK_SHIFT 			4
#define TYPE_SNK 				(0x01 << TYPE_SNK_SHIFT)

/*    REG_INT (0x13)    */
#define INT_ATTACH				0x01
#define INT_DETACH_SHIFT		1
#define INT_DETACH 				(0x01 << INT_DETACH_SHIFT)
#define INT_BC_LVL_SHIFT		2
#define INT_BC_LVL 				(0x01 << INT_BC_LVL_SHIFT)
#define INT_ACC_CHG_SHIFT		3
#define INT_ACC_CHG				(0x01 << INT_ACC_CHG_SHIFT)

#define INT_INDEX	0
#define CC_EN		1
#define NUM_GPIO	2


#define GPIO_CC_INT	106
#define GPIO_CC_ENABLE		52

/******************************************************************************/
enum fusb301_state{
	FUSB301_UNATTACHED_DRP = 0,  // with Try.SNK
	FUSB301_UNATTACHED_SNK,
	FUSB301_UNATTACHED_SRC,

	FUSB301_ATTACHED_SNK,
	FUSB301_ATTACHED_SRC,
	FUSB301_ATTACHED_DEBUG,
	FUSB301_ATTACHED_AUDIO
};

struct fusb301_info {
	struct i2c_client		*i2c;
	struct device *dev_t;
	struct fusb301_platform_data	 *platform_data;
	struct mutex		mutex;
	struct class *fusb_class;
	int irq;
	enum fusb301_type fusb_type;
	enum fusb301_state state;
	int TriedSink;
	struct timer_list	try_sink_timer;
	struct workqueue_struct *try_sink_wqueue;
	struct work_struct try_sink_work;
};

struct gpio fchild_typec_gpio[NUM_GPIO] = {
	{GPIO_CC_INT, GPIOF_IN, "CC_intr"},
	{GPIO_CC_ENABLE, GPIOF_OUT_INIT_HIGH, "CC_En"}
};

enum fusb301_drp_toggle{
	FUSB301_TOGGLE_SNK35_SRC15 = 0,  // default
	FUSB301_TOGGLE_SNK30_SRC20,
	FUSB301_TOGGLE_SNK25_SRC25,
	FUSB301_TOGGLE_SNK20_SRC30,
};

enum fusb301_host_cur{
	FUSB301_HOST_CUR_NO = 0,  // no current
	FUSB301_HOST_CUR_80,  // default USB
	FUSB301_HOST_CUR_180,  // 1.5A
	FUSB301_HOST_CUR_330,  // 3A	
};

enum fusb301_orient{
	FUSB301_ORIENT_NO_CONN = 0,
	FUSB301_ORIENT_CC1_CC,
	FUSB301_ORIENT_CC2_CC,
	FUSB301_ORIENT_FAULT
};

enum fusb301_config_modes{
	FUSB301_MODE_SRC = 0,
	FUSB301_MODE_SRC_ACC,
	FUSB301_MODE_SNK,
	FUSB301_MODE_SNK_ACC,
	FUSB301_MODE_DRP,
	FUSB301_MODE_DRP_ACC
};

#define TTRYTO_EXP_TIME	600
#define TCCDEBOUNCEMAX_TIME	200  


static int fusb301_read_reg(struct i2c_client *i2c, BYTE reg, BYTE *dest)
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

static int fusb301_write_reg(struct i2c_client *i2c, BYTE reg, BYTE value)
{
	int ret;

	ret = i2c_smbus_write_byte_data(i2c, reg, value);
	if (ret < 0)
		USB_LOG_ERROR("failed to write reg(0x%x), ret(%d)\n",reg, ret);

	return ret;
}

static int fusb301_update_reg(struct i2c_client *i2c, u8 reg, BYTE val, BYTE mask)
{
	int ret;

	ret = i2c_smbus_read_byte_data(i2c, reg);
	if (ret >= 0) {
		BYTE old_val = ret & 0xff;
		BYTE new_val = (val & mask) | (old_val & (~mask));
		ret = i2c_smbus_write_byte_data(i2c, reg, new_val);
	}
	return ret;
}

static void fusb301_source_cb(bool attach, int bc_lvl)
{
	USB_LOG_INFO("attached ->%d, BC_LVL ->%d\n",attach, bc_lvl);
}

static void fusb301_sink_cb(bool attach)
{
	USB_LOG_INFO("attached -> %d\n",attach);
    // VBUS switch control
}

static void fusb301_check_type(struct fusb301_info *info, BYTE type)
{
    const char *string;

    if(type & TYPE_AUDIO_ACC)
    {
        info->fusb_type = FUSB301_TYPE_AUDIO;
        string = "AUDIO_ACC";
    }
	else if(type & TYPE_DBG_ACC)
	{
	    info->fusb_type = FUSB301_TYPE_DEBUG;
            string = "DEBUG_ACC";
	}
	else if(type & TYPE_PWR_ACC)
	{
	    info->fusb_type = FUSB301_TYPE_POWER_ACC;
            string = "POWER_ACC";
	}
	else if(type & TYPE_SRC)
	{
	    info->fusb_type = FUSB301_TYPE_SOURCE;
            string = "SOURCE";
	}	
	else if(type & TYPE_SNK)
	{
	    info->fusb_type = FUSB301_TYPE_SINK;
            string = "SINK";
	}
	else
	{
	    printk("%s: No device type!\n", __func__);
            return;
	}
	
       printk("%s: Attached TYPE is %s\n", __func__, string);
}

static ssize_t show_type(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	BYTE rdata;
 	struct fusb301_info *info = dev_get_drvdata(dev);

	fusb301_read_reg(info->i2c, REG_TYPE, &rdata);
	fusb301_check_type(info, rdata);

	switch(info->fusb_type){
		case FUSB301_TYPE_AUDIO:
			return sprintf(buf, "FUSB301_TYPE_AUDIO\n");
		case FUSB301_TYPE_DEBUG:
			return sprintf(buf, "FUSB301_TYPE_DEBUG\n");
		case FUSB301_TYPE_POWER_ACC:
			return sprintf(buf, "FUSB301_TYPE_POWER_ACC\n");
		case FUSB301_TYPE_SOURCE:
			return sprintf(buf, "FUSB301_SOURCE\n");			
		case FUSB301_TYPE_SINK:
			return sprintf(buf, "FUSB301_TYPE_SINK\n");			
		default:
			return sprintf(buf, "TYPE ERROR\n");
	}
	
}


static DEVICE_ATTR(type, 0444, show_type, NULL);


static void fusb301_try_sink_work_func(struct work_struct *work)
{
	struct fusb301_info *info =
		container_of(work, struct fusb301_info, try_sink_work);

	mutex_lock(&info->mutex);

	if(info->state == FUSB301_UNATTACHED_SNK)
	{
		USB_LOG_INFO("FUSB301_UNATTACHED_SNK!\n");
		mod_timer(&info->try_sink_timer, jiffies + msecs_to_jiffies(TCCDEBOUNCEMAX_TIME));
		info->state = FUSB301_UNATTACHED_SRC;
		fusb301_write_reg(info->i2c, REG_MOD, MOD_SRC);
		fusb301_write_reg(info->i2c, REG_MAN, MAN_UNATT_SRC);
	}
	else if(info->state == FUSB301_UNATTACHED_SRC)
	{

		USB_LOG_INFO("FUSB301_UNATTACHED_SRC!\n");
		info->state = FUSB301_UNATTACHED_DRP;
		info->TriedSink = 0;
		fusb301_write_reg(info->i2c, REG_MOD, MOD_DRP);
		fusb301_write_reg(info->i2c, REG_MAN, MAN_UNATT_SNK);
	}

	mutex_unlock(&info->mutex);

	
}
static void __fusb302_try_sink_timer_func(unsigned long param)
{
    struct fusb301_info *info = (struct fusb301_info *)param;	
	
	USB_LOG_INFO("entered and queue work\n");
	queue_work(info->try_sink_wqueue, &info->try_sink_work);
}

static void fusb301_initialization(struct fusb301_info *info)
{

	info->fusb_type = FUSB301_TYPE_NONE;

	fusb301_write_reg(info->i2c, REG_MOD, MOD_DRP_ACC);
	info->state = FUSB301_UNATTACHED_DRP;
	info->TriedSink = 0;
	fusb301_update_reg(info->i2c, REG_CON, 0, CON_INT_MSK);  //unmask global interrupts
	USB_LOG_INFO("fusb301 initialization success!\n");
}


static int fchild_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	int value;

	value = of_get_named_gpio_flags(np, "fchild,int-gpio", 0, NULL);
	if (value >= 0)
		fchild_typec_gpio[INT_INDEX].gpio = value;
	else
		return -ENODEV;
	value = of_get_named_gpio_flags(np, "fchild,enable-gpio", 0, NULL);
	if (value >= 0)
		fchild_typec_gpio[CC_EN].gpio = value;
	else
		return -ENODEV;

	USB_LOG_INFO("Interrupt GPIO = %d\n",fchild_typec_gpio[INT_INDEX].gpio);
	USB_LOG_INFO("CC_ENABLE GPIO = %d\n", fchild_typec_gpio[CC_EN].gpio);
	return 0;
}

static int fusb301_read_device_id(struct fusb301_info *info)
{
	int retry_times = 3;
	u8 device_id;
	int res = -1;

	USB_LOG_INFO("start to read device id\n");
	while(retry_times--) {
		res = fusb301_read_reg(info->i2c, REG_DEV_ID, &device_id);
		if (res >= 0) {
			USB_LOG_INFO("device_id = %x\n", device_id);
			if (device_id == FUSB301_ID) {
				USB_LOG_INFO("read device id success!typeC IC is FairChild FUSB301!\n");
				return 0;
			}
		}
	}
	USB_LOG_ERROR("read device id failed\n");
	return res;

}


static int fusb301_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	struct fusb301_info *info=NULL;
	struct fusb301_platform_data *pdata=client->dev.platform_data;
	int ret = 0;


	USB_LOG_DEBUG("probe start!\n");
	info = kzalloc(sizeof(struct fusb301_info), GFP_KERNEL);
	if (!info) {
		USB_LOG_ERROR("kzalloc fusb301_info failed\n");
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
				sizeof(struct fusb301_platform_data),
				GFP_KERNEL);
		if (!pdata) {
			USB_LOG_ERROR("Failed to allocate memory\n");
			ret = -ENOMEM;
			goto exit_platform_failed;
		}

		client->dev.platform_data = pdata;
		ret = fchild_parse_dt(&client->dev);
		if (ret){
			USB_LOG_ERROR("fchild_parse_dt() err\n");
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
	pdata->source_cb = fusb301_source_cb;
	pdata->sink_cb = fusb301_sink_cb;

	ret = gpio_request_array(fchild_typec_gpio, ARRAY_SIZE(fchild_typec_gpio));
	if (ret < 0) {
		USB_LOG_ERROR("gpio_request_array failed\n");
		goto exit_gpio_request_failed;
	}
       
       info->i2c = client;
	i2c_set_clientdata(client, info);
	info->platform_data = pdata;
	info->irq = gpio_to_irq(fchild_typec_gpio[INT_INDEX].gpio);

	mutex_init(&info->mutex);

	ret = fusb301_read_device_id(info);
	if(ret < 0){
		USB_LOG_ERROR("fusb301_read_device_id fail!\n");
		goto exit_read_id_failed;
	}
	
	info->fusb_class = class_create(THIS_MODULE, "fchild-type-c");
	info->dev_t = device_create(info->fusb_class, NULL, 0, NULL, "fusb301");
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

	info->try_sink_wqueue = create_singlethread_workqueue("fusb301_wqueue");
	INIT_WORK(&info->try_sink_work, fusb301_try_sink_work_func);
	
       fusb301_initialization(info);

	setup_timer(&info->try_sink_timer, __fusb302_try_sink_timer_func,
		(unsigned long)info);


	return 0;

create_dev_failed:
	device_destroy(info->fusb_class, 0);
	class_destroy(info->fusb_class);
exit_read_id_failed:
exit_gpio_request_failed:
	gpio_free_array(fchild_typec_gpio,
	ARRAY_SIZE(fchild_typec_gpio));
exit_platform_failed:
	kfree(info);
exit:
	return ret;
}


static int fusb301_remove(struct i2c_client *client)
{
    struct fusb301_info *info = i2c_get_clientdata(client);
/*
	if (client->irq) {
		disable_irq_wake(client->irq);
		free_irq(client->irq, info);
	}
*/

	USB_LOG_INFO("\nGPIO free Array\n");
	gpio_free_array(fchild_typec_gpio,
		ARRAY_SIZE(fchild_typec_gpio));
	
       device_remove_file(info->dev_t, &dev_attr_type);
	device_destroy(info->fusb_class, 0);
	class_destroy(info->fusb_class);	
	mutex_destroy(&info->mutex);
	i2c_set_clientdata(client, NULL);
	
	kfree(info);
	
	return 0;
}


static int  fusb301_suspend(struct i2c_client *client, pm_message_t message)
{
	return 0;
}

static int  fusb301_resume(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id fusb301_i2c_id[] = {
	{ "fusb301", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, fusb301_i2c_id);

#ifdef CONFIG_OF
static struct of_device_id fusb301_usbtypec_match_table[] = {
        { .compatible = "fchild,usbtypec",},
        {},
};
#else
#define  fusb301_usbtypec_match_table NULL
#endif
 
static struct i2c_driver fusb301_i2c_driver = {
	.driver = {
		.name = "fusb301",
		.owner = THIS_MODULE,
		.of_match_table = fusb301_usbtypec_match_table,
	},
	.probe    = fusb301_probe,
	.remove   = fusb301_remove,
	.suspend  = fusb301_suspend,
	.resume	  = fusb301_resume,
	.id_table = fusb301_i2c_id,
};

static __init int fusb301_i2c_init(void)
{

	return i2c_add_driver(&fusb301_i2c_driver);
}

static __exit void fusb301_i2c_exit(void)
{
	i2c_del_driver(&fusb301_i2c_driver);
}

late_initcall(fusb301_i2c_init);
module_exit(fusb301_i2c_exit);

MODULE_AUTHOR("shuchao gao<gao.shuchao123@zte.com.cn>");
MODULE_DESCRIPTION("I2C bus driver for FUSB301 USB Type-C");
MODULE_LICENSE("GPL v2");
