/*
 * pi5u.c (v1.1) -- PI5U USB TYPE-C Controller device driver
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
#include <linux/reboot.h>


#include "pi5u.h"

#define LOG_TAG "PERICOM-PI5U"
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

/*PERICOM Type-C chip ID*/
#define PI5U_30216A_ID          0x0
#define PI5U_30216D_ID          0x20

/******************************************************************************
* Register addresses
******************************************************************************/
#define REG_DEV_ID              0x01
#define REG_CON                 0x02
#define REG_INT                 0x03
#define REG_CC_STAT             0x04

/******************************************************************************
* Register bits
*******************************************************************************/
/*	  REG_DEV_ID (0x01)    */
/*    REG_CON (0x02)    */
#define TYPE_INT_MASK           0x01
#define TYPE_MODE_SEL_SHIFT     1
#define TYPE_MODE_SEL           (0x03 << TYPE_MODE_SEL_SHIFT)


#define INT_INDEX               0
#define CC_EN                   1
#define NUM_GPIO                2

#define GPIO_CC_INT	            11
#define GPIO_CC_ENABLE		    16


/******************************************************************************/

struct pi5u_info {
	struct i2c_client		*i2c;
	struct device *dev_t;
	struct pi5u_platform_data	 *platform_data;
	struct mutex		mutex;
	struct class *pusb_class;
	int irq;
	enum pi5u_type pusb_type;
};

struct gpio pericom_typec_gpio[NUM_GPIO] = {
	{GPIO_CC_INT, GPIOF_IN, "CC_intr"},
	{GPIO_CC_ENABLE, GPIOF_OUT_INIT_HIGH, "CC_En"}
};

/*******************************NUBIA ADD*********************************/
#define PERICOM_TYPEC_ERROR_STATE       0x4
#define PERICOM_PINCTRL_STATE_ACTIVE    "pericom_pin_active"
#define PERICOM_PINCTRL_STATE_SUSPEND   "pericom_pin_suspend"
struct pericom_pinctrl_info {
	struct pinctrl *pinctrl;
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_suspend;
};
static struct pericom_pinctrl_info pericom_pctrl;
/*******************************NUBIA END*********************************/

static int pi5u_poweroff(struct notifier_block *nb, unsigned long event, void *unused);
static struct notifier_block pi5u_poweroff_notifier = {
        .notifier_call = pi5u_poweroff,
};

/*
struct delayed_work	print_work;
struct i2c_client   *tmp_client;
*/

static int pi5u_poweroff(struct notifier_block *nb, unsigned long event, void *unused)
{
    switch (event) {
        case SYS_RESTART:
            USB_LOG_INFO("SYS_RESTART\n");
            break;
        case SYS_HALT:
            USB_LOG_INFO("SYS_HALT\n");
            break;
        case SYS_POWER_OFF:
            USB_LOG_INFO("SYS_POWER_OFF\n");
            gpio_direction_output(pericom_typec_gpio[CC_EN].gpio,0);
            break;
        default:
            break;
        }
        return NOTIFY_DONE;
}

static int pi5u_read_reg(struct i2c_client *i2c, BYTE reg, BYTE *dest)
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
/*******************************NUBIA ADD*********************************/
static int pericom_typec_reset(struct i2c_client *client,u8 chipid)
{
    u8 regbuf[4] = { 0 };
    u8 writebuf[2] = { 0 };
    int ret = 0;

    /*delay 10ms after enabling typeC*/
    msleep(10);

    writebuf[0] = 0x1;
    writebuf[1] = chipid;
    ret = i2c_smbus_write_i2c_block_data(client, REG_DEV_ID, 2, writebuf);
    if(ret != 0)
    {
        USB_LOG_ERROR("Write i2c block data error !");
        goto error;
    }
    msleep(30);
    writebuf[0] = 0x4;
    writebuf[1] = chipid;
    ret = i2c_smbus_write_i2c_block_data(client, REG_DEV_ID, 2, writebuf);
    if(ret != 0)
    {
        USB_LOG_ERROR("Write i2c block data error !");
        goto error;
    }

    ret = i2c_smbus_read_i2c_block_data(client,REG_DEV_ID,4, regbuf);
    if(ret <= 0)
    {
        USB_LOG_ERROR("Read i2c block data error !");
        goto error;
    }
    USB_LOG_INFO("Chip[0x%x] reset successfully :0x%x,0x%x,0x%x,0x%x\n",chipid,regbuf[0],regbuf[1],regbuf[2],regbuf[3]);
    return 0;
error:
    USB_LOG_ERROR("PI5U chip[0x%x] reset failed!\n",chipid);
    return -1;
}
/*******************************NUBIA END*********************************/
static void pi5u_source_cb(bool attach, int bc_lvl)
{
	USB_LOG_INFO("attached ->%d, BC_LVL ->%d\n",attach, bc_lvl);
}

static void pi5u_sink_cb(bool attach)
{
	USB_LOG_INFO("attached -> %d\n",attach);
    // VBUS switch control
}

static void pi5u_check_type(struct pi5u_info *info, BYTE type)
{
	const char *string;

	if((type & TYPE_MODE_SEL) == PI5U_TYPE_UFP)
	{
		info->pusb_type = PI5U_TYPE_UFP;
		string = "UFP";
	}
	else if((type & TYPE_MODE_SEL) == PI5U_TYPE_DFP)
	{
		info->pusb_type = PI5U_TYPE_DFP;
		string = "DFP";
	}
	else if((type & TYPE_MODE_SEL) == PI5U_TYPE_DRP)
	{
		info->pusb_type = PI5U_TYPE_DRP;
		string = "DRP";
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
	struct pi5u_info *info = dev_get_drvdata(dev);

	pi5u_read_reg(info->i2c, REG_CON, &rdata);
	pi5u_check_type(info, rdata);

	switch(info->pusb_type){
		case PI5U_TYPE_UFP:
			return sprintf(buf, "PI5U_TYPE_UFP\n");
		case PI5U_TYPE_DFP:
			return sprintf(buf, "PI5U_TYPE_DFP\n");
		case PI5U_TYPE_DRP:
			return sprintf(buf, "PI5U_TYPE_DRP\n");
		default:
			return sprintf(buf, "TYPE ERROR\n");
	}
}

static DEVICE_ATTR(type, 0444, show_type, NULL);

static int pi5u_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	int value;

/*******************************NUBIA ADD*********************************/
	pericom_pctrl.pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(pericom_pctrl.pinctrl)) {
		USB_LOG_ERROR("Getting pinctrl handle failed\n");
		return -EINVAL;
	}
	pericom_pctrl.gpio_state_active = pinctrl_lookup_state(pericom_pctrl.pinctrl,
        PERICOM_PINCTRL_STATE_ACTIVE);
	if (IS_ERR_OR_NULL(pericom_pctrl.gpio_state_active)) {
		USB_LOG_ERROR("Failed to get the active state pinctrl handle\n");
		return -EINVAL;
	}
	pericom_pctrl.gpio_state_suspend = pinctrl_lookup_state(pericom_pctrl.pinctrl,
		PERICOM_PINCTRL_STATE_SUSPEND);
	if (IS_ERR_OR_NULL(pericom_pctrl.gpio_state_suspend)) {
		USB_LOG_ERROR("Failed to get the suspend state pinctrl handle\n");
		return -EINVAL;
	}
	value = pinctrl_select_state(pericom_pctrl.pinctrl,
		pericom_pctrl.gpio_state_active);
	if (value)
		USB_LOG_ERROR("Failed to set pin for active state!");
	USB_LOG_INFO("Set pin for active state successfully\n");
/*******************************NUBIA END*********************************/

	value = of_get_named_gpio_flags(np, "pericom,int-gpio", 0, NULL);
	if (value >= 0)
		pericom_typec_gpio[INT_INDEX].gpio = value;
	else
		return -ENODEV;
	value = of_get_named_gpio_flags(np, "pericom,enable-gpio", 0, NULL);
	if (value >= 0)
		pericom_typec_gpio[CC_EN].gpio = value;
	else
		return -ENODEV;
 
	USB_LOG_INFO("Interrupt GPIO = %d\n",pericom_typec_gpio[INT_INDEX].gpio);
	USB_LOG_INFO("CC_ENABLE GPIO = %d\n", pericom_typec_gpio[CC_EN].gpio);
	return 0;
}

static int pi5u_read_device_id(struct pi5u_info *info,u8 *device_id)
{
	int retry_times = 3;
	int res = -1;
	USB_LOG_INFO("start to read device id\n");
	while(retry_times--) {
		res = pi5u_read_reg(info->i2c, REG_DEV_ID, device_id);
		if (res >= 0) {
			USB_LOG_INFO("device_id = %x\n", *device_id);
			if (*device_id == PI5U_30216A_ID || *device_id == PI5U_30216D_ID) {
				USB_LOG_INFO("typeC IC is PERICOM PI5U !\n");
				return 0;
			}
		}
	}
	USB_LOG_ERROR("read device id failed\n");
	return res;

}
 /*******************************NUBIA ADD*********************************/
 /*
static void pi5u_print_TypecReg (struct work_struct *work)
{
	u8 regbuf[4] = { 0 };
	int ret = 0;
	ret = i2c_smbus_read_i2c_block_data(tmp_client,REG_DEV_ID,4, regbuf);
	if(ret <= 0)
	{
		USB_LOG_ERROR("Read i2c block data error !");
		return ;
	}
	USB_LOG_ERROR("***********************PRINT TYPEC REG***************************\n");
	USB_LOG_ERROR("[0x1]:0x%x  [0x2]:0x%x  [0x3]:0x%x  [0x4]:0x%x\n",
		regbuf[0],regbuf[1],regbuf[2],regbuf[3]);
	schedule_delayed_work(&print_work, 5 * HZ);

}*/
irqreturn_t pericom_30216A_irq_thread_handler(int irq, void *dev_id)
{
	struct pi5u_info *info =  dev_id;
	u8 regbuf[4] = { 0 };
	u8 writebuf[2] = { 0 };
	int ret = 0;
	/*1.mask interrupt*/
	writebuf[0] = 0x5;
	writebuf[1] = PI5U_30216A_ID;
	ret = i2c_smbus_write_i2c_block_data(info->i2c, REG_DEV_ID, 2, writebuf);
	if(ret != 0)
	{
		USB_LOG_ERROR("Mask interrupt failed!");
		return IRQ_HANDLED;
	}
	/*2.delay 20 ms*/
	msleep(20);
	/*3. read byte 1-4*/
	ret = i2c_smbus_read_i2c_block_data(info->i2c,REG_DEV_ID,4, regbuf);
	if(ret <= 0)
	{
		USB_LOG_ERROR("Read i2c byte 1-4 failed!");
		goto exit_unmask_int;
	}
	USB_LOG_ERROR("Interrupt: 0x%x,0x%x,0x%x,0x%x\n",regbuf[0],regbuf[1],regbuf[2],regbuf[3]);
	/*4.proccessing*/
	if(regbuf[3] == PERICOM_TYPEC_ERROR_STATE)
	{
		USB_LOG_ERROR("TypeC state error!");
		writebuf[0] = 0x1;
		ret = i2c_smbus_write_i2c_block_data(info->i2c, REG_DEV_ID, 2, writebuf);
		if(ret != 0)
		{
		USB_LOG_ERROR("Write i2c block data error !");
		goto exit_unmask_int;
		}
	}
exit_unmask_int:
	/*5.unmask interrupt*/
	msleep(20);
	writebuf[0] = 0x4;
	ret = i2c_smbus_write_i2c_block_data(info->i2c, REG_DEV_ID, 2, writebuf);
	if(ret != 0)
	{
		USB_LOG_ERROR("Write i2c block data error !");
	}
	return IRQ_HANDLED;
}
 /*******************************NUBIA END*********************************/
static int pi5u_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	struct pi5u_info *info=NULL;
	struct pi5u_platform_data *pdata=client->dev.platform_data;
	int ret = 0;
    u8 chip_id;

	USB_LOG_DEBUG("probe start!\n");
	info = kzalloc(sizeof(struct pi5u_info), GFP_KERNEL);
	if (!info) {
		USB_LOG_ERROR("kzalloc pi5u_info failed\n");
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
				sizeof(struct pi5u_platform_data),
				GFP_KERNEL);
		if (!pdata) {
			USB_LOG_ERROR("Failed to allocate memory\n");
			ret = -ENOMEM;
			goto exit_platform_failed;
		}

		client->dev.platform_data = pdata;
		ret = pi5u_parse_dt(&client->dev);
		if (ret){
			USB_LOG_ERROR("pi5u_parse_dt() err\n");
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
	pdata->source_cb = pi5u_source_cb;
	pdata->sink_cb = pi5u_sink_cb;

	ret = gpio_request_array(pericom_typec_gpio, ARRAY_SIZE(pericom_typec_gpio));
	if (ret < 0) {
		USB_LOG_ERROR("gpio_request_array failed\n");
		goto exit_gpio_request_failed;
	}

	/*enable CC chip*/
	ret = gpio_direction_output(pericom_typec_gpio[CC_EN].gpio,1);
	if(ret != 0){
		USB_LOG_ERROR("set CC chip enable failed!\n");
		goto exit_enableCC_failed;
	}
	register_reboot_notifier(&pi5u_poweroff_notifier);
	info->i2c = client;
	i2c_set_clientdata(client, info);
	info->platform_data = pdata;
	info->irq = gpio_to_irq(pericom_typec_gpio[INT_INDEX].gpio);

	mutex_init(&info->mutex);
	ret = pi5u_read_device_id(info,&chip_id);
	if(ret < 0){
		USB_LOG_ERROR("pi5u_read_device_id fail!\n");
		goto exit_read_id_failed;
	}

	/*******************************NUBIA ADD*************************************/
	if(chip_id == PI5U_30216A_ID)
	{
		ret = request_threaded_irq(info->irq,NULL,pericom_30216A_irq_thread_handler,
			IRQF_ONESHOT | IRQF_TRIGGER_LOW, LOG_TAG, info);
		if(ret < 0){
			USB_LOG_ERROR("request IRQ failed!\n");
			goto exit_read_id_failed;
		}
		enable_irq(info->irq);
	}
	pericom_typec_reset(info->i2c,chip_id);
	/*******************************NUBIA END*************************************/

	/*
	INIT_DELAYED_WORK(&print_work, pi5u_print_TypecReg);
	tmp_client = info->i2c;
	schedule_delayed_work(&print_work, 10 * HZ);
	*/
	info->pusb_class = class_create(THIS_MODULE, "pi5u-type-c");
	info->dev_t = device_create(info->pusb_class, NULL, 0, NULL, "pi5u");
	if (IS_ERR(info->dev_t)) {
		ret = PTR_ERR(info->dev_t);
		USB_LOG_ERROR("device_create  failed\n");
		goto exit_create_dev_failed;
	}
	ret = device_create_file(info->dev_t, &dev_attr_type);
	if (ret < 0) {
		USB_LOG_ERROR("device_create_file  failed\n");
		goto exit_create_dev_failed;
	}
	dev_set_drvdata(info->dev_t, info);

	return 0;
exit_create_dev_failed:
	device_destroy(info->pusb_class, 0);
	class_destroy(info->pusb_class);
/*******************************NUBIA ADD*************************************/
	if(chip_id == PI5U_30216A_ID)
	{
		free_irq(info->irq, info);
	}
/*******************************NUBIA END*************************************/
exit_read_id_failed:
exit_gpio_request_failed:
	gpio_free_array(pericom_typec_gpio,
	ARRAY_SIZE(pericom_typec_gpio));
exit_enableCC_failed:
exit_platform_failed:
	kfree(info);
exit:
	return ret;
}


static int pi5u_remove(struct i2c_client *client)
{
	struct pi5u_info *info = i2c_get_clientdata(client);
	USB_LOG_INFO("\nGPIO free Array\n");
	gpio_free_array(pericom_typec_gpio,
		ARRAY_SIZE(pericom_typec_gpio));
	device_remove_file(info->dev_t, &dev_attr_type);
	device_destroy(info->pusb_class, 0);
	class_destroy(info->pusb_class);
	mutex_destroy(&info->mutex);
	i2c_set_clientdata(client, NULL);
	unregister_reboot_notifier(&pi5u_poweroff_notifier);
	kfree(info);
	return 0;
}


static int  pi5u_suspend(struct i2c_client *client, pm_message_t message)
{
	return 0;
}

static int  pi5u_resume(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id pi5u_i2c_id[] = {
	{ "pi5u", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, pi5u_i2c_id);

#ifdef CONFIG_OF
static struct of_device_id pi5u_usbtypec_match_table[] = {
        { .compatible = "pericom,usbtypec",},
        {},
};
#else
#define  pi5u_usbtypec_match_table NULL
#endif

static struct i2c_driver pi5u_i2c_driver = {
	.driver = {
		.name = "pi5u",
		.owner = THIS_MODULE,
		.of_match_table = pi5u_usbtypec_match_table,
	},
	.probe    = pi5u_probe,
	.remove   = pi5u_remove,
	.suspend  = pi5u_suspend,
	.resume	  = pi5u_resume,
	.id_table = pi5u_i2c_id,
};

static __init int pi5u_i2c_init(void)
{

	return i2c_add_driver(&pi5u_i2c_driver);
}

static __exit void pi5u_i2c_exit(void)
{
	i2c_del_driver(&pi5u_i2c_driver);
}

late_initcall(pi5u_i2c_init);
module_exit(pi5u_i2c_exit);

MODULE_AUTHOR("shuchao gao<gao.shuchao123@zte.com.cn>");
MODULE_DESCRIPTION("I2C bus driver for pi5u USB Type-C");
MODULE_LICENSE("GPL v2");
