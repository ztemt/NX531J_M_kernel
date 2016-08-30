/******************** (C) COPYRIGHT 2013 ZTEMT ********************
*
* File Name          : hall_pair_out.c
* Authors            : Zhu Bing
* Version            : V.1.0.0
* Date               : 04/17/2014
*
********************************************************************************
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
* OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
* PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
********************************************************************************
********************************************************************************
Version History.

Revision 1-0-0 04/17/2014
 first revision

*******************************************************************************/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/miscdevice.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/of_gpio.h>
#include <linux/pm_runtime.h>
#include <linux/of_gpio.h>
#include <linux/mutex.h>
#include "hall_device.h"

#define LOG_TAG "HALL_DEVICE"
#define DEBUG_ON //DEBUG SWITCH

#define SENSOR_LOG_FILE__ strrchr(__FILE__, '/') ? (strrchr(__FILE__, '/')+1) : __FILE__

#ifdef  CONFIG_FEATURE_NUBIA_SENSORS_LOG_ON
#define SENSOR_LOG_ERROR(fmt, args...) printk(KERN_ERR   "[%s] [%s: %d] "  fmt,\
                                              LOG_TAG,__FUNCTION__, __LINE__, ##args)
    #ifdef  DEBUG_ON
#define SENSOR_LOG_INFO(fmt, args...)  printk(KERN_INFO  "[%s] [%s: %d] "  fmt,\
                                              LOG_TAG,__FUNCTION__, __LINE__, ##args)

#define SENSOR_LOG_DEBUG(fmt, args...) printk(KERN_DEBUG "[%s] [%s: %d] "  fmt,\
                                              LOG_TAG,__FUNCTION__, __LINE__, ##args)
    #else
#define SENSOR_LOG_INFO(fmt, args...)
#define SENSOR_LOG_DEBUG(fmt, args...)
    #endif

#else
#define SENSOR_LOG_ERROR(fmt, args...)
#define SENSOR_LOG_INFO(fmt, args...)
#define SENSOR_LOG_DEBUG(fmt, args...)
#endif

//#define HALL_VDD_MAX_UV 1950000
//#define HALL_VDD_MIN_UV  1750000
static dev_t const   hall_device_dev_t   = MKDEV(MISC_MAJOR, 252);

static struct class  *hall_device_class;

static const struct dev_pm_ops hall_device_pm_ops = {
    .suspend = hall_device_suspend,
    .resume  = hall_device_resume,
};

static const struct i2c_device_id hall_device_idtable_id[] = {
     { "zte,hall_pair_out", 0 },
     { },
 };

static struct of_device_id of_hall_device_idtable[] = {
     { .compatible = "zte,hall_pair_out",},
     {}
};

static struct i2c_driver hall_device_driver = {
    .driver = {
        .name = "hall_pair_out",
        .of_match_table = of_hall_device_idtable,
        .pm = &hall_device_pm_ops,
    },
    .id_table = hall_device_idtable_id,
    .probe = hall_device_probe,
    .remove = hall_device_remove,
};


static int __init hall_device_init(void)
{
        SENSOR_LOG_INFO("driver: init\n");
        return i2c_add_driver(&hall_device_driver);
}

static void __exit hall_device_exit(void)
{
        SENSOR_LOG_INFO("driver: exit\n");
        i2c_del_driver(&hall_device_driver);
}
static void  hall_device_power_parse(struct hall_device_chip * chip)
{
	int rc=0;
	u32 tmp;
	struct device_node *np=chip->client->dev.of_node;
	rc=of_property_read_u32(np, "hall_device,vdd-max", &tmp);
	chip->vdd_max=(!rc?tmp:1950000);
	rc=of_property_read_u32(np,"hall_device,vdd-min",&tmp);
	chip->vdd_min=(!rc?tmp:1750000);
	SENSOR_LOG_INFO("the vdd value is between%d and %d\n", chip->vdd_max,chip->vdd_min);
}
static int  hall_device_power_init(struct hall_device_chip * chip, bool on)
{
	int rc=0;
	if(on){
		chip->vdd=regulator_get(&chip->client->dev, "vdd");
		if (IS_ERR(chip->vdd)) {
			rc = PTR_ERR(chip->vdd);
			SENSOR_LOG_ERROR("Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}
		if(regulator_count_voltages(chip->vdd)>0){
			rc=regulator_set_voltage(chip->vdd, chip->vdd_min, chip->vdd_max);
			if(rc){
				SENSOR_LOG_ERROR("set regulator vdd failed\n");
				goto set_hall_vdd_failed;
			}
		}
		rc=regulator_enable(chip->vdd);
		if(rc){
			SENSOR_LOG_ERROR("enable regulator vdd failed\n");
			goto enable_hall_vdd_failed;
		}
	}else{
		regulator_disable(chip->vdd);
		if(regulator_count_voltages(chip->vdd)>0)
			regulator_set_voltage(chip->vdd, 0, chip->vdd_max);
		regulator_put(chip->vdd);
	}
	SENSOR_LOG_INFO("SUCCESS\n");
	return rc;
enable_hall_vdd_failed:
		if(regulator_count_voltages(chip->vdd)>0)
			regulator_set_voltage(chip->vdd, 0, chip->vdd_max);
set_hall_vdd_failed:
		regulator_put(chip->vdd);
		return rc;

}
/***ztemt add start,2015.11.05***/
static int hall_device_pinctrl_init(struct hall_device_chip*chip, bool on)
{
	if(on){
	chip->pinctrl_info.pinctrl = devm_pinctrl_get(&(chip->client->dev));
	if(IS_ERR_OR_NULL(chip->pinctrl_info.pinctrl)) {
		SENSOR_LOG_ERROR("[hall_device]%s : hall_device get pinctrl info error.\n",__func__);
		return PTR_ERR(chip->pinctrl_info.pinctrl);
		} else {
		chip->pinctrl_info.pin_active = pinctrl_lookup_state(chip->pinctrl_info.pinctrl ,"hall_device_int_active");
		if(IS_ERR_OR_NULL(chip->pinctrl_info.pin_active)) {
			SENSOR_LOG_ERROR("[hall_device]%s : hall_device get pin_active info error.\n",__func__);
			return PTR_ERR(chip->pinctrl_info.pin_active);

			}
		chip->pinctrl_info.pin_suspend = pinctrl_lookup_state(chip->pinctrl_info.pinctrl, "hall_device_int_suspend");
		if(IS_ERR_OR_NULL(chip->pinctrl_info.pin_suspend)) {
			SENSOR_LOG_ERROR("[hall_device]%s : hall_device get pin_suspend info error.\n",__func__);
			return PTR_ERR(chip->pinctrl_info.pin_suspend);
			}
		}
		return 0;
	}else{
		devm_pinctrl_put(chip->pinctrl_info.pinctrl);
		return 0;
	}

}

static int hall_device_set_pinctrl(const struct hall_device_chip *chip, bool active)
{
	int ret = 0;
	if(!chip->pinctrl_info.pinctrl || !chip->pinctrl_info.pin_active || !chip->pinctrl_info.pin_suspend) {
		SENSOR_LOG_ERROR("%s : pinctrl is invalid, skip.\n",__func__);
		return -ENXIO;
		}
	if(active) {
			ret = pinctrl_select_state(chip->pinctrl_info.pinctrl,chip->pinctrl_info.pin_active);
			if(ret<0)
				return -EPERM;
		} else {
			ret = pinctrl_select_state(chip->pinctrl_info.pinctrl,chip->pinctrl_info.pin_suspend);
			if(ret<0)
				return -EPERM;
		}
	SENSOR_LOG_DEBUG("%s : set pinctrl to [%s], ret = %d.\n",__func__,active ? "active" : "suspend",ret);
	return 0;
}
/***ztemt add end***/
static void hall_device_irq_work_s(struct work_struct *work)
{
	struct hall_device_chip *chip = container_of(work, struct hall_device_chip, irq_work_s);
    unsigned int hall_device_state;

    mutex_lock(&chip->lock);
    hall_device_state = gpio_get_value(chip->irq_s.irq_pin) ? MAGNETIC_DEVICE_FAR : MAGNETIC_DEVICE_NEAR;

    if (hall_device_state==chip->state_s)
    {
        SENSOR_LOG_INFO("S [%s] same state\n",hall_device_state==1? "NEAR" : "FAR");
    }
    else
    {
        chip->state_s = hall_device_state;
        chip->state_n = gpio_get_value(chip->irq_n.irq_pin) ? MAGNETIC_DEVICE_FAR : MAGNETIC_DEVICE_NEAR;
        SENSOR_LOG_INFO("N is %s, S is %s\n",chip->state_n==1? "NEAR" : "FAR",chip->state_s==1? "NEAR" : "FAR");
        input_report_rel(chip->idev, REL_RX, chip->state_s);
        input_report_rel(chip->idev, REL_RY, chip->state_n);
        input_sync(chip->idev);
    }
    if (chip->state_s!=MAGNETIC_DEVICE_NEAR)
    {
       wake_lock_timeout(&chip->wakeup_wakelock.lock,3000);

    }
    hall_device_irq_enable(&(chip->irq_s), true, true);
	mutex_unlock(&chip->lock);
};

static void hall_device_irq_work_n(struct work_struct *work)
{
	struct hall_device_chip *chip = container_of(work, struct hall_device_chip, irq_work_n);
    unsigned int hall_device_state;

	mutex_lock(&chip->lock);
    hall_device_state = gpio_get_value(chip->irq_n.irq_pin) ? MAGNETIC_DEVICE_FAR : MAGNETIC_DEVICE_NEAR;

    if (hall_device_state==chip->state_n)
    {
        SENSOR_LOG_INFO("MAGNETIC_DEVICE N [%s] same state\n",hall_device_state==1? "NEAR" : "FAR");
    }
    else
    {
        chip->state_n = hall_device_state;
        chip->state_s = gpio_get_value(chip->irq_s.irq_pin) ? MAGNETIC_DEVICE_FAR : MAGNETIC_DEVICE_NEAR;
        SENSOR_LOG_INFO("N is %s, S is %s\n",chip->state_n==1? "NEAR" : "FAR",chip->state_s==1? "NEAR" : "FAR");
        input_report_rel(chip->idev, REL_RX, chip->state_s);
        input_report_rel(chip->idev, REL_RY, chip->state_n);
        input_sync(chip->idev);
    }
    if (chip->state_n!=MAGNETIC_DEVICE_NEAR)
    {
		wake_lock_timeout(&chip->wakeup_wakelock.lock, 3000);
    }
    hall_device_irq_enable(&(chip->irq_n), true, true);
	mutex_unlock(&chip->lock);
};


static void hall_device_check_state(struct hall_device_chip *chip)
{
    chip->state_n = gpio_get_value(chip->irq_n.irq_pin) ? MAGNETIC_DEVICE_FAR : MAGNETIC_DEVICE_NEAR;
    chip->state_s = gpio_get_value(chip->irq_s.irq_pin) ? MAGNETIC_DEVICE_FAR : MAGNETIC_DEVICE_NEAR;

    SENSOR_LOG_INFO("N is %s, S is %s\n",chip->state_n==1? "NEAR" : "FAR",chip->state_s==1? "NEAR" : "FAR");

    input_report_rel(chip->idev, REL_RX, chip->state_s);
    input_report_rel(chip->idev, REL_RY, chip->state_n);
    input_sync(chip->idev);
};

static irqreturn_t hall_device_irq_s(int irq, void *handle)
{
	struct hall_device_chip *chip = handle;
    SENSOR_LOG_INFO("enter hall_s!\n");
    hall_device_irq_enable(&(chip->irq_s), false, false);

    if (true == chip->enabled)
    {
        wake_lock_timeout(&chip->wakeup_wakelock.lock,1000);
    }

	if (0==schedule_work(&chip->irq_work_s))
    {
        SENSOR_LOG_INFO("schedule_work failed!\n");
    }

    SENSOR_LOG_INFO("exit hall_s!\n");

	return IRQ_HANDLED;
}

static irqreturn_t hall_device_irq_n(int irq, void *handle)
{
	struct hall_device_chip *chip = handle;
    SENSOR_LOG_INFO("enter hall_n!\n");
    hall_device_irq_enable(&(chip->irq_n), false, false);
    if (true == chip->enabled)
    {
        wake_lock_timeout(&chip->wakeup_wakelock.lock,1000);
    }

	if (0==schedule_work(&chip->irq_work_n))
    {
        SENSOR_LOG_INFO("schedule_work failed!\n");
    }

    SENSOR_LOG_INFO("exit  hall_n!\n");

	return IRQ_HANDLED;
}


static void hall_device_irq_enable(struct hall_device_irq * irq, bool enable, bool flag_sync)
{
    if (enable == irq->enabled)
    {
        SENSOR_LOG_INFO("doubule %s irq %d, retern here\n",enable? "enable" : "disable", irq->irq_num);
        return;
    }
    else
    {
        irq->enabled  = enable;
        SENSOR_LOG_INFO("%s irq %d\n",enable? "enable" : "disable",irq->irq_num);
    }

    if (enable)
    {
        enable_irq(irq->irq_num);
    }
    else
    {
        if (flag_sync)
        {
            disable_irq(irq->irq_num);
        }
        else
        {
            disable_irq_nosync(irq->irq_num);
        }
    }
}


static void hall_device_enable(struct hall_device_chip *chip, int on)
{
    SENSOR_LOG_INFO("%s hall_device\n",on? "enable" : "disable");

	if (on)
    {
        chip->state_s = MAGNETIC_DEVICE_UNKNOW;
        chip->state_n = MAGNETIC_DEVICE_UNKNOW;
		hall_device_irq_enable(&(chip->irq_s), true, true);
		hall_device_irq_enable(&(chip->irq_n), true, true);
        hall_device_check_state(chip);
	}
    else
    {
        hall_device_irq_enable(&(chip->irq_s), false, true);
        hall_device_irq_enable(&(chip->irq_n), false, true);
    }
}

static ssize_t hall_value_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct hall_device_chip *chip = dev_get_drvdata(dev);
	SENSOR_LOG_INFO("\n");
	schedule_delayed_work(&chip->flush_work, msecs_to_jiffies(200));
	return snprintf(buf,PAGE_SIZE,"hall_n value is %d, hall_s value is %d\n",chip->state_n,chip->state_s);
}


static ssize_t hall_hw_count_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct hall_device_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->hall_hw_device_count);
}

static ssize_t hall_device_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct hall_device_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->enabled);
}

static ssize_t hall_device_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct hall_device_chip *chip = dev_get_drvdata(dev);
	bool value;

	if (strtobool(buf, &value))
		return -EINVAL;
    mutex_lock(&chip->lock);

    chip->enabled = (value>0) ? true : false;
    hall_device_enable(chip, chip->enabled);

    mutex_unlock(&chip->lock);

	return size;
}

static struct device_attribute attrs_hall_device[] = {
	__ATTR(enable,                          0640,   hall_device_enable_show,            hall_device_enable_store),
    __ATTR(hall_hw_count,                   0444,   hall_hw_count_show,                 NULL),
    __ATTR(hall_value,                      0444,   hall_value_show,                    NULL),
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attrs_hall_device); i++)
		if (device_create_file(dev, attrs_hall_device + i))
			goto error;
	return 0;

error:
	for ( ; i >= 0; i--)
		device_remove_file(dev, attrs_hall_device + i);
	dev_err(dev, "%s:Unable to create interface\n", __func__);
	return -1;
}

static void hall_device_chip_data_init(struct hall_device_chip *chip)
{
    chip->enabled = false;
    chip->irq_s.enabled = true;
    chip->irq_n.enabled = true;
    chip->wakeup_wakelock.name = "hall_device_wakelock";
    //chip->wakeup_wakelock.locked = false;
   // chip->on_irq_working = false;
    chip->hall_hw_device_count = 2;
    chip->state_s = MAGNETIC_DEVICE_UNKNOW;
    chip->state_n = MAGNETIC_DEVICE_UNKNOW;
}

static int hall_device_parse_dt(struct hall_device_chip *chip)
{
	struct device_node *np = chip->client->dev.of_node;
	chip->irq_s.irq_pin = of_get_named_gpio(np, "hall_device,s-irq-gpio", 0);
	chip->irq_n.irq_pin = of_get_named_gpio(np, "hall_device,n-irq-gpio", 0);

    SENSOR_LOG_INFO("irq_s.irq_pin is %d, irq_n.irq_pin is %d\n",chip->irq_s.irq_pin,chip->irq_n.irq_pin);
    return 0;
}

static void hall_device_flush_work_func(struct work_struct *work)
{
	struct hall_device_chip *chip = container_of(work, struct hall_device_chip, flush_work.work);
    SENSOR_LOG_INFO("prob Enter\n");
	hall_device_check_state(chip);
    SENSOR_LOG_INFO("prob Exit\n");
}
static  int hall_device_set_irq_pin_s(struct hall_device_chip *chip)
{
	int ret;
 	ret = gpio_request(chip->irq_s.irq_pin, "hall_device_irq_s");
    	if (ret){
		SENSOR_LOG_INFO("gpio %d is busy and then to free it\n",chip->irq_s.irq_pin);
		gpio_free(chip->irq_s.irq_pin);
		ret = gpio_request(chip->irq_s.irq_pin, "hall_device_irq_s");
        	if (ret){
            		SENSOR_LOG_INFO("gpio %d is busy and then to free it\n",chip->irq_s.irq_pin);
            		return ret;
		}
	}

    	ret=gpio_direction_input(chip->irq_s.irq_pin);
	if(ret){
		SENSOR_LOG_INFO("set gpio direction input failed\n");
		return ret;
	}

    	gpio_set_value(chip->irq_s.irq_pin, 1);
	return 0;

}
static  int hall_device_set_irq_pin_n(struct hall_device_chip *chip)
{
	int ret;
 	ret = gpio_request(chip->irq_n.irq_pin, "hall_device_irq_n");
    	if (ret){
       	 SENSOR_LOG_INFO("gpio %d is busy and then to free it\n",chip->irq_n.irq_pin);
              gpio_free(chip->irq_n.irq_pin);
        	ret = gpio_request(chip->irq_n.irq_pin, "hall_device_irq_n");
        	if (ret) {
            		SENSOR_LOG_INFO("gpio %d is busy and then to free it\n",chip->irq_n.irq_pin);
            		return ret;
          	}
    	}
     	ret =gpio_direction_input(chip->irq_n.irq_pin);
	if(ret){
		SENSOR_LOG_INFO("set gpio direction input failed\n");
		return ret;
	}
    	gpio_set_value(chip->irq_n.irq_pin, 1);
	return 0;

}
static int hall_device_probe(struct i2c_client *client,
                  const struct i2c_device_id *id)
{
    int ret = 0;
	static struct hall_device_chip *chip;

    SENSOR_LOG_INFO("prob start\n");

    chip = kzalloc(sizeof(struct hall_device_chip), GFP_KERNEL);
    if (!chip) {
        ret = -ENOMEM;
        goto malloc_failed;
    }

	chip->client = client;
	i2c_set_clientdata(client, chip);

    ret=hall_device_pinctrl_init(chip,true);
	if(ret)
	{
		SENSOR_LOG_ERROR("pinctrl init failed!\n");
		goto pinctrl_init_failed;
	}

    ret=hall_device_set_pinctrl(chip, true);
	if(ret){
		SENSOR_LOG_ERROR("set pinctrl state failed\n");
		goto pinctrl_set_failed;
	}

    hall_device_chip_data_init(chip);
    hall_device_power_parse(chip);
    ret=hall_device_power_init(chip, true);
	if(ret){
		SENSOR_LOG_ERROR("power init failed\n");
		goto  power_init_failed;
	}

    hall_device_parse_dt(chip);

    INIT_DELAYED_WORK(&chip->flush_work, hall_device_flush_work_func);

    SENSOR_LOG_INFO("hall_device_int_s is %d",chip->irq_s.irq_pin);
    SENSOR_LOG_INFO("hall_device_int_n is %d",chip->irq_n.irq_pin);

	mutex_init(&chip->lock);


    hall_device_class   = class_create(THIS_MODULE, "hall_device");

    chip->hall_device_dev = device_create(hall_device_class, NULL, hall_device_dev_t, &hall_device_driver ,"hall_device");
    if (IS_ERR(chip->hall_device_dev))
    {
       ret = PTR_ERR(chip->hall_device_dev);
       goto create_hall_device_dev_failed;
    }

	dev_set_drvdata(chip->hall_device_dev, chip);

    ret =hall_device_set_irq_pin_s(chip);
    if(ret){
		SENSOR_LOG_ERROR("hall device set irq pin falied\n");
		goto create_hall_device_dev_failed;
    }
    chip->irq_s.irq_num = gpio_to_irq(chip->irq_s.irq_pin);
    INIT_WORK(&chip->irq_work_s, hall_device_irq_work_s);
    ret = request_threaded_irq(chip->irq_s.irq_num, NULL, &hall_device_irq_s, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_ONESHOT, "hall_device_irq_s", chip);
    if (ret) {
        SENSOR_LOG_ERROR("Failed to request irq %d\n", chip->irq_s.irq_num);
        goto irq_s_register_fail;
    }

   ret =hall_device_set_irq_pin_n(chip);
    if(ret){
		SENSOR_LOG_ERROR("hall device set irq pin falied\n");
		goto create_hall_device_dev_failed;
    }
     chip->irq_n.irq_num = gpio_to_irq(chip->irq_n.irq_pin);
    INIT_WORK(&chip->irq_work_n, hall_device_irq_work_n);
    ret = request_threaded_irq(chip->irq_n.irq_num , NULL, &hall_device_irq_n, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_ONESHOT, "hall_device_irq_n", chip);
    if (ret) {
        SENSOR_LOG_ERROR("Failed to request irq %d\n", chip->irq_n.irq_num );
        goto irq_n_register_fail;
    }

    chip->idev = input_allocate_device();
    if (!chip->idev)
    {
        SENSOR_LOG_ERROR("no memory for idev\n");
        ret = -ENODEV;
        goto input_alloc_failed;
    }
    chip->idev->name = "hall_device";
    chip->idev->id.bustype = BUS_I2C;

    set_bit(EV_REL,     chip->idev->evbit);
    set_bit(REL_RX,     chip->idev->relbit);  //HALL S
    set_bit(REL_RY,     chip->idev->relbit);  //HALL N


    ret = input_register_device(chip->idev);
    if (ret) {
        input_free_device(chip->idev);
        SENSOR_LOG_ERROR("cant register input '%s'\n",chip->idev->name);
        goto input_register_failed;
    }

    create_sysfs_interfaces(chip->hall_device_dev);

    hall_device_irq_enable(&(chip->irq_s), false, true);
    hall_device_irq_enable(&(chip->irq_n), false, true);

    wake_lock_init(&chip->wakeup_wakelock.lock, WAKE_LOCK_SUSPEND, chip->wakeup_wakelock.name);
    SENSOR_LOG_INFO("prob success\n");

    return 0;

input_register_failed:
    input_free_device(chip->idev);
input_alloc_failed:
malloc_failed:
irq_n_register_fail:
irq_s_register_fail:
create_hall_device_dev_failed:
    chip->hall_device_dev = NULL;
    class_destroy(hall_device_class);
    hall_device_power_init(chip, false);
power_init_failed:

pinctrl_set_failed:
    hall_device_pinctrl_init(chip, false);
pinctrl_init_failed:
    kfree(chip);
    SENSOR_LOG_INFO("prob failed\n");

    return  -ENODEV;

}

//resume
static int hall_device_resume(struct device *dev)
{
	struct hall_device_chip *chip = dev_get_drvdata(dev);

    SENSOR_LOG_INFO("enter\n");
    if (true==chip->enabled)
    {
        disable_irq_wake(chip->irq_s.irq_num);
        disable_irq_wake(chip->irq_n.irq_num);
    }
    SENSOR_LOG_INFO("eixt\n");
    return 0 ;
}

//suspend
static int hall_device_suspend(struct device *dev)
{
	struct hall_device_chip *chip = dev_get_drvdata(dev);

    SENSOR_LOG_INFO("enter\n");
    if (true==chip->enabled)
    {
        enable_irq_wake(chip->irq_s.irq_num);
        enable_irq_wake(chip->irq_n.irq_num);
    }
    SENSOR_LOG_INFO("eixt\n");
    return 0 ;
}


 /**
  * hall_device_remove() - remove device
  * @client: I2C client device
  */
 static int hall_device_remove(struct i2c_client *client)
 {
     struct shtc1_data *chip_data = i2c_get_clientdata(client);

     SENSOR_LOG_INFO("hall_device_remove\n");

     kfree(chip_data);
     return 0;
 }


MODULE_DEVICE_TABLE(i2c, hall_device_idtable);

module_init(hall_device_init);
module_exit(hall_device_exit);

MODULE_DESCRIPTION("hall_pair_out driver");
MODULE_AUTHOR("ZhuBing, ZTEMT");
MODULE_LICENSE("GPL");
