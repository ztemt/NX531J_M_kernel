/*
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
 */

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

#define LOG_TAG  "IR_REMOTE"
#define DEBUG_ON

#define SENSOR_LOG_FILE__ strrchr(__FILE__, '/') ? (strrchr(__FILE__, '/') + 1) : __FILE__

#ifdef CONFIG_FEATURE_NUBIA_SENSORS_LOG_ON
#define SENSOR_LOG_ERROR(fmt, args...) printk(KERN_ERR "[%s] [%s: %d] " fmt,\
						LOG_TAG,__FUNCTION__, __LINE__, ##args)
	#ifdef DEBUG_ON
#define SENSOR_LOG_INFO(fmt, args...) printk(KERN_INFO "[%s] [%s: %d] " fmt,\
						LOG_TAG,__FUNCTION__, __LINE__, ##args)

#define SENSOR_LOG_DEBUG(fmt, args...) printk(KERN_DEBUG "[%s] [%s: %d] " fmt,\
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

/* POWER SUPPLY VOLTAGE RANGE */
#define IR_REMOTE_VDD_MIN_UV 1800000
#define IR_REMOTE_VDD_MAX_UV 1800000

static dev_t const i_three_ir_device_dev_t = MKDEV(MISC_MAJOR, 254);

struct i_three_chip {
	struct i2c_client *client;
	struct regulator *power;
	bool power_on;
	struct class *ir_remote;
	struct device *ir_class_dev;
};

static int i_three_power_init(struct i_three_chip *chip)
{
	int rc;

	chip->power = regulator_get(&(chip->client->dev), "vdd");
	if (IS_ERR(chip->power)) {
		rc = PTR_ERR(chip->power);
		SENSOR_LOG_ERROR("Regulator get failed chip->power rc=%d\n", rc);
		goto error;
	}

	if (regulator_count_voltages(chip->power) > 0) {
		rc = regulator_set_voltage(chip->power,
				IR_REMOTE_VDD_MIN_UV, IR_REMOTE_VDD_MAX_UV);
		if (rc) {
			SENSOR_LOG_ERROR("Regulator set chip->power failed rc=%d\n", rc);
			goto error_set_voltage;
		}
	}

	rc = regulator_set_optimum_mode(chip->power, 600000);
	if (rc < 0) {
		SENSOR_LOG_ERROR("Regulator chip->power set_opt failed rc=%d\n", rc);
		goto error_set_optimum;
	}

	SENSOR_LOG_INFO("power init success\n");
	return 0;

error_set_optimum:
	regulator_set_voltage(chip->power, 0, IR_REMOTE_VDD_MAX_UV);
	regulator_put(chip->power);
error_set_voltage:
	regulator_put(chip->power);
error:
	SENSOR_LOG_INFO("power init failed\n");
	return rc;
}

static void i_three_power_deinit(struct i_three_chip *chip)
{
	regulator_put(chip->power);
}

static int i_three_power_on(struct i_three_chip *chip, bool enable)
{
	int rc;

	if (enable == chip->power_on) {
		SENSOR_LOG_INFO("double %s power, retern here\n",
				enable ? "enable" : "disable");
		return 0;
	}
	else {
		SENSOR_LOG_INFO("%s power\n", enable ? "enable" : "disable");
	}

	if (enable) {
		rc = regulator_enable(chip->power);
		if (rc) {
			SENSOR_LOG_ERROR("Regulator chip->power enable failed rc=%d\n", rc);
			goto err_power_enable_failed;
		}
		chip->power_on = true;
	}
	else {
		rc = regulator_disable(chip->power);
		if (rc) {
			SENSOR_LOG_ERROR("Regulator chip->power enable failed rc=%d\n", rc);
			goto err_power_disable_failed;
		}
		chip->power_on = false;
	}

	return 0;

err_power_disable_failed:
err_power_enable_failed:
	return rc;
}

static void i_three_chip_data_init(struct i_three_chip *chip)
{
	chip->power_on = false;
}

static ssize_t attr_enable_ir_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct i_three_chip *chip = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", chip->power_on);
}

static ssize_t attr_enable_ir_store(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	struct i_three_chip *chip = dev_get_drvdata(dev);
	unsigned long val;
	int ret;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	ret = i_three_power_on(chip, !!val);
	if (ret)
		SENSOR_LOG_ERROR("power %s failed\n", val ? "on" : "off");

	return size;
}

static struct device_attribute i_three_ir_attributes[] = {
	__ATTR(enable, 0644, attr_enable_ir_show,
			attr_enable_ir_store),
	__ATTR_NULL,
};

static int create_device_attributes(struct device *dev,
	struct device_attribute *attrs)
{
	int i;
	int err = 0;

	for (i = 0; NULL != attrs[i].attr.name; ++i) {
		err = device_create_file(dev, &attrs[i]);
		if (err)
			break;
	}

	if (err) {
		for (--i; i >= 0; --i)
			device_remove_file(dev, &attrs[i]);
	}

	return err;
}

static void remove_device_attributes(struct device *dev,
	struct device_attribute *attrs)
{
	int i;

	for (i = 0; NULL != attrs[i].attr.name; ++i)
		device_remove_file(dev, &attrs[i]);
}

static int create_sysfs_interfaces(struct i_three_chip *chip)
{
	int err = 0;

	chip->ir_remote = class_create(THIS_MODULE, "ir_remote");
	if (IS_ERR(chip->ir_remote)) {
		err = PTR_ERR(chip->ir_remote);
		goto exit_class_create_failed;
	}

	chip->ir_class_dev = device_create(
					chip->ir_remote,
					NULL,
					i_three_ir_device_dev_t,
					chip,
					"ir_remote");
	if (IS_ERR(chip->ir_class_dev)) {
		err = PTR_ERR(chip->ir_class_dev);
		goto exit_class_device_create_failed;
	}

	err = create_device_attributes(
			chip->ir_class_dev,
			i_three_ir_attributes);
	if (err)
		goto exit_device_attributes_create_failed;

	return err;

exit_device_attributes_create_failed:
	device_destroy(chip->ir_remote, i_three_ir_device_dev_t);
exit_class_device_create_failed:
	class_destroy(chip->ir_remote);
exit_class_create_failed:
	return err;
}

static void remove_sysfs_interfaces(struct i_three_chip *chip)
{
	remove_device_attributes(chip->ir_class_dev,
			i_three_ir_attributes);
	device_destroy(chip->ir_remote,
			i_three_ir_device_dev_t);
	class_destroy(chip->ir_remote);
}

static int i_three_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	static struct i_three_chip *chip;
	int ret;

	SENSOR_LOG_INFO("prob start\n");

	chip = kzalloc(sizeof(struct i_three_chip), GFP_KERNEL);
	if (!chip) {
		ret = -ENOMEM;
		goto malloc_chip_failed;
	}

	i_three_chip_data_init(chip);

	chip->client = client;
	i2c_set_clientdata(client, chip);

	ret = create_sysfs_interfaces(chip);
	if (ret < 0) {
		SENSOR_LOG_ERROR("create sysfs failed\n");
		goto err_create_sysfs;
	}

	ret = i_three_power_init(chip);
	if (ret) {
		SENSOR_LOG_ERROR("power init failed\n");
		goto power_init_failed;
	}

	ret = i_three_power_on(chip, true);
	if (ret) {
		SENSOR_LOG_ERROR("power on failed\n");
		goto power_on_failed;
	}

	SENSOR_LOG_INFO("prob success\n");
	return 0;

power_on_failed:
	i_three_power_deinit(chip);
power_init_failed:
	remove_sysfs_interfaces(chip);
err_create_sysfs:
	kfree(chip);
malloc_chip_failed:
	SENSOR_LOG_INFO("prob failed\n");
	return ret;
}

/*
 * i_three_remove() - remove device
 * @client: I2C client device
 */
static int i_three_remove(struct i2c_client *client)
{
	struct i_three_chip *chip = i2c_get_clientdata(client);

	i_three_power_deinit(chip);
	kfree(chip);
	chip = NULL;
	SENSOR_LOG_INFO("i_three_remove\n");
	return 0;
}

static int i_three_resume(struct device *dev)
{
	struct i_three_chip *chip = dev_get_drvdata(dev);

	SENSOR_LOG_INFO("enter\n");
	i_three_power_on(chip, true);
	SENSOR_LOG_INFO("eixt\n");

	return 0;
}

static int i_three_suspend(struct device *dev)
{
	struct i_three_chip *chip = dev_get_drvdata(dev);

	SENSOR_LOG_INFO("enter\n");
	i_three_power_on(chip, false);
	SENSOR_LOG_INFO("eixt\n");

	return 0;
}

static const struct dev_pm_ops i_three_pm_ops = {
	.suspend = i_three_suspend,
	.resume  = i_three_resume,
};

static const struct i2c_device_id i_three_idtable_id[] = {
	{"uei,i_three", 0},
	{},
};

static struct of_device_id of_i_three_idtable[] = {
	{.compatible = "uei,i_three",},
	{}
};

MODULE_DEVICE_TABLE(i2c, i_three_idtable);

static struct i2c_driver i_three_driver = {
	.driver = {
		.name = "i_three",
		.of_match_table = of_i_three_idtable,
		.pm = &i_three_pm_ops,
	},
	.id_table = i_three_idtable_id,
	.probe = i_three_probe,
	.remove = i_three_remove,
};

static int __init i_three_init(void)
{
	SENSOR_LOG_INFO("driver: init\n");
	return i2c_add_driver(&i_three_driver);
}

static void __exit i_three_exit(void)
{
	SENSOR_LOG_INFO("driver: exit\n");
	i2c_del_driver(&i_three_driver);
}

module_init(i_three_init);
module_exit(i_three_exit);

MODULE_DESCRIPTION("UEI i_three driver");
MODULE_LICENSE("GPL");
