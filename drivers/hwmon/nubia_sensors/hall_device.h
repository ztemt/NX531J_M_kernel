#ifndef __HALL_DEVICE_PAIR_OUT_H
#define __HALL_DEVICE_PAIR_OUT_H

#include <linux/types.h>
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
#include <linux/wakelock.h>


#define MAGNETIC_DEVICE_NEAR      1  //Near
#define MAGNETIC_DEVICE_FAR       2  //Far
#define MAGNETIC_DEVICE_UNKNOW   -1  //Unknow

struct hall_device_irq {
    unsigned int irq_num;
    unsigned int irq_pin;
    bool enabled;
};

struct hall_device_wake_lock{
    struct wake_lock lock;
   // bool   locked;
    char   *name;
};
/***ztemt add start,2015.11.23***/
struct hall_device_pinctrl {
	struct pinctrl  *pinctrl;
	struct pinctrl_state *pin_active;
	struct pinctrl_state *pin_suspend;
};
/***ztemt add end***/

struct hall_device_chip {
	struct mutex lock;
	struct i2c_client *client;

    struct hall_device_irq irq_s;
    struct hall_device_irq irq_n;

	struct work_struct irq_work_s;
	struct work_struct irq_work_n;

	struct delayed_work flush_work;

	struct input_dev *idev;

    struct device *hall_device_dev;

    struct hall_device_wake_lock wakeup_wakelock;

	//struct hrtimer unlock_wakelock_timer;

    bool enabled;
   // bool on_irq_working;

    unsigned int state_s;
    unsigned int state_n;

    unsigned int hall_hw_device_count;
    struct regulator	*vdd;
    int vdd_max;
    int vdd_min;
    struct hall_device_pinctrl  pinctrl_info;
};


static void hall_device_check_state(struct hall_device_chip *chip);
static void hall_device_enable(struct hall_device_chip *chip, int on);
static ssize_t hall_device_enable_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t hall_device_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
static void hall_device_irq_enable(struct hall_device_irq * irq, bool enable, bool flag_sync);
static irqreturn_t hall_device_irq_n(int irq, void *handle);
static irqreturn_t hall_device_irq_s(int irq, void *handle);
static void hall_device_irq_work_n(struct work_struct *work);
static void hall_device_irq_work_s(struct work_struct *work);
static int create_sysfs_interfaces(struct device *dev);
static int hall_device_suspend(struct device *dev);
static int hall_device_resume(struct device *dev);
static int hall_device_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int hall_device_remove(struct i2c_client *client);
static int __init hall_device_init(void);
static void __exit hall_device_exit(void);
static void hall_device_chip_data_init(struct hall_device_chip *chip);
//static void hall_device_wakelock_ops(struct hall_device_wake_lock *wakelock, bool enable);



#endif /* __HALL_DEVICE_PAIR_OUT_H */
