/* drivers/input/touchscreen/gt1x.c
 *
 * 2010 - 2014 Goodix Technology.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * Version: 1.0
 * Revision Record:
 *      V1.0:  first release. 2014/09/28.
 *
 */

#include <linux/irq.h>
#include "gt1x.h"
#include "gt1x_generic.h"

#ifdef CONFIG_OF
#include <linux/err.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#endif

#if GTP_ICS_SLOT_REPORT
#include <linux/input/mt.h>
#endif

/*** ZTEMT Add, 2015/01/09 ***/
#ifdef CONFIG_ZTEMT_HW_VERSION
extern int ztemt_get_hw_id(void);
#endif
/*** ZTEMT end ***/

static s32 gt1x_halt = 0;
static struct work_struct gt1x_work;
static struct input_dev *input_dev;
static struct workqueue_struct *gt1x_wq;
static const char *gt1x_ts_name = "goodix-ts";
static const char *input_dev_phys = "input/ts";

extern struct gt1x_version_info gt1x_version;
extern u8 gt1x_wakeup_gesture;

/*0 stands for hand mode, 1 stands for glove mode added by ztemt 2015.01.01*/
u8 gt1x_touch_mode = 0;

/*** ZTEMT start 20141218***/
#ifdef CONFIG_FB
struct notifier_block fb_notif;
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data);
#elif defined CONFIG_HAS_EARLYSUSPEND
static void gt1x_ts_early_suspend(struct early_suspend *h);
static void gt1x_ts_late_resume(struct early_suspend *h);
#endif
/*ZTEMT end*/

#define GOODIX_PINCTRL_STATE_SLEEP "gt1x_int_sleep"
#define GOODIX_PINCTRL_STATE_DEFAULT "gt1x_int_default"

/*open/short test*/
extern s32 gtp_test_sysfs_init(void);
extern void gtp_test_sysfs_deinit(void);

/**
 * gt1x_i2c_write - i2c write.
 * @addr: register address.
 * @buffer: data buffer.
 * @len: the bytes of data to write.
 *Return: 0: success, otherwise: failed
 */
s32 gt1x_i2c_write(u16 addr, u8 * buffer, s32 len)
{
	struct i2c_msg msg = {
		.flags = 0,
		.addr = gt1x_i2c_client->addr,
	};
	return _do_i2c_write(&msg, addr, buffer, len);
}

/**
 * gt1x_i2c_read - i2c read.
 * @addr: register address.
 * @buffer: data buffer.
 * @len: the bytes of data to write.
 *Return: 0: success, otherwise: failed
 */
s32 gt1x_i2c_read(u16 addr, u8 * buffer, s32 len)
{
	u8 addr_buf[GTP_ADDR_LENGTH] = { (addr >> 8) & 0xFF, addr & 0xFF };
	struct i2c_msg msgs[2] = {
		{
		 .addr = gt1x_i2c_client->addr,
		 .flags = 0,
		 .buf = addr_buf,
		 .len = GTP_ADDR_LENGTH},
		{
		 .addr = gt1x_i2c_client->addr,
		 .flags = I2C_M_RD}
	};
	return _do_i2c_read(msgs, addr, buffer, len);
}

static spinlock_t irq_lock;
static s32 irq_is_disable = 0;
static s32 irq_is_wake = 0;

/**
 * gt1x_irq_enable - enable irq function.
 *
 */
void gt1x_irq_enable(void)
{
	unsigned long irqflags = 0;

	GTP_DEBUG_FUNC();

	spin_lock_irqsave(&irq_lock, irqflags);
	if (irq_is_disable) {
		enable_irq(gt1x_i2c_client->irq);
		irq_is_disable = 0;
	}
	spin_unlock_irqrestore(&irq_lock, irqflags);
}

/**
 * gt1x_irq_enable - disable irq function.
 *
 */
void gt1x_irq_disable(void)
{
	unsigned long irqflags;

	GTP_DEBUG_FUNC();

	spin_lock_irqsave(&irq_lock, irqflags);
	if (!irq_is_disable) {
		irq_is_disable = 1;
		disable_irq_nosync(gt1x_i2c_client->irq);
	}
	spin_unlock_irqrestore(&irq_lock, irqflags);
}

/**
 * gt1x_irq_wake_enable - enable irq wake function.
 *
 */
void gt1x_irq_wake_enable(void)
{
	unsigned long irqflags = 0;

	GTP_DEBUG_FUNC();

	spin_lock_irqsave(&irq_lock, irqflags);
	if (!irq_is_wake) {
		enable_irq_wake(gt1x_i2c_client->irq);
		irq_is_wake = 1;
	}
	spin_unlock_irqrestore(&irq_lock, irqflags);
}

/**
 * gt1x_irq_wake_disable - disable irq wake function.
 *
 */
void gt1x_irq_wake_disable(void)
{
	unsigned long irqflags;

	GTP_DEBUG_FUNC();

	spin_lock_irqsave(&irq_lock, irqflags);
	if (irq_is_wake) {
		irq_is_wake = 0;
		disable_irq_wake(gt1x_i2c_client->irq);
	}
	spin_unlock_irqrestore(&irq_lock, irqflags);
}

void gt1x_power_switch(s32 state)
{
}

int gt1x_debug_proc(u8 * buf, int count)
{
	return -1;
}

#if GTP_CHARGER_SWITCH
u32 gt1x_get_charger_status(void)
{
#error Need to get charger status of your platform.
}
#endif

/**
 * gt1x_ts_irq_handler - External interrupt service routine for interrupt mode.
 * @irq:  interrupt number.
 * @dev_id: private data pointer.
 * Return: Handle Result.
 *  		IRQ_HANDLED: interrupt handled successfully
 */
static irqreturn_t gt1x_ts_irq_handler(int irq, void *dev_id)
{
	GTP_DEBUG_FUNC();
	queue_work(gt1x_wq, &gt1x_work);
	return IRQ_HANDLED;
}

/**
 * gt1x_touch_down - Report touch point event .
 * @id: trackId
 * @x:  input x coordinate
 * @y:  input y coordinate
 * @w:  input pressure
 * Return: none.
 */
void gt1x_touch_down(s32 x, s32 y, s32 size, s32 id)
{
#if GTP_CHANGE_X2Y
	GTP_SWAP(x, y);
#endif

#if GTP_ICS_SLOT_REPORT
	input_mt_slot(input_dev, id);
	input_report_abs(input_dev, ABS_MT_PRESSURE, size);
	input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, size);
	input_report_abs(input_dev, ABS_MT_TRACKING_ID, id);
	input_report_abs(input_dev, ABS_MT_POSITION_X, x);
	input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
#else
	input_report_key(input_dev, BTN_TOUCH, 1);
	if ((!size) && (!id)) {
		/* for virtual button */
		input_report_abs(input_dev, ABS_MT_PRESSURE, 100);
		input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, 100);
	} else {
		input_report_abs(input_dev, ABS_MT_PRESSURE, size);
		input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, size);
		input_report_abs(input_dev, ABS_MT_TRACKING_ID, id);
	}
	input_report_abs(input_dev, ABS_MT_POSITION_X, x);
	input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
	input_mt_sync(input_dev);
#endif
}

/**
 * gt1x_touch_up -  Report touch release event.
 * @id: trackId
 * Return: none.
 */
void gt1x_touch_up(s32 id)
{
#if GTP_ICS_SLOT_REPORT
	input_mt_slot(input_dev, id);
	input_report_abs(input_dev, ABS_MT_TRACKING_ID, -1);
#else
	input_report_key(input_dev, BTN_TOUCH, 0);
	input_mt_sync(input_dev);
#endif
}

/**
 * gt1x_ts_work_func - Goodix touchscreen work function.
 * @iwork: work struct of gt1x_workqueue.
 * Return: none.
 */
static void gt1x_ts_work_func(struct work_struct *work)
{
	u8 end_cmd = 0;
	u8 finger = 0;
	s32 ret = 0;
	u8 point_data[11] = { 0 };

#if GTP_GESTURE_WAKEUP
	if (gt1x_wakeup_gesture) {
		ret = gesture_event_handler(input_dev);
		if (ret >= 0) {
			goto exit_work_func;
		}
	}
#endif

	if (gt1x_halt) {
		GTP_DEBUG("return for interrupt after suspend...  ");
		return;
	}

	ret = gt1x_i2c_read(GTP_READ_COOR_ADDR, point_data, sizeof(point_data));
	if (ret < 0) {
		GTP_ERROR("I2C transfer error!");
#if !GTP_ESD_PROTECT
		gt1x_power_reset();
#endif
		goto exit_work_func;
	}

	finger = point_data[0];
	if (finger == 0x00) {
		gt1x_request_event_handler();
	}

	if ((finger & 0x80) == 0) {
#if HOTKNOT_BLOCK_RW
		if (!hotknot_paired_flag)
#endif
		{
			GTP_ERROR("buffer not ready:0x%02x", finger);
			return;
		}
	}
#if HOTKNOT_BLOCK_RW
	ret = hotknot_event_handler(point_data);
	if (!ret) {
		goto exit_work_func;
	}
#endif

#if GTP_PROXIMITY
	ret = gt1x_prox_event_handler(point_data);
	if (ret > 0) {
		goto exit_work_func;
	}
#endif

#if GTP_WITH_STYLUS
	ret = gt1x_touch_event_handler(point_data, input_dev, pen_dev);
#else
	ret = gt1x_touch_event_handler(point_data, input_dev, NULL);
#endif
	if (ret) {
		return;
	}

exit_work_func:
	if (!gt1x_rawdiff_mode) {
		ret = gt1x_i2c_write(GTP_READ_COOR_ADDR, &end_cmd, 1);
		if (ret < 0) {
			GTP_INFO("I2C write end_cmd  error!");
		}
	}
}

/**
 * gt1x_request_io_port - Request gpio(INT & RST) ports.
 */
static s32 gt1x_request_io_port(void)
{
	s32 ret = 0;

	GTP_DEBUG_FUNC();
	ret = GTP_GPIO_REQUEST(GTP_INT_PORT, "GTP_INT_IRQ");
	if (ret < 0) {
		GTP_ERROR("Failed to request GPIO:%d, ERRNO:%d", (s32) GTP_INT_PORT, ret);
		ret = -ENODEV;
		goto error1;
	} else {
		GTP_GPIO_AS_INT(GTP_INT_PORT);
		gt1x_i2c_client->irq = GTP_INT_IRQ;
	}

	ret = GTP_GPIO_REQUEST(GTP_RST_PORT, "GTP_RST_PORT");
	if (ret < 0) {
		GTP_ERROR("Failed to request GPIO:%d, ERRNO:%d", (s32) GTP_RST_PORT, ret);
		ret = -ENODEV;
		goto error2;
	} else {
		GTP_GPIO_AS_INPUT(GTP_RST_PORT);
	}

	return ret;

error2:
	GTP_GPIO_FREE(GTP_INT_PORT);
error1:
	return ret;
}

/**
 * gt1x_request_irq - Request interrupt.
 * Return
 *      0: succeed, -1: failed.
 */
s32 gt1x_request_irq(void)
{
	s32 ret = -1;
	const u8 irq_table[] = GTP_IRQ_TAB;

	GTP_DEBUG_FUNC();
	GTP_DEBUG("INT trigger type:%x", gt1x_int_type);

	ret = request_irq(gt1x_i2c_client->irq, gt1x_ts_irq_handler, irq_table[gt1x_int_type], gt1x_i2c_client->name, gt1x_i2c_client);
	if (ret) {
		GTP_ERROR("Request IRQ failed!ERRNO:%d.", ret);
		GTP_GPIO_AS_INPUT(GTP_INT_PORT);
		GTP_GPIO_FREE(GTP_INT_PORT);

		return -1;
	} else {
		gt1x_irq_disable();
		return 0;
	}
}

/**
 * gt1x_request_input_dev -  Request input device Function.
 * Return
 *      0: succeed, -1: failed.
 */
static s8 gt1x_request_input_dev(void)
{
	s8 ret = -1;
#if GTP_HAVE_TOUCH_KEY
	u8 index = 0;
#endif

	GTP_DEBUG_FUNC();

	input_dev = input_allocate_device();
	if (input_dev == NULL) {
		GTP_ERROR("Failed to allocate input device.");
		return -ENOMEM;
	}

	input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
#if GTP_ICS_SLOT_REPORT
	input_mt_init_slots(input_dev, 16, 0);	// in case of "out of memory"
#else
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
#endif
	set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

#if GTP_HAVE_TOUCH_KEY
	for (index = 0; index < GTP_MAX_KEY_NUM; index++) {
		input_set_capability(input_dev, EV_KEY, gt1x_touch_key_array[index]);
	}
#endif

#if GTP_GESTURE_WAKEUP
	input_set_capability(input_dev, EV_KEY, KEY_F5);
	input_set_capability(input_dev, EV_KEY, KEY_F6);
	input_set_capability(input_dev, EV_KEY, KEY_F7);
	input_set_capability(input_dev, EV_KEY, KEY_F8);
	input_set_capability(input_dev, EV_KEY, KEY_F9);
	input_set_capability(input_dev, EV_KEY, KEY_F10);
#endif

#if GTP_CHANGE_X2Y
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, gt1x_abs_y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, gt1x_abs_x_max, 0, 0);
#else
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, gt1x_abs_x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, gt1x_abs_y_max, 0, 0);
#endif
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 255, 0, 0);

	input_dev->name = gt1x_ts_name;
	input_dev->phys = input_dev_phys;
	input_dev->id.bustype = BUS_I2C;
	input_dev->id.vendor = 0xDEAD;
	input_dev->id.product = 0xBEEF;
	input_dev->id.version = 10427;

	ret = input_register_device(input_dev);
	if (ret) {
		GTP_ERROR("Register %s input device failed", input_dev->name);
		return -ENODEV;
	}
/*** ZTEMT start 20141218***/
#if defined(CONFIG_FB)
        fb_notif.notifier_call = fb_notifier_callback;
        ret = fb_register_client(&fb_notif);
        if (ret)
            GTP_ERROR("Unable to register fb_notifier: %d\n",ret);
#endif
/*** ZTEMT end***/

	return 0;
}

/*ZTEMT Added by luochangyang, 2014/01/08*/
#define VCC_I2C
/*ZTEMT Added by luochangyang, 2014/01/08*/
#ifdef CONFIG_OF
static int reg_set_optimum_mode_check(struct regulator *reg, int load_uA)
{
	return (regulator_count_voltages(reg) > 0) ?
		regulator_set_optimum_mode(reg, load_uA) : 0;
}

static int goodix_power_on(struct device *dev)
{
	int rc;
    static struct regulator *vcc_ana;
#if defined (VCC_I2C)
    static struct regulator *vcc_i2c;
#endif
/*** ZTEMT Add, 2015/01/09 ***/
#ifdef CONFIG_ZTEMT_HW_VERSION
    int ztemt_hw_bl_id = 0;
#endif
/*** ZTEMT end ***/

	vcc_ana = regulator_get(dev, "vdd_ana");
	if (IS_ERR(vcc_ana))
    {
		rc = PTR_ERR(vcc_ana);
		dev_err(dev, "Regulator get failed vcc_ana rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(vcc_ana) > 0)
    {
		rc = regulator_set_voltage(vcc_ana, 2850000, 3100000); //nubia for tp voltage
		if (rc)
        {
			dev_err(dev, "Regulator set ana vtg failed rc=%d\n", rc);
			goto error_set_vtg_vcc_ana;
		}
	}

    rc = reg_set_optimum_mode_check(vcc_ana, 15000);
    if (rc < 0)
    {
        dev_err(dev, "Regulator vcc_ana set_opt failed rc=%d\n", rc);
        return rc;
    }

    rc = regulator_enable(vcc_ana);
    if (rc)
    {
        dev_err(dev, "Regulator vcc_ana enable failed rc=%d\n", rc);
        goto error_reg_en_vcc_ana;
    }
#if defined (VCC_I2C)
/*** ZTEMT Add, 2015/01/09 ***/
#ifdef CONFIG_ZTEMT_HW_VERSION
    ztemt_hw_bl_id = ztemt_get_hw_id();
            if(ztemt_hw_bl_id){
                vcc_i2c = regulator_get(dev, "vcc_i2c2");
                dev_err(dev, "ZTEMT  ztemt_get_hw_id = B\n");
            }else{
                vcc_i2c = regulator_get(dev, "vcc_i2c");
                dev_err(dev, "ZTEMT  ztemt_get_hw_id = A\n");
            }

#else
    vcc_i2c = regulator_get(dev, "vcc_i2c");
#endif
/*** ZTEMT end ***/
	if (IS_ERR(vcc_i2c))
    {
		rc = PTR_ERR(vcc_i2c);
		dev_err(dev, "Regulator get failed rc=%d\n", rc);
		goto error_reg_opt_vcc_dig;
	}

	if (regulator_count_voltages(vcc_i2c) > 0)
    {
 		rc = regulator_set_voltage(vcc_i2c, 1800000, 1800000);
		if (rc)
        {
			dev_err(dev, "Regulator set i2c vtg failed rc=%d\n", rc);
			goto error_set_vtg_i2c;
		}
	}

    rc = reg_set_optimum_mode_check(vcc_i2c, 10000);
    if (rc < 0)
    {
        dev_err(dev, "Regulator vcc_i2c set_opt failed rc=%d\n", rc);
        goto error_set_vtg_i2c;
    }

    rc = regulator_enable(vcc_i2c);
    if (rc)
    {
        dev_err(dev, "Regulator vcc_i2c enable failed rc=%d\n", rc);
        goto error_reg_en_vcc_i2c;
    }

#endif
    msleep(100);

    return 0;
#if defined (VCC_I2C)
error_reg_en_vcc_i2c:
    reg_set_optimum_mode_check(vcc_i2c, 0);
error_set_vtg_i2c:
    regulator_put(vcc_i2c);
error_reg_opt_vcc_dig:
    regulator_disable(vcc_ana);
#endif
error_reg_en_vcc_ana:
    reg_set_optimum_mode_check(vcc_ana, 0);
error_set_vtg_vcc_ana:
	regulator_put(vcc_ana);
	return rc;
}
#if 0
static int goodix_parse_dt(struct device *dev,
			struct goodix_ts_platform_data *pdata)
{
	struct device_node *np = dev->of_node;

	/* reset, irq gpio info */
	pdata->reset_gpio = of_get_named_gpio(np, "goodix,reset-gpio", 0);
	pdata->irq_gpio = of_get_named_gpio(np, "goodix,irq-gpio", 0);
    pdata->irq_gpio_flags = IRQF_TRIGGER_FALLING;

	return 0;
}
#endif
#endif
/*ZTEMT END*/
/*[BUGFIX]Add Begin by TCTSZ-LZ 2014-5-5,PR-667466. TP INT pull-up enable.*/
struct gtp_pinctrl_info {
	struct pinctrl *pinctrl;
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_suspend;
};

static struct gtp_pinctrl_info gt9xx_pctrl;
static int gtp_pinctrl_init(struct device *dev)
{
	gt9xx_pctrl.pinctrl = devm_pinctrl_get(dev);

	if (IS_ERR_OR_NULL(gt9xx_pctrl.pinctrl)) {
		pr_err("%s:%d Getting pinctrl handle failed\n",
			__func__, __LINE__);
		return -EINVAL;
	}
	gt9xx_pctrl.gpio_state_active = pinctrl_lookup_state(
					       gt9xx_pctrl.pinctrl,
					       GOODIX_PINCTRL_STATE_DEFAULT);

	if (IS_ERR_OR_NULL(gt9xx_pctrl.gpio_state_active)) {
		pr_err("%s:%d Failed to get the active state pinctrl handle\n",
			__func__, __LINE__);
		return -EINVAL;
	}
	gt9xx_pctrl.gpio_state_suspend = pinctrl_lookup_state(
						gt9xx_pctrl.pinctrl,
						GOODIX_PINCTRL_STATE_SLEEP);

	if (IS_ERR_OR_NULL(gt9xx_pctrl.gpio_state_suspend)) {
		pr_err("%s:%d Failed to get the suspend state pinctrl handle\n",
				__func__, __LINE__);
		return -EINVAL;
	}
	return 0;
}
/*[BUGFIX]Add End by TCTSZ-LZ 2014-5-5,PR-667466. TP INT pull-up enable.*/
/**
 * gt1x_ic_ver_show -   ic_ver attribute show added by ztemt 2014.12.09.
 * @dev: i2c device struct.
 * @attr: device attribute.
 * @buf: data buffer.
 * Return data size that writes to the buffer.
 */
static ssize_t gt1x_ic_ver_show (struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	unsigned char config = 0;
	struct gt1x_version_info ver_info;

	const u8 cfg_grp0[] = GTP_CFG_GROUP0;
	const u8 cfg_grp1[] = GTP_CFG_GROUP1_HAND;
	const u8 cfg_grp2[] = GTP_CFG_GROUP2_HAND;
	const u8 cfg_grp3[] = GTP_CFG_GROUP3;
	const u8 cfg_grp4[] = GTP_CFG_GROUP4;
	const u8 cfg_grp5[] = GTP_CFG_GROUP5;

	const u8 *cfgs[] = {
		cfg_grp0, cfg_grp1, cfg_grp2,
		cfg_grp3, cfg_grp4, cfg_grp5
	};

	ret = gt1x_i2c_read(GTP_REG_CONFIG_DATA, &config, 1);
	if (ret < 0) {
		GTP_ERROR("Get IC's config data failed");
		return 0;
	}

	ret = gt1x_read_version(&ver_info);
	if (ret != 0) {
		GTP_ERROR("Get version failed");
		return 0;
	}

	/*use IC default firmware, so firmware versions of IC and driver are the same.*/
	return sprintf(buf,
		"GT%s in  TP IC VER: 0x%06X, CFG:0x%X, SENSORID:%d\n"
		"GT%s in Driver VER: 0x%06X, CFG:0x%X, SENSORID:%d\n",
		ver_info.product_id,
		ver_info.patch_id,
		config,
		ver_info.sensor_id,
		ver_info.product_id,
		ver_info.patch_id,
		cfgs[ver_info.sensor_id][0],
		ver_info.sensor_id
		);
}

/**
 * gt1x_touch_mode_show -   touch_mode attribute show added by ztemt 2015.01.01.
 * @dev: i2c device struct.
 * @attr: device attribute.
 * @buf: data buffer.
 * Return data size that writes to the buffer.
 */
static ssize_t gt1x_touch_mode_show (struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", gt1x_touch_mode);
}

/**
 * gt1x_touch_mode_store -   touch_mode attribute store added by ztemt 2015.01.01.
 * @dev: i2c device struct.
 * @attr: device attribute.
 * @buf: data buffer.
 * @size:data buffer size.
 * Return data size that writes to the buffer.
 */
static ssize_t gt1x_touch_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	u8 recv = 0;
	s32 ret = 0;
	u8 cfg_grp1_hand[] = GTP_CFG_GROUP1_HAND;
	u8 cfg_grp1_glove[] = GTP_CFG_GROUP1_GLOVE;
	u8 cfg_grp1_leather[] = GTP_CFG_GROUP1_LEATHER;
	u8 cfg_grp2_hand[] = GTP_CFG_GROUP2_HAND;
	u8 cfg_grp2_glove[] = GTP_CFG_GROUP2_GLOVE;
	u8 cfg_grp2_leather[] = GTP_CFG_GROUP2_LEATHER;

	if (kstrtou8(buf, 10, &recv))
		return -EINVAL;

	if ((recv != 0) && (recv != 1) && (recv != 2)) {
		GTP_ERROR("touch mode should be 0/1 or 2");
		return -EINVAL;
	}

	if (gt1x_version.sensor_id == 1) {
		if (recv == 0) {
			ret = gt1x_send_cfg(cfg_grp1_hand, CFG_GROUP_LEN(cfg_grp1_hand));
			if (ret) {
				GTP_ERROR("faied to gt1x_send_cfg");
				goto error;
			}

			gt1x_touch_mode = 0;
		} else if (recv == 1) {
			ret = gt1x_send_cfg(cfg_grp1_glove, CFG_GROUP_LEN(cfg_grp1_glove));
			if (ret) {
				GTP_ERROR("faied to gt1x_send_cfg");
				goto error;
			}

			gt1x_touch_mode = 1;
		} else if (recv == 2) {
			ret = gt1x_send_cfg(cfg_grp1_leather, CFG_GROUP_LEN(cfg_grp1_leather));
			if (ret) {
				GTP_ERROR("faied to gt1x_send_cfg");
				goto error;
			}

			gt1x_touch_mode = 2;
		}
	} else if (gt1x_version.sensor_id == 2) {
		if (recv == 0) {
			ret = gt1x_send_cfg(cfg_grp2_hand, CFG_GROUP_LEN(cfg_grp2_hand));
			if (ret) {
				GTP_ERROR("faied to gt1x_send_cfg");
				goto error;
			}

			gt1x_touch_mode = 0;
		} else if (recv == 1) {
			ret = gt1x_send_cfg(cfg_grp2_glove, CFG_GROUP_LEN(cfg_grp2_glove));
			if (ret) {
				GTP_ERROR("faied to gt1x_send_cfg");
				goto error;
			}

			gt1x_touch_mode = 1;
		} else if (recv == 2) {
			ret = gt1x_send_cfg(cfg_grp2_leather, CFG_GROUP_LEN(cfg_grp2_leather));
			if (ret) {
				GTP_ERROR("faied to gt1x_send_cfg");
				goto error;
			}

			gt1x_touch_mode = 2;
		}
	}

	GTP_INFO("success to set touch mode as %d, sensor id = %d",
			gt1x_touch_mode, gt1x_version.sensor_id);

error:
	return size;
}

/**
 * gt1x_wakeup_gesture_show -   wakeup_gesture attribute show added by ztemt 2015.01.03.
 * @dev: i2c device struct.
 * @attr: device attribute.
 * @buf: data buffer.
 * Return data size that writes to the buffer.
 */
static ssize_t gt1x_wakeup_gesture_show (struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", gt1x_wakeup_gesture);
}

/**
 * gt1x_wakeup_gesture_store -   wakeup_gesture attribute store added by ztemt 2015.01.03.
 * @dev: i2c device struct.
 * @attr: device attribute.
 * @buf: data buffer.
 * @size:data buffer size.
 * Return data size that writes to the buffer.
 */
static ssize_t gt1x_wakeup_gesture_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	u8 recv = 0;

	if (kstrtou8(buf, 10, &recv))
		return -EINVAL;
	/*ztemt for uniform sys node interface*/
	if(recv==255)
		recv=0;
	/*ztemt end*/
	if ((recv != 0) && (recv != 1)) {
		GTP_ERROR("wakeup gesture should be 0 or 1");
		return -EINVAL;
	}

	gt1x_wakeup_gesture = recv;

	GTP_INFO("success to set wakeup gesture as %d", gt1x_wakeup_gesture);

	return size;
}

static struct device_attribute gt1x_attrs[] = {
	__ATTR(ic_ver,         0444, gt1x_ic_ver_show,         NULL),
	__ATTR(touch_mode,     0664, gt1x_touch_mode_show,     gt1x_touch_mode_store),
	__ATTR(wakeup_gesture, 0664, gt1x_wakeup_gesture_show, gt1x_wakeup_gesture_store),
};

/**
 * gt1x_create_sysfs_interfaces -   create sysfs interfaces.
 * @client: i2c device struct.
 * Return  0: succeed, -others: failed.
 */
static int gt1x_create_sysfs_interfaces(struct device *dev)
{
	int i = 0;
	int ret = 0;

	for (i = 0; i < ARRAY_SIZE(gt1x_attrs); i++) {
		ret = device_create_file(dev, gt1x_attrs + i);
		if (ret)
			goto error;
	}
	return ret;

error:
	for (; i >= 0; i--)
		device_remove_file(dev, gt1x_attrs + i);

	GTP_ERROR("Unable to create interfaces\n");
	return ret;
}

/**
 * gt1x_remove_sysfs_interfaces -   remove sysfs interfaces.
 * @client: i2c device struct.
 */
static void gt1x_remove_sysfs_interfaces(struct device *dev)
{
	int i = 0;
	for (i = 0; i < ARRAY_SIZE(gt1x_attrs); i++)
		device_remove_file(dev, gt1x_attrs + i);
}

/**
 * gt1x_ts_probe -   I2c probe.
 * @client: i2c device struct.
 * @id: device id.
 * Return  0: succeed, -1: failed.
 */
static int gt1x_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	s32 ret = -1;
#if GTP_AUTO_UPDATE
	struct task_struct *thread = NULL;
#endif
	//do NOT remove these logs
	GTP_INFO("GTP Driver Version: %s", GTP_DRIVER_VERSION);
	//GTP_INFO("GTP Driver Built@%s, %s", __TIME__, __DATE__);
	GTP_INFO("GTP I2C Address: 0x%02x", client->addr);

	gt1x_i2c_client = client;
	spin_lock_init(&irq_lock);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		GTP_ERROR("I2C check functionality failed.");
		return -ENODEV;
	}
/*[BUGFIX]Add Begin by TCTSZ-LZ 2014-5-5,PR-667466. TP INT pull-up enable.*/
    ret = gtp_pinctrl_init(&client->dev);
    if (ret < 0){
		GTP_ERROR("GTP pinctrl init failed.");
        goto pinctrl_init_failed;
    }
    ret = pinctrl_select_state(gt9xx_pctrl.pinctrl,
            gt9xx_pctrl.gpio_state_active);
    if (ret){
		GTP_ERROR("cannot set pin to gpio_state_active state.");
		goto pinctrl_sel_failed;
    }
/*[BUGFIX]Add End by TCTSZ-LZ 2014-5-5,PR-667466. TP INT pull-up enable.*/

	ret = gt1x_request_io_port();
	if (ret < 0) {
		GTP_ERROR("GTP request IO port failed.");
		return ret;
	}

	ret = goodix_power_on(&client->dev);
	if (ret) {
		GTP_ERROR("gt1x power on failed.");
		goto power_on_failed;
	}

	gt1x_power_switch(SWITCH_ON);



	/* select i2c address */
	gt1x_select_addr();
	msleep(10);
	ret = gt1x_i2c_test();
	if(ret){
		GTP_ERROR("i2c_test failed, gt1x probe exit.");
		goto i2c_test_failed;
	}
	ret = gt1x_init();
	if (ret) {
		GTP_ERROR("gt1x init failed. Continue to execute.");
	}

	INIT_WORK(&gt1x_work, gt1x_ts_work_func);

	ret = gt1x_request_input_dev();
	if (ret < 0) {
		GTP_ERROR("GTP request input dev failed");
		goto request_input_failed;
	}

	ret = gt1x_request_irq();
	if (ret < 0) {
		GTP_INFO("GTP works in polling mode.");
	} else {
		GTP_INFO("GTP works in interrupt mode.");
	}

/*
#if GTP_GESTURE_WAKEUP
	enable_irq_wake(client->irq);
#endif
*/

	gt1x_irq_enable();

#if GTP_ESD_PROTECT
	// must before auto update
	gt1x_init_esd_protect();
	gt1x_esd_switch(SWITCH_ON);
#endif

#if GTP_AUTO_UPDATE
	thread = kthread_run(gt1x_auto_update_proc, (void *)NULL, "gt1x_auto_update");
	if (IS_ERR(thread)) {
		ret = PTR_ERR(thread);
		GTP_INFO("Failed to create auto-update thread: %d.", ret);
	}
#endif

	/*add ic_ver attribute added by ztemt 2014.12.09*/
	GTP_INFO("begin to create attribute(s)\n");

	ret = gt1x_create_sysfs_interfaces(&client->dev);
	if (ret) {
		GTP_ERROR("gt1x_create_sysfs_interfaces failed");
		goto create_sys_inf_failed;
	}

	ret = gtp_test_sysfs_init();
	if (ret) {
		GTP_ERROR("gtp_test_sysfs_init failed");
		goto test_sysfs_init_failed;
	}

	return 0;

test_sysfs_init_failed:
	gt1x_remove_sysfs_interfaces(&client->dev);
create_sys_inf_failed:
request_input_failed:
i2c_test_failed:
power_on_failed:
pinctrl_sel_failed:
pinctrl_init_failed:
	GTP_GPIO_FREE(GTP_RST_PORT);
	GTP_GPIO_FREE(GTP_INT_PORT);
	return ret;
}

/**
 * gt1x_ts_remove -  Goodix touchscreen driver release function.
 * @client: i2c device struct.
 * Return  0: succeed, -1: failed.
 */
static int gt1x_ts_remove(struct i2c_client *client)
{
	GTP_DEBUG_FUNC();
	GTP_INFO("GTP driver removing...");

/*** ZTEMT start 20141218***/
#if defined(CONFIG_FB)
        if (fb_unregister_client(&fb_notif))
            GTP_ERROR("Error occurred while unregistering fb_notifier.\n");
#endif
/*ZTEMT END*/

/*
#if GTP_GESTURE_WAKEUP
	disable_irq_wake(client->irq);
#endif
*/

#if GTP_CREATE_WR_NODE
	gt1x_deinit_tool_node();
#endif

#if GTP_ESD_PROTECT
	gt1x_deinit_esd_protect();
#endif

	input_unregister_device(input_dev);

	gt1x_remove_sysfs_interfaces(&client->dev);

	gtp_test_sysfs_deinit();


	return 0;
}

/*** ZTEMT start 20141218***/
#if defined(CONFIG_PM) || defined(CONFIG_FB)
/*** ZTEMT END***/

/**
 * gt1x_ts_suspend - i2c suspend callback function.
 * @dev: i2c device.
 * Return  0: succeed, -1: failed.
 */
static int gt1x_ts_suspend(struct device *dev)
{
	s32 ret = -1;
#if GTP_HOTKNOT && !HOTKNOT_BLOCK_RW
	u8 buf[1] = { 0 };
#endif

	GTP_INFO("TPD suspend start...");

#if GTP_PROXIMITY
	if (gt1x_proximity_flag == 1) {
		GTP_INFO("Suspend: proximity is detected!");
		return 0;
	}
#endif
/*[BUGFIX]Add Begin by TCTSZ-LZ 2014-5-5,PR-667466. TP INT pull-up enable.*/
    ret = pinctrl_select_state(gt9xx_pctrl.pinctrl,
            gt9xx_pctrl.gpio_state_suspend);
    if (ret)
        pr_err("%s:%d cannot set pin to suspend state",
            __func__, __LINE__);
/*[BUGFIX]Add End by TCTSZ-LZ 2014-5-5,PR-667466. TP INT pull-up enable.*/

#if GTP_HOTKNOT
	if (hotknot_enabled) {
#if HOTKNOT_BLOCK_RW
		if (hotknot_paired_flag) {
			GTP_INFO("Suspend: hotknot is paired!");
			return 0;
		}
#else
		gt1x_i2c_read(GTP_REG_HN_PAIRED, buf, sizeof(buf));
		GTP_DEBUG("0x81AA: 0x%02X", buf[0]);
		if (buf[0] == 0x55 || hotknot_transfer_mode) {
			GTP_INFO("Suspend: hotknot is paired!");
			return 0;
		}
#endif
	}
#endif

	gt1x_halt = 1;
#if GTP_ESD_PROTECT
	gt1x_esd_switch(SWITCH_OFF);
#endif
	gt1x_irq_disable();
	cancel_work_sync(&gt1x_work);

/*
#if GTP_GESTURE_WAKEUP
	gesture_clear_wakeup_data();
	if (gesture_enabled) {
		gesture_enter_doze();
		gt1x_irq_enable();
		gt1x_halt = 0;
	} else
#endif
	{
		ret = gt1x_enter_sleep();
		if (ret < 0) {
			GTP_ERROR("GTP early suspend failed.");
		}
	}
*/

	if(gt1x_wakeup_gesture)
		gesture_clear_wakeup_data();

	if ((!gt1x_wakeup_gesture) || (gt1x_wakeup_gesture && !gesture_enabled)) {
		ret = gt1x_enter_sleep();
		if (ret < 0) {
			GTP_ERROR("GTP early suspend failed.");
		}
	} else {
		gesture_enter_doze();
		gt1x_irq_enable();
		gt1x_irq_wake_enable();
		gt1x_halt = 0;
	}

	/* to avoid waking up while not sleeping
	   delay 48 + 10ms to ensure reliability */
	msleep(58);
	return 0;
}

/**
 * gt1x_ts_resume - i2c resume callback function.
 * @dev: i2c device.
 * Return  0: succeed, -1: failed.
 */
static int gt1x_ts_resume(struct device *dev)
{
	s32 ret = -1;

	GTP_INFO("TPD resume start...");

#if GTP_PROXIMITY
	if (gt1x_proximity_flag == 1) {
		GTP_INFO("Resume: proximity is on!");
		return 0;
	}
#endif

#if GTP_HOTKNOT
	if (hotknot_enabled) {
#if HOTKNOT_BLOCK_RW
		if (hotknot_paired_flag) {
			hotknot_paired_flag = 0;
			GTP_INFO("Resume: hotknot is paired!");
			return 0;
		}
#endif
	}
#endif
	/*[BUGFIX]Add Begin by TCTSZ-LZ 2014-5-5,PR-667466. TP INT pull-up enable.*/
		ret = pinctrl_select_state(gt9xx_pctrl.pinctrl,
				gt9xx_pctrl.gpio_state_active);
		if (ret)
			pr_err("%s:%d cannot set pin to suspend state",
				__func__, __LINE__);
	/*[BUGFIX]Add End by TCTSZ-LZ 2014-5-5,PR-667466. TP INT pull-up enable.*/

	ret = gt1x_wakeup_sleep();
	if (ret < 0) {
		GTP_ERROR("GTP later resume failed.");
	}
#if GTP_HOTKNOT
	if (!hotknot_enabled) {
		gt1x_send_cmd(GTP_CMD_HN_EXIT_SLAVE, 0);
	}
#endif

#if GTP_CHARGER_SWITCH
	gt1x_charger_config(0);
	gt1x_charger_switch(SWITCH_ON);
#endif

	gt1x_halt = 0;
	gt1x_irq_enable();

	if(gt1x_wakeup_gesture && gesture_enabled)
		gt1x_irq_wake_disable();

#if GTP_ESD_PROTECT
	gt1x_esd_switch(SWITCH_ON);
#endif

	GTP_DEBUG("tpd resume end.");

	return 0;
}

/*** ZTEMT start 20141218***/
#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;

	if (evdata && evdata->data && event == FB_EVENT_BLANK && gt1x_i2c_client) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK) {
			gt1x_ts_resume(NULL);
			GTP_INFO("ztemt %s: Wake!\n", __func__);
		}
		else if (*blank == FB_BLANK_POWERDOWN) {
			gt1x_ts_suspend(NULL);
			GTP_INFO("ztemt %s: Sleep!\n", __func__);
		}
	}

	return 0;
}
/*** ZTEMT END***/
#elif defined(CONFIG_HAS_EARLYSUSPEND)

static void gt1x_ts_early_suspend(struct early_suspend *h)
{
	gt1x_ts_suspend(NULL);
}

static void gt1x_ts_late_resume(struct early_suspend *h)
{
	gt1x_ts_resume(NULL);
}

struct early_suspend early_suspend;

#endif

static const struct dev_pm_ops gt1x_ts_pm_ops = {
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = NULL,
	.resume = NULL,
#endif
};
#endif

static const struct i2c_device_id gt1x_ts_id[] = {
	{GTP_I2C_NAME, 0},
	{}
};

#ifdef CONFIG_OF                //Open firmware must be defined for dts useage
static struct of_device_id gt1x_match_table[] = {
	{ .compatible = "goodix,gt1x",}, //Compatible node must match dts
    { },
};
#else
#define goodix_match_table NULL
#endif

static struct i2c_driver gt1x_ts_driver = {
	.probe = gt1x_ts_probe,
	.remove = gt1x_ts_remove,
	.id_table = gt1x_ts_id,
	.driver = {
		   .name = GTP_I2C_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = gt1x_match_table,
#ifdef CONFIG_PM
		   .pm = &gt1x_ts_pm_ops,
#endif
#ifdef CONFIG_OF
		   .of_match_table = gt1x_match_table,
#endif
		   },
};

/**
 * gt1x_ts_init - Driver Install function.
 * Return   0---succeed.
 */
static int gt1x_ts_init(void)
{
	s32 ret;

	GTP_DEBUG_FUNC();
	GTP_INFO("GTP driver installing...");
	gt1x_wq = create_singlethread_workqueue("gt1x_wq");
	if (!gt1x_wq) {
		GTP_ERROR("Creat workqueue failed.");
		return -ENOMEM;
	}
#ifdef CONFIG_HAS_EARLYSUSPEND
	early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	early_suspend.suspend = gt1x_ts_early_suspend;
	early_suspend.resume = gt1x_ts_late_resume;
	register_early_suspend(&early_suspend);
#endif

	ret = i2c_add_driver(&gt1x_ts_driver);
	return ret;
}

/**
 * gt1x_ts_exit - Driver uninstall function.
 * Return   0---succeed.
 */
static void gt1x_ts_exit(void)
{
	GTP_DEBUG_FUNC();
	GTP_INFO("GTP driver exited.");
	i2c_del_driver(&gt1x_ts_driver);
	if (gt1x_wq) {
		destroy_workqueue(gt1x_wq);
	}

	gt1x_deinit();

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&early_suspend);
#endif
}

late_initcall(gt1x_ts_init);
module_exit(gt1x_ts_exit);

MODULE_DESCRIPTION("GTP Series Driver");
MODULE_LICENSE("GPL");
