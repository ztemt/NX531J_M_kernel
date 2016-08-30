/* Copyright (c) 2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include "gf_platform.h"

static struct kobject *g_gf_kobj = NULL;
static struct gf_dev *g_gf_dev = NULL;

/* The main reason to have this class is to make mdev/udev create the
 * /dev/dev character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */
static struct class *g_gf_class = NULL;

/* irq related */
static unsigned char irq_is_disable = 0;
static unsigned char irq_is_wake = 0;

static DECLARE_BITMAP(minors, N_PLATFORM_MINORS);
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

#ifdef CONFIG_ZTEMT_FP_COMPATIBLE
static char ztemt_hw_version[10]="*,*";
static char ztemt_hw_version_A[10]="0,0";//pcb version A

static char ztemt_fp_goodix[10] = "0"; // 0 GND goodix ;1 NC egis ;2 VCC FPC
static char ztemt_fp_gpio_target[10] = "*";
static int  ztemt_fp_target_get(char *param)
{
    memcpy(ztemt_fp_gpio_target, param, strlen(param));
    return 0;
}
early_param("fp_target", ztemt_fp_target_get);
static int  ztemt_hw_version_get(char *param)
{
    memcpy(ztemt_hw_version, param, strlen(param));
    return 0;
}
early_param("pcb_setup", ztemt_hw_version_get);
#endif

/* irq related operation */
void gf_irq_op(int irq, int enable)
{
	if (enable && irq_is_disable) {
		irq_is_disable = 0;
		enable_irq(irq);
		GF_LOG_INFO("enable_irq\n");
	} else if (!enable && !irq_is_disable) {
		irq_is_disable = 1;
		disable_irq_nosync(irq);
		GF_LOG_INFO("disable_irq_nosync\n");
	}
}

void gf_irq_wake_op(int irq, int enable)
{
	if (enable && !irq_is_wake) {
		irq_is_wake = 1;
		enable_irq_wake(irq);
		GF_LOG_INFO("enable_irq_wake\n");
	} else if (!enable && irq_is_wake) {
		irq_is_wake = 0;
		disable_irq_wake(irq);
		GF_LOG_INFO("disable_irq_wake\n");
	}
}

void gf_irq_enable(struct gf_dev *gf_dev)
{
	int ret = 0;

	gpio_direction_input(gf_dev->irq_gpio);
	ret = pinctrl_select_state(gf_dev->gf_pctrl.pinctrl,
		gf_dev->gf_pctrl.gpio_int_active);
	gf_irq_op(gf_dev->irq, GF_INT_ENABLE);
}

void gf_irq_disable(struct gf_dev *gf_dev)
{
	gpio_direction_output(gf_dev->irq_gpio, 0);
	gf_irq_op(gf_dev->irq, GF_INT_DISABLE);
}

int gf_parse_dts(struct gf_dev *gf_dev)
{
	u32 temp_val = 0;
 	struct device_node *np = gf_dev->pdev->dev.of_node;

    GF_LOG_INFO("start\n");

    gf_dev->reset_gpio = of_get_named_gpio_flags(np, "gf-gpio-rst", 0, &temp_val);
    if (gf_dev->reset_gpio <= 0) {
        GF_LOG_ERROR("failed to get reset gpio\n");
        return -1;
    }

    gf_dev->enable_gpio = of_get_named_gpio_flags(np, "gf-gpio-enable", 0, &temp_val);
    if (gf_dev->enable_gpio <= 0) {
        GF_LOG_ERROR("failed to get enable gpio\n");
        return -1;
    }

    gf_dev->irq_gpio = of_get_named_gpio_flags(np, "gf-gpio-drdy", 0, &temp_val);
    if(gf_dev->irq_gpio <= 0) {
        GF_LOG_ERROR("failed to get irq gpio\n");
        return -1;
    }

	gf_dev->irq = gpio_to_irq(gf_dev->irq_gpio);

    GF_LOG_INFO("rst = %d, int = %d\n", (int)gf_dev->reset_gpio,
		(int)gf_dev->irq_gpio);

    return 0;
}

int gf_init_gpio(struct gf_dev *gf_dev)
{
    int ret = 0;

    GF_LOG_INFO("start\n");

	ret = gpio_request(gf_dev->enable_gpio, "gf_enable");
	if (ret) {
		GF_LOG_ERROR("failed to request gf_enable gpio %d\n", gf_dev->enable_gpio);
		goto err_request_enable;
	}
	gpio_direction_output(gf_dev->enable_gpio, 1);
	gpio_set_value(gf_dev->enable_gpio, 1);

	msleep(10);

    ret = gpio_request(gf_dev->irq_gpio, "gf_irq");
    if (ret) {
        GF_LOG_ERROR("failed to request gf_irq gpio %d\n", gf_dev->irq_gpio);
        goto err_requeset_irq;
    }
    gpio_direction_output(gf_dev->irq_gpio, 0);
    gpio_direction_input(gf_dev->irq_gpio);

    ret = gpio_request(gf_dev->reset_gpio, "gf_rst");
    if (ret) {
        GF_LOG_ERROR("failed to request gf_rst gpio %d\n", gf_dev->reset_gpio);
        goto err_request_rst;
    }
    gpio_direction_output(gf_dev->reset_gpio, 1);
	gpio_set_value(gf_dev->reset_gpio, 1);

	GF_LOG_INFO("success\n");
	return 0;

err_request_rst:
    gpio_free(gf_dev->irq_gpio);
err_requeset_irq:
	gpio_free(gf_dev->enable_gpio);
err_request_enable:
    return ret;
}

int gf_init_pinctrl(struct gf_dev *gf_dev, struct device *dev)
{
    GF_LOG_INFO("start\n");

	gf_dev->gf_pctrl.pinctrl= devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(gf_dev->gf_pctrl.pinctrl)) {
        GF_LOG_ERROR("failed to get pinctrl handle\n");
        return -EINVAL;
    }

	gf_dev->gf_pctrl.gpio_state_active = pinctrl_lookup_state(gf_dev->gf_pctrl.pinctrl,
		"gf_active");
	if (IS_ERR_OR_NULL(gf_dev->gf_pctrl.gpio_state_active)) {
		GF_LOG_ERROR("failed to get active state pinctrl handle\n");
		return -EINVAL;
	}

	gf_dev->gf_pctrl.gpio_state_suspend = pinctrl_lookup_state(gf_dev->gf_pctrl.pinctrl,
		"gf_suspend");
	if (IS_ERR_OR_NULL(gf_dev->gf_pctrl.gpio_state_suspend)) {
		GF_LOG_ERROR("failed to get suspend state pinctrl handle\n");
		return -EINVAL;
	}

	gf_dev->gf_pctrl.gpio_int_active = pinctrl_lookup_state(gf_dev->gf_pctrl.pinctrl,
		"gf_int_active");
	if (IS_ERR_OR_NULL(gf_dev->gf_pctrl.gpio_int_active)) {
		GF_LOG_ERROR("failed to get int active state pinctrl handle\n");
		return -EINVAL;
	}

    GF_LOG_INFO("success\n");

	return 0;
}

int gf_pinctrl_set(struct gf_dev *gf_dev, bool active)
{
    int ret = 0;

    GF_LOG_INFO("start\n");

    if (active) {
        ret = pinctrl_select_state(gf_dev->gf_pctrl.pinctrl,
			gf_dev->gf_pctrl.gpio_state_active);
    } else {
        ret = pinctrl_select_state(gf_dev->gf_pctrl.pinctrl,
			gf_dev->gf_pctrl.gpio_state_suspend);
    }

    GF_LOG_INFO("set %s = %d\n", active? "avtive" : "suspend", ret);

    return ret;
}

void gf_power_on(struct gf_dev* gf_dev)
{
    if (gpio_is_valid(gf_dev->enable_gpio)) {
        gpio_set_value(gf_dev->enable_gpio, 1);
    }
    msleep(10);

    GF_LOG_INFO("end\n");
}

void gf_power_off(struct gf_dev* gf_dev)
{
    if (gpio_is_valid(gf_dev->enable_gpio)) {
        gpio_set_value(gf_dev->enable_gpio, 0);
    }
    GF_LOG_INFO("\n");
}

void gf_hw_reset(struct gf_dev *gf_dev, unsigned int delay_ms)
{
    GF_LOG_INFO("start\n");
	gpio_set_value(gf_dev->reset_gpio, 1);
	msleep(1);
	gpio_set_value(gf_dev->reset_gpio, 0);
	msleep(6);
	gpio_set_value(gf_dev->reset_gpio, 1);
	msleep(delay_ms);
    GF_LOG_INFO("end\n");
}

#ifdef CONFIG_FB
static int gf_fb_notifier_callback(struct notifier_block *self,
	unsigned long event, void *data)
{
	struct gf_dev *gf_dev = container_of(self, struct gf_dev, fb_notifier);
	struct fb_event *evdata = data;
	int *blank;

	GF_LOG_DEBUG("event = %lu\n", event);

	if (evdata && evdata->data && event == FB_EVENT_BLANK) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK) {
			GF_LOG_INFO("FB_BLANK_UNBLANK\n");
			if (gf_dev->device_available == 1) {
				gf_dev->fb_black = 0;
#ifdef GF_FASYNC
				if (gf_dev->async)
					kill_fasync(&gf_dev->async, SIGIO, POLL_IN);
#endif
				gf_dev->device_available = 1;
			}
		} else if (*blank == FB_BLANK_POWERDOWN) {
		    GF_LOG_INFO("FB_BLANK_POWERDOWN\n");
			if (gf_dev->device_available == 1) {
				gf_dev->fb_black = 1;
#ifdef GF_FASYNC
				if (gf_dev->async)
					kill_fasync(&gf_dev->async, SIGIO, POLL_IN);
#endif
				gf_dev->device_available = 1;
			}
		}
	}

	return 0;
}

static void gf_setup_fb_notifier(struct gf_dev *gf_dev)
{
	int rc = 0;

	gf_dev->fb_notifier.notifier_call = gf_fb_notifier_callback;

	rc = fb_register_client(&gf_dev->fb_notifier);
	if (rc)
		GF_LOG_ERROR("failed to register fb_notifier: %d\n", rc);
}
#endif

static ssize_t gf_wake_up_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "wake up = %d\n", g_gf_dev->wake_up);
}

static ssize_t gf_wake_up_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t size)
{
	int val = 0;

	sscanf(buf, "%d", &val);

	if ((val != 0) && (val != 1)) {
		GF_LOG_ERROR("invalid value\n");
		goto exit;
	}

    g_gf_dev->wake_up = val;

	GF_LOG_INFO("wake up as = %d\n", g_gf_dev->wake_up);

exit:
	return size;
}

static struct kobj_attribute gf_wake_up_attribute =
	__ATTR(wake_up, 0664, gf_wake_up_show, gf_wake_up_store);

static struct attribute *attrs[] = {
	&gf_wake_up_attribute.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = attrs,
};

int gf_sys_init(void)
{
	int retval = 0;

    GF_LOG_INFO("start\n");

	g_gf_kobj = kobject_create_and_add("fingerprint", kernel_kobj);
	if (!g_gf_kobj) {
		GF_LOG_ERROR("failed to create kobj\n");
		return -ENOMEM;
	}

	/* Create the files associated with this kobject */
	retval = sysfs_create_group(g_gf_kobj, &attr_group);
	if (retval)
		kobject_put(g_gf_kobj);

    GF_LOG_INFO("end\n");

	return retval;
}

void gf_sys_uninit(struct kobject *id_kobj)
{
    GF_LOG_INFO("start\n");
	sysfs_remove_group(id_kobj, &attr_group);
	kobject_put(id_kobj);

    GF_LOG_INFO("end\n");
}

static long gf_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct gf_dev *gf_dev = (struct gf_dev *)filp->private_data;
	struct gf_key gf_key = { 0 };
	int retval = 0;

	GF_LOG_DEBUG("start\n");

	if (_IOC_TYPE(cmd) != GF_IOC_TEE_MAGIC)
		return -ENODEV;
	if (_IOC_DIR(cmd) & _IOC_READ)
		retval = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	if ((retval == 0) && (_IOC_DIR(cmd) & _IOC_WRITE))
		retval = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (retval)
		return -EFAULT;

	GF_LOG_DEBUG("cmd = 0x%x\n", cmd);

	if (gf_dev->device_available == 0) {
		if ((cmd == GF_IOC_TEE_POWER_ON) || (cmd == GF_IOC_TEE_POWER_OFF)) {
            GF_LOG_INFO("power cmd\n");
        } else {
            GF_LOG_INFO("already power off\n");
            return -ENODEV;
        }
	}

	switch (cmd) {
	case GF_IOC_TEE_DISABLE_IRQ:
		GF_LOG_INFO("GF_IOC_DISABLE_IRQ\n");
		gf_irq_disable(gf_dev);
		break;
	case GF_IOC_TEE_ENABLE_IRQ:
		GF_LOG_INFO("GF_IOC_ENABLE_IRQ\n");
		gf_irq_enable(gf_dev);
		break;
	case GF_IOC_TEE_SETSPEED:
		break;
	case GF_IOC_TEE_RESET:
		GF_LOG_INFO("GF_IOC_RESET\n");
		gf_hw_reset(gf_dev, 70);
		break;
	case GF_IOC_TEE_COOLBOOT:
		GF_LOG_INFO("GF_IOC_COOLBOOT\n");
		gf_power_off(gf_dev);
		mdelay(5);
		gf_power_on(gf_dev);
		break;
	case GF_IOC_TEE_SENDKEY:
		if (copy_from_user(&gf_key, (struct gf_key *)arg, sizeof(struct gf_key))) {
			GF_LOG_ERROR("failed to copy data from userspace\n");
			retval = -EFAULT;
			break;
		}
		if (0x66 == gf_key.key)
			break; //avoid HOME KEY
		else if (0x74 == gf_key.key) {
			GF_LOG_INFO("GF_IOC_SENDKEY\n");
			gf_key.key = KEY_F11; //change power key to F11
			wake_lock_timeout(&gf_dev->wake_lock, 3 * 1000);
		}
		input_report_key(gf_dev->input, gf_key.key, gf_key.value);
		input_sync(gf_dev->input);
		break;
	case GF_IOC_TEE_CLK_READY:
		break;
	case GF_IOC_TEE_CLK_UNREADY:
		break;
	case GF_IOC_TEE_PM_FBCABCK:
		__put_user(gf_dev->fb_black, (u8 __user *) arg);
		break;
	case GF_IOC_TEE_POWER_ON:
		GF_LOG_INFO("GF_IOC_POWER_ON\n");
		if (gf_dev->device_available == 0)
			gf_power_on(gf_dev);
		gf_dev->device_available = 1;
		break;
	case GF_IOC_TEE_POWER_OFF:
		GF_LOG_INFO("GF_IOC_POWER_OFF\n");
		if(gf_dev->device_available == 1)
			gf_power_off(gf_dev);
		gf_dev->device_available = 0;
		break;
	default:
		retval = -EFAULT;
		GF_LOG_ERROR("unsupport cmd: 0x%x\n", cmd);
		break;
	}

	return retval;
}

#ifdef CONFIG_COMPAT
static long gf_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
     return gf_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#endif

static irqreturn_t gf_irq(int irq, void* handle)
{
    struct gf_dev *gf_dev = (struct gf_dev *)handle;

#ifdef GF_FASYNC
	if (gf_dev->async)
		kill_fasync(&gf_dev->async, SIGIO, POLL_IN);
#endif

    return IRQ_HANDLED;
}

static int gf_open(struct inode *inode, struct file *filp)
{
    struct gf_dev *gf_dev = NULL;
    int	status = -ENXIO;

    GF_LOG_INFO("start\n");

    mutex_lock(&device_list_lock);

    list_for_each_entry(gf_dev, &device_list, device_entry) {
		if(gf_dev->devt == inode->i_rdev) {
		    GF_LOG_DEBUG("Found\n");
		    status = 0;
		    break;
		}
    }

	if (status == 0) {
	    gf_dev->users++;
	    filp->private_data = gf_dev;
	    nonseekable_open(inode, filp);
	    GF_LOG_DEBUG("success to open device. irq = %d\n", gf_dev->irq);
		if (gf_dev->users == 1)
			gf_irq_enable(gf_dev);
        /*power the sensor*/
        gf_power_on(gf_dev);
	    gf_hw_reset(gf_dev, 360);
        gf_dev->device_available = 1;
	} else {
		GF_LOG_ERROR("No device for minor %d\n", iminor(inode));
    }
    mutex_unlock(&device_list_lock);

    GF_LOG_INFO("end\n");
    return status;
}

static int gf_release(struct inode *inode, struct file *filp)
{
    struct gf_dev *gf_dev;
    int status = 0;

    GF_LOG_INFO("start\n");

    mutex_lock(&device_list_lock);

    gf_dev = filp->private_data;
    filp->private_data = NULL;

    /* last close */
    gf_dev->users --;
    if (!gf_dev->users) {
		GF_LOG_DEBUG("disble_irq. irq = %d\n", gf_dev->irq);
		gf_irq_disable(gf_dev);
        /*power off the sensor*/
        gf_dev->device_available = 0;
        gf_power_off(gf_dev);
    }
    mutex_unlock(&device_list_lock);

    GF_LOG_INFO("end\n");
    return status;
}

#ifdef GF_FASYNC
static int gf_fasync(int fd, struct file *filp, int mode)
{
    struct gf_dev *gf_dev = filp->private_data;
    int ret = 0;

    GF_LOG_INFO("start\n");

    ret = fasync_helper(fd, filp, mode, &gf_dev->async);

    GF_LOG_INFO("end\n");
    return ret;
}
#endif

static const struct file_operations gf_fops = {
    .owner =	THIS_MODULE,
    /* REVISIT switch to aio primitives, so that userspace
     * gets more complete API coverage.  It'll simplify things
     * too, except for the locking.
     */
    .unlocked_ioctl = gf_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = gf_compat_ioctl,
#endif
    .open    = gf_open,
    .release = gf_release,
#ifdef GF_FASYNC
    .fasync  = gf_fasync,
#endif
};

static int gf_probe(struct platform_device *pdev)
{
    struct gf_dev *gf_dev = NULL;
	unsigned long minor = 0;
	int ret = 0;

    GF_LOG_INFO("Platform Device start\n");

    /* Allocate driver data */
    gf_dev = kzalloc(sizeof(*gf_dev), GFP_KERNEL);
    if (!gf_dev) {
		GF_LOG_ERROR("failed to alloc memory for gf device\n");
		return -ENOMEM;
    }

	gf_dev->device_available = 0;
	gf_dev->fb_black = 0;

    /* Initialize the driver data */
    gf_dev->pdev = pdev;
	platform_set_drvdata(pdev, gf_dev);
	g_gf_dev = gf_dev;

    wake_lock_init(&gf_dev->wake_lock, WAKE_LOCK_SUSPEND, "goodixfp_wakelock");
    spin_lock_init(&gf_dev->platform_lock);
    INIT_LIST_HEAD(&gf_dev->device_entry);

    ret = gf_parse_dts(gf_dev);
	if (ret) {
		GF_LOG_ERROR("failed to gf_parse_dts\n");
		goto err_dts_parse;
	}

	ret = gf_init_gpio(gf_dev);
    if (ret) {
		GF_LOG_ERROR("failed to gf_init_gpio\n");
        goto err_gpio_init;
    }

    ret = gf_init_pinctrl(gf_dev, &pdev->dev);
	if (ret) {
		GF_LOG_ERROR("failed to gf_init_pinctrl\n");
		goto err_pinctrl_init;
	}

	ret = gf_pinctrl_set(gf_dev, true);
    if (ret) {
        GF_LOG_ERROR("failed to gf_pinctrl_set\n");
        goto err_pinctrl_set;
    }

    ret = gf_sys_init();
	if (ret) {
		GF_LOG_ERROR("failed to gf_sys_init\n");
		goto err_sys_init;
	}

    /* If we can allocate a minor number, hook up this device.
     * Reusing minors is fine so long as udev or mdev is working.
     */
    mutex_lock(&device_list_lock);
    minor = find_first_zero_bit(minors, N_PLATFORM_MINORS);
    if (minor < N_PLATFORM_MINORS) {
    	struct device *dev = NULL;

    	gf_dev->devt = MKDEV(PLATFORMDEV_MAJOR, minor);
    	dev = device_create(g_gf_class, &pdev->dev, gf_dev->devt,
    		gf_dev, DEV_NAME);
    	ret = IS_ERR(dev) ? PTR_ERR(dev) : 0;
    } else {
    	GF_LOG_ERROR("no minor number available\n");
    	ret = -ENODEV;
    }
    if (ret  == 0) {
    	set_bit(minor, minors);
    	list_add(&gf_dev->device_entry, &device_list);
		mutex_unlock(&device_list_lock);
    } else {
		mutex_unlock(&device_list_lock);
		GF_LOG_ERROR("failed to mkdev\n");
		goto err_mkdev;
	}

	/*register device within input system*/
	gf_dev->input = input_allocate_device();
	if (gf_dev->input == NULL) {
	    GF_LOG_ERROR("failed to allocate input device.\n");
	    ret = -ENOMEM;
	    goto err_input_allocate;
	}

	__set_bit(EV_KEY, gf_dev->input->evbit);
	__set_bit(GF_INPUT_HOME_KEY, gf_dev->input->keybit);
	__set_bit(GF_INPUT_MENU_KEY, gf_dev->input->keybit);
	__set_bit(GF_INPUT_BACK_KEY, gf_dev->input->keybit);
	__set_bit(GF_FF_KEY, gf_dev->input->keybit);
	__set_bit(GF_POWER_KEY, gf_dev->input->keybit);

	gf_dev->input->name = "gf318m-key";
	if (input_register_device(gf_dev->input)) {
	    GF_LOG_ERROR("failed to register input device.\n");
		goto err_input_register;
	}

	ret = request_threaded_irq(gf_dev->irq, NULL, gf_irq,
		IRQF_TRIGGER_RISING | IRQF_ONESHOT,	dev_name(&pdev->dev), gf_dev);
	if (ret) {
		GF_LOG_ERROR("failed to request threaded irq\n");
		goto err_request_irq;
	} else {
		gf_irq_disable(gf_dev);
	}

#ifdef CONFIG_FB
	gf_setup_fb_notifier(gf_dev);
#endif

	gf_irq_enable(gf_dev);

    GF_LOG_INFO("success\n");
    return 0;

err_request_irq:
	input_unregister_device(gf_dev->input);
err_input_register:
	input_free_device(gf_dev->input);
err_input_allocate:
	if (gf_dev->devt != 0) {
		mutex_lock(&device_list_lock);
		list_del(&gf_dev->device_entry);
		device_destroy(g_gf_class, gf_dev->devt);
		clear_bit(MINOR(gf_dev->devt), minors);
		mutex_unlock(&device_list_lock);
	}
err_mkdev:
	gf_sys_uninit(g_gf_kobj);
err_sys_init:
err_pinctrl_set:
    devm_pinctrl_put(gf_dev->gf_pctrl.pinctrl);
err_pinctrl_init:
    gpio_free(gf_dev->reset_gpio);
    gpio_free(gf_dev->irq_gpio);
	gpio_free(gf_dev->enable_gpio);
err_gpio_init:
err_dts_parse:
	gf_dev->device_available = 0;
	if (gf_dev) {
	    kfree(gf_dev);
		gf_dev = NULL;
	}

    GF_LOG_INFO("failed\n");
    return ret;
}

static int gf_remove(struct platform_device *pdev)
{
    struct gf_dev *gf_dev = platform_get_drvdata(pdev);

	GF_LOG_INFO("start\n");

    /* make sure ops on existing fds can abort cleanly */
    if (gf_dev->irq) {
		free_irq(gf_dev->irq, gf_dev);
    }

    spin_lock_irq(&gf_dev->platform_lock);
    gf_dev->pdev = NULL;
    platform_set_drvdata(pdev, NULL);
    spin_unlock_irq(&gf_dev->platform_lock);

    /* prevent new opens */
    mutex_lock(&device_list_lock);
    sysfs_remove_group(g_gf_kobj, &attr_group);
	kobject_put(g_gf_kobj);
    list_del(&gf_dev->device_entry);
    device_destroy(g_gf_class, gf_dev->devt);
	clear_bit(MINOR(gf_dev->devt), minors);

    if (gf_dev->users == 0) {
		if (gf_dev->input != NULL) {
		    input_unregister_device(gf_dev->input);
			input_free_device(gf_dev->input);
		}

		if (gf_dev) {
			kfree(gf_dev);
			gf_dev = NULL;
		}
    }
    mutex_unlock(&device_list_lock);

    GF_LOG_INFO("end\n");
    return 0;
}

static int gf_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct gf_dev *gf_dev = platform_get_drvdata(pdev);

    GF_LOG_INFO("start\n");

	if (gf_dev->wake_up == true) {
		gf_irq_wake_op(gf_dev->irq, GF_INT_ENABLE);
	} else {
		gf_irq_disable(gf_dev);
	}

    GF_LOG_INFO("end\n");

    return 0 ;
}

static int gf_resume(struct platform_device *pdev)
{
    struct gf_dev *gf_dev= platform_get_drvdata(pdev);

	GF_LOG_INFO("start\n");

	if (gf_dev->wake_up == true) {
		gf_irq_wake_op(gf_dev->irq, GF_INT_DISABLE);
	} else {
		gf_irq_enable(gf_dev);
	}

	GF_LOG_INFO("end\n");

     return 0;
}

static struct of_device_id of_gf_device_idtable[] = {
	{.compatible = "goodix,gf318m",},
	{}
};

static struct platform_driver gf_platform_driver = {
    .driver = {
		.name =	PLATFORM_DEV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_gf_device_idtable,
    },
    .probe = gf_probe,
    .remove = gf_remove,
    .suspend = gf_suspend,
	.resume = gf_resume,
};

static int __init gf_init(void)
{
    int status = 0;

#ifdef CONFIG_NUBIA_FP_AUTODETECT
    if (fingerprint_device_autodetect(AUTODETECT_NAME)==false) {
        return -ENODEV;
    }
#endif

    /* Claim our 256 reserved device numbers.  Then register a class
     * that will key udev/mdev to add/remove /dev nodes.  Last, register
     * the driver which manages those device numbers.
     */
    BUILD_BUG_ON(N_PLATFORM_MINORS > 256);

    status = register_chrdev(PLATFORMDEV_MAJOR, CHRD_DRIVER_NAME, &gf_fops);
    if (status < 0) {
		GF_LOG_ERROR("failed to register char device\n");
		return status;
    }

    g_gf_class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(g_gf_class)) {
		unregister_chrdev(PLATFORMDEV_MAJOR, gf_platform_driver.driver.name);
		GF_LOG_ERROR("failed to create class\n");
		return PTR_ERR(g_gf_class);
    }

    status = platform_driver_register(&gf_platform_driver);
    if (status < 0) {
		class_destroy(g_gf_class);
		unregister_chrdev(PLATFORMDEV_MAJOR, gf_platform_driver.driver.name);
		GF_LOG_ERROR("failed to register platform driver\n");
    }
    return status;
}
module_init(gf_init);

static void __exit gf_exit(void)
{
    platform_driver_unregister(&gf_platform_driver);
    class_destroy(g_gf_class);
    unregister_chrdev(PLATFORMDEV_MAJOR, gf_platform_driver.driver.name);
}
module_exit(gf_exit);

MODULE_AUTHOR("nubia@zte.com.cn>");
MODULE_DESCRIPTION("gfx18m platform driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("gfx18m platform driver");
