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
#include "gf-spi.h"

static struct kobject *g_gf_kobj = NULL;
static struct gf_dev *g_gf_dev = NULL;

/* The main reason to have this class is to make mdev/udev create the
 * /dev/spidevB.C character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */
static struct class *g_gf_spi_class = NULL;

/* irq related */
static unsigned char irq_is_disable = 0;
static unsigned char irq_is_wake = 0;

static DECLARE_BITMAP(minors, N_SPI_MINORS);
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

/*************************data stream**************************
 *	FRAME NO  | RING_H  | RING_L  |  DATA STREAM | CHECKSUM |
 *     1B      |   1B    |  1B     |    2048B     |  2B      |
 ************************************************************/
static unsigned bufsize = 8 * (2048 + 5);
module_param(bufsize, uint, S_IRUGO);
MODULE_PARM_DESC(bufsize, "data bytes in biggest supported SPI message");

#if FW_UPDATE
static struct config_buf config_buf_list[] = {
    {
		.date = 0x7df051c,
		.buffer = {
		    0x41,0x3c,0x3c,0xe4,0x0c,0x30,0x3f,0x02,0x00,0x50,0x40,0x50,0x50,0xe4,0x0c,0x30,
		    0x2f,0x03,0x00,0x03,0x11,0xa0,0x0d,0x00,0x14,0x03,0x0f,0x0f,0x0f,0xb2,0x3f,0xb3,
		    0x33,0x03,0x90,0x01,0x40,0x05,0x0e,0x80,0x20,0x0f,0x22,0x00,0x08,0x07,0x08,0x06,
		    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x00,0x25,0x04,0xca,0xa4,0x26,0x66,0x00,
		    0x00,0x00,0x01,0x00,0x01,0x0f,0x96,0x00,0x01,0x02,0x85,0x00,0x03,0x20,0x20,0x50,
		    0x3e,0x11,0x01,0x00,0x00,0x00,0x00,0x03,0x09,0x00,0x31,0x00,0x07,0x14,0x41,0x00,
		    0x50,0x00,0x00,0x00,0x20,0x00,0x04,0x00,0x32,0x01,0xa0,0x00,0x00,0x79,0xc8,0x00,
		    0x00,0x00,0x28,0x00,0x05,0x04,0x30,0x00,0x08,0x00,0x07,0x00,0x20,0x00,0x18,0x00,
		    0x3d,0x00,0x48,0x00,0x22,0x00,0x00,0x00,0x03,0x07,0x80,0x00,0x20,0x00,0x20,0x00,
		    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x2c,0x01
		},
	},
};

static struct gf_config config_list[] = {
    {
		.type = 0,
		.pid = 1,
		.config_num = 1,
		.config = &config_buf_list[0],
    }
};

static unsigned char GF_FW[]=
{
	#include "gf_fw.i"
};
#define FW_LENGTH (42 * 1024)

unsigned char *fw = GF_FW;
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

void gf_spi_setup(struct gf_dev *gf_dev, int max_speed_hz)
{
    gf_dev->spi->mode = SPI_MODE_0; //CPOL=CPHA=0
    gf_dev->spi->max_speed_hz = max_speed_hz;
    gf_dev->spi->bits_per_word = 8;
    spi_setup(gf_dev->spi);
}

#ifdef SPI_ASYNC
static void gf_spi_complete(void *arg)
{
    complete(arg);
}
#endif

/**********************************************************
 *Message format:
 *	write cmd   |  ADDR_H |ADDR_L  |  data stream  |
 *    1B         |   1B    |  1B    |  length       |
 *
 * read buffer length should be 1 + 1 + 1 + data_length
 ***********************************************************/
int gf_spi_write_bytes(struct gf_dev *gf_dev,
	u16 addr, u32 data_len, u8 *tx_buf)
{
#ifdef SPI_ASYNC
    DECLARE_COMPLETION_ONSTACK(read_done);
#endif

    struct spi_message msg;
    struct spi_transfer *xfer = NULL;
    int ret = 0;

    xfer = kzalloc(sizeof(*xfer), GFP_KERNEL);
    if (xfer == NULL) {
		GF_LOG_ERROR("failed to kzalloc for command\n");
		return -ENOMEM;
    }

    /* send gf command to device */
    spi_message_init(&msg);

    tx_buf[0] = GF_W;
    tx_buf[1] = (u8)((addr >> 8) & 0xFF);
    tx_buf[2] = (u8)(addr & 0xFF);

    xfer[0].tx_buf = tx_buf;
    xfer[0].len = data_len + 3;
    xfer[0].delay_usecs = 5;

    spi_message_add_tail(xfer, &msg);

#ifdef SPI_ASYNC
    msg.complete = gf_spi_complete;
    msg.context = &read_done;

    spin_lock_irq(&gf_dev->spi_lock);
    ret = spi_async(gf_dev->spi, &msg);
    spin_unlock_irq(&gf_dev->spi_lock);

    if (ret == 0)
		wait_for_completion(&read_done);
	else
		GF_LOG_ERROR("failed to spi write = %d\n", ret);
#else
    ret = spi_sync(gf_dev->spi, &msg);
#endif

	if (xfer) {
    	kfree(xfer);
		xfer = NULL;
	}
    return ret;
}

/*************************************************************
 *First message:
 *	write cmd   |  ADDR_H |ADDR_L  |
 *    1B         |   1B    |  1B    |
 *Second message:
 *	read cmd   |  data stream  |
 *    1B        |   length    |
 *
 * read buffer length should be 1 + 1 + 1 + 1 + data_length
 **************************************************************/
int gf_spi_read_bytes(struct gf_dev *gf_dev, u16 addr, u32 data_len,
	u8 *rx_buf)
{
#ifdef SPI_ASYNC
    DECLARE_COMPLETION_ONSTACK(write_done);
#endif

    struct spi_message msg;
    struct spi_transfer *xfer = NULL;
    int ret = 0;

    xfer = kzalloc(sizeof(*xfer) << 1, GFP_KERNEL);/* two messages */
    if (xfer == NULL) {
		GF_LOG_ERROR("failed to kzalloc for command\n");
		return -ENOMEM;
    }

    /* send gf command to device */
    spi_message_init(&msg);

    rx_buf[0] = GF_W;
    rx_buf[1] = (u8)((addr >> 8) & 0xFF);
    rx_buf[2] = (u8)(addr & 0xFF);

    xfer[0].tx_buf = rx_buf;
    xfer[0].len = 3;
    xfer[0].delay_usecs = 5;

    spi_message_add_tail(&xfer[0], &msg);

    /*if wanted to read data from gf. Should write Read command to device before read
    	   any data from device.
     */
    ret = spi_sync(gf_dev->spi, &msg);
	if (ret) {
		GF_LOG_ERROR("failed to spi_sync = %d\n", ret);
		goto err_gf_w;
	}

    spi_message_init(&msg);
    rx_buf[4] = GF_R;
    xfer[1].tx_buf = &rx_buf[4];
    xfer[1].rx_buf = &rx_buf[4];
    xfer[1].len = data_len + 1;
    xfer[1].delay_usecs = 5;

    spi_message_add_tail(&xfer[1], &msg);

#ifdef SPI_ASYNC
    msg.complete = gf_spi_complete;
    msg.context = &write_done;

    spin_lock_irq(&gf_dev->spi_lock);
    ret = spi_async(gf_dev->spi, &msg);
    spin_unlock_irq(&gf_dev->spi_lock);

    if (ret == 0)
		wait_for_completion(&write_done);
	else
    	GF_LOG_ERROR("ret = %d\n", ret);
#else
    ret = spi_sync(gf_dev->spi, &msg);
#endif

err_gf_w:
	if (xfer) {
	    kfree(xfer);
		xfer = NULL;
	}
    return ret;
}

static int gf_spi_write_byte(struct gf_dev *gf_dev, u16 addr, u8 value)
{
    int status = 0;

	GF_LOG_INFO("addr = 0x%x, value = 0x%x\n", addr, value);

    mutex_lock(&gf_dev->buf_lock);

    gf_dev->buffer[GF_WDATA_OFFSET] = value;
    status = gf_spi_write_bytes(gf_dev, addr, 1, gf_dev->buffer);

    mutex_unlock(&gf_dev->buf_lock);
    return status;
}

static int gf_spi_read_byte(struct gf_dev *gf_dev, u16 addr, u8 *value)
{
    int status = 0;
    mutex_lock(&gf_dev->buf_lock);

    status = gf_spi_read_bytes(gf_dev, addr, 1, gf_dev->buffer);
    *value = gf_dev->buffer[GF_RDATA_OFFSET];

    GF_LOG_DEBUG("value = 0x%x, buffer[3] = 0x%x\n", *value, gf_dev->buffer[3]);
    mutex_unlock(&gf_dev->buf_lock);

    return status;
}

int gf_parse_dts(struct gf_dev *gf_dev)
{
	u32 temp_val = 0;
 	struct device_node *np = gf_dev->spi->dev.of_node;

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

	gf_dev->tz_enable = of_property_read_bool(np, "gf,tz-enable");
	GF_LOG_INFO("tz enable = %s\n", gf_dev->tz_enable ? "true" : "false");

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

static ssize_t gf_version_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	int ret = 0, i = 0;
	unsigned char version[GF_VERSION_SIZE] = { 0 };

	if (g_gf_dev->tz_enable == false) {
		memset(g_gf_dev->buffer, 0, GF_VERSION_SIZE + GF_RDATA_OFFSET);
		ret = gf_spi_read_bytes(g_gf_dev, GF_VERSION, GF_VERSION_SIZE,
			g_gf_dev->buffer);
		memcpy(version, g_gf_dev->buffer + GF_RDATA_OFFSET, GF_VERSION_SIZE);

		for (i = 0; i < GF_VERSION_SIZE; i++)
		    GF_LOG_INFO("version[%d] = 0x%x\n", i, version[i]);

		return snprintf(buf, PAGE_SIZE, "%c%c%c%c%c%c%c%x.%x.%x\n", version[0],
			version[1], version[2], version[3], version[4], version[5], version[6],
			version[7], version[8], version[9]);
	} else {
		return snprintf(buf, PAGE_SIZE, "It is not permitted\n");
	}
}

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

static ssize_t gf_mode_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	u8 mode = 0;

	if (g_gf_dev->tz_enable == false) {
		gf_spi_read_byte(g_gf_dev, GF_MODE_STATUS, &mode);
	    GF_LOG_INFO("mode = %d\n", mode);
		return snprintf(buf, PAGE_SIZE, "mode = %d\n", mode);
	} else {
		return snprintf(buf, PAGE_SIZE, "It is not permitted\n");
	}
}

static ssize_t gf_mode_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t size)
{
	int val = 0;

	sscanf(buf, "%d", &val);

	if ((val != 0) && (val != 1) && (val != 2) && (val != 3) && (val != 0x56)) {
		GF_LOG_ERROR("invalid value\n");
		goto exit;
	}

	if (g_gf_dev->tz_enable == false) {
		gf_spi_write_byte(g_gf_dev, GF_MODE_STATUS, val);
		GF_LOG_INFO("success to set mode as = %d\n", val);
	}

exit:
	return size;
}

static struct kobj_attribute gf_version_attribute =
	__ATTR(version, 0444, gf_version_show, NULL);
static struct kobj_attribute gf_wake_up_attribute =
	__ATTR(wake_up, 0664, gf_wake_up_show, gf_wake_up_store);
static struct kobj_attribute gf_mode_attribute =
	__ATTR(mode, 0664, gf_mode_show, gf_mode_store);

static struct attribute *attrs[] = {
	&gf_version_attribute.attr,
	&gf_wake_up_attribute.attr,
	&gf_mode_attribute.attr,
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

#if FW_UPDATE
static bool hw_config(struct gf_dev *gf_dev)
{
    mutex_lock(&gf_dev->buf_lock);
    memcpy(gf_dev->buffer + GF_WDATA_OFFSET, config_list[0].config[0].buffer,
		GF_CFG_LEN);
    gf_spi_write_bytes(gf_dev, GF_CFG_ADDR, GF_CFG_LEN, gf_dev->buffer);
    mutex_unlock(&gf_dev->buf_lock);

    return true;
}

static int isUpdate(struct gf_dev *gf_dev)
{
    unsigned char version[16] = { 0 };
    unsigned int ver_fw = 0;
    unsigned int ver_file = 0;
    unsigned char* fw = GF_FW;
    unsigned char fw_running = 0;
    int OFFSET = 7;

    msleep(300);
    gf_spi_read_byte(gf_dev, 0x41e4, &fw_running);
    GF_LOG_INFO("%s: 0x41e4 = 0x%x\n", __func__, fw_running);

    if (fw_running == 0xbe) {
		/* firmware running */
		ver_file = (int)(fw[12] & 0xF0) << 12;
		ver_file |= (int)(fw[12] & 0x0F)<< 8;
		ver_file |= fw[13];	//get the fw version in the i file

		/* in case we want to upgrade to a special firmware. Such as debug firmware */
		if (ver_file != 0x5a5a) {
		    mutex_lock(&gf_dev->buf_lock);
		    gf_spi_read_bytes(gf_dev, 0x8000, 16, gf_dev->buffer);
		    memcpy(version, gf_dev->buffer + GF_RDATA_OFFSET, 16);
		    mutex_unlock(&gf_dev->buf_lock);

		    if (memcmp(version, GF_PID, GF_PID_LEN)) {
				GF_LOG_INFO("version: 0x%x-0x%x-0x%x-0x%x-0x%x-0x%x\n",
					version[0], version[1],	version[2], version[3], version[4],
					version[5]);
				return 1;
		    }

		    if((version[OFFSET] > 9) || ((version[OFFSET + 1]) > 9)) {
				GF_LOG_INFO("version: 8-0x%x; 9-0x%x\n", version[OFFSET],
					version[OFFSET + 1]);
				return 1;
		    }

		    //get the current fw version
		    ver_fw  = (unsigned int)version[OFFSET] << 16;
		    ver_fw |= (unsigned int)version[OFFSET + 1] << 8;
		    ver_fw |= (unsigned int)version[OFFSET + 2];
		    GF_LOG_INFO("ver_fw: 0x%06x; ver_file:0x%06x\n", ver_fw, ver_file);

		    if (ver_fw == ver_file) {
				/*if the running firmware is or ahead of the file's firmware. No need to do upgrade */
				return 0;
		    }
		}
		GF_LOG_INFO("Current Ver: 0x%x, Upgrade to Ver: 0x%x\n", ver_fw, ver_file);
    }else {
		/* no firmware */
		GF_LOG_INFO("No running firmware. Value = 0x%x\n", fw_running);
    }
    return 1;
}

static u8 is_9p_ready_ok(struct gf_dev *gf_dev)
{
    u8 tmpBuf[16] = { 0 };
    u8 *ptr = NULL;
    u16 time_out = 0;
    gf_spi_read_bytes(gf_dev, 0x4220, 4, tmpBuf);

    ptr = &tmpBuf[GF_RDATA_OFFSET];

    while ((ptr[0] != 0x02) || (ptr[1] != 0x08) || (ptr[2] != 0x90) || \
		(ptr[3] != 0x00)) {
		time_out++;
		if (time_out > 200) {
		    return 0;
		}

		gf_spi_read_bytes(gf_dev, 0x4220, 4, tmpBuf);
		ptr = &tmpBuf[GF_RDATA_OFFSET];
    }
    GF_LOG_INFO("timeout = %d\n", time_out);
    return 1;
}

static int gf_fw_update_init(struct gf_dev *gf_dev)
{
    u8 retry_cnt = 5;
    u8 value = 0;

    while (retry_cnt--) {
		//reset and delay 5ms
		gf_reset(gf_dev);
		gf_spi_setup(gf_dev, 960000);

		if (!is_9p_ready_ok(gf_dev)) {
		    GF_LOG_ERROR("failed to check 9p ver\n");
		    retry_cnt = 0xFF;
		    break;
		}

		mdelay(10);
		gf_spi_write_byte(gf_dev, 0x5081, 0x00);

		gf_spi_write_byte(gf_dev, 0x4180, 0x0C);
		gf_spi_read_byte(gf_dev, 0x4180, &value);
		if (value == 0x0C) {
		    GF_LOG_INFO("hold SS51 and DSP successfully\n");
		    break;
		}
    }

    if(retry_cnt == 0xFF) {
		GF_LOG_INFO("faile to hold SS51 and DSP\n");
		return 0;
    } else {
		GF_LOG_INFO("Hold retry_cnt = %d\n", retry_cnt);
		gf_spi_write_byte(gf_dev, 0x4010, 0);
		return 1;
    }
}

static void gf_timer_work(struct work_struct *work)
{
    unsigned char value[4] = { 0 };
    unsigned char* p_fw = GF_FW;
    struct gf_dev *gf_dev = NULL;
    int ret = 0;
    u8 mode = 0xFF;

    GF_LOG_INFO("start\n");
    if (work == NULL) {
		GF_LOG_INFO("wrong work\n");
		return;
    }
    gf_dev = container_of(work, struct gf_dev, spi_work);

    if(gf_dev->mode == GF_FF_MODE)
		goto exit;

    mutex_lock(&gf_dev->buf_lock);
    gf_dev->spi->max_speed_hz = 960000;//SPI_SPEED_MIN;
    spi_setup(gf_dev->spi);
    mutex_unlock(&gf_dev->buf_lock);

    gf_spi_read_byte(gf_dev, 0x8040, &value[0]);
    gf_spi_read_byte(gf_dev, 0x8000, &value[1]);
    gf_spi_read_byte(gf_dev, 0x8043, &value[2]);

    if (value[0] == 0xC6 && value[1] == 0x47) {
		gf_spi_write_byte(gf_dev, 0x8040, 0xAA);
		mdelay(1);
    } else {
		gf_spi_read_byte(gf_dev, 0x8040, &value[0]);
		gf_spi_read_byte(gf_dev, 0x8000, &value[1]);
		gf_spi_read_byte(gf_dev, 0x8043, &value[2]);
		if ((value[0] == 0xC6) && (value[1] == 0x47)) {
		    gf_spi_write_byte(gf_dev, 0x8040, 0xAA);
		    mdelay(1);
		} else {
		    GF_LOG_INFO("hardware works abnormal,do reset! 0x8040 = 0x%x \
				0x8000 = 0x%x 0x8046 = 0x%x\n", value[0], value[1], value[2]);
			gf_irq_disable(gf_dev);
		    gf_reset(gf_dev);

		    gf_spi_read_byte(gf_dev, 0x41e4, &value[0]);
		    gf_spi_read_byte(gf_dev, 0x8000, &value[1]);
		    GF_LOG_INFO("read 0x41e4 finish value = 0x%x, 0x8000=0x%x\n",
				value[0], value[1]);

		    if (value[0] != 0xbe) {
				gf_spi_read_byte(gf_dev, 0x41e4, &value[0]);
				if (value[0] != 0xbe) {
				    /*******************firmware update*********************/
				    GF_LOG_INFO("firmware update start\n");
				    del_timer_sync(&gf_dev->gf_timer);
				    if (gf_fw_update_init(gf_dev)) {
						gf_fw_update(gf_dev, p_fw, FW_LENGTH);
						gf_reset();
				    }
				    gf_dev->gf_timer.expires = jiffies + 2 * HZ;
				    add_timer(&gf_dev->gf_timer);
				}
		    }
		    /**********************update config*************************/
		    ret = gf_spi_write_byte(gf_dev, 0x8040, 0xAA);
		    if (!ret)
				GF_LOG_INFO("write 0x8040 fail\n");

		    if (!hw_config(gf_dev))
				GF_LOG_INFO("write config fail\n");
		    gf_irq_enable(gf_dev);
		}
    }
    /* if mode was changed by reset, we should set the mode  back to the primary mode */
    gf_spi_read_byte(gf_dev, GF_MODE_STATUS, &mode);
    if (mode != gf_dev->mode) {
		GF_LOG_INFO("set mode back\n");
		gf_spi_write_byte(gf_dev, GF_MODE_STATUS, gf_dev->mode);
		gf_spi_read_byte(gf_dev, GF_MODE_STATUS, &mode);
		GF_LOG_INFO("mode444 = %d\n", mode);
    }

exit:
    mod_timer(&gf_dev->gf_timer, jiffies + 2*HZ);
    GF_LOG_INFO("end\n");
}

static void gf_timer_func(unsigned long arg)
{
    struct gf_dev* gf_dev = (struct gf_dev*)arg;
    schedule_work(&gf_dev->spi_work);
}
#endif

static ssize_t gf_write(struct file *filp, const char __user *buf,
	size_t count, loff_t *f_pos)
{
    struct gf_dev *gf_dev = filp->private_data;
    ssize_t	status = 0;

    if(count > bufsize) {
		GF_LOG_ERROR("count(%ld) > max bufsize(%u)\n", count, bufsize);
		return -EMSGSIZE;
    }

    mutex_lock(&gf_dev->buf_lock);

    status = copy_from_user(gf_dev->buffer + GF_WDATA_OFFSET, buf, count);
    if (status == 0) {
		gf_dev->spi->max_speed_hz = 960000;
		spi_setup(gf_dev->spi);

		status = gf_spi_write_bytes(gf_dev, GF_BUFFER_DATA, count, gf_dev->buffer);
    } else {
		GF_LOG_ERROR("failed to copy data from user\n");
		status = -EFAULT;
    }
    mutex_unlock(&gf_dev->buf_lock);

    return status;
}

static ssize_t gf_read(struct file *filp, char __user *buf, size_t count,
	loff_t *f_pos)
{
    struct gf_dev *gf_dev = filp->private_data;
    ssize_t	status = 0;

    if ((count > bufsize) || (count == 0)) {
		GF_LOG_ERROR("Max size = %d, count = %ld\n", bufsize, count);
		return -EMSGSIZE;
    }

    mutex_lock(&gf_dev->buf_lock);
    gf_dev->spi->max_speed_hz = 4800000;
    spi_setup(gf_dev->spi);

	GF_LOG_ERROR("read count = %ld\n", count);

    status = gf_spi_read_bytes(gf_dev, GF_BUFFER_DATA, count, gf_dev->buffer);
    if (status == 0) {
		unsigned long missing = 0;
		missing = copy_to_user((void __user*)((unsigned long)buf),
			gf_dev->buffer + GF_RDATA_OFFSET, count);
		status = count;
		GF_LOG_ERROR("missing = %lu, status = %ld\n", missing, status);
    } else {
		GF_LOG_ERROR("failed to read data from spi device\n");
		status = -EFAULT;
    }
    mutex_unlock(&gf_dev->buf_lock);

    return status;
}

static long gf_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct gf_dev *gf_dev = (struct gf_dev *)filp->private_data;
	struct gf_ioc_transfer *ioc = NULL;
	struct gf_key gf_key = { 0 };
	int retval = 0;
	u32 tmp = 0;
	u8 *temp_buf = NULL;

	GF_LOG_DEBUG("start\n");

	if (gf_dev->tz_enable == false) {
		if (_IOC_TYPE(cmd) != GF_IOC_REE_MAGIC)
		return -ENOTTY;

	    if (_IOC_DIR(cmd) & _IOC_READ)
			retval = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	    if (retval == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
			retval = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	    if (retval)
			return -EFAULT;

		GF_LOG_DEBUG("cmd = 0x%x, GF_IOC_CMD = 0x%lx\n", cmd, GF_IOC_REE_CMD);

	    switch(cmd) {
		case GF_IOC_REE_CMD:
		    ioc = kzalloc(sizeof(*ioc), GFP_KERNEL);
			if (ioc == NULL) {
				GF_LOG_ERROR("failed to kzalloc for ioc\n");
				retval = -EFAULT;
				break;
			}

		    /* copy command data from user to kernel */
		    if (copy_from_user(ioc, (struct gf_ioc_transfer*)arg, sizeof(*ioc))) {
				GF_LOG_ERROR("failed to copy command from user to kernel\n");
				retval = -EFAULT;
				break;
		    }

			if ((ioc->len > bufsize) || (ioc->len == 0)) {
				GF_LOG_ERROR("request length[%d] > [%d]\n", ioc->len, bufsize);
				retval = -EMSGSIZE;
				break;
		    }

		    mutex_lock(&gf_dev->buf_lock);

		    gf_dev->spi->max_speed_hz = 960000;
		    spi_setup(gf_dev->spi);

		    if (ioc->cmd == GF_R) {
				temp_buf = (void __user *)(unsigned long)ioc->buf;

				/* if want to read data from hardware */
				GF_LOG_INFO("read data from 0x%x, len = 0x%x\n", (int)ioc->addr,
					(int)ioc->len);
				gf_spi_read_bytes(gf_dev, ioc->addr, ioc->len, gf_dev->buffer);

				if (copy_to_user((void __user*)((unsigned long)ioc->buf),
					gf_dev->buffer + GF_RDATA_OFFSET, ioc->len)) {
				    GF_LOG_ERROR("failed to copy data from kernel to user\n");
				    retval = -EFAULT;
				    mutex_unlock(&gf_dev->buf_lock);
				    break;
				}
		    } else if (ioc->cmd == GF_W) {
				GF_LOG_INFO("write data from 0x%x, len = 0x%x\n", ioc->addr,
					ioc->len);
				if (ioc->addr == GF_MODE_STATUS) {
				    temp_buf = (void __user *)(unsigned long)ioc->buf;
				    GF_LOG_DEBUG("temp_buf = %x,%x,%x,%x\n", temp_buf[0],
						temp_buf[1], temp_buf[2], temp_buf[3]);
					gf_dev->mode = temp_buf[0];
					GF_LOG_INFO("set mode as 0x%x\n", gf_dev->mode);
				}
				if (copy_from_user(gf_dev->buffer + GF_WDATA_OFFSET,
					(void __user*)((unsigned long)ioc->buf), ioc->len)) {
				    GF_LOG_ERROR("failed to copy data from user to kernel\n");
				    retval = -EFAULT;
				    mutex_unlock(&gf_dev->buf_lock);
				    break;
				}
				gf_spi_write_bytes(gf_dev, ioc->addr, ioc->len, gf_dev->buffer);
		    } else {
			    retval = -EFAULT;
				GF_LOG_ERROR("unknown command\n");
		    }
		    mutex_unlock(&gf_dev->buf_lock);
		    break;
		case GF_IOC_REE_REINIT:
			GF_LOG_INFO("reinit\n");
			gf_irq_disable(gf_dev);
		    gf_hw_reset(gf_dev, 50);
			gf_irq_enable(gf_dev);
		    break;
		case GF_IOC_REE_SETSPEED:
		    retval = __get_user(tmp, (u32 __user*)arg);
		    if (tmp > 8000000) {
				GF_LOG_ERROR("maximum SPI speed is 8MHz\n");
				retval = -EMSGSIZE;
				break;
		    }
		    if (retval == 0) {
				gf_dev->spi->max_speed_hz = tmp;
				spi_setup(gf_dev->spi);
				GF_LOG_INFO("spi speed changed to %d\n", tmp);
		    }
		    break;
		default:
			retval = -EFAULT;
		    GF_LOG_ERROR("unknown command(0x%x)\n", cmd);
		    break;
	    }

		if (ioc != NULL) {
			kfree(ioc);
			ioc = NULL;
	    }
		GF_LOG_INFO("end\n");
	} else {
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
				wake_lock_timeout(&g_gf_dev->wake_lock, 3 * 1000);
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
	}
	return retval;
}

#ifdef CONFIG_COMPAT
static long gf_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
     return gf_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#endif

static unsigned int gf_poll(struct file *filp, struct poll_table_struct *wait)
{
    struct gf_dev *gf_dev = filp->private_data;

    gf_spi_read_byte(gf_dev, GF_BUFFER_STATUS, &gf_dev->buf_status);

    if ((gf_dev->buf_status & GF_BUF_STA_MASK) == GF_BUF_STA_READY) {
		return (POLLIN | POLLRDNORM);
    } else {
		GF_LOG_DEBUG("Poll no data\n");
    }
    return 0;
}

static irqreturn_t gf_irq(int irq, void* handle)
{
    struct gf_dev *gf_dev = (struct gf_dev *)handle;
    u8 mode = 0x80;
    u8 status = 0;

	if (gf_dev->tz_enable == false) {
	    gf_spi_read_byte(gf_dev, GF_BUFFER_STATUS, &status);
	    GF_LOG_DEBUG("IRQ status = 0x%x\n", status);

	    if (!(status & GF_BUF_STA_MASK)) {
			GF_LOG_ERROR("Invalid IRQ = 0x%x\n", status);
			return IRQ_HANDLED;
	    }

	    gf_spi_read_byte(gf_dev, GF_MODE_STATUS, &mode);
	    GF_LOG_INFO("status = 0x%x, mode = %d\n", status, mode);

	    switch(mode) {
		case GF_FF_MODE:
		    if ((status & GF_HOME_KEY_MASK) && (status & GF_HOME_KEY_STA)) {
				GF_LOG_INFO("wake device\n");
				gf_spi_write_byte(gf_dev, GF_MODE_STATUS, 0x00);
				input_report_key(gf_dev->input, GF_FF_KEY, 1);
				input_sync(gf_dev->input);
				input_report_key(gf_dev->input, GF_FF_KEY, 0);
				input_sync(gf_dev->input);
		    } else {
				break;
		    }

		case GF_IMAGE_MODE:
#ifdef GF_FASYNC
		    if (gf_dev->async) {
				GF_LOG_INFO("async\n");
				kill_fasync(&gf_dev->async, SIGIO, POLL_IN);
		    }
#endif
		    break;

		case GF_KEY_MODE:
		    GF_LOG_INFO("Key mode: status = 0x%x\n", status);
		    if ((status & GF_KEY_MASK) && (status & GF_BUF_STA_MASK)) {
				if (status & GF_HOME_KEY_MASK) {
				    input_report_key(gf_dev->input, GF_INPUT_HOME_KEY, (status & GF_HOME_KEY_STA) >> 4);
				    input_sync(gf_dev->input);
				} else if (status & GF_MENU_KEY_MASK) {
				    input_report_key(gf_dev->input, GF_INPUT_MENU_KEY, (status & GF_MENU_KEY_STA) >> 2);
				    input_sync(gf_dev->input);
				} else if (status & GF_BACK_KEY_MASK) {
				    input_report_key(gf_dev->input, GF_INPUT_BACK_KEY, (status & GF_BACK_KEY_STA));
				    input_sync(gf_dev->input);
				}
		    }
		    gf_spi_write_byte(gf_dev, GF_BUFFER_STATUS, (status & 0x7F));
		    break;

		case GF_SLEEP_MODE:
		    GF_LOG_ERROR("Should not happen in sleep mode.\n");
		    break;

		case GF_DEBUG_MODE:
#ifdef GF_FASYNC
		    if (gf_dev->async) {
				kill_fasync(&gf_dev->async, SIGIO, POLL_IN);
		    }
#endif
		    break;

		default:
		    GF_LOG_ERROR("Unknown mode = 0x%x\n", mode);
		    break;
	    }
	} else {
#ifdef GF_FASYNC
		if (gf_dev->async)
			kill_fasync(&gf_dev->async, SIGIO, POLL_IN);
#endif
	}
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
		mutex_lock(&gf_dev->buf_lock);
		if (gf_dev->buffer == NULL) {
		    gf_dev->buffer = kzalloc(bufsize + GF_RDATA_OFFSET, GFP_KERNEL);
		    if (gf_dev->buffer == NULL) {
				GF_LOG_DEBUG("open/ENOMEM\n");
				status = -ENOMEM;
		    }
		}
		mutex_unlock(&gf_dev->buf_lock);

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
		}
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
    .write = gf_write,
    .read  = gf_read,
    .unlocked_ioctl = gf_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = gf_compat_ioctl,
#endif
    .open    = gf_open,
    .release = gf_release,
    .poll    = gf_poll,
#ifdef GF_FASYNC
    .fasync  = gf_fasync,
#endif
};

static int gf_probe(struct spi_device *spi)
{
    struct gf_dev *gf_dev = NULL;
    int ret = 0;
	unsigned long minor = 0;
    unsigned char version[GF_VERSION_SIZE] = { 0 };

    GF_LOG_INFO("Spi Device start\n");

    /* Allocate driver data */
    gf_dev = kzalloc(sizeof(*gf_dev), GFP_KERNEL);
    if (!gf_dev) {
		GF_LOG_ERROR("failed to alloc memory for gf device\n");
		return -ENOMEM;
    }

	gf_dev->device_available = 0;
	gf_dev->fb_black = 0;

    /* Initialize the driver data */
    gf_dev->spi = spi;
	spi_set_drvdata(spi, gf_dev);
	g_gf_dev = gf_dev;

    wake_lock_init(&gf_dev->wake_lock, WAKE_LOCK_SUSPEND, "goodixfp_wakelock");
    spin_lock_init(&gf_dev->spi_lock);
    mutex_init(&gf_dev->buf_lock);
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

    ret = gf_init_pinctrl(gf_dev, &spi->dev);
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
    minor = find_first_zero_bit(minors, N_SPI_MINORS);
    if (minor < N_SPI_MINORS) {
    	struct device *dev;

    	gf_dev->devt = MKDEV(SPIDEV_MAJOR, minor);
    	dev = device_create(g_gf_spi_class, &spi->dev, gf_dev->devt,
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

	gf_dev->buffer = kzalloc(bufsize + GF_RDATA_OFFSET, GFP_KERNEL);
	if(gf_dev->buffer == NULL) {
	    ret = -ENOMEM;
		GF_LOG_ERROR("failed to alloc memory for buffer.\n");
	    goto err_buffer_kzalloc;
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

	/* SPI parameters */
	if (gf_dev->tz_enable == false) {
		gf_spi_setup(gf_dev, 960000);

#if FW_UPDATE
		if (isUpdate(gf_dev)) {
		    unsigned char* fw = GF_FW;
		    /* do upgrade action */
		    if (gf_fw_update_init(gf_dev)) {
				gf_fw_update(gf_dev, fw, FW_LENGTH);
				gf_hw_reset(gf_dev, 50);
		    }
		}

		/* write config */
		if (!hw_config(gf_dev))
		    GF_LOG_ERROR("failed to write config\n");
#endif

		msleep(50);//need to discuss??
		ret = gf_spi_read_bytes(gf_dev, GF_VERSION, GF_VERSION_SIZE, gf_dev->buffer);
		memcpy(version, gf_dev->buffer + GF_RDATA_OFFSET, GF_VERSION_SIZE);

		GF_LOG_INFO("version = %c%c%c%c%c%c%c%x.%x.%x\n", version[0],
			version[1], version[2], version[3], version[4], version[5], version[6],
			version[7], version[8], version[9]);
	}

	ret = request_threaded_irq(gf_dev->irq, NULL, gf_irq,
		IRQF_TRIGGER_RISING | IRQF_ONESHOT,	dev_name(&spi->dev), gf_dev);
	if (ret) {
		GF_LOG_ERROR("failed to request threaded irq\n");
		goto err_request_irq;
	} else {
		gf_irq_disable(gf_dev);
	}

#if ESD_PROTECT
	INIT_WORK(&gf_dev->spi_work, gf_timer_work);
	init_timer(&gf_dev->gf_timer);
	gf_dev->gf_timer.function = gf_timer_func;
	gf_dev->gf_timer.expires = jiffies + 3 * HZ;
	gf_dev->gf_timer.data = gf_dev;
	add_timer(&gf_dev->gf_timer);
#endif

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
    kfree(gf_dev->buffer);
err_buffer_kzalloc:
	if (gf_dev->devt != 0) {
		mutex_lock(&device_list_lock);
		list_del(&gf_dev->device_entry);
		device_destroy(g_gf_spi_class, gf_dev->devt);
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

static int gf_remove(struct spi_device *spi)
{
    struct gf_dev *gf_dev = spi_get_drvdata(spi);

	GF_LOG_INFO("start\n");

    /* make sure ops on existing fds can abort cleanly */
    if (gf_dev->irq) {
		free_irq(gf_dev->irq, gf_dev);
    }

#if ESD_PROTECT
    del_timer_sync(&gf_dev->gf_timer);
    cancel_work_sync(&gf_dev->spi_work);
#endif

    spin_lock_irq(&gf_dev->spi_lock);
    gf_dev->spi = NULL;
    spi_set_drvdata(spi, NULL);
    spin_unlock_irq(&gf_dev->spi_lock);

    /* prevent new opens */
    mutex_lock(&device_list_lock);
    sysfs_remove_group(g_gf_kobj, &attr_group);
	kobject_put(g_gf_kobj);
    list_del(&gf_dev->device_entry);
    device_destroy(g_gf_spi_class, gf_dev->devt);
	clear_bit(MINOR(gf_dev->devt), minors);

    if (gf_dev->users == 0) {
		if (gf_dev->input != NULL) {
		    input_unregister_device(gf_dev->input);
			input_free_device(gf_dev->input);
		}

		if (gf_dev->buffer != NULL) {
		    kfree(gf_dev->buffer);
			gf_dev->buffer = NULL;
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

static int gf_suspend(struct spi_device *dev, pm_message_t mesg)
{
	struct gf_dev *gf_dev = spi_get_drvdata(dev);

    GF_LOG_INFO("start\n");

	if (gf_dev->wake_up == true) {
		gf_irq_wake_op(gf_dev->irq, GF_INT_ENABLE);
	} else {
		gf_irq_disable(gf_dev);
	}

    GF_LOG_INFO("end\n");

    return 0 ;
}

static int gf_resume(struct spi_device *dev)
{
    struct gf_dev *gf_dev= spi_get_drvdata(dev);

	GF_LOG_INFO("start\n");

	if (gf_dev->wake_up == true) {
		gf_irq_wake_op(gf_dev->irq, GF_INT_DISABLE);
	} else {
		gf_irq_enable(gf_dev);
	}

	GF_LOG_INFO("end\n");

     return 0;
}

static struct spi_driver gf_spi_driver = {
    .driver = {
		.name =	SPI_DEV_NAME,
		.owner = THIS_MODULE,
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
    BUILD_BUG_ON(N_SPI_MINORS > 256);

    status = register_chrdev(SPIDEV_MAJOR, CHRD_DRIVER_NAME, &gf_fops);
    if (status < 0) {
		GF_LOG_ERROR("failed to register char device\n");
		return status;
    }

    g_gf_spi_class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(g_gf_spi_class)) {
		unregister_chrdev(SPIDEV_MAJOR, gf_spi_driver.driver.name);
		GF_LOG_ERROR("failed to create class\n");
		return PTR_ERR(g_gf_spi_class);
    }

    status = spi_register_driver(&gf_spi_driver);
    if (status < 0) {
		class_destroy(g_gf_spi_class);
		unregister_chrdev(SPIDEV_MAJOR, gf_spi_driver.driver.name);
		GF_LOG_ERROR("failed to register SPI driver\n");
    }
    return status;
}
module_init(gf_init);

static void __exit gf_exit(void)
{
    spi_unregister_driver(&gf_spi_driver);
    class_destroy(g_gf_spi_class);
    unregister_chrdev(SPIDEV_MAJOR, gf_spi_driver.driver.name);
}
module_exit(gf_exit);

MODULE_AUTHOR("Jiangtao Yi, <yijiangtao@goodix.com>");
MODULE_DESCRIPTION("User mode SPI device interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:gf-spi");
