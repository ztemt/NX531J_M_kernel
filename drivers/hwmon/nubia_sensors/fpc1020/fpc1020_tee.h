#ifndef _FPC1020_TEE_H_
#define _FPC1020_TEE_H_

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/irq.h>
#include <asm/irq.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/input.h>
#include <linux/wakelock.h>
#include <linux/pinctrl/consumer.h>
#include <linux/clk.h>
#include <soc/qcom/scm.h>
#include "fpc1020_log.h"

#define FPC1020_DEV_NAME    "fpc"
#define FPC1020_CLASS_NAME  "fpc"
#define CHRD_DRIVER_NAME    "fpc"
#define AUTODETECT_NAME     "fpc"

#define FPC1020_RESET_LOW_US    1000
#define FPC1020_RESET_HIGH1_US  100
#define FPC1020_RESET_HIGH2_US  1250
#define FPC_TTW_HOLD_TIME       1000
#define SPIDEV_MAJOR	        154
#define N_SPI_MINORS		    32

struct fpc1020_data {
	struct device *dev;
	int irq_gpio;
	int rst_gpio;
	int enable_gpio;
	int irq_num;
	struct mutex lock;
	bool prepared;
	bool wakeup_enabled;
	unsigned long minor;
	dev_t dev_num;

    struct device *node_dev;
	struct list_head device_entry;
    struct class *class;
	struct pinctrl *ts_pinctrl;
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_suspend;
	struct wake_lock ttw_wl;
};

static int fpc1020_request_named_gpio(struct fpc1020_data *fpc1020,	const char *label, int *gpio);
static int fpc1020_pinctrl_init(struct fpc1020_data *fpc1020);
static int fpc1020_pinctrl_select(struct fpc1020_data *fpc1020, bool on);
static ssize_t wakeup_enable_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t irq_get(struct device *device, struct device_attribute *attribute, char* buffer);
static ssize_t irq_ack(struct device *device, struct device_attribute *attribute, const char *buffer, size_t count);
static int fpc1020_probe(struct platform_device *pdev);
static int __init fpc1020_init(void);

#ifdef CONFIG_NUBIA_FP_AUTODETECT
extern int fingerprint_device_autodetect(char *target_fingerprint_name);
#endif

#endif
