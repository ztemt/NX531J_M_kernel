#ifndef __FPAUTODETECT_H__
#define __FPAUTODETECT_H__


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


#define FPAUTODETECT_LOG_TAG "FPAUTODETECT"
#define FPAUTODETECT_LOG_ON
//#define FPAUTODETECT_DEBUG_ON

#ifdef  FPAUTODETECT_LOG_ON
#define FPAUTODETECT_LOG_ERROR(fmt, args...) printk(KERN_ERR "[%s] [%s: %d] "  fmt, \
	FPAUTODETECT_LOG_TAG, __FUNCTION__, __LINE__, ##args)
#define FPAUTODETECT_LOG_INFO(fmt, args...) printk(KERN_INFO "[%s] [%s: %d] "  fmt, \
	FPAUTODETECT_LOG_TAG, __FUNCTION__, __LINE__, ##args)

    #ifdef  FPAUTODETECT_DEBUG_ON
#define  FPAUTODETECT_LOG_DEBUG(fmt, args...) printk(KERN_DEBUG "[%s] [%s: %d] "  fmt, \
	FPAUTODETECT_LOG_TAG, __FUNCTION__, __LINE__, ##args)
    #else
#define FPAUTODETECT_LOG_DEBUG(fmt, args...)
    #endif
#else
#define FPAUTODETECT_LOG_ERROR(fmt, args...)
#define FPAUTODETECT_LOG_INFO(fmt, args...)
#define FPAUTODETECT_LOG_DEBUG(fmt, args...)
#endif

#define CHAR_LENGTH 20
#define FINGERPRINT_DEVICE_NAME_GOODIX  "goodix"
#define FINGERPRINT_DEVICE_NAME_EGIS    "egis"
#define FINGERPRINT_DEVICE_NAME_FPC     "fpc"
#define FINGERPRINT_DEVICE_NAME_UNKNOW  "unkonw"

#define FINGERPRINT_GPIO_STATUS_GOODIX  '0'
#define FINGERPRINT_GPIO_STATUS_EGIS    '1'
#define FINGERPRINT_GPIO_STATUS_FPC     '2'
#define FINGERPRINT_GPIO_STATUS_UNKNOW  "unkonw"

extern void ztemt_get_hw_pcb_version(char* result);
extern void ztemt_get_config_standard(char* result);
static int fingerprint_gpio_status_get(char *param);
int fingerprint_device_autodetect(char *target_fingerprint_name);
#endif
