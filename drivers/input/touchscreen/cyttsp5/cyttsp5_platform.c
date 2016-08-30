/*
 * cyttsp5_platform.c
 * Cypress TrueTouch(TM) Standard Product V5 Platform Module.
 * For use with Cypress Txx5xx parts.
 * Supported parts include:
 * TMA5XX
 *
 * Copyright (C) 2013-2014 Cypress Semiconductor
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Contact Cypress Semiconductor at www.cypress.com <ttdrivers@cypress.com>
 *
 */

#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/input/mt.h>

#include "cyttsp5_regs.h"
#include "cyttsp5_platform.h"

extern int rst_gpio_number;
extern int irq_gpio_number;
#define CYTTSP5_I2C_RST_GPIO    gpio_to_irq(rst_gpio_number)
#define CYTTSP5_I2C_IRQ_GPIO    gpio_to_irq(irq_gpio_number)

#define CYTTSP5_GW_FIRMWARE_VERSION     0x0C0E


#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_FW_UPGRADE
/* FW for Panel ID = 0x00 */
#include "cyttsp5_fw_pid00.h"
static struct cyttsp5_touch_firmware cyttsp5_firmware_pid00 = {
	.img = cyttsp4_img_pid00,
	.size = ARRAY_SIZE(cyttsp4_img_pid00),
	.ver = cyttsp4_ver_pid00,
	.vsize = ARRAY_SIZE(cyttsp4_ver_pid00),
	.panel_id = 0x00,
};

/* FW for Panel ID = 0x01 */
#include "cyttsp5_fw_pid01.h"
static struct cyttsp5_touch_firmware cyttsp5_firmware_pid01 = {
	.img = cyttsp4_img_pid01,
	.size = ARRAY_SIZE(cyttsp4_img_pid01),
	.ver = cyttsp4_ver_pid01,
	.vsize = ARRAY_SIZE(cyttsp4_ver_pid01),
	.panel_id = 0x01,
};

/* FW for Panel ID not enabled (legacy) */
#include "cyttsp5_fw.h"
static struct cyttsp5_touch_firmware cyttsp5_firmware = {
	.img = cyttsp4_img,
	.size = ARRAY_SIZE(cyttsp4_img),
	.ver = cyttsp4_ver,
	.vsize = ARRAY_SIZE(cyttsp4_ver),
};
#else
/* FW for Panel ID not enabled (legacy) */
static struct cyttsp5_touch_firmware cyttsp5_firmware = {
	.img = NULL,
	.size = 0,
	.ver = NULL,
	.vsize = 0,
};
#endif

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_TTCONFIG_UPGRADE
/* TT Config for Panel ID = 0x00 */
#include "cyttsp5_params_pid00.h"
static struct touch_settings cyttsp5_sett_param_regs_pid00 = {
	.data = (uint8_t *)&cyttsp4_param_regs_pid00[0],
	.size = ARRAY_SIZE(cyttsp4_param_regs_pid00),
	.tag = 0,
};

static struct touch_settings cyttsp5_sett_param_size_pid00 = {
	.data = (uint8_t *)&cyttsp4_param_size_pid00[0],
	.size = ARRAY_SIZE(cyttsp4_param_size_pid00),
	.tag = 0,
};

static struct cyttsp5_touch_config cyttsp5_ttconfig_pid00 = {
	.param_regs = &cyttsp5_sett_param_regs_pid00,
	.param_size = &cyttsp5_sett_param_size_pid00,
	.fw_ver = ttconfig_fw_ver_pid00,
	.fw_vsize = ARRAY_SIZE(ttconfig_fw_ver_pid00),
	.panel_id = 0x00,
};

/* TT Config for Panel ID = 0x01 */
#include "cyttsp5_params_pid01.h"
static struct touch_settings cyttsp5_sett_param_regs_pid01 = {
	.data = (uint8_t *)&cyttsp4_param_regs_pid01[0],
	.size = ARRAY_SIZE(cyttsp4_param_regs_pid01),
	.tag = 0,
};

static struct touch_settings cyttsp5_sett_param_size_pid01 = {
	.data = (uint8_t *)&cyttsp4_param_size_pid01[0],
	.size = ARRAY_SIZE(cyttsp4_param_size_pid01),
	.tag = 0,
};

static struct cyttsp5_touch_config cyttsp5_ttconfig_pid01 = {
	.param_regs = &cyttsp5_sett_param_regs_pid01,
	.param_size = &cyttsp5_sett_param_size_pid01,
	.fw_ver = ttconfig_fw_ver_pid01,
	.fw_vsize = ARRAY_SIZE(ttconfig_fw_ver_pid01),
	.panel_id = 0x01,
};

/* TT Config for Panel ID not enabled (legacy)*/
#include "cyttsp5_params.h"
static struct touch_settings cyttsp5_sett_param_regs = {
	.data = (uint8_t *)&cyttsp4_param_regs[0],
	.size = ARRAY_SIZE(cyttsp4_param_regs),
	.tag = 0,
};

static struct touch_settings cyttsp5_sett_param_size = {
	.data = (uint8_t *)&cyttsp4_param_size[0],
	.size = ARRAY_SIZE(cyttsp4_param_size),
	.tag = 0,
};

static struct cyttsp5_touch_config cyttsp5_ttconfig = {
	.param_regs = &cyttsp5_sett_param_regs,
	.param_size = &cyttsp5_sett_param_size,
	.fw_ver = ttconfig_fw_ver,
	.fw_vsize = ARRAY_SIZE(ttconfig_fw_ver),
};
#else
/* TT Config for Panel ID not enabled (legacy)*/
static struct cyttsp5_touch_config cyttsp5_ttconfig = {
	.param_regs = NULL,
	.param_size = NULL,
	.fw_ver = NULL,
	.fw_vsize = 0,
};
#endif

static struct cyttsp5_touch_firmware *cyttsp5_firmwares[] = {
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_FW_UPGRADE
	&cyttsp5_firmware_pid00,
	&cyttsp5_firmware_pid01,
#endif
	NULL, /* Last item should always be NULL */
};

static struct cyttsp5_touch_config *cyttsp5_ttconfigs[] = {
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_TTCONFIG_UPGRADE
	&cyttsp5_ttconfig_pid00,
	&cyttsp5_ttconfig_pid01,
#endif
	NULL, /* Last item should always be NULL */
};

struct cyttsp5_loader_platform_data _cyttsp5_loader_platform_data = {
	.fw = &cyttsp5_firmware,
	.ttconfig = &cyttsp5_ttconfig,
	.fws = cyttsp5_firmwares,
	.ttconfigs = cyttsp5_ttconfigs,
	.flags = CY_LOADER_FLAG_CALIBRATE_AFTER_FW_UPGRADE,
};

int cyttsp5_xres(struct cyttsp5_core_platform_data *pdata,
		struct device *dev)
{
	//int rst_gpio = pdata->rst_gpio;
    int rst_gpio = rst_gpio_number;
	int rc = 0;

	gpio_direction_output(rst_gpio, 1);
	msleep(20);
	gpio_direction_output(rst_gpio, 0);
	msleep(40);
	gpio_direction_output(rst_gpio, 1);
	msleep(20);
	dev_info(dev,
		"%s: RESET CYTTSP gpio=%d r=%d\n", __func__,
		rst_gpio, rc);
	return rc;
}

int cyttsp5_init(struct cyttsp5_core_platform_data *pdata,
		int on, struct device *dev)
{
	int rc = 0;

    if (!pdata) {
		dev_err(dev,"%s NULL Pointer detected!\n",__func__);
		WARN_ON(1);
		return -EINVAL;
	}
	/* reset, irq gpio info */
    pdata->rst_gpio = rst_gpio_number;
    pdata->irq_gpio = irq_gpio_number;

    dev_dbg(dev, "--------pdata->rst_gpio------------: %d\n",(pdata->rst_gpio));
    dev_dbg(dev, "--------pdata->irq_gpio------------: %d\n",(pdata->irq_gpio));

	if (on) {
#if 1
	if(!gpio_is_valid(rst_gpio_number))
		return -ENODEV;
   	 rc = gpio_request(rst_gpio_number, "CYTTSP5_I2C_RST_GPIO");
    	if (rc < 0) 
    	{
        	printk("Failed to request GPIO:%d, ERRNO:%d", (s32)rst_gpio_number, rc);
        	rc = -ENODEV;
    	}
   	    else
   	    {
        	gpio_direction_output(rst_gpio_number, 1);
		
        }

	if(!gpio_is_valid(irq_gpio_number))
		return -ENODEV;

	rc = gpio_request(irq_gpio_number, "CYTTSP5_I2C_IRQ_GPIO");
	if (rc < 0) {
			printk("Failed request CYTTSP5_I2C_IRQ_GPIO.\n");
			return rc;
		}
	gpio_direction_input(irq_gpio_number);
#else        
		rc = gpio_request(rst_gpio, NULL);
		if (rc < 0) {
			gpio_free(rst_gpio);
			rc = gpio_request(rst_gpio, NULL);
		}
		if (rc < 0) {
			dev_err(dev,
				"%s: Fail request gpio=%d\n", __func__,
				rst_gpio);
		} else {
			rc = gpio_direction_output(rst_gpio, 1);
			if (rc < 0) {
				pr_err("%s: Fail set output gpio=%d\n",
					__func__, rst_gpio);
				gpio_free(rst_gpio);
			} else {
				rc = gpio_request(irq_gpio, NULL);
				if (rc < 0) {
					gpio_free(irq_gpio);
					rc = gpio_request(irq_gpio,
						NULL);
				}
				if (rc < 0) {
					dev_err(dev,
						"%s: Fail request gpio=%d\n",
						__func__, irq_gpio);
					gpio_free(rst_gpio);
				} else {
					gpio_direction_input(irq_gpio);
				}
			}
		}
#endif
	} else {
		gpio_free(rst_gpio_number);
		gpio_free(irq_gpio_number);
	}

	dev_dbg(dev, "%s: INIT CYTTSP RST gpio=%d and IRQ gpio=%d r=%d\n",
		__func__, (pdata->rst_gpio), (pdata->irq_gpio), rc);
	return rc;
}

static int cyttsp5_wakeup(struct cyttsp5_core_platform_data *pdata,
		struct device *dev, atomic_t *ignore_irq)
{
	return 0;
}

static int cyttsp5_sleep(struct cyttsp5_core_platform_data *pdata,
		struct device *dev, atomic_t *ignore_irq)
{
	return 0;
}

int cyttsp5_power(struct cyttsp5_core_platform_data *pdata,
		int on, struct device *dev, atomic_t *ignore_irq)
{
	if (on)
		return cyttsp5_wakeup(pdata, dev, ignore_irq);

	return cyttsp5_sleep(pdata, dev, ignore_irq);
}

int cyttsp5_irq_stat(struct cyttsp5_core_platform_data *pdata,
		struct device *dev)
{
	return gpio_get_value(irq_gpio_number);
}

/*** ZTEMT Added by luochangyang, 2013/09/12 ***/
int cyttsp_check_version(struct cyttsp5_core_platform_data *pdata, struct device *dev)
{
    u16 fw_ver_ic = 0;
    u16 fw_ver_dr = 0;

	if (!pdata) {
		dev_info(dev, "%s NULL Pointer detected!\n",__func__);
		WARN_ON(1);
		return -EINVAL;
	}
	
	memcpy(pdata->cy_fw_file_name, "cyttsp5_fw.bin", CY_FW_FILE_NAME_LEN);

 /*   if (si->fw_ver_ic == 0x0584 || si->fw_ver_ic == 0x05A2
        || (si->fw_ver_ic & 0xFFF0) == 0x05C0 || si->fw_ver_ic == 0x0000) {
        si->fw_ver_dr = CYTTSP5_TPK_FIRMEARE_VERSION;

        return 1;
    }
*/
    switch (pdata->fw_ver_ic & 0xFF00) {
        case 0x0C00:
            pdata->fw_ver_dr = CYTTSP5_GW_FIRMWARE_VERSION;
            break;
 /*		case 0x0A00:
            pdata->fw_ver_dr = CYTTSP5_TPK_FIRMEARE_VERSION;
			memcpy(pdata->cy_fw_file_name, "cyttsp5_fw_1.bin", CY_FW_FILE_NAME_LEN);
            break;
*/
        default:
            pdata->fw_ver_dr = 0x0000;
            dev_info(dev, "%s: FW didn't match, will NOT upgarde!\n", __func__);
            return -1;
    }

	/*For NX506J GW TP write FW 0x0701 */
    /*
	if (si->fw_ver_ic == 0x0701 && (si->fw_ver_dr & 0xFFF0) == 0x0900) {
        return 1;
    }
*/
	if ((pdata->fw_ver_ic & 0xFF00) == (pdata->fw_ver_dr & 0xFF00)) {
		fw_ver_ic = pdata->fw_ver_ic & 0x00FF;
		fw_ver_dr = pdata->fw_ver_dr & 0x00FF;

		if (fw_ver_ic  == fw_ver_dr) {/*equal*/
	        return 0;
		} else if (fw_ver_dr > fw_ver_ic) {
			return 1;
		} else {
			return -1;
		}
	} else {
		dev_info(dev, "%s: FW didn't match, will NOT upgarde!\n", __func__);
		return -1;
	}
}
/***ZTEMT END***/


#ifdef CYTTSP5_DETECT_HW
int cyttsp5_detect(struct cyttsp5_core_platform_data *pdata,
		struct device *dev, cyttsp5_platform_read read)
{
	int retry = 1;
	int rc;
	char buf[1];

	while (retry--) {
		/* Perform reset, wait for 100 ms and perform read */
		dev_vdbg(dev, "%s: Performing a reset\n", __func__);
		pdata->xres(pdata, dev);
		msleep(100);
		rc = read(dev, buf, 1);
		if (!rc)
			return 0;

		dev_vdbg(dev, "%s: Read unsuccessful, try=%d\n",
			__func__, 1 - retry);
	}

	return rc;
}
#endif
