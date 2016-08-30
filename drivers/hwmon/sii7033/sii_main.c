/*
*Copyright (C) 2013-2014 Silicon Image, Inc.
*
*This program is free software; you can redistribute it and/or
*modify it under the terms of the GNU General Public License as
*published by the Free Software Foundation version 2.
*This program is distributed AS-IS WITHOUT ANY WARRANTY of any
*kind, whether express or implied; INCLUDING without the implied warranty
*of MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE or NON-INFRINGEMENT.
*See the GNU General Public License for more details at
*http://www.gnu.org/licenses/gpl-2.0.html.
*/
#include "Wrap.h"
#include "si_time.h"
#include "si_usbpd_core.h"
#include "si_usbpd_main.h"
#include "si_usbpd_regs.h"

#include <linux/regulator/consumer.h>

#define DRP 0
#define DFP 1
#define UFP 2
#define INT_INDEX 0

dev_t dev_num;
struct class *usbpd_class;
static struct i2c_client *i2c_dev_client;

struct gpio sii70xx_gpio[NUM_GPIO] = {
	{GPIO_USBPD_INT, GPIOF_IN, "USBPD_intr"},
	{GPIO_VBUS_SRC, GPIOF_OUT_INIT_LOW, "SRC_En"},
	{GPIO_VBUS_SNK, GPIOF_OUT_INIT_LOW, "SNK_VBUS_En"},
	{GPIO_RESET_CTRL, GPIOF_OUT_INIT_HIGH, "RESET_CTRL_En"},
	{GPIO_LEV_SHF, GPIOF_OUT_INIT_HIGH, "LEV_SHF_En"}
};

int drp_mode = DRP;
module_param(drp_mode, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(drp_mode, \
	"An integer parameter to switch	PD mode between DRP(0) DFP(1) and UFP(2)");


void initialisethreadIds(struct sii_usbp_policy_engine *pdev)
{
	return;
}

static int si_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	int value;

	value = of_get_named_gpio_flags(np, "simg-70xx,irq-gpio", 0, NULL);
	if (value >= 0)
		sii70xx_gpio[INT_INDEX].gpio = value;
	else
		return -ENODEV;

	value = of_get_named_gpio_flags(np, "simg-70xx,vbus-src", 0, NULL);
	if (value >= 0)
		sii70xx_gpio[VBUS_SRC].gpio = value;
	else
		return -ENODEV;

	value = of_get_named_gpio_flags(np, "simg-70xx,vbus-snk", 0, NULL);
	if (value >= 0)
		sii70xx_gpio[VBUS_SNK].gpio = value;
	else
		return -ENODEV;
	value = of_get_named_gpio_flags(np, "simg-70xx,reset-enable", 0, NULL);
	if (value >= 0)
		sii70xx_gpio[RESET_CTRL].gpio = value;
	else
		return -ENODEV;
	value = of_get_named_gpio_flags(np, "simg-70xx,lev-shf", 0, NULL);
	if (value >= 0)
		sii70xx_gpio[LEV_SHF].gpio = value;
	else
		return -ENODEV;

	pr_info("Interrupt GPIO = %d\n", sii70xx_gpio[INT_INDEX].gpio);
	pr_info("VBUS SRC GPIO = %d\n", sii70xx_gpio[VBUS_SRC].gpio);
	pr_info("VBUS SNK GPIO = %d\n", sii70xx_gpio[VBUS_SNK].gpio);
	pr_info("Reset GPIO = %d\n", sii70xx_gpio[RESET_CTRL].gpio);
	pr_info("LEV_SHF GPIO = %d\n", sii70xx_gpio[LEV_SHF].gpio);
	return 0;
}


static int reg_set_optimum_mode_check(struct regulator *reg, int load_uA)
{
	return (regulator_count_voltages(reg) > 0) ?
		regulator_set_optimum_mode(reg, load_uA) : 0;
}

static int sii7033_power_on(struct device *dev)
{

	int rc;
    	static struct regulator *vcc_ana;
    	static struct regulator *vcc_i2c;
		
	pr_info("\n --- sii7033_power_on ---\n");
	vcc_ana = regulator_get(dev, "vdd_ana");
	if (IS_ERR(vcc_ana))
    {
		rc = PTR_ERR(vcc_ana);
		dev_err(dev, "Regulator get failed vcc_ana rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(vcc_ana) > 0)
    {
		rc = regulator_set_voltage(vcc_ana, 3300000, 3300000);
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

	vcc_i2c = regulator_get(dev, "vcc_i2c");


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

    msleep(100);
    
    return 0;


error_reg_en_vcc_i2c:
    reg_set_optimum_mode_check(vcc_i2c, 0);
error_set_vtg_i2c:
    regulator_put(vcc_i2c);
error_reg_opt_vcc_dig:
    regulator_disable(vcc_ana);
error_reg_en_vcc_ana:
    reg_set_optimum_mode_check(vcc_ana, 0);
error_set_vtg_vcc_ana:
	regulator_put(vcc_ana);
	return rc;

}



static int sii_i2c_remove(struct i2c_client *client)
{
	pr_info("\nGPIO free Array\n");
	gpio_free_array(sii70xx_gpio,
		ARRAY_SIZE(sii70xx_gpio));
	return 0;
}

static int sii_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret = 0;

	pr_info(" Probe called\n");
	dev_dbg(&client->dev, "Enter\n");

	pr_info("\n=======================================\n");
	pr_info("\n --- SII70XX Driver v8.3 ---\n");
	pr_info("\n=======================================\n");

	if (client->dev.of_node) {
		ret = si_parse_dt(&client->dev);
		if (ret)
			return -ENODEV;
	} else
		return -ENODEV;

	ret = gpio_request_array(sii70xx_gpio, ARRAY_SIZE(sii70xx_gpio));

	if (ret < 0) {
		dev_err(&client->dev, "gpio_request_array failed");
		return -EINVAL;
	}

	ret = sii7033_power_on(&client->dev);
	if (ret < 0) {
	dev_err(&client->dev, "sii7033_power_on failed");
	return -EINVAL;
	}

	
	i2c_dev_client = client;
	usbpd_pf_i2c_init(client->adapter);

	pr_info("\ndrp_mode 0x%x\n",
		drp_mode);
	/*pdev->pd_mode = drp_mode;*/
	sii70xx_device_init(&client->dev,
		(struct gpio *)sii70xx_gpio);

	return ret;
}

static struct i2c_device_id sii_i2c_id[] = {
	{ SII_DRIVER_NAME, 0x68},
	{}
};

static const struct of_device_id sii_match_table[] = {
	{.compatible = COMPATIBLE_NAME},
	{}
};

static struct i2c_driver sii_i2c_driver = {
	.driver = {
		.name = SII_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = sii_match_table,
	},
	.probe = sii_i2c_probe,
	.remove =  sii_i2c_remove,
	.id_table = sii_i2c_id,
};

static int __init i2c_init(void)
{
	int ret = -ENODEV;

	pr_info("\n !!! i2c init called !!!\n");
	ret = i2c_add_driver(&sii_i2c_driver);
	if (ret < 0) {
		if (ret == 0)
			i2c_del_driver(&sii_i2c_driver);
		    pr_info("failed !\n\nCHECK POWER AND CONNECTION ");
		    pr_info("TO Sii70xx Starter Kit.\n\n");
		    goto err_exit;
	}
	pr_info("\n ret  = %x\n", ret);

	goto done;

err_exit:
	pr_info("\n exit ret value = %x\n", ret);
done:
	pr_info("returning %d\n", ret);
	return ret;
}

static int __init si_drv_70xx_init(void)
{
	int ret;

	pr_info("\n !!!si_drv_70xx_init!!!\n");

	ret = i2c_init();

	return ret;
}

static void __exit si_drv_70xx_exit(void)
{
	pr_info("si_70xx_exit called\n");

	usbpd_device_exit(&i2c_dev_client->dev);
	pr_info("client removed\n");
	i2c_del_driver(&sii_i2c_driver);
	pr_info("driver unloaded.\n");
}

module_init(si_drv_70xx_init);
module_exit(si_drv_70xx_exit);

MODULE_DESCRIPTION("Silicon Image SiI70xx switch driver");
MODULE_AUTHOR("Silicon Image <http://www.siliconimage.com>");
MODULE_LICENSE("GPL");


