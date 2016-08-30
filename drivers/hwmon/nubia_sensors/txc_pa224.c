/*
 * This file is part of the PA224 sensor driver.
 * PA224 is combined proximity, and VCSEL.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 *Reversion
 *
 *
 *
 
 when         	who         		Remark : what, where, why          		version
-----------   	------------     	-----------------------------------   	------------------
2015/11/17	Simon Hsueh	& Allen Hsiao		For PA224 interrupt mode and oil alg   	v1.1.1
==========================================================================================
*/
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/string.h>
#include <linux/irq.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
#include <linux/ioctl.h>
#include <asm/atomic.h>
#include <linux/wakelock.h>


#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#ifdef SENSORS_CLASS_DEV
#include <linux/sensors.h>
#endif

#include "txc_pa224.h"



#define PA224_DRV_NAME "pa224"
#define DRIVER_VERSION "1.3.0"

#define LOG_TAG "TXC-PA224"
#define DEBUG_ON //DEBUG SWITCH

#define SENSOR_LOG_FILE__ strrchr(__FILE__, '/') ? (strrchr(__FILE__, '/') + 1) : __FILE__

#define SENSOR_LOG_ERROR(fmt, args...) printk(KERN_ERR "[%s] [%s:%d] " fmt,\
					LOG_TAG, __FUNCTION__, __LINE__, ##args)
#define SENSOR_LOG_INFO(fmt, args...)  printk(KERN_INFO "[%s] [%s:%d] "  fmt,\
					LOG_TAG, __FUNCTION__, __LINE__, ##args)
#ifdef  DEBUG_ON
#define SENSOR_LOG_DEBUG(fmt, args...) printk(KERN_DEBUG "[%s] [%s:%d] "  fmt,\
					LOG_TAG, __FUNCTION__, __LINE__, ##args)
#else
#define SENSOR_LOG_DEBUG(fmt, args...)
#endif

/* Oil */
#define TXC_ABS(x) (x) >= 0 ? (x):(x)*(-1)
#define TXC_SIZE(arr) (sizeof(arr)/sizeof(arr[0]))
#define TXC_SUM(arr, sum) \
do { \
	int i = 0; \
	int size = TXC_SIZE(arr); \
	for (i=0; i<size; i++) \
		sum += arr[i]; \
} while (0)

#define TXC_ABS_SUM(arr, sum) \
do { \
	int i = 0; \
	int size = TXC_SIZE(arr); \
	for (i = 0; i < size; i++) \
		sum += TXC_ABS(arr[i]); \
} while (0)

#define IS_CLOSE(arr, close) \
do { \
	int i = 0; \
	int size = TXC_SIZE(arr); \
	close = 1; \
	while (i < size && close == 1) { \
		if (arr[i] >= 0) \
			i++; \
		else \
			close = 0; \
	} \
}while (0)

#define IS_AWAY(arr, away) \
do { \
	int i = 0; \
	int size = TXC_SIZE(arr); \
	away = 1; \
	while (i < size && away == 1) { \
		if ( arr[i] <= 0) \
			i++; \
		else \
			away = 0; \
	}  \
}while (0)

#define INPUT_NAME_PS       "proximity"
#define MISC_DEV_NAME       "ps_dev"
#define PS_CAL_FILE_PATH    "/persist/sensors/xtalk_cal"

#define saturation_delay    100
#define sequence_dealy      15
#define OIL_EFFECT          35
#define ps_ary_size         4

static int far_ps_min = PA24_PS_OFFSET_MAX;
static int saturation_flag = 0;
static const int ps_steady = ps_ary_size + 4;
u8 ps_seq_far[ps_ary_size];
u8 ps_seq_oil[ps_ary_size];
u8 ps_seq_near[ps_ary_size];
static int oil_occurred = 0;

static dev_t const pa224_proximity_dev_t = MKDEV(MISC_MAJOR, 101);
static struct class         *proximity_class;
#ifdef SENSORS_CLASS_DEV
static struct sensors_classdev sensors_proximity_cdev = {
	 .name = "pa224-proximity",
	 .vendor = "txc",
	 .version = 1,
	 .handle = SENSORS_PROXIMITY_HANDLE,
	 .type = SENSOR_TYPE_PROXIMITY,
	 .max_range = "5",
	 .resolution = "5.0",
	 .sensor_power = "3",
	 .min_delay = 1000, /* in microseconds */
	 .fifo_reserved_event_count = 0,
	 .fifo_max_event_count = 0,
	 .enabled = 0,
	 .delay_msec = 100,
	 .sensors_enable = NULL,
	 .sensors_poll_delay = NULL,
 };
#endif
/* Global Variant */
static int ps_had_load_cal_param = 0;
struct i2c_client *pa224_i2c_client = NULL;
struct pa224_data *pdev_data = NULL;

/*----------------------------------------------------------------------------*/
/*
 * internally used functions
 */
/* I2C Read */
static int i2c_read_reg(struct i2c_client *client, u8 reg, u8 *buf)
{
	int ret = 0;
	int i = 0;

	struct pa224_data *data = i2c_get_clientdata(client);

	mutex_lock(&data->i2c_lock);

	for (i = 0; i < I2C_RETRY_TIMES; i++) {
		ret = i2c_smbus_read_byte_data(client, reg);
		if (ret < 0) {
			SENSOR_LOG_ERROR("failed to read i2c addr=%x\n", data->client->addr);
			msleep(I2C_RETRY_DELAY);
		} else {
			*buf = (u8) ret;
			mutex_unlock(&data->i2c_lock);
			return 0;	
		}
	}
	mutex_unlock(&data->i2c_lock);
	
	return ret;
}
/* I2C Write */
static int i2c_write_reg(struct i2c_client *client, u8 reg, u8 value)
{
	int ret = 0;
	int i = 0;

	struct pa224_data *data = i2c_get_clientdata(client);

	mutex_lock(&data->i2c_lock);
	for (i = 0; i < I2C_RETRY_TIMES; i++)
	{
		ret = i2c_smbus_write_byte_data(client, reg, value);
		if (ret < 0) {
			SENSOR_LOG_ERROR("failed to write i2c addr=%x\n", 0x1E);
			msleep(I2C_RETRY_DELAY);
		} else {
			mutex_unlock(&data->i2c_lock);
			return 0;
		}
	}
	mutex_unlock(&data->i2c_lock);

	return ret;
}
/* Calibration file handle*/
static int pa224_read_file(char *filename, u8* param)
{
	struct file  *fop;
	mm_segment_t old_fs;

	fop = filp_open(filename, O_RDONLY, 0444);
	if (IS_ERR(fop)) {
		SENSOR_LOG_INFO("Filp_open error!! Path = %s\n", filename);
		return -ERR_FILE_OPS;
	}

	old_fs = get_fs();
	set_fs(get_ds()); //set_fs(KERNEL_DS);

	fop->f_op->llseek(fop, 0, 0);
	fop->f_op->read(fop, param, strlen(param), &fop->f_pos);

	set_fs(old_fs);

	filp_close(fop, NULL);

	return 0;
}

static ssize_t pa224_write_file(char *filename, u8* param, int count)
{
	struct file  *fop;
	mm_segment_t old_fs;

	fop = filp_open(filename, O_CREAT | O_RDWR | O_TRUNC, 0666);
	if (IS_ERR(fop)) {
		SENSOR_LOG_INFO("Create file error!! Path = %s\n",filename);
		return -ERR_FILE_OPS;
	}

	old_fs = get_fs();
	set_fs(get_ds()); //set_fs(KERNEL_DS);
	fop->f_op->write(fop, (char *)param, count, &fop->f_pos);
	set_fs(old_fs);

	filp_close(fop, NULL);
	SENSOR_LOG_INFO("save PS calibration file Success!!");
	return 0;
}
static void pa224_ps_load_calibration_param(char *filename,struct i2c_client *client)
{
	struct pa224_data *data = i2c_get_clientdata(client);
	u8 param[2] = {PA24_PS_OFFSET_DEFAULT, PA24_MIN_NEAR_CNT};
	int ret;

	ret = pa224_read_file(filename, param);
	if (ret < 0) {
		data->crosstalk = PA24_PS_OFFSET_DEFAULT;
		data->near_diff_cnt = PA24_MIN_NEAR_CNT;
		data->far_diff_cnt = PA24_MIN_NEAR_CNT / 2;
	} else {
		data->crosstalk = param[0];
		data->near_diff_cnt = param[1];
		data->far_diff_cnt = param[1] / 2;
	}
	SENSOR_LOG_INFO("prox debug: crosstalk = %d near_diff_cnt = %d\n", data->crosstalk, data->near_diff_cnt);

	ps_had_load_cal_param = 1;
}
static int pa224_enable_ps(struct i2c_client *client, int enable)
{
	struct pa224_data *data = i2c_get_clientdata(client);
	struct pa224_platform_data *pdata=data->platform_data;
	int i = 0;
	int err = 0;
	u8 psdata = 0;

	if (!ps_had_load_cal_param)
		pa224_ps_load_calibration_param(PS_CAL_FILE_PATH, client);
	data->ps_enable = enable;

	if (PS_POLLING) {
		i2c_write_reg(client, REG_CFG0, (enable << 1));
		if (enable)
			schedule_delayed_work(&data->ps_dwork, msecs_to_jiffies(data->ps_enable_delay));
	} else if (!PS_POLLING) {
		if (enable) {
			if (!data->vdd_always_on) {
				if (pdata && pdata->power_on) {
					pdata->power_on(true);
					pa224_init_client(client);
				}
			}
			saturation_flag = 0;
			oil_occurred = 0;		
			data->ps_status = PA24_PS_FAR_DISTANCE;
			data->ps_last_status = PA24_PS_UNKOWN_DISTANCE;

			for (i = 0;i < ps_ary_size; i++)
			{
				ps_seq_far[i] = 255;
				ps_seq_oil[i] = 255;
				ps_seq_near[i]= 255;
			}

			i2c_write_reg(client, REG_PS_TH, 0xFF);
			i2c_write_reg(client, REG_PS_TL, 0x00);

			i2c_write_reg(client, REG_CFG1 ,(PA24_LED_CURR << 4) | (PA24_PS_PRST << 2));
			i2c_write_reg(client, REG_CFG2, (PA24_PS_MODE << 6) | (PA24_PS_SET << 2));

			i2c_write_reg(client, REG_CFG0, (enable << 1));
			msleep(PA24_PS_ENABLE_DELAY);

			/* Window type */
			data->ps_thrd_high = PA24_PS_OFFSET_MAX;
			data->ps_thrd_low = PA24_PS_OFFSET_MAX - 1;

			/* report far/near event when first enable*/
			err= i2c_read_reg(client, REG_PS_DATA, &psdata);
			if (err < 0) {
			    SENSOR_LOG_ERROR("i2c_read function err = %d\n",err);
			    return -1;
			}
			if (psdata <= data->ps_thrd_low)
			    data->ps_status = PA24_PS_FAR_DISTANCE;
			else if (psdata >= data->ps_thrd_high)
			    data->ps_status = PA24_PS_NEAR_DISTANCE;
			pa224_report_event(data);

			/* set threshold and start irq*/
			pa224_irq_enable(data, true, false);
			i2c_write_reg(client, REG_PS_TH, data->ps_thrd_high);
			i2c_write_reg(client, REG_PS_TL, data->ps_thrd_low);

		} else {
			i2c_write_reg(client, REG_CFG0, (enable << 1));
			pa224_irq_enable(data, false, true);
			if (!data->vdd_always_on) {
				if (pdata && pdata->power_on) {
					pdata->power_on(false);
				}
			}
		}
	}

	return 0;
}

void pa224_swap(u8 *x, u8 *y)
{
	u8 temp = *x;
	*x = *y;
	*y = temp;
}

static int pa224_get_psoffset(struct i2c_client *client)
{
	struct pa224_data *data = i2c_get_clientdata(client);

	int i, j;
	int ret;
	u16 sum_of_pdata = 0;
	int tmp_uncover_data = 0;
	u8 temp_pdata[20], cfg0data = 0, cfg2data = 0;
	unsigned int ArySize = 12;
	int scale = ArySize / 4;

	SENSOR_LOG_INFO("START proximity sensor calibration\n");

	/*Offset mode & disable intr from ps*/
	ret = i2c_read_reg(client, REG_CFG2, &cfg2data);
	ret = i2c_write_reg(client, REG_CFG2, 0x08);

	/*Set crosstalk = 0*/
	ret = i2c_write_reg(client, REG_PS_OFFSET, 0x00);

	/*PS On*/
	ret = i2c_read_reg(client, REG_CFG0, &cfg0data);
	ret = i2c_write_reg(client, REG_CFG0, cfg0data | 0x02);

	for (i = 0; i < ArySize; i++)
	{
		mdelay(50);
		ret = i2c_read_reg(client,REG_PS_DATA,temp_pdata + i);
		SENSOR_LOG_INFO("temp_data = %d\n", temp_pdata[i]);
	}

	/* pdata sorting */
	for (i = 0; i < ArySize - 1; i++)
	for (j = i + 1; j < ArySize; j++)
		if (temp_pdata[i] > temp_pdata[j])
			pa224_swap(temp_pdata + i, temp_pdata + j);

	/* calculate the cross-talk using central 10 data */
	for (i = scale; i < ArySize - scale; i++)
	{
		SENSOR_LOG_INFO("temp_pdata = %d\n", temp_pdata[i]);
		sum_of_pdata = sum_of_pdata + temp_pdata[i];
	}
	tmp_uncover_data = sum_of_pdata * 2 / ArySize;
	if (tmp_uncover_data > PA24_PS_MAX_UNCOVER_DATA) {
		SENSOR_LOG_ERROR("uncover data too big, keep sensor naked or structure is not qualified!\n");
		ret = i2c_write_reg(client, REG_CFG2, cfg2data);
		return tmp_uncover_data;
	}
	mutex_lock(&data->dev_lock);
	data->crosstalk = tmp_uncover_data;
	mutex_unlock(&data->dev_lock);
	SENSOR_LOG_INFO("sum_of_pdata = %d   cross_talk = %d\n",
                        sum_of_pdata, data->crosstalk);

	/* Restore CFG2 */
	ret = i2c_write_reg(client, REG_CFG2, cfg2data);

	SENSOR_LOG_INFO("FINISH get proximity sensor\n");

	return data->crosstalk;
}
/*
 * return value
 * -1: need naked calibration
 * -2: need 3cm gray-card cailibraion
 */
static int pa224_run_calibration(struct i2c_client *client)
{
	struct pa224_data *data = i2c_get_clientdata(client);
	int i, j;	
	int ret;
	u16 sum_of_pdata = 0;
	int tmp_near_diff = 0;
	u8 temp_pdata[20], buftemp[2], cfg0data = 0, cfg2data = 0;
	unsigned int ArySize = 12;
	int scale = ArySize / 4;
		
	SENSOR_LOG_INFO("start calc base-noise value\n");
	if (data->crosstalk < PA24_PS_UNCOVER_MIN_SAFE || data->crosstalk > PA24_PS_MAX_UNCOVER_DATA) {
		SENSOR_LOG_ERROR("Need Run Naked Calibration first or fail!\n");
		return -ERR_NAKED_CAL;
	}

	/* Prevent interrput */
	/*Offset mode & disable intr from ps*/
	ret = i2c_read_reg(client, REG_CFG2, &cfg2data);
	ret = i2c_write_reg(client, REG_CFG2, 0x08);

	/*Set crosstalk = 0*/	
	ret = i2c_write_reg(client, REG_PS_OFFSET, 0x00);

	/*PS On*/
	ret = i2c_read_reg(client, REG_CFG0, &cfg0data);
	ret = i2c_write_reg(client, REG_CFG0, cfg0data | 0x02); 		

	for (i = 0; i < ArySize; i++)
	{
		mdelay(50);
		ret = i2c_read_reg(client,REG_PS_DATA,temp_pdata + i);
		SENSOR_LOG_INFO("temp_data = %d\n", temp_pdata[i]);
	}	
	
	/* pdata sorting */
	for (i = 0; i < ArySize - 1; i++)
	for (j = i + 1; j < ArySize; j++)
		if (temp_pdata[i] > temp_pdata[j])
			pa224_swap(temp_pdata + i, temp_pdata + j);	
	
	/* calculate the cross-talk using central 10 data */
	for (i = scale; i < ArySize - scale; i++)
	{
		SENSOR_LOG_INFO("temp_pdata = %d\n",temp_pdata[i]);
		sum_of_pdata = sum_of_pdata + temp_pdata[i];
	}

	tmp_near_diff = sum_of_pdata * 2 / ArySize - data->crosstalk;
	SENSOR_LOG_INFO("tmp_near_diff = %d\n", tmp_near_diff);
	if (tmp_near_diff < PA24_MIN_NEAR_CNT || tmp_near_diff > PA24_MAX_NEAR_CNT) {
		SENSOR_LOG_ERROR("thres calibration failed\n");
		ret = i2c_write_reg(client, REG_CFG2, cfg2data);
		return -ERR_THRES_CAL;
	}
	mutex_lock(&data->dev_lock);
	data->near_diff_cnt = tmp_near_diff;
	data->far_diff_cnt = tmp_near_diff / 2;
	mutex_unlock(&data->dev_lock);
	SENSOR_LOG_INFO("sum_of_pdata = %d   near_diff_cnt = %d\n",
                        sum_of_pdata, data->near_diff_cnt);

	ret = i2c_write_reg(client, REG_CFG2, cfg2data);


	buftemp[0] = (u8)data->crosstalk;
	buftemp[1] = (u8)data->near_diff_cnt;

	if (pa224_write_file(PS_CAL_FILE_PATH, buftemp, sizeof(buftemp)) < 0) {
		SENSOR_LOG_INFO("Open PS calibration file error!!");
		return -ERR_FILE_OPS;
	}

	SENSOR_LOG_INFO("FINISH proximity sensor calibration\n");

	return data->near_diff_cnt;
}
static int pa224_get_ps_value(struct i2c_client *client)
{
	u8 regdata = 0;
	
	i2c_read_reg(client, REG_PS_DATA, &regdata);
	
	return regdata;
}

static void pa224_report_event(struct pa224_data *data)
{
	if (data->ps_status != data->ps_last_status) {
		input_report_rel(data->ps_input_dev, REL_RZ, data->ps_status);
		input_sync(data->ps_input_dev);
		data->ps_last_status = data->ps_status;
		SENSOR_LOG_DEBUG("data->ps_status = %d\n",data->ps_status);
	}
	return;
}
/*
 * Initialization function
 */

static int pa224_init_client(struct i2c_client *client)
{
	struct pa224_data *data = i2c_get_clientdata(client);
	
	/* Dealy time setting */
	data->ps_poll_delay = PA24_PS_POLL_DELAY;
	data->ps_enable_delay = PA24_PS_ENABLE_DELAY;

  	/* Initialize Sensor */
  	i2c_write_reg(client,REG_CFG1,
		(PA24_LED_CURR	<< 4)| (PA24_PS_PRST << 2) );
  
  	i2c_write_reg(client,REG_CFG3,
		(PA24_INT_TYPE	<< 6)| (PA24_PS_PERIOD << 3) );
  
  	i2c_write_reg(client, REG_PS_SET, 0x82); 

	i2c_write_reg(client, REG_CFG4, 0x0C);

	i2c_write_reg(client,REG_CFG2,
		((PA24_PS_MODE	<< 6)|(PA24_INT_TYPE << 2)));
	SENSOR_LOG_INFO("pa224_init_client ok\n");
	
	return 0;
}
static int pa224_get_object(struct i2c_client *client)
{
	struct pa224_data *data = i2c_get_clientdata(client);	
	u8 psdata = pa224_get_ps_value(client);

	SENSOR_LOG_INFO("PS:%d\n", psdata);
	SENSOR_LOG_INFO("ps_status:%d\n", data->ps_status);
	SENSOR_LOG_INFO("ps_thrd_low:%d\n", data->ps_thrd_low);
	SENSOR_LOG_INFO("ps_thrd_high:%d\n", data->ps_thrd_high);
	switch (data->ps_status) {
		case PA24_PS_NEAR_DISTANCE:
			if (psdata < data->ps_thrd_low) {
				data->ps_status = PA24_PS_FAR_DISTANCE;
				SENSOR_LOG_INFO("Object Far\n");
			}
			break;
		case PA24_PS_FAR_DISTANCE:
			if (psdata > data->ps_thrd_high) {
				data->ps_status = PA24_PS_NEAR_DISTANCE;
				SENSOR_LOG_INFO("Object Near\n");
			}
			break;
	}

	return data->ps_status;
}
/*----------------------------------------------------------------------------*/

/* For HAL to Enable PS */
static ssize_t pa224_show_enable_ps_sensor(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct pa224_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", data->ps_enable);
}
static ssize_t pa224_store_enable_ps_sensor(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct pa224_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	unsigned long val = simple_strtoul(buf, NULL, 10);
	int err = -1;

	SENSOR_LOG_INFO("enable ps sensor (%ld)\n", val);

	if ((val != 0) && (val != 1)) {
		SENSOR_LOG_INFO("enable ps sensor=%ld\n", val);
		return count;
	}

	err = pa224_enable_ps(client, val);
	if (err < 0)
		SENSOR_LOG_ERROR("pa224_enable_ps error\n");
	return count;
}
#ifdef SENSORS_CLASS_DEV
static int pa224_ps_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct pa224_data *data = container_of(sensors_cdev,
			struct pa224_data, ps_cdev);
	SENSOR_LOG_INFO("enter\n");
	if ((enable != 0) && (enable != 1)) {
		SENSOR_LOG_ERROR("invalid value(%d)\n", enable);
		return -EINVAL;
	}

	return pa224_enable_ps(data->client, enable);
}
#endif
static ssize_t pa224_show_ps_poll_delay(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct pa224_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", data->ps_poll_delay);	// return in micro-second
}

static ssize_t pa224_store_ps_poll_delay(struct device *dev,
					struct device_attribute *attr, const char *buf, size_t count)
{
	struct pa224_data *data = dev_get_drvdata(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);

	if (val < 10)
	    val = 10;

	data->ps_poll_delay = (unsigned int)val;

	return count;
}

/* PS Value */
static ssize_t pa224_show_ps(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct pa224_data *data = dev_get_drvdata(dev);
	input_report_rel(data->ps_input_dev, REL_RZ, data->ps_status);
	input_sync(data->ps_input_dev);
	return sprintf(buf, "%d\n", pa224_get_ps_value(data->client));
}
/* Write/Read Register data */
static ssize_t pa224_show_reg(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct pa224_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	int ret;
	int i = 0;
	int count = 0;
	u8 regdata = 0;

	for (i = 0; i < 19; i++)
	{	
		ret = i2c_read_reg(client, 0x00+i, &regdata);

		if (ret < 0)
			break;
		else
			count += sprintf(buf+count, "[%x] = (%x)\n", 0x00+i, regdata);
	}

	return count;	
}
static ssize_t pa224_store_reg(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct pa224_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	int addr, cmd, ret;

	if(2 != sscanf(buf, "%x %x", &addr, &cmd))
	{
		SENSOR_LOG_ERROR("invalid format: '%s'\n", buf);
		return count;
	}
		
	ret = i2c_write_reg(client, addr, cmd);
	
	return count;	
}
static ssize_t pa224_ps_offset_store(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct pa224_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	pa224_get_psoffset(client);
	return count;
}
static ssize_t pa224_ps_offset_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct pa224_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	return sprintf(buf, "%d\n", pa224_get_psoffset(client));
}
/* PS Calibration */
static ssize_t pa224_ps_calibration_store(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct pa224_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	pa224_run_calibration(client);
	return count;
}
static ssize_t pa224_ps_calibration_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct pa224_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	return sprintf(buf, "%d\n", pa224_run_calibration(client));
	
}
/* Device init */
static ssize_t pa224_store_dev_init(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}
static ssize_t pa224_chip_name_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	sprintf(buf, "%s\n",PA224_DRV_NAME);
	return strlen(buf);
}
static ssize_t pa224_prox_debug_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct pa224_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "prox debug flag is %s\n", data->flag_prox_debug? "true" : "false");
}
static ssize_t pa224_cal_debug_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct pa224_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "prox debug: crosstalk = %d near_diff_cnt = %d\n", data->crosstalk, data->near_diff_cnt);
}
static ssize_t pa224_prox_uncover_data_max_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	SENSOR_LOG_DEBUG("max uncover data is %d\n", PA24_PS_MAX_UNCOVER_DATA);
	return sprintf(buf, "%d\n", PA24_PS_UNCOVER_MAX_SAFE);
}
static ssize_t pa224_ps_max_thres_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct pa224_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", data->crosstalk + PA24_MAX_NEAR_CNT);
}
static ssize_t pa224_ps_min_thres_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct pa224_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", data->crosstalk + PA24_MIN_NEAR_CNT);
}

static ssize_t pa224_prox_debug_store(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	unsigned long val = 0;
	struct pa224_data *data = dev_get_drvdata(dev);
	SENSOR_LOG_ERROR("enter\n");
	if (kstrtoul(buf, 10, &val)) {
		return -EINVAL;
	}

	if (val) {
		data->flag_prox_debug = true;
	} else {
		data->flag_prox_debug = false;
	}
	SENSOR_LOG_ERROR("exit\n");
	return size;
}
static ssize_t pa224_prox_data_max_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return sprintf(buf, "%d", PA24_PS_TH_MAX);
}
static struct device_attribute attrs_prox_device[] = {
	__ATTR(enable, 0664, pa224_show_enable_ps_sensor, pa224_store_enable_ps_sensor),
	__ATTR(delay, 0664, pa224_show_ps_poll_delay, pa224_store_ps_poll_delay),
	__ATTR(prox_value, 0444, pa224_show_ps, NULL),
	__ATTR(reg, 0664, pa224_show_reg, pa224_store_reg),
	__ATTR(prox_thres, 0664, pa224_ps_calibration_show, pa224_ps_calibration_store),
	__ATTR(prox_thres_max, 0440, pa224_ps_max_thres_show, NULL),
	__ATTR(prox_thres_min, 0440, pa224_ps_min_thres_show, NULL),
	__ATTR(prox_init, 0220, NULL, pa224_store_dev_init),
	__ATTR(prox_debug, 0640, pa224_prox_debug_show, pa224_prox_debug_store),
	__ATTR(prox_data_max, 0444, pa224_prox_data_max_show, NULL),
	__ATTR(prox_uncover_max, 0444, pa224_prox_uncover_data_max_show, NULL),
	__ATTR(chip_name, 0444, pa224_chip_name_show, NULL),
	__ATTR(prox_offset_cal, 0664, pa224_ps_offset_show, pa224_ps_offset_store),
	__ATTR(prox_cal_debug, 0444, pa224_cal_debug_show, NULL),
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attrs_prox_device); i++)
	{
		if (device_create_file(dev, attrs_prox_device + i))
			return -ENODEV;
	}
	return 0;
}

static void remove_sysfs_interfaces(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(attrs_prox_device); i++)
		device_remove_file(dev, attrs_prox_device + i);
	return;
}
/*work que function*/
static void pa224_work_func_proximity(struct work_struct *work)
{
	struct pa224_data *data = container_of(work, 
						struct pa224_data, ps_dwork.work);
	int Pval;
		
  	Pval = pa224_get_object(data->client);
	SENSOR_LOG_INFO("PS value: %d\n", Pval);

	input_report_rel(data->ps_input_dev, REL_DIAL, Pval ? PA24_PS_FAR_DISTANCE: PA24_PS_NEAR_DISTANCE);
	input_sync(data->ps_input_dev);

	if(PS_POLLING && data->ps_enable)
		schedule_delayed_work(&data->ps_dwork, msecs_to_jiffies(data->ps_poll_delay));
	
}

static void pa224_get_ps_slope_array(u8 *ps_seq, int *slope, u8 ps, int arysize)
{
	int i;

	for (i=0; i<arysize-1; i++)
	{
		ps_seq[arysize-1-i] = ps_seq[arysize-1-i-1];
		if (ps_seq[arysize-1-i] == 0)
			ps_seq[arysize-1-i] = ps;
	}
	ps_seq[0] = (int)ps;

	for (i=0; i<arysize-1; i++)
	{
		slope[i] = (int)(ps_seq[i] - ps_seq[i+1]);
	}
	return;
}
static int pa224_check_intr(struct i2c_client *client) 
{
	struct pa224_data *data = i2c_get_clientdata(client);
	int res;
	u8 psdata=0;
	u8 cfgdata=0;
	static int far_loop = 0;

	int slope[ps_ary_size-1];	
	int sum = 0, abs_sum = 0, ps_sum = 0;
	SENSOR_LOG_INFO("enter.\n");
	res = i2c_read_reg(client, REG_PS_DATA, &psdata);
	if (res < 0) {
		SENSOR_LOG_ERROR("i2c_read function err res = %d\n",res);
		return -1;
	}

	if (data->flag_prox_debug) {
		input_report_rel(data->ps_input_dev, REL_MISC, psdata > 0 ? psdata : 1);
		input_sync(data->ps_input_dev);
		goto check_intr_exit;
	}
	//SUNLIGHT
	if (psdata == 0) {

		if (data->ps_status == PA24_PS_NEAR_DISTANCE) {
			i2c_write_reg(client,REG_CFG1,
						(PA24_LED_CURR << 4)| (PA24_PS_PRST << 2) );
		}
		saturation_flag = 1;

		if (oil_occurred && far_ps_min < PA24_PS_OFFSET_MAX) {
			data->ps_thrd_high = far_ps_min + OIL_EFFECT + data->near_diff_cnt;
			data->ps_thrd_low = far_ps_min + OIL_EFFECT;			
		} else if (!oil_occurred && far_ps_min < PA24_PS_OFFSET_MAX) {
			data->ps_thrd_high = far_ps_min + data->near_diff_cnt;
			data->ps_thrd_low = far_ps_min + data->near_diff_cnt - data->far_diff_cnt;//data->near_diff_cnt - PA24_NEAR_FAR_CNT;
		} else if (far_ps_min == PA24_PS_OFFSET_MAX) {
			data->ps_thrd_high = PA24_PS_OFFSET_MAX;
			data->ps_thrd_low = PA24_PS_OFFSET_MAX - 1;
		}
		msleep(saturation_delay);
		SENSOR_LOG_INFO("Sun light!!, ht=%d, lt=%d, far_ps_min=%d\n", data->ps_thrd_high, data->ps_thrd_low, far_ps_min);
		data->ps_status = PA24_PS_FAR_DISTANCE;
		goto check_intr_exit;
	}
	//FARTHER AWAY
	if (psdata < data->ps_thrd_low && data->ps_status == PA24_PS_FAR_DISTANCE) {

		pa224_get_ps_slope_array(ps_seq_far, slope, psdata, ps_ary_size);
		TXC_SUM(ps_seq_far, ps_sum);
		TXC_SUM(slope, sum);
		TXC_ABS_SUM(slope, abs_sum);
		SENSOR_LOG_INFO("slope : %d %d %d\n", slope[2], slope[1], slope[0]);
		SENSOR_LOG_INFO("value : %d %d %d %d\n", ps_seq_far[3], ps_seq_far[2], ps_seq_far[1], ps_seq_far[0]);
		SENSOR_LOG_INFO("saturation_flag=%d\n", saturation_flag);
		//If saturation happened, the average ps value must be greater than (far_ps_min-10) and also  steady
		if ( (saturation_flag && ps_sum/ps_ary_size >= ( far_ps_min > 10 ? (far_ps_min - 10) : far_ps_min ))
			  || !saturation_flag || (saturation_flag && far_ps_min == PA24_PS_OFFSET_MAX) )
		{
			//STEADY
			if (abs_sum < ps_steady) {
				if (saturation_flag)
					saturation_flag = 0;				

				data->ps_status = PA24_PS_FAR_DISTANCE;
				oil_occurred = 0;
				far_ps_min = ps_sum / ps_ary_size;
				data->ps_thrd_high = far_ps_min + data->near_diff_cnt;
				data->ps_thrd_low = far_ps_min > 15 ? (far_ps_min - 5) : 15;
				i2c_write_reg(client, REG_CFG3, (PA24_INT_TYPE << 6) | (PA24_PS_PERIOD << 3));
				SENSOR_LOG_INFO("FAR, far_ps_min %3d high low : %3d %3d\n", far_ps_min, data->ps_thrd_high, data->ps_thrd_low);
				pa224_report_event(data);
			}	
		}
		msleep(sequence_dealy);	
	}
	//NEAR 
	else if (psdata > data->ps_thrd_high)
	{
#if 1
		int i = 0;
		for (i = 0; i < ps_ary_size; i++) {
			res = i2c_read_reg(client, REG_PS_DATA, ps_seq_near+i);
			if (i > 0)
				slope[i-1] = (int)(ps_seq_near[i] - ps_seq_near[i-1]);
			mdelay(5);
		}
#endif
		//pa224_get_ps_slope_array(ps_seq_near, slope, psdata, ps_ary_size);
		SENSOR_LOG_ERROR("slope : %d %d %d\n", slope[2], slope[1], slope[0]);
		SENSOR_LOG_ERROR("value : %d %d %d %d\n", ps_seq_near[3], ps_seq_near[2], ps_seq_near[1], ps_seq_near[0]);
		TXC_ABS_SUM(slope, abs_sum);
		oil_occurred = 0;
		if (abs_sum < ps_steady) {
			data->ps_status = PA24_PS_NEAR_DISTANCE;
			i2c_write_reg(client,REG_CFG1,
					(PA24_LED_CURR << 4)| (1 << 2) );

			if (psdata >= 254) {
				far_loop = 0;
				oil_occurred = 1;
				data->ps_thrd_low = far_ps_min + OIL_EFFECT;
				data->ps_thrd_high = 0xFF;
			} else {
				data->ps_thrd_low = far_ps_min + (data->near_diff_cnt - data->far_diff_cnt);//(PA24_NEAR_FAR_CNT - 3));
				data->ps_thrd_high = 254;//(far_ps_min + OIL_EFFECT);
			}
			SENSOR_LOG_INFO("NER, far_ps_min:%3d psdata:%3d high low:%3d %3d\n", far_ps_min, psdata, data->ps_thrd_high, data->ps_thrd_low);
            pa224_report_event(data);
		} else if (abs_sum > 20) {
			/*Flicker light*/
			i2c_write_reg(client, REG_CFG3, (PA24_INT_TYPE << 6)| (0 << 3));
			SENSOR_LOG_ERROR("Flicker light!!!!");
		}
	}
	//FAR AWAY
	if (psdata < data->ps_thrd_low && data->ps_status == PA24_PS_NEAR_DISTANCE)
	{
		if (oil_occurred) {
			far_loop++;
			pa224_get_ps_slope_array(ps_seq_oil, slope, psdata, ps_ary_size);
			TXC_SUM(ps_seq_oil, ps_sum);
			TXC_SUM(slope, sum);
			TXC_ABS_SUM(slope, abs_sum);	
			SENSOR_LOG_INFO("slope : %d %d %d\n", slope[2], slope[1], slope[0]);
			SENSOR_LOG_INFO("value : %d %d %d %d\n", ps_seq_oil[3], ps_seq_oil[2], ps_seq_oil[1], ps_seq_oil[0]);
			//STEADY
			if (abs_sum < ps_steady || far_loop > 10) {
				i2c_write_reg(client,REG_CFG1,
					(PA24_LED_CURR << 4)| (PA24_PS_PRST << 2) );			
				data->ps_status = PA24_PS_FAR_DISTANCE;
				oil_occurred = 0;
				if (far_loop <= 10) {
					far_ps_min = ps_sum / ps_ary_size;
					data->ps_thrd_high = far_ps_min + data->near_diff_cnt;
					data->ps_thrd_low = far_ps_min > 5 ? (far_ps_min - 5) : 5;
				} else {
					SENSOR_LOG_INFO("far_loop > 10\n");
					far_ps_min = far_ps_min + 15;
					data->ps_thrd_high = far_ps_min + data->near_diff_cnt;
					data->ps_thrd_low = far_ps_min + (data->near_diff_cnt - data->far_diff_cnt);//PA24_NEAR_FAR_CNT);
				}	
				i2c_write_reg(client, REG_CFG3, (PA24_INT_TYPE << 6)| (PA24_PS_PERIOD << 3));
				SENSOR_LOG_INFO("OIL to FAR, far_ps_min %3d high low : %3d %3d\n", far_ps_min, data->ps_thrd_high, data->ps_thrd_low);		
                pa224_report_event(data);
            }
			msleep(sequence_dealy);
		} else {
			i2c_write_reg(client,REG_CFG1,
					(PA24_LED_CURR << 4)| (PA24_PS_PRST << 2) );
			data->ps_status = PA24_PS_FAR_DISTANCE;
			data->ps_thrd_high = far_ps_min + data->near_diff_cnt;
			data->ps_thrd_low = far_ps_min + data->near_diff_cnt - data->far_diff_cnt;//PA24_NEAR_FAR_CNT;
			SENSOR_LOG_ERROR("FAR, far_ps_min %3d high low : %3d %3d\n", far_ps_min, data->ps_thrd_high, data->ps_thrd_low);
			pa224_report_event(data);
        }

	}

check_intr_exit:

	i2c_write_reg(client, REG_PS_TL, data->ps_thrd_low);
	i2c_write_reg(client, REG_PS_TH, data->ps_thrd_high);

	/* Clear PS INT FLAG */
	res = i2c_read_reg(client, REG_CFG2, &cfgdata);
	if (res < 0) {
		SENSOR_LOG_ERROR("i2c_read function err res = %d\n",res);
		return -ERR_DEV_OPS;
	}
	cfgdata = cfgdata & 0xFD;

	res = i2c_write_reg(client,REG_CFG2, cfgdata);
	if (res < 0) {
		SENSOR_LOG_ERROR("i2c_send function err res = %d\n",res);
		return -ERR_DEV_OPS;
	}

	res = i2c_read_reg(client, REG_CFG2, &cfgdata);
	if (cfgdata & 2) {
		cfgdata = cfgdata & 0xFD;
		res = i2c_write_reg(client,REG_CFG2, cfgdata);
 		if (res < 0) {
		    SENSOR_LOG_ERROR("i2c_send function err res = %d\n",res);
		    return -1;  
		}
	}
	SENSOR_LOG_INFO("exit.\n");
	return 0;
}

static void pa224_irq_enable(struct pa224_data *data, bool enable, bool flag_sync)
{
	if (enable == data->irq_enabled) {
	    SENSOR_LOG_DEBUG("doubule %s irq %d\n",enable? "enable" : "disable",
		    data->irq);
	    return;
	} else {
		    data->irq_enabled = enable;
	}

	if (enable) {
	    enable_irq(data->irq);
	} else {
	    if (flag_sync) {
	        disable_irq(data->irq);
	    } else {
	        disable_irq_nosync(data->irq);
	    }
	}
}

static void pa224_work_func_irq(struct work_struct *work)
{
	struct pa224_data *data;
	struct i2c_client *client;
	SENSOR_LOG_INFO("IRQ Work INT\n");
	data = container_of((struct work_struct *)work, struct pa224_data, irq_dwork);
	client = data->client;
	/* Add Oil Alg */
	wake_lock_timeout(&data->pa224_wake_lock, msecs_to_jiffies(1000));
	pa224_check_intr(client);
}

static irqreturn_t pa224_irq(int irq, void *info)
{
	struct i2c_client *client=(struct i2c_client *)info;
	struct pa224_data *data = i2c_get_clientdata(client);
	if (0 == queue_work(data->irq_work_queue, &data->irq_dwork)){
	    SENSOR_LOG_ERROR("schedule_work failed!\n");
	}
	return IRQ_HANDLED;
}
static int pa224_open(struct inode *inode, struct file *file)
{
	SENSOR_LOG_INFO("pa224 dev File Open\n");
	return 0;
}

static int pa224_release(struct inode *inode, struct file *file)
{
	SENSOR_LOG_INFO("pa224 dev File Close\n");
	return 0;
}


/*IOCTL*/
static long pa224_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int enable;
	int ret=-1;
	u8 alspsdata;
	if (pa224_i2c_client == NULL) {
		SENSOR_LOG_INFO("i2c driver not installed\n");
		return -ENODEV;
	}

	switch (cmd) {
		case PA24_IOCTL_PS_ENABLE:      	
			SENSOR_LOG_INFO("IOCTL PS enable=%lu\n",arg);		
			ret = copy_from_user(&enable,(void __user *)arg, sizeof(enable));
			if (ret) {
				SENSOR_LOG_INFO("PS enable copy data failed\n");
				return -EFAULT;
			}
			
			pa224_enable_ps(pa224_i2c_client, enable);

			break;	

		case PA24_IOCTL_PS_GET_DATA:    	 
			SENSOR_LOG_INFO("IOCTL PS Read data\n");
			alspsdata = pa224_get_ps_value(pa224_i2c_client);
			ret = copy_to_user((void __user *)arg,&alspsdata,sizeof(alspsdata));
			if (ret) {
				SENSOR_LOG_INFO("PS Read data copy data failed\n");
				return -EFAULT;
			}

			break;

		case PA24_IOCTL_PS_CALIBRATION:       
			SENSOR_LOG_INFO("IOCTL PS Calibration\n");
			alspsdata = pa224_get_psoffset(pa224_i2c_client);
			ret = copy_to_user((void __user *)arg,&alspsdata,sizeof(alspsdata));
			if (ret) {
				SENSOR_LOG_INFO("PS Calibration copy data failed\n");
				return -EFAULT;
			}		

			break;

		default:
			break;
	}
	return 0;
}

static const struct file_operations pa224_fops = {
	.owner = THIS_MODULE,
	.open = pa224_open,
	.release = pa224_release,
	.unlocked_ioctl = pa224_ioctl,
};

static struct miscdevice pa224_ps_device = {

	.minor = MISC_DYNAMIC_MINOR,
	.name = MISC_DEV_NAME,
	.fops = &pa224_fops,

};


/*Suspend/Resume*/
static int pa224_suspend(struct device *dev)
{
	SENSOR_LOG_ERROR("suspend\n");
	return 0;
}

static int pa224_resume(struct device *dev)
{
	SENSOR_LOG_ERROR("resume\n");
	return 0;
}

static int pa224_pinctrl_init(struct pa224_data *data, struct device *dev)
{
	int rc;

	data->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(data->pinctrl)) {
		SENSOR_LOG_ERROR("data->pinctrl is NULL\n");
		return PTR_ERR(data->pinctrl);
	}

	data->pin_default = pinctrl_lookup_state(data->pinctrl, "pa224_default");
	if (IS_ERR_OR_NULL(data->pin_default)) {
		SENSOR_LOG_ERROR("lookup default state failed\n");
		return PTR_ERR(data->pin_default);
	}

	data->pin_sleep = pinctrl_lookup_state(data->pinctrl, "pa224_sleep");
	if (IS_ERR_OR_NULL(data->pin_sleep)) {
		SENSOR_LOG_ERROR("lookup sleep state failed\n");
		return PTR_ERR(data->pin_sleep);
	}

	if (!IS_ERR_OR_NULL(data->pinctrl)) {
		rc = pinctrl_select_state(data->pinctrl, data->pin_default);
		if (rc) {
			SENSOR_LOG_ERROR("select default state failed\n");
			return rc;
		}
	}
	SENSOR_LOG_INFO("pinctrl init success\n");
	return 0;
}

static int sensor_regulator_configure(struct pa224_data *data, bool on)
{
	int rc;

	if (!on) {
		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd, 0,
				PA224_VDD_MAX_UV);

		regulator_put(data->vdd);
		regulator_disable(data->vdd);
	} else {
		data->vdd = regulator_get(&data->client->dev, "vdd");
		if (IS_ERR(data->vdd)) {
			rc = PTR_ERR(data->vdd);
			SENSOR_LOG_ERROR("Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}
		if (!IS_ERR_OR_NULL(data->vdd)) {
		    if (regulator_count_voltages(data->vdd) > 0) {
			    rc = regulator_set_voltage(data->vdd,
				    PA224_VDD_MIN_UV, PA224_VDD_MAX_UV);
			    if (rc) {
				    SENSOR_LOG_ERROR("Regulator set failed vdd rc=%d\n",
					    rc);
				    goto reg_vdd_put;
			    }
		    }

		    rc = regulator_enable(data->vdd);
		    if (rc) {
			    SENSOR_LOG_ERROR(
				    "Regulator enable vdd failed. rc=%d\n", rc);
			    goto reg_vdd_put;
		    }
		}
	}

	return 0;

reg_vdd_put:
	regulator_put(data->vdd);
	return rc;
}


static int sensor_regulator_power_on(struct pa224_data *data, bool on)
{
	int rc = 0;

	if (!on) {
	    if (!IS_ERR_OR_NULL(data->vdd)) {
	        rc = regulator_disable(data->vdd);
	        if (rc) {
			    SENSOR_LOG_ERROR(
				    "Regulator vdd disable failed rc=%d\n", rc);
			    return rc;
	        }
	    }
	} else {
	    if (!IS_ERR_OR_NULL(data->vdd)) {
	        rc = regulator_enable(data->vdd);
	        if (rc) {
			    SENSOR_LOG_ERROR(
				    "Regulator vdd enable failed rc=%d\n", rc);
			    return rc;
	        }
	    }
	}
	SENSOR_LOG_INFO("power %s\n", on ? "on":"off");
	mdelay(5);

	return rc;
}

static int sensor_platform_hw_power_on(bool on)
{
	if (pdev_data == NULL)
		return -ENODEV;

	sensor_regulator_power_on(pdev_data, on);

	return 0;
}

static int sensor_platform_hw_init(void)
{
	struct i2c_client *client;
	struct pa224_data *data;
	int error;

	if (pdev_data == NULL)
		return -ENODEV;

	data = pdev_data;
	client = data->client;

	error = sensor_regulator_configure(data, true);
	if (error < 0) {
		SENSOR_LOG_ERROR("unable to configure regulator\n");
		return error;
	}

	if (gpio_is_valid(data->platform_data->irq_gpio)) {
		/* configure pa224 irq gpio */
		SENSOR_LOG_INFO("gpio value is %d \n", gpio_get_value(data->platform_data->irq_gpio));
		error = gpio_request_one(data->platform_data->irq_gpio,
				GPIOF_DIR_IN,
				"pa224_irq_gpio");
		if (error) {
			SENSOR_LOG_ERROR("unable to request gpio %d\n",
				data->platform_data->irq_gpio);
		}

		data->irq = client->irq =
			gpio_to_irq(data->platform_data->irq_gpio);
	} else {
		SENSOR_LOG_ERROR("irq gpio not provided\n");
	}
	return 0;
}

static void sensor_platform_hw_exit(void)
{
	struct pa224_data *data = pdev_data;

	if (data == NULL)
		return;

	sensor_regulator_configure(data, false);

	if (gpio_is_valid(data->platform_data->irq_gpio))
		gpio_free(data->platform_data->irq_gpio);
}

static int sensor_parse_dt(struct device *dev,
		struct pa224_platform_data *pdata,
		struct pa224_data *data)
{
	struct device_node *np = dev->of_node;

	unsigned int tmp = 0;
	int rc = 0;

	/* set functions of platform data */
	pdata->init = sensor_platform_hw_init;
	pdata->exit = sensor_platform_hw_exit;
	pdata->power_on = sensor_platform_hw_power_on;

	/* irq gpio */
	rc = of_get_named_gpio(dev->of_node,"txc,irq-gpio", 0);
	if (rc < 0) {
		SENSOR_LOG_ERROR("Unable to read irq gpio\n");
		return rc;
	}
	pdata->irq_gpio = rc;
	SENSOR_LOG_INFO("irq gpio is %d\n", pdata->irq_gpio);

	/* vdd-always-on flag */
	rc = of_property_read_u32(np, "txc,vdd-always-on", &tmp);
	if (rc < 0) {
		SENSOR_LOG_ERROR("Unable to read vdd always on flag\n");
		return rc;
	}
	data->vdd_always_on = tmp;
	SENSOR_LOG_INFO("vdd always-on flag is %d\n", data->vdd_always_on);
	/* ps tuning data*/
	rc = of_property_read_u32(np, "txc,ps_threshold_low", &tmp);
	data->ps_thrd_low= (!rc ? tmp : 30);
	SENSOR_LOG_INFO("ps_threshold_low is %d\n", data->ps_thrd_low);

	rc = of_property_read_u32(np, "txc,ps_threshold_high", &tmp);
	data->ps_thrd_high = (!rc ? tmp : 120);    
	SENSOR_LOG_INFO("ps_threshold_low is %d\n", data->ps_thrd_high);

	return 0;
}
static int pa224_read_device_id(struct pa224_data *data)
{
	int retry_times = 3;
	u8 device_id;
	int res = -1;
	struct i2c_client *client = data->client;
	while (retry_times--) {
		res = i2c_read_reg(client, REG_ID, &device_id);
		if (res >= 0) {
			SENSOR_LOG_INFO("device_id = %d\n", device_id);
			if (device_id == 0x11) {
				SENSOR_LOG_INFO("read device id success\n");
				return 0;
			}
		}
	}
	SENSOR_LOG_ERROR("read device id failed\n");
	return res;
}

static const struct i2c_device_id pa224_id[] = {
	{ PA224_DRV_NAME, 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, pa224_id);

static struct of_device_id pa224_match_table[] = {
	{ .compatible = "txc,pa224",},
	{ },
};

static const struct dev_pm_ops pa224_pm_ops = {
	.suspend	= pa224_suspend,
	.resume 	= pa224_resume,
};

static struct i2c_driver pa224_driver = {
	.driver = {
		.name	= PA224_DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = pa224_match_table,
		.pm = &pa224_pm_ops,
	},
	.probe	= pa224_probe,
	.remove	= pa224_remove,
	.id_table = pa224_id,
};
/*
 * I2C init/probing/exit functions
 */

static int pa224_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct pa224_data *data = NULL;
	struct pa224_platform_data *pdata=client->dev.platform_data;
	int err = 0;
	SENSOR_LOG_DEBUG("probe start\n");
	data = kzalloc(sizeof(struct pa224_data), GFP_KERNEL);
	if (!data) {
		SENSOR_LOG_ERROR("kzalloc pa224_data failed\n");
		err = -ENOMEM;
		goto exit;	
	}

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_I2C_BLOCK)) {
		SENSOR_LOG_ERROR("i2c_check_functionality error");
		goto exit;
	}

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
				sizeof(struct pa224_platform_data),
				GFP_KERNEL);
		if (!pdata) {
			SENSOR_LOG_ERROR("Failed to allocate memory\n");
			err = -ENOMEM;
			goto exit_platform_failed;
		}

		client->dev.platform_data = pdata;
		
		/* Parse device tree */
		err = sensor_parse_dt(&client->dev, pdata, data);
		if (err) {
			SENSOR_LOG_ERROR("sensor_parse_dt() err\n");
			goto exit_platform_failed;
		}
	} else {
		pdata = client->dev.platform_data;
		if (!pdata) {
			SENSOR_LOG_ERROR("No platform data\n"); 
			err = -ENODEV;
			goto exit_platform_failed;
		}
	}
	/*init pin state*/
	err = pa224_pinctrl_init(data, &client->dev);
	if (err) {
		SENSOR_LOG_ERROR("init pinctrl state failed\n");
		data->pinctrl = NULL;
		data->pin_default = NULL;
		data->pin_sleep = NULL;
	}

	mutex_init(&data->i2c_lock);
	mutex_init(&data->dev_lock);
	pdev_data = data;

	data->client = client;
	i2c_set_clientdata(client, data);

	data->platform_data  = pdata;
	pa224_i2c_client = client;

	/* h/w initialization */
	if (pdata->init)
		err = pdata->init();
	data->flag_prox_debug = false;

	/*read device id*/
	err = pa224_read_device_id(data);
	if (err < 0) {
		SENSOR_LOG_ERROR("pa224_read_device_id failed\n");
		goto exit_no_dev_error;
	}
	proximity_class = class_create(THIS_MODULE, "proximity");
	data->proximity_dev = device_create(proximity_class, NULL, pa224_proximity_dev_t, &pa224_driver ,"proximity");
	if (IS_ERR(data->proximity_dev)) {
		err = PTR_ERR(data->proximity_dev);
		SENSOR_LOG_ERROR("device_create proximity failed\n");
		goto create_proximity_dev_failed;
	}

	dev_set_drvdata(data->proximity_dev, data);

	err = create_sysfs_interfaces(data->proximity_dev);
	if (err < 0) {
		SENSOR_LOG_ERROR("create sysfs interfaces failed\n");
		goto create_sysfs_interface_error;
	}

	err = misc_register(&pa224_ps_device);
	if (err) {
		SENSOR_LOG_INFO("miscdev regist error\n");
	}
#ifdef SENSORS_CLASS_DEV
	/* Register to sensors class */
	data->ps_cdev = sensors_proximity_cdev;
	data->ps_cdev.sensors_enable = pa224_ps_set_enable;
	data->ps_cdev.sensors_poll_delay = NULL,
	err = sensors_classdev_register(&client->dev, &data->ps_cdev);
	if (err) {
		SENSOR_LOG_ERROR("Unable to register to sensors class: %d\n",err);
		goto create_sysfs_interface_error;
	}
#endif
	/* allocate proximity input_device */
	data->ps_input_dev = input_allocate_device();
	if (IS_ERR_OR_NULL(data->ps_input_dev)) {
		err = -ENOMEM;
		SENSOR_LOG_INFO("could not allocate input device\n");
		goto exit_unregister_sensorclass;
	}

	input_set_drvdata(data->ps_input_dev, data);
	data->ps_input_dev->name = INPUT_NAME_PS;
	data->ps_input_dev->id.bustype = BUS_I2C;
	set_bit(EV_REL, data->ps_input_dev->evbit);
	set_bit(REL_RZ,  data->ps_input_dev->relbit);
	set_bit(REL_MISC,  data->ps_input_dev->relbit);

	SENSOR_LOG_INFO("registering proximity input device\n");
	err = input_register_device(data->ps_input_dev);			
	if (err < 0) {
		SENSOR_LOG_INFO("could not register input device\n");
		err = -ENOMEM;
		goto exit_unregister_sensorclass;
	}
	wake_lock_init(&data->pa224_wake_lock, WAKE_LOCK_SUSPEND ,"pa224_wake_lock");
	/*Device Initialize*/
	err = pa224_init_client(client);
	if (err < 0) {
		SENSOR_LOG_ERROR("init pa224 failed when probe\n");
		goto input_dev_exit;
	}

	if (PS_POLLING)
		INIT_DELAYED_WORK(&data->ps_dwork, pa224_work_func_proximity);
	else
		INIT_WORK(&data->irq_dwork, pa224_work_func_irq);

	data->irq_work_queue = create_singlethread_workqueue("pa224_work_queue");
	if (IS_ERR_OR_NULL(data->irq_work_queue)){
	    err = -ENOMEM;
	    SENSOR_LOG_ERROR( "cannot create work taos_work_queue, err = %d",err);
	    goto input_dev_exit;
	}

	irq_set_irq_wake(client->irq, 1);
	/*Interrupt Regist*/
	if (!PS_POLLING ) {	
		err = request_irq(data->irq, pa224_irq,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				PA224_DRV_NAME, (void *)client);

		if (err) {
			SENSOR_LOG_INFO("Could not get IRQ\n");
		}
	}
	pa224_irq_enable(data, false, true);
	if (!data->vdd_always_on) {
		if (pdata && pdata->power_on) {
			pdata->power_on(false);
		}
	}
	SENSOR_LOG_INFO("probe ok.\n");
	return 0;
input_dev_exit:
	if (!IS_ERR_OR_NULL(data->ps_input_dev)) {
		input_unregister_device(data->ps_input_dev);
		input_free_device(data->ps_input_dev);
	}
exit_unregister_sensorclass:
	wake_lock_destroy(&data->pa224_wake_lock);
#ifdef SENSORS_CLASS_DEV
	sensors_classdev_unregister(&data->ps_cdev);
#endif
create_sysfs_interface_error:
	remove_sysfs_interfaces(data->proximity_dev);
create_proximity_dev_failed:
	data->proximity_dev = NULL;
	device_destroy(proximity_class, pa224_proximity_dev_t);
	class_destroy(proximity_class);
exit_no_dev_error:
	if (pdata->power_on)
		pdata->power_on(false);
	if(pdata->exit)
		pdata->exit();
exit_platform_failed:
	kfree(data);
exit:
	return err;
}

static int pa224_remove(struct i2c_client *client)
{
	struct pa224_data *data = i2c_get_clientdata(client);
	struct pa224_platform_data *pdata = data->platform_data;
	struct device *dev = data->proximity_dev;

	if (data->ps_enable) 
		pa224_enable_ps(client, 0);

	input_unregister_device(data->ps_input_dev);
 	input_free_device(data->ps_input_dev);
	remove_sysfs_interfaces(dev);
	wake_lock_destroy(&data->pa224_wake_lock);
	misc_deregister(&pa224_ps_device);

	if (!PS_POLLING)
		free_irq(data->irq, client);
	
	if (pdata->power_on)
		pdata->power_on(false);

	if (pdata->exit)
		pdata->exit();

	mutex_destroy(&data->i2c_lock);
	mutex_destroy(&data->dev_lock);
	
	kfree(data);
	
	return 0;
}


static int __init pa224_init(void)
{
	return i2c_add_driver(&pa224_driver);
}

static void __exit pa224_exit(void)
{
	i2c_del_driver(&pa224_driver);
}

MODULE_AUTHOR("Sensor Team, TXC");
MODULE_DESCRIPTION("PA224 proximity sensor driver.");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRIVER_VERSION);

module_init(pa224_init);
module_exit(pa224_exit);


