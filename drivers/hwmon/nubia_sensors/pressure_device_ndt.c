/*
 *
 * nubia pressure sensor driver.
 *
 * Copyright (c) 2015  nubia tech Ltd.
 * Copyright (c) The Linux Foundation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "pressure_device_ndt.h"

#define LOG_TAG "PRESSURE_DEVICE_NDT"
#define DEBUG_ON //DEBUG SWITCH

#define SENSOR_LOG_FILE__ strrchr(__FILE__, '/') ? (strrchr(__FILE__, '/')+1) : __FILE__

#ifdef  CONFIG_FEATURE_NUBIA_SENSORS_LOG_ON
#define SENSOR_LOG_ERROR(fmt, args...) printk(KERN_ERR   "[%s] [%s: %d] "  fmt,\
                                              LOG_TAG,__FUNCTION__, __LINE__, ##args)
#define SENSOR_LOG_INFO(fmt, args...)  printk(KERN_INFO  "[%s] [%s: %d] "  fmt,\
                                              LOG_TAG,__FUNCTION__, __LINE__, ##args)
    #ifdef  DEBUG_ON
#define SENSOR_LOG_DEBUG(fmt, args...) printk(KERN_DEBUG "[%s] [%s: %d] "  fmt,\
                                              LOG_TAG,__FUNCTION__, __LINE__, ##args)
    #else
#define SENSOR_LOG_DEBUG(fmt, args...)
    #endif

#else
#define SENSOR_LOG_ERROR(fmt, args...)
#define SENSOR_LOG_INFO(fmt, args...)
#define SENSOR_LOG_DEBUG(fmt, args...)
#endif

#define PAGE_SZIE 						128
#define FILE_PATH						 "/persist/sensors/fw_read.bin"
#define SMART_KEY_CFG                    "/persist/sensors/smart_key_cfg"
#define PRESSURE_MAX_RW_TRIES 			3
#define I2C_BLOCK_LENGTH      			6

#define IC_RESET_MCU           	 		0x01
#define IC_MANUFACTURER_ID_ADDR 		0x03
#define IC_MODULE_ID_ADDR 				0x04
#define IC_FW_VERSION_ADDR				0x05
#define IC_HOST_STATUS					0x50

#define IC_WAKE_UP_CMD					0x06
#define IC_SLEEP_CMD					0x07
#define IC_DEBUG_MODE					0x60
#define IC_DATA_READY					0x61
#define IC_DEBUG_DATA1					0x62
#define IC_DEBUG_DATA2					0x63
#define IC_DEBUG_DATA3					0x64
#define IC_CHANNEL_KEY_UP				0x80
#define IC_CHANNEL_KEY_DOWN				0x81
#define IC_KEY_SENSITIVITY				0xd0

#define FW_MANUFACTURER_ID_OFFSET 		0x04
#define FW_MODULE_ID_OFFSET 			0x06
#define FW_VERSION_OFFSET 				0x08
#define FW_DATA_LENGTH					0x0c
#define FW_DATA_OFFSET					256

#define MAX_UINT16					65535
#define  WRITE_TIMEOUT              25

static dev_t const   pressure_device_dev_t   = MKDEV(MISC_MAJOR, 253);
static struct class  *pressure_device_class;
char start_cmd[]={0x00, 0xAA, 0x55, 0xA5, 0x5A};
char skip_cmd[]={0x00, 0x7E, 0xE7, 0xEE, 0x77};

static int pressure_sensor_reboot_nb_handler(struct notifier_block *cb, unsigned long code, void *unused)
{
		int rc;
		struct pressure_sensor_data *pdata = container_of(cb,
		struct pressure_sensor_data, pressure_sensor_reboot_nb);
		switch (code) {
		    case SYS_DOWN:
		    case SYS_HALT:
		    case SYS_POWER_OFF:
		        rc = pressure_host_status_report(pdata, POWEROFF);
		        if (rc < 0) {
		            SENSOR_LOG_ERROR("goto power off mode failed\n");
		        }
		        SENSOR_LOG_INFO("recv power off notifier!");
	        default:
		        break;
		}
		return NOTIFY_DONE;
}

static int _pressure_read_reg(struct i2c_client *client, u16 addr, u8 *val, u16 len)
{
		struct i2c_msg msg[2];
		u8 buf;
		int retry = 0;
		int status = 0;

		buf = addr & 0xff;

		/* Write register */
		msg[0].addr = client->addr;
		msg[0].flags = 0;
		msg[0].len = 1;
		msg[0].buf = &buf;

		/* Read data */
		msg[1].addr = client->addr;
		msg[1].flags = I2C_M_RD;
		msg[1].len = len;
		msg[1].buf = val;

		do {
			status = i2c_transfer(client->adapter, msg, 2);
			if (status == 2) {
				SENSOR_LOG_DEBUG( "read %x@%d --> %x (%d)\n", addr, len, msg[0].addr, status);
				return status;
			}
			msleep(1);
		} while (++retry < PRESSURE_MAX_RW_TRIES);

		SENSOR_LOG_ERROR( "i2c read failed\n");
		return -ETIMEDOUT;
}
static int pressure_read_reg(struct pressure_sensor_data *pdata, u16 reg ,void *val,u16 length)
{
		struct i2c_client *client = pdata->client;
		return  _pressure_read_reg(client, reg, val, length);
}

static int pressure_write_reg(struct pressure_sensor_data *pdata ,u16 addr, void *buf, u16 length)
{
		struct i2c_client *client = pdata->client;
		return _pressure_write_reg(client, addr, buf, length);
}

static int _pressure_write_reg(struct i2c_client *client, u16 addr, u8 *value, u16 len)
{
		u8 buf[I2C_BLOCK_LENGTH];
		u8 reg_addr;
		int i = 0;
		struct i2c_msg msg;
		int status = 0;
		int retry = 0;

		if (len > I2C_BLOCK_LENGTH)
				return -EINVAL;

		reg_addr = addr & 0xff;

		/*write data*/
		msg.addr = client->addr;
		msg.flags = 0;
		msg.buf = buf;
		msg.buf[i++] = reg_addr;
		memcpy(&msg.buf[i], value, len);
		msg.len = i + len;

		do {
			status = i2c_transfer(client->adapter, &msg, 1);
			if (status == 1) {
				SENSOR_LOG_DEBUG( "write %x@%d --> %x (%d)\n", addr, len, msg.addr, status);
				return status;
			}
			msleep(1);
		} while (++retry < PRESSURE_MAX_RW_TRIES);

		SENSOR_LOG_ERROR("i2c write failed\n");

		return -ETIMEDOUT;
}

static int pressure_host_status_report(struct pressure_sensor_data *pdata, u8 status)
{
		int err = 0;
		err = pressure_vlps_write_reg(pdata, IC_HOST_STATUS, &status, 1);
		if (err < 0) {
			SENSOR_LOG_INFO("ctrl slave status failed.\n");
		}
		return err;
}

static int pressure_sensor_suspend(struct device *dev)
{
		int err = 0;
		struct pressure_sensor_data *pdata = dev_get_drvdata(dev);
		pdata->suspended = true;

		err = pressure_host_status_report(pdata, STANDBY);
		if (err < 0) {
		    SENSOR_LOG_ERROR("make slave ic go to sleep state failed\n");
		    return 0;
		}
		if (pdata->support_smart_key) {
			if(pdata->support_stay_awake) {
			    err = pressure_host_status_report(pdata, HALFWAKE);
			    if (err < 0) {
			        SENSOR_LOG_ERROR("make slave ic go to halfwake state failed\n");
			        return err;
			    }
			    SENSOR_LOG_DEBUG("pressure goto half-wake mode\n");
			}
			err = pressure_device_pinctrl_configure(pdata, false);
			if (err <0) {
			    SENSOR_LOG_ERROR("cfg pinctrl to sleep failed\n");
			    return err;
			}
		}
		SENSOR_LOG_DEBUG("suspend\n");
		return 0;
}

static int pressure_sensor_resume(struct device *dev)
{
		int err = 0;
		struct pressure_sensor_data *pdata = dev_get_drvdata(dev);
		pdata->suspended = false;
		err = pressure_host_status_report(pdata, NORMAL);
		if (err < 0) {
			SENSOR_LOG_ERROR("wake up slave ic failed\n");
			return 0;
		}
		if (pdata->support_smart_key) {
			err = pressure_device_pinctrl_configure(pdata, true);
			if (err <0) {
				SENSOR_LOG_ERROR("cfg pinctrl to default failed\n");
				return err;
			}
		}
		SENSOR_LOG_DEBUG("resume\n");
		return 0;
}


static void pressure_debug_work_func(struct work_struct *work)
{
		int err = 0;
		int len = 0;
		short int buf[3];
		struct pressure_sensor_data *pdata;

		pdata = container_of((struct delayed_work *)work, struct pressure_sensor_data, debug_work);

		err = pressure_vlps_read_reg(pdata, IC_DATA_READY, &len, 1);
		if (err < 0) {
			SENSOR_LOG_ERROR("read data_ready reg failed\n");
			return;
		}

		if (0 == len) {
			SENSOR_LOG_INFO("no data ready\n");
		}

		if (len > 0) {

			err = pressure_vlps_read_reg(pdata, IC_DEBUG_DATA1, &buf[0], err);
			if (err < 0) {
				err = pressure_read_reg(pdata, IC_DEBUG_DATA1, &buf[0], err);
				if (err < 0) {
					SENSOR_LOG_ERROR("read debug data1 failed\n");
					return;
				}
			}

			err = pressure_vlps_read_reg(pdata, IC_DEBUG_DATA2, &buf[1], err);
			if (err < 0) {
				err = pressure_read_reg(pdata, IC_DEBUG_DATA2, &buf[1], err);
				if (err < 0) {
					SENSOR_LOG_ERROR("read debug data2 failed\n");
					return;
				}
			}

			err = pressure_vlps_read_reg(pdata, IC_DEBUG_DATA3, &buf[2], err);
			if (err < 0) {
				err = pressure_read_reg(pdata, IC_DEBUG_DATA3, &buf[2], err);
				if (err < 0) {
					SENSOR_LOG_ERROR("read debug data3 failed\n");
					return;
				}
			}

			SENSOR_LOG_INFO("key1: %d  key2: %d key3: %d\n", buf [0], buf[1], buf[2]);
		}

		schedule_delayed_work(&pdata->debug_work, msecs_to_jiffies(500));
}

static int pressure_eeprom_write(struct pressure_sensor_data *pdata, const char *buf, unsigned offset, int count, u8 *writebuf)
{
		int i = 0;
		struct i2c_msg msg;
		int status;
		unsigned long timeout, write_time;
		unsigned next_page;
		struct i2c_client *client = pdata->client;
		offset &= 0xffff;

		/* write_max is at most a page */
		if (count > pdata->page_size)
				count = pdata->page_size;

		/* Never roll over backwards, to the start of this page */
		next_page = roundup(offset + 1, pdata->page_size);
		if (offset + count > next_page)
				count = next_page - offset;

		msg.addr = client->addr;
		msg.flags = 0;

		/* msg.buf is u8 and casts will mask the values */
		msg.buf = writebuf;
		msg.buf[i++] = offset >> 8;
		msg.buf[i++] = offset;
		memcpy(&msg.buf[i], buf, count);
		msg.len = i + count;//PAGE_SIZE;//
		/*
		* Writes fail if the previous one didn't complete yet. We may
		* loop a few times until this one succeeds, waiting at least
		* long enough for one entire page write to work.
		* */
		timeout = jiffies + msecs_to_jiffies(WRITE_TIMEOUT);
		mutex_lock(&pdata->lock);
		do {
				write_time = jiffies;
				status = i2c_transfer(client->adapter, &msg, 1);
				if (status == 1)
						status = count;
						//SENSOR_LOG_DEBUG( "write %d@%d --> %d (%ld)\n",count, offset, status, jiffies);

				if (status == count) {
						mutex_unlock(&pdata->lock);
						return count;
				}

				/* REVISIT: at HZ=100, this is sloooow */
				msleep(1);
		} while (time_before(write_time, timeout));
		mutex_unlock(&pdata->lock);
		return -ETIMEDOUT;
}

static int pressure_fw_write(struct pressure_sensor_data *pdata, const char *buf, int count)
{
		int retval = 0;
		int off = 0;
		u8 *write_buf;

		if (unlikely(!count))
			return count;

		if (count%128 != 0) {
			SENSOR_LOG_ERROR("fw length is not 128*\n");
			return -1;
		}

		write_buf = kzalloc(pdata->page_size + 2, GFP_KERNEL);
		if (!write_buf) {
			SENSOR_LOG_ERROR("allocate write_buf failed");
			return -ENOMEM;
		}

		while (count) {
			int	status;
			status = pressure_eeprom_write(pdata, buf, off, count, write_buf);

			if (status <= 0) {
				if (retval == 0)
					retval = status;
				break;
			}
			buf += status;
			off += status;
			count -= status;
			retval += status;
			msleep(10);
		}

		kfree(write_buf);
		return retval;
}

static int pressure_fw_check(struct pressure_sensor_data *pdata, const char *fw_buf, int fw_size, bool save_flag)
{
		int retval = 0;
		int off = 0;
		u8 *read_buf;
		u8 *copy_buf;
		int i = 0;

		if(fw_buf == NULL) {
			SENSOR_LOG_ERROR("fw_buf is NULL!\n");
			return -1;
		}

		read_buf = (u8 *) kzalloc(fw_size, GFP_KERNEL);
		if (!read_buf) {
			SENSOR_LOG_ERROR("allocate readbuf failed");
			return -1;
		}
		copy_buf = read_buf;
		if (unlikely(!fw_size)) {
			goto fail;
		}

		SENSOR_LOG_DEBUG("read size= %d!\n",fw_size);
		while (fw_size) {
			int	status = 0;
			status = pressure_eeprom_read(pdata, read_buf, off, fw_size);

			if (status <= 0) {
					if (retval == 0)
						retval = status;
					break;
			}
			read_buf += status;
			off += status;
			fw_size -= status;
			retval += status;
			msleep(10);
			if (retval < 0) {
				SENSOR_LOG_ERROR("eeprom read error\n");
				goto fail;
			}
		}

		SENSOR_LOG_DEBUG("start checking...\n");
		for(i = 0; i < fw_size; i++) {
			if(fw_buf[i] != copy_buf[i]) {
				SENSOR_LOG_ERROR("check failed!\n");
				goto fail;
			}
		}

		SENSOR_LOG_INFO("fw checking ok...\n");
		if (save_flag){
			SENSOR_LOG_DEBUG("start write...\n");
			retval = pressure_file_write(FILE_PATH, read_buf, fw_size);
			if(retval < 0) {
				SENSOR_LOG_ERROR("save read data failed\n");
				goto fail;
			}
		}
		return 0;
fail:
		kfree(read_buf);
		return -1;
}

static int pressure_eeprom_read(struct pressure_sensor_data *pdata, u8  *read_buf, unsigned offset, int count)
{
		struct i2c_msg msg[2];
		u8 reg16[2];
		struct i2c_client *client = pdata->client;
		unsigned long timeout, read_time;
		int status;

		memset(msg, 0, sizeof(msg));
		/*
		* Slave address and byte offset derive from the offset.
		*/

		offset &= 0xffff;

		/* write_max is at most a page */
		if (count > pdata->page_size)
				count = pdata->page_size;

		reg16[0] = (offset >> 8) & 0xff;
		reg16[1] = offset & 0xff;

		msg[0].addr = client->addr;
		msg[0].flags = 0;
		msg[0].buf = reg16;
		msg[0].len = 2;

		msg[1].addr = client->addr;
		msg[1].flags = I2C_M_RD;
		msg[1].buf = read_buf;
		msg[1].len = count;

		/*
		* Reads fail if the previous write didn't complete yet. We may
		* loop a few times until this one succeeds, waiting at least
		* long enough for one entire page write to work.
		*/
		timeout = jiffies + msecs_to_jiffies(WRITE_TIMEOUT);
		do {
			mutex_lock(&pdata->lock);
			read_time = jiffies;
			status = i2c_transfer(client->adapter, msg, 2);
			if (status == 2)
					status = count;
			//SENSOR_LOG_DEBUG("read %d@%d --> %d (%ld)\n",count, offset, status, jiffies);
			if (status == count) {
				mutex_unlock(&pdata->lock);
				return count;
			}

			/* REVISIT: at HZ=100, this is sloooow */
			msleep(1);
		} while (time_before(read_time, timeout));
		mutex_unlock(&pdata->lock);
		return -ETIMEDOUT;
}
static int pressure_load_smartkey_param(struct pressure_sensor_data *data)
{
		int err;
		err = pressure_file_read(SMART_KEY_CFG , (char *)&(data->support_stay_awake),
                        sizeof(data->support_stay_awake));
		if (err < 0) {
		    SENSOR_LOG_ERROR("read smart key conf file error\n");
		    return err;
		}
		return 0;
}
static int pressure_file_read(char *file_path, char *read_buf ,int count)
{
		struct file *file_p;
		mm_segment_t old_fs;
		int vfs_retval = -EINVAL;

		SENSOR_LOG_DEBUG("read infomation : size =%d\n", count);
		if (NULL == file_path) {
		    SENSOR_LOG_ERROR("file_path is NULL\n");
		    return -EINVAL;
		}

		file_p = filp_open(file_path, O_RDWR, 0666);
		if (IS_ERR(file_p)) {
		    SENSOR_LOG_INFO("file does not exist\n");
		    goto error;
		}

		old_fs = get_fs();
		set_fs(KERNEL_DS);

		file_p->f_pos = 0;
		vfs_retval = vfs_read(file_p, (char*)read_buf, count, &file_p->f_pos);
		if (vfs_retval < 0) {
		    SENSOR_LOG_ERROR("[read file <%s>failed]\n",file_path);
		    goto file_close;
		}

		SENSOR_LOG_INFO("read ok\n");

file_close:
		set_fs(old_fs);
		filp_close(file_p, NULL);
error:
		return vfs_retval;
}
static int pressure_file_write(char *file_path, const char *write_buf ,int count)
{
		struct file *file_p;
		mm_segment_t old_fs;
		int vfs_write_retval=0;

		SENSOR_LOG_DEBUG("write infomation : size =%d\n",count);
		if (NULL == file_path) {
			SENSOR_LOG_ERROR("file_path is NULL\n");
		}

		file_p = filp_open(file_path, O_CREAT|O_RDWR , 0666);
		if (IS_ERR(file_p)) {
			SENSOR_LOG_ERROR("[open file <%s>failed]\n",file_path);
			goto error;
		}
		old_fs = get_fs();
		set_fs(KERNEL_DS);

		vfs_write_retval = vfs_write(file_p, (char*)write_buf, count, &file_p->f_pos);
		if (vfs_write_retval < 0) {
			SENSOR_LOG_ERROR("[write file <%s>failed]\n",file_path);
			goto file_close;
		}

		SENSOR_LOG_INFO("write ok\n");

file_close:
		set_fs(old_fs);
		filp_close(file_p, NULL);
error:
		return vfs_write_retval;
}

static int pressure_read_ic_id(struct pressure_sensor_data *pdata)
{
		u16 ic_id = 65534;
		int ret = -1;

		ret = pressure_vlps_read_reg(pdata, IC_FW_VERSION_ADDR, &ic_id, 2);
		if (ret < 0) {
			SENSOR_LOG_ERROR("read ver reg failed\n");
		}

		return ic_id;
}

static bool perssure_need_upgrade_flag(const struct firmware *fw,struct pressure_sensor_data *pdata)
{
		u16 ic_id = 65534;
		u16 fw_id = 0;

		ic_id = pressure_read_ic_id(pdata);

		fw_id = (u16) *(fw->data + FW_VERSION_OFFSET );
		fw_id = (fw_id << 8) + (*(fw->data + FW_VERSION_OFFSET + 1));

		SENSOR_LOG_INFO("fw_id=%d:ic_id=%d\n", fw_id, ic_id);

		if (8 == ic_id && 9 == fw_id)
			return true;

		SENSOR_LOG_INFO("manufactor_id:%x\n", pdata->fw_info.manufactor_id);
		if (pdata->fw_info.manufactor_id != 0x1001)
			return true;

		/*not allow upgrading from 9 to 12, for the data partitions have changed*/
		if (ic_id <=9 && fw_id >= 12)
			return false;

		return (ic_id >= 12)? ((fw_id > ic_id) ? true : false): false;
}

static int pressure_slave_ic_reset(struct pressure_sensor_data *pdata)
{
		int ret = 0;

		mutex_lock(&pdata->lock);
		ret = pressure_device_reset_on(pdata, 1);
		if (ret < 0) {
			ret = pressure_device_reset_on(pdata, 1);
			if (ret < 0) {
				mutex_unlock(&pdata->lock);
				SENSOR_LOG_ERROR("set reset-pin high failed!\n");
				return ret;
			}
		}

		msleep(1);

		ret = pressure_device_reset_on(pdata, 0);
		if (ret < 0) {
			ret = pressure_device_reset_on(pdata, 0);
			if(ret < 0){
					SENSOR_LOG_ERROR("set reset-pin low failed!\n");
					mutex_unlock(&pdata->lock);
					return ret;
			}
		}
		mutex_unlock(&pdata->lock);

		return ret;
}

static int pressure_slave_vlps_ops(struct pressure_sensor_data *pdata)
{
		int ret = 0;
		ret = pressure_device_vlps_on(pdata, 1);
		if (ret < 0) {
			ret = pressure_device_vlps_on(pdata, 1);
			if(ret < 0){
					SENSOR_LOG_ERROR("set vlps-pin failed!\n");
					return ret;
			}
		}
		mdelay(1);
		ret = pressure_device_vlps_on(pdata, 0);
		if (ret < 0) {
			ret = pressure_device_vlps_on(pdata, 0);
			if (ret < 0) {
				SENSOR_LOG_ERROR("set vlps-pin failed!\n");
				return ret;
			}
		}
		mdelay(1);
		ret = pressure_device_vlps_on(pdata, 1);
		if (ret < 0) {
			ret = pressure_device_vlps_on(pdata, 1);
			if (ret < 0) {
				SENSOR_LOG_ERROR("set vlps-pin failed!\n");
				return ret;
			}
		}
		mdelay(1);
		ret = pressure_device_vlps_on(pdata, 0);
		if (ret < 0) {
			ret = pressure_device_vlps_on(pdata, 0);
			if (ret < 0) {
				SENSOR_LOG_ERROR("set vlps-pin failed!\n");
				return ret;
			}
		}
		mdelay(1);
		ret = pressure_device_vlps_on(pdata, 1);
		if (ret < 0) {
			ret = pressure_device_vlps_on(pdata, 1);
			if (ret < 0) {
				SENSOR_LOG_ERROR("set vlps-pin failed!\n");
				return ret;
			}
		}
		mdelay(1);
		SENSOR_LOG_DEBUG("reset vlps.\n");
		return ret;
}

static int prepare_fw_upgrade_work(struct pressure_sensor_data *pdata, bool on)
{

		int ret = -1;

		ret = pressure_slave_ic_reset(pdata);
		if (ret < 0){
			SENSOR_LOG_ERROR("reset slave ic failed.\n");
			return ret;
		}

		/*wait for 5ms to start upgrade*/
		msleep(5);

		if (on) {
			SENSOR_LOG_DEBUG("try to send start upgrade cmd!\n");
			ret = pressure_write_reg(pdata, 0, start_cmd, sizeof(start_cmd));
			if (ret < 0) {
					SENSOR_LOG_ERROR("try to send start upgrade cmd failed\n");
					return ret;
			}

			msleep(2000);

		} else {

			SENSOR_LOG_DEBUG("try to send skip upgrade cmd!\n");
			ret = pressure_write_reg(pdata, 0, skip_cmd, sizeof(skip_cmd));
			if(ret < 0)	{
				SENSOR_LOG_ERROR("try to send skip upgrade cmd failed\n");
				return ret;
			}

		}
		SENSOR_LOG_DEBUG("prepare_fw_upgrade_work success!\n");
		return ret;

}
static int discard_fw_head_info(const struct firmware *fw, unsigned int *fw_start, unsigned int *fw_data_length)
{
		int err = -1;
		int head_data[2];

		head_data[0] = (int) *(fw->data);
		head_data[1] = (int) *(fw->data + 1);

		if (head_data[0] == 0xA5 && head_data[1] == 0x5A) {
			*fw_start = FW_DATA_OFFSET;
			*fw_data_length = fw->size - FW_DATA_OFFSET;
			err = 0;

		} else {
			*fw_start = 0;
			*fw_data_length = fw->size;
		}
		SENSOR_LOG_DEBUG("head_data[0]=%x,head_data[1]=%x\n", head_data[0], head_data[1]);
		SENSOR_LOG_DEBUG("fw_start =%d,fw_data_length =%d\n", *fw_start, *fw_data_length);
		return err;
}

static int pressure_reset_device_work(struct pressure_sensor_data *pdata)
{
		int rc = -1;
		u8 slave_status = -1;

		rc =  pressure_slave_ic_reset(pdata);
		if (rc < 0) {
			SENSOR_LOG_ERROR("reset slave ic fail\n");
			return rc;
		}

		msleep(5);
		SENSOR_LOG_DEBUG("try to send skip upgrade cmd!\n");
		rc = pressure_write_reg(pdata, 0, skip_cmd, sizeof(skip_cmd));
		if(rc < 0) {
			SENSOR_LOG_ERROR("try to send skip upgrade cmd failed\n");
			return rc;
		}

		msleep(400);
		rc = pressure_host_status_report(pdata, NORMAL);
		if (rc < 0) {
			SENSOR_LOG_ERROR("wake up slave ic failed\n");
			return rc;
		}

		rc = pressure_read_reg(pdata, IC_HOST_STATUS, &(slave_status), 1);
		if (rc < 0) {
			rc = pressure_read_reg(pdata, IC_HOST_STATUS, &(slave_status), 1);
			if (rc < 0) {
				SENSOR_LOG_ERROR("wake up slave ic failed\n");
				return rc;
			}
		}

		if (slave_status == NORMAL) {
			SENSOR_LOG_ERROR("wake up slave to normal state\n");
			rc = 0;
		} else {
			msleep(1);
			rc = pressure_host_status_report(pdata, NORMAL);
		}
		return rc;
}

static int pressure_fw_upgrade(struct device *dev, bool force)
{
		int rc = -1;
		bool fw_upgrade = false;
		unsigned int fw_start = 0;
		unsigned int fw_data_length = 0;
		bool check_flag = true;
		int retry_times = 2;

		struct pressure_sensor_data *pdata = dev_get_drvdata(dev);
		const struct firmware *fw = NULL;

		if (pdata->suspended) {
				SENSOR_LOG_ERROR("Device is in suspend state: Exit FW upgrade\n");
				return -EBUSY;
		}

		rc = request_firmware(&fw, pdata->fw_name, dev);
		if (rc < 0) {
				SENSOR_LOG_ERROR("Request firmware failed - %s (%d)\n",pdata->fw_name, rc);
				return rc;
		}
		SENSOR_LOG_DEBUG("request firmware success...\n");
		SENSOR_LOG_DEBUG("Firmware Info fw size:%ld\n",fw->size);

		pdata->fw_size = fw->size;

		rc = discard_fw_head_info(fw, &fw_start, &fw_data_length);
		if (rc < 0) {
			SENSOR_LOG_ERROR("do not have header info\n");
		}

		fw_upgrade = perssure_need_upgrade_flag(fw, pdata);

		if (force) {
			fw_upgrade = true;
		}

		if (!fw_upgrade) {
			SENSOR_LOG_INFO("Exiting fw upgrade...\n");
			release_firmware(fw);
			return -EFAULT;
		}

		rc = prepare_fw_upgrade_work(pdata, fw_upgrade);
		if (rc < 0) {
			SENSOR_LOG_ERROR("prepare_fw_upgrade_work failed!\n");
			return rc;
		}

		SENSOR_LOG_DEBUG("start to download firmware....\n");
		while(retry_times) {
			rc = pressure_fw_write(pdata, fw->data + fw_start, fw_data_length);
			if (rc < 0) {
				SENSOR_LOG_ERROR("update failed (%d). try later...\n", rc);
				retry_times--;
				continue;
			}
			if (check_flag) {
				rc = pressure_fw_check(pdata, fw->data +fw_start, fw_data_length, false);
				if (rc >= 0) {
					SENSOR_LOG_ERROR("pressure fw check success!\n");
					break;
				}
				retry_times--;
			} else {
				break;
			}
		};

		msleep(1);

		if ((rc = pressure_reset_device_work(pdata)) < 0)
			goto release_fw;

		return 0;

release_fw:
		release_firmware(fw);
		return rc;
}
static int pressure_vlps_read_reg(struct pressure_sensor_data *pdata ,u16 addr, void *buf, u16 length)
{
		int err = 0;
		int i = 0;

		do {
			mutex_lock(&pdata->lock);
			err = pressure_slave_vlps_ops(pdata);
			if (err < 0) {
				mutex_unlock(&pdata->lock);
				return -1;
			}
			err = pressure_read_reg(pdata, addr, buf, length);
			if (err >= 0) {
				mutex_unlock(&pdata->lock);
				break;
			} else {
			    mdelay(2);
			    err = pressure_read_reg(pdata, addr, buf, length);
			    if (err >= 0) {
			        mutex_unlock(&pdata->lock);
			        break;
			    }
			}
			mutex_unlock(&pdata->lock);
		} while (++i < 10);

		return err;
}
static int pressure_vlps_write_reg(struct pressure_sensor_data *pdata ,u16 addr, void *buf, u16 length)
{
		int err = 0;
		int i = 0;

		do {
			mutex_lock(&pdata->lock);
			err = pressure_slave_vlps_ops(pdata);
			if (err < 0) {
			    mutex_unlock(&pdata->lock);
			    return -1;
			}
			err = pressure_write_reg(pdata, addr, buf, length);
			if (err >= 0) {
				mutex_unlock(&pdata->lock);
				break;
			} else {
			    mdelay(2);
			    err = pressure_write_reg(pdata, addr, buf, length);
			    if (err >= 0) {
			        mutex_unlock(&pdata->lock);
			        break;
			    }
			}
			mutex_unlock(&pdata->lock);
		} while (++i < 10);

		return err;
}
static ssize_t pressure_update_fw_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
		struct pressure_sensor_data *data = dev_get_drvdata(dev);
		return snprintf(buf, 2, "%d\n", data->loading_fw);
}

static ssize_t pressure_update_fw_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
		struct pressure_sensor_data *data = dev_get_drvdata(dev);
		unsigned long val;
		int rc;

		if (size > 2)
			return -EINVAL;

		rc = kstrtoul(buf, 10, &val);
		if (rc != 0)
			return rc;

		if (!data->loading_fw  && val) {
			data->loading_fw = true;
			pressure_fw_upgrade(dev, false);
			data->loading_fw = false;
		}
		if (data->support_smart_key) {
		    pressure_device_irq_enable(&(data->irq), true, false);
		    if (pressure_load_smartkey_param(data) < 0) {
		        SENSOR_LOG_ERROR("load smart para failed\n");
		        return size;
		    }
		}
		return size;
}

static ssize_t pressure_force_update_fw_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
		struct pressure_sensor_data *data = dev_get_drvdata(dev);
		unsigned long val;
		int rc;
		SENSOR_LOG_DEBUG("pressure_force_update_fw_store start!!!\n");
		if (size > 2)
			return -EINVAL;

		rc = kstrtoul(buf, 10, &val);
		if (rc != 0)
			return rc;
		SENSOR_LOG_DEBUG("val = %ld!\n",val);
		if (!data->loading_fw  && val) {
			data->loading_fw = true;
			SENSOR_LOG_DEBUG("pressure_fw_upgrade start!!!");
			pressure_fw_upgrade(dev, true);
			data->loading_fw = false;
		}

		return size;
}


static ssize_t pressure_fw_name_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
		struct pressure_sensor_data *data = dev_get_drvdata(dev);
		return snprintf(buf, FW_NAME_MAX_LEN - 1, "%s\n", data->fw_name);
}

static ssize_t pressure_fw_name_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
		struct pressure_sensor_data *data = dev_get_drvdata(dev);

		if (size > FW_NAME_MAX_LEN - 1)
			return -EINVAL;

		strlcpy(data->fw_name, buf, size);
		if (data->fw_name[size-1] == '\n')
			data->fw_name[size-1] = 0;

		return size;
}

static ssize_t pressure_debug_show(struct device *dev, struct device_attribute *attr, char *buf)
{
		return 0;
}

static ssize_t pressure_debug_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
		struct pressure_sensor_data *pdata = dev_get_drvdata(dev);
		unsigned long val;
		int rc;
		u8 normal_mode = 0;
		u8 debug_mode = 1;

		SENSOR_LOG_DEBUG("pressure_fw_update_debug_store start!!!\n");
		if (size > 2)
			return -EINVAL;

		rc = kstrtoul(buf, 10, &val);
		if (rc != 0)
			return rc;
		SENSOR_LOG_DEBUG("val = %ld!\n",val);

		if (val) {
			rc = pressure_vlps_write_reg(pdata, IC_DEBUG_MODE, &debug_mode, 1);
			if(rc < 0) {
				SENSOR_LOG_DEBUG("goto debug mode failed\n");
				return -1;
			}
			schedule_delayed_work(&pdata->debug_work, msecs_to_jiffies(100));
		} else {
			cancel_delayed_work_sync(&pdata->debug_work);

			rc = pressure_vlps_write_reg(pdata, IC_DEBUG_MODE, &normal_mode, 1);
			if(rc < 0) {
				SENSOR_LOG_DEBUG("exit debug mode failed\n");
				return -1;
			}
		}

		return rc;
}

static ssize_t pressure_fw_info_show(struct device *dev, struct device_attribute *attr, char *buf)
{
		struct pressure_sensor_data *pdata = dev_get_drvdata(dev);
		int err = -1;

		err = pressure_ic_info_read(pdata);
		if (err < 0) {
			SENSOR_LOG_ERROR("read fw_info failed]\n");
			return -1;
		}

		return sprintf(buf, "manufactor_id: %x,\nmodule_id: %x,\nfw_version: %x,\n",
				pdata->fw_info.manufactor_id,
				pdata->fw_info.module_id,
				pdata->fw_info.fw_version);
}


static ssize_t pressure_key_value_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
		int ret = 0;
		u8 key_up = 0;
		u8 key_down = 0;
		struct pressure_sensor_data *pdata = dev_get_drvdata(dev);

		ret = pressure_vlps_read_reg(pdata, IC_CHANNEL_KEY_UP ,&key_up, 1);
		if (ret < 0) {
			return ret;
		}
		ret = pressure_vlps_read_reg(pdata, IC_CHANNEL_KEY_DOWN ,&key_down,1);
		if (ret < 0) {
			return ret;
		}

		return sprintf(buf, "%d,%d\n", key_up, key_down);
}

static ssize_t pressure_sensitivity_attr_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
		struct pressure_sensor_data *pdata = dev_get_drvdata(dev);
		int err = 0;

		err = pressure_vlps_read_reg(pdata, IC_KEY_SENSITIVITY, &(pdata->sensitivity), 1);
		if (err < 0) {
			pdata->sensitivity = -1;
		}

		return sprintf(buf, "%d\n", pdata->sensitivity);
}

static ssize_t pressure_sensitivity_attr_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
		struct pressure_sensor_data *pdata = dev_get_drvdata(dev);
		unsigned long val;
		int rc;
		u8 s_val;

		SENSOR_LOG_DEBUG("pressure_sensitivity_attr_store start!!!");
		if (size > 2)
			return -EINVAL;

		rc = kstrtoul(buf, 10, &val);
		if (rc != 0)
			return rc;
		SENSOR_LOG_DEBUG("val = %ld!\n",val);

		s_val = val & 0xff;

		rc = pressure_vlps_write_reg(pdata, IC_KEY_SENSITIVITY, &s_val, 1);
		if(rc < 0){
			SENSOR_LOG_ERROR("write sensitivity attr fail\n");
			return -1;
		}
		mutex_lock(&pdata->lock);
		pdata->sensitivity = s_val;
		mutex_unlock(&pdata->lock);

		return size;
}

static ssize_t pressure_slave_status_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
		struct pressure_sensor_data *pdata = dev_get_drvdata(dev);
		int err = 0;
		char status[10];

		err = pressure_vlps_read_reg(pdata, IC_HOST_STATUS, &(pdata->slave_status), 1);
		if (err < 0) {
			pdata->slave_status = -1;
		}

		switch(pdata->slave_status) {
			case 0:
				strcpy(status, "normal");
				break;
			case 1:
				strcpy(status, "standby");
				break;
			case 2:
				strcpy(status, "poweroff");
				break;
			case 3:
				strcpy(status, "startup");
				break;
			default:
				strcpy(status, "unkown");
			break;
		}

		return sprintf(buf, "slave status:%s\n", status);
}
static ssize_t pressure_slave_status_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
		struct pressure_sensor_data *pdata = dev_get_drvdata(dev);
		int rc;
		SENSOR_LOG_DEBUG("pressure_report_host_status_store start!!!");

		rc = pressure_host_status_report(pdata, POWEROFF);
		if (rc < 0) {
			SENSOR_LOG_ERROR("wake up slave ic failed\n");
		}
		SENSOR_LOG_DEBUG("pressure sensor goto power off mode!!!");

		return size;
}
static ssize_t pressure_reset_device_show(struct device *dev, struct device_attribute *attr, char *buf)
{
		return 0;
}
static ssize_t pressure_reset_device_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
		struct pressure_sensor_data *pdata = dev_get_drvdata(dev);
		int rc;
		SENSOR_LOG_DEBUG("pressure_reset_device_store start!!!");

		rc = pressure_reset_device_work(pdata);
		if (rc < 0) {
			SENSOR_LOG_ERROR("reset and wake up slave device failed\n");
		}

		return size;
}
static ssize_t pressure_device_vlps_wakeup_show(struct device *dev, struct device_attribute *attr, char *buf)
{
		return 0;
}
static ssize_t pressure_device_vlps_wakeup_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
		struct pressure_sensor_data *pdata = dev_get_drvdata(dev);
		int rc;
		SENSOR_LOG_DEBUG("start!");
		mutex_lock(&pdata->lock);
		rc = pressure_slave_vlps_ops(pdata);
		if (rc < 0) {
		    mutex_unlock(&pdata->lock);
		    SENSOR_LOG_ERROR("vlps wakeup failed\n");
		}
		mutex_unlock(&pdata->lock);
		return size;
}
static ssize_t pressure_enable_smart_key_show(struct device *dev, struct device_attribute *attr, char *buf)
{
		struct pressure_sensor_data *pdata = dev_get_drvdata(dev);
		return sprintf(buf, "%d\n" , pdata->support_stay_awake);
}
static ssize_t pressure_enable_smart_key_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
		struct pressure_sensor_data *pdata = dev_get_drvdata(dev);
		int rc;
		unsigned long val;
		SENSOR_LOG_DEBUG("start!");
		if (size > 2)
			return -EINVAL;

		rc = kstrtoul(buf, 10, &val);
		if (rc != 0)
			return -EINVAL;
		SENSOR_LOG_DEBUG("val = %ld!\n",val);

		if (pdata->support_smart_key) {
			if (val){
				pdata->support_stay_awake = 1;
				SENSOR_LOG_DEBUG("enable smart key stay half-wakeup\n");
			} else {
				pdata->support_stay_awake = 0;
				SENSOR_LOG_DEBUG("disable smart key stay half-wakeup\n");
			}
			rc = pressure_file_write(SMART_KEY_CFG, (const char *)&pdata->support_stay_awake,
                        sizeof(pdata->support_stay_awake));
			if (rc < 0) {
			    SENSOR_LOG_ERROR("write smart key cfg error\n");
			    return size;
			}
		}
		return size;
}
static struct device_attribute attrs_pressure_device[] = {
		__ATTR(enable_update_fw, 0664, pressure_update_fw_show,pressure_update_fw_store),
		__ATTR(force_update_fw, 0640, pressure_update_fw_show,pressure_force_update_fw_store),
		__ATTR(fw_name, 0664, pressure_fw_name_show,pressure_fw_name_store),
		__ATTR(pressure_debug, 0664, pressure_debug_show, pressure_debug_store),
		__ATTR(fw_info, 0444, pressure_fw_info_show ,NULL),
		__ATTR(key_value, 0440, pressure_key_value_show ,NULL),
		__ATTR(sensitivity_attr, 0664, pressure_sensitivity_attr_show ,pressure_sensitivity_attr_store),
		__ATTR(report_host_status, 0664, pressure_slave_status_show, pressure_slave_status_store),
		__ATTR(reset, 0640, pressure_reset_device_show, pressure_reset_device_store),
		__ATTR(vlps_wakeup, 0664, pressure_device_vlps_wakeup_show, pressure_device_vlps_wakeup_store),
};
static struct device_attribute attrs_smartkey_device[] = {
		__ATTR(enable_smart_key, 0640, pressure_enable_smart_key_show, pressure_enable_smart_key_store),
};

static int create_sysfs_interfaces(struct device *dev)
{
		int i;
		struct pressure_sensor_data *pdata = dev_get_drvdata(dev);
		for (i = 0; i < ARRAY_SIZE(attrs_pressure_device); i++)
		{
			if (device_create_file(dev, attrs_pressure_device + i)) {
				SENSOR_LOG_ERROR("Unable to create interface\n");
				return -1;
			}
		}
		if (pdata->support_smart_key) {
		    for (i = 0; i < ARRAY_SIZE(attrs_smartkey_device); i++)
		    {
			    if (device_create_file(dev, attrs_smartkey_device + i)) {
				    SENSOR_LOG_ERROR("Unable to create smartkey interface\n");
				    return -1;
			    }
		    }
		}
		return 0;
}

static int remove_sysfs_interfaces(struct device *dev)
{
		int i;
		struct pressure_sensor_data *pdata = dev_get_drvdata(dev);
		for (i = 0; i < ARRAY_SIZE(attrs_pressure_device); i++)
		{
		    device_remove_file(dev, attrs_pressure_device + i);
		}
		if (pdata->support_smart_key) {
		    for (i = 0; i < ARRAY_SIZE(attrs_smartkey_device); i++)
		    {
		        device_remove_file(dev, attrs_smartkey_device + i);
		    }
		}
		return 0;
}

static int pressure_ic_info_read(struct pressure_sensor_data *pdata)
{
		int err = 0;

		err = pressure_vlps_read_reg(pdata, IC_MANUFACTURER_ID_ADDR, &(pdata->fw_info.manufactor_id), 2);
		if (err < 0) {
			pdata->fw_info.manufactor_id = -1;
		}

		err = pressure_read_reg(pdata, IC_MODULE_ID_ADDR, &(pdata->fw_info.module_id), 2);
		if (err < 0) {
			pdata->fw_info.module_id = -1;
		}

		err = pressure_read_reg(pdata, IC_FW_VERSION_ADDR, &(pdata->fw_info.fw_version), 2);
		if (err < 0) {
			pdata->fw_info.fw_version = -1;
		}

		err = pressure_vlps_read_reg(pdata, IC_KEY_SENSITIVITY, &(pdata->sensitivity), 1);
		if (err < 0) {
			pdata->sensitivity = -1;
		}
		err = pressure_read_reg(pdata, IC_HOST_STATUS, &(pdata->slave_status), 1);
		if (err < 0) {
			pdata->slave_status = -1;
		}

		return err;
}

static int pressure_sensor_data_init(struct pressure_sensor_data *pdata)
{
		int err = 0;

		pdata->suspended = false;
		pdata->loading_fw = false;
		pdata->page_size = PAGE_SZIE;
		pdata->wake_from_suspended = false;
		pdata->pressure_sensor_reboot_nb.notifier_call =
                        pressure_sensor_reboot_nb_handler;
		INIT_DELAYED_WORK(&pdata->debug_work, pressure_debug_work_func);

		INIT_WORK(&pdata->irq_work, pressure_key_irq_work);
		wake_lock_init(&pdata->pressure_wake_lock, WAKE_LOCK_SUSPEND, "pressure_wake_lock");

		pdata->fw_size = 0;

		err = pressure_ic_info_read(pdata);
		if (err < 0) {
			SENSOR_LOG_ERROR("read ic info from IC failed");
			return err;
		}

		return 0;
}
static int pressure_device_pin_init(struct pressure_sensor_data *pdata, bool on)
{
		int ret = -1;
		unsigned char label_name[32];
		if (on) {
			snprintf(label_name, 32, "%s", "pressure_reset_pin");
			if (gpio_is_valid(pdata->reset_pin)) {
				ret = gpio_request(pdata->reset_pin, label_name);
				if (ret) {
					gpio_free(pdata->reset_pin);
					ret = gpio_request(pdata->reset_pin, label_name);
					if (ret) {
						SENSOR_LOG_ERROR("Failed to request reset gpio %d\n", pdata->reset_pin);
						return -1;
					}
				}
				ret = pressure_device_reset_on(pdata, 0);
				if (ret < 0) {
						return ret;
				}
			} else {
				SENSOR_LOG_ERROR("reset gpio not provided\n");
			}

			snprintf(label_name, 32, "%s", "pressure_vlps_pin");
			if (gpio_is_valid(pdata->vlps_pin)) {
				ret = gpio_request(pdata->vlps_pin, label_name);
				if (ret) {
					gpio_free(pdata->vlps_pin);
					ret = gpio_request(pdata->vlps_pin, label_name);
					if (ret) {
						SENSOR_LOG_ERROR("Failed to request reset gpio %d\n", pdata->reset_pin);
						return -1;
					}
				}
				ret = pressure_device_vlps_on(pdata, 1);
				if (ret < 0) {
						return ret;
				}
			} else {
				SENSOR_LOG_ERROR("reset gpio not provided\n");
			}
		} else {

			if (pdata->reset_pin)
				gpio_free(pdata->reset_pin);
			if (pdata->vlps_pin)
				gpio_free(pdata->vlps_pin);
		}
		return 0;
}

static int pressure_device_pinctrl_init(struct pressure_sensor_data *pdata, bool on)
{
		int ret = -1;
		struct i2c_client *client;
		client = pdata->client;
		if(on) {
			pdata->pin.pinctrl = devm_pinctrl_get(&client->dev);
			if (IS_ERR_OR_NULL(pdata->pin.pinctrl)) {
				SENSOR_LOG_INFO("driver does not use pinctrl\n");
				pdata->pin.pinctrl = NULL;
				return -ENODEV;
			}

			pdata->pin.pin_default = pinctrl_lookup_state(pdata->pin.pinctrl, "pressure_ndt_default");
			if (IS_ERR_OR_NULL(pdata->pin.pin_default)) {
				SENSOR_LOG_ERROR("lookup default state failed\n");
				return PTR_ERR(pdata->pin.pin_sleep);
			}

			pdata->pin.pin_sleep = pinctrl_lookup_state(pdata->pin.pinctrl, "pressure_ndt_sleep");
			if (IS_ERR_OR_NULL(pdata->pin.pin_sleep)) {
				SENSOR_LOG_ERROR("lookup sleep state failed\n");
				return PTR_ERR(pdata->pin.pin_sleep);
			}
			if (!IS_ERR_OR_NULL(pdata->pin.pinctrl)) {
				ret = pinctrl_select_state(pdata->pin.pinctrl, pdata->pin.pin_default);
				if (ret) {
					SENSOR_LOG_ERROR("select default state failed\n");
					return ret;
				}
			}
		} else {
			pdata->pin.pinctrl = NULL;
			pdata->pin.pin_sleep = NULL;
			pdata->pin.pin_default = NULL;
		}

		SENSOR_LOG_INFO("pinctrl init success\n");
		return 0;
}
static int pressure_device_pinctrl_configure(struct pressure_sensor_data *pdata, bool on)
{
		int ret = 0;
		if (!IS_ERR_OR_NULL(pdata->pin.pinctrl)) {
			if (on) {
				ret = pinctrl_select_state(pdata->pin.pinctrl, pdata->pin.pin_default);
				if (ret) {
					SENSOR_LOG_ERROR("select default state failed\n");
					return ret;
				}
			} else {
				ret = pinctrl_select_state(pdata->pin.pinctrl, pdata->pin.pin_sleep);
				if (ret) {
					SENSOR_LOG_ERROR("select sleep state failed\n");
					return ret;
				}
			}
		} else {
			SENSOR_LOG_ERROR("pinctrl is NULL\n");
			return -EINVAL;
		}
		return 0;
}
static int pressure_device_irq_init(struct pressure_sensor_data *pdata, bool on)
{
		int ret = -1;
		unsigned char label_name[32];
		if (on) {
			snprintf(label_name, 32, "%s", "pressure_int_pin");
			if (gpio_is_valid(pdata->irq.irq_pin)) {
				/* configure pressure sensor irq gpio */
				SENSOR_LOG_INFO("gpio value is %d \n", gpio_get_value(pdata->irq.irq_pin));
				ret = gpio_request_one(pdata->irq.irq_pin,
										GPIOF_DIR_IN,
										label_name);
				if (ret) {
					SENSOR_LOG_ERROR("unable to request gpio %d\n", pdata->irq.irq_pin);
					return -1;
				}
				pdata->irq.irq_num = pdata->client->irq = gpio_to_irq(pdata->irq.irq_pin);
				irq_set_irq_wake(pdata->irq.irq_num, 1);
				pressure_device_irq_enable(&(pdata->irq), false, false);
				ret = request_threaded_irq(pdata->client->irq, NULL,
									&pressure_key_irq_handler,
									IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
									"pressure_irq",
									pdata);
				if (ret != 0) {
				    gpio_free(pdata->irq.irq_pin);
				    return -1;
				}
				SENSOR_LOG_INFO("irq init success\n");
		    } else {
				SENSOR_LOG_ERROR("irq gpio not provided\n");
				return -1;
		    }
		} else {
		    if (pdata->irq.irq_pin)
		        gpio_free(pdata->irq.irq_pin);
		}
		return 0;
}

static int pressure_device_hw_init(struct pressure_sensor_data *pdata, bool on)
{

		int err = 0;
		err = pressure_device_pin_init(pdata, on);
		if (err < 0) {
		    SENSOR_LOG_ERROR("pressure_device_resetpin_init failed\n");
		    return err;
		}
		if (pdata->support_smart_key) {
		    err = pressure_device_pinctrl_init(pdata, on);
		    if (err < 0) {
		        SENSOR_LOG_ERROR("pressure device pinctrl init failed\n");
		        return err;
		    }
		    err = pressure_device_irq_init(pdata, on);
		    if (err < 0) {
		        SENSOR_LOG_ERROR("pressure_device_irqpin_init failed\n");
		        return err;
		    }
		}
		if (on)
		    SENSOR_LOG_INFO("pressure sensor hw init success\n");
		else
		    SENSOR_LOG_INFO("pressure sensor hw deinit success\n");
		return err;
}

static void pressure_device_irq_enable(struct smart_key_irq *irq_info, bool enable, bool flag_sync)
{
		if (enable == irq_info->enabled) {
		    SENSOR_LOG_INFO("doubule %s irq %d, retern here\n",enable? "enable" : "disable", irq_info->irq_num);
		    return;
		} else {
		    irq_info->enabled  = enable;
		    SENSOR_LOG_INFO("%s irq %d\n",enable? "enable" : "disable",irq_info->irq_num);
		}

		if (enable) {
		    enable_irq(irq_info->irq_num);
		} else {
		    if (flag_sync) {
		        disable_irq(irq_info->irq_num);
		    } else {
		        disable_irq_nosync(irq_info->irq_num);
		    }
		}
}

static void pressure_key_irq_work(struct work_struct *work)
{
		struct pressure_sensor_data *pdata = container_of(work, struct pressure_sensor_data, irq_work);
		struct input_dev *input = pdata->idev;

		pdata->key.state = gpio_get_value(pdata->irq.irq_pin) ? PULL_UP : PULL_DOWN;

		if (pdata->key.state != pdata->key.prev_state) {
		    pdata->key.prev_state = pdata->key.state;
		    SENSOR_LOG_INFO("PRESSURE KEY STATE is %s\n", pdata->key.state ==2 ? "PULL_UP" : "PULL_DOWN");
		    if (pdata->wake_from_suspended) {
		        input_event(input, EV_KEY, pdata->key.wakeup_code, !(pdata->key.state & 0x2));
				SENSOR_LOG_INFO("IRQ: wakeup from suspend\n");
				pdata->wake_from_suspended = false;
		    } else {
		        input_event(input, EV_KEY, pdata->key.code, !(pdata->key.state & 0x2));
		    }
		    input_sync(pdata->idev);
		}
		pressure_device_irq_enable(&(pdata->irq), true, false);
}

static irqreturn_t pressure_key_irq_handler(int irq, void *handle)
{
		struct pressure_sensor_data *pdata = handle;
		pressure_device_irq_enable(&(pdata->irq), false, false);
		pdata->wake_from_suspended = pdata->suspended;
		if (0 == schedule_work(&pdata->irq_work)) {
		    SENSOR_LOG_ERROR("schedule_work failed!\n");
		}
		return IRQ_HANDLED;
}


static int pressure_input_device_init(struct pressure_sensor_data *pdata)
{
		int err = 0;
		pdata->idev = input_allocate_device();
		if (IS_ERR_OR_NULL(pdata->idev)) {
		    SENSOR_LOG_ERROR("allocate input dev fail.\n");
		    return -ENODEV;
		}
		input_set_drvdata(pdata->idev, pdata);
		pdata->idev->name = "pressure_key";
		pdata->idev->id.bustype = BUS_HOST;
		pdata->idev->phys = "pressure_key/input0";
		pdata->idev->dev.parent = pdata->pressure_dev;
		pdata->idev->id.product = 0x276;
		pdata->idev->id.vendor= 0x3002;
		pdata->idev->id.version= 0x3001;
		set_bit(EV_KEY, pdata->idev->evbit);
		set_bit(pdata->key.code, pdata->idev->keybit);
		set_bit(pdata->key.wakeup_code, pdata->idev->keybit);
		err = input_register_device(pdata->idev);
		if (err) {
		    SENSOR_LOG_ERROR("register input device fialed\n");
		    err = -ENODEV;
		    goto input_alloc_failed;
		}
		return 0;
input_alloc_failed:
		if(!IS_ERR_OR_NULL(pdata->idev))
		    input_free_device(pdata->idev);
		return err;
}

static int pressure_device_vlps_on(struct pressure_sensor_data *pdata, bool on)
{
		int rc = 0;
		if (on) {

		    rc = gpio_direction_output(pdata->vlps_pin, 1);
		    if (rc) {
		        SENSOR_LOG_ERROR("set_direction for vlps gpio failed\n");
		        return -EINVAL;
		    }
		    gpio_set_value_cansleep(pdata->vlps_pin, 1);

		} else {

		    rc = gpio_direction_output(pdata->vlps_pin, 0);
		    if (rc) {
		        SENSOR_LOG_ERROR("set_direction for vlps gpio failed\n");
		        return -EINVAL;
		    }

		    gpio_set_value_cansleep(pdata->vlps_pin, 0);
		}
		return rc;
}

static int pressure_device_reset_on(struct pressure_sensor_data *pdata, bool on)
{
		int rc = 0;
		if (on){
		    rc = gpio_direction_output(pdata->reset_pin, 1);
		    if (rc) {
		        SENSOR_LOG_ERROR("set_direction for reset gpio failed\n");
		        return -EINVAL;
		    }
		    gpio_set_value_cansleep(pdata->reset_pin, 1);
		    SENSOR_LOG_DEBUG("set reset_pin high\n");
		} else {

		    rc = gpio_direction_output(pdata->reset_pin, 0);
		    if (rc) {
		        SENSOR_LOG_ERROR("set_direction for reset gpio failed\n");
		        return -EINVAL;
		    }
		    gpio_set_value_cansleep(pdata->reset_pin, 0);
		    SENSOR_LOG_DEBUG("set reset_pin low\n");
		}
		return rc;
}


static int pressure_read_device_id(struct pressure_sensor_data *pdata)
{
		int err = 0;
		err = pressure_vlps_read_reg(pdata, IC_MANUFACTURER_ID_ADDR, &(pdata->fw_info.manufactor_id), 2);
		if (err < 0) {
		     SENSOR_LOG_INFO("read device id failed\n");
		     pdata->fw_info.manufactor_id = -1;
		     return err;
		}
		return err;
}
static int pressure_parse_dt(struct device *dev, struct pressure_sensor_data *pdata)
{
		int rc;
		const char *pname;
		struct device_node *np = dev->of_node;

		rc = of_property_read_string(np, "pressure,fw-name", &pname);
		strcpy(pdata->fw_name,pname);

		if (rc && (rc != -EINVAL)) {
			SENSOR_LOG_ERROR("Unable to read fw name\n");
			return rc;
		}

		rc = of_get_named_gpio_flags(np, "pressure,reset-pin", 0, NULL);
		if (rc < 0) {
			SENSOR_LOG_ERROR("Unable to read reset gpio\n");
			return rc;
		}
		pdata->reset_pin = rc;
		SENSOR_LOG_INFO("reset_pin is %d\n",pdata->reset_pin);

		rc = of_get_named_gpio_flags(np, "pressure,vlps-pin", 0, NULL);
		if (rc < 0) {
			SENSOR_LOG_ERROR("Unable to read vlps gpio\n");
			return rc;
		}
		pdata->vlps_pin = rc;

		SENSOR_LOG_INFO("vlps_pin is %d\n",pdata->vlps_pin);

		rc = of_property_read_u32(np, "pressure,support_smart_key",
                                    &pdata->support_smart_key);
		if (rc < 0) {
			SENSOR_LOG_ERROR("pressure_key,code dose not exist\n ");
			pdata->support_smart_key = 0;
		}
		SENSOR_LOG_INFO("support_smart_key is %d\n", pdata->support_smart_key);
		if (pdata->support_smart_key) {
			rc = of_get_named_gpio_flags(np, "pressure,irq-pin", 0, NULL);
			if (rc < 0) {
				SENSOR_LOG_ERROR("Unable to read irq gpio\n");
				return -ENODEV;
			}
			pdata->irq.irq_pin = rc;
			SENSOR_LOG_INFO("int_pin is %d\n", pdata->irq.irq_pin);

			rc = of_property_read_u32(np, "pressure_key,code",
			                            &pdata->key.code);
			if (rc < 0) {
				SENSOR_LOG_ERROR("pressure_key,code dose not exist\n ");
			}
			SENSOR_LOG_INFO("pressure key code is %d\n", pdata->key.code);
			rc = of_property_read_u32(np, "pressure_key,wakeup_code",
			                            &pdata->key.wakeup_code);
			if (rc < 0) {
				SENSOR_LOG_ERROR("pressure_key,wakeup_code dose not exist\n ");
			}
			SENSOR_LOG_INFO("pressure key code is %d\n", pdata->key.wakeup_code);
			rc = of_property_read_u32(np, "pressure_key,type",
                                        &pdata->key.type);
			if (rc < 0){
				pdata->key.type = EV_KEY;
			}
		}
		return 0;
}

static const struct dev_pm_ops pressure_sensor_pm_ops = {
		.suspend = pressure_sensor_suspend,
		.resume = pressure_sensor_resume,
};

static const struct i2c_device_id pressure_sensor_id[] = {
		{"pressure_sensor", 0},
		{},
};

MODULE_DEVICE_TABLE(i2c, pressure_sensor_id);

static struct of_device_id pressure_match_table[] = {
		{ .compatible = "nubia,pressure_sensor_ndt",},
		{ },
};

static struct i2c_driver pressure_sensor_driver = {
		.probe = pressure_sensor_probe,
		.remove = pressure_sensor_remove,
		.driver = {
			.name = "pressure_sensor",
			.owner = THIS_MODULE,
			.of_match_table = pressure_match_table,
			.pm = &pressure_sensor_pm_ops,
			},
		.id_table = pressure_sensor_id,
};

static int pressure_sensor_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
		struct pressure_sensor_data *pdata;
		int err = -1;

		SENSOR_LOG_DEBUG("pressure probe enter!\n");

		pdata = kzalloc(sizeof(struct pressure_sensor_data), GFP_KERNEL);
		if (!pdata) {
			SENSOR_LOG_ERROR("fail to allocate pressure_sensor_data");
			return -ENOMEM;
		}

		err = pressure_parse_dt(&client->dev, pdata);
		if (err) {
			SENSOR_LOG_ERROR("DT parsing failed\n");
			goto init_device_fail;
		}

		mutex_init(&pdata->lock);

		if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
			SENSOR_LOG_ERROR("I2C_FUNC_I2C not supported\n");
			return -ENODEV;
		}

		pdata->client = client;
		i2c_set_clientdata(client, pdata);

		err = pressure_read_device_id(pdata);
		if (err < 0) {
				SENSOR_LOG_ERROR("read pressure device id failed\n");
				goto init_device_fail;
		}

		SENSOR_LOG_INFO("firmware name is %s\n",pdata->fw_name);

		pressure_device_class = class_create(THIS_MODULE, "pressure_device");

		pdata->pressure_dev = device_create(pressure_device_class, NULL, pressure_device_dev_t,
			&pressure_sensor_driver ,"pressure_device");
		if (IS_ERR(pdata->pressure_dev)) {
			goto init_device_fail;
		}

		dev_set_drvdata(pdata->pressure_dev, pdata);
		err = create_sysfs_interfaces(pdata->pressure_dev);
		if (err < 0) {
			SENSOR_LOG_ERROR("create_sysfs_interfaces failed\n");
			goto create_pressure_dev_failed;
		}
		err = pressure_device_hw_init(pdata, 1);
		if (err < 0) {
			SENSOR_LOG_ERROR("pressure_device_hw_init failed\n");
			goto create_sysfs_interfaces_failed;
		}

		err = pressure_input_device_init(pdata);
		if (err < 0) {
			SENSOR_LOG_ERROR("pressure_input_device_init failed\n");
			goto hw_init_exit;
		}

		if (pressure_sensor_data_init(pdata) < 0)
			goto input_device_exit;

		msleep(1);
		if (pressure_reset_device_work(pdata) < 0)
		    goto pressure_data_exit;

		register_reboot_notifier(&pdata->pressure_sensor_reboot_nb);

		SENSOR_LOG_INFO("prob success\n");
		return 0;
pressure_data_exit:
		wake_lock_destroy(&pdata->pressure_wake_lock);
		mutex_destroy(&pdata->lock);
input_device_exit:
		if (!IS_ERR_OR_NULL(pdata->idev)) {
			input_unregister_device(pdata->idev);
			pdata->idev = NULL;
        }
hw_init_exit:
		pressure_device_hw_init(pdata, 0);
create_sysfs_interfaces_failed:
		remove_sysfs_interfaces(pdata->pressure_dev);
create_pressure_dev_failed:
		device_destroy(pressure_device_class, pressure_device_dev_t);
		class_destroy(pressure_device_class);
init_device_fail:
		kfree(pdata);
		SENSOR_LOG_INFO("probe failed\n");
		return -ENODEV;
}

static int pressure_sensor_remove(struct i2c_client *client)
{
		struct pressure_sensor_data *pdata = i2c_get_clientdata(client);

		pressure_device_hw_init(pdata, 0);
		input_unregister_device(pdata->idev);
		remove_sysfs_interfaces(pdata->pressure_dev);
		mutex_destroy(&pdata->lock);
		wake_lock_destroy(&pdata->pressure_wake_lock);
		kfree(pdata);
		SENSOR_LOG_INFO("pressure_device_remove\n");
		return 0;
}

static int __init pressure_sensor_init(void)
{
		return i2c_add_driver(&pressure_sensor_driver);
}

static void __exit pressure_sensor_exit(void)
{
		i2c_del_driver(&pressure_sensor_driver);
}

module_init(pressure_sensor_init);
module_exit(pressure_sensor_exit);

MODULE_DESCRIPTION("Pressure newdegree sensor driver");
MODULE_LICENSE("GPL v2");
