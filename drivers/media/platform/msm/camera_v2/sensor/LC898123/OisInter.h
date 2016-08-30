/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef MSM_OIS_H
#define MSM_OIS_H

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <soc/qcom/camera2.h>
#include <media/v4l2-subdev.h>
#include <media/msmb_camera.h>
#include "msm_camera_i2c.h"
#include "msm_camera_dt_util.h"
#include "msm_camera_io_util.h"

struct msm_ois_ctrl_t {
	struct msm_camera_i2c_client i2c_client;
	enum af_camera_name cam_name;
      enum msm_actuator_data_type i2c_data_type;
	enum cci_i2c_master_t cci_master;
};

#define		VERNUM		0x05			//  

void CntWrt( void *	PcSetDat, unsigned short UsDatNum );
void CntRd(unsigned int addr, void *PcSetDat, unsigned short UsDatNum );
void	WitTim( unsigned short	UsWitTim );
void WPBCtrl( unsigned char UcCtrl );
void RamRead32A( unsigned short addr, void * data);
void RamWrite32A(unsigned int addr, unsigned int data);
int32_t msm_ois_lc898123_init_AF(void);
void msm_ois_setFocusPosition(unsigned short pos);
void msm_ois_lc898123_set_stillmovie_mode(int mode);
void msm_ois_lc898123_enable(int enable);

#endif

