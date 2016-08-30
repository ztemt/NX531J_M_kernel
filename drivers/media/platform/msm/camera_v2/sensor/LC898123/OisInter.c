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

#define pr_fmt(fmt) "%s:%d " fmt, __func__, __LINE__

#include <linux/module.h>
#include "msm_sd.h"
#include "OisInter.h"
#include "Ois.h"
#include "msm_cci.h"

/*#define MSM_OIS_DEBUG*/
#undef CDBG
#ifdef MSM_OIS_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) pr_debug(fmt, ##args)
#endif

static struct msm_ois_ctrl_t msm_ois_lc898123_t;

static struct msm_camera_i2c_fn_t msm_sensor_cci_func_tbl = {
	.i2c_read = msm_camera_cci_i2c_read,
	.i2c_read_seq = msm_camera_cci_i2c_read_seq,
	.i2c_write_seq = msm_camera_cci_i2c_write_seq,
	.i2c_write = msm_camera_cci_i2c_write,
	.i2c_write_table = msm_camera_cci_i2c_write_table,
	.i2c_write_seq_table = msm_camera_cci_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
		msm_camera_cci_i2c_write_table_w_microdelay,
	.i2c_util = msm_sensor_cci_i2c_util,
};

#define		DeviceAddr		0x7C  	// Device address of driver IC

/*------------------------------------------------------
  	IIC wrtie multi bytes function (Max 64 bytes)
  	Parameters:	*PcSetDat  -- buffer pointer (the first byte is register address)
  				UsDatNum   -- data length
--------------------------------------------------------*/
void CntWrt( void *	PcSetDat, unsigned short UsDatNum )
{
    int rc = 0;
    unsigned char *SetDat = (unsigned char *)PcSetDat;
    enum msm_camera_i2c_reg_addr_type adtype = msm_ois_lc898123_t.i2c_client.addr_type;
    uint16_t uNum = UsDatNum;

    if (PcSetDat == NULL)
    {
        return;
    }
    if (UsDatNum > 1)
    {
        uNum = UsDatNum - 1;
    }
    
    msm_ois_lc898123_t.i2c_client.cci_client->sid = 0x7C >> 1;
    msm_ois_lc898123_t.i2c_client.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
    msm_ois_lc898123_t.i2c_data_type = MSM_ACTUATOR_BYTE_DATA;
    
    rc = msm_ois_lc898123_t.i2c_client.i2c_func_tbl->i2c_write_seq(
                  &msm_ois_lc898123_t.i2c_client,
                  SetDat[0],
                  &SetDat[1],
                  uNum);

    msm_ois_lc898123_t.i2c_client.addr_type = adtype;

    if (rc < 0) 
    {
        pr_err("%s: line %d rc = %d\n", __func__, __LINE__, rc);
    }

    return;
}

/*------------------------------------------------------
  	IIC read multi bytes function (Max 64 bytes)
  	Parameters:	addr	   -- register address
  				*PcSetDat  -- buffer pointer
  				UsDatNum   -- data length
--------------------------------------------------------*/
void CntRd(unsigned int addr, void *PcSetDat, unsigned short UsDatNum )
{
    int rc = 0;
    enum msm_camera_i2c_reg_addr_type adtype = msm_ois_lc898123_t.i2c_client.addr_type;
    uint8_t *pSetData = (uint8_t *)PcSetDat;

    if (pSetData == NULL)
    {
        return;
    }

    msm_ois_lc898123_t.i2c_client.cci_client->sid = 0x7C >> 1;
    msm_ois_lc898123_t.i2c_client.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
    msm_ois_lc898123_t.i2c_data_type = MSM_ACTUATOR_BYTE_DATA;
    
    rc = msm_ois_lc898123_t.i2c_client.i2c_func_tbl->i2c_read_seq(
                    &(msm_ois_lc898123_t.i2c_client),
                    addr,
                    pSetData,
                    UsDatNum);

    if (rc < 0) 
    {
        pr_err("%s: line %d rc = %d\n", __func__, __LINE__, rc);
    }

    msm_ois_lc898123_t.i2c_client.addr_type = adtype;

    return;
}

//********************************************************************************
// Function Name 	: WitTim
// Retun Value		: NON
// Argment Value	: Wait Time(ms)
// Explanation		: Timer Wait Function
//********************************************************************************
void	WitTim( unsigned short	UsWitTim )
{
    // To call your Wait function here
    msleep(UsWitTim);

}

//********************************************************************************
// Function Name 	: WPBCtrl
// Retun Value		: NON
// Argment Value	: UcCtrl (0: flash write disable  1: flash write enable)
// Explanation		: Flash write protection control
//					: WPB port = High (Flash write enable)
//                  : WPB port = Low (Flash write disable)
//********************************************************************************
void WPBCtrl( unsigned char UcCtrl )
{
    int rc = 0;
    uint16_t uSid = msm_ois_lc898123_t.i2c_client.cci_client->sid;

    msm_ois_lc898123_t.i2c_client.cci_client->sid = 0x34 >> 1;
    msm_ois_lc898123_t.i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
    
    if (UcCtrl == WPB_OFF)
    {
        pr_err("WPB_OFF \n");
        rc = msm_ois_lc898123_t.i2c_client.i2c_func_tbl->i2c_write(
             &msm_ois_lc898123_t.i2c_client,
             0x4670,
             0x00,
             MSM_ACTUATOR_BYTE_DATA);
        if (rc < 0)
        {
            pr_err("failed \n");
        }        
        rc = msm_ois_lc898123_t.i2c_client.i2c_func_tbl->i2c_write(
             &msm_ois_lc898123_t.i2c_client,
             0x4666,
             0xD7,
             MSM_ACTUATOR_BYTE_DATA);
        if (rc < 0)
        {
            pr_err("failed \n");
        }
    }
    else if (UcCtrl == WPB_ON)
    {
        pr_err("WPB_ON \n");
        rc = msm_ois_lc898123_t.i2c_client.i2c_func_tbl->i2c_write(
             &msm_ois_lc898123_t.i2c_client,
             0x4666,
             0x01,
             MSM_ACTUATOR_BYTE_DATA);
        if (rc < 0)
        {
            pr_err("failed \n");
        }        
    }

    msm_ois_lc898123_t.i2c_client.cci_client->sid = uSid;
}

/*------------------------------------------------------
  	IIC read 4 bytes function
  	Parameters:	addr(2bytes), *data(4bytes)
-------------------------------------------------------*/ 
void RamRead32A( unsigned short addr, void * data)
{
    uint8_t Regdata[4];
    uint32_t *temp_read_data_32 = (uint32_t *)data;
    int32_t rc = 0;
    
    msm_ois_lc898123_t.i2c_client.cci_client->sid = 0x7C >> 1;
    msm_ois_lc898123_t.i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
    msm_ois_lc898123_t.i2c_data_type = MSM_ACTUATOR_BYTE_DATA;
    
    rc =  msm_ois_lc898123_t.i2c_client.i2c_func_tbl->i2c_read_seq(
                &msm_ois_lc898123_t.i2c_client,
                addr, &Regdata[0],
                4);
    
    if (rc < 0) 
    {
        pr_err("%s: line %d rc = %d\n", __func__, __LINE__, rc);
    }
    
    *temp_read_data_32=  ((Regdata[0] << 24) | (Regdata[1] << 16) | (Regdata[2]<< 8) | Regdata[3]);
};

/*--------------------------------------------------------
  	IIC wrtie 4 bytes function
  	Parameters:	addr(2bytes), data(4bytes)
--------------------------------------------------------*/
void RamWrite32A(unsigned int addr, unsigned int data)
{
    uint8_t reqdata[4];
    int32_t rc=0;
    
    reqdata[0] = (data >> 24) & 0xFF;
    reqdata[1] = (data >> 16) & 0xFF;
    reqdata[2] = (data >> 8) & 0xFF;
    reqdata[3] = (data) & 0xFF;
    
    msm_ois_lc898123_t.i2c_client.cci_client->sid = 0x7C >> 1;
    msm_ois_lc898123_t.i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
    msm_ois_lc898123_t.i2c_data_type = MSM_ACTUATOR_BYTE_DATA;
    
    rc =  msm_ois_lc898123_t.i2c_client.i2c_func_tbl->i2c_write_seq(
                &msm_ois_lc898123_t.i2c_client,
                addr, &reqdata[0],
                4);
    
    if (rc < 0) {
        pr_err("%s: line %d rc = %d\n", __func__, __LINE__, rc);
    }
      
};

EXPORT_SYMBOL(CntWrt);
EXPORT_SYMBOL(CntRd);
EXPORT_SYMBOL(WitTim);
EXPORT_SYMBOL(WPBCtrl);
EXPORT_SYMBOL(RamRead32A);
EXPORT_SYMBOL(RamWrite32A);

void msm_ois_init_cci_lc898123(void)
{
    int rc = 0;
    rc = msm_ois_lc898123_t.i2c_client.i2c_func_tbl->i2c_util(
                       &msm_ois_lc898123_t.i2c_client, MSM_CCI_INIT);
    
    if (rc < 0)
    {
        pr_err("msm_ois_init_cci_lc898123 failed\n");
    }
}

void msm_ois_release_cci_lc898123(void)
{
    int rc;
    
    rc = msm_ois_lc898123_t.i2c_client.i2c_func_tbl->i2c_util(
            &msm_ois_lc898123_t.i2c_client, MSM_CCI_RELEASE);
    
    if (rc < 0)
    {
        pr_err("msm_ois_release_cci_lc898123 failed\n");
    }
}

void msm_ois_lc898123_enable(int enable)
{
    if (enable == 1)
    {
        RamWrite32A(CMD_OIS_ENABLE , OIS_ENABLE );
        RamWrite32A(CMD_PAN_TILT ,	PAN_TILT_ON );
    }
    else
    {
        RamWrite32A(CMD_OIS_ENABLE , OIS_DISABLE );
        RamWrite32A(CMD_PAN_TILT ,	PAN_TILT_OFF );
    }
}

void msm_ois_lc898123_set_stillmovie_mode(int mode)
{

    switch( mode )
    {
        case 0:
        {
            RamWrite32A( CMD_MOVE_STILL_MODE ,	STILL_MODE ) ;
            break;
        }
        case 1:
        {
            RamWrite32A( CMD_MOVE_STILL_MODE ,	STILL_MODE1 ) ;
            break;
        }
        case 2:
        {
            RamWrite32A( CMD_MOVE_STILL_MODE ,	STILL_MODE2 ) ;
            break;
        }
        case 3:
        {
            RamWrite32A( CMD_MOVE_STILL_MODE ,	STILL_MODE3 ) ;
            break;
        }
        case 10:
        {
            RamWrite32A( CMD_MOVE_STILL_MODE ,	MOVIE_MODE ) ;
            break;
        }
        case 11:
        {
            RamWrite32A( CMD_MOVE_STILL_MODE ,	MOVIE_MODE1 ) ;
            break;
        }
        case 12:
        {
            RamWrite32A( CMD_MOVE_STILL_MODE ,	MOVIE_MODE2 ) ;
            break;
        }
        case 13:
        {
            RamWrite32A( CMD_MOVE_STILL_MODE ,	MOVIE_MODE3 ) ;
            break;
        }
        default:
            break;
    }
}

int32_t msm_ois_lc898123_init_AF(void)
{
    unsigned int dat = 0, cnt = 0, fver = 0;
    unsigned int err = 0;
    unsigned short sver = 0;

    RamWrite32A(CMD_IO_ADR_ACCESS, 0xD00100);
    RamRead32A(CMD_IO_DAT_ACCESS, &dat);
    pr_err("LC898123 Chip type: %02X \n", dat & 0x0F);
    
    do{
    	RamWrite32A(CMD_IO_ADR_ACCESS, 0xD000AC );
    	RamRead32A(CMD_IO_DAT_ACCESS, &dat ); 
        
        msleep(5);
    } while( dat != 1 && cnt++ < 20);
    pr_err("LC898123 0xD000AC : %08X \n", dat);
    
    /* Check firmware version */
    if( dat == 1)
    {
        RamRead32A(SiVerNum, &fver);
        pr_err("\n LC898123 firmware version: %08X \n", fver);
    }

    sver = fver & 0xFF;
    
    #if 1 	// If no need to update firmware, comment out
    if ( dat == 0 || ((sver < VERNUM) && (sver != 0x04))) // firmware version is old ?
    //if ( dat == 0 ) // firmware version is old ?
    {
        pr_err("LC898123 updated to new firmware version begin flash");
        // update firmware
        err  = FlashUpdate();
        if(err == 0)
        {
            pr_err("LC898123 updated to new firmware version");
        } 
        else 
        {
            pr_err("LC898123 firmware update fail err :%d  !", (int)err);
        }
    }
    #endif
    
    //RamWrite32A(CMD_RETURN_TO_CENTER , BOTH_SRV_ON );
    //msleep(50);
    
    /* AF Operation */
    //msm_ois_setFocusPosition( 1000 ); // Move lens to 100 code
    
    //RamWrite32A(CMD_OIS_ENABLE , OIS_ENABLE );
    //RamWrite32A(CMD_PAN_TILT ,	PAN_TILT_ON ) ;

    return 0;
}


static unsigned char OisRdStatus( unsigned char UcStBitChk )
{
	unsigned long	UlReadVal ;
	
	RamRead32A( CMD_READ_STATUS , &UlReadVal );
      //pr_err("UlReadVal :%d\n", (int)UlReadVal);

	if( UcStBitChk ){
		UlReadVal &= READ_STATUS_INI ;
	}
	if( !UlReadVal ){
		return( SUCCESS );
	}else{
		return( FAILURE );
	}
}

void msm_ois_setFocusPosition(unsigned short pos)
{
    UINT8	UcStRd = 1;

    pr_err("pos :%d\n", pos);
    RamWrite32A( CMD_AF_POSITION, pos| 0x00010000 ) ;		// bit 16 : FST mode
    
    while( UcStRd ) 
    {
        UcStRd = OisRdStatus(1);
        //pr_err("UcStRd :%d\n", UcStRd);
    }
}

EXPORT_SYMBOL(msm_ois_lc898123_init_AF);
EXPORT_SYMBOL(msm_ois_setFocusPosition);
EXPORT_SYMBOL(msm_ois_lc898123_set_stillmovie_mode);
EXPORT_SYMBOL(msm_ois_lc898123_enable);

static int32_t msm_ois_i2c_init_lc898123(void)
{
    int32_t rc = 0;
    struct msm_camera_cci_client *cci_client = NULL;
    
    CDBG("Enter\n");
    
    msm_ois_lc898123_t.cci_master = 0;
    msm_ois_lc898123_t.cam_name = 0;
    
    msm_ois_lc898123_t.i2c_client.i2c_func_tbl = &msm_sensor_cci_func_tbl;
    msm_ois_lc898123_t.i2c_client.cci_client = kzalloc(sizeof(
            struct msm_camera_cci_client), GFP_KERNEL);
    
    if (!msm_ois_lc898123_t.i2c_client.cci_client) {
        pr_err("failed no memory\n");
        return -ENOMEM;
    }
    
    cci_client = msm_ois_lc898123_t.i2c_client.cci_client;
    cci_client->cci_subdev = msm_cci_get_subdev();
    
    msm_ois_lc898123_t.i2c_client.cci_client->sid = 0x7C >> 1;
    msm_ois_lc898123_t.i2c_client.cci_client->retries = 3;
    msm_ois_lc898123_t.i2c_client.cci_client->id_map = 0;
    msm_ois_lc898123_t.i2c_client.cci_client->cci_i2c_master = 0;
    msm_ois_lc898123_t.i2c_client.cci_client->i2c_freq_mode = I2C_FAST_MODE;

    CDBG("Exit\n");

    return rc;
}

static int __init msm_ois_init_module_lc898123(void)
{
	int32_t rc = 0;
	CDBG("Enter\n");
	rc = msm_ois_i2c_init_lc898123();
	if (!rc)
		return rc;
	CDBG("%s:%d rc %d\n", __func__, __LINE__, rc);
	return rc;
}

module_init(msm_ois_init_module_lc898123);
MODULE_DESCRIPTION("MSM OIS");
MODULE_LICENSE("GPL v2");

