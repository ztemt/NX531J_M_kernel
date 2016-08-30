//********************************************************************************
//		<< LC898123 Evaluation Soft >>
//********************************************************************************
//==============================================================================
// Calibration Data Memory Map
//==============================================================================
#define	CALIBRATION_DATA_ADDRESS	0x0100
#define	USER_AREA_ADDRESS			0x01B0

// Calibration Status
#define	CALIBRATION_STATUS		( FLASH_SECTOR_BUFFER + 0x00000000 )
// Hall amplitude Calibration X
#define	HALL_MAX_BEFORE_X		( FLASH_SECTOR_BUFFER + 0x00000004 )
#define	HALL_MIN_BEFORE_X		( FLASH_SECTOR_BUFFER + 0x00000008 )
#define	HALL_MAX_AFTER_X		( FLASH_SECTOR_BUFFER + 0x0000000C )
#define	HALL_MIN_AFTER_X		( FLASH_SECTOR_BUFFER + 0x00000010 )
// Hall amplitude Calibration Y
#define	HALL_MAX_BEFORE_Y		( FLASH_SECTOR_BUFFER + 0x00000014 )
#define	HALL_MIN_BEFORE_Y		( FLASH_SECTOR_BUFFER + 0x00000018 )
#define	HALL_MAX_AFTER_Y		( FLASH_SECTOR_BUFFER + 0x0000001C )
#define	HALL_MIN_AFTER_Y		( FLASH_SECTOR_BUFFER + 0x00000020 )
// Hall Bias/Offset
#define	HALL_BIAS_DAC_X			( FLASH_SECTOR_BUFFER + 0x00000024 )
#define	HALL_OFFSET_DAC_X		( FLASH_SECTOR_BUFFER + 0x00000028 )
#define	HALL_BIAS_DAC_Y			( FLASH_SECTOR_BUFFER + 0x0000002C )
#define	HALL_OFFSET_DAC_Y		( FLASH_SECTOR_BUFFER + 0x00000030 )
// Loop Gain Calibration X
#define	LOOP_GAIN_VALUE_X		( FLASH_SECTOR_BUFFER + 0x00000034 )
// Loop Gain Calibration Y
#define	LOOP_GAIN_VALUE_Y		( FLASH_SECTOR_BUFFER + 0x00000038 )
// Lens Center Calibration
#define	LENS_CENTER_VALUE_X		( FLASH_SECTOR_BUFFER + 0x0000003C )
#define	LENS_CENTER_VALUE_Y		( FLASH_SECTOR_BUFFER + 0x00000040 )
// Optical Center Calibration
#define	OPT_CENTER_VALUE_X		( FLASH_SECTOR_BUFFER + 0x00000044 )
#define	OPT_CENTER_VALUE_Y		( FLASH_SECTOR_BUFFER + 0x00000048 )
// Gyro Offset Calibration
#define	GYRO_OFFSET_VALUE_X		( FLASH_SECTOR_BUFFER + 0x0000004C )
#define	GYRO_OFFSET_VALUE_Y		( FLASH_SECTOR_BUFFER + 0x00000050 )
// Gyro Gain Calibration
#define	GYRO_GAIN_VALUE_X		( FLASH_SECTOR_BUFFER + 0x00000054 )
#define	GYRO_GAIN_VALUE_Y		( FLASH_SECTOR_BUFFER + 0x00000058 )
#define HALL_BIAS_DAC_AF		( FLASH_SECTOR_BUFFER + 0x0000005C )
#define HALL_OFFSET_DAC_AF		( FLASH_SECTOR_BUFFER + 0x00000060 )
#define LOOP_GAIN_VALUE_AF		( FLASH_SECTOR_BUFFER + 0x00000064 )
#define LENS_CENTER_VALUE_AF	( FLASH_SECTOR_BUFFER + 0x00000068 )
#define MAGNIFICATION_AF		( FLASH_SECTOR_BUFFER + 0x0000006C )

#define	HALL_MAX_BEFORE_Z		( FLASH_SECTOR_BUFFER + 0x00000070 )
#define	HALL_MIN_BEFORE_Z		( FLASH_SECTOR_BUFFER + 0x00000074 )
#define	HALL_MAX_AFTER_Z		( FLASH_SECTOR_BUFFER + 0x00000078 )
#define	HALL_MIN_AFTER_Z		( FLASH_SECTOR_BUFFER + 0x0000007C )
#define MIDDLE_POS_AF_CODE		( FLASH_SECTOR_BUFFER + 0x00000080 )

#define RESERVE_AREA_TOP		( FLASH_SECTOR_BUFFER + 0x00000090 )
// User area data
#define POS1_X					( RESERVE_AREA_TOP + 0x00000000 )
#define POS1_Y					( RESERVE_AREA_TOP + 0x00000002 )
#define SIZE1_X					( RESERVE_AREA_TOP + 0x00000004 )
#define SIZE1_Y					( RESERVE_AREA_TOP + 0x00000006 )
#define POS4_X					( RESERVE_AREA_TOP + 0x00000008 )
#define POS4_Y					( RESERVE_AREA_TOP + 0x0000000A )
#define SIZE4_X					( RESERVE_AREA_TOP + 0x0000000C )
#define SIZE4_Y					( RESERVE_AREA_TOP + 0x0000000E )
#define POS7_X					( RESERVE_AREA_TOP + 0x00000010 )
#define POS7_Y					( RESERVE_AREA_TOP + 0x00000012 )
#define SIZE7_X					( RESERVE_AREA_TOP + 0x00000014 )
#define SIZE7_Y					( RESERVE_AREA_TOP + 0x00000016 )
#define DISTANCE_X				( RESERVE_AREA_TOP + 0x00000018 )
#define DISTANCE_Y				( RESERVE_AREA_TOP + 0x0000001A )
#define CORRELATION_X			( RESERVE_AREA_TOP + 0x0000001C )
#define CORRELATION_Y			( RESERVE_AREA_TOP + 0x0000001E )

#define USER_AREA_TOP			( FLASH_SECTOR_BUFFER + 0x000000B0 )
#ifdef	HF_LINEAR_ENA
#define	LN_ZONE1_COEFA			( FLASH_SECTOR_BUFFER + 0x000000B0 )
#define	LN_ZONE2_COEFA			( FLASH_SECTOR_BUFFER + 0x000000B4 )
#define	LN_ZONE3_COEFA			( FLASH_SECTOR_BUFFER + 0x000000B8 )
#define	LN_ZONE4_COEFA			( FLASH_SECTOR_BUFFER + 0x000000BC )
#define	LN_ZONE5_COEFA			( FLASH_SECTOR_BUFFER + 0x000000C0 )
#define	LN_ZONE6_COEFA			( FLASH_SECTOR_BUFFER + 0x000000C4 )

#define	LN_ZONE1_COEFB			( FLASH_SECTOR_BUFFER + 0x000000C8 )
#define	LN_ZONE2_COEFB			( FLASH_SECTOR_BUFFER + 0x000000CC )
#define	LN_ZONE3_COEFB			( FLASH_SECTOR_BUFFER + 0x000000D0 )
#define	LN_ZONE4_COEFB			( FLASH_SECTOR_BUFFER + 0x000000D4 )
#define	LN_ZONE5_COEFB			( FLASH_SECTOR_BUFFER + 0x000000D8 )
#define	LN_ZONE6_COEFB			( FLASH_SECTOR_BUFFER + 0x000000DC )

#define	LN_ZONE1_THR			( FLASH_SECTOR_BUFFER + 0x000000E0 )
#define	LN_ZONE2_THR			( FLASH_SECTOR_BUFFER + 0x000000E4 )
#define	LN_ZONE3_THR			( FLASH_SECTOR_BUFFER + 0x000000E8 )
#define	LN_ZONE4_THR			( FLASH_SECTOR_BUFFER + 0x000000EC )
#define	LN_ZONE5_THR			( FLASH_SECTOR_BUFFER + 0x000000F0 )
#endif	// HF_LINEAR_ENA

#define USER_AREA_END			( FLASH_SECTOR_BUFFER + 0x000000FF )
//==============================================================================
//DMA
//==============================================================================
#define 	CmdCommand						0x0028
#define 	CmdEventCommand					0x002C
#define		SinWaveC						0x00b8
#define			SinWaveC_Pt						0x0000 + SinWaveC
#define			SinWaveC_Regsiter				0x0004 + SinWaveC_Pt
#define			SinWaveC_SignFlag				0x0004 + SinWaveC_Regsiter

#define		SinWave							0x00c4
				// SinGenerator.h SinWave_t
#define			SinWave_Offset					0x0000 + SinWave
#define			SinWave_Phase					0x0004 + SinWave_Offset
#define			SinWave_Gain					0x0004 + SinWave_Phase
#define			SinWave_Output					0x0004 + SinWave_Gain
#define			SinWave_OutAddr					0x0004 + SinWave_Output
#define		CosWave							0x00d8
				// SinGenerator.h SinWave_t
#define			CosWave_Offset					0x0000 + CosWave
#define			CosWave_Phase					0x0004 + CosWave_Offset
#define			CosWave_Gain					0x0004 + CosWave_Phase
#define			CosWave_Output					0x0004 + CosWave_Gain
#define			CosWave_OutAddr					0x0004 + CosWave_Output
#define		GYRO_RAM_COMMON					0x00ec
				// GyroFilterDelay.h GYRO_RAM_COMMON_t
#define			GYRO_RAM_GX_ADIDAT				0x0000 + GYRO_RAM_COMMON
#define			GYRO_RAM_GY_ADIDAT				0x0004 + GYRO_RAM_GX_ADIDAT
#define			GYRO_RAM_SINDX					0x0004 + GYRO_RAM_GY_ADIDAT
#define			GYRO_RAM_SINDY					0x0004 + GYRO_RAM_SINDX
#define			GYRO_RAM_GXLENSZ				0x0004 + GYRO_RAM_SINDY
#define			GYRO_RAM_GYLENSZ				0x0004 + GYRO_RAM_GXLENSZ
#define			GYRO_RAM_GXOX_OUT				0x0004 + GYRO_RAM_GYLENSZ
#define			GYRO_RAM_GYOX_OUT				0x0004 + GYRO_RAM_GXOX_OUT
#define			GYRO_RAM_GXOFFZ					0x0004 + GYRO_RAM_GYOX_OUT
#define			GYRO_RAM_GYOFFZ					0x0004 + GYRO_RAM_GXOFFZ
#define			GYRO_RAM_GYRO_LimitX			0x0004 + GYRO_RAM_GYOFFZ
#define			GYRO_RAM_GYRO_LimitY			0x0004 + GYRO_RAM_GYRO_LimitX
#define			GYRO_RAM_GYROX_AFCnt			0x0004 + GYRO_RAM_GYRO_LimitY
#define			GYRO_RAM_GYROY_AFCnt			0x0004 + GYRO_RAM_GYROX_AFCnt
#define			GYRO_RAM_GYRO_Switch			0x0004 + GYRO_RAM_GYROY_AFCnt		// 1Byte
#define			GYRO_RAM_GYRO_AF_Switch			0x0001 + GYRO_RAM_GYRO_Switch		// 1Byte
#define		GYRO_RAM_X						0x0128
				// GyroFilterDelay.h GYRO_RAM_t
#define			GYRO_RAM_GYROX_OFFSET			0x0000 + GYRO_RAM_X
#define			GYRO_RAM_GX2X4XF_IN				0x0004 + GYRO_RAM_GYROX_OFFSET
#define			GYRO_RAM_GX2X4XF_OUT			0x0004 + GYRO_RAM_GX2X4XF_IN
#define			GYRO_RAM_GXFAST					0x0004 + GYRO_RAM_GX2X4XF_OUT
#define			GYRO_RAM_GXSLOW					0x0004 + GYRO_RAM_GXFAST
#define			GYRO_RAM_GYROX_G1OUT			0x0004 + GYRO_RAM_GXSLOW
#define			GYRO_RAM_GYROX_G2OUT			0x0004 + GYRO_RAM_GYROX_G1OUT
#define			GYRO_RAM_GYROX_G3OUT			0x0004 + GYRO_RAM_GYROX_G2OUT
#define			GYRO_RAM_GYROX_OUT				0x0004 + GYRO_RAM_GYROX_G3OUT
#define		GYRO_RAM_Y						0x014c
				// GyroFilterDelay.h GYRO_RAM_t
#define			GYRO_RAM_GYROY_OFFSET			0x0000 + GYRO_RAM_Y
#define			GYRO_RAM_GY2X4XF_IN				0x0004 + GYRO_RAM_GYROY_OFFSET
#define			GYRO_RAM_GY2X4XF_OUT			0x0004 + GYRO_RAM_GY2X4XF_IN
#define			GYRO_RAM_GYFAST					0x0004 + GYRO_RAM_GY2X4XF_OUT
#define			GYRO_RAM_GYSLOW					0x0004 + GYRO_RAM_GYFAST
#define			GYRO_RAM_GYROY_G1OUT			0x0004 + GYRO_RAM_GYSLOW
#define			GYRO_RAM_GYROY_G2OUT			0x0004 + GYRO_RAM_GYROY_G1OUT
#define			GYRO_RAM_GYROY_G3OUT			0x0004 + GYRO_RAM_GYROY_G2OUT
#define			GYRO_RAM_GYROY_OUT				0x0004 + GYRO_RAM_GYROY_G3OUT
#define		GyroFilterDelayX				0x0170
#define		GyroFilterDelayX_GXH1Z2			0x0184	// delay3[1]
#define		GyroFilterDelayY				0x0198
#define		GyroFilterDelayY_GYH1Z2			0x01ac	// delay3[1]
#define		HALL_RAM_COMMON					0x01c0
				//  HallFilterDelay.h HALL_RAM_COMMON_t
#define			HALL_RAM_HXIDAT					0x0000 + HALL_RAM_COMMON
#define			HALL_RAM_HYIDAT					0x0004 + HALL_RAM_HXIDAT
#define			HALL_RAM_GYROX_OUT				0x0004 + HALL_RAM_HYIDAT
#define			HALL_RAM_GYROY_OUT				0x0004 + HALL_RAM_GYROX_OUT
#define		HallFilterDelayX				0x01d0
#define			HallFilterD_HXDAZ1				0x0010 + HallFilterDelayX
#define		HallFilterDelayY				0x0220
#define			HallFilterD_HYDAZ1				0x0010 + HallFilterDelayY
#define		HALL_RAM_X						0x0270
				//  HallFilterDelay.h HALL_RAM_t
#define			HALL_RAM_HXOFF					0x0000 + HALL_RAM_X
#define			HALL_RAM_HXOFF1					0x0004 + HALL_RAM_HXOFF
#define			HALL_RAM_HXOUT0					0x0004 + HALL_RAM_HXOFF1
#define			HALL_RAM_HXOUT1					0x0004 + HALL_RAM_HXOUT0
#define			HALL_RAM_SINDX0					0x0004 + HALL_RAM_HXOUT1
#define			HALL_RAM_HXLOP					0x0004 + HALL_RAM_SINDX0
#define			HALL_RAM_SINDX1					0x0004 + HALL_RAM_HXLOP
#define			HALL_RAM_HALL_X_OUT				0x0004 + HALL_RAM_SINDX1
					// MoveAvg	2ch
#define		HALL_RAM_HALL_SwitchX			0x02bc
#define		HALL_RAM_Y						0x02c0
				//  HallFilterDelay.h HALL_RAM_t
#define			HALL_RAM_HYOFF					0x0000 + HALL_RAM_Y
#define			HALL_RAM_HYOFF1					0x0004 + HALL_RAM_HYOFF
#define			HALL_RAM_HYOUT0					0x0004 + HALL_RAM_HYOFF1
#define			HALL_RAM_HYOUT1					0x0004 + HALL_RAM_HYOUT0
#define			HALL_RAM_SINDY0					0x0004 + HALL_RAM_HYOUT1
#define			HALL_RAM_HYLOP					0x0004 + HALL_RAM_SINDY0
#define			HALL_RAM_SINDY1					0x0004 + HALL_RAM_HYLOP
#define			HALL_RAM_HALL_Y_OUT				0x0004 + HALL_RAM_SINDY1
					// MoveAvg	2ch
#define		HALL_RAM_HALL_SwitchY			0x030c

#define		HXOFF1							0x0310
#define		HYOFF1							0x0314
#define		PanTilt_DMA						0x0318
#define			PanTilt_DMA_ScTpdSts			0x000C + PanTilt_DMA
#define		RdWrCalibrationData_Req			0x0328
#define		FlashRdWr_Req					0x032c
#define		CLAF_CNT						0x0330
#define		OLAF_IF							0x0330
#define		PMEMCMD							0x0330
#define			PMEMCMD_CONTROL					0x0000 + PMEMCMD
#define			PMEMCMD_LENGTH					0x0004 + PMEMCMD_CONTROL
#define			PMEMCMD_ADDR					0x0004 + PMEMCMD_LENGTH
#define			PMEMCMD_DATA					0x0004 + PMEMCMD_ADDR
#define		FROMCMD							0x0350
#define		OLAF_DMA						0x0354
#define		OLAF_GPCOM						0x03AC
#define		CLAF_RAMA						0x03C0
#define			CLAF_RAMA_AFCNT					0x0000 + CLAF_RAMA
#define			CLAF_RAMA_AFADIN				0x0004 + CLAF_RAMA_AFCNT
#define			CLAF_RAMA_AFOFFSET				0x0004 + CLAF_RAMA_AFADIN
#define			CLAF_RAMA_AFADOFF				0x0004 + CLAF_RAMA_AFOFFSET
#define			CLAF_RAMA_AFLOP1				0x0004 + CLAF_RAMA_AFADOFF
#define			CLAF_RAMA_AFTARGET				0x0004 + CLAF_RAMA_AFLOP1
#define			CLAF_RAMA_AFDEV					0x0004 + CLAF_RAMA_AFTARGET
#define			CLAF_RAMA_HLDINT				0x0004 + CLAF_RAMA_AFDEV
#define			CLAF_RAMA_AFLOP2				0x0004 + CLAF_RAMA_HLDINT
#define			CLAF_RAMA_AFBRAKE				0x0004 + CLAF_RAMA_AFLOP2
#define			CLAF_RAMA_AFKICK				0x0004 + CLAF_RAMA_AFBRAKE
#define			CLAF_RAMA_AFD1Z1				0x0004 + CLAF_RAMA_AFKICK
#define			CLAF_RAMA_AFSPDJG				0x0004 + CLAF_RAMA_AFD1Z1
#define			CLAF_RAMA_AFKBOUT				0x0004 + CLAF_RAMA_AFSPDJG
#define			CLAF_RAMA_AFCNTO				0x0004 + CLAF_RAMA_AFKBOUT
#define			CLAF_RAMA_AFOUT					0x0004 + CLAF_RAMA_AFCNTO
#define		CLAF_DELAY						0x0400
#define			CLAF_DELAY_AFDZ0				0x0000 + CLAF_DELAY
#define			CLAF_DELAY_AFDZ1				0x0004 + CLAF_DELAY_AFDZ0
#define			CLAF_DELAY_AFIZ0				0x0004 + CLAF_DELAY_AFDZ1
#define			CLAF_DELAY_AFIZ1				0x0004 + CLAF_DELAY_AFIZ0
#define			CLAF_DELAY_AFPZ0				0x0004 + CLAF_DELAY_AFIZ1
#define			CLAF_DELAY_AFPZ1				0x0004 + CLAF_DELAY_AFPZ0
#define			CLAF_DELAY_AFPZ2				0x0004 + CLAF_DELAY_AFPZ1
#define			CLAF_DELAY_AFPZ3				0x0004 + CLAF_DELAY_AFPZ2
#define			CLAF_DELAY_AFOZ0				0x0004 + CLAF_DELAY_AFPZ3
#define			CLAF_DELAY_AFOZ1				0x0004 + CLAF_DELAY_AFOZ0
#define		WaitTimerData					0x0468
				// CommonLibrary.h  WaitTimer_Type
#define			WaitTimerData_UiWaitCounter		0x0000 + WaitTimerData
#define			WaitTimerData_UiTargetCount		0x0004 + WaitTimerData_UiWaitCounter
#define			WaitTimerData_UiBaseCount		0x0004 + WaitTimerData_UiTargetCount
#define		StMeasureFunc					0x0478
				// MeasureFilter.h	MeasureFunction_Type
#define			StMeasFunc_SiSampleNum			0x0000 + StMeasureFunc
#define			StMeasFunc_SiSampleMax			0x0004 + StMeasFunc_SiSampleNum
#define		StMeasFunc_MFA					0x0480
#define			StMeasFunc_MFA_SiMax1			0x0000 + StMeasFunc_MFA
#define			StMeasFunc_MFA_SiMin1			0x0004 + StMeasFunc_MFA_SiMax1
#define			StMeasFunc_MFA_UiAmp1			0x0004 + StMeasFunc_MFA_SiMin1
#define			StMeasFunc_MFA_UiDUMMY1			0x0004 + StMeasFunc_MFA_UiAmp1
#define			StMeasFunc_MFA_LLiIntegral1		0x0004 + StMeasFunc_MFA_UiDUMMY1
#define			StMeasFunc_MFA_LLiAbsInteg1		0x0008 + StMeasFunc_MFA_LLiIntegral1
#define			StMeasFunc_MFA_PiMeasureRam1	0x0008 + StMeasFunc_MFA_LLiAbsInteg1
#define		StMeasFunc_MFB					0x04A8
#define			StMeasFunc_MFB_SiMax2			0x0000 + StMeasFunc_MFB
#define			StMeasFunc_MFB_SiMin2			0x0004 + StMeasFunc_MFB_SiMax2
#define			StMeasFunc_MFB_UiAmp2			0x0004 + StMeasFunc_MFB_SiMin2
#define			StMeasFunc_MFB_UiDUMMY1			0x0004 + StMeasFunc_MFB_UiAmp2
#define			StMeasFunc_MFB_LLiIntegral2		0x0004 + StMeasFunc_MFB_UiDUMMY1
#define			StMeasFunc_MFB_LLiAbsInteg2		0x0008 + StMeasFunc_MFB_LLiIntegral2
#define			StMeasFunc_MFB_PiMeasureRam2	0x0008 + StMeasFunc_MFB_LLiAbsInteg2
#define		MeasureFilterA_Delay			0x04D0
				// MeasureFilter.h	MeasureFilter_Delay_Type
#define			MeasureFilterA_Delay_z11		0x0000 + MeasureFilterA_Delay
#define			MeasureFilterA_Delay_z12		0x0004 + MeasureFilterA_Delay_z11
#define			MeasureFilterA_Delay_z21		0x0004 + MeasureFilterA_Delay_z12
#define			MeasureFilterA_Delay_z22		0x0004 + MeasureFilterA_Delay_z21
#define		MeasureFilterB_Delay			0x04E0
				// MeasureFilter.h	MeasureFilter_Delay_Type
#define			MeasureFilterB_Delay_z11		0x0000 + MeasureFilterB_Delay
#define			MeasureFilterB_Delay_z12		0x0004 + MeasureFilterB_Delay_z11
#define			MeasureFilterB_Delay_z21		0x0004 + MeasureFilterB_Delay_z12
#define			MeasureFilterB_Delay_z22		0x0004 + MeasureFilterB_Delay_z21
#define		MeasureParameter				0x04F0
#define		StCalibrationData				0x04FC
				// Calibration.h  CalibrationData_Type
#define			StCaliData_UsCalibrationStatus	0x0000 + StCalibrationData
#define			StCaliData_SiHallMax_Before_X	0x0004 + StCaliData_UsCalibrationStatus
#define			StCaliData_SiHallMin_Before_X	0x0004 + StCaliData_SiHallMax_Before_X
#define			StCaliData_SiHallMax_After_X	0x0004 + StCaliData_SiHallMin_Before_X
#define			StCaliData_SiHallMin_After_X	0x0004 + StCaliData_SiHallMax_After_X
#define			StCaliData_SiHallMax_Before_Y	0x0004 + StCaliData_SiHallMin_After_X
#define			StCaliData_SiHallMin_Before_Y	0x0004 + StCaliData_SiHallMax_Before_Y
#define			StCaliData_SiHallMax_After_Y	0x0004 + StCaliData_SiHallMin_Before_Y
#define			StCaliData_SiHallMin_After_Y	0x0004 + StCaliData_SiHallMax_After_Y
#define			StCaliData_UiHallBias_X			0x0004 + StCaliData_SiHallMin_After_Y
#define			StCaliData_UiHallOffset_X		0x0004 + StCaliData_UiHallBias_X
#define			StCaliData_UiHallBias_Y			0x0004 + StCaliData_UiHallOffset_X
#define			StCaliData_UiHallOffset_Y		0x0004 + StCaliData_UiHallBias_Y
#define			StCaliData_SiLoopGain_X			0x0004 + StCaliData_UiHallOffset_Y
#define			StCaliData_SiLoopGain_Y			0x0004 + StCaliData_SiLoopGain_X
#define			StCaliData_SiLensCen_Offset_X	0x0004 + StCaliData_SiLoopGain_Y
#define			StCaliData_SiLensCen_Offset_Y	0x0004 + StCaliData_SiLensCen_Offset_X
#define			StCaliData_SiOtpCen_Offset_X	0x0004 + StCaliData_SiLensCen_Offset_Y
#define			StCaliData_SiOtpCen_Offset_Y	0x0004 + StCaliData_SiOtpCen_Offset_X
#define			StCaliData_SiGyroOffset_X		0x0004 + StCaliData_SiOtpCen_Offset_Y
#define			StCaliData_SiGyroOffset_Y		0x0004 + StCaliData_SiGyroOffset_X
#define			StCaliData_SiGyroGain_X			0x0004 + StCaliData_SiGyroOffset_Y
#define			StCaliData_SiGyroGain_Y			0x0004 + StCaliData_SiGyroGain_X
#define			StCaliData_UiHallBias_AF		0x0004 + StCaliData_SiGyroGain_Y
#define			StCaliData_UiHallOffset_AF		0x0004 + StCaliData_UiHallBias_AF
#define			StCaliData_SiLoopGain_AF		0x0004 + StCaliData_UiHallOffset_AF
#define			StCaliData_SiAD_Offset_AF		0x0004 + StCaliData_SiLoopGain_AF
#define			StCaliData_SiMagnification_AF	0x0004 + StCaliData_SiAD_Offset_AF
#define		StCalibDataAdd					0x056C
#define			StCalDatAd_SiHallMax_Before_Z	0x0000 + StCalibDataAdd
#define			StCalDatAd_SiHallMin_Before_Z	0x0004 + StCalibDataAdd
#define			StCalDatAd_SiHallMax_After_Z	0x0008 + StCalibDataAdd
#define			StCalDatAd_SiHallMin_After_Z	0x000C + StCalibDataAdd
#define		FW_Updata_Req					0x057C
#define		ReturnToCenter_Req				0x0580
#define		PanTilt_Req						0x0584
#define		OIS_Enable_Req					0x0588
#define		MovieStil_Mode_Req				0x058C
#define		Calibration_Req					0x0590
#define		ChaseConfirmation_Req			0x0594
#define		GyroSignalConfirmation_Req		0x0598
#define		CarrierFreqMode_Req				0x059C
#define		AfTargetCode_Req				0x05a0
#define		UiMesCnt						0x05a4
#define		UiBefVlX						0x05a8
#define		UiBefVlY						0x05ac
#define		GyroTemporaryRam				0x05b0
#define		RequestTable					0x05b8
#define		OverData						0x05ec



//==============================================================================
//DMB
//==============================================================================
#define		SiVerNum						0x8000
#define		WaveCoeff						0x8004
#define		MeasureFilterA_Coeff			0x80a4
				// MeasureFilter.h  MeasureFilter_Type
#define			MeasureFilterA_Coeff_b1			0x0000 + MeasureFilterA_Coeff
#define			MeasureFilterA_Coeff_c1			0x0004 + MeasureFilterA_Coeff_b1
#define			MeasureFilterA_Coeff_a1			0x0004 + MeasureFilterA_Coeff_c1
#define			MeasureFilterA_Coeff_b2			0x0004 + MeasureFilterA_Coeff_a1
#define			MeasureFilterA_Coeff_c2			0x0004 + MeasureFilterA_Coeff_b2
#define			MeasureFilterA_Coeff_a2			0x0004 + MeasureFilterA_Coeff_c2
#define		MeasureFilterB_Coeff			0x80bc
				// MeasureFilter.h  MeasureFilter_Type
#define			MeasureFilterB_Coeff_b1			0x0000 + MeasureFilterB_Coeff
#define			MeasureFilterB_Coeff_c1			0x0004 + MeasureFilterB_Coeff_b1
#define			MeasureFilterB_Coeff_a1			0x0004 + MeasureFilterB_Coeff_c1
#define			MeasureFilterB_Coeff_b2			0x0004 + MeasureFilterB_Coeff_a1
#define			MeasureFilterB_Coeff_c2			0x0004 + MeasureFilterB_Coeff_b2
#define			MeasureFilterB_Coeff_a2			0x0004 + MeasureFilterB_Coeff_c2
#define		ctr_initDa_InitData				0x80d4
#define		PWM_Coeff						0x80e4
#define		ASCII_Table						0x80e8
#define		CommandDeocdeTable				0x80f8
#define		HallFilterCoeffX				0x8138
				// HallFilterCoeff.h  DM_HFC_t
#define			HallFilterCoeffX_HXIGAIN		0x0000 + HallFilterCoeffX
#define			HallFilterCoeffX_GYROXOUTGAIN	0x0004 + HallFilterCoeffX_HXIGAIN
#define			HallFilterCoeffX_HXOFFGAIN		0x0004 + HallFilterCoeffX_GYROXOUTGAIN

#define			HallFilterCoeffX_hxiab			0x0004 + HallFilterCoeffX_HXOFFGAIN
#define			HallFilterCoeffX_hxiac			0x0004 + HallFilterCoeffX_hxiab
#define			HallFilterCoeffX_hxiaa			0x0004 + HallFilterCoeffX_hxiac
#define			HallFilterCoeffX_hxibb			0x0004 + HallFilterCoeffX_hxiaa
#define			HallFilterCoeffX_hxibc			0x0004 + HallFilterCoeffX_hxibb
#define			HallFilterCoeffX_hxiba			0x0004 + HallFilterCoeffX_hxibc
#define			HallFilterCoeffX_hxdab			0x0004 + HallFilterCoeffX_hxiba
#define			HallFilterCoeffX_hxdac			0x0004 + HallFilterCoeffX_hxdab
#define			HallFilterCoeffX_hxdaa			0x0004 + HallFilterCoeffX_hxdac
#define			HallFilterCoeffX_hxdbb			0x0004 + HallFilterCoeffX_hxdaa
#define			HallFilterCoeffX_hxdbc			0x0004 + HallFilterCoeffX_hxdbb
#define			HallFilterCoeffX_hxdba			0x0004 + HallFilterCoeffX_hxdbc
#define			HallFilterCoeffX_hxdcb			0x0004 + HallFilterCoeffX_hxdba
#define			HallFilterCoeffX_hxdcc			0x0004 + HallFilterCoeffX_hxdcb
#define			HallFilterCoeffX_hxdca			0x0004 + HallFilterCoeffX_hxdcc
#define			HallFilterCoeffX_hxpgain0		0x0004 + HallFilterCoeffX_hxdca
#define			HallFilterCoeffX_hxigain0		0x0004 + HallFilterCoeffX_hxpgain0
#define			HallFilterCoeffX_hxdgain0		0x0004 + HallFilterCoeffX_hxigain0
#define			HallFilterCoeffX_hxpgain1		0x0004 + HallFilterCoeffX_hxdgain0
#define			HallFilterCoeffX_hxigain1		0x0004 + HallFilterCoeffX_hxpgain1
#define			HallFilterCoeffX_hxdgain1		0x0004 + HallFilterCoeffX_hxigain1
#define			HallFilterCoeffX_hxgain0		0x0004 + HallFilterCoeffX_hxdgain1
#define			HallFilterCoeffX_hxgain1		0x0004 + HallFilterCoeffX_hxgain0

#define			HallFilterCoeffX_hxsb			0x0004 + HallFilterCoeffX_hxgain1
#define			HallFilterCoeffX_hxsc			0x0004 + HallFilterCoeffX_hxsb
#define			HallFilterCoeffX_hxsa			0x0004 + HallFilterCoeffX_hxsc

#define			HallFilterCoeffX_hxob			0x0004 + HallFilterCoeffX_hxsa
#define			HallFilterCoeffX_hxoc			0x0004 + HallFilterCoeffX_hxob
#define			HallFilterCoeffX_hxod			0x0004 + HallFilterCoeffX_hxoc
#define			HallFilterCoeffX_hxoe			0x0004 + HallFilterCoeffX_hxod
#define			HallFilterCoeffX_hxoa			0x0004 + HallFilterCoeffX_hxoe
#define			HallFilterCoeffX_hxpb			0x0004 + HallFilterCoeffX_hxoa
#define			HallFilterCoeffX_hxpc			0x0004 + HallFilterCoeffX_hxpb
#define			HallFilterCoeffX_hxpd			0x0004 + HallFilterCoeffX_hxpc
#define			HallFilterCoeffX_hxpe			0x0004 + HallFilterCoeffX_hxpd
#define			HallFilterCoeffX_hxpa			0x0004 + HallFilterCoeffX_hxpe
#define		HallFilterCoeffY				0x81d4
				// HallFilterCoeff.h  DM_HFC_t
#define			HallFilterCoeffY_HYIGAIN		0x0000 + HallFilterCoeffY
#define			HallFilterCoeffY_GYROYOUTGAIN	0x0004 + HallFilterCoeffY_HYIGAIN
#define			HallFilterCoeffY_HYOFFGAIN		0x0004 + HallFilterCoeffY_GYROYOUTGAIN

#define			HallFilterCoeffY_hyiab			0x0004 + HallFilterCoeffY_HYOFFGAIN
#define			HallFilterCoeffY_hyiac			0x0004 + HallFilterCoeffY_hyiab
#define			HallFilterCoeffY_hyiaa			0x0004 + HallFilterCoeffY_hyiac
#define			HallFilterCoeffY_hyibb			0x0004 + HallFilterCoeffY_hyiaa
#define			HallFilterCoeffY_hyibc			0x0004 + HallFilterCoeffY_hyibb
#define			HallFilterCoeffY_hyiba			0x0004 + HallFilterCoeffY_hyibc
#define			HallFilterCoeffY_hydab			0x0004 + HallFilterCoeffY_hyiba
#define			HallFilterCoeffY_hydac			0x0004 + HallFilterCoeffY_hydab
#define			HallFilterCoeffY_hydaa			0x0004 + HallFilterCoeffY_hydac
#define			HallFilterCoeffY_hydbb			0x0004 + HallFilterCoeffY_hydaa
#define			HallFilterCoeffY_hydbc			0x0004 + HallFilterCoeffY_hydbb
#define			HallFilterCoeffY_hydba			0x0004 + HallFilterCoeffY_hydbc
#define			HallFilterCoeffY_hydcb			0x0004 + HallFilterCoeffY_hydba
#define			HallFilterCoeffY_hydcc			0x0004 + HallFilterCoeffY_hydcb
#define			HallFilterCoeffY_hydca			0x0004 + HallFilterCoeffY_hydcc
#define			HallFilterCoeffY_hypgain0		0x0004 + HallFilterCoeffY_hydca
#define			HallFilterCoeffY_hyigain0		0x0004 + HallFilterCoeffY_hypgain0
#define			HallFilterCoeffY_hydgain0		0x0004 + HallFilterCoeffY_hyigain0
#define			HallFilterCoeffY_hypgain1		0x0004 + HallFilterCoeffY_hydgain0
#define			HallFilterCoeffY_hyigain1		0x0004 + HallFilterCoeffY_hypgain1
#define			HallFilterCoeffY_hydgain1		0x0004 + HallFilterCoeffY_hyigain1
#define			HallFilterCoeffY_hygain0		0x0004 + HallFilterCoeffY_hydgain1
#define			HallFilterCoeffY_hygain1		0x0004 + HallFilterCoeffY_hygain0
#define			HallFilterCoeffY_hysb			0x0004 + HallFilterCoeffY_hygain1
#define			HallFilterCoeffY_hysc			0x0004 + HallFilterCoeffY_hysb
#define			HallFilterCoeffY_hysa			0x0004 + HallFilterCoeffY_hysc
#define			HallFilterCoeffY_hyob			0x0004 + HallFilterCoeffY_hysa
#define			HallFilterCoeffY_hyoc			0x0004 + HallFilterCoeffY_hyob
#define			HallFilterCoeffY_hyod			0x0004 + HallFilterCoeffY_hyoc
#define			HallFilterCoeffY_hyoe			0x0004 + HallFilterCoeffY_hyod
#define			HallFilterCoeffY_hyoa			0x0004 + HallFilterCoeffY_hyoe
#define			HallFilterCoeffY_hypb			0x0004 + HallFilterCoeffY_hyoa
#define			HallFilterCoeffY_hypc			0x0004 + HallFilterCoeffY_hypb
#define			HallFilterCoeffY_hypd			0x0004 + HallFilterCoeffY_hypc
#define			HallFilterCoeffY_hype			0x0004 + HallFilterCoeffY_hypd
#define			HallFilterCoeffY_hypa			0x0004 + HallFilterCoeffY_hype
#define		HallFilterLimitX				0x8270
#define		HallFilterLimitY				0x8288
#define		HallFilterShiftX				0x82a0
#define		HallFilterShiftY				0x82a6
#define		GyroFilterTableX				0x82ac
				// GyroFilterCoeff.h  DM_GFC_t
#define			GyroFilterTableX_gx45x			0x0000 + GyroFilterTableX
#define			GyroFilterTableX_gx45y			0x0004 + GyroFilterTableX_gx45x
#define			GyroFilterTableX_gxgyro			0x0004 + GyroFilterTableX_gx45y
#define			GyroFilterTableX_gxsengen		0x0004 + GyroFilterTableX_gxgyro
#define			GyroFilterTableX_gxl1b			0x0004 + GyroFilterTableX_gxsengen
#define			GyroFilterTableX_gxl1c			0x0004 + GyroFilterTableX_gxl1b
#define			GyroFilterTableX_gxl1a			0x0004 + GyroFilterTableX_gxl1c
#define			GyroFilterTableX_gxl2b			0x0004 + GyroFilterTableX_gxl1a
#define			GyroFilterTableX_gxl2c			0x0004 + GyroFilterTableX_gxl2b
#define			GyroFilterTableX_gxl2a			0x0004 + GyroFilterTableX_gxl2c
#define			GyroFilterTableX_gxigain		0x0004 + GyroFilterTableX_gxl2a
#define			GyroFilterTableX_gxh1b			0x0004 + GyroFilterTableX_gxigain
#define			GyroFilterTableX_gxh1c			0x0004 + GyroFilterTableX_gxh1b
#define			GyroFilterTableX_gxh1a			0x0004 + GyroFilterTableX_gxh1c
#define			GyroFilterTableX_gxk1b			0x0004 + GyroFilterTableX_gxh1a
#define			GyroFilterTableX_gxk1c			0x0004 + GyroFilterTableX_gxk1b
#define			GyroFilterTableX_gxk1a			0x0004 + GyroFilterTableX_gxk1c
#define			GyroFilterTableX_gxgain			0x0004 + GyroFilterTableX_gxk1a
#define			GyroFilterTableX_gxzoom			0x0004 + GyroFilterTableX_gxgain
#define			GyroFilterTableX_gxlens			0x0004 + GyroFilterTableX_gxzoom
#define			GyroFilterTableX_gxt2b			0x0004 + GyroFilterTableX_gxlens
#define			GyroFilterTableX_gxt2c			0x0004 + GyroFilterTableX_gxt2b	
#define			GyroFilterTableX_gxt2a			0x0004 + GyroFilterTableX_gxt2c	
#define			GyroFilterTableX_afzoom			0x0004 + GyroFilterTableX_gxt2a
#define		GyroFilterTableY				0x830c
				// GyroFilterCoeff.h  DM_GFC_t
#define			GyroFilterTableY_gy45y			0x0000 + GyroFilterTableY
#define			GyroFilterTableY_gy45x			0x0004 + GyroFilterTableY_gy45y
#define			GyroFilterTableY_gygyro			0x0004 + GyroFilterTableY_gy45x
#define			GyroFilterTableY_gysengen		0x0004 + GyroFilterTableY_gygyro
#define			GyroFilterTableY_gyl1b			0x0004 + GyroFilterTableY_gysengen
#define			GyroFilterTableY_gyl1c			0x0004 + GyroFilterTableY_gyl1b
#define			GyroFilterTableY_gyl1a			0x0004 + GyroFilterTableY_gyl1c
#define			GyroFilterTableY_gyl2b			0x0004 + GyroFilterTableY_gyl1a
#define			GyroFilterTableY_gyl2c			0x0004 + GyroFilterTableY_gyl2b
#define			GyroFilterTableY_gyl2a			0x0004 + GyroFilterTableY_gyl2c
#define			GyroFilterTableY_gyigain		0x0004 + GyroFilterTableY_gyl2a
#define			GyroFilterTableY_gyh1b			0x0004 + GyroFilterTableY_gyigain
#define			GyroFilterTableY_gyh1c			0x0004 + GyroFilterTableY_gyh1b
#define			GyroFilterTableY_gyh1a			0x0004 + GyroFilterTableY_gyh1c
#define			GyroFilterTableY_gyk1b			0x0004 + GyroFilterTableY_gyh1a
#define			GyroFilterTableY_gyk1c			0x0004 + GyroFilterTableY_gyk1b
#define			GyroFilterTableY_gyk1a			0x0004 + GyroFilterTableY_gyk1c
#define			GyroFilterTableY_gygain			0x0004 + GyroFilterTableY_gyk1a
#define			GyroFilterTableY_gyzoom			0x0004 + GyroFilterTableY_gygain
#define			GyroFilterTableY_gylens			0x0004 + GyroFilterTableY_gyzoom
#define			GyroFilterTableY_gyt2b			0x0004 + GyroFilterTableY_gylens
#define			GyroFilterTableY_gyt2c			0x0004 + GyroFilterTableY_gyt2b
#define			GyroFilterTableY_gyt2a			0x0004 + GyroFilterTableY_gyt2c
#define			GyroFilterTableY_afzoom			0x0004 + GyroFilterTableY_gyt2a
#define		GF_LimitX						0x836c
				// GyroFilterCoeff.h  GF_Limit_t
#define			GF_LimitX_ILIMT					0x0000 + GF_LimitX
#define			GF_LimitX_JLIMT					0x0004 + GF_LimitX_ILIMT
#define			GF_LimitX_HLIMT					0x0004 + GF_LimitX_JLIMT

#define		GF_LimitY						0x8378
				// GyroFilterCoeff.h  GF_Limit_t
#define			GF_LimitY_ILIMT					0x0000 + GF_LimitY
#define			GF_LimitY_JLIMT					0x0004 + GF_LimitY_ILIMT
#define			GF_LimitY_HLIMT					0x0004 + GF_LimitY_JLIMT

#define		GF_ShiftX						0x8384
#define		GF_ShiftY						0x8388
#define		PanTilt_InitData				0x838c
#define		PanTilt_DMB						0x8394
#define		PanTiltCoeff					0x83b8
#define		PanTiltConstValue				0x8438
#if 0

#define		HighCmdDecodeTable				0x844C
#define		TARGETCNT						0x8488
 #define			TARGETCNT_SiSubVal				0x0000 + TARGETCNT
 #define			TARGETCNT_SiTgeVal				0x0004 + TARGETCNT_SiSubVal
#define		CURDRV_AF						0x8490
 #define			CURDRV_AF_SiPlsVal				0x0000 + CURDRV_AF
 #define			CURDRV_AF_SiMnsVal				0x0004 + CURDRV_AF
//#define		PWM_CARR						0x84A0
#define		CURDRV_X						0x84C0
 #define			CURDRV_X_SiPlsVal				0x0000 + CURDRV_X
 #define			CURDRV_X_SiMnsVal				0x0004 + CURDRV_X
#define		CURDRV_Y						0x84D0
 #define			CURDRV_Y_SiPlsVal				0x0000 + CURDRV_Y
 #define			CURDRV_Y_SiMnsVal				0x0004 + CURDRV_Y
#ifdef	HF_LINEAR_ENA
#define		HAL_LN_CORRECT					0x8518
#define			HAL_LN_COEFAX					0x0000 + HAL_LN_CORRECT
#endif	// HF_LINEAR_ENA

#else

#define		HighCmdDecodeTable				0x8534
#define		TARGETCNT						0x8570
 #define			TARGETCNT_SiSubVal				0x0000 + TARGETCNT
 #define			TARGETCNT_SiTgeVal				0x0004 + TARGETCNT_SiSubVal
#define		CURDRV_AF						0x8578
 #define			CURDRV_AF_SiPlsVal				0x0000 + CURDRV_AF
 #define			CURDRV_AF_SiMnsVal				0x0004 + CURDRV_AF
//#define		PWM_CARR						0x84A0
#define		CURDRV_X						0x85A8
 #define			CURDRV_X_SiPlsVal				0x0000 + CURDRV_X
 #define			CURDRV_X_SiMnsVal				0x0004 + CURDRV_X
#define		CURDRV_Y						0x85B8
 #define			CURDRV_Y_SiPlsVal				0x0000 + CURDRV_Y
 #define			CURDRV_Y_SiMnsVal				0x0004 + CURDRV_Y

#endif
//==============================================================================
//IO
//==============================================================================
#define SYSCONT_123		0xD00000	// System Control配置アドレス
#define 		SYS_DSP_REMAP						(SYSCONT_123 + 0xAC)

#define CVER_123		0xD00100	// LSI Version 読み出しアドレス

#define SOFTRESET		0xD0006C	// Soft reset setting
#define OSCRSEL			0xD00090	// OSC Frequency 1
#define OSCCURSEL		0xD00094	// OSC Frequency 2
#define IOPLEVR			0xD00104	// IO port level read

#define ADDA_123		0xD01000	// A/D & D/A I/F配置アドレス
				// LC898123_DMIO.h  ADDA_t
#define 		ADDA_reserve0_0						0x0000 + ADDA_123
#define 		ADDA_reserve0_1						0x0004 + ADDA_reserve0_0
#define 		ADDA_FSCTRL							0x0004 + ADDA_reserve0_1	
#define 		ADDA_ADDAINT						0x0004 + ADDA_FSCTRL		
#define 		ADDA_ADE							0x0004 + ADDA_ADDAINT	
#define 		ADDA_ADAV							0x0004 + ADDA_ADE		
#define 		ADDA_ADORDER						0x0004 + ADDA_ADAV		
#define 		ADDA_EXTEND							0x0004 + ADDA_ADORDER	
#define 		ADDA_AD0O							0x0004 + ADDA_EXTEND		
#define 		ADDA_AD1O							0x0004 + ADDA_AD0O		
#define 		ADDA_AD2O							0x0004 + ADDA_AD1O		
#define 		ADDA_AD3O							0x0004 + ADDA_AD2O		
#define 		ADDA_reserve1_0						0x0004 + ADDA_AD3O		
#define 		ADDA_reserve1_1						0x0004 + ADDA_reserve1_0	
#define 		ADDA_reserve1_2						0x0004 + ADDA_reserve1_1	
#define 		ADDA_reserve1_3						0x0004 + ADDA_reserve1_2	
#define 		ADDA_DASELW							0x0004 + ADDA_reserve1_3	
#define 		ADDA_DASU							0x0004 + ADDA_DASELW		
#define 		ADDA_DAHD							0x0004 + ADDA_DASU		
#define 		ADDA_DASWAP							0x0004 + ADDA_DAHD		
#define 		ADDA_DASEL							0x0004 + ADDA_DASWAP		
#define 		ADDA_DAO							0x0004 + ADDA_DASEL		
#if 0
#define PWM_123			0xD02000	// PWM I/F配置アドレス
#define PORT_123		0xE00000	// PORT I/F配置アドレス
#define TIMER_123		0xE01000	// TIMER I/F配置アドレス
#define SPIM_123		0xE02000	// SPI Master I/F配置アドレス
#define SPIS_123		0xE03000	// SPI Slave I/F配置アドレス
#define UART_123		0xE04000	// UART I/F配置アドレス
#define I2CS_123		0xE05000	// I2C Slave配置アドレス
#endif
#define FLASHROM_123	0xE07000	// Flash Memory I/F配置アドレス
#define 		FLASHROM_RDAT						(FLASHROM_123 + 0x00) 
#define 		FLASHROM_WDAT						(FLASHROM_123 + 0x04) 
#define 		FLASHROM_ADR						(FLASHROM_123 + 0x08) 
#define 		FLASHROM_ACSCNT						(FLASHROM_123 + 0x0C) 
#define 		FLASHROM_CMD						(FLASHROM_123 + 0x10) 
#define 		FLASHROM_WPB						(FLASHROM_123 + 0x14) 
#define 		FLASHROM_INT						(FLASHROM_123 + 0x18) 
#define 		FLASHROM_SEL						(FLASHROM_123 + 0x1C) 
#define 		FLASHROM_TRC						(FLASHROM_123 + 0x20) 
#define 		FLASHROM_TCFSH						(FLASHROM_123 + 0x04) 
#define 		FLASHROM_TCONFEN					(FLASHROM_123 + 0x28) 
#define 		FLASHROM_TNVS						(FLASHROM_123 + 0x2C) 
#define 		FLASHROM_TPGS						(FLASHROM_123 + 0x30) 
#define 		FLASHROM_TPROG						(FLASHROM_123 + 0x34) 
#define 		FLASHROM_TADSH						(FLASHROM_123 + 0x38) 
#define 		FLASHROM_TRCVP						(FLASHROM_123 + 0x3C) 
#define 		FLASHROM_TRCVS						(FLASHROM_123 + 0x40) 
#define 		FLASHROM_TRCVC						(FLASHROM_123 + 0x44) 
#define 		FLASHROM_TERASES					(FLASHROM_123 + 0x48) 
#define 		FLASHROM_TERASEC					(FLASHROM_123 + 0x4C) 
#define 		FLASHROM_TRW						(FLASHROM_123 + 0x50) 
