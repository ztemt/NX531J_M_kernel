//********************************************************************************
//		<< LC898123 Evaluation Soft>>
//		Program Name	: Ois.h
// 		Explanation		: LC898123 Global Declaration & ProtType Declaration
//		Design			: Y.Yamada
//		History			: First edition	
//********************************************************************************
//****************************************************
//	STRUCTURE 
//****************************************************
#define		INT16	short
#define		INT32	long
#define		INT64	long long
#define		UINT8	unsigned char
#define		UINT16	unsigned short
#define		UINT32	unsigned long
#define		UINT64	unsigned long long

#include "OisAPI.h"

#ifdef	__OIS_TYPE_XC__					// for LC898123XC
#include "OisLc898123XC.h"
#else
#include "OisLc898123AXD.h"
#endif



/**************** Model name *****************/
/**************** FW version *****************/
// #define	MDL_VER			0x01
 #define	FW_VER			0x05		//ATMEL Version
/**************** Select Mode **************/

#define		NEUTRAL_CENTER		// Upper Position Current 0mA Measurement
#define		NEUTRAL_CENTER_FINE		// Optimize natural center current
//#define		HF_LINEAR_ENA		// 

#define	LSIAXC			0xB4		// B4:123AXC
#define	LSIAXD			0xB6		// B6:123AXD
#define	DSPAXC			0x00		// AXC	model number of DSP
#define	DSPAXD			0x80		// AXD	model number of DSP

// Command Status
#define		EXE_END		0x00000002L		// Execute End (Adjust OK)
#define		EXE_HXADJ	0x00000006L		// Adjust NG : X Hall NG (Gain or Offset)
#define		EXE_HYADJ	0x0000000AL		// Adjust NG : Y Hall NG (Gain or Offset)
#define		EXE_LXADJ	0x00000012L		// Adjust NG : X Loop NG (Gain)
#define		EXE_LYADJ	0x00000022L		// Adjust NG : Y Loop NG (Gain)
#define		EXE_GXADJ	0x00000042L		// Adjust NG : X Gyro NG (offset)
#define		EXE_GYADJ	0x00000082L		// Adjust NG : Y Gyro NG (offset)
#define		EXE_ERR		0x00000099L		// Execute Error End
#ifdef	__OIS_CLOSED_AF__
 #define	EXE_HZADJ	0x00100002L		// Adjust NG : AF Hall NG (Gain or Offset)
 #define	EXE_LZADJ	0x00200002L		// Adjust NG : AF Loop NG (Gain)
#endif

#define		EXE_HXMVER	0x06		// X Err
#define		EXE_HYMVER	0x0A		// Y Err

// Gyro Examination of Acceptance
#define		EXE_GXABOVE	0x06		// X Above
#define		EXE_GXBELOW	0x0A		// X Below
#define		EXE_GYABOVE	0x12		// Y Above
#define		EXE_GYBELOW	0x22		// Y Below

// Common Define
#define	SUCCESS			0x00		// Success
#define	FAILURE			0x01		// Failure

#ifndef ON
 #define	ON				0x01		// ON
 #define	OFF				0x00		// OFF
#endif
#define	SPC				0x02		// Special Mode

#define	X_DIR			0x00		// X Direction
#define	Y_DIR			0x01		// Y Direction
#define	Z_DIR			0x02		// Z Direction(AF)

#define	WPB_OFF			0x01		// Write protect OFF
#define WPB_ON			0x00		// Write protect ON

#define	MD5_MAIN		0x01		// main array
#define MD5_NVR			0x02		// NVR
#define MD5_BOTH		(MD5_MAIN | MD5_NVR)

#define		SXGAIN_LOP		0x30000000
#define		SYGAIN_LOP		0x30000000
#define		XY_BIAS			0x40000000
#define		XY_OFST			0x80000000

#ifdef	__OIS_CLOSED_AF__
 #define	SZGAIN_LOP		0x30000000
 #define	Z_BIAS			0x40000000
 #define	Z_OFST			0x80000000
#endif

// mode
#define		GEA_MINMAX_MODE		0x00		// min, max mode
#define		GEA_MEAN_MODE		0x01		// mean mode
 
struct STFILREG {
	UINT16	UsRegAdd ;
	UINT8	UcRegDat ;
} ;													// Register Data Table

struct STFILRAM {
	UINT16	UsRamAdd ;
	UINT32	UlRamDat ;
} ;													// Filter Coefficient Table

struct STCMDTBL {
	UINT16	Cmd ;
	UINT32	UiCmdStf ;
	void ( *UcCmdPtr )( void ) ;
} ;

/************************************************/
/*	Command										*/
/************************************************/
#define		CMD_IO_ADR_ACCESS				0xC000				// IO Write Access
#define		CMD_IO_DAT_ACCESS				0xD000				// IO Read Access
#define		CMD_REMAP						0xF001				// Remap
#define		CMD_REBOOT						0xF003				// Reboot
#define		CMD_RETURN_TO_CENTER			0xF010				// Center Servo ON/OFF choose axis
	#define		BOTH_SRV_OFF					0x00000000			// Both   Servo OFF
	#define		XAXS_SRV_ON						0x00000001			// X axis Servo ON
	#define		YAXS_SRV_ON						0x00000002			// Y axis Servo ON
	#define		BOTH_SRV_ON						0x00000003			// Both   Servo ON
	#define		ZAXS_SRV_OFF					0x00000004			// Z axis Servo OFF
	#define		ZAXS_SRV_ON						0x00000005			// Z axis Servo ON
#define		CMD_PAN_TILT					0xF011				// Pan Tilt Enable/Disable
	#define		PAN_TILT_OFF					0x00000000			// Pan/Tilt OFF
	#define		PAN_TILT_ON						0x00000001			// Pan/Tilt ON
#define		CMD_OIS_ENABLE					0xF012				// Ois Enable/Disable
	#define		OIS_DISABLE						0x00000000			// OIS Disable
	#define		OIS_ENABLE						0x00000001			// OIS Enable
	#define		OIS_ENA_NCL						0x00000002			// OIS Enable ( none Delay clear )
	#define		OIS_ENA_DOF						0x00000004			// OIS Enable ( Drift offset exec )
#define		CMD_MOVE_STILL_MODE				0xF013				// Select mode
	#define		MOVIE_MODE						0x00000000			// Movie mode
	#define		STILL_MODE						0x00000001			// Still mode
	#define		MOVIE_MODE1						0x00000002			// Movie Preview mode 1
	#define		STILL_MODE1						0x00000003			// Still Preview mode 1
	#define		MOVIE_MODE2						0x00000004			// Movie Preview mode 2
	#define		STILL_MODE2						0x00000005			// Still Preview mode 2
	#define		MOVIE_MODE3						0x00000006			// Movie Preview mode 3
	#define		STILL_MODE3						0x00000007			// Still Preview mode 3
#define		CMD_CHASE_CONFIRMATION			0xF015				// Hall Chase confirmation
#define		CMD_GYRO_SIG_CONFIRMATION		0xF016				// Gyro Signal confirmation
#define		CMD_FLASH_LOAD					0xF017				// Flash Load
//#define		CMD_FLASH_STORE					0xF018				// Flash Write
	#define		HALL_CALB_FLG					0x00008000			
		#define		HALL_CALB_BIT					0x00FF00FF
	#define		GYRO_GAIN_FLG					0x00004000			
	#define		ANGL_CORR_FLG					0x00002000			
	#define		FOCL_GAIN_FLG					0x00001000			
	#define		CLAF_CALB_FLG					0x00000800			// CLAF Hall calibration
	#define		HLLN_CALB_FLG					0x00000400			// Hall linear calibration
#define		CMD_AF_POSITION					0xF01A				// AF Position
#define		CMD_GYRO_RD_ACCS				0xF01D				// Gyro Read Acess
#define		CMD_GYRO_WR_ACCS				0xF01E				// Gyro Write Acess

#define		CMD_READ_STATUS					0xF100				// Status Read

#define		READ_STATUS_INI					0x01000000

#define		STBOSCPLL						0x00D00074			// STB OSC
	#define		OSC_STB							0x00000002			// OSC standby

/**************************************************** *************************************/
// GyroFilterDefine.h *******************************************************************
#define CmEqSw1			0			// Select AD input(0: Off, 1: On)
#define CmEqSw2			1			// Select Sin wave input(0: Off, 1: On)
#define CmShakeOn		2			// Setting image stabilization enable(0: Off, 1: On)
#define CmRecMod		4			// Recording Mode(0: Off, 1: On)
#define CmCofCnt		5			// Coefficient setting(0: Off, 1: On)
#define CmTpdCnt		6			// Tripod Cntrol(0: Off, 1: On)
#define CmIntDrft		7			// Drift Integral Subtraction(0: Off, 1: On)
#define CmAfZoom		0			// AF Zoom Control



// Calibration.h *******************************************************************
#define	HLXO				0x00000001			// D/A Converter Channel Select HLXO
#define	HLYO				0x00000002			// D/A Converter Channel Select HLYO
#define	HLXBO				0x00000004			// D/A Converter Channel Select HLXBO
#define	HLYBO				0x00000008			// D/A Converter Channel Select HLYBO
#define	HLAFO				0x00000010			// D/A Converter Channel Select HLAFO
#define	HLAFBO				0x00000020			// D/A Converter Channel Select HLAFBO



// MeasureFilter.h *******************************************************************
typedef struct {
	INT32				SiSampleNum ;			// Measure Sample Number
	INT32				SiSampleMax ;			// Measure Sample Number Max

	struct {
		INT32			SiMax1 ;				// Max Measure Result
		INT32			SiMin1 ;				// Min Measure Result
		UINT32	UiAmp1 ;				// Amplitude Measure Result
		INT64		LLiIntegral1 ;			// Integration Measure Result
		INT64		LLiAbsInteg1 ;			// Absolute Integration Measure Result
		INT32			PiMeasureRam1 ;			// Measure Delay RAM Address
	} MeasureFilterA ;

	struct {
		INT32			SiMax2 ;				// Max Measure Result
		INT32			SiMin2 ;				// Min Measure Result
		UINT32	UiAmp2 ;				// Amplitude Measure Result
		INT64		LLiIntegral2 ;			// Integration Measure Result
		INT64		LLiAbsInteg2 ;			// Absolute Integration Measure Result
		INT32			PiMeasureRam2 ;			// Measure Delay RAM Address
	} MeasureFilterB ;
} MeasureFunction_Type ;

/*** caution [little-endian] ***/

#ifdef __OIS_BIG_ENDIAN__
// Big endian
// Word Data Union
union	WRDVAL{
	INT16	SsWrdVal ;
	UINT16	UsWrdVal ;
	UINT8	UcWrkVal[ 2 ] ;
	signed char		ScWrkVal[ 2 ] ;
	struct {
		UINT8	UcHigVal ;
		UINT8	UcLowVal ;
	} StWrdVal ;
} ;


union	DWDVAL {
	UINT32	UlDwdVal ;
	UINT16	UsDwdVal[ 2 ] ;
	struct {
		UINT16	UsHigVal ;
		UINT16	UsLowVal ;
	} StDwdVal ;
	struct {
		UINT8	UcRamVa3 ;
		UINT8	UcRamVa2 ;
		UINT8	UcRamVa1 ;
		UINT8	UcRamVa0 ;
	} StCdwVal ;
} ;

union	ULLNVAL {
	UINT64	UllnValue ;
	UINT32	UlnValue[ 2 ] ;
	struct {
		UINT32	UlHigVal ;
		UINT32	UlLowVal ;
	} StUllnVal ;
} ;


// Float Data Union
union	FLTVAL {
	float			SfFltVal ;
	UINT32	UlLngVal ;
	UINT16	UsDwdVal[ 2 ] ;
	struct {
		UINT16	UsHigVal ;
		UINT16	UsLowVal ;
	} StFltVal ;
} ;

#else	// BIG_ENDDIAN
// Little endian
// Word Data Union
union	WRDVAL{
	INT16	SsWrdVal ;
	UINT16	UsWrdVal ;
	UINT8	UcWrkVal[ 2 ] ;
	signed char		ScWrkVal[ 2 ] ;
	struct {
		UINT8	UcLowVal ;
		UINT8	UcHigVal ;
	} StWrdVal ;
} ;


union	DWDVAL {
	UINT32	UlDwdVal ;
	UINT16	UsDwdVal[ 2 ] ;
	struct {
		UINT16	UsLowVal ;
		UINT16	UsHigVal ;
	} StDwdVal ;
	struct {
		UINT8	UcRamVa0 ;
		UINT8	UcRamVa1 ;
		UINT8	UcRamVa2 ;
		UINT8	UcRamVa3 ;
	} StCdwVal ;
} ;

union	ULLNVAL {
	UINT64	UllnValue ;
	UINT32	UlnValue[ 2 ] ;
	struct {
		UINT32	UlLowVal ;
		UINT32	UlHigVal ;
	} StUllnVal ;
} ;

// Float Data Union
union	FLTVAL {
	float			SfFltVal ;
	UINT32	UlLngVal ;
	UINT16	UsDwdVal[ 2 ] ;
	struct {
		UINT16	UsLowVal ;
		UINT16	UsHigVal ;
	} StFltVal ;
} ;
#endif	// __OIS_BIG_ENDIAN__

typedef union WRDVAL	UnWrdVal ;
typedef union DWDVAL	UnDwdVal;
typedef union ULLNVAL	UnllnVal;
typedef union FLTVAL	UnFltVal ;


typedef struct STADJPAR {
	struct {
		UINT32	UlAdjPhs ;				// Hall Adjust Phase

		UINT16	UsHlxCna ;				// Hall Center Value after Hall Adjust
		UINT16	UsHlxMax ;				// Hall Max Value
		UINT16	UsHlxMxa ;				// Hall Max Value after Hall Adjust
		UINT16	UsHlxMin ;				// Hall Min Value
		UINT16	UsHlxMna ;				// Hall Min Value after Hall Adjust
		UINT16	UsHlxGan ;				// Hall Gain Value
		UINT16	UsHlxOff ;				// Hall Offset Value
		UINT16	UsAdxOff ;				// Hall A/D Offset Value
		UINT16	UsHlxCen ;				// Hall Center Value

		UINT16	UsHlyCna ;				// Hall Center Value after Hall Adjust
		UINT16	UsHlyMax ;				// Hall Max Value
		UINT16	UsHlyMxa ;				// Hall Max Value after Hall Adjust
		UINT16	UsHlyMin ;				// Hall Min Value
		UINT16	UsHlyMna ;				// Hall Min Value after Hall Adjust
		UINT16	UsHlyGan ;				// Hall Gain Value
		UINT16	UsHlyOff ;				// Hall Offset Value
		UINT16	UsAdyOff ;				// Hall A/D Offset Value
		UINT16	UsHlyCen ;				// Hall Center Value

#ifdef	__OIS_CLOSED_AF__
		UINT16	UsHlzCna ;				// Z Hall Center Value after Hall Adjust
		UINT16	UsHlzMax ;				// Z Hall Max Value
		UINT16	UsHlzMxa ;				// Z Hall Max Value after Hall Adjust
		UINT16	UsHlzMin ;				// Z Hall Min Value
		UINT16	UsHlzMna ;				// Z Hall Min Value after Hall Adjust
		UINT16	UsHlzGan ;				// Z Hall Gain Value
		UINT16	UsHlzOff ;				// Z Hall Offset Value
		UINT16	UsAdzOff ;				// Z Hall A/D Offset Value
		UINT16	UsHlzCen ;				// Z Hall Center Value
#endif
	} StHalAdj ;

	struct {
		UINT32	UlLxgVal ;				// Loop Gain X
		UINT32	UlLygVal ;				// Loop Gain Y
#ifdef	__OIS_CLOSED_AF__
		UINT32	UlLzgVal ;				// Loop Gain Z
#endif
	} StLopGan ;

	struct {
		UINT16	UsGxoVal ;				// Gyro A/D Offset X
		UINT16	UsGyoVal ;				// Gyro A/D Offset Y
		UINT16	UsGxoSts ;				// Gyro Offset X Status
		UINT16	UsGyoSts ;				// Gyro Offset Y Status
	} StGvcOff ;
	
	UINT8		UcOscVal ;				// OSC value

} stAdjPar ;

typedef struct STMESRAM {
	INT32	SlMeasureMaxValue ;
	INT32	SlMeasureMinValue ;
	INT32	SlMeasureAmpValue ;
	INT32	SlMeasureAveValue ;
} stMesRam ;									// Struct Measure Ram

typedef struct STHALLINEAR {
	UINT16	XCoefA[6] ;
	UINT16	XCoefB[6] ;
	UINT16	XZone[5] ;
	UINT16	YCoefA[6] ;
	UINT16	YCoefB[6] ;
	UINT16	YZone[5] ;
} stHalLinear ;									// Struct 

__OIS_CMD_HEADER__	stAdjPar	StAdjPar ;				// Execute Command Parameter

	#define		BOTH_ON			0x00
	#define		XONLY_ON		0x01
	#define		YONLY_ON		0x02
	#define		BOTH_OFF		0x03
	#define		ZONLY_OFF		0x04
	#define		ZONLY_ON		0x05

	#define		SINEWAVE	0
	#define		XHALWAVE	1
	#define		YHALWAVE	2
	#define		ZHALWAVE	3
	#define		XACTTEST	10
	#define		YACTTEST	11
	#define		CIRCWAVE	255

	#define		HALL_H_VAL	0x3F800000			/* 1.0 */

 #define		PTP_BEFORE		0
 #define		PTP_AFTER		1
 #define		PTP_ACCEPT		2

/*******************************************************************************
 ******************************************************************************/
#define _GET_UINT32(n,b)				\
{										\
	(n) = ( (UINT32) (b)[0]       )		\
		| ( (UINT32) (b)[1] <<  8 )		\
		| ( (UINT32) (b)[2] << 16 )		\
		| ( (UINT32) (b)[3] << 24 );	\
}

#define _PUT_UINT32(n,b)				\
{										\
	(b)[0] = (UINT8) ( (n)       );		\
	(b)[1] = (UINT8) ( (n) >>  8 );		\
	(b)[2] = (UINT8) ( (n) >> 16 );		\
	(b)[3] = (UINT8) ( (n) >> 24 );		\
}
#define _GET_UINT16(n,b)				\
{										\
	(n) = ( (UINT16) (b)[0]       )		\
		| ( (UINT16) (b)[1] <<  8 );	\
}

#define _PUT_UINT16(n,b)				\
{										\
	(b)[0] = (UINT8) ( (n)       );		\
	(b)[1] = (UINT8) ( (n) >>  8 );		\
}

#define _GET_UINT16BIG(n,b)				\
{										\
	(n) = ( (UINT16) (b)[1]       )		\
		| ( (UINT16) (b)[0] <<  8 );	\
}

#define _PUT_UINT16BIG(n,b)				\
{										\
	(b)[1] = (UINT8) ( (n)       );		\
	(b)[0] = (UINT8) ( (n) >>  8 );		\
}

