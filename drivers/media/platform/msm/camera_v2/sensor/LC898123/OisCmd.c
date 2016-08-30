//********************************************************************************
//
//		<< LC898123 Evaluation Soft >>
//	    Program Name	: OisCmd.c
//		Design			: Y.Shigoeka
//		History			: First edition						
//********************************************************************************
//**************************
//	Include Header File		
//**************************
#define		__OISCMD__

#include	"Ois.h"

#ifdef DEBUG
#include <AT91SAM7S.h>
#include <us.h>
extern void dbg_Dump( unsigned char * pBuffer, int Len);
 #define TRACE(fmt, ...)		dbgu_printf(fmt, ## __VA_ARGS__)
 #define TRACE_DUMP(x,y)		dbg_Dump(x,y)
#else
 #define TRACE(...)
 #define TRACE_DUMP(x,y)
#endif
//****************************************************
//	CUSTOMER NECESSARY CREATING LIST
//****************************************************
/* for I2C communication */ 
extern	void RamWrite32A(unsigned int addr, unsigned int data);
extern 	void RamRead32A( unsigned short addr, void * data );
extern void	WitTim( unsigned short	UsWitTim );

//**************************
//	extern  Function LIST
//**************************
extern INT16 Calibration_VerifyUpdate_PreRead(void);
extern INT16 Calibration_VerifyUpdate(void);
extern INT16 	FlashNVRSectorErase_Byte( UINT8 );
extern void 	FlashNVR_ReadData_Byte( UINT8, UINT8, UINT8 *, UINT16 );
extern INT16 	FlashNVR_WriteData_Byte( UINT8, UINT8, UINT8 *, UINT16 );

extern UINT8  FLASH_SECTOR_BUFFER[256];
//**************************
//	Local Function LIST
//**************************
void	IniCmd( void ) ;									// Command Execute Process Initial
void	MemClr( UINT8	*NcTgtPtr, UINT16	UsClrSiz );
void	IniPtAve( void ) ;									// Average setting
UINT32	TneRunZ( void );
UINT32	TnePtp ( UINT8	UcDirSel, UINT8	UcBfrAft );
UINT32	TneCen( UINT8	UcTneAxs, UnDwdVal	StTneVal );
UINT32	TneOff( UnDwdVal, UINT8 ) ;							// Hall Offset Tuning
UINT32	TneBia( UnDwdVal, UINT8 ) ;							// Hall Bias Tuning
void	MesFil( UINT8 ) ;									// Measure Filter Setting
void	ClrMesFil( void );
void	MeasureStart( INT32 , INT32 , INT32 ) ;				// Measure Start Function
void	MeasureWait( void ) ;								// Measure Wait
void	MemoryClear( UINT16 , UINT16 ) ;					// Memory Cloear
void	SetWaitTime( UINT16 ) ; 							// Set Wait Timer
UINT32	LopGan( UINT8	UcDirSel );
UINT32	TneGvc( void );
void	OisEnaNCL( void );
void	OisEnaDrCl( void );
void	OisEnaDrNcl( void );
void	OisDis( void );
void	SetRec( void );
void	SetStill( void );
UINT8	TneHvc( void );
void	TneFin( void );
void	IniNvc( INT16 SsX, INT16 SsY );
void	TneSltPos( UINT8 UcPos );
void	TneVrtPos( UINT8 UcPos );
void	TneHrzPos( UINT8 UcPos );
void	SetSinWavGenInt( void );
void	SetTransDataAdr( UINT16 UsLowAddress , UINT32 UlLowAdrBeforeTrans );
void	DacControl( UINT8 UcMode, UINT32 UiChannel, UINT32 PuiData );
INT16 WrGyroGainData_NV( UINT32 UlReadValX , UINT32 UlReadValY );
UINT16 TneADO( void );
void	OscStb( void );
UINT8	 MesRam( INT32 , INT32 , INT32 , stMesRam* , stMesRam* );

UINT8	TstActMov( UINT8 );
UINT8 FrqDet( void );


#define abs(a)   (a >0 ? a: (-a))

//**************************
//	define					
//**************************
#define 	MES_XG1			0								// LXG1 Measure Mode
#define 	MES_XG2			1								// LXG2 Measure Mode

#define 	HALL_ADJ		0
#define 	LOOPGAIN		1
#define 	THROUGH			2
#define 	NOISE			3
#define		OSCCHK			4

// Measure Mode

 #define 	TNE 			80								// Waiting Time For Movement

 /******* Hall calibration Type 1 *******/
 
 #define 	OFFSET_DIV		2								// Divide Difference For Offset Step
 #define 	TIME_OUT		20								// Time Out Count
 #define	BIAS_HLMT		(UINT32)0xBF000000
 #define	BIAS_LLMT		(UINT32)0x20000000
 #define 	MARGIN			0x0300							// Margin

 #define 	BIAS_ADJ_RANGE_XY	0xCCCC						// 80%

 #define 	HALL_MAX_RANGE_XY	BIAS_ADJ_RANGE_XY + MARGIN
 #define 	HALL_MIN_RANGE_XY	BIAS_ADJ_RANGE_XY - MARGIN


#ifdef	__OIS_CLOSED_AF__
// #define	BIAS_ADJ_RANGE_Z	0xAFDE						// 68.7%
 #define	BIAS_ADJ_RANGE_Z	0x8CCC						// 55.0%	ver 000A3  
 #define 	HALL_MAX_RANGE_Z	BIAS_ADJ_RANGE_Z + MARGIN
 #define 	HALL_MIN_RANGE_Z	BIAS_ADJ_RANGE_Z - MARGIN
#endif

 #define 	DECRE_CAL		0x0100							// decrease value
/***************************************/
#define		SLT_OFFSET		(0x1000)
#define		LENS_MARGIN		(0x0800)
#define		PIXEL_SIZE		(1.12f)							// pixel size 1.12um
#define		SPEC_RANGE		(120.0f)						// spec need movable range 130um
#define		SPEC_PIXEL		(PIXEL_SIZE / SPEC_RANGE)		// spec need movable range pixel
/***************************************/
// Threshold of osciration amplitude
#define ULTHDVAL	0x01000000								// Threshold of the hale value


//**************************
//	Global Variable			
//**************************
INT16		SsNvcX = 1 ;									// NVC move direction X
INT16		SsNvcY = 1 ;									// NVC move direction Y

UINT16	UsValBef,UsValNow ;


//**************************
//	Const					
//**************************
//********************************************************************************
// Function Name 	: IniCmd
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Command Execute Process Initial
// History			: First edition 						
//********************************************************************************
void	IniCmd( void )
{

	MemClr( ( UINT8 * )&StAdjPar, sizeof( stAdjPar ) ) ;	// Adjust Parameter Clear
	
}


//********************************************************************************
// Function Name 	: MemClr
// Retun Value		: void
// Argment Value	: Clear Target Pointer, Clear Byte Number
// Explanation		: Memory Clear Function
// History			: First edition 						
//********************************************************************************
void	MemClr( UINT8	*NcTgtPtr, UINT16	UsClrSiz )
{
	UINT16	UsClrIdx ;

	for ( UsClrIdx = 0 ; UsClrIdx < UsClrSiz ; UsClrIdx++ )
	{
		*NcTgtPtr	= 0 ;
		NcTgtPtr++ ;
	}
}


//********************************************************************************
// Function Name 	: TneRun
// Retun Value		: Hall Tuning SUCCESS or FAILURE
// Argment Value	: NON
// Explanation		: Hall System Auto Adjustment Function
// History			: First edition 						
//********************************************************************************
UINT32	TneRun( void )
{
	UINT32	UlHlySts, UlHlxSts, UlAtxSts, UlAtySts, UlGvcSts ;
//	UINT8	UcDrvMod ;
	UnDwdVal		StTneVal ;
	UINT32	UlFinSts , UlReadVal ;

	// Check relation the firmware version of LC898123
	/* Chaeck Driver gain */
	UlHlxSts = UlHlySts = 0x00 ;
	RamRead32A( CURDRV_X_SiPlsVal , &UlReadVal ) ;
	if( ( UlReadVal > (UINT32)0x7FFFFFFF ) || ( (UINT32)0x60000000 > UlReadVal )){
		UlHlxSts = EXE_HXADJ;
	}
	RamRead32A( CURDRV_X_SiMnsVal , &UlReadVal ) ;
	if( ( UlReadVal > (UINT32)0x7FFFFFFF ) || ( (UINT32)0x60000000 > UlReadVal )){
		UlHlxSts = EXE_HXADJ;
	}
	RamRead32A( CURDRV_Y_SiPlsVal , &UlReadVal ) ;
	if( ( UlReadVal > (UINT32)0x7FFFFFFF ) || ( (UINT32)0x60000000 > UlReadVal )){
		UlHlySts = EXE_HYADJ;
	}
	RamRead32A( CURDRV_Y_SiMnsVal , &UlReadVal ) ;
	if( ( UlReadVal > (UINT32)0x7FFFFFFF ) || ( (UINT32)0x60000000 > UlReadVal )){
		UlHlySts = EXE_HYADJ;
	}
	if( UlHlxSts || UlHlySts ){
		UlFinSts	= UlHlxSts | UlHlySts ;
		StAdjPar.StHalAdj.UlAdjPhs = UlFinSts ;
		return( UlFinSts ) ;
	}


	RtnCen( BOTH_OFF ) ;		// Both OFF
	WitTim( TNE ) ;

	RamWrite32A( HALL_RAM_HXOFF,  0x00000000 ) ;		// X Offset Clr
	RamWrite32A( HALL_RAM_HYOFF,  0x00000000 ) ;		// Y Offset Clr
	RamWrite32A( HallFilterCoeffX_hxgain0 , SXGAIN_LOP ) ;
	RamWrite32A( HallFilterCoeffY_hygain0 , SYGAIN_LOP ) ;
	DacControl( 0 , HLXBO , XY_BIAS ) ;
	RamWrite32A( StCaliData_UiHallBias_X , XY_BIAS ) ;
	DacControl( 0 , HLYBO , XY_BIAS ) ;
	RamWrite32A( StCaliData_UiHallBias_Y , XY_BIAS ) ;
	DacControl( 0 , HLXO, XY_OFST ) ;
	RamWrite32A( StCaliData_UiHallOffset_X , XY_OFST ) ;
	DacControl( 0 , HLYO, XY_OFST ) ;
	RamWrite32A( StCaliData_UiHallOffset_Y , XY_OFST ) ;

	StTneVal.UlDwdVal	= TnePtp( Y_DIR , PTP_BEFORE ) ;
	UlHlySts	= TneCen( Y_DIR, StTneVal ) ;
	
	RtnCen( YONLY_ON ) ;		// Y ON / X OFF
	WitTim( TNE ) ;

	StTneVal.UlDwdVal	= TnePtp( X_DIR , PTP_BEFORE ) ;
	UlHlxSts	= TneCen( X_DIR, StTneVal ) ;

	if( (UlHlxSts != EXE_HXADJ) && (UlHlySts != EXE_HYADJ) ){
		RtnCen( XONLY_ON ) ;		// Y OFF / X ON
		WitTim( TNE ) ;

		StTneVal.UlDwdVal	= TnePtp( Y_DIR , PTP_AFTER ) ;
		UlHlySts	= TneCen( Y_DIR, StTneVal ) ;
	
		RtnCen( YONLY_ON ) ;		// Y ON / X OFF
		WitTim( TNE ) ;

		StTneVal.UlDwdVal	= TnePtp( X_DIR , PTP_AFTER ) ;
		UlHlxSts	= TneCen( X_DIR, StTneVal ) ;
	
		RtnCen( BOTH_OFF ) ;		// Both OFF
	
#ifdef	NEUTRAL_CENTER
		TneHvc();
#endif	//NEUTRAL_CENTER
	

		WitTim( TNE ) ;

		StAdjPar.StHalAdj.UsAdxOff = StAdjPar.StHalAdj.UsHlxCna  ;
		StAdjPar.StHalAdj.UsAdyOff = StAdjPar.StHalAdj.UsHlyCna  ;
		
TRACE("    Xadof = %04xh \n", StAdjPar.StHalAdj.UsAdxOff ) ;
TRACE("    Yadof = %04xh \n", StAdjPar.StHalAdj.UsAdyOff ) ;
	
		RamWrite32A( HALL_RAM_HXOFF,  (UINT32)((StAdjPar.StHalAdj.UsAdxOff << 16 ) & 0xFFFF0000 )) ;
		RamWrite32A( HALL_RAM_HYOFF,  (UINT32)((StAdjPar.StHalAdj.UsAdyOff << 16 ) & 0xFFFF0000 )) ;
		
		RamRead32A( StCaliData_UiHallOffset_X , &UlReadVal ) ;
		StAdjPar.StHalAdj.UsHlxOff = (UINT16)( UlReadVal >> 16 ) ;
		
		RamRead32A( StCaliData_UiHallBias_X , &UlReadVal ) ;
		StAdjPar.StHalAdj.UsHlxGan = (UINT16)( UlReadVal >> 16 ) ;
		
		RamRead32A( StCaliData_UiHallOffset_Y , &UlReadVal ) ;
		StAdjPar.StHalAdj.UsHlyOff = (UINT16)( UlReadVal >> 16 ) ;
		
		RamRead32A( StCaliData_UiHallBias_Y , &UlReadVal ) ;
		StAdjPar.StHalAdj.UsHlyGan = (UINT16)( UlReadVal >> 16 ) ;
		
#ifdef	NEUTRAL_CENTER_FINE
		TneFin();

TRACE("    XadofFin = %04xh \n", StAdjPar.StHalAdj.UsAdxOff ) ;
TRACE("    YadofFin = %04xh \n", StAdjPar.StHalAdj.UsAdyOff ) ;
		RamWrite32A( HALL_RAM_HXOFF,  (UINT32)((StAdjPar.StHalAdj.UsAdxOff << 16 ) & 0xFFFF0000 )) ;
		RamWrite32A( HALL_RAM_HYOFF,  (UINT32)((StAdjPar.StHalAdj.UsAdyOff << 16 ) & 0xFFFF0000 )) ;
#endif	//NEUTRAL_CENTER_FINE

		RtnCen( BOTH_ON ) ;		// Y ON / X ON

		WitTim( TNE ) ;

		// X Loop Gain Adjust
		UlAtxSts	= LopGan( X_DIR ) ;
	
		// Y Loop Gain Adjust
		UlAtySts	= LopGan( Y_DIR ) ;
	
	}else{
		RtnCen( BOTH_OFF ) ;		// Both OFF
		WitTim( TNE ) ;
		UlAtxSts = EXE_LXADJ;
		UlAtySts = EXE_LYADJ;
	}
	
	UlGvcSts = TneGvc() ;

	UlFinSts	= ( UlHlxSts - EXE_END ) + ( UlHlySts - EXE_END ) + ( UlAtxSts - EXE_END ) + ( UlAtySts - EXE_END )  + ( UlGvcSts - EXE_END ) + EXE_END ;
	StAdjPar.StHalAdj.UlAdjPhs = UlFinSts ;

	
	return( UlFinSts ) ;
}


#ifdef	__OIS_CLOSED_AF__
//********************************************************************************
// Function Name 	: TneRunZ
// Retun Value		: Z axis Hall Tuning SUCCESS or FAILURE
// Argment Value	: NON
// Explanation		: Z_axis Hall System Auto Adjustment Function
// History			: First edition 						
//********************************************************************************
UINT32	TneRunZ( void )
{
	UINT32	UlHlzSts, UlAtzSts ;
	UnDwdVal		StTneVal ;
	UINT32	UlFinSts , UlReadVal ;

	UlHlzSts = 0x00 ;

	RamRead32A( CURDRV_AF_SiPlsVal , &UlReadVal ) ;
	if( ( UlReadVal > (UINT32)0x7FFFFFFF ) || ( (UINT32)0x60000000 > UlReadVal )){
		UlHlzSts = EXE_HZADJ;
	}
	RamRead32A( CURDRV_AF_SiMnsVal , &UlReadVal ) ;
	if( ( UlReadVal > (UINT32)0x7FFFFFFF ) || ( (UINT32)0x60000000 > UlReadVal )){
		UlHlzSts = EXE_HZADJ;
	}
	if( UlHlzSts ){
		UlFinSts	= UlHlzSts ;
		StAdjPar.StHalAdj.UlAdjPhs = UlFinSts ;
		return( UlFinSts ) ;
	}
	
	RtnCen( ZONLY_OFF ) ;
	WitTim( TNE ) ;

//TRACE_USB("TGT,%d,%d", BIAS_ADJ_RANGE_Z, MARGIN );
	
	RamWrite32A( CLAF_RAMA_AFOFFSET,  0x00000000 ) ;		// Z Offset Clr
	RamWrite32A( CLAF_Gain_afloop2 , SZGAIN_LOP ) ;
	DacControl( 0 , HLAFBO , Z_BIAS ) ;
	RamWrite32A( StCaliData_UiHallBias_AF , Z_BIAS) ;
	DacControl( 0 , HLAFO, Z_OFST ) ;
	RamWrite32A( StCaliData_UiHallOffset_AF , Z_OFST ) ;

	StTneVal.UlDwdVal	= TnePtp( Z_DIR , PTP_BEFORE ) ;
	UlHlzSts	= TneCen( Z_DIR, StTneVal ) ;
	
	WitTim( TNE ) ;

	UlReadVal = 0x00010000 - (UINT32)StAdjPar.StHalAdj.UsHlzCna ;
	StAdjPar.StHalAdj.UsAdzOff = (UINT16)UlReadVal ;
	
	
	RamWrite32A( CLAF_RAMA_AFOFFSET,  (UINT32)((StAdjPar.StHalAdj.UsAdzOff << 16 ) & 0xFFFF0000 )) ;
	
	RamRead32A( StCaliData_UiHallOffset_AF , &UlReadVal ) ;
	StAdjPar.StHalAdj.UsHlzOff = (UINT16)( UlReadVal >> 16 ) ;
	
	RamRead32A( StCaliData_UiHallBias_AF , &UlReadVal ) ;
	StAdjPar.StHalAdj.UsHlzGan = (UINT16)( UlReadVal >> 16 ) ;
	
	RtnCen( ZONLY_ON ) ;		// Z ON
	
	WitTim( TNE ) ;

	// Z Loop Gain Adjust
	UlAtzSts	= LopGan( Z_DIR ) ;
//	UlAtzSts	= EXE_END ;

	UlFinSts	= ( UlHlzSts - EXE_END ) + ( UlAtzSts - EXE_END ) + EXE_END ;
	StAdjPar.StHalAdj.UlAdjPhs = UlFinSts ;

	
	return( UlFinSts ) ;
}

//********************************************************************************
// Function Name 	: TneRunA
// Retun Value		: AF + OIS Hall Tuning SUCCESS or FAILURE
// Argment Value	: NON
// Explanation		: AF + OIS Hall System Auto Adjustment Function
// History			: First edition 						
//********************************************************************************
UINT32	TneRunA( void )
{
	UINT32	UlFinSts ;

	UlFinSts = TneRunZ();
	UlFinSts |= TneRun();
	
	StAdjPar.StHalAdj.UlAdjPhs = UlFinSts ;
	return( UlFinSts ) ;
}

#endif

//********************************************************************************
// Function Name 	: TnePtp
// Retun Value		: Hall Top & Bottom Gaps
// Argment Value	: X,Y Direction, Adjust Before After Parameter
// Explanation		: Measuring Hall Paek To Peak
// History			: First edition 						
//********************************************************************************

UINT32	TnePtp ( UINT8	UcDirSel, UINT8	UcBfrAft )
{
	UnDwdVal		StTneVal ;
	INT32			SlMeasureParameterA , SlMeasureParameterB ;
	INT32			SlMeasureParameterNum ;
	INT32			SlMeasureMaxValue , SlMeasureMinValue ;

//	SlMeasureParameterNum	=	2004 ;		// 20.0195/0.010 < x
	SlMeasureParameterNum	=	2000 ;		// 20.0195/0.010 < x

	if( UcDirSel == X_DIR ) {								// X axis
		SlMeasureParameterA		=	HALL_RAM_HXIDAT ;		// Set Measure RAM Address
		SlMeasureParameterB		=	HALL_RAM_HYIDAT ;		// Set Measure RAM Address
	} else if( UcDirSel == Y_DIR ) {						// Y axis
		SlMeasureParameterA		=	HALL_RAM_HYIDAT ;		// Set Measure RAM Address
		SlMeasureParameterB		=	HALL_RAM_HXIDAT ;		// Set Measure RAM Address
#ifdef	__OIS_CLOSED_AF__
	} else {												// Z axis
		SlMeasureParameterA		=	CLAF_RAMA_AFADIN ;		// Set Measure RAM Address
		SlMeasureParameterB		=	CLAF_RAMA_AFADIN ;		// Set Measure RAM Address
#endif
	}
	SetSinWavGenInt();
	
	RamWrite32A( SinWave_Offset		,	0x105E36 ) ;				// Freq Setting = Freq * 80000000h / Fs	: 10Hz
	RamWrite32A( SinWave_Gain		,	0x7FFFFFFF ) ;				// Set Sine Wave Gain
	RamWrite32A( SinWaveC_Regsiter	,	0x00000001 ) ;				// Sine Wave Start
	if( UcDirSel == X_DIR ) {
		SetTransDataAdr( SinWave_OutAddr	,	(UINT32)HALL_RAM_SINDX1 ) ;	// Set Sine Wave Input RAM
	}else if( UcDirSel == Y_DIR ){
		SetTransDataAdr( SinWave_OutAddr	,	(UINT32)HALL_RAM_SINDY1 ) ;	// Set Sine Wave Input RAM
#ifdef	__OIS_CLOSED_AF__
	}else{
		SetTransDataAdr( SinWave_OutAddr	,	(UINT32)CLAF_RAMA_AFOUT ) ;	// Set Sine Wave Input RAM
#endif
	}
	
	MesFil( THROUGH ) ;					// Filter setting for measurement

	MeasureStart( SlMeasureParameterNum , SlMeasureParameterA , SlMeasureParameterB ) ;					// Start measure
	
	MeasureWait() ;						// Wait complete of measurement
	
	RamWrite32A( SinWaveC_Regsiter	,	0x00000000 ) ;								// Sine Wave Stop
	
	if( UcDirSel == X_DIR ) {
		SetTransDataAdr( SinWave_OutAddr	,	(UINT32)0x00000000 ) ;	// Set Sine Wave Input RAM
		RamWrite32A( HALL_RAM_SINDX1		,	0x00000000 ) ;				// DelayRam Clear
	}else if( UcDirSel == Y_DIR ){
		SetTransDataAdr( SinWave_OutAddr	,	(UINT32)0x00000000 ) ;	// Set Sine Wave Input RAM
		RamWrite32A( HALL_RAM_SINDY1		,	0x00000000 ) ;				// DelayRam Clear
#ifdef	__OIS_CLOSED_AF__
	}else{
		SetTransDataAdr( SinWave_OutAddr	,	(UINT32)0x00000000 ) ;	// Set Sine Wave Input RAM
		RamWrite32A( CLAF_RAMA_AFOUT		,	0x00000000 ) ;				// DelayRam Clear
#endif
	}
	RamRead32A( StMeasFunc_MFA_SiMax1 , ( UINT32 * )&SlMeasureMaxValue ) ;	// Max value
	RamRead32A( StMeasFunc_MFA_SiMin1 , ( UINT32 * )&SlMeasureMinValue ) ;	// Min value

	StTneVal.StDwdVal.UsHigVal = (UINT16)((SlMeasureMaxValue >> 16) & 0x0000FFFF );
	StTneVal.StDwdVal.UsLowVal = (UINT16)((SlMeasureMinValue >> 16) & 0x0000FFFF );
//TRACE_USB("PTP,%d,%04x,%04x",  UcDirSel, StTneVal.StDwdVal.UsHigVal, StTneVal.StDwdVal.UsLowVal );
	
	if( UcBfrAft == 0 ) {
		if( UcDirSel == X_DIR ) {
			StAdjPar.StHalAdj.UsHlxCen	= ( ( INT16 )StTneVal.StDwdVal.UsHigVal + ( INT16 )StTneVal.StDwdVal.UsLowVal ) / 2 ;
			StAdjPar.StHalAdj.UsHlxMax	= StTneVal.StDwdVal.UsHigVal ;
			StAdjPar.StHalAdj.UsHlxMin	= StTneVal.StDwdVal.UsLowVal ;
		} else if( UcDirSel == Y_DIR ){
			StAdjPar.StHalAdj.UsHlyCen	= ( ( INT16 )StTneVal.StDwdVal.UsHigVal + ( INT16 )StTneVal.StDwdVal.UsLowVal ) / 2 ;
			StAdjPar.StHalAdj.UsHlyMax	= StTneVal.StDwdVal.UsHigVal ;
			StAdjPar.StHalAdj.UsHlyMin	= StTneVal.StDwdVal.UsLowVal ;
#ifdef	__OIS_CLOSED_AF__
		} else {
			StAdjPar.StHalAdj.UsHlzCen	= ( ( INT16 )StTneVal.StDwdVal.UsHigVal + ( INT16 )StTneVal.StDwdVal.UsLowVal ) / 2 ;
			StAdjPar.StHalAdj.UsHlzMax	= StTneVal.StDwdVal.UsHigVal ;
			StAdjPar.StHalAdj.UsHlzMin	= StTneVal.StDwdVal.UsLowVal ;
//TRACE_USB("PTP,%d,%04x,%04x",  UcDirSel, StTneVal.StDwdVal.UsHigVal, StTneVal.StDwdVal.UsLowVal );
#endif
		}
	} else {
		if( UcDirSel == X_DIR ){
			StAdjPar.StHalAdj.UsHlxCna	= ( ( INT16 )StTneVal.StDwdVal.UsHigVal + ( INT16 )StTneVal.StDwdVal.UsLowVal ) / 2 ;
			StAdjPar.StHalAdj.UsHlxMxa	= StTneVal.StDwdVal.UsHigVal ;
			StAdjPar.StHalAdj.UsHlxMna	= StTneVal.StDwdVal.UsLowVal ;
		} else if( UcDirSel == Y_DIR ){
			StAdjPar.StHalAdj.UsHlyCna	= ( ( INT16 )StTneVal.StDwdVal.UsHigVal + ( INT16 )StTneVal.StDwdVal.UsLowVal ) / 2 ;
			StAdjPar.StHalAdj.UsHlyMxa	= StTneVal.StDwdVal.UsHigVal ;
			StAdjPar.StHalAdj.UsHlyMna	= StTneVal.StDwdVal.UsLowVal ;
#ifdef	__OIS_CLOSED_AF__
		} else {
			StAdjPar.StHalAdj.UsHlzCna	= ( ( INT16 )StTneVal.StDwdVal.UsHigVal + ( INT16 )StTneVal.StDwdVal.UsLowVal ) / 2 ;
			StAdjPar.StHalAdj.UsHlzMxa	= StTneVal.StDwdVal.UsHigVal ;
			StAdjPar.StHalAdj.UsHlzMna	= StTneVal.StDwdVal.UsLowVal ;
//TRACE_USB("PTP,%d,%04x,%04x",  UcDirSel, StTneVal.StDwdVal.UsHigVal, StTneVal.StDwdVal.UsLowVal );
#endif
		}
	}

TRACE("ADJ(%d) MAX = %04x, MIN = %04x, CNT = %04x, ", UcDirSel, StTneVal.StDwdVal.UsHigVal, StTneVal.StDwdVal.UsLowVal, ( ( signed int )StTneVal.StDwdVal.UsHigVal + ( signed int )StTneVal.StDwdVal.UsLowVal ) / 2 ) ;
	StTneVal.StDwdVal.UsHigVal	= 0x7fff - StTneVal.StDwdVal.UsHigVal ;		// Maximum Gap = Maximum - Hall Peak Top
	StTneVal.StDwdVal.UsLowVal	= StTneVal.StDwdVal.UsLowVal - 0x7fff ; 	// Minimum Gap = Hall Peak Bottom - Minimum

TRACE("GapH = %04x, GapL = %04x\n", StTneVal.StDwdVal.UsHigVal, StTneVal.StDwdVal.UsLowVal ) ;
TRACE("Raw MAX = %08x, MIN = %08x\n, ", (unsigned int)SlMeasureMaxValue , (unsigned int)SlMeasureMinValue ) ;
	
	return( StTneVal.UlDwdVal ) ;
}

//********************************************************************************
// Function Name 	: TneCen
// Retun Value		: Hall Center Tuning Result
// Argment Value	: X,Y Direction, Hall Top & Bottom Gaps
// Explanation		: Hall Center Tuning Function
// History			: First edition 						
//********************************************************************************
UINT32	TneCen( UINT8	UcTneAxs, UnDwdVal	StTneVal )
{
	UINT8 	UcTmeOut, UcTofRst ;
	UINT16	UsBiasVal ;
	UINT32	UlTneRst, UlBiasVal , UlValNow ;
	UINT16	UsHalMaxRange , UsHalMinRange ;
	
	UcTmeOut	= 1 ;
	UlTneRst	= FAILURE ;
	UcTofRst	= FAILURE ;

#ifdef	__OIS_CLOSED_AF__
	if(UcTneAxs == Z_DIR){
		UsHalMaxRange = HALL_MAX_RANGE_Z ;
		UsHalMinRange = HALL_MIN_RANGE_Z ;
	}else{
		UsHalMaxRange = HALL_MAX_RANGE_XY ;
		UsHalMinRange = HALL_MIN_RANGE_XY ;
	}
#else
	UsHalMaxRange = HALL_MAX_RANGE_XY ;
	UsHalMinRange = HALL_MIN_RANGE_XY ;
#endif
	while ( UlTneRst && (UINT32)UcTmeOut )
	{
		if( UcTofRst == FAILURE ) {
			StTneVal.UlDwdVal	= TneOff( StTneVal, UcTneAxs ) ;
		} else {
			StTneVal.UlDwdVal	= TneBia( StTneVal, UcTneAxs ) ;
			UcTofRst	= FAILURE ;
			if( UcTneAxs == X_DIR ) {
				RamRead32A( StCaliData_UiHallBias_X , &UlBiasVal ) ;
			}else if( UcTneAxs == Y_DIR ){
				RamRead32A( StCaliData_UiHallBias_Y , &UlBiasVal ) ;
#ifdef	__OIS_CLOSED_AF__
			}else{
				RamRead32A( StCaliData_UiHallBias_AF , &UlBiasVal ) ;
#endif
			}
			if(UlBiasVal == 0x00000000){
				UcTmeOut = TIME_OUT;
			}
		}

TRACE("  No = %04d", UcTmeOut ) ;
		if( (StTneVal.StDwdVal.UsHigVal > MARGIN ) && (StTneVal.StDwdVal.UsLowVal > MARGIN ) )	/* position check */
		{
			UcTofRst	= SUCCESS ;
TRACE("  TofR = SUCC" ) ;
			UsValBef = UsValNow = 0x0000 ;
		}else if( (StTneVal.StDwdVal.UsHigVal <= MARGIN ) && (StTneVal.StDwdVal.UsLowVal <= MARGIN ) ){
			UcTofRst	= SUCCESS ;
			UlTneRst	= (UINT32)FAILURE ;
//		}else if( ((UINT16)0xFFFF - ( StTneVal.StDwdVal.UsHigVal + StTneVal.StDwdVal.UsLowVal )) > BIAS_ADJ_OVER ) {
//			UcTofRst	= SUCCESS ;
//			UlTneRst	= (UINT32)FAILURE ;
		}else{
			UcTofRst	= FAILURE ;
TRACE("  TofR = FAIL" ) ;
			
			UsValBef = UsValNow ;

			if( UcTneAxs == X_DIR  ) {
				RamRead32A( StCaliData_UiHallOffset_X , &UlValNow ) ;
				UsValNow = (UINT16)( UlValNow >> 16 ) ;
			}else if( UcTneAxs == Y_DIR ){
				RamRead32A( StCaliData_UiHallOffset_Y , &UlValNow ) ;
				UsValNow = (UINT16)( UlValNow >> 16 ) ;
#ifdef	__OIS_CLOSED_AF__
			}else{
				RamRead32A( StCaliData_UiHallOffset_AF , &UlValNow ) ;
				UsValNow = (UINT16)( UlValNow >> 16 ) ;
#endif
			}
			if( ((( UsValBef & 0xFF00 ) == 0x1000 ) && ( UsValNow & 0xFF00 ) == 0x1000 )
			 || ((( UsValBef & 0xFF00 ) == 0xEF00 ) && ( UsValNow & 0xFF00 ) == 0xEF00 ) )
			{
				if( UcTneAxs == X_DIR ) {
					RamRead32A( StCaliData_UiHallBias_X , &UlBiasVal ) ;
					UsBiasVal = (UINT16)( UlBiasVal >> 16 ) ;
				}else if( UcTneAxs == Y_DIR ){
					RamRead32A( StCaliData_UiHallBias_Y , &UlBiasVal ) ;
					UsBiasVal = (UINT16)( UlBiasVal >> 16 ) ;
#ifdef	__OIS_CLOSED_AF__
				}else{
					RamRead32A( StCaliData_UiHallBias_AF , &UlBiasVal ) ;
					UsBiasVal = (UINT16)( UlBiasVal >> 16 ) ;
#endif
				}
				
				if( UsBiasVal > DECRE_CAL )
				{
					UsBiasVal -= DECRE_CAL ;
				}
				
				if( UcTneAxs == X_DIR ) {
					UlBiasVal = ( UINT32 )( UsBiasVal << 16 ) ;
					DacControl( 0 , HLXBO , UlBiasVal ) ;
					RamWrite32A( StCaliData_UiHallBias_X , UlBiasVal ) ;
				}else if( UcTneAxs == Y_DIR ){
					UlBiasVal = ( UINT32 )( UsBiasVal << 16 ) ;
					DacControl( 0 , HLYBO , UlBiasVal ) ;
					RamWrite32A( StCaliData_UiHallBias_Y , UlBiasVal ) ;
#ifdef	__OIS_CLOSED_AF__
				}else{
					UlBiasVal = ( UINT32 )( UsBiasVal << 16 ) ;
					DacControl( 0 , HLAFBO , UlBiasVal ) ;
					RamWrite32A( StCaliData_UiHallBias_AF , UlBiasVal ) ;
#endif
				}
			}

		}
		
		if((( (UINT16)0xFFFF - ( StTneVal.StDwdVal.UsHigVal + StTneVal.StDwdVal.UsLowVal )) < UsHalMaxRange )
		&& (( (UINT16)0xFFFF - ( StTneVal.StDwdVal.UsHigVal + StTneVal.StDwdVal.UsLowVal )) > UsHalMinRange ) ) {
			if(UcTofRst	== SUCCESS)
			{
				UlTneRst	= (UINT32)SUCCESS ;
				break ;
			}
		}
		UlTneRst	= (UINT32)FAILURE ;
		UcTmeOut++ ;
TRACE("  Tne = FAIL\n" ) ;

		if ( UcTmeOut >= TIME_OUT ) {
			UcTmeOut	= 0 ;
		}		 																							// Set Time Out Count
	}

	SetSinWavGenInt() ;		// 
	
	if( UlTneRst == (UINT32)FAILURE ) {
		if( UcTneAxs == X_DIR ) {
			UlTneRst					= EXE_HXADJ ;
			StAdjPar.StHalAdj.UsHlxGan	= 0xFFFF ;
			StAdjPar.StHalAdj.UsHlxOff	= 0xFFFF ;
		}else if( UcTneAxs == Y_DIR ) {
			UlTneRst					= EXE_HYADJ ;
			StAdjPar.StHalAdj.UsHlyGan	= 0xFFFF ;
			StAdjPar.StHalAdj.UsHlyOff	= 0xFFFF ;
#ifdef	__OIS_CLOSED_AF__
		} else {
			UlTneRst					= EXE_HZADJ ;
			StAdjPar.StHalAdj.UsHlzGan	= 0xFFFF ;
			StAdjPar.StHalAdj.UsHlzOff	= 0xFFFF ;
#endif
		}
	} else {
		UlTneRst	= EXE_END ;
	}

	return( UlTneRst ) ;
}



//********************************************************************************
// Function Name 	: TneBia
// Retun Value		: Hall Top & Bottom Gaps
// Argment Value	: Hall Top & Bottom Gaps , X,Y Direction
// Explanation		: Hall Bias Tuning Function
// History			: First edition 						
//********************************************************************************
UINT32	TneBia( UnDwdVal	StTneVal, UINT8	UcTneAxs )
{
	UINT32			UlSetBia , UlSetBia_Bk;
	UINT16			UsHalAdjRange;
#ifdef	__OIS_CLOSED_AF__
	if(UcTneAxs == Z_DIR){
		UsHalAdjRange = BIAS_ADJ_RANGE_Z ;
	}else{
		UsHalAdjRange = BIAS_ADJ_RANGE_XY ;
	}
#else
		UsHalAdjRange = BIAS_ADJ_RANGE_XY ;
#endif

	if( UcTneAxs == X_DIR ) {
		RamRead32A( StCaliData_UiHallBias_X , &UlSetBia ) ;		
	} else if( UcTneAxs == Y_DIR ) {
		RamRead32A( StCaliData_UiHallBias_Y , &UlSetBia ) ;		
#ifdef	__OIS_CLOSED_AF__
	} else {
		RamRead32A( StCaliData_UiHallBias_AF , &UlSetBia ) ;		
#endif
	}
	UlSetBia_Bk = UlSetBia ;	// backup

	if( UlSetBia == 0x00000000 )	UlSetBia = 0x01000000 ;
	UlSetBia = (( UlSetBia >> 16 ) & (UINT32)0x0000FF00 ) ;
	UlSetBia *= (UINT32)UsHalAdjRange ;
	UlSetBia /= (UINT32)( 0xFFFF - ( StTneVal.StDwdVal.UsHigVal + StTneVal.StDwdVal.UsLowVal )) ;
	if( UlSetBia > (UINT32)0x0000FFFF )		UlSetBia = 0x0000FFFF ;
	UlSetBia = ( UlSetBia << 16 ) ;
	if( UlSetBia > BIAS_HLMT )		UlSetBia = BIAS_HLMT ;
	if( UlSetBia < BIAS_LLMT )		UlSetBia = BIAS_LLMT ;
	
	if(( (UlSetBia_Bk == BIAS_LLMT) && (UlSetBia == BIAS_LLMT) ) || ( UlSetBia_Bk == BIAS_HLMT && UlSetBia == BIAS_HLMT )){
		UlSetBia = 0x00000000 ;		// note :  0x20000000 =< Bia =< 0xBF000000
TRACE("		BIAS = ERR\n " ) ;
	}

	if( UcTneAxs == X_DIR ) {
		DacControl( 0 , HLXBO , UlSetBia ) ;
TRACE("		HLXBO = %08x\n ",  (unsigned int)UlSetBia ) ;
		RamWrite32A( StCaliData_UiHallBias_X , UlSetBia) ;
	} else if( UcTneAxs == Y_DIR ){
		DacControl( 0 , HLYBO , UlSetBia ) ;
TRACE("		HLYBO = %08x\n ",  (unsigned int)UlSetBia ) ;
		RamWrite32A( StCaliData_UiHallBias_Y , UlSetBia) ;
#ifdef	__OIS_CLOSED_AF__
	} else {
		DacControl( 0 , HLAFBO , UlSetBia ) ;
		RamWrite32A( StCaliData_UiHallBias_AF , UlSetBia) ;
#endif
	}
TRACE("( AXIS = %02x , BIAS = %08xh ) , \n", UcTneAxs , (INT16)UlSetBia ) ;
	
	StTneVal.UlDwdVal	= TnePtp( UcTneAxs , PTP_AFTER ) ;

	return( StTneVal.UlDwdVal ) ;
}



//********************************************************************************
// Function Name 	: TneOff
// Retun Value		: Hall Top & Bottom Gaps
// Argment Value	: Hall Top & Bottom Gaps , X,Y Direction
// Explanation		: Hall Offset Tuning Function
// History			: First edition 						
//********************************************************************************
UINT32	TneOff( UnDwdVal	StTneVal, UINT8	UcTneAxs )
{
	UINT32	UlSetOff ;
	UINT32	UlSetVal ;

	
	if( UcTneAxs == X_DIR ) {
		RamRead32A( StCaliData_UiHallOffset_X , &UlSetOff ) ;
	} else if( UcTneAxs == Y_DIR ){
		RamRead32A( StCaliData_UiHallOffset_Y , &UlSetOff ) ;
#ifdef	__OIS_CLOSED_AF__
	} else {
		RamRead32A( StCaliData_UiHallOffset_AF , &UlSetOff ) ;
#endif
	}
	UlSetOff 	= ( UlSetOff >> 16 ) ;

	if ( StTneVal.StDwdVal.UsHigVal > StTneVal.StDwdVal.UsLowVal ) {
		UlSetVal	= ( UINT32 )(( StTneVal.StDwdVal.UsHigVal - StTneVal.StDwdVal.UsLowVal ) / OFFSET_DIV ) ;	// Calculating Value For Increase Step
		UlSetOff	+= UlSetVal ;	// Calculating Value For Increase Step
		if( UlSetOff > 0x0000FFFF )		UlSetOff = 0x0000FFFF ;
	} else {
		UlSetVal	= ( UINT32 )(( StTneVal.StDwdVal.UsLowVal - StTneVal.StDwdVal.UsHigVal ) / OFFSET_DIV ) ;	// Calculating Value For Decrease Step
		if( UlSetOff < UlSetVal ){
			UlSetOff	= 0x00000000 ;
		}else{
			UlSetOff	-= UlSetVal ;	// Calculating Value For Decrease Step
		}
	}

	if( UlSetOff > ( INT32 )0x0000EFFF ) {
		UlSetOff	= 0x0000EFFF ;
	} else if( UlSetOff < ( INT32 )0x00001000 ) {
		UlSetOff	= 0x00001000 ;
	}

	UlSetOff = ( UlSetOff << 16 ) ;
	
	if( UcTneAxs == X_DIR ) {
		DacControl( 0 , HLXO, UlSetOff ) ;
TRACE("		HLXO = %08x\n ",  (unsigned int)UlSetOff ) ;
		RamWrite32A( StCaliData_UiHallOffset_X , UlSetOff ) ;
	} else if( UcTneAxs == Y_DIR ){
		DacControl( 0 , HLYO, UlSetOff ) ;
TRACE("		HLYO = %08x\n ",  (unsigned int)UlSetOff ) ;
		RamWrite32A( StCaliData_UiHallOffset_Y , UlSetOff ) ;
#ifdef	__OIS_CLOSED_AF__
	} else {
		DacControl( 0 , HLAFO, UlSetOff ) ;
		RamWrite32A( StCaliData_UiHallOffset_AF , UlSetOff ) ;
#endif
	}
TRACE("( AXIS = %02x , OFST = %08xh ) , \n", UcTneAxs , (INT16)UlSetOff ) ;

	StTneVal.UlDwdVal	= TnePtp( UcTneAxs, PTP_AFTER ) ;

	return( StTneVal.UlDwdVal ) ;
}


//********************************************************************************
// Function Name 	: MesFil
// Retun Value		: NON
// Argment Value	: Measure Filter Mode
// Explanation		: Measure Filter Setting Function
// History			: First edition 		
//********************************************************************************
void	MesFil( UINT8	UcMesMod )		// 20.019kHz
{
	UINT32	UlMeasFilaA , UlMeasFilaB , UlMeasFilaC ;
	UINT32	UlMeasFilbA , UlMeasFilbB , UlMeasFilbC ;

	if( !UcMesMod ) {								// Hall Bias&Offset Adjust
		
		UlMeasFilaA	=	0x02F19B01 ;	// LPF 150Hz
		UlMeasFilaB	=	0x02F19B01 ;
		UlMeasFilaC	=	0x7A1CC9FF ;
		UlMeasFilbA	=	0x7FFFFFFF ;	// Through
		UlMeasFilbB	=	0x00000000 ;
		UlMeasFilbC	=	0x00000000 ;
		
	} else if( UcMesMod == LOOPGAIN ) {				// Loop Gain Adjust

		UlMeasFilaA	=	0x115CC757 ;	// LPF1000Hz
		UlMeasFilaB	=	0x115CC757 ;
		UlMeasFilaC	=	0x5D467153 ;
		UlMeasFilbA	=	0x7F667431 ;	// HPF30Hz
		UlMeasFilbB	=	0x80998BCF ;
		UlMeasFilbC	=	0x7ECCE863 ;
		
	} else if( UcMesMod == THROUGH ) {				// for Through

		UlMeasFilaA	=	0x7FFFFFFF ;	// Through
		UlMeasFilaB	=	0x00000000 ;
		UlMeasFilaC	=	0x00000000 ;
		UlMeasFilbA	=	0x7FFFFFFF ;	// Through
		UlMeasFilbB	=	0x00000000 ;
		UlMeasFilbC	=	0x00000000 ;

	} else if( UcMesMod == NOISE ) {				// SINE WAVE TEST for NOISE

		UlMeasFilaA	=	0x02F19B01 ;	// LPF150Hz
		UlMeasFilaB	=	0x02F19B01 ;
		UlMeasFilaC	=	0x7A1CC9FF ;
		UlMeasFilbA	=	0x02F19B01 ;	// LPF150Hz
		UlMeasFilbB	=	0x02F19B01 ;
		UlMeasFilbC	=	0x7A1CC9FF ;

	} else if(UcMesMod == OSCCHK) {
		UlMeasFilaA	=	0x05C141BB ;	// LPF300Hz
		UlMeasFilaB	=	0x05C141BB ;
		UlMeasFilaC	=	0x747D7C88 ;
		UlMeasFilbA	=	0x05C141BB ;	// LPF300Hz
		UlMeasFilbB	=	0x05C141BB ;
		UlMeasFilbC	=	0x747D7C88 ;
	}
	
	RamWrite32A ( MeasureFilterA_Coeff_a1	, UlMeasFilaA ) ;
	RamWrite32A ( MeasureFilterA_Coeff_b1	, UlMeasFilaB ) ;
	RamWrite32A ( MeasureFilterA_Coeff_c1	, UlMeasFilaC ) ;

	RamWrite32A ( MeasureFilterA_Coeff_a2	, UlMeasFilbA ) ;
	RamWrite32A ( MeasureFilterA_Coeff_b2	, UlMeasFilbB ) ;
	RamWrite32A ( MeasureFilterA_Coeff_c2	, UlMeasFilbC ) ;

	RamWrite32A ( MeasureFilterB_Coeff_a1	, UlMeasFilaA ) ;
	RamWrite32A ( MeasureFilterB_Coeff_b1	, UlMeasFilaB ) ;
	RamWrite32A ( MeasureFilterB_Coeff_c1	, UlMeasFilaC ) ;

	RamWrite32A ( MeasureFilterB_Coeff_a2	, UlMeasFilbA ) ;
	RamWrite32A ( MeasureFilterB_Coeff_b2	, UlMeasFilbB ) ;
	RamWrite32A ( MeasureFilterB_Coeff_c2	, UlMeasFilbC ) ;
}

//********************************************************************************
// Function Name 	: ClrMesFil
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Clear Measure Filter Function
// History			: First edition 						
//********************************************************************************
void	ClrMesFil( void )
{
	RamWrite32A ( MeasureFilterA_Delay_z11	, 0 ) ;
	RamWrite32A ( MeasureFilterA_Delay_z12	, 0 ) ;

	RamWrite32A ( MeasureFilterA_Delay_z21	, 0 ) ;
	RamWrite32A ( MeasureFilterA_Delay_z22	, 0 ) ;

	RamWrite32A ( MeasureFilterB_Delay_z11	, 0 ) ;
	RamWrite32A ( MeasureFilterB_Delay_z12	, 0 ) ;

	RamWrite32A ( MeasureFilterB_Delay_z21	, 0 ) ;
	RamWrite32A ( MeasureFilterB_Delay_z22	, 0 ) ;
}

//********************************************************************************
// Function Name 	: MeasureStart
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Measure start setting Function
// History			: First edition 						
//********************************************************************************
void	MeasureStart( INT32 SlMeasureParameterNum , INT32 SlMeasureParameterA , INT32 SlMeasureParameterB )
{
	MemoryClear( StMeasFunc_SiSampleNum , sizeof( MeasureFunction_Type ) ) ;
	RamWrite32A( StMeasFunc_MFA_SiMax1	 , 0x80000000 ) ;					// Set Min 
	RamWrite32A( StMeasFunc_MFB_SiMax2	 , 0x80000000 ) ;					// Set Min 
	RamWrite32A( StMeasFunc_MFA_SiMin1	 , 0x7FFFFFFF ) ;					// Set Max 
	RamWrite32A( StMeasFunc_MFB_SiMin2	 , 0x7FFFFFFF ) ;					// Set Max 
	
	SetTransDataAdr( StMeasFunc_MFA_PiMeasureRam1	 , ( UINT32 )SlMeasureParameterA ) ;		// Set Measure Filter A Ram Address
	SetTransDataAdr( StMeasFunc_MFB_PiMeasureRam2	 , ( UINT32 )SlMeasureParameterB ) ;		// Set Measure Filter B Ram Address
	RamWrite32A( StMeasFunc_SiSampleNum	 , 0 ) ;													// Clear Measure Counter 
	ClrMesFil() ;						// Clear Delay Ram
//	SetWaitTime(50) ;
	SetWaitTime(1) ;
	RamWrite32A( StMeasFunc_SiSampleMax	 , SlMeasureParameterNum ) ;						// Set Measure Max Number

}
	
//********************************************************************************
// Function Name 	: MeasureWait
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Wait complete of Measure Function
// History			: First edition 						
//********************************************************************************
void	MeasureWait( void )
{
	UINT32			SlWaitTimerSt ;
	
	SlWaitTimerSt = 1 ;
	while( SlWaitTimerSt ){
		RamRead32A( StMeasFunc_SiSampleMax , &SlWaitTimerSt ) ;
	}
}
	
//********************************************************************************
// Function Name 	: MemoryClear
// Retun Value		: NON
// Argment Value	: Top pointer , Size
// Explanation		: Memory Clear Function
// History			: First edition 						
//********************************************************************************
void	MemoryClear( UINT16 UsSourceAddress, UINT16 UsClearSize )
{
	UINT16	UsLoopIndex ;

	for ( UsLoopIndex = 0 ; UsLoopIndex < UsClearSize ; UsLoopIndex += 4 ) {
		RamWrite32A( UsSourceAddress + UsLoopIndex	, 	0x00000000 ) ;				// 4Byte
//TRACE("MEM CLR ADR = %04xh \n",UsSourceAddress + UsLoopIndex) ;
	}
}
	
//********************************************************************************
// Function Name 	: SetWaitTime
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Set Timer wait Function
// History			: First edition 						
//********************************************************************************
#define 	ONE_MSEC_COUNT	20			// 20.0195kHz * 20 ≒ 1ms
void	SetWaitTime( UINT16 UsWaitTime )
{
	RamWrite32A( WaitTimerData_UiWaitCounter	, 0 ) ;
	RamWrite32A( WaitTimerData_UiTargetCount	, (UINT32)(ONE_MSEC_COUNT * UsWaitTime)) ;
}


//********************************************************************************
// Function Name 	: LopGan
// Retun Value		: Execute Result
// Argment Value	: X,Y Direction
// Explanation		: Loop Gain Adjust Function
// History			: First edition 						
//********************************************************************************
//#define 	LOOP_NUM		2669			// 20.019kHz/0.120kHz*16times
//#define 	LOOP_FREQ		0x00C46951		// 	120Hz  = Freq * 80000000h / Fs
//#define 	LOOP_NUM		2464			// 20.019kHz/0.130kHz*16times
//#define 	LOOP_FREQ		0x00D4C9F1		// 	130Hz  = Freq * 80000000h / Fs
//#define 	LOOP_NUM		2136			// 20.019kHz/0.150kHz*16times
//#define 	LOOP_FREQ		0x00F586D9		// 	150Hz  = Freq * 80000000h / Fs
#define 	LOOP_NUM		2002			// 20.019kHz/0.160kHz*16times
#define 	LOOP_FREQ		0x0105E52C		// 	160Hz  = Freq * 80000000h / Fs
//#define 	LOOP_GAIN		0x1999999A		// -14dB
#define 	LOOP_GAIN		0x13333333		// -16.47dB
//#define 	LOOP_GAIN		0x0CCCCCCD		// -20dB
//#define 	LOOP_GAIN		0x040C3713		// -30dB
//#define 	LOOP_GAIN		0x0207567A		// -36dB

#define 	LOOP_MAX_X		SXGAIN_LOP << 1	// x2
#define 	LOOP_MIN_X		SXGAIN_LOP >> 1	// x0.5
#define 	LOOP_MAX_Y		SYGAIN_LOP << 1	// x2
#define 	LOOP_MIN_Y		SYGAIN_LOP >> 1	// x0.5

#ifdef	__OIS_CLOSED_AF__
#define 	LOOP_NUM_Z		1885			// 20.019kHz/0.170kHz*16times
#define 	LOOP_FREQ_Z		0x0116437F		// 	170Hz  = Freq * 80000000h / Fs
#define 	LOOP_GAIN_Z		0x0207567A		// -36dB
#define 	LOOP_MAX_Z		SZGAIN_LOP << 1	// x2
#define 	LOOP_MIN_Z		SZGAIN_LOP >> 1	// x0.5
#endif

UINT32	LopGan( UINT8	UcDirSel )
{
	UnllnVal		StMeasValueA , StMeasValueB ;
	INT32			SlMeasureParameterA , SlMeasureParameterB ;
	INT32			SlMeasureParameterNum ;
	UINT64	UllCalculateVal ;
	UINT32	UlReturnState ;
	UINT16	UsSinAdr ;
	UINT32	UlLopFreq , UlLopGain;
#ifdef	__OIS_CLOSED_AF__
	UINT32	UlSwitchBk ;
#endif
	
	SlMeasureParameterNum	=	(INT32)LOOP_NUM ;
	
	if( UcDirSel == X_DIR ) {		// X axis
		SlMeasureParameterA		=	HALL_RAM_HXOUT1 ;		// Set Measure RAM Address
		SlMeasureParameterB		=	HALL_RAM_HXLOP ;		// Set Measure RAM Address
		RamWrite32A( HallFilterCoeffX_hxgain0 , SXGAIN_LOP ) ;
		UsSinAdr = HALL_RAM_SINDX0;
		UlLopFreq = LOOP_FREQ;
		UlLopGain = LOOP_GAIN;
	} else if( UcDirSel == Y_DIR ){						// Y axis
		SlMeasureParameterA		=	HALL_RAM_HYOUT1 ;		// Set Measure RAM Address
		SlMeasureParameterB		=	HALL_RAM_HYLOP ;		// Set Measure RAM Address
		RamWrite32A( HallFilterCoeffY_hygain0 , SYGAIN_LOP ) ;
		UsSinAdr = HALL_RAM_SINDY0;
		UlLopFreq = LOOP_FREQ;
		UlLopGain = LOOP_GAIN;
#ifdef	__OIS_CLOSED_AF__
	} else {						// Z axis
		SlMeasureParameterNum	=	(INT32)LOOP_NUM_Z ;
		SlMeasureParameterA		=	CLAF_RAMA_AFLOP2 ;		// Set Measure RAM Address
		SlMeasureParameterB		=	CLAF_DELAY_AFPZ0 ;		// Set Measure RAM Address
		RamWrite32A( CLAF_Gain_afloop2 , SZGAIN_LOP ) ;
		RamRead32A( CLAF_RAMA_AFCNT , &UlSwitchBk ) ;
//		RamWrite32A( CLAF_RAMA_AFCNT , UlSwitchBk & 0xffffffef ) ;
		RamWrite32A( CLAF_RAMA_AFCNT , UlSwitchBk & 0xffffff0f ) ;
		UsSinAdr = CLAF_RAMA_AFCNTO;
		UlLopFreq = LOOP_FREQ_Z;
		UlLopGain = LOOP_GAIN_Z;
#endif
	}
	
	SetSinWavGenInt();

	RamWrite32A( SinWave_Offset		,	UlLopFreq ) ;								// Freq Setting
	RamWrite32A( SinWave_Gain		,	UlLopGain ) ;								// Set Sine Wave Gain					
	
	RamWrite32A( SinWaveC_Regsiter	,	0x00000001 ) ;								// Sine Wave Start

	SetTransDataAdr( SinWave_OutAddr	,	( UINT32 )UsSinAdr ) ;	// Set Sine Wave Input RAM
	
	MesFil( LOOPGAIN ) ;					// Filter setting for measurement
	
	MeasureStart( SlMeasureParameterNum , SlMeasureParameterA , SlMeasureParameterB ) ;					// Start measure
	
	MeasureWait() ;						// Wait complete of measurement

	RamRead32A( StMeasFunc_MFA_LLiAbsInteg1 		, &StMeasValueA.StUllnVal.UlLowVal ) ;	// X axis
	RamRead32A( StMeasFunc_MFA_LLiAbsInteg1 + 4 	, &StMeasValueA.StUllnVal.UlHigVal ) ;
	RamRead32A( StMeasFunc_MFB_LLiAbsInteg2 		, &StMeasValueB.StUllnVal.UlLowVal ) ;	// Y axis
	RamRead32A( StMeasFunc_MFB_LLiAbsInteg2 + 4		, &StMeasValueB.StUllnVal.UlHigVal ) ;
	
	SetSinWavGenInt();		// Sine wave stop
	
	SetTransDataAdr( SinWave_OutAddr	,	(UINT32)0x00000000 ) ;	// Set Sine Wave Input RAM
	RamWrite32A( UsSinAdr		,	0x00000000 ) ;				// DelayRam Clear
	
	if( UcDirSel == X_DIR ) {		// X axis
		UllCalculateVal = ( StMeasValueB.UllnValue * 1000 / StMeasValueA.UllnValue ) * SXGAIN_LOP / 1000 ;
		if( UllCalculateVal > (UINT64)0x000000007FFFFFFF )		UllCalculateVal = (UINT64)0x000000007FFFFFFF ;
		StAdjPar.StLopGan.UlLxgVal = (UINT32)UllCalculateVal ;
		RamWrite32A( HallFilterCoeffX_hxgain0 , StAdjPar.StLopGan.UlLxgVal ) ;
		if( UllCalculateVal > LOOP_MAX_X ){
			UlReturnState = EXE_LXADJ ;
		}else if( UllCalculateVal < LOOP_MIN_X ){
			UlReturnState = EXE_LXADJ ;
		}else{
			UlReturnState = EXE_END ;
		}
		
	}else if( UcDirSel == Y_DIR ){							// Y axis
		UllCalculateVal = ( StMeasValueB.UllnValue * 1000 / StMeasValueA.UllnValue ) * SYGAIN_LOP / 1000 ;
		if( UllCalculateVal > (UINT64)0x000000007FFFFFFF )		UllCalculateVal = (UINT64)0x000000007FFFFFFF ;
		StAdjPar.StLopGan.UlLygVal = (UINT32)UllCalculateVal ;
		RamWrite32A( HallFilterCoeffY_hygain0 , StAdjPar.StLopGan.UlLygVal ) ;
		if( UllCalculateVal > LOOP_MAX_Y ){
			UlReturnState = EXE_LYADJ ;
		}else if( UllCalculateVal < LOOP_MIN_Y ){
			UlReturnState = EXE_LYADJ ;
		}else{
			UlReturnState = EXE_END ;
		}
#ifdef	__OIS_CLOSED_AF__
	}else{							// Z axis
		UllCalculateVal = ( StMeasValueB.UllnValue * 1000 / StMeasValueA.UllnValue ) * SZGAIN_LOP / 1000 ;
		if( UllCalculateVal > (UINT64)0x000000007FFFFFFF )		UllCalculateVal = (UINT64)0x000000007FFFFFFF ;
		StAdjPar.StLopGan.UlLzgVal = (UINT32)UllCalculateVal ;
		RamWrite32A( CLAF_Gain_afloop2 , StAdjPar.StLopGan.UlLzgVal ) ;
		if( UllCalculateVal > LOOP_MAX_Z ){
			UlReturnState = EXE_LZADJ ;
		}else if( UllCalculateVal < LOOP_MIN_Z ){
			UlReturnState = EXE_LZADJ ;
		}else{
			UlReturnState = EXE_END ;
		}
		RamWrite32A( CLAF_RAMA_AFCNT , UlSwitchBk ) ;
#endif
	}
	
	return( UlReturnState ) ;

}




//********************************************************************************
// Function Name 	: TneGvc
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Tunes the Gyro VC offset
// History			: First edition 						
//********************************************************************************
#define 	GYROF_NUM		2048			// 2048times
#define 	GYROF_UPPER		0x06D6			// 
#define 	GYROF_LOWER		0xF92A			// 
UINT32	TneGvc( void )
{
	UINT32	UlRsltSts;
	INT32			SlMeasureParameterA , SlMeasureParameterB ;
	INT32			SlMeasureParameterNum ;
	UnllnVal		StMeasValueA , StMeasValueB ;
	INT32			SlMeasureAveValueA , SlMeasureAveValueB ;
	
	
	//平均値測定
	
	MesFil( THROUGH ) ;					// Set Measure Filter

	SlMeasureParameterNum	=	GYROF_NUM ;					// Measurement times
	SlMeasureParameterA		=	GYRO_RAM_GX_ADIDAT ;		// Set Measure RAM Address
	SlMeasureParameterB		=	GYRO_RAM_GY_ADIDAT ;		// Set Measure RAM Address
	
	MeasureStart( SlMeasureParameterNum , SlMeasureParameterA , SlMeasureParameterB ) ;					// Start measure
	
//	ClrMesFil();					// Clear Delay RAM
//	SetWaitTime(50) ;
//	SetWaitTime(1) ;
	
	MeasureWait() ;					// Wait complete of measurement
	
TRACE("Read Adr = %04x, %04xh \n",StMeasFunc_MFA_LLiIntegral1 + 4 , StMeasFunc_MFA_LLiIntegral1) ;
	RamRead32A( StMeasFunc_MFA_LLiIntegral1 		, &StMeasValueA.StUllnVal.UlLowVal ) ;	// X axis
	RamRead32A( StMeasFunc_MFA_LLiIntegral1 + 4		, &StMeasValueA.StUllnVal.UlHigVal ) ;
	RamRead32A( StMeasFunc_MFB_LLiIntegral2 		, &StMeasValueB.StUllnVal.UlLowVal ) ;	// Y axis
	RamRead32A( StMeasFunc_MFB_LLiIntegral2 + 4		, &StMeasValueB.StUllnVal.UlHigVal ) ;
	
TRACE("GX_OFT = %08x, %08xh \n",(unsigned int)StMeasValueA.StUllnVal.UlHigVal,(unsigned int)StMeasValueA.StUllnVal.UlLowVal) ;
TRACE("GY_OFT = %08x, %08xh \n",(unsigned int)StMeasValueB.StUllnVal.UlHigVal,(unsigned int)StMeasValueB.StUllnVal.UlLowVal) ;
	SlMeasureAveValueA = (INT32)( (INT64)StMeasValueA.UllnValue / SlMeasureParameterNum ) ;
	SlMeasureAveValueB = (INT32)( (INT64)StMeasValueB.UllnValue / SlMeasureParameterNum ) ;
TRACE("GX_AVEOFT = %08xh \n",(unsigned int)SlMeasureAveValueA) ;
TRACE("GY_AVEOFT = %08xh \n",(unsigned int)SlMeasureAveValueB) ;
	
	SlMeasureAveValueA = ( SlMeasureAveValueA >> 16 ) & 0x0000FFFF ;
	SlMeasureAveValueB = ( SlMeasureAveValueB >> 16 ) & 0x0000FFFF ;
	
	SlMeasureAveValueA = 0x00010000 - SlMeasureAveValueA ;
	SlMeasureAveValueB = 0x00010000 - SlMeasureAveValueB ;
	
	UlRsltSts = EXE_END ;
	StAdjPar.StGvcOff.UsGxoVal = ( UINT16 )( SlMeasureAveValueA & 0x0000FFFF );		//Measure Result Store
	if(( (INT16)StAdjPar.StGvcOff.UsGxoVal > (INT16)GYROF_UPPER ) || ( (INT16)StAdjPar.StGvcOff.UsGxoVal < (INT16)GYROF_LOWER )){
		UlRsltSts |= EXE_GXADJ ;
	}
	RamWrite32A( GYRO_RAM_GXOFFZ , (( SlMeasureAveValueA << 16 ) & 0xFFFF0000 ) ) ;		// X axis Gyro offset
	
	StAdjPar.StGvcOff.UsGyoVal = ( UINT16 )( SlMeasureAveValueB & 0x0000FFFF );		//Measure Result Store
	if(( (INT16)StAdjPar.StGvcOff.UsGyoVal > (INT16)GYROF_UPPER ) || ( (INT16)StAdjPar.StGvcOff.UsGyoVal < (INT16)GYROF_LOWER )){
		UlRsltSts |= EXE_GYADJ ;
	}
	RamWrite32A( GYRO_RAM_GYOFFZ , (( SlMeasureAveValueB << 16 ) & 0xFFFF0000 ) ) ;		// Y axis Gyro offset
	
TRACE("GX_AVEOFT_RV = %08xh \n",(unsigned int)SlMeasureAveValueA) ;
TRACE("GY_AVEOFT_RV = %08xh \n",(unsigned int)SlMeasureAveValueB) ;
	
	RamWrite32A( GYRO_RAM_GYROX_OFFSET , 0x00000000 ) ;			// X axis Drift Gyro offset
	RamWrite32A( GYRO_RAM_GYROY_OFFSET , 0x00000000 ) ;			// Y axis Drift Gyro offset
	RamWrite32A( GyroFilterDelayX_GXH1Z2 , 0x00000000 ) ;		// X axis H1Z2 Clear
	RamWrite32A( GyroFilterDelayY_GYH1Z2 , 0x00000000 ) ;		// Y axis H1Z2 Clear
	
	return( UlRsltSts );
	
		
}



//********************************************************************************
// Function Name 	: RtnCen
// Retun Value		: Command Status
// Argment Value	: Command Parameter
// Explanation		: Return to center Command Function
// History			: First edition 						
//********************************************************************************
UINT8	RtnCen( UINT8	UcCmdPar )
{
	UINT8	UcSndDat = 1 ;
	
	if( !UcCmdPar ){								// X,Y centering
		RamWrite32A( CMD_RETURN_TO_CENTER , BOTH_SRV_ON ) ;
	}else if( UcCmdPar == XONLY_ON ){				// only X centering
		RamWrite32A( CMD_RETURN_TO_CENTER , XAXS_SRV_ON ) ;
	}else if( UcCmdPar == YONLY_ON ){				// only Y centering
		RamWrite32A( CMD_RETURN_TO_CENTER , YAXS_SRV_ON ) ;
#ifdef	__OIS_CLOSED_AF__
	}else if( UcCmdPar == ZONLY_OFF ){				// only Z centering off
		RamWrite32A( CMD_RETURN_TO_CENTER , ZAXS_SRV_OFF ) ;
	}else if( UcCmdPar == ZONLY_ON ){				// only Z centering
		RamWrite32A( CMD_RETURN_TO_CENTER , ZAXS_SRV_ON ) ;
#endif
	}else{											// Both off
		RamWrite32A( CMD_RETURN_TO_CENTER , BOTH_SRV_OFF ) ;
	}
	
	while( UcSndDat ) {
		UcSndDat = RdStatus(1);
	}
#ifdef	__OIS_CLOSED_AF__	
	if( UcCmdPar == ZONLY_OFF ){				// only Z centering off
		RamWrite32A( CLAF_RAMA_AFOUT		,	0x00000000 ) ;				// DelayRam Clear
	}
#endif	
//TRACE("RtnCen() = %02x\n", UcSndDat ) ;
	return( UcSndDat );
}



//********************************************************************************
// Function Name 	: OisEna
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: OIS Enable Control Function
// History			: First edition 						
//********************************************************************************
void	OisEna( void )
{
	UINT8	UcStRd = 1;
	
	RamWrite32A( CMD_OIS_ENABLE , OIS_ENABLE ) ;
	while( UcStRd ) {
		UcStRd = RdStatus(1);
	}
//TRACE(" OisEna( Status) = %02x\n", UcStRd ) ;
}

//********************************************************************************
// Function Name 	: OisEnaNCL
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: OIS Enable Control Function w/o delay clear
// History			: First edition 						
//********************************************************************************
void	OisEnaNCL( void )
{
	UINT8	UcStRd = 1;
	
	RamWrite32A( CMD_OIS_ENABLE , OIS_ENA_NCL | OIS_ENABLE ) ;
	while( UcStRd ) {
		UcStRd = RdStatus(1);
	}
//TRACE(" OisEnaNCL( Status) = %02x\n", UcStRd ) ;
}

//********************************************************************************
// Function Name 	: OisEnaDrCl
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: OIS Enable Control Function w/o delay clear
// History			: First edition 						
//********************************************************************************
void	OisEnaDrCl( void )
{
	UINT8	UcStRd = 1;
	
	RamWrite32A( CMD_OIS_ENABLE , OIS_ENA_DOF | OIS_ENABLE ) ;
	while( UcStRd ) {
		UcStRd = RdStatus(1);
	}
//TRACE(" OisEnaDrCl( Status) = %02x\n", UcStRd ) ;
}

//********************************************************************************
// Function Name 	: OisEnaDrNcl
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: OIS Enable Control Function w/o delay clear
// History			: First edition 						
//********************************************************************************
void	OisEnaDrNcl( void )
{
	UINT8	UcStRd = 1;
	
	RamWrite32A( CMD_OIS_ENABLE , OIS_ENA_DOF | OIS_ENA_NCL | OIS_ENABLE ) ;
	while( UcStRd ) {
		UcStRd = RdStatus(1);
	}
//TRACE(" OisEnaDrCl( Status) = %02x\n", UcStRd ) ;
}
//********************************************************************************
// Function Name 	: OisDis
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: OIS Disable Control Function
// History			: First edition 						
//********************************************************************************
void	OisDis( void )
{
	UINT8	UcStRd = 1;
	
	RamWrite32A( CMD_OIS_ENABLE , OIS_DISABLE ) ;
	while( UcStRd ) {
		UcStRd = RdStatus(1);
	}
//TRACE(" OisDis( Status) = %02x\n", UcStRd ) ;
}


//********************************************************************************
// Function Name 	: SetRec
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: Rec Mode Enable Function
// History			: First edition 						
//********************************************************************************
void	SetRec( void )
{
	UINT8	UcStRd = 1;
	
	RamWrite32A( CMD_MOVE_STILL_MODE ,	MOVIE_MODE ) ;
	while( UcStRd ) {
		UcStRd = RdStatus(1);
	}
//TRACE(" SetRec( Status) = %02x\n", UcStRd ) ;
}


//********************************************************************************
// Function Name 	: SetStill
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: Set Still Mode Enable Function
// History			: First edition 						
//********************************************************************************
void	SetStill( void )
{
	UINT8	UcStRd = 1;
	
	RamWrite32A( CMD_MOVE_STILL_MODE ,	STILL_MODE ) ;
	while( UcStRd ) {
		UcStRd = RdStatus(1);
	}
//TRACE(" SetRec( Status) = %02x\n", UcStRd ) ;
}

//********************************************************************************
// Function Name 	: SetRecPreview
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: Rec Preview Mode Enable Function
// History			: First edition 						
//********************************************************************************
void	SetRecPreview( UINT8 mode )
{
	UINT8	UcStRd = 1;
	
	switch( mode ){
	case 0:
		RamWrite32A( CMD_MOVE_STILL_MODE ,	MOVIE_MODE ) ;
		break;
	case 1:
		RamWrite32A( CMD_MOVE_STILL_MODE ,	MOVIE_MODE1 ) ;
		break;
	case 2:
		RamWrite32A( CMD_MOVE_STILL_MODE ,	MOVIE_MODE2 ) ;
		break;
	case 3:
		RamWrite32A( CMD_MOVE_STILL_MODE ,	MOVIE_MODE3 ) ;
		break;
	}
	while( UcStRd ) {
		UcStRd = RdStatus(1);
	}
//TRACE(" SetRec( %02x ) = %02x\n", mode , UcStRd ) ;
}


//********************************************************************************
// Function Name 	: SetStillPreview
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: Set Still Preview Mode Enable Function
// History			: First edition 						
//********************************************************************************
void	SetStillPreview( unsigned char mode )
{
	UINT8	UcStRd = 1;
	
	switch( mode ){
	case 0:
		RamWrite32A( CMD_MOVE_STILL_MODE ,	STILL_MODE ) ;
		break;
	case 1:
		RamWrite32A( CMD_MOVE_STILL_MODE ,	STILL_MODE1 ) ;
		break;
	case 2:
		RamWrite32A( CMD_MOVE_STILL_MODE ,	STILL_MODE2 ) ;
		break;
	case 3:
		RamWrite32A( CMD_MOVE_STILL_MODE ,	STILL_MODE3 ) ;
		break;
	}
	while( UcStRd ) {
		UcStRd = RdStatus(1);
	}
//TRACE(" SetRec( %02x ) = %02x\n", mode , UcStRd ) ;
}



//********************************************************************************
// Function Name 	: SetSinWavePara
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Sine wave Test Function
// History			: First edition 						
//********************************************************************************
	/********* Parameter Setting *********/
	/* Servo Sampling Clock		=	20.0195kHz						*/
	/* Freq						=	SinFreq*80000000h/Fs			*/
	/* 05 00 XX MM 				XX:Freq MM:Sin or Circle */
const UINT32	CucFreqVal[ 17 ]	= {
		0xFFFFFFFF,				//  0:  Stop
		0x0001A306,				//  1: 1Hz
		0x0003460B,				//  2: 2Hz
		0x0004E911,				//  3: 3Hz	
		0x00068C16,				//  4: 4Hz
		0x00082F1C,				//  5: 5Hz
		0x0009D222,				//  6: 6Hz
		0x000B7527,				//  7: 7Hz
		0x000D182D,				//  8: 8Hz
		0x000EBB32,				//  9: 9Hz
		0x00105E38,				//  A: 10Hz
		0x0012013E,				//  B: 11Hz
		0x0013A443,				//  C: 12Hz
		0x00154749,				//  D: 13Hz
		0x0016EA4E,				//  E: 14Hz
		0x00188D54,				//  F: 15Hz
		0x001A305A				// 10: 16Hz
	} ;
	
// 	RamWrite32A( SinWave.Gain , 0x00000000 ) ;			// Gainはそれぞれ設定すること
// 	RamWrite32A( CosWave.Gain , 0x00000000 ) ;			// Gainはそれぞれ設定すること
void	SetSinWavePara( UINT8 UcTableVal ,  UINT8 UcMethodVal )
{
	UINT32	UlFreqDat ;

	
	if(UcTableVal > 0x10 )
		UcTableVal = 0x10 ;			/* Limit */
	UlFreqDat = CucFreqVal[ UcTableVal ] ;	
	
	if( UcMethodVal == CIRCWAVE) {
		RamWrite32A( SinWave_Phase	,	0x00000000 ) ;		// 正弦波の位相量
		RamWrite32A( CosWave_Phase 	,	0x20000000 );		// 正弦波の位相量
	}else{
		RamWrite32A( SinWave_Phase	,	0x00000000 ) ;		// 正弦波の位相量
		RamWrite32A( CosWave_Phase 	,	0x00000000 );		// 正弦波の位相量
	}


	if( UlFreqDat == 0xFFFFFFFF )			/* Sine波中止 */
	{
		RamWrite32A( SinWave_Offset		,	0x00000000 ) ;									// 発生周波数のオフセットを設定
		RamWrite32A( SinWave_Phase		,	0x00000000 ) ;									// 正弦波の位相量
//		RamWrite32A( SinWave_Gain		,	0x00000000 ) ;									// 発生周波数のアッテネータ(初期値は0[dB])
//		SetTransDataAdr( SinWave_OutAddr	,	 (UINT32)SinWave_Output );			// 出力先アドレス

		RamWrite32A( CosWave_Offset		,	0x00000000 );									// 発生周波数のオフセットを設定
		RamWrite32A( CosWave_Phase 		,	0x00000000 );									// 正弦波の位相量
//		RamWrite32A( CosWave_Gain 		,	0x00000000 );									// 発生周波数のアッテネータ(初期値はCut)
//		SetTransDataAdr( CosWave_OutAddr	,	 (UINT32)CosWave_Output );			// 出力先アドレス

		RamWrite32A( SinWaveC_Regsiter	,	0x00000000 ) ;									// Sine Wave Stop
		SetTransDataAdr( SinWave_OutAddr	,	0x00000000 ) ;		// 出力先アドレス
		SetTransDataAdr( CosWave_OutAddr	,	0x00000000 );		// 出力先アドレス
		RamWrite32A( HALL_RAM_HXOFF1		,	0x00000000 ) ;				// DelayRam Clear
		RamWrite32A( HALL_RAM_HYOFF1		,	0x00000000 ) ;				// DelayRam Clear
	}
	else
	{
		RamWrite32A( SinWave_Offset		,	UlFreqDat ) ;									// 発生周波数のオフセットを設定
		RamWrite32A( CosWave_Offset		,	UlFreqDat );									// 発生周波数のオフセットを設定

		RamWrite32A( SinWaveC_Regsiter	,	0x00000001 ) ;									// Sine Wave Start
		SetTransDataAdr( SinWave_OutAddr	,	(UINT32)HALL_RAM_HXOFF1 ) ;		// 出力先アドレス
		SetTransDataAdr( CosWave_OutAddr	,	(UINT32)HALL_RAM_HYOFF1 ) ;		// 出力先アドレス

	}
	
	
}




//********************************************************************************
// Function Name 	: SetPanTiltMode
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Pan-Tilt Enable/Disable
// History			: First edition 						
//********************************************************************************
void	SetPanTiltMode( UINT8 UcPnTmod )
{
	UINT8	UcStRd = 1;
	
	switch ( UcPnTmod ) {
		case OFF :
			RamWrite32A( CMD_PAN_TILT ,	PAN_TILT_OFF ) ;
//TRACE(" PanTilt OFF\n");
			break ;
		case ON :
			RamWrite32A( CMD_PAN_TILT ,	PAN_TILT_ON ) ;
//TRACE(" PanTilt ON\n");
			break ;
	}
	while( UcStRd ) {
		UcStRd = RdStatus(1);
	}
//TRACE(" PanTilt( Status) = %02x , %02x \n", UcStRd , UcPnTmod ) ;

}



 #ifdef	NEUTRAL_CENTER
//********************************************************************************
// Function Name 	: TneHvc
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Tunes the Hall VC offset
// History			: First edition 				
//********************************************************************************
UINT8	TneHvc( void )
{
	UINT8	UcRsltSts;
	INT32			SlMeasureParameterA , SlMeasureParameterB ;
	INT32			SlMeasureParameterNum ;
	UnllnVal		StMeasValueA , StMeasValueB ;
	INT32			SlMeasureAveValueA , SlMeasureAveValueB ;
	
	RtnCen( BOTH_OFF ) ;		// Both OFF
	
	WitTim( 500 ) ;
	
	//平均値測定
	
	MesFil( THROUGH ) ;					// Set Measure Filter

	SlMeasureParameterNum	=	64 ;		// 64times
	SlMeasureParameterA		=	(UINT32)HALL_RAM_HXIDAT ;		// Set Measure RAM Address
	SlMeasureParameterB		=	(UINT32)HALL_RAM_HYIDAT ;		// Set Measure RAM Address

	MeasureStart( SlMeasureParameterNum , SlMeasureParameterA , SlMeasureParameterB ) ;					// Start measure

//	ClrMesFil();					// Clear Delay RAM
//	SetWaitTime(50) ;

	MeasureWait() ;					// Wait complete of measurement

	RamRead32A( StMeasFunc_MFA_LLiIntegral1 		, &StMeasValueA.StUllnVal.UlLowVal ) ;	// X axis
	RamRead32A( StMeasFunc_MFA_LLiIntegral1 + 4 	, &StMeasValueA.StUllnVal.UlHigVal ) ;
	RamRead32A( StMeasFunc_MFB_LLiIntegral2 		, &StMeasValueB.StUllnVal.UlLowVal ) ;	// Y axis
	RamRead32A( StMeasFunc_MFB_LLiIntegral2 + 4	, &StMeasValueB.StUllnVal.UlHigVal ) ;

	SlMeasureAveValueA = (INT32)((( (INT64)StMeasValueA.UllnValue * 100 ) / SlMeasureParameterNum ) / 100 ) ;
	SlMeasureAveValueB = (INT32)((( (INT64)StMeasValueB.UllnValue * 100 ) / SlMeasureParameterNum ) / 100 ) ;

	StAdjPar.StHalAdj.UsHlxCna = ( UINT16 )(( SlMeasureAveValueA >> 16 ) & 0x0000FFFF );		//Measure Result Store
	StAdjPar.StHalAdj.UsHlxCen = StAdjPar.StHalAdj.UsHlxCna;											//Measure Result Store

	StAdjPar.StHalAdj.UsHlyCna = ( UINT16 )(( SlMeasureAveValueB >> 16 ) & 0x0000FFFF );		//Measure Result Store
	StAdjPar.StHalAdj.UsHlyCen = StAdjPar.StHalAdj.UsHlyCna;											//Measure Result Store

	UcRsltSts = EXE_END ;				// Clear Status

	return( UcRsltSts );
}
 #endif	//NEUTRAL_CENTER

 #ifdef	NEUTRAL_CENTER_FINE
//********************************************************************************
// Function Name 	: TneFin
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Tunes the Hall VC offset current optimize
// History			: First edition 				
//********************************************************************************
void	TneFin( void )
{
	UINT32	UlReadVal ;
	UINT16	UsAdxOff, UsAdyOff ;
	INT32			SlMeasureParameterNum ;
//	INT32			SlMeasureMaxValueA , SlMeasureMaxValueB ;
//	INT32			SlMeasureMinValueA , SlMeasureMinValueB ;
//	INT32			SlMeasureAmpValueA , SlMeasureAmpValueB ;
	INT32			SlMeasureAveValueA , SlMeasureAveValueB ;
	UnllnVal		StMeasValueA , StMeasValueB ;
	UINT32	UlMinimumValueA, UlMinimumValueB ;
	UINT16	UsAdxMin, UsAdyMin ;
	UINT8	UcFin ;
	
	// Get natural center offset
	RamRead32A( HALL_RAM_HXOFF,  &UlReadVal ) ;
	UsAdxOff = UsAdxMin = (UINT16)( UlReadVal >> 16 ) ;

	RamRead32A( HALL_RAM_HYOFF,  &UlReadVal ) ;
	UsAdyOff = UsAdyMin = (UINT16)( UlReadVal >> 16 ) ;
//TRACE("*****************************************************\n" );
//TRACE("TneFin: Before Adx=%04X, Ady=%04X\n", UsAdxOff, UsAdyOff );

	// Servo ON
	RtnCen( BOTH_ON ) ;
	WitTim( TNE ) ;

	MesFil( THROUGH ) ;					// Filter setting for measurement

	SlMeasureParameterNum = 2000 ;

	MeasureStart( SlMeasureParameterNum , HALL_RAM_HALL_X_OUT , HALL_RAM_HALL_Y_OUT ) ;					// Start measure

	MeasureWait() ;						// Wait complete of measurement

//	RamRead32A( StMeasFunc_MFA_SiMax1 , ( UINT32 * )&SlMeasureMaxValueA ) ;		// Max value
//	RamRead32A( StMeasFunc_MFA_SiMin1 , ( UINT32 * )&SlMeasureMinValueA ) ;		// Min value
//	RamRead32A( StMeasFunc_MFA_UiAmp1 , ( UINT32 * )&SlMeasureAmpValueA ) ;		// Amp value
	RamRead32A( StMeasFunc_MFA_LLiIntegral1 	, &StMeasValueA.StUllnVal.UlLowVal ) ;	// X axis
	RamRead32A( StMeasFunc_MFA_LLiIntegral1 + 4 , &StMeasValueA.StUllnVal.UlHigVal ) ;
	SlMeasureAveValueA = (INT32)((( (INT64)StMeasValueA.UllnValue * 100 ) / SlMeasureParameterNum ) / 100 ) ;

//	RamRead32A( StMeasFunc_MFB_SiMax2 , ( UINT32 * )&SlMeasureMaxValueB ) ;	// Max value
//	RamRead32A( StMeasFunc_MFB_SiMin2 , ( UINT32 * )&SlMeasureMinValueB ) ;	// Min value
//	RamRead32A( StMeasFunc_MFB_UiAmp2 , ( UINT32 * )&SlMeasureAmpValueB ) ;		// Amp value
	RamRead32A( StMeasFunc_MFB_LLiIntegral2 	, &StMeasValueB.StUllnVal.UlLowVal ) ;	// Y axis
	RamRead32A( StMeasFunc_MFB_LLiIntegral2 + 4	, &StMeasValueB.StUllnVal.UlHigVal ) ;
	SlMeasureAveValueB = (INT32)((( (INT64)StMeasValueB.UllnValue * 100 ) / SlMeasureParameterNum ) / 100 ) ;




	UlMinimumValueA = abs(SlMeasureAveValueA) ;
	UlMinimumValueB = abs(SlMeasureAveValueB) ;
	UcFin = 0x11 ;

	while( UcFin ) {
		if( UcFin & 0x01 ) {
			if( UlMinimumValueA >= abs(SlMeasureAveValueA) ) {
				UlMinimumValueA = abs(SlMeasureAveValueA) ;
				UsAdxMin = UsAdxOff ;
				// 収束を早めるために、出力値に比例させる
				if( SlMeasureAveValueA > 0 )
					UsAdxOff = (INT16)UsAdxOff + (SlMeasureAveValueA >> 17) + 1 ;
				else
					UsAdxOff = (INT16)UsAdxOff + (SlMeasureAveValueA >> 17) - 1 ;

				RamWrite32A( HALL_RAM_HXOFF,  (UINT32)((UsAdxOff << 16 ) & 0xFFFF0000 )) ;
			} else {
//TRACE("X fine\n");
				UcFin &= 0xFE ;
			}
		}

		if( UcFin & 0x10 ) {
			if( UlMinimumValueB >= abs(SlMeasureAveValueB) ) {
				UlMinimumValueB = abs(SlMeasureAveValueB) ;
				UsAdyMin = UsAdyOff ;
				// 収束を早めるために、出力値に比例させる
				if( SlMeasureAveValueB > 0 )
					UsAdyOff = (INT16)UsAdyOff + (SlMeasureAveValueB >> 17) + 1 ;
				else
					UsAdyOff = (INT16)UsAdyOff + (SlMeasureAveValueB >> 17) - 1 ;

				RamWrite32A( HALL_RAM_HYOFF,  (UINT32)((UsAdyOff << 16 ) & 0xFFFF0000 )) ;
			} else {
//TRACE("Y fine\n");
				UcFin &= 0xEF ;
			}
		}
		
		MeasureStart( SlMeasureParameterNum , HALL_RAM_HALL_X_OUT , HALL_RAM_HALL_Y_OUT ) ;					// Start measure
		MeasureWait() ;						// Wait complete of measurement

//		RamRead32A( StMeasFunc_MFA_SiMax1 , ( UINT32 * )&SlMeasureMaxValueA ) ;		// Max value
//		RamRead32A( StMeasFunc_MFA_SiMin1 , ( UINT32 * )&SlMeasureMinValueA ) ;		// Min value
//		RamRead32A( StMeasFunc_MFA_UiAmp1 , ( UINT32 * )&SlMeasureAmpValueA ) ;		// Amp value
		RamRead32A( StMeasFunc_MFA_LLiIntegral1 	, &StMeasValueA.StUllnVal.UlLowVal ) ;	// X axis
		RamRead32A( StMeasFunc_MFA_LLiIntegral1 + 4 , &StMeasValueA.StUllnVal.UlHigVal ) ;
		SlMeasureAveValueA = (INT32)((( (INT64)StMeasValueA.UllnValue * 100 ) / SlMeasureParameterNum ) / 100 ) ;

//		RamRead32A( StMeasFunc_MFB_SiMax2 , ( UINT32 * )&SlMeasureMaxValueB ) ;	// Max value
//		RamRead32A( StMeasFunc_MFB_SiMin2 , ( UINT32 * )&SlMeasureMinValueB ) ;	// Min value
//		RamRead32A( StMeasFunc_MFB_UiAmp2 , ( UINT32 * )&SlMeasureAmpValueB ) ;		// Amp value
		RamRead32A( StMeasFunc_MFB_LLiIntegral2 	, &StMeasValueB.StUllnVal.UlLowVal ) ;	// Y axis
		RamRead32A( StMeasFunc_MFB_LLiIntegral2 + 4	, &StMeasValueB.StUllnVal.UlHigVal ) ;
		SlMeasureAveValueB = (INT32)((( (INT64)StMeasValueB.UllnValue * 100 ) / SlMeasureParameterNum ) / 100 ) ;

//TRACE("-->Adx %04X, Ady %04X\n", UsAdxOff, UsAdyOff );


	}	// while


//TRACE("TneFin: After Adx=%04X, Ady=%04X\n", UsAdxMin, UsAdyMin );


	StAdjPar.StHalAdj.UsHlxCna = UsAdxMin;								//Measure Result Store
	StAdjPar.StHalAdj.UsHlxCen = StAdjPar.StHalAdj.UsHlxCna;			//Measure Result Store

	StAdjPar.StHalAdj.UsHlyCna = UsAdyMin;								//Measure Result Store
	StAdjPar.StHalAdj.UsHlyCen = StAdjPar.StHalAdj.UsHlyCna;			//Measure Result Store

	StAdjPar.StHalAdj.UsAdxOff = StAdjPar.StHalAdj.UsHlxCna  ;
	StAdjPar.StHalAdj.UsAdyOff = StAdjPar.StHalAdj.UsHlyCna  ;

	// Servo OFF
	RtnCen( BOTH_OFF ) ;		// Both OFF

}
 #endif	//NEUTRAL_CENTER_FINE

//********************************************************************************
// Function Name 	: IniNvc
// Retun Value		: NON
// Argment Value	: direction
// Explanation		: Set each direction sign function
//********************************************************************************
void	IniNvc( INT16 SsX, INT16 SsY )
{
	SsNvcX = SsX ;
	SsNvcY = SsY ;
}

//********************************************************************************
// Function Name 	: TneSltPos
// Retun Value		: NON
// Argment Value	: Position number(1, 2, 3, 4, 5, 6, 7, 0:reset)
// Explanation		: Move measurement position function
//********************************************************************************
void	TneSltPos( UINT8 UcPos )
{
	INT16 SsOff = 0x0000 ;

	UcPos &= 0x07 ;
	
	if ( UcPos ) {
		SsOff = SLT_OFFSET * (UcPos - 4);
	}

//TRACE("X = %04X, Y = %04X \n", SsOff, SsOff );

	RamWrite32A( HALL_RAM_HXOFF1,  (INT32)((SsOff * SsNvcX) << 16) ) ;
	RamWrite32A( HALL_RAM_HYOFF1,  (INT32)((SsOff * SsNvcY) << 16) ) ;

}

//********************************************************************************
// Function Name 	: TneVrtPos
// Retun Value		: NON
// Argment Value	: Position number(1, 2, 3, 4, 5, 6, 7, 0:reset)
// Explanation		: Move measurement position function
//********************************************************************************
void	TneVrtPos( UINT8 UcPos )
{
	INT16 SsOff = 0x0000 ;

	UcPos &= 0x07 ;
	
	if ( UcPos ) {
		SsOff = SLT_OFFSET * (UcPos - 4);
	}

//TRACE("X = %04X, Y = %04X \n", SsOff, SsOff );

	RamWrite32A( HALL_RAM_HXOFF1,  (INT32)0 ) ;
	RamWrite32A( HALL_RAM_HYOFF1,  (INT32)((SsOff * SsNvcY) << 16) ) ;
}

//********************************************************************************
// Function Name 	: TneHrzPos
// Retun Value		: NON
// Argment Value	: Position number(1, 2, 3, 4, 5, 6, 7, 0:reset)
// Explanation		: Move measurement position function
//********************************************************************************
void	TneHrzPos( UINT8 UcPos )
{
	INT16 SsOff = 0x0000 ;

	UcPos &= 0x07 ;
	
	if ( UcPos ) {
		SsOff = SLT_OFFSET * (UcPos - 4);
	}

//TRACE("X = %04X, Y = %04X \n", SsOff, SsOff );

	RamWrite32A( HALL_RAM_HXOFF1,  (INT32)((SsOff * SsNvcX) << 16) ) ;
	RamWrite32A( HALL_RAM_HYOFF1,  (INT32)0 ) ;
}

//********************************************************************************
// Function Name 	: SetSinWavGenInt
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Sine wave generator initial Function
// History			: First edition 						
//********************************************************************************
void	SetSinWavGenInt( void )
{
	
	RamWrite32A( SinWave_Offset		,	0x00000000 ) ;		// 発生周波数のオフセットを設定
	RamWrite32A( SinWave_Phase		,	0x00000000 ) ;		// 正弦波の位相量
	RamWrite32A( SinWave_Gain		,	0x00000000 ) ;		// 発生周波数のアッテネータ(初期値は0[dB])
//	RamWrite32A( SinWave_Gain		,	0x7FFFFFFF ) ;		// 発生周波数のアッテネータ(初期値はCut)
//	SetTransDataAdr( SinWave_OutAddr	,	(UINT32)SinWave_Output ) ;		// 初期値の出力先アドレスは、自分のメンバ

	RamWrite32A( CosWave_Offset		,	0x00000000 );		// 発生周波数のオフセットを設定
	RamWrite32A( CosWave_Phase 		,	0x20000000 );		// 正弦波の位相量
	RamWrite32A( CosWave_Gain 		,	0x00000000 );		// 発生周波数のアッテネータ(初期値はCut)
//	RamWrite32A( CosWave_Gain 		,	0x7FFFFFFF );		// 発生周波数のアッテネータ(初期値は0[dB])
//	SetTransDataAdr( CosWave_OutAddr	,	(UINT32)CosWave_Output );		// 初期値の出力先アドレスは、自分のメンバ
	
	RamWrite32A( SinWaveC_Regsiter	,	0x00000000 ) ;								// Sine Wave Stop
	
}


//********************************************************************************
// Function Name 	: SetTransDataAdr
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Trans Address for Data Function
// History			: First edition 						
//********************************************************************************
void	SetTransDataAdr( UINT16 UsLowAddress , UINT32 UlLowAdrBeforeTrans )
{
	UnDwdVal	StTrsVal ;
	
	if( UlLowAdrBeforeTrans < 0x00009000 ){
		StTrsVal.UlDwdVal = UlLowAdrBeforeTrans ;
	}else{
		StTrsVal.StDwdVal.UsHigVal = (UINT16)(( UlLowAdrBeforeTrans & 0x0000F000 ) >> 8 ) ;
		StTrsVal.StDwdVal.UsLowVal = (UINT16)( UlLowAdrBeforeTrans & 0x00000FFF ) ;
	}
//TRACE(" TRANS  ADR = %04xh , DAT = %08xh \n",UsLowAddress , StTrsVal.UlDwdVal ) ;
	RamWrite32A( UsLowAddress	,	StTrsVal.UlDwdVal );
	
}


//********************************************************************************
// Function Name 	: RdStatus
// Retun Value		: 0:success 1:FAILURE
// Argment Value	: bit check  0:ALL  1:bit24
// Explanation		: High level status check Function
// History			: First edition 						
//********************************************************************************
UINT8	RdStatus( UINT8 UcStBitChk )
{
	UINT32	UlReadVal ;
	
	RamRead32A( CMD_READ_STATUS , &UlReadVal );
//TRACE(" (Rd St) = %08x\n", (unsigned INT16)UlReadVal ) ;
	if( UcStBitChk ){
		UlReadVal &= READ_STATUS_INI ;
	}
	if( !UlReadVal ){
		return( SUCCESS );
	}else{
		return( FAILURE );
	}
}


//********************************************************************************
// Function Name 	: DacControl
// Retun Value		: Firmware version
// Argment Value	: NON
// Explanation		: Dac Control Function
// History			: First edition 						
//********************************************************************************
void	DacControl( UINT8 UcMode, UINT32 UiChannel, UINT32 PuiData )
{
	UINT32	UlAddaInt ;
	if( !UcMode ) {
		RamWrite32A( CMD_IO_ADR_ACCESS , ADDA_DASEL ) ;
		RamWrite32A( CMD_IO_DAT_ACCESS , UiChannel ) ;
		RamWrite32A( CMD_IO_ADR_ACCESS , ADDA_DAO ) ;
		RamWrite32A( CMD_IO_DAT_ACCESS , PuiData ) ;
		;
		;
		UlAddaInt = 0x00000040 ;
		while ( (UlAddaInt & 0x00000040) != 0 ) {
			RamWrite32A( CMD_IO_ADR_ACCESS , ADDA_ADDAINT ) ;
			RamRead32A(  CMD_IO_DAT_ACCESS , &UlAddaInt ) ;
			; 
		}
	} else {
		RamWrite32A( CMD_IO_ADR_ACCESS , ADDA_DASEL ) ;
		RamWrite32A( CMD_IO_DAT_ACCESS , UiChannel ) ;
		RamWrite32A( CMD_IO_ADR_ACCESS , ADDA_DAO ) ;
		RamRead32A(  CMD_IO_DAT_ACCESS , &PuiData ) ;
		;
		;
		UlAddaInt = 0x00000040 ;
		while ( (UlAddaInt & 0x00000040) != 0 ) {
			RamWrite32A( CMD_IO_ADR_ACCESS , ADDA_ADDAINT ) ;
			RamRead32A(  CMD_IO_DAT_ACCESS , &UlAddaInt ) ;
			;
		}
	}

	return ;
}

//********************************************************************************
// Function Name 	: WrHallCalData
// Retun Value		: 0:OK, 1:NG
// Argment Value	: NON
// Explanation		: Flash Write Hall Calibration Data Function
// History			: First edition 						
//********************************************************************************
INT16	WrHallCalData( void )
{
	UINT32	UlReadVal ;
	INT16 iRetVal;

	RamRead32A(  StCaliData_UsCalibrationStatus , &UlReadVal ) ;
#ifdef	__OIS_CLOSED_AF__
	UlReadVal &= ~( HALL_CALB_FLG | CLAF_CALB_FLG | HALL_CALB_BIT );
#else
	UlReadVal &= ~( HALL_CALB_FLG | HALL_CALB_BIT );
#endif
	UlReadVal |= StAdjPar.StHalAdj.UlAdjPhs ;
	RamWrite32A( StCaliData_UsCalibrationStatus , 	UlReadVal ) ;

	RamWrite32A( StCaliData_SiHallMax_Before_X ,	(UINT32)(StAdjPar.StHalAdj.UsHlxMax << 16)) ;
	RamWrite32A( StCaliData_SiHallMin_Before_X ,	(UINT32)(StAdjPar.StHalAdj.UsHlxMin << 16)) ;
	RamWrite32A( StCaliData_SiHallMax_After_X ,		(UINT32)(StAdjPar.StHalAdj.UsHlxMxa << 16)) ;
	RamWrite32A( StCaliData_SiHallMin_After_X ,		(UINT32)(StAdjPar.StHalAdj.UsHlxMna << 16)) ;
	RamWrite32A( StCaliData_SiHallMax_Before_Y ,	(UINT32)(StAdjPar.StHalAdj.UsHlyMax << 16)) ;
	RamWrite32A( StCaliData_SiHallMin_Before_Y ,	(UINT32)(StAdjPar.StHalAdj.UsHlyMin << 16)) ;
	RamWrite32A( StCaliData_SiHallMax_After_Y ,		(UINT32)(StAdjPar.StHalAdj.UsHlyMxa << 16)) ;
	RamWrite32A( StCaliData_SiHallMin_After_Y ,		(UINT32)(StAdjPar.StHalAdj.UsHlyMna << 16)) ;
	RamWrite32A( StCaliData_UiHallBias_X ,			(UINT32)(StAdjPar.StHalAdj.UsHlxGan << 16)) ;
	RamWrite32A( StCaliData_UiHallOffset_X ,		(UINT32)(StAdjPar.StHalAdj.UsHlxOff << 16)) ;
	RamWrite32A( StCaliData_UiHallBias_Y ,			(UINT32)(StAdjPar.StHalAdj.UsHlyGan << 16)) ;
	RamWrite32A( StCaliData_UiHallOffset_Y ,		(UINT32)(StAdjPar.StHalAdj.UsHlyOff << 16)) ;
	
	RamWrite32A( StCaliData_SiLoopGain_X ,			StAdjPar.StLopGan.UlLxgVal ) ;
	RamWrite32A( StCaliData_SiLoopGain_Y ,			StAdjPar.StLopGan.UlLygVal ) ;
	RamWrite32A( StCaliData_SiLensCen_Offset_X ,	(UINT32)(StAdjPar.StHalAdj.UsAdxOff << 16)) ;
	RamWrite32A( StCaliData_SiLensCen_Offset_Y ,	(UINT32)(StAdjPar.StHalAdj.UsAdyOff << 16)) ;
	RamWrite32A( StCaliData_SiOtpCen_Offset_X ,		0L ) ;
	RamWrite32A( StCaliData_SiOtpCen_Offset_Y ,		0L ) ;
	RamWrite32A( StCaliData_SiGyroOffset_X ,		(UINT32)(StAdjPar.StGvcOff.UsGxoVal << 16)) ;
	RamWrite32A( StCaliData_SiGyroOffset_Y ,		(UINT32)(StAdjPar.StGvcOff.UsGyoVal << 16)) ;
	
#ifdef	__OIS_CLOSED_AF__
	RamWrite32A( StCalDatAd_SiHallMax_Before_Z ,	(UINT32)(StAdjPar.StHalAdj.UsHlzMax << 16)) ;
	RamWrite32A( StCalDatAd_SiHallMin_Before_Z ,	(UINT32)(StAdjPar.StHalAdj.UsHlzMin << 16)) ;
	RamWrite32A( StCalDatAd_SiHallMax_After_Z ,		(UINT32)(StAdjPar.StHalAdj.UsHlzMxa << 16)) ;
	RamWrite32A( StCalDatAd_SiHallMin_After_Z ,		(UINT32)(StAdjPar.StHalAdj.UsHlzMna << 16)) ;
	RamWrite32A( StCaliData_UiHallBias_AF ,			(UINT32)(StAdjPar.StHalAdj.UsHlzGan << 16)) ;
	RamWrite32A( StCaliData_UiHallOffset_AF ,		(UINT32)(StAdjPar.StHalAdj.UsHlzOff << 16)) ;
	RamWrite32A( StCaliData_SiAD_Offset_AF ,		(UINT32)(StAdjPar.StHalAdj.UsAdzOff << 16)) ;
	RamWrite32A( StCaliData_SiLoopGain_AF ,			StAdjPar.StLopGan.UlLzgVal ) ;
#endif

	// Back Up Read calibration sector to buffer
	iRetVal = Calibration_VerifyUpdate_PreRead();
	if(iRetVal != 0) return(iRetVal);
	
	_PUT_UINT32( UlReadVal,										CALIBRATION_STATUS	) ;
	_PUT_UINT32( (UINT32)(StAdjPar.StHalAdj.UsHlxMax << 16),	HALL_MAX_BEFORE_X	) ;
	_PUT_UINT32( (UINT32)(StAdjPar.StHalAdj.UsHlxMin << 16),	HALL_MIN_BEFORE_X	) ;
	_PUT_UINT32( (UINT32)(StAdjPar.StHalAdj.UsHlxMxa << 16),	HALL_MAX_AFTER_X	) ;
	_PUT_UINT32( (UINT32)(StAdjPar.StHalAdj.UsHlxMna << 16),	HALL_MIN_AFTER_X	) ;
	_PUT_UINT32( (UINT32)(StAdjPar.StHalAdj.UsHlyMax << 16),	HALL_MAX_BEFORE_Y	) ;
	_PUT_UINT32( (UINT32)(StAdjPar.StHalAdj.UsHlyMin << 16),	HALL_MIN_BEFORE_Y	) ;
	_PUT_UINT32( (UINT32)(StAdjPar.StHalAdj.UsHlyMxa << 16),	HALL_MAX_AFTER_Y	) ;
	_PUT_UINT32( (UINT32)(StAdjPar.StHalAdj.UsHlyMna << 16),	HALL_MIN_AFTER_Y	) ;
	_PUT_UINT32( (UINT32)(StAdjPar.StHalAdj.UsHlxGan << 16),	HALL_BIAS_DAC_X		) ;
	_PUT_UINT32( (UINT32)(StAdjPar.StHalAdj.UsHlxOff << 16),	HALL_OFFSET_DAC_X	) ;
	_PUT_UINT32( (UINT32)(StAdjPar.StHalAdj.UsHlyGan << 16),	HALL_BIAS_DAC_Y		) ;
	_PUT_UINT32( (UINT32)(StAdjPar.StHalAdj.UsHlyOff << 16),	HALL_OFFSET_DAC_Y	) ;
	
	_PUT_UINT32( StAdjPar.StLopGan.UlLxgVal, 					LOOP_GAIN_VALUE_X	) ;
	_PUT_UINT32( StAdjPar.StLopGan.UlLygVal,					LOOP_GAIN_VALUE_Y	) ;
	_PUT_UINT32( (UINT32)(StAdjPar.StHalAdj.UsAdxOff << 16),	LENS_CENTER_VALUE_X	) ;
	_PUT_UINT32( (UINT32)(StAdjPar.StHalAdj.UsAdyOff << 16),	LENS_CENTER_VALUE_Y	) ;
	_PUT_UINT32( 0L,											OPT_CENTER_VALUE_X	) ;
	_PUT_UINT32( 0L,											OPT_CENTER_VALUE_Y	) ;
	_PUT_UINT32( (UINT32)(StAdjPar.StGvcOff.UsGxoVal << 16),	GYRO_OFFSET_VALUE_X	) ;
	_PUT_UINT32( (UINT32)(StAdjPar.StGvcOff.UsGyoVal << 16),	GYRO_OFFSET_VALUE_Y	) ;

#ifdef	__OIS_CLOSED_AF__
	_PUT_UINT32( (UINT32)(StAdjPar.StHalAdj.UsHlzMax << 16),	HALL_MAX_BEFORE_Z	) ;
	_PUT_UINT32( (UINT32)(StAdjPar.StHalAdj.UsHlzMin << 16),	HALL_MIN_BEFORE_Z	) ;
	_PUT_UINT32( (UINT32)(StAdjPar.StHalAdj.UsHlzMxa << 16),	HALL_MAX_AFTER_Z	) ;
	_PUT_UINT32( (UINT32)(StAdjPar.StHalAdj.UsHlzMna << 16),	HALL_MIN_AFTER_Z	) ;
	_PUT_UINT32( (UINT32)(StAdjPar.StHalAdj.UsHlzGan << 16),	HALL_BIAS_DAC_AF	) ;
	_PUT_UINT32( (UINT32)(StAdjPar.StHalAdj.UsHlzOff << 16),	HALL_OFFSET_DAC_AF	) ;
	_PUT_UINT32( (UINT32)(StAdjPar.StHalAdj.UsAdzOff << 16),	LENS_CENTER_VALUE_AF) ;
	_PUT_UINT32( StAdjPar.StLopGan.UlLzgVal,							LOOP_GAIN_VALUE_AF	) ;
#endif

	// Flash update procedure
	iRetVal = Calibration_VerifyUpdate();

	return iRetVal;
}

//********************************************************************************
// Function Name 	: WrGyroGainData
// Retun Value		: 0:OK, 1:NG
// Argment Value	: NON
// Explanation		: Flash Write Hall Calibration Data Function
// History			: First edition 						
//********************************************************************************
INT16	WrGyroGainData( void )
{
	UINT32	UlReadVal ;
	UINT32	UlZoomX, UlZoomY;
	INT16 iRetVal;

	RamRead32A(  StCaliData_UsCalibrationStatus , &UlReadVal ) ;
	UlReadVal &= ~GYRO_GAIN_FLG;
	RamWrite32A( StCaliData_UsCalibrationStatus , 	UlReadVal ) ;
	
	RamRead32A(  GyroFilterTableX_gxzoom , &UlZoomX ) ;
	RamWrite32A( StCaliData_SiGyroGain_X ,	UlZoomX ) ;
	
	RamRead32A(  GyroFilterTableY_gyzoom , &UlZoomY ) ;
	RamWrite32A( StCaliData_SiGyroGain_Y ,	UlZoomY ) ;
	
	// Read calibration sector to buffer
	iRetVal = Calibration_VerifyUpdate_PreRead();
	if(iRetVal != 0) return(iRetVal);

	_PUT_UINT32( UlReadVal,											CALIBRATION_STATUS	) ;
	_PUT_UINT32( UlZoomX,											GYRO_GAIN_VALUE_X	) ;
	_PUT_UINT32( UlZoomY,											GYRO_GAIN_VALUE_Y	) ;

	iRetVal = Calibration_VerifyUpdate();

	return iRetVal;
}

//********************************************************************************
// Function Name 	: WrGyroGainData_NV
// Retun Value		: 0:OK, 1:NG
// Argment Value	: UlReadValX: gyro gain X, UlReadValY: gyro gain Y
// Explanation		: Flash Write Hall Calibration Data Function
// History			: First edition 						
//********************************************************************************
INT16	WrGyroGainData_NV( UINT32 UlReadValX , UINT32 UlReadValY )
{
	UINT32	UlReadVal ;
	INT16 iRetVal;

	RamRead32A(  StCaliData_UsCalibrationStatus , &UlReadVal ) ;
	UlReadVal &= ~GYRO_GAIN_FLG;
	RamWrite32A( StCaliData_UsCalibrationStatus , 	UlReadVal ) ;
	
	RamWrite32A( StCaliData_SiGyroGain_X ,		UlReadValX ) ;
	
	RamWrite32A( StCaliData_SiGyroGain_Y ,		UlReadValY ) ;
	
	// Read calibration sector to buffer
	iRetVal = Calibration_VerifyUpdate_PreRead();
	if(iRetVal != 0) return(iRetVal);

	_PUT_UINT32( UlReadVal,											CALIBRATION_STATUS	) ;
	_PUT_UINT32( UlReadValX,											GYRO_GAIN_VALUE_X	) ;
	_PUT_UINT32( UlReadValY,											GYRO_GAIN_VALUE_Y	) ;

	iRetVal = Calibration_VerifyUpdate();

	return iRetVal;
}
#ifdef	HF_LINEAR_ENA
//********************************************************************************
// Function Name 	: SetHalLnData
// Retun Value		: 0:OK, 1:NG
// Argment Value	: non
// Explanation		: SRAM Write Hall Calibration Data Function
// History			: First edition 						
//********************************************************************************
void	SetHalLnData( UINT16 *UsPara )
{
	UINT8	i;
	UINT16	UsRdAdr;
	
	UsRdAdr = HAL_LN_COEFAX;
	
	for( i=0; i<17 ; i++ ){
		RamWrite32A( UsRdAdr + (i * 4) , (UINT32)UsPara[i*2+1] << 16 | (unsigned long)UsPara[i*2] );
	}
}

//********************************************************************************
// Function Name 	: WrHalLnData
// Retun Value		: 0:OK, 1:NG
// Argment Value	: UcMode	0:disable	1:enable
// Explanation		: Flash Write Hall Calibration Data Function
// History			: First edition 						
//********************************************************************************
int	WrHalLnData( unsigned char UcMode )
{
	UINT32		UlReadVal ;
	INT32		iRetVal;
	UINT16		UsRdAdr;
	UnDwdVal	StRdDat[17];
	UnDwdVal	StWrDat;
	UINT8		i;
	
	// Read calibration sector to buffer
	FlashNVR_ReadData_Byte(	CALIBRATION_DATA_ADDRESS, FLASH_SECTOR_BUFFER, 256	);
TRACE_DUMP( FLASH_SECTOR_BUFFER, 256 );

	RamRead32A(  StCaliData_UsCalibrationStatus , &UlReadVal ) ;
	if( UcMode ){
		UlReadVal &= ~HLLN_CALB_FLG;
	}else{
		UlReadVal |= HLLN_CALB_FLG;
	}
	RamWrite32A( StCaliData_UsCalibrationStatus , 	UlReadVal ) ;

	PUT_UINT32( UlReadVal ,	CALIBRATION_STATUS	) ;		// status store
	

	UsRdAdr = HAL_LN_COEFAX;

	for( i=0; i<17 ; i++ ){
		RamRead32A( UsRdAdr + (i * 4) , &(StRdDat[i].UlDwdVal));
//TRACE("StRdDat[ %04X ].UlDwdVal= %08X \n", i, StRdDat[i].UlDwdVal );
	}
	for( i=0; i<9 ; i++ ){
	//						Y								X
		StWrDat.UlDwdVal = (INT32)StRdDat[i+8].StDwdVal.UsHigVal << 16 | StRdDat[i].StDwdVal.UsLowVal ;
//TRACE(" %08X : ", StWrDat.UlDwdVal );
		PUT_UINT32( StWrDat.UlDwdVal , LN_ZONE1_COEFA + i * 8 ) ;					// coefficient store
	}
//TRACE("\n");
	for( i=0; i<9 ; i++ ){
	//						Y								X
		StWrDat.UlDwdVal = (INT32)StRdDat[i+9].StDwdVal.UsLowVal << 16 | StRdDat[i].StDwdVal.UsHigVal ;
//TRACE(" %08X : ", StWrDat.UlDwdVal );
		PUT_UINT32( StWrDat.UlDwdVal , LN_ZONE1_COEFA + 4 + i * 8 ) ;				// coefficient store
	}
//TRACE("\n");
	
	//
	// Flash update procedure
	//

	// Sector erase
	FlashNVRSectorErase_Byte( CALIBRATION_DATA_ADDRESS );

	// Write calibration sector from buffer
	FlashNVR_WriteData_Byte(	CALIBRATION_DATA_ADDRESS, FLASH_SECTOR_BUFFER, 256	);
TRACE_DUMP( FLASH_SECTOR_BUFFER, 256 );

	// Sector erify function
//	iRetVal = FlashNVRVerify_Byte( CALIBRATION_DATA_ADDRESS, FLASH_SECTOR_BUFFER, 256 );
	iRetVal = FlashCheck_NVRVerify();

	return iRetVal;
}
#endif	// HF_LINEAR_ENA



//********************************************************************************
// Function Name 	: RdHallCalData
// Retun Value		: Read calibration data
// Argment Value	: NON
// Explanation		: Read Hall Calibration Data in Data Ram
// History			: First edition 						
//********************************************************************************
void	RdHallCalData( void )
{
	UnDwdVal		StReadVal ;

	RamRead32A(  StCaliData_UsCalibrationStatus, &StAdjPar.StHalAdj.UlAdjPhs ) ;

	RamRead32A( StCaliData_SiHallMax_Before_X,	&StReadVal.UlDwdVal ) ;
	StAdjPar.StHalAdj.UsHlxMax = StReadVal.StDwdVal.UsHigVal ;
	RamRead32A( StCaliData_SiHallMin_Before_X, &StReadVal.UlDwdVal ) ;
	StAdjPar.StHalAdj.UsHlxMin = StReadVal.StDwdVal.UsHigVal ;
	RamRead32A( StCaliData_SiHallMax_After_X,	&StReadVal.UlDwdVal ) ;
	StAdjPar.StHalAdj.UsHlxMxa = StReadVal.StDwdVal.UsHigVal ;
	RamRead32A( StCaliData_SiHallMin_After_X,	&StReadVal.UlDwdVal ) ;
	StAdjPar.StHalAdj.UsHlxMna = StReadVal.StDwdVal.UsHigVal ;
	RamRead32A( StCaliData_SiHallMax_Before_Y, &StReadVal.UlDwdVal ) ;
	StAdjPar.StHalAdj.UsHlyMax = StReadVal.StDwdVal.UsHigVal ;
	RamRead32A( StCaliData_SiHallMin_Before_Y, &StReadVal.UlDwdVal ) ;
	StAdjPar.StHalAdj.UsHlyMin = StReadVal.StDwdVal.UsHigVal ;
	RamRead32A( StCaliData_SiHallMax_After_Y,	&StReadVal.UlDwdVal ) ;
	StAdjPar.StHalAdj.UsHlyMxa = StReadVal.StDwdVal.UsHigVal ;
	RamRead32A( StCaliData_SiHallMin_After_Y,	&StReadVal.UlDwdVal ) ;
	StAdjPar.StHalAdj.UsHlyMna = StReadVal.StDwdVal.UsHigVal ;
	RamRead32A( StCaliData_UiHallBias_X,	&StReadVal.UlDwdVal ) ;
	StAdjPar.StHalAdj.UsHlxGan = StReadVal.StDwdVal.UsHigVal ;
	RamRead32A( StCaliData_UiHallOffset_X,	&StReadVal.UlDwdVal ) ;
	StAdjPar.StHalAdj.UsHlxOff = StReadVal.StDwdVal.UsHigVal ;
	RamRead32A( StCaliData_UiHallBias_Y,	&StReadVal.UlDwdVal ) ;
	StAdjPar.StHalAdj.UsHlyGan = StReadVal.StDwdVal.UsHigVal ;
	RamRead32A( StCaliData_UiHallOffset_Y,	&StReadVal.UlDwdVal ) ;
	StAdjPar.StHalAdj.UsHlyOff = StReadVal.StDwdVal.UsHigVal ;

	RamRead32A( StCaliData_SiLoopGain_X,	&StAdjPar.StLopGan.UlLxgVal ) ;
	RamRead32A( StCaliData_SiLoopGain_Y,	&StAdjPar.StLopGan.UlLygVal ) ;

	RamRead32A( StCaliData_SiLensCen_Offset_X,	&StReadVal.UlDwdVal ) ;
	StAdjPar.StHalAdj.UsAdxOff = StReadVal.StDwdVal.UsHigVal ;
	RamRead32A( StCaliData_SiLensCen_Offset_Y,	&StReadVal.UlDwdVal ) ;
	StAdjPar.StHalAdj.UsAdyOff = StReadVal.StDwdVal.UsHigVal ;

	RamRead32A( StCaliData_SiGyroOffset_X,		&StReadVal.UlDwdVal ) ;
	StAdjPar.StGvcOff.UsGxoVal = StReadVal.StDwdVal.UsHigVal ;
	RamRead32A( StCaliData_SiGyroOffset_Y,		&StReadVal.UlDwdVal ) ;
	StAdjPar.StGvcOff.UsGyoVal = StReadVal.StDwdVal.UsHigVal ;

}


//********************************************************************************
// Function Name 	: TneADO
// Retun Value		: 0x0000:PASS, 0x0001:X MAX OVER, 0x0002:Y MAX OVER, 0x0003:X MIN OVER, 0x0004:Y MIN OVER, FFFF:Verify error
//					: 0x0100:X MAX RANGE ERROR, 0x0200:Y MAX RANGE ERROR, 0x0300:X MIN RANGE ERROR, 0x0400:Y MIN ERROR
// Argment Value	: 
// Explanation		: calculation margin Function
// History			: First edition 						
//********************************************************************************
UINT16	TneADO( )
{
	UINT16	UsSts = 0 ;
	INT32	iRetVal;
	INT32 limit ;
	INT32 gxgain ;
	INT32 gygain ;
	INT16 gout_x_marginp ;
	INT16 gout_x_marginm ;
	INT16 gout_y_marginp ;
	INT16 gout_y_marginm ;

	INT16 x_max ;
	INT16 x_min ;
	INT16 x_off ;
	INT16 y_max ;
	INT16 y_min ;
	INT16 y_off ;
	INT16 x_max_after ;
	INT16 x_min_after ;
	INT16 y_max_after ;
	INT16 y_min_after ;
	INT16 gout_x ;
	INT16 gout_y ;

	//
	// Flash update procedure
	//
	// Read calibration sector to buffer
	iRetVal = Calibration_VerifyUpdate_PreRead();
	if(iRetVal != 0) return(iRetVal);

	// Read calibration data
	RdHallCalData();

	x_max = (INT16)StAdjPar.StHalAdj.UsHlxMxa ;
	x_min = (INT16)StAdjPar.StHalAdj.UsHlxMna ;
	x_off = (INT16)StAdjPar.StHalAdj.UsAdxOff ;
	y_max = (INT16)StAdjPar.StHalAdj.UsHlyMxa ;
	y_min = (INT16)StAdjPar.StHalAdj.UsHlyMna ;
	y_off = (INT16)StAdjPar.StHalAdj.UsAdyOff ;

	RamRead32A( GF_LimitX_HLIMT,	&limit ) ;
	RamRead32A( StCaliData_SiGyroGain_X,	&gxgain ) ;
	RamRead32A( StCaliData_SiGyroGain_Y,	&gygain ) ;

	x_max_after = (x_max - x_off) ;
	if (x_off < 0)
	{
	    if ((0x7FFF - abs(x_max)) < abs(x_off)) x_max_after = 0x7FFF ;
	}

	x_min_after = (x_min - x_off) ;
	if (x_off > 0)
	{
	    if ((0x7FFF - abs(x_min)) < abs(x_off)) x_min_after = 0x8001 ;
	}

	y_max_after = (y_max - y_off) ;
	if (y_off < 0)
	{
	    if ((0x7FFF - abs(y_max)) < abs(y_off)) y_max_after = 0x7FFF ;
	}

	y_min_after = (y_min - y_off);
	if (y_off > 0)
	{
	    if ((0x7FFF - abs(y_min)) < abs(y_off)) y_min_after = 0x8001 ;
	}

	gout_x = (INT16)((INT32)(((float)gxgain / 0x7FFFFFFF) * limit * 4) >> 16);
	gout_y = (INT16)((INT32)(((float)gygain / 0x7FFFFFFF) * limit * 4) >> 16);

//TRACE( "ADOFF X\t=\t0x%04X\r\n", x_off ) ;
//TRACE( "ADOFF Y\t=\t0x%04X\r\n", y_off ) ;
//TRACE( "MAX GOUT X\t=\t0x%04X\r\n", gout_x ) ;
//TRACE( "MIN GOUT X\t=\t0x%04X\r\n", (gout_x * -1) ) ;
//TRACE( "MAX GOUT Y\t=\t0x%04X\r\n", gout_y) ;
//TRACE( "MIN GOUT Y\t=\t0x%04X\r\n", (gout_y * -1) ) ;

	gout_x_marginp = (INT16)(gout_x + LENS_MARGIN);			// MARGIN X+
	gout_x_marginm = (INT16)((gout_x + LENS_MARGIN) * -1);	// MARGIN X-
	gout_y_marginp = (INT16)(gout_y + LENS_MARGIN);			// MARGIN Y+
	gout_y_marginm = (INT16)((gout_y + LENS_MARGIN) * -1);	// MARGIN Y-

//TRACE( "MAX GOUT with margin X\t=\t0x%04X\r\n", gout_x_marginp ) ;
//TRACE( "MIN GOUT with margin X\t=\t0x%04X\r\n", gout_x_marginm ) ;
//TRACE( "MAX GOUT with margin Y\t=\t0x%04X\r\n", gout_y_marginp ) ;
//TRACE( "MIN GOUT with margin Y\t=\t0x%04X\r\n", gout_y_marginm ) ;

//TRACE( "MAX AFTER X\t=\t0x%04X\r\n", x_max_after ) ;
//TRACE( "MIN AFTER X\t=\t0x%04X\r\n", x_min_after ) ;
//TRACE( "MAX AFTER Y\t=\t0x%04X\r\n", y_max_after ) ;
//TRACE( "MIN AFTER Y\t=\t0x%04X\r\n", y_min_after ) ;

	// マージンがまったくないものは不良とする
	if (x_max_after < gout_x) {
		UsSts = 1 ;
	}
	else if (y_max_after < gout_y) {
		UsSts = 2 ;
	}
	else if (x_min_after > (gout_x * -1)) {
		UsSts = 3 ;
	}
	else if (y_min_after > (gout_y * -1)) {
		UsSts = 4 ;
	}
	else {
		// マージンオーバーであれば、ADOFFSETを更新する
		if (x_max_after < gout_x_marginp) {
			x_off -= (gout_x_marginp - x_max_after);
//TRACE( "UPDATE ADOFF X\t=\t0x%04X\r\n", x_off ) ;
		}
		if (x_min_after > gout_x_marginm) {
			x_off += abs(x_min_after - gout_x_marginm);
//TRACE( "UPDATE ADOFF X\t=\t0x%04X\r\n", x_off ) ;
		}
		if (y_max_after < gout_y_marginp) {
			y_off -= (gout_y_marginp - y_max_after);
//TRACE( "UPDATE ADOFF Y\t=\t0x%04X\r\n", y_off ) ;
		}
		if (y_min_after > gout_y_marginm) {
			y_off += abs(y_min_after - gout_y_marginm);
//TRACE( "UPDATE ADOFF X\t=\t0x%04X\r\n", y_off ) ;
		}
		
		if ( (StAdjPar.StHalAdj.UsAdxOff != (UINT16)x_off) || (StAdjPar.StHalAdj.UsAdyOff != (UINT16)y_off) ) {
			StAdjPar.StHalAdj.UsAdxOff = x_off ;
			StAdjPar.StHalAdj.UsAdyOff = y_off ;

			RamWrite32A( StCaliData_SiLensCen_Offset_X ,	(UINT32)(StAdjPar.StHalAdj.UsAdxOff << 16) ) ;
			RamWrite32A( StCaliData_SiLensCen_Offset_Y ,	(UINT32)(StAdjPar.StHalAdj.UsAdyOff << 16) ) ;

			_PUT_UINT32( (UINT32)(StAdjPar.StHalAdj.UsAdxOff << 16),	LENS_CENTER_VALUE_X	) ;
			_PUT_UINT32( (UINT32)(StAdjPar.StHalAdj.UsAdyOff << 16),	LENS_CENTER_VALUE_Y	) ;
			iRetVal = Calibration_VerifyUpdate();
		}
	}

	// *******************************
	// effective range check
	// *******************************
	if (UsSts == 0) {
		UINT16 UsReadVal ;
		float flDistanceX, flDistanceY ;
		float flDistanceAD = SLT_OFFSET * 6 ;

		// effective range check
		_GET_UINT16( UsReadVal,	DISTANCE_X	) ;
		flDistanceX = (float)UsReadVal / 10.0f ;
//TRACE("DISTANCE (X, Y) pixel = (%04X", UsReadVal );

		_GET_UINT16( UsReadVal,	DISTANCE_Y	) ;
		flDistanceY = (float)UsReadVal / 10.0f ;
//TRACE(", %04X)\r\n", UsReadVal );

//TRACE("DISTANCE (X, Y) pixel = (%x, %x)\r\n", (int)(flDistanceX * 10.0), (int)(flDistanceY * 10.0) );
//TRACE("X MAX um = %d\r\n", (int)((x_max_after * (flDistanceX / flDistanceAD)) * PIXEL_SIZE) ) ;
//TRACE("Y MAX um = %d\r\n", (int)((y_max_after * (flDistanceY / flDistanceAD)) * PIXEL_SIZE) ) ;
//TRACE("X MIN um = %d\r\n", (int)((abs(x_min_after) * (flDistanceX / flDistanceAD)) * PIXEL_SIZE) ) ;
//TRACE("Y MIN um = %d\r\n", (int)((abs(y_min_after) * (flDistanceY / flDistanceAD)) * PIXEL_SIZE) ) ;
//TRACE("SPEC PIXEL = %d\r\n", (int)SPEC_PIXEL ) ;

		if ( (x_max_after * (flDistanceX / flDistanceAD)) < SPEC_PIXEL ) {
//TRACE("X MAX < 85um\r\n");
			// error
			UsSts |= 0x0100 ;
		}
		else if ( (y_max_after * (flDistanceY / flDistanceAD)) < SPEC_PIXEL ) {
//TRACE("Y MAX < 85um\r\n");
			// error
			UsSts |= 0x0200 ;
		}
		else if ( (abs(x_min_after) * (flDistanceX / flDistanceAD)) < SPEC_PIXEL ) {
//TRACE("X MIN < 85um\r\n");
			// error
			UsSts |= 0x0300 ;
		}
		else if ( (abs(y_min_after) * (flDistanceY / flDistanceAD)) < SPEC_PIXEL ) {
//TRACE("Y MAX < 85um\r\n");
			// error
			UsSts |= 0x0400 ;
		}
	}
	return( UsSts ) ;

}


//********************************************************************************
// Function Name 	: OscStb
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: Osc Standby Function
// History			: First edition 						
//********************************************************************************
void	OscStb( void )
{
	RamWrite32A( CMD_IO_ADR_ACCESS , STBOSCPLL ) ;
	RamWrite32A( CMD_IO_DAT_ACCESS , OSC_STB ) ;
}


//********************************************************************************
// Function Name 	: RunHea
// Retun Value		: Result
// Argment Value	: NON
// Explanation		: Hall Examination of Acceptance
// History			: First edition 						
//********************************************************************************
 #define		ACT_CHK_LVL		0x33320000		// 0.4
 #define		ACT_CHK_FRQ		0x00068C16		// 4Hz
 #define		ACT_CHK_NUM		5005			// 20.0195/0.004 < x
 #define		ACT_THR			0x0A3D0000		// 20dB 0.4*10^(-20dB/20)*FFFF

UINT8	RunHea( void )
{
	UINT8 	UcRst ;

	UcRst = EXE_END ;
	UcRst |= TstActMov( X_DIR) ;
	UcRst |= TstActMov( Y_DIR) ;
	
//TRACE("UcRst = %02x\n", UcRst ) ;
	
	return( UcRst ) ;
}
UINT8	TstActMov( UINT8 UcDirSel )
{
	UINT8	UcRsltSts = 0;
	INT32	SlMeasureParameterNum ;
	INT32	SlMeasureParameterA , SlMeasureParameterB ;

	UINT32	UlMsppVal ;

	SlMeasureParameterNum	=	ACT_CHK_NUM ;

	if( UcDirSel == X_DIR ) {								// X axis
		SlMeasureParameterA		=	HallFilterD_HXDAZ1 ;		// Set Measure RAM Address
		SlMeasureParameterB		=	HallFilterD_HYDAZ1 ;		// Set Measure RAM Address
	} else if( UcDirSel == Y_DIR ) {						// Y axis
		SlMeasureParameterA		=	HallFilterD_HYDAZ1 ;		// Set Measure RAM Address
		SlMeasureParameterB		=	HallFilterD_HXDAZ1 ;		// Set Measure RAM Address
	}
	SetSinWavGenInt();
	
	RamWrite32A( SinWave_Offset		,	ACT_CHK_FRQ ) ;				// Freq Setting = Freq * 80000000h / Fs	: 5Hz
	RamWrite32A( SinWave_Gain		,	ACT_CHK_LVL ) ;				// Set Sine Wave Gain
	RamWrite32A( SinWaveC_Regsiter	,	0x00000001 ) ;				// Sine Wave Start
	if( UcDirSel == X_DIR ) {
		SetTransDataAdr( SinWave_OutAddr	,	(UINT32)HALL_RAM_HXOFF1 ) ;	// Set Sine Wave Input RAM
	}else if( UcDirSel == Y_DIR ){
		SetTransDataAdr( SinWave_OutAddr	,	(UINT32)HALL_RAM_HYOFF1 ) ;	// Set Sine Wave Input RAM
	}
	MesFil( NOISE ) ;					// 測定用フィルターを設定する。

	MeasureStart( SlMeasureParameterNum , SlMeasureParameterA , SlMeasureParameterB ) ;					// Start measure
	
	MeasureWait() ;						// Wait complete of measurement
	
	RamWrite32A( SinWaveC_Regsiter	,	0x00000000 ) ;								// Sine Wave Stop
	
	if( UcDirSel == X_DIR ) {
		SetTransDataAdr( SinWave_OutAddr	,	(UINT32)0x00000000 ) ;	// Set Sine Wave Input RAM
		RamWrite32A( HALL_RAM_HXOFF1		,	0x00000000 ) ;				// DelayRam Clear
	}else if( UcDirSel == Y_DIR ){
		SetTransDataAdr( SinWave_OutAddr	,	(UINT32)0x00000000 ) ;	// Set Sine Wave Input RAM
		RamWrite32A( HALL_RAM_HYOFF1		,	0x00000000 ) ;				// DelayRam Clear
	}
	RamRead32A( StMeasFunc_MFA_UiAmp1 , ( UINT32 * )&UlMsppVal ) ;	// amp1 value


//TRACE(" DIR = %04x, PP = %08x, ", UcDirSel, (unsigned int)UlMsppVal ) ;

	
	UcRsltSts = EXE_END ;
	if( UlMsppVal > ACT_THR ){
		if ( !UcDirSel ) {					// AXIS X
			UcRsltSts = EXE_HXMVER ;
		}else{								// AXIS Y
			UcRsltSts = EXE_HYMVER ;
		}
	}

	return( UcRsltSts ) ;

}

//********************************************************************************
// Function Name 	: RunGea
// Retun Value		: Result
// Argment Value	: NON
// Explanation		: Gyro Examination of Acceptance
// History			: First edition 						
//********************************************************************************
#define 	GEA_NUM		512			// 512times
// #define		GEA_DIF_HIG		0x0062			// 2021_32.8lsb/°/s    max 3.0°/s-p-p
 #define		GEA_DIF_HIG		0x0057			// 2030_87.5lsb/°/s    max 1.0°/s-p-p
 #define		GEA_DIF_LOW		0x0001				// Gyro Examination of Acceptance
 
UINT8	RunGea( void )
{
	UnllnVal	StMeasValueA , StMeasValueB ;
	INT32		SlMeasureParameterA , SlMeasureParameterB ;
	UINT8 		UcRst, UcCnt, UcXLowCnt, UcYLowCnt, UcXHigCnt, UcYHigCnt ;
	UINT16		UsGxoVal[10], UsGyoVal[10], UsDif;
	INT32		SlMeasureParameterNum , SlMeasureAveValueA , SlMeasureAveValueB ;

	
	UcRst = EXE_END ;
	UcXLowCnt = UcYLowCnt = UcXHigCnt = UcYHigCnt = 0 ;
	
	MesFil( THROUGH ) ;				// 測定用フィルターを設定する。
	
	for( UcCnt = 0 ; UcCnt < 10 ; UcCnt++ )
	{
		//平均値測定
	
		MesFil( THROUGH ) ;					// Set Measure Filter

		SlMeasureParameterNum	=	GEA_NUM ;					// Measurement times
		SlMeasureParameterA		=	GYRO_RAM_GX_ADIDAT ;		// Set Measure RAM Address
		SlMeasureParameterB		=	GYRO_RAM_GY_ADIDAT ;		// Set Measure RAM Address
		
		MeasureStart( SlMeasureParameterNum , SlMeasureParameterA , SlMeasureParameterB ) ;					// Start measure
	
		MeasureWait() ;					// Wait complete of measurement
	
//TRACE("Read Adr = %04x, %04xh \n",StMeasFunc_MFA_LLiIntegral1 + 4 , StMeasFunc_MFA_LLiIntegral1) ;
		RamRead32A( StMeasFunc_MFA_LLiIntegral1 		, &StMeasValueA.StUllnVal.UlLowVal ) ;	// X axis
		RamRead32A( StMeasFunc_MFA_LLiIntegral1 + 4		, &StMeasValueA.StUllnVal.UlHigVal ) ;
		RamRead32A( StMeasFunc_MFB_LLiIntegral2 		, &StMeasValueB.StUllnVal.UlLowVal ) ;	// Y axis
		RamRead32A( StMeasFunc_MFB_LLiIntegral2 + 4		, &StMeasValueB.StUllnVal.UlHigVal ) ;
	
//TRACE("GX_OFT = %08x, %08xh \n",(unsigned int)StMeasValueA.StUllnVal.UlHigVal,(unsigned int)StMeasValueA.StUllnVal.UlLowVal) ;
//TRACE("GY_OFT = %08x, %08xh \n",(unsigned int)StMeasValueB.StUllnVal.UlHigVal,(unsigned int)StMeasValueB.StUllnVal.UlLowVal) ;
		SlMeasureAveValueA = (INT32)( (INT64)StMeasValueA.UllnValue / SlMeasureParameterNum ) ;
		SlMeasureAveValueB = (INT32)( (INT64)StMeasValueB.UllnValue / SlMeasureParameterNum ) ;
//TRACE("GX_AVEOFT = %08xh \n",(unsigned int)SlMeasureAveValueA) ;
//TRACE("GY_AVEOFT = %08xh \n",(unsigned int)SlMeasureAveValueB) ;
		// X
		UsGxoVal[UcCnt] = (UINT16)( SlMeasureAveValueA >> 16 );	// 平均値測定
		
		// Y
		UsGyoVal[UcCnt] = (UINT16)( SlMeasureAveValueB >> 16 );	// 平均値測定
		
//TRACE("UcCnt = %02x, UsGxoVal[UcCnt] = %04x\n", UcCnt, UsGxoVal[UcCnt] ) ;
//TRACE("UcCnt = %02x, UsGyoVal[UcCnt] = %04x\n", UcCnt, UsGyoVal[UcCnt] ) ;
		
		
		if( UcCnt > 0 )
		{
			if ( (INT16)UsGxoVal[0] > (INT16)UsGxoVal[UcCnt] ) {
				UsDif = (UINT16)((INT16)UsGxoVal[0] - (INT16)UsGxoVal[UcCnt]) ;
			} else {
				UsDif = (UINT16)((INT16)UsGxoVal[UcCnt] - (INT16)UsGxoVal[0]) ;
			}
			
			if( UsDif > GEA_DIF_HIG ) {
				//UcRst = UcRst | EXE_GXABOVE ;
				UcXHigCnt ++ ;
			}
			if( UsDif < GEA_DIF_LOW ) {
				//UcRst = UcRst | EXE_GXBELOW ;
				UcXLowCnt ++ ;
			}
//TRACE("CNT = %02x  ,  X diff = %04x ", UcCnt , UsDif ) ;
			
			if ( (INT16)UsGyoVal[0] > (INT16)UsGyoVal[UcCnt] ) {
				UsDif = (UINT16)((INT16)UsGyoVal[0] - (INT16)UsGyoVal[UcCnt]) ;
			} else {
				UsDif = (UINT16)((INT16)UsGyoVal[UcCnt] - (INT16)UsGyoVal[0]) ;
			}
			
			if( UsDif > GEA_DIF_HIG ) {
				//UcRst = UcRst | EXE_GYABOVE ;
				UcYHigCnt ++ ;
			}
			if( UsDif < GEA_DIF_LOW ) {
				//UcRst = UcRst | EXE_GYBELOW ;
				UcYLowCnt ++ ;
			}
//TRACE("  Y diff = %04x \n", UsDif ) ;
		}
	}
	
	if( UcXHigCnt >= 1 ) {
		UcRst = UcRst | EXE_GXABOVE ;
	}
	if( UcXLowCnt > 8 ) {
		UcRst = UcRst | EXE_GXBELOW ;
	}
	
	if( UcYHigCnt >= 1 ) {
		UcRst = UcRst | EXE_GYABOVE ;
	}
	if( UcYLowCnt > 8 ) {
		UcRst = UcRst | EXE_GYBELOW ;
	}
	
//TRACE("UcRst = %02x\n", UcRst ) ;
	
	return( UcRst ) ;
}

//********************************************************************************
// Function Name 	: FrqDet
// Retun Value		: 0:PASS, 1:OIS X NG, 2:OIS Y NG, 4:CLAF NG
// Argment Value	: NON
// Explanation		: Module Check 
// History			: First edition 						
//********************************************************************************
UINT8 FrqDet( void )
{
	INT32 SlMeasureParameterA , SlMeasureParameterB ;
	INT32 SlMeasureParameterNum ;
	UINT32 UlXasP_P , UlYasP_P ;
#ifdef	__OIS_CLOSED_AF__
	UINT32 UlAasP_P ;
#endif	// __OIS_CLOSED_AF__

	UINT8 UcRtnVal;

	UcRtnVal = 0;

	//Measurement Setup
	MesFil( OSCCHK ) ;													// Set Measure Filter

	SlMeasureParameterNum	=	1000 ;									// 1000times( 50ms )
	SlMeasureParameterA		=	(UINT32)HALL_RAM_HXOUT0 ;		// Set Measure RAM Address
	SlMeasureParameterB		=	(UINT32)HALL_RAM_HYOUT0 ;		// Set Measure RAM Address

	// impulse Set
//	RamWrite32A( HALL_RAM_HXOFF1 , STEP1 ) ;							// X manual 
//	RamWrite32A( HALL_RAM_HYOFF1 , STEP1 ) ;							// Y manual

//	RamWrite32A( HALL_RAM_HXOFF1 , STEP2 ) ;							// X manual 
//	RamWrite32A( HALL_RAM_HYOFF1 , STEP2 ) ;							// Y manual
	WitTim( 300 ) ;

	// Start measure
	MeasureStart( SlMeasureParameterNum , SlMeasureParameterA , SlMeasureParameterB ) ;	
	SetWaitTime(1) ;
	MeasureWait() ;														// Wait complete of measurement
	RamRead32A( StMeasFunc_MFA_UiAmp1, &UlXasP_P ) ;					// X Axis Peak to Peak
	RamRead32A( StMeasFunc_MFB_UiAmp2, &UlYasP_P ) ;					// Y Axis Peak to Peak
//TRACE("UlXasP_P = %X\r\n", (unsigned int)UlXasP_P ) ;
//TRACE("UlYasP_P = %X\r\n", (unsigned int)UlYasP_P ) ;

	WitTim( 50 ) ;

	// Osc Value Check X
	if(  UlXasP_P > ULTHDVAL ){
		UcRtnVal = 1;
	}
	// Osc Value Check Y
	if(  UlYasP_P > ULTHDVAL ){
		UcRtnVal |= 2;
	}

#ifdef	__OIS_CLOSED_AF__
	// CLAF
	SlMeasureParameterA		=	(UINT32)CLAF_RAMA_AFDEV ;		// Set Measure RAM Address
	SlMeasureParameterB		=	(UINT32)CLAF_RAMA_AFDEV ;		// Set Measure RAM Address

	// impulse Set
//	RamWrite32A( CLAF_RAMA_AFTARGET , STEP1 ) ;							// CLAF manual 

//	RamWrite32A( CLAF_RAMA_AFTARGET , STEP2 ) ;							// CLAF manual 
	WitTim( 300 ) ;

	// Start measure
	MeasureStart( SlMeasureParameterNum , SlMeasureParameterA , SlMeasureParameterB ) ;	
	SetWaitTime(1) ;
	MeasureWait() ;														// Wait complete of measurement
	RamRead32A( StMeasFunc_MFA_UiAmp1, &UlAasP_P ) ;					// CLAF Axis Peak to Peak
//TRACE("UlAasP_P = %X\r\n", (int)UlAasP_P ) ;

	WitTim( 50 ) ;

	// Osc Value Check CLAF
	if(  UlAasP_P > ULTHDVAL ){
		UcRtnVal |= 4;
	}
	
#endif	// __OIS_CLOSED_AF__

	return(UcRtnVal);													// Retun Status value
}


//********************************************************************************
// Function Name 	: SetTregAf
// Retun Value		: 
// Argment Value	: Min:000h Max:7FFh (11bit) in the case of Bi-direction
// Argment Value	: Min:000h Max:3FFh (10bit) in the case of Uni-direction
// Explanation		: 
// History			: First edition 						2014.06.19 T.Tokoro
//********************************************************************************
void	SetTregAf( unsigned short UsTregAf )
{
	UINT8	UcStRd = 1;

	RamWrite32A( CMD_AF_POSITION,	UsTregAf| 0x00010000 ) ;		// bit 16 : FST mode
	while( UcStRd ) {
		UcStRd = RdStatus(1);
	}
}
//********************************************************************************
// Function Name 	: GetGyroWhoAmI
// Retun Value		: Result
// Argment Value	: NON
// Explanation		: Get Gyro Who Am I
// History			: First edition 						
//********************************************************************************
void GetGyroWhoAmI( UINT8 * UcWho )
{
	UINT32	UlVal ;
	
	RamWrite32A( 0xF01D , 0x75000000 ) ;
	WitTim( 5 ) ;
	
	RamRead32A( 0xF01D , &UlVal ) ;
//TRACE("%08x \n", UlVal );
	
	*UcWho = (UINT8)( UlVal >> 24 ) ;
}

//********************************************************************************
// Function Name 	: GetGyroID
// Retun Value		: Result
// Argment Value	: NON
// Explanation		: Get Gyro ID
// History			: First edition 						
//********************************************************************************
void GetGyroID( UINT8 * UcGid )
{
	UINT32	UlCnt;
	UINT32	UlVal;
	
	for( UlCnt = 0; UlCnt < 8; UlCnt++ ){
		RamWrite32A( 0xF01E, 0x6D000000 ) ;
		WitTim( 5 ) ;
		
		RamWrite32A( 0xF01E, ( 0x6E000000 | ( UlCnt << 16 ) ) ) ;
		WitTim( 5 ) ;
		
		RamWrite32A( 0xF01D, 0x6F000000 ) ;
		WitTim( 5 ) ;
		
		RamRead32A( 0xF01D, &UlVal ) ;
//TRACE("%08x \n", UlVal );
		
		WitTim( 5 ) ;
		UcGid[UlCnt] = (UINT8)( UlVal >> 24 ) ;
	}
}

//********************************************************************************
// Function Name 	: GyroSleep
// Retun Value		: Result
// Argment Value	: NON
// Explanation		: Gyro Sleep Control
// History			: First edition 						
//********************************************************************************
void GyroSleep( UINT8 UcCtrl )
{
	UINT8	UcReg;
	UINT32	UlVal;
	
	RamWrite32A( 0xF01D, 0x6B000000 ) ;
	WitTim( 5 ) ;
	
	RamRead32A( 0xF01D, &UlVal ) ;
	WitTim( 5 ) ;
	
	UcReg = (UINT8)( UlVal >> 24 ) ;
	
	if( UcCtrl == ON ){
		UcReg = UcReg | 0x40;	// Bit6(SLEEP) ON
	}
	else if( UcCtrl == OFF ){
		UcReg = UcReg & 0xBF;	// Bit6(SLEEP) OFF
	}
	
	RamWrite32A( 0xF01E, ( 0x6B000000 | ( UcReg << 16 ) ) ) ;
}
//********************************************************************************
// Function Name 	: RunGea2
// Retun Value		: Result
// Argment Value	: NON
// Explanation		: Gyro Examination of Acceptance
// History			: First edition 						
//********************************************************************************
#define 	GEA_NUM2		2048			// 2048times
// level of judgement
#define		GEA_MAX_LVL		0x0A41			// 2030_87.5lsb/°/s    max 30°/s-p-p
#define		GEA_MIN_LVL		0x1482			// 2030_87.5lsb/°/s    min 60°/s-p-p
UINT8	RunGea2( UINT8 UcMode )
{
	INT32	SlMeasureParameterA , SlMeasureParameterB ;
	UINT8 	UcRst ;
	UINT16	UsGyrXval , UsGyrYval;
	INT32	SlMeasureParameterNum ;
	UINT8	UcStRd;
	UINT32	UlSwRd , UlGyroConfig ; //, UlGyCnt;
	
	stMesRam 		StMesRamA ;
	stMesRam 		StMesRamB ;
	
	UcRst = EXE_END ;
	
	OisDis() ;
	
	RamRead32A( GYRO_RAM_GYRO_Switch , &UlSwRd);
	RamWrite32A( GYRO_RAM_GYRO_Switch , UlSwRd & 0xFFFFFFFC ) ;
	
	RamWrite32A( CMD_GYRO_RD_ACCS , 0x1B000000 );
	UcStRd = 1;
	while( UcStRd ) {
		UcStRd = RdStatus(1);
	}
	RamRead32A( CMD_GYRO_RD_ACCS , &UlGyroConfig );		/* FS_SEL backup */
//TRACE("GYCONFIG = %08x \n",(UINT32)UlGyroConfig ) ;

////////// ////////// ////////// ////////// //////////

	RamWrite32A( CMD_GYRO_WR_ACCS , 0x1B180000 );		/* FS_SEL=3固定 & Disable Self Test */
	UcStRd = 1;
	while( UcStRd ) {
		UcStRd = RdStatus(1);
	}
	
	SlMeasureParameterNum	=	GEA_NUM2 ;					// Measurement times
	SlMeasureParameterA		=	GYRO_RAM_GX_ADIDAT ;		// Set Measure RAM Address
	SlMeasureParameterB		=	GYRO_RAM_GY_ADIDAT ;		// Set Measure RAM Address
	
	MesRam( SlMeasureParameterA, SlMeasureParameterB, SlMeasureParameterNum, &StMesRamA, &StMesRamB );
	
	if( UcMode == GEA_MINMAX_MODE ){	// min, max mode
		
TRACE("GX [max] = %08x, [min] = %08xh \n", (unsigned int)StMesRamA.SlMeasureMaxValue, (unsigned int)StMesRamA.SlMeasureMinValue) ;
TRACE("GY [max] = %08x, [min] = %08xh \n", (unsigned int)StMesRamB.SlMeasureMaxValue, (unsigned int)StMesRamB.SlMeasureMinValue) ;

TRACE("ABS_GX [max] = %08x, [min] = %08xh \n", (unsigned int)abs(StMesRamA.SlMeasureMaxValue), (unsigned int)abs(StMesRamA.SlMeasureMinValue)) ;
TRACE("ABS_GY [max] = %08x, [min] = %08xh \n", (unsigned int)abs(StMesRamB.SlMeasureMaxValue), (unsigned int)abs(StMesRamB.SlMeasureMinValue)) ;
		
		// X
		if( abs(StMesRamA.SlMeasureMaxValue) >= abs(StMesRamA.SlMeasureMinValue) ) {
			UsGyrXval = (UINT16)( abs(StMesRamA.SlMeasureMaxValue) >> 16 );		// max value
		}
		else{
			UsGyrXval = (UINT16)( abs(StMesRamA.SlMeasureMinValue) >> 16 );		// max value
		}
		
		// Y
		if( abs(StMesRamB.SlMeasureMaxValue) >= abs(StMesRamB.SlMeasureMinValue) ) {
			UsGyrYval = (UINT16)( abs(StMesRamB.SlMeasureMaxValue) >> 16 );		// max value
		}
		else{
			UsGyrYval = (UINT16)( abs(StMesRamB.SlMeasureMinValue) >> 16 );		// max value
		}
		
	}
	else{								// mean mode
		
//TRACE("GX [ave] = %08xh \n", (UINT32)StMesRamA.SlMeasureAveValue) ;
//TRACE("GY [ave] = %08xh \n", (UINT32)StMesRamB.SlMeasureAveValue) ;
		
//TRACE("ABS_GX [ave] = %08xh \n", (UINT32)abs(StMesRamA.SlMeasureAveValue)) ;
//TRACE("ABS_GY [ave] = %08xh \n", (UINT32)abs(StMesRamB.SlMeasureAveValue)) ;
		
		// X
		UsGyrXval = (UINT16)( abs(StMesRamA.SlMeasureAveValue) >> 16 );		// ave value
		
		// Y
		UsGyrYval = (UINT16)( abs(StMesRamB.SlMeasureAveValue) >> 16 );		// ave value
		
	}
		
//TRACE("UsGyrXval = %04x\n", UsGyrXval ) ;
//TRACE("UsGyrYval = %04x\n", UsGyrYval ) ;
		
	if( UsGyrXval > GEA_MAX_LVL ) {
		UcRst = UcRst | EXE_GXABOVE ;
	}
	if( UsGyrYval > GEA_MAX_LVL ) {
		UcRst = UcRst | EXE_GYABOVE ;
	}
	
	if( StMesRamA.SlMeasureMinValue == StMesRamA.SlMeasureMaxValue ){
		UcRst = UcRst | EXE_GXABOVE ;
	}
	if( StMesRamB.SlMeasureMinValue == StMesRamB.SlMeasureMaxValue ){
		UcRst = UcRst | EXE_GYABOVE ;
	}

////////// ////////// ////////// ////////// //////////

	RamWrite32A( CMD_GYRO_WR_ACCS , 0x1BD80000 );		/* FS_SEL=3固定 & Enable Self Test */
	UcStRd = 1;
	while( UcStRd ) {
		UcStRd = RdStatus(1);
	}
	
	WitTim( 50 ) ;					/* 50ms*/
	
	SlMeasureParameterNum	=	GEA_NUM2 ;					// Measurement times
	SlMeasureParameterA		=	GYRO_RAM_GX_ADIDAT ;		// Set Measure RAM Address
	SlMeasureParameterB		=	GYRO_RAM_GY_ADIDAT ;		// Set Measure RAM Address

	MesRam( SlMeasureParameterA, SlMeasureParameterB, SlMeasureParameterNum, &StMesRamA, &StMesRamB );

	if( UcMode == GEA_MINMAX_MODE ){	// min, max mode

//TRACE("GX [max] = %08x, [min] = %08xh \n", (UINT32)StMesRamA.SlMeasureMaxValue, (UINT32)StMesRamA.SlMeasureMinValue) ;
//TRACE("GY [max] = %08x, [min] = %08xh \n", (UINT32)StMesRamB.SlMeasureMaxValue, (UINT32)StMesRamB.SlMeasureMinValue) ;

		// X
		UsGyrXval = (UINT16)( StMesRamA.SlMeasureMinValue >> 16 );		// min value
		
		// Y
		UsGyrYval = (UINT16)( StMesRamB.SlMeasureMinValue >> 16 );		// min value
	
	}
	else{								// mean mode
//TRACE("GX [ave] = %08xh \n", (UINT32)StMesRamA.SlMeasureAveValue) ;
//TRACE("GY [ave] = %08xh \n", (UINT32)StMesRamB.SlMeasureAveValue) ;
		
		// X
		UsGyrXval = (UINT16)( StMesRamA.SlMeasureAveValue >> 16 );		// ave value
		
		// Y
		UsGyrYval = (UINT16)( StMesRamB.SlMeasureAveValue >> 16 );		// ave value
		
	}
	
//TRACE("UsGyrXval = %04x\n", UsGyrXval ) ;
//TRACE("UsGyrYval = %04x\n", UsGyrYval ) ;
	
	if( UsGyrXval < GEA_MIN_LVL ) {
		UcRst = UcRst | EXE_GXBELOW ;
	}
	if( UsGyrYval < GEA_MIN_LVL ) {
		UcRst = UcRst | EXE_GYBELOW ;
	}
	
	if( StMesRamA.SlMeasureMinValue == StMesRamA.SlMeasureMaxValue ){
		UcRst = UcRst | EXE_GXBELOW ;
	}
	if( StMesRamB.SlMeasureMinValue == StMesRamB.SlMeasureMaxValue ){
		UcRst = UcRst | EXE_GYBELOW ;
	}
	
	RamWrite32A( CMD_GYRO_WR_ACCS , 0x1B000000 | ( UlGyroConfig >> 8));		/* 元の設定値に戻す */
	UcStRd = 1;
	while( UcStRd ) {
		UcStRd = RdStatus(1);
	}
	
//TRACE("GYCONFIG = %08x \n",(UINT32)(0x1B000000 | ( UlGyroConfig >> 8)) ) ;
	
	RamWrite32A( GYRO_RAM_GYRO_Switch , UlSwRd | 0x00000001 ) ;
//TRACE("UcRst = %02x\n", UcRst ) ;
	
	return( UcRst ) ;
}

//********************************************************************************
// Function Name 	: MesRam
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Measure 
// History			: First edition 						2015.07.06 
//********************************************************************************
UINT8	 MesRam( INT32 SlMeasureParameterA, INT32 SlMeasureParameterB, INT32 SlMeasureParameterNum, stMesRam* pStMesRamA, stMesRam* pStMesRamB )
{
	UnllnVal	StMeasValueA , StMeasValueB ;
	
	MesFil( THROUGH ) ;							// Set Measure Filter
	
	MeasureStart( SlMeasureParameterNum, 		// 
				  SlMeasureParameterA,			// 
				  SlMeasureParameterB	) ;		// Start measure
	
	MeasureWait() ;								// Wait complete of measurement
	
	// A : X axis
	RamRead32A( StMeasFunc_MFA_SiMax1 , &(pStMesRamA->SlMeasureMaxValue) ) ;			// Max value
	RamRead32A( StMeasFunc_MFA_SiMin1 , &(pStMesRamA->SlMeasureMinValue) ) ;			// Min value
	RamRead32A( StMeasFunc_MFA_UiAmp1 , &(pStMesRamA->SlMeasureAmpValue) ) ;			// Amp value
	RamRead32A( StMeasFunc_MFA_LLiIntegral1,	 &(StMeasValueA.StUllnVal.UlLowVal) ) ;	// Integration Low
	RamRead32A( StMeasFunc_MFA_LLiIntegral1 + 4, &(StMeasValueA.StUllnVal.UlHigVal) ) ;	// Integration Hig
	pStMesRamA->SlMeasureAveValue = 
				(INT32)( (INT64)StMeasValueA.UllnValue / SlMeasureParameterNum ) ;	// Ave value
	
	// B : Y axis
	RamRead32A( StMeasFunc_MFB_SiMax2 , &(pStMesRamB->SlMeasureMaxValue) ) ;			// Max value
	RamRead32A( StMeasFunc_MFB_SiMin2 , &(pStMesRamB->SlMeasureMinValue) ) ;			// Min value
	RamRead32A( StMeasFunc_MFB_UiAmp2 , &(pStMesRamB->SlMeasureAmpValue) ) ;			// Amp value
	RamRead32A( StMeasFunc_MFB_LLiIntegral2,	 &(StMeasValueB.StUllnVal.UlLowVal) ) ;	// Integration Low
	RamRead32A( StMeasFunc_MFB_LLiIntegral2 + 4, &(StMeasValueB.StUllnVal.UlHigVal) ) ;	// Integration Hig
	pStMesRamB->SlMeasureAveValue = 
				(INT32)( (INT64)StMeasValueB.UllnValue / SlMeasureParameterNum ) ;	// Ave value
	
	return( 0 );
}
