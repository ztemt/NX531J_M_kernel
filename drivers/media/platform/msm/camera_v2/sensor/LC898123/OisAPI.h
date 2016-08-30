//********************************************************************************
//		<< LC898122 Evaluation Soft>>
//		Program Name	: OisAPI_S21N02A.h
// 		Explanation		: API List for customers
//		History			: First edition						2015.08.26 K.abe
//********************************************************************************
//****************************************************
//	extern selector for API
//****************************************************
#ifdef	__OISCMD__
	#define	__OIS_CMD_HEADER__
#else
	#define	__OIS_CMD_HEADER__		extern
#endif

//****************************************************
//	MODE SELECTORS (Compile Switches)
//****************************************************
#define			__OIS_MODULE_CALIBRATION__		// for module maker to done the calibration. 
//#define 	   	__OIS_TYPE_XC__					// for LC898123XC.
//#define 		__EXTRA_E0_COMMAND__				// for Flash calibration data upload.
//#define		__OIS_BIG_ENDIAN__				// for read calibration data in Flash.

//#define		__OIS_CLOSED_AF__

//****************************************************
//	API FUNCTION LIST	
//****************************************************
/* AF position Control [mandatory] */
__OIS_CMD_HEADER__ void	SetTregAf( UINT16 UsTregAf );				// Bi-direction :  Min:000h Max:7FFh (11bit) 
																	// Uni-direction : Min:000h Max:3FFh (10bit) 

/* Status Read and OIS enable [mandatory] */
__OIS_CMD_HEADER__	UINT8	RdStatus( UINT8 UcStBitChk );			// Status Read whether initialization finish or not.
__OIS_CMD_HEADER__	void	OisEna( void );							// OIS Enable Function

/* Others [option] */
__OIS_CMD_HEADER__	UINT8	RtnCen( UINT8	UcCmdPar ) ;			// Return to Center Function. Hall servo on/off
__OIS_CMD_HEADER__	void	SetPanTiltMode( UINT8 UcPnTmod );		// default ON.
__OIS_CMD_HEADER__	void	RdHallCalData( void );					// Read Hall Calibration Data in Data Ram
__OIS_CMD_HEADER__  void	SetSinWavePara( UINT8 , UINT8 ) ;		// Sin wave Test Function

__OIS_CMD_HEADER__	UINT8	RunHea( void ) ;						// Hall Examination of Acceptance
__OIS_CMD_HEADER__	UINT8	RunGea( void ) ;						// Gyro Examination of Acceptance
__OIS_CMD_HEADER__	INT16	FlashCheck_MainVerify( void );			// Flash Code verification
__OIS_CMD_HEADER__	INT16	FlashCheck_NVRVerify( void ); 			// NVR data verification

#ifdef	__OIS_MODULE_CALIBRATION__

/* Calibration Main [mandatory] */
#ifdef	__OIS_CLOSED_AF__
__OIS_CMD_HEADER__	UINT32	TneRunA( void );						// calibration with close AF
#else
__OIS_CMD_HEADER__	UINT32	TneRun( void );							// calibration for bi-direction AF
#endif
__OIS_CMD_HEADER__ INT16	WrHallCalData( void );					// upload the calibration data except gyro gain to Flash
__OIS_CMD_HEADER__ INT16	WrGyroGainData( void );					// upload the gyro gain to Flash

/* Flash Update */
__OIS_CMD_HEADER__	INT16	FlashUpdate( void ) ;					// upload DSP code to Flash
#endif

