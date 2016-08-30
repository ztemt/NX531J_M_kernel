//********************************************************************************
//		<< LC898123 Evaluation Soft >>
//********************************************************************************

//**************************
//	Include Header File		
//**************************
#include	"Ois.h"
#include	"md5.h"
#ifdef			__OIS_TYPE_XC__					// for LC898123XC
#include	"FromCode_XC.h"	// LC898123 firmware
#else
#include	"FromCode_S16N03A_MTM_LANA2.h"	// LC898123 firmware
#endif
#include	"PmemCode.h"		// firmware for burst translation

#ifdef DEBUG
#include <AT91SAM7S.h>
#include <us.h>
 #define TRACE(fmt, ...)		dbgu_printf(fmt, ## __VA_ARGS__)
 #define TRACE_DUMP(x,y)		dbg_Dump(x,y)
#else
 #define TRACE(...)
 #define TRACE_DUMP(x,y)
#endif

//#define _Upload_Ver2_
//#define   __CRC_VERIFY__		// select CRC16 for verification.

#define VERIFY_SIZE_CRC	4
#define VERIFY_SIZE_MD5	16

//****************************************************
//	CUSTOMER NECESSARY CREATING LIST
//****************************************************
/* for I2C communication */ 
extern	void RamWrite32A(unsigned int, unsigned int );
extern 	void RamRead32A( unsigned short, void * );
/* for I2C Multi Translation : Burst Mode*/
extern 	void CntWrt( void *, unsigned short) ;
extern void CntRd( unsigned int addr, void *	PcSetDat, unsigned short	UsDatNum )  ;
/* WPB control for LC898123AXD*/
extern void WPBCtrl( unsigned char UcCtrl );
/* for Wait timer [Need to adjust for your system] */ 
extern void	WitTim( unsigned short	UsWitTim );

//****************************************************
//	LOCAL RAM LIST
//****************************************************
INT16 FlashWrite_NVRVerify( void );

UINT8  NVR0_Backup[256];
UINT8  FLASH_SECTOR_BUFFER[256];
UINT8  NVR2_Backup[256];

UINT32 CRC_Reg = 0x0000ffff;


//********************************************************************************
// Function Name 	: [Extra E0 Command] IOWrite32A
// Retun Value		: None
// Argment Value	: IOadrs, IOdata
// Explanation		: Read From code version Command
// History			: First edition 						
//********************************************************************************
void IOWrite32A( UINT32 IOadrs, UINT32 IOdata )
{
#ifdef __EXTRA_E0_COMMAND__
	UINT8 UcBuf[9];
	UcBuf[0] = 0xE8;
	UcBuf[1] = (UINT8)(IOdata >> 24);
	UcBuf[2] = (UINT8)(IOdata >> 16);
	UcBuf[3] = (UINT8)(IOdata >> 8);
	UcBuf[4] = (UINT8)(IOdata >> 0);
	UcBuf[5] = (UINT8)(IOadrs >> 16);
	UcBuf[6] = (UINT8)(IOadrs >> 8);
	UcBuf[7] = (UINT8)(IOadrs >> 0);
	CntWrt( UcBuf, 8 ) ;
#else
	RamWrite32A( CMD_IO_ADR_ACCESS, IOadrs ) ;
	RamWrite32A( CMD_IO_DAT_ACCESS, IOdata ) ;					// TPGS Flash spec.  min. 2.5usec max. 3.15uec
#endif	
};


//********************************************************************************
// Function Name 	: [Extra E0 Command] IOWriteDouble32A
// Retun Value		: None
// Argment Value	: IOadrs1, IOdata1, IOadrs2, IOdata2, 
// Explanation		: Read From code version Command
// History			: First edition 						
//********************************************************************************
void IOWriteDouble32A( UINT32 IOadrs1, UINT32 IOdata1, UINT32 IOadrs2, UINT32 IOdata2 )
{
#ifdef __EXTRA_E0_COMMAND__
	UINT8 UcBuf[15];
	UcBuf[0] = 0xE8;
	UcBuf[1] = (UINT8)(IOdata1 >> 24);
	UcBuf[2] = (UINT8)(IOdata1 >> 16);
	UcBuf[3] = (UINT8)(IOdata1 >> 8);
	UcBuf[4] = (UINT8)(IOdata1 >> 0);
	UcBuf[5] = (UINT8)(IOadrs1 >> 16);
	UcBuf[6] = (UINT8)(IOadrs1 >> 8);
	UcBuf[7] = (UINT8)(IOadrs1 >> 0);
	UcBuf[8] = (UINT8)(IOdata2 >> 24);
	UcBuf[9] = (UINT8)(IOdata2 >> 16);
	UcBuf[10] = (UINT8)(IOdata2 >> 8);
	UcBuf[11] = (UINT8)(IOdata2 >> 0);
	UcBuf[12] = (UINT8)(IOadrs2 >> 16);
	UcBuf[13] = (UINT8)(IOadrs2 >> 8);
	UcBuf[14] = (UINT8)(IOadrs2 >> 0);
	CntWrt( UcBuf, 15 ) ;
#else
	RamWrite32A( CMD_IO_ADR_ACCESS, IOadrs1 ) ;
	RamWrite32A( CMD_IO_DAT_ACCESS, IOdata1 ) ;					// TPGS Flash spec.  min. 2.5usec max. 3.15uec
	RamWrite32A( CMD_IO_ADR_ACCESS, IOadrs2 ) ;
	RamWrite32A( CMD_IO_DAT_ACCESS, IOdata2 ) ;					// TPGS Flash spec.  min. 2.5usec max. 3.15uec
#endif	
};

//********************************************************************************
// Function Name 	: [Extra E0 Command] IORead4times32A
// Retun Value		: 4 read data
// Argment Value	: None
// Explanation		: Read From code version Command
// History			: First edition 						
//********************************************************************************
void IORead4times32A( UINT32* Dat )  
{
#ifdef __EXTRA_E0_COMMAND__
	UINT8 UcBuf[16];
	CntRd( 0xE8, UcBuf, 16 )  ;
	
	Dat[0] = ((UINT32)UcBuf[0]<<24) | ((UINT32)UcBuf[1]<<16) | ((UINT32)UcBuf[2]<<8) | (UINT32)UcBuf[3] ;
	Dat[1] = ((UINT32)UcBuf[4]<<24) | ((UINT32)UcBuf[5]<<16) | ((UINT32)UcBuf[6]<<8) | (UINT32)UcBuf[7] ;
	Dat[2] = ((UINT32)UcBuf[8]<<24) | ((UINT32)UcBuf[9]<<16) | ((UINT32)UcBuf[10]<<8) | (UINT32)UcBuf[11] ;
	Dat[3] = ((UINT32)UcBuf[12]<<24) | ((UINT32)UcBuf[13]<<16) | ((UINT32)UcBuf[14]<<8) | (UINT32)UcBuf[15] ;	

//TRACE("%08X \n", Dat[0] );	
//TRACE("%08X \n", Dat[1] );	
//TRACE("%08X \n", Dat[2] );	
//TRACE("%08X \n", Dat[3] );	

#else	
	RamRead32A( CMD_IO_DAT_ACCESS , &Dat[0] ) ;
	RamRead32A( CMD_IO_DAT_ACCESS , &Dat[1] ) ;
	RamRead32A( CMD_IO_DAT_ACCESS , &Dat[2] ) ;
	RamRead32A( CMD_IO_DAT_ACCESS , &Dat[3] ) ;
#endif
}


//********************************************************************************
// Function Name 	: WPB level read
// Retun Value		: 0: WPB active error , 1: WPB active
// Argment Value	: NON
// Explanation		: Read WPB level
// History			: First edition 						
//********************************************************************************
UINT8 ReadWPB( void )
{
#ifdef __OIS_TYPE_XC__					// for LC898123XC	
	return ( 1 ) ;
#else		
	UINT32	UlReadVal, UlCnt=0;

	do{
		RamWrite32A( CMD_IO_ADR_ACCESS, IOPLEVR ) ;		
		RamRead32A ( CMD_IO_DAT_ACCESS, &UlReadVal ) ;
		if( (UlReadVal & 0x0400) != 0 )	return ( 1 ) ;
		WitTim( 1 );		
	}while ( UlCnt++ < 10 );

	return ( 0 );
#endif
}

#if 0
//********************************************************************************
// Function Name 	: Verison check
// Retun Value		: EXE_ERR, EXE_END
// Argment Value	: NON
// Explanation		: Check relation the firmware version
// History			: First edition 						
//********************************************************************************
UINT16 CheckVersion( void )
{
	UINT32	UlReadVal ;

	// Confirm the firmware version of LC898123
	RamRead32A( SiVerNum, &UlReadVal );
//TRACE("ATMEL FW ver = %08xh \n", (unsigned int)VERNUM ) ;
//TRACE("DSP   FW ver = %08xh \n", (unsigned int)UlReadVal ) ;

	if ( UlReadVal != VERNUM )
		return EXE_ERR ;
	else
		return EXE_END ;
}
#endif

//********************************************************************************
// Function Name 	: Reset FlashRelease
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: <Flash Memory> Flash memory Active
// History			: First edition 						
//********************************************************************************
void FlashResetRelease(void)
{
	UINT32 UlReadVal;
	// Set RESET
	RamWrite32A( CMD_IO_ADR_ACCESS, SOFTRESET );
	RamRead32A ( CMD_IO_DAT_ACCESS, &UlReadVal );
	RamWrite32A( CMD_IO_DAT_ACCESS, (UlReadVal | 0x00000010) );// Reset release
}

//********************************************************************************
// Function Name 	: Reset Flash
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: <Flash Memory> Reset flash memory
// History			: First edition 						
//********************************************************************************
void FlashReset(void)
{
	UINT32 UlReadVal;
	// Set RESET
	RamWrite32A( CMD_IO_ADR_ACCESS, SOFTRESET ) ;
	RamRead32A ( CMD_IO_DAT_ACCESS, &UlReadVal ) ;
	RamWrite32A( CMD_IO_DAT_ACCESS, (UlReadVal & (~0x00000010)) ) ;
}


//********************************************************************************
// Function Name 	: FlashNVRSectorErase_Byte
// Retun Value		: 0: PASS, 5: WPB LOW ERROR
// Argment Value	: Address : 0 ~ 767 (Byte)  3 sesion
// Explanation		: <Flash Memory> Sector Erase
// History			: First edition 						
//********************************************************************************
INT16 FlashNVRSectorErase_Byte( UINT8 Sel )
{
	UINT8 UcCnt;
	UINT32 UlReadVal;

	if (Sel >= 3) return(1);

	// Release RESET
	FlashResetRelease();
	// Auto configuration start
	RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_WPB ) ;		// Disable write protect
	RamWrite32A( CMD_IO_DAT_ACCESS, 1 ) ;
	RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_SEL ) ;
	RamWrite32A( CMD_IO_DAT_ACCESS, 7 ) ;
	RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_ADR ) ;
	RamWrite32A( CMD_IO_DAT_ACCESS, 0x00010100	  ) ;		// FLA_NVR=1, A[8]=1, A[7..0]=0x00
	RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_ACSCNT ) ;
	RamWrite32A( CMD_IO_DAT_ACCESS, 7 ) ;
	RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_CMD ) ;
	RamWrite32A( CMD_IO_DAT_ACCESS, 7 ) ;  					// Auto configuration
	// Flash access timing Setting
	RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_TERASES ) ;
	RamWrite32A( CMD_IO_DAT_ACCESS, 0x5A ) ;					// TERASES Flash spec.  Flash spec.  min. 4msec max. 5msec
	// NVR Addres Set
	RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_ADR ) ;			// Set NVR address
	RamWrite32A( CMD_IO_DAT_ACCESS, 0x00010000	  ) ;
	// Sel Set
	RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_SEL ) ;			// Flash selector
	RamWrite32A( CMD_IO_DAT_ACCESS, (1<<Sel) ) ;
	// WP disable
//	RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_WPB ) ;			// Disable write protect
//	RamWrite32A( CMD_IO_DAT_ACCESS, 1 ) ;
	WPBCtrl(WPB_OFF) ;											// Disable write protect
	if ( ReadWPB() != 1 )	return ( 5 );						// WPB LOW ERROR
	
	// Execute
	RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_CMD ) ;
	RamWrite32A( CMD_IO_DAT_ACCESS, 4 ) ;						// Sector Erase Start
	/*5msec wait*/
	RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_INT ) ;	
	UcCnt = 0;
	do{	
		RamRead32A(  CMD_IO_DAT_ACCESS, &UlReadVal ) ;
		WitTim( 5 );
		if(++UcCnt >= 2 ){ WPBCtrl(WPB_ON); FlashReset(); return ( 2 );	}			// TimeOutError
	}while( UlReadVal == 0x80 );

	// WriteProtect Enable
	WPBCtrl(WPB_ON) ;											// Enable write protect

	FlashReset();												// ***** FLASH RESET *****
	return ( 0 );
}


//********************************************************************************
// Function Name 	: FlashNVR_ReadData_Byte
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: <Flash Memory> Read Data
// History			: First edition 						
//********************************************************************************
void FlashNVR_ReadData_Byte( UINT8 Sel, UINT8 SetAddress, UINT8 * ReadPtr, UINT16 Num )
{
	UINT16 UsNum;
	UINT32 UlReadVal;

	if(Sel >= 3) return;
	if(Num == 0 || Num > 256) return; 
	if(SetAddress + Num > 256 ) return; 

	// Release RESET
	FlashResetRelease();
	// Auto configuration start
	RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_WPB ) ;		// Disable write protect
	RamWrite32A( CMD_IO_DAT_ACCESS, 1 ) ;
	RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_SEL ) ;
	RamWrite32A( CMD_IO_DAT_ACCESS, 7 ) ;
	RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_ADR ) ;
	RamWrite32A( CMD_IO_DAT_ACCESS, 0x00010100	  ) ;		// FLA_NVR=1, A[8]=1, A[7..0]=0x00
	RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_ACSCNT ) ;
	RamWrite32A( CMD_IO_DAT_ACCESS, 7 ) ;
	RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_CMD ) ;
	RamWrite32A( CMD_IO_DAT_ACCESS, 7 ) ;  					// Auto configuration

	// Count Set
	RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_ACSCNT ) ;			// for 1 byte read count
	RamWrite32A( CMD_IO_DAT_ACCESS, Num -1 ) ;
	// Sel Set
	RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_SEL ) ;		// Flash selector
	RamWrite32A( CMD_IO_DAT_ACCESS, (1<<Sel) ) ;

	// NVR Addres Set
	RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_ADR ) ;		// Set NVR address
	RamWrite32A( CMD_IO_DAT_ACCESS, 0x00010000 +  SetAddress  ) ;
	// Read Start
	RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_CMD ) ;
	RamWrite32A( CMD_IO_DAT_ACCESS, 1 ) ;					// Read Start		

	RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_RDAT ) ;
	for ( UsNum = 0; UsNum < Num; UsNum++ )
	{
		RamRead32A(  CMD_IO_DAT_ACCESS, &UlReadVal ) ;
		ReadPtr[ UsNum ] = (UINT8)(UlReadVal>>(Sel*8));
	}

	FlashReset();												// ***** FLASH RESET *****
}

//********************************************************************************
// Function Name 	: FlashNVR_WriteData_Byte
// Retun Value		: 0: PASS, 5: WPB_LOW ERROR
// Argment Value	: NON
// Explanation		: <Flash Memory> Read Data
// History			: First edition 						
//********************************************************************************
INT16 FlashNVR_WriteData_Byte( UINT8 Sel,UINT8 SetAddress, UINT8 * WritePtr, UINT16 Num )
{
	UINT16 UsNum;

	if(Sel >= 3) return(1);
	if(Num == 0 || Num > 256) return(1); 
	if(SetAddress + Num > 256 ) return(1); 

	// Release RESET
	FlashResetRelease();
	// Auto configuration start
	RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_WPB ) ;		// Disable write protect
	RamWrite32A( CMD_IO_DAT_ACCESS, 1 ) ;
	RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_SEL ) ;
	RamWrite32A( CMD_IO_DAT_ACCESS, 7 ) ;
	RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_ADR ) ;
	RamWrite32A( CMD_IO_DAT_ACCESS, 0x00010100	  ) ;		// FLA_NVR=1, A[8]=1, A[7..0]=0x00
	RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_ACSCNT ) ;
	RamWrite32A( CMD_IO_DAT_ACCESS, 7 ) ;
	RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_CMD ) ;
	RamWrite32A( CMD_IO_DAT_ACCESS, 7 ) ;  					// Auto configuration
	// Flash access timing Setting
	RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_TPGS ) ;
	RamWrite32A( CMD_IO_DAT_ACCESS, 0x73 ) ;					// TPGS Flash spec.  min. 2.5usec max. 3.15uec
	RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_TPROG ) ;
	RamWrite32A( CMD_IO_DAT_ACCESS, 0x43 ) ;					// TPROG Flash spec.  min. 6usec max. 7.5usec
	// WP disable	
//	RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_WPB ) ;			// Disable write protect
//	RamWrite32A( CMD_IO_DAT_ACCESS, 1 ) ;
	WPBCtrl(WPB_OFF) ;											// Disable write protect
	if ( ReadWPB() != 1 )	return ( 5 );						// WPB LOW ERROR
	// Count Set
	RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_ACSCNT ) ;			// for 1 byte read count
	RamWrite32A( CMD_IO_DAT_ACCESS, 0) ;						// ここは必ず0にしなければならない
	// Sel Set
	RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_SEL ) ;		// Flash selector
	RamWrite32A( CMD_IO_DAT_ACCESS, (1<<Sel) ) ;
	// NVR Addres Set
	RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_ADR ) ;		// Set NVR address
	RamWrite32A( CMD_IO_DAT_ACCESS, 0x00010000 + SetAddress ) ;
	for ( UsNum = 0; UsNum < Num; UsNum++ )
	{
		// Data Set
		RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_WDAT ) ;		// Ser write data
		RamWrite32A( CMD_IO_DAT_ACCESS, ((UINT32)(WritePtr[ UsNum ] ) <<(Sel*8)) ) ;
		// Execute
		RamWrite32A( CMD_IO_ADR_ACCESS , FLASHROM_CMD ) ;
		RamWrite32A( CMD_IO_DAT_ACCESS , 2) ;					// Program Start
		/*20usec wait*/
	}
	// WriteProtect
	RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_WPB ) ;			// Enable write protect
	RamWrite32A( CMD_IO_DAT_ACCESS, 0 ) ;
	WPBCtrl(WPB_ON) ;											// Enable write protect

	FlashReset();												// ***** FLASH RESET *****
	return ( 0 );
}


//********************************************************************************
// Function Name 	: PmemWriteByBoot
// Retun Value		: 0: OK 1:  Verify Error
// Argment Value	: 
// Explanation		: Read From code version Command
// History			: First edition 						
//********************************************************************************
UINT8 PmemWriteByBoot( void )
{
	UINT8  UcCnt;	
	UINT32 UlCnt;
	UINT32 UlNum;
      UINT32 *pExtraPmemCode = 0;
	
#ifdef _Upload_Ver2_
	RamWrite32A( PMEMCMD + 4,  20 );
	for ( UlCnt = 0, UlNum = 0x80000; UlCnt < (sizeof (ContinuouslyTranslationCode))/4 ; UlCnt+=5, UlNum += 8 )
	{
	    UINT32 UlData[5];
		RamWrite32A( PMEMCMD + 8, UlNum );
		RamWrite32A( PMEMCMD + 12       ,  ContinuouslyTranslationCode[0+ UlCnt] );
		RamWrite32A( PMEMCMD + 12 + 4	,  ContinuouslyTranslationCode[1+ UlCnt] );
		RamWrite32A( PMEMCMD + 12 + 8	,  ContinuouslyTranslationCode[2+ UlCnt] );
		RamWrite32A( PMEMCMD + 12 + 12	,  ContinuouslyTranslationCode[3+ UlCnt] );
		RamWrite32A( PMEMCMD + 12 + 16	,  ContinuouslyTranslationCode[4+ UlCnt] );
		RamWrite32A( PMEMCMD, 1 );
#if 1	// Read check
		RamWrite32A( PMEMCMD, 2 );
		RamRead32A( PMEMCMD + 12 		,	&(UlData[0]) );
		RamRead32A( PMEMCMD + 12 + 4 	,	&(UlData[1]) );
		RamRead32A( PMEMCMD + 12 + 8 	,	&(UlData[2]) );
		RamRead32A( PMEMCMD + 12 + 12	,	&(UlData[3]) );
		RamRead32A( PMEMCMD + 12 + 16	,	&(UlData[4]) );
		for (UcCnt=0; UcCnt<5; UcCnt++){
			if( ContinuouslyTranslationCode[UcCnt+UlCnt] != UlData[UcCnt] )	return(2);	// PMEM Copy Error 
		}
#endif
	}
#else
	UINT8 UcBuf[16];

	RamWrite32A( PMEMCMD + 4,  20 );
      pExtraPmemCode = &ExtraPmemCode[0];
	for ( UlCnt = 0, UlNum = 0x81280; UlCnt < (sizeof (ExtraPmemCode)/4) ; UlCnt+=5, UlNum += 8 )
	{
		RamWrite32A( PMEMCMD + 8, UlNum );
		RamWrite32A( PMEMCMD + 12       ,    *(pExtraPmemCode++) );
		RamWrite32A( PMEMCMD + 12 + 4	,  *(pExtraPmemCode++) );
		RamWrite32A( PMEMCMD + 12 + 8	,  *(pExtraPmemCode++) );
		RamWrite32A( PMEMCMD + 12 + 12	,  *(pExtraPmemCode++) );
		RamWrite32A( PMEMCMD + 12 + 16	,  *(pExtraPmemCode++) );
		RamWrite32A( PMEMCMD, 1 );
	}
	
	RamWrite32A( CmdEventCommand, 0x081280 ) ;		// Execute

	for ( UlCnt = 0; UlCnt <=  sizeof (ContinuouslyTranslationCode) ; UlCnt+=15 )
	{
		UcBuf[0] = 0xEE;
		for (UcCnt=0; UcCnt<15; UcCnt++){
			UcBuf[ UcCnt +1 ] = ContinuouslyTranslationCode[ UcCnt+UlCnt ];
		}
		CntWrt( UcBuf, 16 ) ;
	}

	UcBuf[0] = 0xEF;
	UcBuf[1] = 0xFF;
	CntWrt( UcBuf, 2 ) ;				// Execute
#endif
	return(0);
}



//********************************************************************************
// Function Name 	: FlashUpdateMain
// Retun Value		: 0: PASS, 1: MAGIC CODE ERASED, 2: PMEM-DOWNLOAD ERROR 3: VERIFY ERROR 
//					: 4: ES type Error 6:TIMEOUT ERROR
// Argment Value	: NON
// Explanation		: Flash Write Hall Calibration Data Function
// History			: First edition 						
//********************************************************************************
UINT8 FlashUpdateMain( const UINT8* NVRUploadTable, const UINT32* MainUploadTable, const UINT8* CRCParity, const UINT8* MD5Parity )
{
	UINT32 UlNum;
	UINT32 UlReadVal; 
	UINT8  UcData[ 64 ] ;
	UINT8  UcCnt;
//--------------------------------------------------------------------------------
// 0. Start up to boot exection 
//--------------------------------------------------------------------------------
	// Release RESET
	FlashResetRelease();				/* DownloadCodeにはFlashReleaseがないのでAutoConfigを含めて再設定必要 */
	// Auto configuration start	
	RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_WPB ) ;		// Disable write protect
	RamWrite32A( CMD_IO_DAT_ACCESS, 1 ) ;
	RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_SEL ) ;
	RamWrite32A( CMD_IO_DAT_ACCESS, 7 ) ;
	RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_ADR ) ;
	RamWrite32A( CMD_IO_DAT_ACCESS, 0x00010100	 ) ;		// FLA_NVR=1, A[8]=1, A[7..0]=0x00
	RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_ACSCNT ) ;
	RamWrite32A( CMD_IO_DAT_ACCESS, 7 ) ;
	RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_CMD ) ;
	RamWrite32A( CMD_IO_DAT_ACCESS, 7 ) ;  					// Auto configuration

	RamWrite32A( CMD_IO_ADR_ACCESS, SYS_DSP_REMAP) ;			// Read remap flag
	RamRead32A(  CMD_IO_DAT_ACCESS, &UlReadVal ) ;
	if( UlReadVal != 0) {	
		// Flash access timing Setting
		RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_TPGS ) ;
		RamWrite32A( CMD_IO_DAT_ACCESS, 0x73 ) ;					// TPGS Flash spec.  min. 2.5usec max. 3.15uec
		RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_TPROG ) ;
		RamWrite32A( CMD_IO_DAT_ACCESS, 0x43 ) ;					// TPROG Flash spec.  min. 6usec max. 7.5usec

		WPBCtrl(WPB_OFF) ;										// Disable write protect
		if ( ReadWPB() != 1 )	return ( 5 );					// WPB LOW ERROR

		// Magic code erase
//		RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_WPB ) ;		// Disable write protect
//		RamWrite32A( CMD_IO_DAT_ACCESS, 1 ) ;
		RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_ADR ) ;		// NVR SELECT
		RamWrite32A( CMD_IO_DAT_ACCESS, 0x00010001 ) ;			// Set address of Magic code[1]
		RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_ACSCNT	 ) ;
		RamWrite32A( CMD_IO_DAT_ACCESS, 0 ) ;					// for 1 byte program
		RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_SEL ) ;		// Flash selector
		RamWrite32A( CMD_IO_DAT_ACCESS, 1 ) ;
		RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_WDAT ) ;		// write data '00' set
		RamWrite32A( CMD_IO_DAT_ACCESS, 0 ) ;
		RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_CMD ) ;		// Address 0x0001
		RamWrite32A( CMD_IO_DAT_ACCESS, 2 ) ;  					// 1 byte Program
		/*20usec wait*/
		RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_WPB ) ;		// Enable write protect
		RamWrite32A( CMD_IO_DAT_ACCESS, 0 ) ;
		WPBCtrl(WPB_ON) ;										// Enable write protect

		RamWrite32A(CMD_REBOOT , 0 ) ;							// exit from boot
		UcCnt = 0;
		do{		
			WitTim( 40 );		
			/* ここはReboot状態なので、コマンドを連続して送らなければならない */
			RamWrite32A( CMD_IO_ADR_ACCESS, SYS_DSP_REMAP) ;		// Read remap flag  
			RamRead32A(  CMD_IO_DAT_ACCESS, &UlReadVal ) ;  	// 0.35sec (by 400kz)
			if(++UcCnt >= 10 )		return ( 1 );				// TimeOutError
		}while(UlReadVal != 0);
	}
//--------------------------------------------------------------------------------
// 1. PMEM code donload for continuously I2C translation 
//--------------------------------------------------------------------------------
	// AXC AXD check for FromCode
	RamWrite32A( CMD_IO_ADR_ACCESS, CVER_123 ) ;
	RamRead32A(  CMD_IO_DAT_ACCESS, &UlReadVal) ;
#ifdef __OIS_TYPE_XC__					// for LC898123XC	
	if ( (UINT8)UlReadVal== 0xB3 || (UINT8)UlReadVal == 0xB5 )
#else
	if ( (UINT8)UlReadVal== 0xB4 || (UINT8)UlReadVal == 0xB6 )
#endif
	{
		if( PmemWriteByBoot() != 0) return (2);
	}else{
		return (4);
	}

//--------------------------------------------------------------------------------
// 2. PMEM Execute & Full Erase 
//--------------------------------------------------------------------------------
	WPBCtrl(WPB_OFF) ;										// Disable write protect
	if ( ReadWPB() != 1 )	return ( 5 );					// WPB LOW ERROR

	/* Remap Execution (average necessary time 40 msec ) */
	RamWrite32A( CMD_REMAP, 0 ) ;								// Remap Command
	
	UcCnt = 0;
	do{		// Max.100msec : 
		WitTim( 40 );
		CntRd( 0xE0, UcData, 1 ) ;						// 0.1sec (by 400kz)
		if(++UcCnt >= 4 ){
			/* Reboot without automatic Flash download */
			UcData[ 0 ] = 0xE0;		// Reboot Commnand
			UcData[ 1 ] = 0x01;
			CntWrt( UcData, 2 ) ;
			return ( 6 );		// TimeOutError
		}
	}while(UcData[0] == 0);
//--------------------------------------------------------------------------------
// 3. Updata Main Code translation
//--------------------------------------------------------------------------------
	/* Main translation to work Ram */
	for( UlNum=0; UlNum< 4096 ; UlNum += 32 ){ 
		UcData[ 0 ] = 0xE1;		// Main Translation 0 Commnand
		for(UcCnt = 0; UcCnt < 16; UcCnt++ ){
			UcData[ (UcCnt * 3) + 1] = (UINT8)(MainUploadTable[ UcCnt + UlNum ]  );
			UcData[ (UcCnt * 3) + 2] = (UINT8)(MainUploadTable[ UcCnt + UlNum ] >> 8 );
			UcData[ (UcCnt * 3) + 3] = (UINT8)(MainUploadTable[ UcCnt + UlNum ] >>16);
//			TRACE("%02X%02X%02X\n", UcData[ (UcCnt * 3) + 1], UcData[ (UcCnt * 3) + 2], UcData[ (UcCnt * 3) + 3] );
		}	
		CntWrt( UcData, 49 ) ;

		UcData[ 0 ] = 0xE2;		// Main Translation 1 Commnand
		for(UcCnt = 0; UcCnt < 16; UcCnt++ ){
			UcData[ (UcCnt * 3) + 1] = (UINT8)(MainUploadTable[ UcCnt + UlNum +16 ]  );
			UcData[ (UcCnt * 3) + 2] = (UINT8)(MainUploadTable[ UcCnt + UlNum +16 ] >> 8 );
			UcData[ (UcCnt * 3) + 3] = (UINT8)(MainUploadTable[ UcCnt + UlNum +16 ] >>16);
//			TRACE("%02X%02X%02X\n", UcData[ (UcCnt * 3) + 1], UcData[ (UcCnt * 3) + 2], UcData[ (UcCnt * 3) + 3] );
		}	
		CntWrt( UcData, 49 ) ;
	}
	// Wait for last flash writing finalization
	UcCnt = 0;
	do{		// Max.2msec :
		WitTim( 1 );
		CntRd( 0xE2, UcData, 1 ) ;						// 0.1sec (by 400kz)
		if(++UcCnt >= 3 ){
			/* Reboot without automatic Flash download */
			UcData[ 0 ] = 0xE5;		// Reboot Commnand
			UcData[ 1 ] = 0x01;
			CntWrt( UcData, 2 ) ;
			return ( 6 );			// TimeOutError
		}
	}while(UcData[0] != 0);
//--------------------------------------------------------------------------------
// 4. Update NVR Data translation
//--------------------------------------------------------------------------------
	/* NVR0 area translation to work Ram (average time 1~2 msec) */
	UcData[ 0 ] = 0xE3;		// NVR0 Translation Commnand
	for(UcCnt = 0; UcCnt < 32; UcCnt++ ){
		UcData[ UcCnt+ 1] = NVRUploadTable[ UcCnt ] ;
	}	
	CntWrt( UcData, 33 ) ;
	UcCnt = 0;
	do{		// Max.2msec :
		WitTim( 1 );
		CntRd( 0xE3, UcData, 1 ) ;						// 0.1sec (by 400kz)
		if(++UcCnt >= 3 ){
			/* Reboot without automatic Flash download */
			UcData[ 0 ] = 0xE5;		// Reboot Commnand
			UcData[ 1 ] = 0x01;
			CntWrt( UcData, 2 ) ;
			return ( 6 );			// TimeOutError
		}
	}while(UcData[0] != 0);
//--------------------------------------------------------------------------------
// 5. Verify execute
//--------------------------------------------------------------------------------
	/* Verify Start  */
	UcData[ 0 ] = 0xE4;		// Verify Start Command
	UcData[ 1 ] = 0x01;
	for(UcCnt = 0; UcCnt < 16 ; UcCnt++ ){
		UcData[ UcCnt+2 ] = CcHashCode[ UcCnt ];
	}
	CntWrt( UcData, 18 ) ;	

	UcCnt = 0;
	do{		// Max.100msec :
		WitTim( 20 );
		CntRd( 0xE4, UcData, 1 ) ;						// 0.1sec (by 400kz)
		if(++UcCnt >= 8 )
		{
			/* Reboot without automatic Flash download */
			UcData[ 0 ] = 0xE5;		// Reboot Commnand
			UcData[ 1 ] = 0x01;
			CntWrt( UcData, 2 ) ;
			return ( 6 );			// TimeOutError
		}
	}while(UcData[0] != 0);
//--------------------------------------------------------------------------------
// 6. Verify Check
//--------------------------------------------------------------------------------
	WPBCtrl(WPB_ON) ;										// Enable write protect

	/* Verify Result Read  */
	CntRd( 0xE5, UcData, VERIFY_SIZE_CRC + VERIFY_SIZE_MD5 ) ;
//	TRACE("CRC verify: %02X%02X%02X%02X\n", UcData[0], UcData[1], UcData[2], UcData[3] );
//	TRACE("MD5 verify: %02X%02X%02X%02X", UcData[4], UcData[5], UcData[6], UcData[7] );
//	TRACE("%02X%02X%02X%02X", UcData[8], UcData[9], UcData[10], UcData[11] );
//	TRACE("%02X%02X%02X%02X", UcData[12], UcData[13], UcData[14], UcData[15] );
//	TRACE("%02X%02X%02X%02X\n", UcData[16], UcData[17], UcData[18], UcData[19] );

	for(UcCnt = 0; UcCnt < VERIFY_SIZE_CRC; UcCnt++ ){
		if(UcData[UcCnt] != CRCParity[UcCnt]){
			/* Reboot without automatic Flash download */
			UcData[ 0 ] = 0xE5;		// Reboot Commnand
			UcData[ 1 ] = 0x01;
			CntWrt( UcData, 2 ) ;
//TRACE("FAIL CRC");
			return( 3 );	// Fail
		}
	}

	for(UcCnt = 0; UcCnt < VERIFY_SIZE_MD5; UcCnt++ ){
		if(UcData[UcCnt+ VERIFY_SIZE_CRC] != MD5Parity[UcCnt]){
			/* Reboot without automatic Flash download */
			UcData[ 0 ] = 0xE5;		// Reboot Commnand
			UcData[ 1 ] = 0x01;
			CntWrt( UcData, 2 ) ;
//TRACE("FAIL MD5");
			return( 3 );	// Fail
		}
	}

	/* Reboot with automatic Flash download */
	UcData[ 0 ] = 0xE5;		// Reboot Commnand
	UcData[ 1 ] = 0x00;
	CntWrt( UcData, 2 ) ;
//TRACE("SUCCESS");
//--------------------------------------------------------------------------------
// 7. Update NVR MD5
//--------------------------------------------------------------------------------
	UcCnt = 0;
	do{		
		WitTim( 40 );		
		/* ここはReboot状態なので、コマンドを連続して送らなければならない */
		RamWrite32A( CMD_IO_ADR_ACCESS, SYS_DSP_REMAP) ;		// Read remap flag  
		RamRead32A(  CMD_IO_DAT_ACCESS, &UlReadVal ) ;  	// 0.35sec (by 400kz)
		if(++UcCnt >= 10 )		return ( 1 );				// TimeOutError
	}while(UlReadVal == 0);

	return ( FlashWrite_NVRVerify() );

}

//********************************************************************************
// Function Name 	: FlashUpdate
// Retun Value		: 0: PASS, 1: MAGIC CODE ERASED, 2: VERIFY ERROR 3: NVR VERIFY ERROR
//					: 4: LSI ERROR, 5: WPB LOW ERROR
// Argment Value	: NON
// Explanation		: Flash Write Hall Calibration Data Function
// History			: First edition 						
//********************************************************************************
#define REPEAT_NUM 2
INT16 FlashUpdate(void)
{
	UINT8 UcCnt;
	UINT8 UcAns;
	
	for(UcCnt = 0; UcCnt < REPEAT_NUM; UcCnt++ ){
		UcAns = FlashUpdateMain( CcMagicCode, ClFromCode, CcCRC16Code, CcHashCode  );
		if( UcAns == 0 ){
			break;
		}else{
			/* WPB port control forcely low voltage & Flash circuit disable forcely*/
			WPBCtrl(WPB_ON) ;
			FlashReset();	
			WitTim( 50 );
		}
	}
	return ( (INT16)UcAns );
}


/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
/* function name    : CCITT_CRC16                                                        */
/* input parameter  :                                                                    */
/* output parameter :                                                                    */
/* comment          : CCITT CRC-16(FAX)                                                  */
/*                    CRC演算は、誤り検出能力は大きいが、シリアル処理のため負荷が重い    */
/*                                                                            2015.08.25 */
/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
void CRC16_main( UINT8 *p, int Num )
{
	unsigned int tmp0, tmp5, tmp12;
	unsigned int temp, data;
      int i = 0, j = 0;

	for(i=0 ; i<Num ; i++) {
		temp = (unsigned int)*p++;		// データを設定

		for(j=0 ; j<8 ; j++) {
			data = temp & 0x00000001;	// 1bit(LSB)を抽出
			temp = temp >> 1;

            tmp0 = ((CRC_Reg >> 15) ^ data) & 0x00000001;
            tmp5 = (((tmp0 << 4) ^ CRC_Reg) & 0x00000010) << 1;
            tmp12 = (((tmp0 << 11) ^ CRC_Reg) & 0x00000800) << 1;
            CRC_Reg = (CRC_Reg << 1) & 0x0000efde;
            CRC_Reg = CRC_Reg | tmp0 | tmp5 | tmp12;
		}
	}
}

//********************************************************************************
// Function Name 	: FlashCheck_MainVerify
// Retun Value		: INT16 0:Ok, 1:NG main array, 2:NG NVR, 
// Argment Value	: NON
// Explanation		: Main area's accuracy check by MD5 verification
// History			: First edition 						
//********************************************************************************
INT16 FlashCheck_MainVerify( void )
{
	INT16 iResult = 0;
	UINT32 UlNum;
	UINT32 UlReadVal[4];
	UINT8 UcFlaData[3];
	UINT8 UcCnt;
	
	/* Main */
#ifdef __CRC_VERIFY__
	UINT8 pCRC[2];
	CRC_Reg = 0x0000ffff;
#else
	UINT8 pMD5[16];
    md5_context ctx;
	md5_starts( &ctx );
#endif
	// Release RESET
	FlashResetRelease();		// Reset release
	IOWrite32A( FLASHROM_WPB , 1 );				// Disable write protect
	IOWrite32A( FLASHROM_SEL  , 7 );			// Disable write protect
	IOWrite32A( FLASHROM_ADR  , 0x00010100 );	// FLA_NVR=1, A[8]=1, A[7..0]=0x00
	IOWrite32A( FLASHROM_ACSCNT  ,  7 );		// for 7 times repeat
	IOWrite32A( FLASHROM_CMD  ,  7 );			// Auto configuration

	// Main Area Read 
//	IOWrite32A( FLASHROM_SEL , 7 );				// Flash selector
	IOWrite32A( FLASHROM_ADR , 0 );					// Set NVR address
	IOWrite32A( FLASHROM_ACSCNT , (4096 -1) );		// for 256 byte read count
	IOWrite32A( FLASHROM_CMD , 1 );		// Read Start
	RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_RDAT ) ;
	for (UlNum= 0 ; UlNum< 4096 ; UlNum+=4)
	{
		IORead4times32A( UlReadVal )  ;
		for (UcCnt= 0 ; UcCnt< 4 ; UcCnt++){		
#ifdef __CRC_VERIFY__
			// LSB first 
			UcFlaData[0] = UlReadVal[UcCnt] & 0xFF;
			UcFlaData[1] = (UlReadVal[UcCnt] >> 8) & 0xFF;
			UcFlaData[2] = (UlReadVal[UcCnt] >> 16) & 0xFF;
//TRACE("%02X%02X%02X\n", UcFlaData[2], UcFlaData[1], UcFlaData[0] );	
			CRC16_main( UcFlaData, 3 );
		}
	}
	pCRC[0] = (UINT8)(CRC_Reg>>8);
	pCRC[1] = (UINT8)CRC_Reg;

//	IOWrite32A( FLASHROM_SEL , 4 );						// Flash selector
	IOWrite32A( FLASHROM_ACSCNT , (2 -1) );				// for 256 byte read count
	IOWrite32A( FLASHROM_ADR , (0x00010000 + 0x20) );	// Set NVR address
	IOWrite32A( FLASHROM_CMD , 1 );						// Read Start

	RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_RDAT ) ;
	for (UlNum= 0 ; UlNum< 2 ; UlNum++){
		RamRead32A(  CMD_IO_DAT_ACCESS, &UlReadVal[0] ) ;
//TRACE("%02X, %02X\n", UlReadVal[0], pCRC[ UlNum] ) );
		if ((UINT8)(UlReadVal[0]>>16) != pCRC[ UlNum] ) {
			iResult = 1;	break;
		}
	}
#else
			// MSB first 	
			UcFlaData[0] = (UlReadVal[UcCnt] >> 16) & 0xFF;
			UcFlaData[1] = (UlReadVal[UcCnt] >> 8) & 0xFF;
			UcFlaData[2] = UlReadVal[UcCnt] & 0xFF;
//TRACE("%02X%02X%02X\n", UcFlaData[0], UcFlaData[1], UcFlaData[2] );	
			md5_update( &ctx, (UINT8 *)UcFlaData, 3 );
		}
	}
	md5_finish( &ctx, pMD5 );

//	IOWrite32A( FLASHROM_SEL , 4 );						// Flash selector
	IOWrite32A( FLASHROM_ACSCNT , (16 -1) );			// for 256 byte read count
	IOWrite32A( FLASHROM_ADR , 0x00010000  );			// Set NVR address
	IOWrite32A( FLASHROM_CMD , 1 );						// Read Start

	RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_RDAT ) ;
	for (UlNum= 0 ; UlNum< 16 ; UlNum+=4){

		IORead4times32A( UlReadVal )  ;

		for (UcCnt= 0 ; UcCnt< 4 ; UcCnt++){
//TRACE("%02X, %02X\n", pMD5[UlNum +UcCnt], (UINT8)(UlReadVal[UcCnt]>>16) );
			if ((UINT8)(UlReadVal[UcCnt]>>16) != pMD5[UlNum +UcCnt] ){
				iResult = 1;	break;
			}
		}
	}
#endif
	// Set RESET
	FlashReset();

	return ( iResult  );
}

//********************************************************************************
// Function Name 	:  FlashCheck_NVRVerify
// Retun Value		: INT16 0:Ok, 5: WPB LOW ERROR
// Argment Value	: NON
// Explanation		: NVR area's accuracy check by MD5 verification
// History			: First edition 						
//********************************************************************************
INT16 FlashCheck_NVRVerify( void )
{
	INT16 iResult = 0;
	UINT32 UlNum;
	UINT32 UlReadVal[4];	
	UINT8 UcNvrData[2];
	UINT8 UcCnt;
	
	/* NVR */	
#ifdef __CRC_VERIFY__
	CRC_Reg = 0x0000ffff;
#else
	UINT8 pMD5[16];
    md5_context ctx;
	md5_starts( &ctx );
#endif	

	// Release RESET
	FlashResetRelease();		// Reset release
	IOWrite32A( FLASHROM_WPB , 1 );				// Disable write protect
	IOWrite32A( FLASHROM_SEL  , 7 );			// Disable write protect
	IOWrite32A( FLASHROM_ADR  , 0x00010100 );	// FLA_NVR=1, A[8]=1, A[7..0]=0x00
	IOWrite32A( FLASHROM_ACSCNT  ,  7 );		// for 7 times repeat
	IOWrite32A( FLASHROM_CMD  ,  7 );			// Auto configuration

	// NVR Read
//	IOWrite32A( FLASHROM_SEL , 7 );				// Flash selector
	IOWrite32A( FLASHROM_ADR , 0x00010000 );	// Set NVR address
	IOWrite32A( FLASHROM_ACSCNT , (256 - 1) );	// for 256 byte read count
	IOWrite32A( FLASHROM_CMD , 1 );				// Read Start
	RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_RDAT ) ;
	for ( UlNum = 0; UlNum < 256; UlNum+=4 )
	{
		IORead4times32A( UlReadVal )  ;
		for (UcCnt= 0 ; UcCnt< 4 ; UcCnt++){
#ifdef __CRC_VERIFY__
			// LSB first 
			UcNvrData[0] = UlReadVal[UcCnt]  & 0xFF;				// NVR1_1
			UcNvrData[1] = (UlReadVal[UcCnt]  >> 8) & 0xFF;			// NVR1_2
			NVR2_Backup[ UlNum + UcCnt ] = (UlReadVal[UcCnt]  >> 16) & 0xFF;		
//TRACE("%02X%02X%02X\n", UcExpect[ UlNum + UcCnt ], UcNvrData[1], UcNvrData[0] );	
			CRC16_main( UcNvrData, 2 );
		}
	}
//TRACE("%02X, %02X\n", UcExpect[ 0x22 + UlNum ], pCRC[ UlNum] );
	if( NVR2_Backup[ 0x22 ] != (CRC_Reg>>8) ) iResult = 1;
	if( NVR2_Backup[ 0x23 ] != CRC_Reg 	 ) iResult = 1;
#else
			// MSB first 		
			UcNvrData[0] = (UlReadVal[UcCnt]  >> 8) & 0xFF;			// NVR1_2
			UcNvrData[1] = UlReadVal[UcCnt]  & 0xFF;				// NVR1_1
			NVR2_Backup[ UlNum + UcCnt ] = (UlReadVal[UcCnt]  >> 16) & 0xFF;				
			md5_update( &ctx, (UINT8 *)UcNvrData, 2 );
		}
	}
	md5_finish( &ctx, pMD5 );
	for (UlNum= 0 ; UlNum< 16 ; UlNum++){
		if (NVR2_Backup[ 0x10 + UlNum ] != pMD5[ UlNum ] ){	
			iResult = 1;	break;
		}
	}
#endif
	// Set RESET
	FlashReset();
	return ( iResult );
}


//********************************************************************************
// Function Name 	: FlashWrite_NVRVerify
// Retun Value		: INT16 0:Ok, 5: WPB LOW ERROR
// Argment Value	: NON
// Explanation		: NVR area's accuracy check by MD5 verification
// History			: First edition 						
//********************************************************************************
INT16 FlashWrite_NVRVerify( void )
{
	UINT16 UsNum;
	UINT8 UcNvrData[2];
	UINT32 UlReadVal[4];	
	UINT8 UcCnt;
    md5_context ctx;
	CRC_Reg = 0x0000ffff;
	
	// Release RESET
	FlashResetRelease();		// Reset release
	IOWrite32A( FLASHROM_WPB , 1 );				// Disable write protect
	IOWrite32A( FLASHROM_SEL  , 7 );			// Disable write protect
	IOWrite32A( FLASHROM_ADR  , 0x00010100 );	// FLA_NVR=1, A[8]=1, A[7..0]=0x00
	IOWrite32A( FLASHROM_ACSCNT  ,  7 );		// for 7 times repeat
	IOWrite32A( FLASHROM_CMD  ,  7 );			// Auto configuration
	// Flash access timing Setting
	IOWrite32A( FLASHROM_TPGS, 118 );			// TPGS Flash spec.  min. 2.5usec max. 3.15uec
	IOWrite32A( FLASHROM_TPROG , 70 );			// TPROG Flash spec.  min. 6usec max. 7.5usec
	IOWrite32A( FLASHROM_TERASES , 92 );		// TERASES Flash spec.  Flash spec.  min. 4msec max. 5msec
//--------------------------------------------------------------------------------
// 0. Read All NVR ( Backup sequence )
//--------------------------------------------------------------------------------
	IOWrite32A( FLASHROM_ADR , 0x00010000 );	// Set NVR address
//	IOWrite32A( FLASHROM_SEL , 7 );				// Flash selector
	IOWrite32A( FLASHROM_ACSCNT , (256 - 1) );	// for 256 byte read count
	IOWrite32A( FLASHROM_CMD , 1 );				// Read Start
	RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_RDAT ) ;
	for ( UsNum = 0; UsNum <= 0xFF;  UsNum+=4 )
	{
		IORead4times32A( UlReadVal )  ;
		for (UcCnt= 0 ; UcCnt< 4 ; UcCnt++){
			NVR0_Backup[UsNum + UcCnt]   = (UINT8)UlReadVal[UcCnt];
			FLASH_SECTOR_BUFFER[UsNum + UcCnt]	     = (UINT8)(UlReadVal[UcCnt]>>8);
			NVR2_Backup[UsNum + UcCnt]   = (UINT8)(UlReadVal[UcCnt]>>16);
		}
	}
//--------------------------------------------------------------------------------
// 1. NVR Erase except NVR0
//--------------------------------------------------------------------------------
	// WP disable
	IOWrite32A( FLASHROM_WPB , 1 );							// Disable write protect
	WPBCtrl(WPB_OFF) ;										// Disable write protect
	if ( ReadWPB() != 1 ){ FlashReset(); return ( 5 );}		// WPB LOW ERROR

	IOWrite32A( FLASHROM_ADR , 0x00010000 );				// Set NVR address
	IOWrite32A( FLASHROM_SEL , 0x04 /*NVR2*/ );				// Flash selector
	IOWrite32A( FLASHROM_CMD , 4 );							// Sector Erase Start	
	/*5msec wait*/
//--------------------------------------------------------------------------------
// 2. Update Calibration Code & Create Verify data
//--------------------------------------------------------------------------------
	// Update Verify Code	
	for ( UsNum = 0; UsNum <= 0xFF; UsNum++ )	
	{
		// LSB first 	
		UcNvrData[0] = NVR0_Backup[ UsNum ];	// NVR0
		UcNvrData[1] = FLASH_SECTOR_BUFFER[ UsNum ];		// NVR1	
		CRC16_main( UcNvrData, 2 );
	}
	NVR2_Backup[ 0x22 ] = (UINT8)(CRC_Reg>>8);
	NVR2_Backup[ 0x23 ] = (UINT8)CRC_Reg;
	md5_starts( &ctx );
	for ( UsNum = 0; UsNum <= 0xFF; UsNum++ )	
	{
		// MSB first 		
		UcNvrData[0] = FLASH_SECTOR_BUFFER[ UsNum ];		// NVR1	
		UcNvrData[1] = NVR0_Backup[ UsNum ];	// NVR0
		md5_update( &ctx, UcNvrData, 2);
	}
	md5_finish( &ctx, &(NVR2_Backup[ 0x10 ]) );
//--------------------------------------------------------------------------------
// 3. Write Verify data to NVR
//--------------------------------------------------------------------------------
	// check whether sector erase function finished or not.
	RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_INT ) ;	
	for ( UsNum  = 0; UsNum  < 10; UsNum ++ )						// TimeOut 
	{
		RamRead32A(  CMD_IO_DAT_ACCESS, UlReadVal ) ;
		if( !(UlReadVal[0] ==  0x80) ){
			break;
		}
		WitTim( 2 );
	}
//	IOWrite32A( FLASHROM_SEL , 0x04 /*NVR2*/ );			// Flash selector
	IOWrite32A( FLASHROM_ACSCNT , 0 );						// should be set "0"
	IOWrite32A( FLASHROM_ADR , 0x00010000 );				// Set NVR address
	for ( UsNum = 0; UsNum <= 0x24; UsNum++ )				// Max. 127times
	{
		IOWriteDouble32A( FLASHROM_WDAT, ((UINT32)(NVR2_Backup[UsNum])<<16),
						  FLASHROM_CMD,   2 );	
		/*20usec wait*/
	}
	IOWrite32A( FLASHROM_WPB, 0  );							// Enable write protect
	WPBCtrl(WPB_ON) ;										// Enable write protect
//--------------------------------------------------------------------------------
// 4. Read Verify 
//--------------------------------------------------------------------------------
//	IOWrite32A( FLASHROM_SEL , 0x04 /*NVR1,2*/ );			// Flash selector
	IOWrite32A( FLASHROM_ADR, 0x00010000  );				// Set NVR address
	IOWrite32A( FLASHROM_ACSCNT, (0x24 -1)  );				// for 256 byte read count
	IOWrite32A( FLASHROM_CMD , 1  );						// Read Start

	RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_RDAT ) ;
	for ( UsNum = 0; UsNum <= (0x24-1); UsNum+=4 )
	{
		IORead4times32A( UlReadVal )  ;
		for ( UcCnt= 0; UcCnt< 4; UcCnt++ )
		{
//TRACE("%02X %02X\n", (UINT8)(UlReadVal[ UcCnt ]  >>16 ), NVR2_Backup[UsNum + UcCnt] );	
			if ( (UINT8)(UlReadVal[ UcCnt ]  >>16 ) != NVR2_Backup[UsNum + UcCnt] ){	FlashReset();	return(-1); }
		}
	}
	// Set Flash RESET
	FlashReset();
		
	return ( 0 );
}

//********************************************************************************
// Function Name 	: Calibration_VerifyUpdate_PreRead
// Retun Value		: INT16 0:Ok, 1:CVER error, 5: WPB LOW ERROR
// Argment Value	: NON
// Explanation		: Calculation for MD5 of Flash data Function
// History			: First edition 						
//********************************************************************************
INT16 Calibration_VerifyUpdate_PreRead( void )
{
	UINT8 UcCnt;
	UINT16 UsNum;
	UINT32 UlReadVal[4];

	// Release RESET
	FlashResetRelease();		// Reset release
	IOWrite32A( FLASHROM_WPB , 1 );				// Disable write protect
	IOWrite32A( FLASHROM_SEL  , 7 );			// Disable write protect
	IOWrite32A( FLASHROM_ADR  , 0x00010100 );	// FLA_NVR=1, A[8]=1, A[7..0]=0x00
	IOWrite32A( FLASHROM_ACSCNT  ,  7 );		// for 7 times repeat
	IOWrite32A( FLASHROM_CMD  ,  7 );			// Auto configuration
	// Flash access timing Setting
	IOWrite32A( FLASHROM_TPGS, 118 );			// TPGS Flash spec.  min. 2.5usec max. 3.15uec
	IOWrite32A( FLASHROM_TPROG , 70 );			// TPROG Flash spec.  min. 6usec max. 7.5usec
	IOWrite32A( FLASHROM_TERASES , 92 );		// TERASES Flash spec.  Flash spec.  min. 4msec max. 5msec
//--------------------------------------------------------------------------------
// 0. Read All NVR ( Backup sequence )
//--------------------------------------------------------------------------------
	IOWrite32A( FLASHROM_ADR , 0x00010000 );	// Set NVR address
//	IOWrite32A( FLASHROM_SEL , 0x07 /*All*/ );	// Flash selector
	IOWrite32A( FLASHROM_ACSCNT , (256 -1) );	// for 256 byte read count
	IOWrite32A( FLASHROM_CMD , 1 );		// Read Start
	RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_RDAT ) ;
	for ( UsNum = 0; UsNum <= 0xFF; UsNum+=4 )
	{
		IORead4times32A( UlReadVal )  ;
		for (UcCnt= 0 ; UcCnt< 4 ; UcCnt++){
			NVR0_Backup[UsNum + UcCnt]   = (UINT8)UlReadVal[UcCnt];
			FLASH_SECTOR_BUFFER[UsNum + UcCnt]	     = (UINT8)(UlReadVal[UcCnt]>>8);
			NVR2_Backup[UsNum + UcCnt]   = (UINT8)(UlReadVal[UcCnt]>>16);
		}
//TRACE("%02X %02X%02X%02X\n", UsNum, NVR2_Backup[UsNum], WritePtr[UsNum], NVR0_Backup[UsNum] );	
	}
//--------------------------------------------------------------------------------
// 1. NVR Erase except NVR0
//--------------------------------------------------------------------------------
	// WP disable
	IOWrite32A( FLASHROM_WPB , 1 );							// Disable write protect
	WPBCtrl(WPB_OFF) ;										// Disable write protect
	if ( ReadWPB() != 1 ){ FlashReset(); return ( 5 );}		// WPB LOW ERROR

	IOWrite32A( FLASHROM_ADR , 0x00010000 );				// Set NVR address
	IOWrite32A( FLASHROM_SEL , 0x06 /*NVR1,2*/ );			// Flash selector
	IOWrite32A( FLASHROM_CMD , 4 );							// Sector Erase Start	
	/*5msec wait*/
	return( 0 );
}

//********************************************************************************
// Function Name 	: FlashWrite_NVRVerifyUpdate
// Retun Value		: INT16 0:Ok, 1:CVER error, 5: WPB LOW ERROR
// Argment Value	: NON
// Explanation		: Calculation for MD5 of Flash data Function
// History			: First edition 						
//********************************************************************************
INT16 Calibration_VerifyUpdate( void )
{
	UINT8 UcCnt;
	UINT16 UsNum;
	UINT8 UcNvrData[2];
	UINT32 UlReadVal[4];
    md5_context ctx;
	CRC_Reg = 0x0000ffff;
//--------------------------------------------------------------------------------
// 2. Update Calibration Code & Create Verify data
//--------------------------------------------------------------------------------
	// Update Verify Code	
	for ( UsNum = 0; UsNum <= 0xFF; UsNum++ )	
	{
		// LSB first 	
		UcNvrData[0] = NVR0_Backup[ UsNum ];	// NVR0
		UcNvrData[1] = FLASH_SECTOR_BUFFER[ UsNum ];		// NVR1	
		CRC16_main( UcNvrData, 2 );
	}
	NVR2_Backup[ 0x22 ] = (UINT8)(CRC_Reg>>8);
	NVR2_Backup[ 0x23 ] = (UINT8)CRC_Reg;
	md5_starts( &ctx );
	for ( UsNum = 0; UsNum <= 0xFF; UsNum++ )	
	{
		// MSB first 		
		UcNvrData[0] = FLASH_SECTOR_BUFFER[ UsNum ];		// NVR1	
		UcNvrData[1] = NVR0_Backup[ UsNum ];	// NVR0
		md5_update( &ctx, UcNvrData, 2);
	}
	md5_finish( &ctx, &(NVR2_Backup[ 0x10 ]) );
//--------------------------------------------------------------------------------
// 3. Write calibration & verify data to NVR
//--------------------------------------------------------------------------------
	// check whether sector erase function finished or not.
	RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_INT ) ;	
	for ( UsNum  = 0; UsNum  < 10; UsNum ++ )						// TimeOut 
	{
		RamRead32A(  CMD_IO_DAT_ACCESS, UlReadVal ) ;
		if( !(UlReadVal[0] ==  0x80) ){
			break;
		}
		WitTim( 2 );
	}
//	IOWrite32A( FLASHROM_SEL , 0x06 /*NVR1,2*/ );			// Flash selector
	IOWrite32A( FLASHROM_ACSCNT , 0 );						// should be set "0"
	IOWrite32A( FLASHROM_ADR , 0x00010000 );				// Set NVR address
	for ( UsNum = 0; UsNum <= 0x7F; UsNum++ )				// Max. 127times
	{
		IOWriteDouble32A( FLASHROM_WDAT, ((UINT32)(NVR2_Backup[UsNum])<<16)+((UINT32)(FLASH_SECTOR_BUFFER[UsNum])<<8),
						  FLASHROM_CMD,   2 );	
		/*20usec wait*/
	}
	IOWrite32A( FLASHROM_ADR , 0x00010000 + 0x80 );			// Set NVR address
	for ( UsNum = 0; UsNum <= 0x7F; UsNum++ )				// Max. 127times
	{
		IOWriteDouble32A( FLASHROM_WDAT, ((UINT32)(NVR2_Backup[UsNum+0x80])<<16)+((UINT32)(FLASH_SECTOR_BUFFER[UsNum+0x80])<<8),
					 	  FLASHROM_CMD , 2 );	
		/*20usec wait*/
	}
	IOWrite32A( FLASHROM_WPB, 0  );							// Enable write protect
	WPBCtrl(WPB_ON) ;										// Enable write protect
//--------------------------------------------------------------------------------
// 4. Read Verify 
//--------------------------------------------------------------------------------
//	IOWrite32A( FLASHROM_SEL , 0x06 /*NVR1,2*/ );			// Flash selector
	IOWrite32A( FLASHROM_ADR, 0x00010000  );				// Set NVR address
	IOWrite32A( FLASHROM_ACSCNT, (256 -1)  );				// for 256 byte read count
	IOWrite32A( FLASHROM_CMD , 1  );						// Read Start

	RamWrite32A( CMD_IO_ADR_ACCESS, FLASHROM_RDAT ) ;
	for ( UsNum = 0; UsNum <= 0xFF; UsNum+=4 )
	{
		IORead4times32A( UlReadVal )  ;
		for ( UcCnt= 0; UcCnt< 4; UcCnt++ )
		{
			if ( (UINT8)(UlReadVal[UcCnt]  >>8  ) != FLASH_SECTOR_BUFFER[UsNum + UcCnt] ){	FlashReset();	return(-1); }
			if ( (UINT8)(UlReadVal[UcCnt]  >>16 ) != NVR2_Backup[UsNum + UcCnt] )		 {	FlashReset();	return(-1); }
		}
	}
	// Set Flash RESET
	FlashReset();
		
//TRACE("Ans %d \n", ans );	
	return ( 0 );
}

