/********************************************************************
 FileName:		main.c
 Dependencies:	See INCLUDES section
 Processor:		PIC18, PIC24, and PIC32 USB Microcontrollers
 Hardware:		This demo is natively intended to be used on Microchip USB demo
 				boards supported by the MCHPFSUSB stack.  See release notes for
 				support matrix.  This demo can be modified for use on other hardware
 				platforms.
 Compiler:  	Microchip C18 (for PIC18), C30 (for PIC24), C32 (for PIC32)
 Company:		Microchip Technology, Inc.

 Software License Agreement:

 The software supplied herewith by Microchip Technology Incorporated
 (the ?Company?) for its PIC® Microcontroller is intended and
 supplied to you, the Company?s customer, for use solely and
 exclusively on Microchip PIC Microcontroller products. The
 software is owned by the Company and/or its supplier, and is
 protected under applicable copyright laws. All rights are reserved.
 Any use in violation of the foregoing restrictions may subject the
 user to criminal sanctions under applicable laws, as well as to
 civil liability for the breach of the terms and conditions of this
 license.


********************************************************************
 File Description:

 Change History:
  Rev   Description
  ----  -----------------------------------------
  2.6a  Added support for PIC24FJ256GB210

  2.7   No change

  2.8   Improvements to USBCBSendResume(), to make it easier to use.
  2.9   Added event transfer terminated handler code, for improved MSD
        error case handling.  Increased RAM reserved for the C software 
        stack on PIC18 devices.  The previous version did not allocate
        enough RAM for the worst case scenario, when USB_INTERRUPTS mode
        was used, causing potentially unexpected operation.

 GD 6.11.2023
	Generatore di barre (video composito, vecchio stile)
			
********************************************************************/

/** INCLUDES *******************************************************/

#ifndef __XC16__
#include <timer.h>
#include <ports.h>
#include <outcompare.h>
#include <i2c.h>
#include <ppsnew.h>
#else
// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
#include <xc.h>
#include <ppsnew.h>
#endif
#include "GenericTypedefs.h"
#include "swi2c.h"
//#include "wdt.h"
//#include "PwrMgnt.h"
#include <string.h>

#include "HardwareProfile.h"

#include <libpic30.h>

#include "GeneratoreBarre.h"
#include "uart.h"


/** CONFIGURATION **************************************************/

#if defined(__PIC24EP256GP202__) || defined(__PIC24EP512GP202__)				// 

#ifdef __XC16__
// PIC24EP256GP202 Configuration Bit Settings

// 'C' source line config statements

// FICD
#pragma config ICS = PGD3               // ICD Communication Channel Select bits (Communicate on PGEC3 and PGED3)
#pragma config JTAGEN = OFF             // JTAG Enable bit (JTAG is disabled)

// FPOR
#pragma config ALTI2C1 = OFF            // Alternate I2C1 pins (I2C1 mapped to SDA1/SCL1 pins)
#pragma config ALTI2C2 = OFF            // Alternate I2C2 pins (I2C2 mapped to SDA2/SCL2 pins)
#pragma config WDTWIN = WIN25           // Watchdog Window Select bits (WDT Window is 25% of WDT period)

// FWDT
#pragma config WDTPOST = PS4096         // Watchdog Timer Postscaler bits (1:4,096)
#pragma config WDTPRE = PR32            // Watchdog Timer Prescaler bit (1:32)
#pragma config PLLKEN = ON              // PLL Lock Enable bit (Clock switch to PLL source will wait until the PLL lock signal is valid.)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable bit (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = ON              // Watchdog Timer Enable bit (Watchdog timer always enabled)

// FOSC
#pragma config POSCMD = NONE            // Primary Oscillator Mode Select bits (Primary Oscillator disabled)
#pragma config OSCIOFNC = OFF           // OSC2 Pin Function bit (OSC2 is clock output)
#pragma config IOL1WAY = ON             // Peripheral pin select configuration (Allow only one reconfiguration)
#pragma config FCKSM = CSDCMD           // Clock Switching Mode bits (Both Clock switching and Fail-safe Clock Monitor are disabled)

// FOSCSEL
#pragma config FNOSC = FRCPLL          // Oscillator Source Selection (Internal Fast RC (FRC) Oscillator with postscaler)
#pragma config IESO = ON                // Two-speed Oscillator Start-up Enable bit (Start up device with FRC, then switch to user-selected oscillator source)

// FGS
#pragma config GWRP = OFF               // General Segment Write-Protect bit (General Segment may be written)
#pragma config GCP = OFF                // General Segment Code-Protect bit (General Segment Code protect is Disabled)

#else

_FGS( GWRP_OFF & GCP_OFF ) 
_FOSCSEL(FNOSC_FRCPLL & IESO_OFF)			//FNOSC_FRCPLL 
_FOSC( POSCMD_NONE & OSCIOFNC_OFF & IOL1WAY_OFF & FCKSM_CSDCMD )
_FWDT( WDTPOST_PS4096 & WDTPRE_PR32 & PLLKEN_ON & WINDIS_OFF & FWDTEN_ON )
_FPOR(/* FPWRT_PWR8 & */ /*BOREN_ON & */ ALTI2C1_OFF & ALTI2C2_OFF )
_FICD( ICS_PGD3 & JTAGEN_OFF )		// 
//_FAS( AWRP_OFF & APL_OFF & APLK_OFF)
//_FUID0(x) simpa..
#endif
#endif

/** VARIABLES ******************************************************/


#if SCREENSIZE_X==640
static const char CopyrString[]= {'T','e','r','m','i','n','a','l','e',' ','t','e','s','t','o',' ','8','0','x','3','0',' ',
#else
static const char CopyrString[]= {'T','e','r','m','i','n','a','l','e',' ','t','e','s','t','o',' ','4','0','x','3','0',' ',
#endif
	'v',VERNUMH+'0','.',VERNUML/10+'0',(VERNUML % 10)+'0', ' ','2','5','/','1','1','/','2','3', 0 };

#warning RICONTROLLARE EEprom emulata! non andava ago 21 (forse era errato il check) e modificata libreria flashoperations


WORD videoRAM[VIDEO_BUFSIZE/2];
BYTE cursor_x,cursor_y,cursor_mode;

volatile WORD tick10=0;
volatile BYTE second_10=0;
/*#ifdef USA_SW_RTC 
volatile PIC24_RTCC_DATE currentDate={{0,0,0}};
volatile PIC24_RTCC_TIME currentTime={{0,0,0}};
#else
PIC24_RTCC_DATE currentDate;
PIC24_RTCC_TIME currentTime;
#endif
*/
volatile DWORD milliseconds;
#define TIME_TO_LONG_CLICK 10

struct SAVED_PARAMETERS configParms;


#define TickGet() (tick10)

/** PRIVATE PROTOTYPES *********************************************/
static void InitializeSystem(void);
int UserInit(void);



/** DECLARATIONS ***************************************************/

/********************************************************************
 * Function:        void main(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Main program entry point.
 *
 * Note:            None
 *******************************************************************/

int main(void) {   
	WORD i;

mLED_1=0;




#ifndef __DEBUG
	__delay_ms(50);		// vale *20~ non essendoci ancora PLL!
#endif
	ClrWdt();
  InitializeSystem();

#ifndef __DEBUG
	__delay_ms(100);
#endif
	ClrWdt();

//mLED_2=1;
mLED_1=1;


  /*  TRISB=0;
    while(1) {
      LATB ^= 0xffff;
      __delay32(100); //2.2us
//          __delay_us(2);
    }*/



#if defined (__dsPIC33EP256MU806__) 	
  DataEEInit();
	ClrWdt();
  dataEEFlags.val = 0;
  Nop();
  Nop();
#endif

//	loadSettings();
	resetSettings();


	UserInit();
//mLED_3=1;


screenCLS();
Beep();


  while(1) {

		ClrWdt();



		UserTasks();
	
	  } //end while
    
  
	} //end main

/********************************************************************
 * Function:        static void InitializeSystem(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        InitializeSystem is a centralize initialization
 *                  routine. All required USB initialization routines
 *                  are called from here.
 *
 *                  User application initialization routine should
 *                  also be called from here.                  
 *
 * Note:            None
 *******************************************************************/
unsigned int getBRGFromBaudRate(unsigned long b) {
	unsigned long CLOSEST_UBRG_VALUE,BAUD_ACTUAL;
	unsigned int BAUD_ERROR,BAUD_ERROR_PRECENT;

	CLOSEST_UBRG_VALUE=((GetPeripheralClock()+8ul*b)/  4 /b-1);
//	CLOSEST_UBRG_VALUE=((GetPeripheralClock()+8ul*b)/(4)/b-1) /2;		// :2 !! NON SI CAPISCE MA E' COSì! DIVERSO DA GB106 ECC
	BAUD_ACTUAL=(GetPeripheralClock()/  4 /(CLOSEST_UBRG_VALUE+1));

	BAUD_ERROR=((BAUD_ACTUAL > b) ? BAUD_ACTUAL-b : b-BAUD_ACTUAL);
	BAUD_ERROR_PRECENT=((BAUD_ERROR*100+b/2)/b);
	if(BAUD_ERROR_PRECENT > 3) {
		//UART frequency error is worse than 3%
		}
	else if(BAUD_ERROR_PRECENT > 2) {
		//UART frequency error is worse than 2%
		}

	return (WORD)CLOSEST_UBRG_VALUE;
	}
  
static void InitializeSystem(void) {
  int i;

	ClrWdt();

#if defined (__dsPIC33EP256MU806__) 
#else
	LED0_TRIS=0; 
//	BUTTON0_TRIS=1; BUTTON1_TRIS=1; BUTTON2_TRIS=1; per debug no..
#endif
		// 
#if defined (__dsPIC33EP256MU806__) 
	LATB = 0b0000000000000000;
	LATC = 0b0000000000000000;
	LATD = 0b0000000000000000;
	LATE = 0b0000000000000000;
	LATF = 0b0000000000000000;
	LATG = 0b0000000000000000;
	TRISB= 0b1000000000000000;		// sw1; ana
	TRISC= 0b0000000000000000;		// 
	TRISD= 0b0000000000000000;		// led
	TRISE= 0b0000000000000000;		// 
	TRISF= 0b0000000000001000;		// USBID; 2 led
	TRISG= 0b0000000010000000;		// DTR (boot enable)
	ANSELB=0x0000;
	ANSELC=0x0000;
	ANSELD=0x0000;
	ANSELE=0x0000;
	ANSELG=0x0000;
	TRISFbits.TRISF0=0;		//buzzer
#else
	LATA = 0b0000000000000000;
	LATB = 0b0000000000000000;
	TRISA= 0b0000000000000000;		// 
	TRISB= 0b0000000010000000;		// 2 uscite; 1 led; 1 pulsante; buzzer 
	ANSELA=0x0000;
	ANSELB=0x0000;
#endif

	mInitAllLEDs();
	mInitAllSwitches();


//	_PLLDIV=278 /*205*/;						// M = _PLLDIV+2, 2..513
//	_PLLPRE=6;						// N1 = _PLLPRE+2, 2..33
//	_PLLPOST=0;						// N2 = _PLLPOST, 2 (0), 4 (1), 8 (3)
#if defined (__dsPIC33EP256MU806__) 
//  OSCTUNbits.TUN=21;			//(per andare a 8Mhz per USB!) (non quadrava secondo i calcoli, doveva essere 28 per avere circa il 10% in +...)
  OSCTUNbits.TUN=23;			//(per andare a 8Mhz per USB!) (7.37+(23*.375)%)
//#warning OCCHIO qua è 7.37 non 8!

	PLLFBD = 68; // M = PLLFBD + 2 = 70
  CLKDIVbits.PLLPOST = 0; // N2 = 2
  CLKDIVbits.PLLPRE = 0; // N1 = 2
#warning overcloccare?? 
#else
/*	
  Fosc= Fin*M/(N1*N2)
	0.8MHz<(Fin/N1)<8MHz
	100MHz<(Fin*M/N1)<340MHz
	M=2,3,4,...513
	N1=2...33
	N2=2,4,8
	
	PLLFBD    = M -2;
	CLKDIVbits.PLLPRE = N1 -2;
	CLKDIVbits.PLLPOST = 0b00; //N2={2,4,R,8} */

	CLKDIVbits.FRCDIV = 0b000;		// Set 32MHz FRC postscalar pll 
  
//	_PLLDIV=302;						// M = _PLLDIV+2, 2..513
//	_PLLPRE=6;						// N1 = _PLLPRE+2, 2..33
//	_PLLPOST=0;						// N2 = _PLLPOST, 2 (0), 4 (1), 8 (3)
//  OSCTUNbits.TUN=27;			//142MHz per Chroma
  CLKDIVbits.PLLPRE = 0; // N1 = 2		// in quest'ordine, dice http://www.microchip.com/forums/FindPost/1011737 (ma non è vero...))
  CLKDIVbits.PLLPOST = 0; // N2 = 2  
//#warning overclock!
#ifdef USA_DMA_8BIT
#if SCREENSIZE_X==640
  PLLFBD = 47; // M = PLLFBD + 2 = 54 => 100MHz ossia 50 che diviso 8 per SPI fa ~160nS per pixel
#else
  PLLFBD = 54; // M = PLLFBD + 2 = 54 => 100MHz ossia 50 che diviso 8 per SPI fa ~160nS per pixel
#endif
#else
  PLLFBD = 53;
#endif
  // v. sotto settaggio DMA e SPI 
  // a seconda dei valori si crea uno "sfrigolio" orizzontale... così pare ok
  
#endif


//OSCCONbits.CLKLOCK=1;OSCCONbits.NOSC=1;OSCCONbits.OSWEN=1;
//  while (OSCCONbits.COSC != 0x7)LATB ^= 1;; 


	//Wait for the Primary PLL to lock and then
#ifndef __DEBUG
  while(OSCCONbits.LOCK != 1) 
		ClrWdt();			// boh?
#endif


/*  	TRISB=0;
	while(1) {
		LATB ^= 0xffff;
		ClrWdt();
		}*/


	
    //********* Initialize Peripheral Pin Select (PPS) *************************
    //  This section only pertains to devices that have the PPS capabilities.
    //    When migrating code into an application, please verify that the PPS
    //    setting is correct for the port pins that are used in the application.
#if defined(__PIC24EP512GU810__) || defined (__dsPIC33EP256MU806__) 
// non le trova... manco in XC... froci urfidi...@$%&
// http://www.microchip.com/forums/m801504.aspx  MBedder !
#define PPSIn(fn,pin)    iPPSInput(IN_FN_PPS##fn,IN_PIN_PPS##pin)
#define PPSOut(fn,pin)    iPPSOutput(OUT_PIN_PPS##pin,OUT_FN_PPS##fn)


	PPSUnLock;

#if defined(__dsPIC33EP256MU806__) // pcb 2020 forgetIvreaAna
	PPSOut(_OC2, _RP96);      // buzzer 4KHz 
	PPSOut(_OC1, _RP87);      // output (pwm cmq)
//	PPSOut(_U1TX, _RP65);      // TXD IO1
//	PPSIn(_U1RX, _RP64);      // RXD IO0
#else
	PPSOut(_OC2, _RP127);      // buzzer 4KHz , qua è rimappabile

	PPSOut(_OC1, _RP80);      // output (pwm cmq)
//	PPSOut(_U1TX, _RP126);      // TXD 
//	PPSIn(_U1RX, _RP125);      // RXD 
#endif

	PPSLock;

#else

	PPSUnLock;

	PPSOut(_OC4, _RP43);      // output chroma (pwm cmq)
	PPSOut(_OC2, _RP38);      // output (pwm cmq)
//  PPSOut(_SDO1, _RP40);      // è fisso qua, qua! :D

	PPSOut(_U1TX, _RP37);      // TXD
	PPSIn(_U1RX, _RP20);      // RXD
  
	PPSLock;
	
#endif



skippa:



// non c'è...	EnablePullUpCN20;			//pulsanti
#if defined(__dsPIC33EP256MU806__) 	
	CNPUBbits.CNPUB15=1;
	CNPUDbits.CNPUD0=1;		// bah v. switches
	CNPUDbits.CNPUD9=1;		// pullup I2C.. mancano sul pcb...
	CNPUDbits.CNPUD10=1;		// 
#else
	CNPUBbits.CNPUB7=1;
#endif


#ifdef __XC16
  T2CON = 0;
  T2CONbits.TCS = 0;                    // clock from peripheral clock
  T2CONbits.TCKPS = 0b11;                  // 1:256 prescale
  PR2 = TMR2BASE;                          // rollover every n clocks; 
  T2CONbits.TON = 1;                    // start timer 
  IEC0bits.T2IE=1;
  IPC1bits.T2IP=3;

#ifndef USA_DMA
  T3CON = 0;
  T3CONbits.TCS = 0;                    // clock from peripheral clock
  T3CONbits.TCKPS = 0b00;                  // 1:1 prescale
  PR3 = TMR3BASE;                          // rollover every n clocks; 
  T3CONbits.TON = 1;                    // start timer 
  IEC0bits.T3IE=1;
  IPC2bits.T3IP=5;
  
	SPI1CON1=0b0001011100110110;    // disable SCK OCCHIO IRQ dsPIC; SMP=H; CLKPOL=H; 16bit; 1:4 & 1:3 (70:12=>~5.83MHz=>~160nS)
	SPI1CON2=0b0000000000000001;    // FIFO enable
	SPI1STAT=0b1000000000011000;    // [non 110! [100=Interrupt when one data is shifted into the SPIxSR, and as a result, the TX FIFO has one open memory location]
  IPC2bits.SPI1EIP=5;
#endif
  
#else
	OpenTimer2(T2_ON & T2_IDLE_CON & T2_GATE_OFF & T2_32BIT_MODE_OFF & T2_PS_1_64 & T2_SOURCE_INT,
		TMR2BASE);		//10Hz per timing
	ConfigIntTimer2(T2_INT_PRIOR_4 & T2_INT_ON);
	EnableIntT2;
  
	OpenTimer3(T6_ON & T3_IDLE_CON & T3_GATE_OFF & T3_32BIT_MODE_OFF & T3_PS_1_1 & T3_SOURCE_INT,
		TMR3BASE);		//20uS per HSync
	ConfigIntTimer3(T3_INT_PRIOR_5 & T3_INT_ON);		//STESSA di SPI! per shadow
	EnableIntT3;
  
	OpenSPI1(ENABLE_SCK_PIN /*QUA serve se no non ho gli IRQ SPI! */ & ENABLE_SDO_PIN & SPI_MODE16_ON & 
		SPI_SMP_ON & SPI_CKE_ON & SLAVE_ENABLE_OFF & CLK_POL_ACTIVE_LOW & MASTER_ENABLE_ON &
		SEC_PRESCAL_4_1 & PRI_PRESCAL_1_1,		// 16MHz circa, 14.7.15; in teoria si può andare a 21-22 con prescaler=3 ...
		FRAME_ENABLE_OFF & FRAME_SYNC_OUTPUT & FRAME_POL_ACTIVE_LOW & FRAME_SYNC_EDGE_PRECEDE & FIFO_BUFFER_ENABLE /* mettere ENABLE*/,
		SPI_ENABLE & SPI_IDLE_CON & SPI_RX_OVFLOW_CLR & BUF_ELE_COUNT_0 & BUF_INT_SEL_6 & SPI_SHFTREG_EMPTY & RX_FIFO_EMPTY
		);
//#define  BUF_INT_SEL_6           0xfffb /* Interrupt when last bit is shifted into SPIxSR and 
//                                           as a result TX FIFO is empty */
//#define  BUF_INT_SEL_5           0xfff7 /* Interrupt when the last bit is shifted out of SPIxSR
//                                           and the transmit is complete */
	SetPriorityIntSPI1(5);		// STESSA del timer! per shadow

#endif

#ifdef USA_DMA
  SPI1STATbits.SPIEN=0;
	IFS0bits.DMA0IF = 0;	 // Clear the interrupt flag!

  
  DMA0CON = 0;   // channel off
#ifdef USA_DMA_8BIT
//  finire, è ancora sbagliato.. ma tanto cmq non serve a molto
  DMA0CON = 0b0110000000000001;   // channel off, 8bit, RAM to periph, IRQ=full, register-postinc, one shot [continous]
#else
//  widthDMA=SCREENSIZE_X/16 +HORIZ_PORCH_COMP;
  DMA0CON = 0b0010000000000001;   // channel off, 16bit, RAM to periph, IRQ=full, register-postinc, one shot [continous]
#endif
  DMA0STAL = &videoRAM;  // transfer source physical address
  DMA0STAH = 0;
  DMA0CNT = HORIZ_SYNC_COMP;     // source size
  DMA0PAD = &SPI1BUF;     // transfer destination physical address
  DMARQC = 0;                    // clear all errors
  DMA0REQbits.IRQSEL = 0b00001000;     // Timer 3

  IPC1bits.DMA0IP=5;            // set IPL 5, 
  IEC0bits.DMA0IE=1;

  T3CON = 0;
  T3CONbits.TCS = 0;                    // clock from peripheral clock
  T3CONbits.TCKPS = 0b00;               // 1:1 prescale
  PR3 = TMR3BASE;                       // rollover every n clocks; 
  IEC0bits.T3IE=0;
  T3CONbits.TON = 1;                    // start timer to generate triggers
  
// bisogna fare attenzione: se DMA è più veloce di quanto escono i bit su SPI, si sovrappongono! v. sopra overclock (perché non c'è un divisore preciso qua)
  
#ifdef USA_DMA_8BIT
#if SCREENSIZE_X==640
	SPI1CON1=0b0001001100110111;    // disable SCK; SMP=H; CLKPOL=H; 8bit; 1:1 & 1:3 (50:3=>~12MHz=>~80nS)
#else
	SPI1CON1=0b0001001100100111;    // disable SCK; SMP=H; CLKPOL=H; 8bit; 1:1 & 1:7 (50:7=>~5.83MHz=>~160nS)
#endif
#else
	SPI1CON1=0b0001011100100111;    // disable SCK; SMP=H; CLKPOL=H; 16bit; 1:1 & 1:7 (50:7=>~6.25MHz=>~160nS)
#endif
  
  // [è un pelo troppo lento, con DMA... e 1:8 è troppo veloce e lascia un gap ogni 16bit...]
  // v. TNRBASE: c'è un bit-SPI di attesa dopo ogni transazione, inevitabile, per cui bisogna accelerare qua
//	SPI1CON1=0b0001011100100011;    // disable SCK; SMP=H; CLKPOL=H; 16bit; 1:1 & 1:8 (70:6=>~5.83MHz=>~160nS)
//	SPI1CON1=0b0001011100111010;    // disable SCK; SMP=H; CLKPOL=H; 16bit; 1:1 & 1:8 (70:6=>~5.83MHz=>~160nS)
	SPI1CON2=0b0000000000000000;    // 
	SPI1STAT=0b1000000000000000;    // 
   
  //https://forum.mikroe.com/viewtopic.php?t=44659 per SPI DMA
  
  
  DMA0CONbits.CHEN = 1; // turn on DMA channel 0
  
  MSTRPR=0x0020;      // priorità a DMA  (migliora di poco)
#endif


	ClrWdt();
	



  CM3CON=0;		// comparatori off ?? boh


// buzzer 4KHz
  /* Reset PWM */
  OC2CON1 = 0x0000;
  OC2CON2 = 0x0000;
  
  /* set PWM duty cycle to 50% */
  OC2R    = FCY/BEEP_STD_FREQ/2  /* basato su SysClock => 4KHz circa ecc (16MHz / 256) */; //PWM_PERIOD >> 1; /* set the duty cycle tp 50% */
  OC2RS   = FCY/BEEP_STD_FREQ  /* se uso Timer come Src SEMBRA NON FARE NULLA... qua boh! */;  //PWM_PERIOD - 1;  /* set the period */
  
  /* configure PWM */
  OC2CON2 = 0x001f;   /* 0x001F = Sync with This OC module                               */
  OC2CON1 = 0x1c08 /* 0x0400 => src=Timer3 */;  /* 0x1C08 = Clock source Fcyc, trigger mode 1, Mode 0 (disable OC1) */
  
  /* enable the PWM */
  OC2CON1 |= 0x0006;   /* Mode 6, Edge-aligned PWM Mode */ //  v. beep


// USARE per chroma!!!
  OC4CON1 = 0x0000;
  OC4CON2 = 0x0000;
  
  OC4R    = FCY/4433689L/2  /* basato su SysClock => 4KHz circa ecc (16MHz / 256) */; //PWM_PERIOD >> 1; /* set the duty cycle tp 50% */
  OC4RS   = FCY/4433689L  /* se uso Timer come Src SEMBRA NON FARE NULLA... qua boh! */;  //PWM_PERIOD - 1;  /* set the period */
// dovremmo avere il clock a 142MHz... sintonizzare, tarare...
  
  OC4CON2 = 0x001f;   /* 0x001F = Sync with This OC module                               */
  OC4CON1 = 0x1c00;   /* 0x1C08 = Clock source Fcyc, trigger mode 1, Mode 0 (disable OC4) */


 	i=getBRGFromBaudRate(9600L);
  OpenUART1(UART_EN | UART_IDLE_CON | UART_MODE_SIMPLEX | UART_UEN_00 | UART_DIS_WAKE |
          UART_DIS_LOOPBACK | UART_DIS_ABAUD | UART_UXRX_IDLE_ONE | UART_BRGH_FOUR |
          UART_NO_PAR_8BIT | UART_1STOPBIT,
          UART_INT_TX_EACH_CHAR | UART_SYNC_BREAK_DISABLED | UART_TX_ENABLE |
          UART_INT_RX_CHAR | UART_ADR_DETECT_DIS, i);
  ConfigIntUART1(UART_RX_INT_EN | UART_TX_INT_DIS);
  SetPriorityIntU1RX(UART_RX_INT_PR2);
  EnableIntU1RX;

	} //end InitializeSystem


int UserInit(void) {

	ClrWdt();

//	currentDate.mday=1;		//preset cmq
//	currentDate.mon=1;
  
  OC4CON1 |= 0x0006;
  
  cursor_mode=0x80;


	return 1;
	}

void plotInit(BYTE m) {
  int i;
  
  screenCLS();
  
  if(!m) {

    drawPixel(0, 0, 1);
    drawPixel(240, 40, 1);
    writeStringAt(1, 24, "Riga 3", 1);
    writeStringAt(8, 40, "Riga 5", 1);
    writeStringAt(16, 48, "Riga 6", 1);
    writeStringAt(160, 80, "Riga 10", 1);
    setCursor(0,16);
  writeString(CopyrString);

    drawLine(10,110, 250,200, 1) ;
    drawRectangle(30,30,180,100,1);
    drawBar(230,80,250,120,1);
    drawCircle(40,200,30,1);
    }
  else {             // monoscopio!
    drawRectangle(0,0,SCREENSIZE_X-1,SCREENSIZE_Y-1,1);
    for(i=0; i<SCREENSIZE_X; i+=SCREENSIZE_X/12)
      drawLine(i,0,i,SCREENSIZE_Y-1,1);
    for(i=0; i<SCREENSIZE_Y; i+=SCREENSIZE_Y/9)
      drawLine(0,i,SCREENSIZE_X-1,i,1);
    
    drawCircle(SCREENSIZE_X/2,SCREENSIZE_Y/2,(SCREENSIZE_Y-2)/2,1);
    
    drawCircleFilled(SCREENSIZE_X/2,SCREENSIZE_Y/2,(SCREENSIZE_Y-2)/2/2,0);
    drawCircle(SCREENSIZE_X/2,SCREENSIZE_Y/2,(SCREENSIZE_Y-2)/2/2,1);
    drawLine(SCREENSIZE_X/2-(SCREENSIZE_Y-2)/2,SCREENSIZE_Y/2,
            SCREENSIZE_X/2+(SCREENSIZE_Y-2)/2,SCREENSIZE_Y/2,1);
    drawLine(SCREENSIZE_X/2,SCREENSIZE_Y/2-(SCREENSIZE_Y-2)/2,
            SCREENSIZE_X/2,SCREENSIZE_Y/2+(SCREENSIZE_Y-2)/2,1);
    drawCircleFilled(SCREENSIZE_X/2,SCREENSIZE_Y/2,(SCREENSIZE_Y-2)/10/2,1);
    
    drawLine(SCREENSIZE_X/4.5,SCREENSIZE_Y/4.5,SCREENSIZE_X/2.5,SCREENSIZE_Y/3,1);
    drawLine(SCREENSIZE_X-1-SCREENSIZE_X/2.5,SCREENSIZE_Y/3,
			SCREENSIZE_X-1-SCREENSIZE_X/4.5,SCREENSIZE_Y/4.5,1);
/*    drawLine(SCREENSIZE_X-1-SCREENSIZE_X/4.5,SCREENSIZE_Y-1-SCREENSIZE_Y/3,
            SCREENSIZE_X-1-SCREENSIZE_X/2.5,SCREENSIZE_Y-1-SCREENSIZE_Y/4.5,1);
    drawLine(SCREENSIZE_X-1-SCREENSIZE_X/2.5,SCREENSIZE_Y-1-SCREENSIZE_Y/3,
            SCREENSIZE_X-1-SCREENSIZE_X/4.5,SCREENSIZE_Y-1-SCREENSIZE_Y/4.5,1);*/
    
    drawCircleFilled((SCREENSIZE_X/9)*1,(SCREENSIZE_Y/7)*1,(SCREENSIZE_Y-2)/3.7/2,0);
    //spostare verso bordi o rimpicciolire
    drawCircle((SCREENSIZE_X/9)*1,(SCREENSIZE_Y/7)*1,(SCREENSIZE_Y-2)/3.7/2,1);
    drawLine((SCREENSIZE_X/9)*1-(SCREENSIZE_Y-2)/3.7/2,(SCREENSIZE_Y/7)*1,(SCREENSIZE_X/9)*1+(SCREENSIZE_Y-2)/3.7/2,(SCREENSIZE_Y/7)*1,1);
    drawLine((SCREENSIZE_X/9)*1,(SCREENSIZE_Y/7)*1-(SCREENSIZE_Y-2)/3.7/2,(SCREENSIZE_X/9)*1,(SCREENSIZE_Y/7)*1+(SCREENSIZE_Y-2)/3.7/2,1);
    drawCircleFilled((SCREENSIZE_X/9)*1,(SCREENSIZE_Y/7)*1,(SCREENSIZE_Y-2)/20/2,1);
    
    drawCircleFilled((SCREENSIZE_X/9)*8,(SCREENSIZE_Y/7)*1,(SCREENSIZE_Y-2)/3.7/2,0);
    drawCircle((SCREENSIZE_X/9)*8,(SCREENSIZE_Y/7)*1,(SCREENSIZE_Y-2)/3.7/2,1);
    drawLine((SCREENSIZE_X/9)*8-(SCREENSIZE_Y-2)/3.7/2,(SCREENSIZE_Y/7)*1,(SCREENSIZE_X/9)*8+(SCREENSIZE_Y-2)/3.7/2,(SCREENSIZE_Y/7)*1,1);
    drawLine((SCREENSIZE_X/9)*8,(SCREENSIZE_Y/7)*1-(SCREENSIZE_Y-2)/3.7/2,(SCREENSIZE_X/9)*8,(SCREENSIZE_Y/7)*1+(SCREENSIZE_Y-2)/3.7/2,1);
    drawCircleFilled((SCREENSIZE_X/9)*8,(SCREENSIZE_Y/7)*1,(SCREENSIZE_Y-2)/20/2,1);
    
    drawCircleFilled((SCREENSIZE_X/9)*1,(SCREENSIZE_Y/7)*6,(SCREENSIZE_Y-2)/3.7/2,0);
    drawCircle((SCREENSIZE_X/9)*1,(SCREENSIZE_Y/7)*6,(SCREENSIZE_Y-2)/3.7/2,1);
    drawLine((SCREENSIZE_X/9)*1-(SCREENSIZE_Y-2)/3.7/2,(SCREENSIZE_Y/7)*6,(SCREENSIZE_X/9)*1+(SCREENSIZE_Y-2)/3.7/2,(SCREENSIZE_Y/7)*6,1);
    drawLine((SCREENSIZE_X/9)*1,(SCREENSIZE_Y/7)*6-(SCREENSIZE_Y-2)/3.7/2,(SCREENSIZE_X/9)*1,(SCREENSIZE_Y/7)*6+(SCREENSIZE_Y-2)/3.7/2,1);
    drawCircleFilled((SCREENSIZE_X/9)*1,(SCREENSIZE_Y/7)*6,(SCREENSIZE_Y-2)/20/2,1);
    
    drawCircleFilled((SCREENSIZE_X/9)*8,(SCREENSIZE_Y/7)*6,(SCREENSIZE_Y-2)/3.7/2,0);
    drawCircle((SCREENSIZE_X/9)*8,(SCREENSIZE_Y/7)*6,(SCREENSIZE_Y-2)/3.7/2,1);
    drawLine((SCREENSIZE_X/9)*8-(SCREENSIZE_Y-2)/3.7/2,(SCREENSIZE_Y/7)*6,(SCREENSIZE_X/9)*8+(SCREENSIZE_Y-2)/3.7/2,(SCREENSIZE_Y/7)*6,1);
    drawLine((SCREENSIZE_X/9)*8,(SCREENSIZE_Y/7)*6-(SCREENSIZE_Y-2)/3.7/2,(SCREENSIZE_X/9)*8,(SCREENSIZE_Y/7)*6+(SCREENSIZE_Y-2)/3.7/2,1);
    drawCircleFilled((SCREENSIZE_X/9)*8,(SCREENSIZE_Y/7)*6,(SCREENSIZE_Y-2)/20/2,1);
    
    for(i=0; i<8; i++) {
      drawLine(SCREENSIZE_X/4.6,3+i*2,SCREENSIZE_X/4.6+SCREENSIZE_X/20,3+i*2,1);
      drawLine(SCREENSIZE_X*3/4.2,3+i*2,SCREENSIZE_X*3/4.2+SCREENSIZE_X/20,3+i*2,1);
      drawLine(SCREENSIZE_X/5,SCREENSIZE_Y/2-10+i*2,SCREENSIZE_X/5+SCREENSIZE_X/20,SCREENSIZE_Y/2-10+i*2,1);
      drawLine(SCREENSIZE_X*3/3.8,SCREENSIZE_Y/2-10+i*2,SCREENSIZE_X*3/3.8+SCREENSIZE_X/20,SCREENSIZE_Y/2-10+i*2,1);
      drawLine(SCREENSIZE_X/4.6,SCREENSIZE_Y-(3+i*2),SCREENSIZE_X/4.6+SCREENSIZE_X/20,SCREENSIZE_Y-(3+i*2),1);
      drawLine(SCREENSIZE_X*3/4.2,SCREENSIZE_Y-(3+i*2),SCREENSIZE_X*3/4.2+SCREENSIZE_X/20,SCREENSIZE_Y-(3+i*2),1);
      }
    drawBar(1,SCREENSIZE_Y/2-SCREENSIZE_Y/9/2,SCREENSIZE_X/12,SCREENSIZE_Y/2+SCREENSIZE_Y/2/9,1);
    drawBar(SCREENSIZE_X-SCREENSIZE_X/12,SCREENSIZE_Y/2-SCREENSIZE_Y/9/2,
            SCREENSIZE_X-1,SCREENSIZE_Y/2+SCREENSIZE_Y/2/9,1);
    
    for(i=0; i<9; i++) {
      drawLine(SCREENSIZE_X/2-(pow(i+1,2)),SCREENSIZE_Y-(5+i*4)-1,
              SCREENSIZE_X/2+(pow(i+1,2)),SCREENSIZE_Y-(5+i*4)-1,1);
      drawLine(SCREENSIZE_X/2-(pow(i+1,2)),SCREENSIZE_Y-(5+i*4),
              SCREENSIZE_X/2+(pow(i+1,2)),SCREENSIZE_Y-(5+i*4),1);
      }
    
    drawBar(SCREENSIZE_X/3,SCREENSIZE_Y*7/9-4,SCREENSIZE_X*2/3,SCREENSIZE_Y*7/9+4,0);
    drawRectangle(SCREENSIZE_X/3,SCREENSIZE_Y*7/9-4,SCREENSIZE_X*2/3,SCREENSIZE_Y*7/9+4,1);
    for(i=1; i<8; i++)
      drawLine(SCREENSIZE_X/3+i*(SCREENSIZE_X/3/8),SCREENSIZE_Y*7/9-4,
              SCREENSIZE_X/3+i*(SCREENSIZE_X/3/8),SCREENSIZE_Y*7/9+4,1);
    
    writeStringAt(SCREENSIZE_X/2-21, SCREENSIZE_Y/9+2, "G.Dar", 1);
    }
  }
void UserTasks(void) {
	static BYTE oldSw=15,swPressed=0;
	static WORD cnt=0,cnt2=0;
  static BYTE inited=0,plotmode=0;

  if(!inited) {
    inited=1;
    plotInit(plotmode);
    }

	if(second_10) {
		second_10=0;

		cnt++;
		cnt2++;

    mLED_1_Toggle();    // [was: 1MHz 11/11/23]


		if(cursor_mode & 0x80) {
			if(!(TickGet() % 5)) {
				BYTE oldreverse=configParms.Reverse;
				cursor_mode ^= 1;
//				if(cursor_mode & 1)
        configParms.Reverse=cursor_mode & 1;
					writeCharAt(cursor_x*8, cursor_y*8, ' ', 1);
				configParms.Reverse=oldreverse;
				}
			}



    if(!sw1) {
      if(oldSw & 1) {
        
      
        }
			swPressed++;
      }
		else {
      if(!(oldSw & 1)) {
        if(swPressed>=TIME_TO_LONG_CLICK) {
          Beep();
          configParms.Reverse=!configParms.Reverse;
          plotInit(plotmode);
          }
        else {
          static BYTE aChar=' ';
          putcUART1(aChar++);
          aChar &= 127;
          plotmode=!plotmode;
          plotInit(plotmode);
          }
    		swPressed=0;
        }
      }
    
		if(cnt2>=8) {
			cnt2=0;
			}

		oldSw=(sw1 ? 1 : 0) ;

		}		// second_1



	}


void Beep(void) {

  OC2R    = FCY/BEEP_STD_FREQ/2;
  OC2RS   = FCY/BEEP_STD_FREQ;

  OC2CON1 |= 0x0006;   /* Mode 6, Edge-aligned PWM Mode */
#ifndef __DEBUG
	__delay_ms(100);
	ClrWdt();
	__delay_ms(100);
	ClrWdt();
	__delay_ms(100);
	ClrWdt();
#endif
  OC2CON1 &= ~0x0006; 
	}

void saveSettings(void) {

#if defined (__dsPIC33EP256MU806__) 	
	BYTE *p;
	BYTE i;

  p=(BYTE *)&configParms;
	for(i=0; i<sizeof(struct SAVED_PARAMETERS); i++) 
		EEscrivi_(i,*p++);
#else
	memcpy(I2CBuffer,(unsigned char*)&configParms, sizeof(struct SAVED_PARAMETERS));
	I2CWritePage16(0,sizeof(I2CBuffer));				// 
#endif
	}

void loadSettings(void) {
	BYTE *p;
	BYTE i;

#if defined (__dsPIC33EP256MU806__) 	

rifo:
	configParms.signature=MAKEWORD(EEleggi(offsetof(struct SAVED_PARAMETERS,signature)), EEleggi(offsetof(struct SAVED_PARAMETERS,signature)+1));
	if(configParms.signature != 0x4447) {				// 
		resetSettings();
		goto rifo;
		}
  p=(BYTE *)&configParms;
	for(i=0; i<sizeof(struct SAVED_PARAMETERS); i++) 
		*p++=EEleggi(i);

#else

rifo:
	configParms.signature=MAKEWORD(EEleggi(offsetof(struct SAVED_PARAMETERS,signature)), EEleggi(offsetof(struct SAVED_PARAMETERS,signature)+1));
	if(configParms.signature != 0x4447) {				// 
		resetSettings();
		goto rifo;
		}
  p=(BYTE *)&configParms;
	for(i=0; i<sizeof(struct SAVED_PARAMETERS); i++) 
		*p++=EEleggi(i);
#endif
	}

void resetSettings(void) {
	int i;
	BYTE *p,*p2;
	extern const struct SAVED_PARAMETERS DefaultParameters;

#if defined (__dsPIC33EP256MU806__) 	
	  p=&DefaultParameters;
		for(i=0; i<sizeof(struct SAVED_PARAMETERS); i++) {
  		EEscrivi_(i,*p++);
			} 
#else

	  p=&DefaultParameters;
		for(i=0; i<sizeof(struct SAVED_PARAMETERS); i++) {
  		EEscrivi_(i,*p++);
			} 

	configParms=DefaultParameters;
#warning PArameters manca EEprom

#endif

	for(i=0; i<3; i++) {
		mLED_1=1;
#ifndef __DEBUG
		__delay_ms(200);
#endif
		ClrWdt();
		mLED_1=0;
#ifndef __DEBUG
		__delay_ms(200);
#endif
		ClrWdt();
		}

	}


#if !defined(__PIC24EP256GP202__)	&& !defined(__PIC24EP512GP202__)		// 
/****************************************************************************
  Function:
    DWORD   PIC24RTCCGetDate( void )

  Description:
    This routine reads the date from the RTCC module.

  Precondition:
    The RTCC module has been initialized.


  Parameters:
    None

  Returns:
    DWORD in the format <xx><YY><MM><DD>

  Remarks:
    To catch roll-over, we do two reads.  If the readings match, we return
    that value.  If the two do not match, we read again until we get two
    matching values.

    For the PIC32MX, we use library routines, which return the date in the
    PIC32MX format.
  ***************************************************************************/

DWORD PIC24RTCCGetDate( void ) {
  DWORD_VAL   date1;
  DWORD_VAL   date2;

  do {
    while(RCFGCALbits.RTCSYNC);

    RCFGCALbits.RTCPTR0 = 1;
    RCFGCALbits.RTCPTR1 = 1;
    date1.w[1] = RTCVAL;
    date1.w[0] = RTCVAL;

    RCFGCALbits.RTCPTR0 = 1;
    RCFGCALbits.RTCPTR1 = 1;
    date2.w[1] = RTCVAL;
    date2.w[0] = RTCVAL;

    } while (date1.Val != date2.Val);

  return date1.Val;
	}

/****************************************************************************
  Function:
    DWORD   PIC24RTCCGetTime( void )

  Description:
    This routine reads the time from the RTCC module.

  Precondition:
    The RTCC module has been initialized.

  Parameters:
    None

  Returns:
    DWORD in the format <xx><HH><MM><SS>

  Remarks:
    To catch roll-over, we do two reads.  If the readings match, we return
    that value.  If the two do not match, we read again until we get two
    matching values.

    For the PIC32MX, we use library routines, which return the time in the
    PIC32MX format.
  ***************************************************************************/

DWORD PIC24RTCCGetTime( void ) {
  DWORD_VAL   time1;
  DWORD_VAL   time2;

  do {
    while(RCFGCALbits.RTCSYNC);

    RCFGCALbits.RTCPTR0 = 1;
    RCFGCALbits.RTCPTR1 = 0;
    time1.w[1] = RTCVAL;
    time1.w[0] = RTCVAL;

    RCFGCALbits.RTCPTR0 = 1;
    RCFGCALbits.RTCPTR1 = 0;
    time2.w[1] = RTCVAL;
    time2.w[0] = RTCVAL;

    } while (time1.Val != time2.Val);

    return time1.Val;
	}


/****************************************************************************
  Function:
    void PIC24RTCCSetDate( WORD xx_year, WORD month_day )

  Description:
    This routine sets the RTCC date to the specified value.


  Precondition:
    The RTCC module has been initialized.

  Parameters:
    WORD xx_year    - BCD year in the lower byte
    WORD month_day  - BCD month in the upper byte, BCD day in the lower byte

  Returns:
    None

  Remarks:
    For the PIC32MX, we use library routines.
  ***************************************************************************/

void PIC24RTCCSetDate( WORD xx_year, WORD month_day ) {
  
	UnlockRTCC();
  RCFGCALbits.RTCPTR0 = 1;
  RCFGCALbits.RTCPTR1 = 1;
  RTCVAL = xx_year;
  RTCVAL = month_day;
	}


/****************************************************************************
  Function:
    void PIC24RTCCSetTime( WORD weekDay_hours, WORD minutes_seconds )

  Description:
    This routine sets the RTCC time to the specified value.

  Precondition:
    The RTCC module has been initialized.

  Parameters:
    WORD weekDay_hours      - BCD weekday in the upper byte, BCD hours in the
                                lower byte
    WORD minutes_seconds    - BCD minutes in the upper byte, BCD seconds in
                                the lower byte

  Returns:
    None

  Remarks:
    For the PIC32MX, we use library routines.
  ***************************************************************************/

void PIC24RTCCSetTime( WORD weekDay_hours, WORD minutes_seconds) {

  UnlockRTCC();
  RCFGCALbits.RTCPTR0 = 1;
  RCFGCALbits.RTCPTR1 = 0;
  RTCVAL = weekDay_hours;
  RTCVAL = minutes_seconds;
	}

/****************************************************************************
  Function:
    void UnlockRTCC( void )

  Description:
    This function unlocks the RTCC so we can write a value to it.

  Precondition:
    None

  Parameters:
    None

  Return Values:
    None

  Remarks:
    For the PIC32MX, we use library routines.
  ***************************************************************************/

#define RTCC_INTERRUPT_REGISTER IEC3
#define RTCC_INTERRUPT_VALUE    0x2000

void UnlockRTCC( void ) {
  BOOL interruptsWereOn;

  interruptsWereOn = FALSE;
  if((RTCC_INTERRUPT_REGISTER & RTCC_INTERRUPT_VALUE) == RTCC_INTERRUPT_VALUE) {
    interruptsWereOn = TRUE;
    RTCC_INTERRUPT_REGISTER &= ~RTCC_INTERRUPT_VALUE;
    }

  // Unlock the RTCC module
  __asm__ ("mov #NVMKEY,W0");
  __asm__ ("mov #0x55,W1");
  __asm__ ("mov #0xAA,W2");
  __asm__ ("mov W1,[W0]");
  __asm__ ("nop");
  __asm__ ("mov W2,[W0]");
  __asm__ ("bset RCFGCAL,#13");
  __asm__ ("nop");
  __asm__ ("nop");

  if(interruptsWereOn) {
    RTCC_INTERRUPT_REGISTER |= RTCC_INTERRUPT_VALUE;
    }
	}
#endif

BYTE to_bcd(BYTE n) {
	
	return (n % 10) | ((n / 10) << 4);
	}

BYTE from_bcd(BYTE n) {
	
	return (n & 15) + (10*(n >> 4));
	}



const struct SAVED_PARAMETERS /*packed??*/ DefaultParameters =
	{ 
	0x4447, 	// signature
	16,
	RETICOLO,
	3,			// thickness  /*TMR4BASE*/;                           // 
	100,
	0
	};

#warning NON C'E' EEPROM!

void EEscrivi_(WORD addr,BYTE n) {	// 
  WORD i;
  
/*
  i=DataEERead(addr/2);
  if(addr & 1) {
    i &= 0x00ff;
    i |= ((int)n) << 8;
    }
  else {
    i &= 0xff00;
    i |= n;
    }
  DataEEWrite(i,addr/2);
*/
#warning [was: scrivere sempre su dispari per ultima...]
  }
BYTE EEleggi(WORD addr) {
/*
  WORD i=DataEERead(addr/2);
  
  if(addr & 1) 
    return HIBYTE(i);
  else
    return LOBYTE(i);
*/
  }

