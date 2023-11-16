/********************************************************************
 FileName:		main.c
 Dependencies:	See INCLUDES section
 Processor:		PIC18, PIC24, and PIC32 USB Microcontrollers
 Hardware:		This demo is natively intended to be used on Microchip USB demo
 				boards supported by the MCHPFSUSB stack.  See release notes for
 				support matrix.  This demo can be modified for use on other hardware
 				platforms.
 Complier:  	Microchip C18 (for PIC18), C30 (for PIC24), C32 (for PIC32)
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

#include "HardwareProfile.h"

#include <libpic30.h>

#include "GeneratoreBarre.h"


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


static const char CopyrString[]= {'G','e','n','e','r','a','t','o','r','e',' ','d','i',' ','b','a','r','r','e',' ',
	'v',VERNUMH+'0','.',VERNUML/10+'0',(VERNUML % 10)+'0', ' ','1','2','/','1','1','/','2','3', 0 };

#warning RICONTROLLARE EEprom emulata! non andava ago 21 (forse era errato il check) e modificata libreria flashoperations



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


struct SAVED_PARAMETERS configParms;


#define TickGet() (tick10)

/** PRIVATE PROTOTYPES *********************************************/
static void InitializeSystem(void);
int UserInit(void);


BYTE setFigura(void);


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


/*
    TRISF=0;TRISB=0;
    while(1) {
      LATF ^= 0xffff;
      ClrWdt();
    }*/

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


Beep();

	setFigura();



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
static void InitializeSystem(void) {

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
	OSCTUN=0;			//(da USB)
  CLKDIVbits.PLLPRE = 0; // N1 = 2		// in quest'ordine, dice http://www.microchip.com/forums/FindPost/1011737 (ma non è vero...))
  CLKDIVbits.PLLPOST = 0; // N2 = 2  
  PLLFBD = 74; // M = PLLFBD + 2 = 76
  
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

	PPSOut(_OC2, _RP38);      // output (pwm cmq)
	PPSOut(_OC1, _RP41);      // output (pwm cmq)

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
//	CNPUCbits.CNPUC4=1;
#endif


#ifdef __XC16
  T2CON = 0;
  T2CONbits.TCS = 0;                    // clock from peripheral clock
  T2CONbits.TCKPS = 0b11;                  // 1:64 prescale
  PR2 = TMR2BASE;                           // rollover every n clocks; 1587=44100campioni/sec, 11.8.17
  T2CONbits.TON = 1;                    // start timer to generate ADC triggers
  IEC0bits.T2IE=1;
  IPC1bits.T2IP=3;

#else
	OpenTimer2(T2_ON & T2_IDLE_CON & T2_GATE_OFF & T2_32BIT_MODE_OFF & T2_PS_1_64 & T2_SOURCE_INT,
		TMR2BASE);		//10Hz per timing
	ConfigIntTimer2(T2_INT_PRIOR_4 & T2_INT_ON);
	EnableIntT2;
#endif

	ClrWdt();
	



  CM3CON=0;		// comparatori off ?? boh


// buzzer 4KHz
  /* Reset PWM */
  OC2CON1 = 0x0000;
  OC2CON2 = 0x0000;
  
  /* set PWM duty cycle to 50% */
  OC2R    = FCY/2/BEEP_STD_FREQ/2  /* basato su SysClock => 4KHz circa ecc (16MHz / 256) */; //PWM_PERIOD >> 1; /* set the duty cycle tp 50% */
  OC2RS   = FCY/2/BEEP_STD_FREQ  /* se uso Timer come Src SEMBRA NON FARE NULLA... qua boh! */;  //PWM_PERIOD - 1;  /* set the period */
  
  /* configure PWM */
  OC2CON2 = 0x001f;   /* 0x001F = Sync with This OC module                               */
  OC2CON1 = 0x1c08 /* 0x0400 => src=Timer3 */;  /* 0x1C08 = Clock source Fcyc, trigger mode 1, Mode 0 (disable OC1) */
  
  /* enable the PWM */
  OC2CON1 |= 0x0006;   /* Mode 6, Edge-aligned PWM Mode */ //  v. beep



	} //end InitializeSystem


int UserInit(void) {

	ClrWdt();

//	currentDate.mday=1;		//preset cmq
//	currentDate.mon=1;


	ClrWdt();
	return 1;
	}


void UserTasks(void) {
	static BYTE oldSw=3;
	static WORD cnt=0,cnt2=0;


	if(second_10) {
		second_10=0;

		cnt++;
		cnt2++;

    if(configParms.tipoFigura != NESSUNA)     // bah indica dove sto!
      mLED_1_Toggle();    // 1MHz 11/11/23
    else
      mLED_1_Off();


    if(!sw1) {
      if(oldSw & 1) {
        configParms.tipoFigura++;
        configParms.tipoFigura %= (RUMORE+1);
        setFigura();
        Beep();
        }
      }
    
		if(cnt2>=8) {
			cnt2=0;
			}

		oldSw=(sw1 ? 1 : 0) ;

		}		// second_1


//nonsleep:;
	}


BYTE setFigura(void) {
	WORD n;
	int i;
	BYTE retVal=1;

	switch(configParms.tipoFigura) {
		case NESSUNA:
		  OC1CON1 &= ~0x0006;
#ifdef __XC16
      IEC0bits.T3IE=0;
#else
			DisableIntT3;
#endif
			break;

		case RIGHE_VERTICALI:
#if defined (__dsPIC33EP256MU806__) 
				TRISEbits.TRISE7=1;
#else
				TRISBbits.TRISB0=1;
#endif
#ifdef __XC16
      IEC0bits.T3IE=1;
      IPC2bits.T3IP=5;
      IPC6bits.T4IP=4;
#else
			DisableIntT3;
#endif
		  n = (FCY)/(15625L*configParms.Frequenza);
		  OC1R    = n/16;
		  OC1RS   = n;
		  
		  OC1CON2 = 0x001f;
		  OC1CON1 = 0x1c00;

#ifdef __XC16
      T3CON=0;
      T3CONbits.TCS = 0;                    // clock from peripheral clock
      T3CONbits.TCKPS = 0b00;               // 1:1 prescale
      PR3 = TMR3BASE;                       // rollover every n clocks; 279=4uSec @140MHz, 11/23
      T3CONbits.TON = 1;                    // start timer 
      T4CON=0;
      T4CONbits.TCS = 0;                    // clock from peripheral clock
      T4CONbits.TCKPS = 0b00;                  // 1:1 prescale
      PR4 = configParms.Thickness /*TMR4BASE*/;                           // 
      T4CONbits.TON = 1;                    // start timer to generate ADC triggers
#else
			OpenTimer3(T3_OFF & T3_IDLE_CON & T3_GATE_OFF & T3_PS_1_1 & T3_SOURCE_INT,0);
			DisableIntT3;
#endif
  
  		OC1CON1 |= 0x0006;
			break;

		case RIGHE_ORIZZONTALI:
#ifdef __XC16
      T3CON=0;
      T3CONbits.TCS = 0;                    // clock from peripheral clock
      T3CONbits.TCKPS = 0b00;               // 1:1 prescale
      PR3 = TMR3BASE;                       // rollover every n clocks; 279=4uSec @140MHz, 11/23
      T3CONbits.TON = 1;                    // start timer 
#else
			OpenTimer3(T3_ON & T3_IDLE_CON & T3_GATE_OFF & T3_PS_1_1 & T3_SOURCE_INT,n);
#endif
		  n = (FCY)/(15625L);
		  OC1R    = n/16;
		  OC1RS   = n;
  
#ifdef __XC16
      IEC0bits.T3IE=1;
      IPC2bits.T3IP=5;
      IPC6bits.T4IP=4;
#else
			EnableIntT3;
#endif
		  OC1CON1 = 0x1c00;			//70MHz
  		OC1CON1 |= 0x0006;
			break;

		case RETICOLO:
		  n = (10*FCY/8)/configParms.Frequenza;
#ifdef __XC16
      T3CON=0;
      T3CONbits.TCS = 0;                    // clock from peripheral clock
      T3CONbits.TCKPS = 0b00;               // 1:1 prescale
      PR3 = TMR3BASE;                       // rollover every n clocks; 279=4uSec @140MHz, 11/23
      T3CONbits.TON = 1;                    // start timer 
      T4CON=0;
      T4CONbits.TCS = 0;                    // clock from peripheral clock
      T4CONbits.TCKPS = 0b00;                  // 1:1 prescale
      PR4 = configParms.Thickness /*TMR4BASE*/;                           // 
      T4CONbits.TON = 1;                    // start timer to generate ADC triggers
#else
			OpenTimer3(T3_ON & T3_IDLE_CON & T3_GATE_OFF & T3_PS_1_1 & T3_SOURCE_INT,n);
#endif
		  n = (FCY)/(15625L*configParms.Frequenza);
		  OC1R    = n/16;
		  OC1RS   = n;
  
#ifdef __XC16
      IEC0bits.T3IE=1;
      IPC2bits.T3IP=5;
      IPC6bits.T4IP=4;
#else
			EnableIntT3;
#endif
		  OC1CON1 = 0x1c00;
  		OC1CON1 |= 0x0006;
			break;

		case PUNTI:
		  n = (10*FCY/8)/configParms.Frequenza;
#ifdef __XC16
      T3CON=0;
      T3CONbits.TCS = 0;                    // clock from peripheral clock
      T3CONbits.TCKPS = 0b00;               // 1:1 prescale
      PR3 = TMR3BASE;                       // rollover every n clocks; 279=4uSec @140MHz, 11/23
      T3CONbits.TON = 1;                    // start timer 
      T4CON=0;
      T4CONbits.TCS = 0;                    // clock from peripheral clock
      T4CONbits.TCKPS = 0b00;                  // 1:1 prescale
      PR4 = configParms.Thickness /*TMR4BASE*/;                           // 
      T4CONbits.TON = 1;                    // start timer to generate ADC triggers
#else
			OpenTimer3(T3_ON & T3_IDLE_CON & T3_GATE_OFF & T3_PS_1_1 & T3_SOURCE_INT,n);
#endif
		  n = (FCY)/(15625L*configParms.Frequenza);
		  OC1R    = n/16;
		  OC1RS   = n;
  
#ifdef __XC16
      IEC0bits.T3IE=1;
      IPC2bits.T3IP=5;
      IPC6bits.T4IP=4;
#else
			EnableIntT3;
#endif
		  OC1CON1 = 0x1c00;
  		OC1CON1 |= 0x0006;
			break;

		case SCACCHIERA:
#if defined (__dsPIC33EP256MU806__) 
			TRISEbits.TRISE7=1;
#else
			TRISBbits.TRISB0=1;
#endif
#ifdef __XC16
      T3CON=0;
      T3CONbits.TCS = 0;                    // clock from peripheral clock
      T3CONbits.TCKPS = 0b00;               // 1:1 prescale
      PR3 = TMR3BASE;                       // rollover every n clocks; 279=4uSec @140MHz, 11/23
      T3CONbits.TON = 1;                    // start timer 
#else
			OpenTimer3(T3_ON & T3_IDLE_CON & T3_GATE_OFF & T3_PS_1_1 & T3_SOURCE_INT,n);
#endif
		  n = (FCY)/(15625L*configParms.Frequenza);
		  OC1R    = n/2;
		  OC1RS   = n;
		  
#ifdef __XC16
      IEC0bits.T3IE=1;
      IPC2bits.T3IP=5;
#else
			EnableIntT3;
#endif
		  OC1CON2 = 0x001f;
		  OC1CON1 = 0x1c00;
  		OC1CON1 |= 0x0006;
			break;

// https://www.maximintegrated.com/en/design/technical-documents/app-notes/4/4400.html
		case RUMORE:
#ifdef __XC16
      T3CON=0;
      T3CONbits.TCS = 0;                    // clock from peripheral clock
      T3CONbits.TCKPS = 0b00;               // 1:1 prescale
      PR3 = TMR3BASE;                       // rollover every n clocks; 279=4uSec @140MHz, 11/23
      T3CONbits.TON = 1;                    // start timer 
      IEC0bits.T3IE=1;
      IPC2bits.T3IP=5;
      T4CON=0;
      T4CONbits.TCS = 0;                    // clock from peripheral clock
      T4CONbits.TCKPS = 0b00;                  // 1:1 prescale
      PR4 = configParms.Thickness /*TMR4BASE*/;                           // 
      T4CONbits.TON = 1;                    // start timer to generate ADC triggers
      IPC6bits.T4IP=4;
#else
			OpenTimer3(T3_ON & T3_IDLE_CON & T3_GATE_OFF & T3_PS_1_1 & T3_SOURCE_INT,
				TMR3BASE);			// così a caso
			EnableIntT3;
#endif
		  n = FCY/1000000L;			// 1MHz a caso
		  OC1RS    = n;
		  OC1R   = n/2;
  		OC1CON1 |= 0x0006;
			break;

		}

	return retVal;
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
	RETICOLO,
	16,
	4,			// thickness  /*TMR4BASE*/;                           // 
	100
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

