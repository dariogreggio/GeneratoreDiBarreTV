#include <stdio.h>

#ifndef __XC16__
#include <timer.h>
#include <ports.h>
#else
// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
#include <xc.h>
#endif

#include "HardwareProfile.h"
#include "generictypedefs.h"
#include "swi2c.h"

#include <libpic30.h>

#include "generatoreBarre.h"


extern volatile WORD tick10;
extern volatile BYTE second_10;
BYTE divider1s;
extern volatile DWORD milliseconds;

#define H_BLANK 3
const BYTE sinTable[32]={
  8+H_BLANK,9+H_BLANK,10+H_BLANK,12+H_BLANK,13+H_BLANK,14+H_BLANK,14+H_BLANK,15+H_BLANK,   // + 3 v. sotto
	/*15+H_BLANK, meglio*/15+H_BLANK,14+H_BLANK,14+H_BLANK,13+H_BLANK,12+H_BLANK,10+H_BLANK,9+H_BLANK,
	8+H_BLANK,0x6,0x5,0x3,0x2,0x1,0x1,0x0,    // non usate cmq, v.
	0x0,0x0,0x1,0x1,0x2,0x3,0x5,0x6};			//	https://www.daycounter.com/Calculators/Sine-Generator-Calculator2.phtml


/** VECTOR REMAPPING ***********************************************/
    #if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)
        /*
         *	ISR JUMP TABLE
         *
         *	It is necessary to define jump table as a function because C30 will
         *	not store 24-bit wide values in program memory as variables.
         *
         *	This function should be stored at an address where the goto instructions 
         *	line up with the remapped vectors from the bootloader's linker script.
         *  
         *  For more information about how to remap the interrupt vectors,
         *  please refer to AN1157.  An example is provided below for the T2
         *  interrupt with a bootloader ending at address 0x1400
         */
//        void __attribute__ ((address(0x1404))) ISRTable(){
//        
//        	asm("reset"); //reset instruction to prevent runaway code
//        	asm("goto %0"::"i"(&_T2Interrupt));  //T2Interrupt's address
//        }
    #endif


// ---------------------------------------------------------------------------------------
void _ISR __attribute__((__no_auto_psv__)) _AddressError(void) {
	Nop();
	Nop();
	}

void _ISR __attribute__((__no_auto_psv__)) _StackError(void) {
	Nop();
	Nop();
	}
	

void __attribute__ (( interrupt, no_auto_psv )) _T2Interrupt(void) {
// dev'essere 10Hz

//	WriteTimer2(0);	// WRITETIMER0(0) dovrebbe essere la macro!
//  TMR2=0;		non serve su PIC24...
//	mLED_1_Toggle(); 
//	SD_CS^=1;

	tick10++;
	divider1s++;

	second_10=1;					// flag
	if(divider1s==10) {		// per RealTimeClock
		divider1s=0;

		}

	IFS0bits.T2IF = 0; 			//Clear the Timer2 interrupt status flag
	// non lo trova @#£$% T2_Clear_Intr_Status_Bit; 	
	}



void __attribute__ (( interrupt, shadow,  no_auto_psv )) _T3Interrupt(void) {
	static BYTE colCnt,rowCnt,rowCnt2,rowTgl=0,colTgl=0;
  static WORD lfsr = 0xACE1u,oldlfsr=0;
	BYTE b;
// circa 600 .. 1.2uS @140MHz, 40..80 istruzioni, ottimizzazioni=2

//	mLED_1_Toggle(); 
//http://martin.hinner.info/vga/pal.html
  colCnt++;
  colTgl ^= 1;
  colCnt %= 20;     // con 20 escono 17 righe/punti (52uS su 64); v. configParms.Frequenza...
  if(colCnt<2 || colCnt>18) {
    m_VideoPin=0;
    if(!colCnt) {			// sync, poi blank area
      m_SyncPin=0;
      rowCnt++;
      rowCnt %= 17;     // 16*20 righe=320 ~312.5
      if(!rowCnt) {
        rowTgl ^= 1;
        rowCnt2++;
        rowCnt2 %= 18;     // con 20 escono 18 righe/punti; così è ok; v. configParms.Frequenza...
        if(!rowCnt2) {		// inizio frame
          colTgl = rowTgl = 0;

          LATBbits.LATB12 ^=1;      // test vsync
          }
        }
			}
		}
  else {
	  if(rowCnt2)		// vsync
    	m_SyncPin=1;

	  if(rowCnt2>1)	{	// blank area
  
/* jump table in assembler, v. forum
 * CMQ OLTRE UN TOT DI CASI ANCHE IL C USA JUMP TABLE!
 	mov	#_configParms+4,w4
	mov.b	[w4],w4
	ze	w4,w4
	clr	w5
	sub	w4,#12,[w15]
	subb	w5,#0,[w15]
	bra	gtu,.L15
	bra	w4
.L29:
	bra	.L16
	bra	.L17
	bra	.L18
	bra	.L19
	bra	.L20
	bra	.L21
	bra	.L22
	bra	.L23
	bra	.L24
	bra	.L25
	bra	.L26
	bra	.L27
	bra	.L28*/
      
/*void func(int j);*/
      /*
__asm__ (

//".globl _func\n"
 
".equ arg_Index,w0\n"
 
//_func:
//*** sl arg_Index,arg_Index if gotos ***
"mov     #0,w0\n"
// OCCHIO se ottimizzazioni usano W0 per l'ultima variabile da sopra!! specie con XC e ottimizzazioni 2..
"mov.b   _configParms+4,wreg\n"
"bra     w0\n"
"bra     label_NESSUNA\n"
"bra     label_RIGHEVERTICALI\n"
"bra     label_RIGHEORIZZONTALI\n"
"bra     label_RETICOLO\n"
"bra     label_PUNTI\n"
"bra     label_CROCE\n"
"bra     label_SCACCHIERA\n"
"bra     label_CERCHIO\n"
"bra     label_BIANCO\n"
"bra     label_NERO\n"
"bra     label_MEZZO_ORIZZ\n"
"bra     label_MEZZO_VERT\n"
"bra     label_RUMORE\n"
);
*/
//if(configParms.Reverse)  // fare...

	switch(configParms.tipoFigura) {
		case NESSUNA:
__asm__ ("label_NESSUNA:");
      m_VideoPin = 0;    // 
			break;
		case RIGHE_VERTICALI:
__asm__ ("label_RIGHEVERTICALI:");
      m_VideoPin = 1;    // 
      IEC1bits.T4IE=1; 
      break;
		case RIGHE_ORIZZONTALI:
__asm__ ("label_RIGHEORIZZONTALI:");
	  	if(!rowCnt)
      	m_VideoPin = 1;
			else
      	m_VideoPin = 0;
			break;
		case RETICOLO:
__asm__ ("label_RETICOLO:");
      m_VideoPin = 1;
	  	if(rowCnt) 
        IEC1bits.T4IE=1;
			break;
		case PUNTI:
__asm__ ("label_PUNTI:");
	  	if(!rowCnt) {
        m_VideoPin = 1;
        IEC1bits.T4IE=1;
        }
			break;
		case CROCE:
__asm__ ("label_CROCE:");
	  	if(rowCnt2==10 && !rowCnt) // causa lieve ritardo e la croce esce "svarionata" qua!
        m_VideoPin = 1;
      else if(colCnt==11) {
        m_VideoPin = 1;
        IEC1bits.T4IE=1;
        }
			break;
		case SCACCHIERA:
__asm__ ("label_SCACCHIERA:");
      if(rowTgl != colTgl)
        m_VideoPin = 1;    // 
      else
        m_VideoPin = 0;
			break;
		case CERCHIO:
__asm__ ("label_CERCHIO:");
			if(colCnt == sinTable[rowCnt2-2] || colCnt == (18+H_BLANK-sinTable[rowCnt2-2]))   // col=2..18, row=1..17
	      m_VideoPin = 1;    // 
			else
	      m_VideoPin = 0;    // 
			break;
		case BIANCO:
__asm__ ("label_BIANCO:");
	  	if(rowCnt2<17 /* se no salta vsync... sistemare blank area*/) {
        if(!configParms.Reverse)  //
          m_VideoPin = 1;
        else
          m_VideoPin = 0;
        }
			break;
		case NERO:
__asm__ ("label_NERO:");
	  	if(rowCnt2<17 /* se no salta vsync... sistemare blank area*/) {
        if(!configParms.Reverse)  //
          m_VideoPin = 0;
        else
          m_VideoPin = 1;
        }
			break;
		case MEZZO_ORIZZ:
__asm__ ("label_MEZZO_ORIZZ:");
      if(colCnt>10) {
				if(!configParms.Reverse)  //
	        m_VideoPin = 1;
				else
	        m_VideoPin = 0;
        }
			else
        m_VideoPin = 0;
			break;
		case MEZZO_VERT:
__asm__ ("label_MEZZO_VERT:");
	  	if(rowCnt2>9 && rowCnt2<17 /* se no salta vsync... sistemare blank area*/) {
//opp	  	if(rowCnt2<=10) {
				if(!configParms.Reverse)  //
	        m_VideoPin = 1;
				else
	        m_VideoPin = 0;
        }
			else
        m_VideoPin = 0;
			break;
		case RUMORE:
__asm__ ("label_RUMORE:");
//		  OC1RS   = (FCY/1000000L*rand())/RAND_MAX;
	    b  = ((lfsr >> 0) ^ (lfsr >> 2) ^ (lfsr >> 3) ^ (lfsr >> 5) ) & 1;
  	  lfsr =  (lfsr >> 1) | (b << 15);
			OC1R = lfsr >> 10;		// basato su 64
      m_VideoPin = lfsr & 1;
			break;
		}

		}
		}

	IFS0bits.T3IF = 0; 			//Clear the Timer3 interrupt status flag
	// non lo trova @#£$% T3_Clear_Intr_Status_Bit; 	
	}

void __attribute__ (( interrupt, shadow, no_auto_psv )) _T4Interrupt(void) {
  
//if(configParms.Reverse)  // fare...
  m_VideoPin = 0;

  IEC1bits.T4IE=0;
	IFS1bits.T4IF = 0; 			//Clear the Timer4 interrupt status flag
	// non lo trova @#£$% T4_Clear_Intr_Status_Bit; 	
	}

