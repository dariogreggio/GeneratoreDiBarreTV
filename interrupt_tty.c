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

extern BYTE videoTextRAM[(320/8)*(240/8)];
extern WORD videoCharPointer;
extern WORD verticalLines,horizontalPixels;
static WORD blackline[320/2+HORIZ_PORCH_COMP  +HORIZ_SHIFT_COMP]={0};
//static WORD chekeredline[320/2+HORIZ_PORCH_COMP   +HORIZ_SHIFT_COMP]={0x5555}; per test!
WORD widthDMA;
volatile enum VSYNC_STATE VSyncState=FRONT_PORCH;


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



static WORD *pVideo;
static BYTE soloSync=0;
static WORD curLine=0;
static BYTE spi_cnt=0;
#ifndef USA_DMA
void __attribute__ ((interrupt, shadow, no_auto_psv)) _T3Interrupt(void) {
#warning OCCHIO usare registri in asm con XC e ottimizzazioni 2..
	switch(curLine) {
		case 0:			// 
			pVideo=&videoRAM[0];
			IFS0bits.SPI1IF=1;    // serve??
//			spi_cnt=0;
		case 1:
		case 2:			
		case 3:
		case 4:
		case 5:
		case 6:			// back porch;
		case 7:
		case 8:
		case 9:
		case 10:
		case 11:
		case 12:
		case 13:
		case 14:
			m_SyncPin=0;    // segue da sotto...
      break;
		case 15:		// inactive area top (?)
		case 16:
		case 17:
		case 18:
		case 19:
		case 20:
		case 21:
		case 22:
		case 23:
		case 24:
		case 25:
		case 26:
		case 27:
		case 28:
		case 29:
		case 30:
		case 31:
		case 32:
		case 33:
		case 34:
		case 35:      //312-240 /2


		case 276:
		case 277:
		case 278:
		case 279:
		case 280:
		case 281:
		case 282:
		case 283:
		case 284:
		case 285:
		case 286:
		case 287:
		case 288:
		case 289:
		case 290:
		case 291:
		case 292:
		case 293:
		case 294:
		case 295:
		case 296:
		case 297:
		case 298:			// (front porch) inactive area bottom
		case 299:
		case 300:
		case 301:
		case 302:
		case 303:
		case 304:
		case 305:
		case 306:
		case 307:
		case 308:
			soloSync=1;
//			spi_cnt=0;
			IEC0bits.SPI1IE=1;		
			IFS0bits.SPI1IF=1;		
			break;
		case 309:			// devo mettere una pausina (sull HSync) se no certi monitor si arrabbiano!
			break;
		case 310:			// VSync
			m_SyncPin=0;
			break;
		case 311:
			break;
		case 312:
			curLine=0xffff;
//			soloSync=1;
//			EnableIntSPI1;			mah, non credo
//			IFS0bits.SPI1IF=1;
			break;

		default:
			soloSync=0;
//			spi_cnt=0;
			IEC0bits.SPI1IE=1;
			IFS0bits.SPI1IF=1;		//myWriteSPI1(0);			// scatena 1° IRQ!
			break;
		}

	curLine++;
	__builtin_btg((unsigned int *)&LATB,12);

	IFS0bits.T3IF = 0; 			//Clear the Timer3 interrupt status flag
	//T3_Clear_Intr_Status_Bit; 
	}

void __attribute__ ((interrupt, shadow, no_auto_psv)) _SPI1Interrupt(void) {


// http://www.batsocks.co.uk/readme/video_timing.htm
  switch(spi_cnt) {
		case 0:         // Hsync
			m_SyncPin=0;
      SPI1BUF=0;    // 4.7uS circa
      SPI1BUF=0;
			spi_cnt++;
			break;

		case 1:						// back porch 5.7uS (16bit @6MHz, x2)
			m_SyncPin=1;
      SPI1BUF=0;
      SPI1BUF=0;
			spi_cnt++;
			break;

		case 2:
		case 3:   // 8x16bit, 160nS l'uno => 
			if(soloSync) {
//Nop();Nop();Nop();Nop();Nop();Nop();			// per omogeneità..
        SPI1BUF=0;
        SPI1BUF=0;
        SPI1BUF=0;
        SPI1BUF=0;
        SPI1BUF=0;
        SPI1BUF=0;
        SPI1BUF=0;
        SPI1BUF=0;
				}
			else {
        SPI1BUF=*pVideo++;
        SPI1BUF=*pVideo++;
        SPI1BUF=*pVideo++;
        SPI1BUF=*pVideo++;
        SPI1BUF=*pVideo++;
        SPI1BUF=*pVideo++;
        SPI1BUF=*pVideo++;
        SPI1BUF=*pVideo++;
        }			// soloSync
			spi_cnt++;
			break;

 		case 4:   // 4x16bit, 160nS l'uno => 
			if(soloSync) {
//Nop();Nop();Nop();Nop();Nop();Nop();			// per omogeneità..
        SPI1BUF=0;
        SPI1BUF=0;
        SPI1BUF=0;
        SPI1BUF=0;
				}
			else {
        SPI1BUF=*pVideo++;
        SPI1BUF=*pVideo++;
        SPI1BUF=*pVideo++;
        SPI1BUF=*pVideo++;
        }			// soloSync
			spi_cnt++;
			break;

		case 5:
      SPI1BUF=0;  	// 1.6uS front porch
			IEC0bits.SPI1IE=0;
			spi_cnt=0;
			break;
		}

//	__builtin_btg(&LATB,12);

	IFS0bits.SPI1IF = 0; // Clear the Interrupt flag
	}
#endif

#ifdef USA_DMA
void __attribute__ ((interrupt, shadow, no_auto_psv)) _DMA0Interrupt(void) {
//http://martin.hinner.info/vga/pal.html
//http://www.doom9.org/index.html?/capture/analogue_video.html
  
  if(DMA0CNT==HORIZ_SYNC_COMP /* 8% circa */) {
    curLine++;
//    DMA0CONbits.CHEN = 0; //non serve
    switch(VSyncState) {
      case FRONT_PORCH:   // diminuendo qua, scende; aumentando sale
        if(curLine>=(VERT_PORCH_COMP/2)) {   // front porch % circa
          curLine=0;
    			m_SyncPin=0;
        	__builtin_btg(&LATB,12);    // frame counter :) v. PC_PIC
          VSyncState++;
          }
        else
          m_SyncPin=1;
        DMA0STAL = __builtin_dmaoffset(&blackline);
//        DMA0STAH = 0;
        break;
      case VSYNC:
        if(curLine>=VERT_SYNC_COMP) {   // 15 linee circa
          curLine=0;
    			m_SyncPin=1;
          // ovvero (dice) potrebbero essere tanti impulsi da 1/2 riga (32uS)..
          VSyncState++;
          }
        else
          m_SyncPin=0;    //opzionale
        DMA0STAL = __builtin_dmaoffset(&blackline);
//        DMA0STAH = 0;
        break;
      case BACK_PORCH:
  			m_SyncPin=1;
        // TOTALE 24! (312.5 - 288)
        if(curLine>=(VERT_PORCH_COMP/2)) {   // back porch % circa
          curLine=0;
          VSyncState++;
          }
        DMA0STAL = __builtin_dmaoffset(&blackline);
//        DMA0STAH = 0;
        break;
      case TOP_BORDER:
  			m_SyncPin=1;
        DMA0STAL = __builtin_dmaoffset(&blackline);
//        DMA0STAH = 0;
        if(curLine>=24) {
          curLine=0;
          VSyncState++;
          }
        break;
      case VFRAME:
  			m_SyncPin=1;
        DMA0STAL = /*__builtin_dmaoffset*/ ((WORD*)&videoRAM)+curLine*widthDMA;  // transfer source physical address

        
//        DMA0STAL = __builtin_dmaoffset(&chekeredline);
        
        //        DMA0STAH = 0;
        if(curLine>=SCREENSIZE_Y) {   // BISOGNa CENTRARE! noi abbiamo 240 ma lo standard è 288
          curLine=0;
          VSyncState++;
          }
        break;
      case BOTTOM_BORDER:
    		m_SyncPin=1;
        DMA0STAL = __builtin_dmaoffset(&blackline);     // ev. colore bordo...
//        DMA0STAH = 0;
        if(curLine>=24) {
          curLine=0;
          VSyncState=FRONT_PORCH;
          }
        break;
      }
    DMA0CNT=widthDMA   -1;
//    DMA0CNT=10;
    DMA0CONbits.CHEN = 1; // perché non abbiam messo continous
    }
  else {
//    while(!SPI1STATbits.SPIRBF);    // non va... forse bisognerebbe gestire IRQ del DMA dummy/RX...
    __delay_us(4);
		m_SyncPin=0;
//    DMA0CONbits.CHEN = 0; //non serve
    DMA0STAL = __builtin_dmaoffset(&blackline);  // zeri... dovrebbe
//    DMA0STAH = 0;
    DMA0CNT=HORIZ_SYNC_COMP;
    DMA0CONbits.CHEN = 1; // perché non abbiam messo continous
    }

	IFS0bits.DMA0IF = 0;	 // Clear the interrupt flag!
  }
#endif


BYTE ByteRec;
void __attribute__ ((interrupt, no_auto_psv )) _U1RXInterrupt(void) {

  if(U1STAbits.FERR) {
		U1STAbits.FERR=0;
//	  CommStatus.COMM_FRERR=1;
		}
	if(U1STAbits.OERR) {			// non mi interessano i caratteri ev. in attesa...
		U1STAbits.OERR=0;

//		CommStatus.COMM_OVRERR=1;
		}

	while(!U1STAbits.URXDA);
//	while(DataRdyUART1()) {	// bah non si capisce se ha senso.. credo di no cmq
	  ByteRec = U1RXREG;

    cwrite(ByteRec);
    
//		m_Led2Bit ^= 1;			// ***test
//		CommStatus.FRAME_REC=1;


//  U1RX_Clear_Intr_Status_Bit;
	IFS0bits.U1RXIF = 0;		// teste di merda
	}
