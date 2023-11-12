/********************************************************************
 FileName:     	HardwareProfile - PIC24FJ256GB110 PIM.h
 Dependencies:  See INCLUDES section
 Processor:     PIC24FJ256GB110
 Hardware:      PIC24FJ256GB110 PIM
 Compiler:      Microchip C30
 Company:       Microchip Technology, Inc.

 Software License Agreement:

 The software supplied herewith by Microchip Technology Incorporated
 (the “Company”) for its PIC® Microcontroller is intended and
 supplied to you, the Company’s customer, for use solely and
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
  Rev   Date         Description
  1.0   11/19/2004   Initial release
  2.1   02/26/2007   Updated for simplicity and to use common
                     coding style
  2.3   09/15/2008   Broke out each hardware platform into its own
                     "HardwareProfile - xxx.h" file
				22.6.2017		porcodio GC
				23/7/2020		sempre morte agli umani ;) GD/C su pcb forgetIvrea 2020
********************************************************************/

#ifndef HARDWARE_PROFILE_PIC24FJ256GB110_PIM_H
#define HARDWARE_PROFILE_PIC24FJ256GB110_PIM_H


   
    //Uncomment this to make the output HEX of this project 
    //   to be able to be bootloaded using the HID bootloader
//    #define PROGRAMMABLE_WITH_USB_HID_BOOTLOADER	

    //If the application is going to be used with the HID bootloader
    //  then this will provide a function for the application to 
    //  enter the bootloader from the application (optional)
    #if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)
        #define EnterBootloader() __asm__("goto 0x400")
    #endif   


    /*******************************************************************/
    /*******************************************************************/
    /*******************************************************************/
    /******** Application specific definitions *************************/
    /*******************************************************************/
    /*******************************************************************/
    /*******************************************************************/

    /** Board definition ***********************************************/
    //These definitions will tell the main() function which board is
    //  currently selected.  This will allow the application to add
    //  the correct configuration bits as wells use the correct
    //  initialization functions for the board.  These defitions are only
    //  required in the stack provided demos.  They are not required in
    //  final application design.
    #if defined (__dsPIC33EP512MU810__) || defined (__dsPIC33EP256MU806__) || defined(__PIC24EP512GU810__) || defined(__PIC24EP256GP202__) || defined(__PIC24EP512GP202__)
    
        #define GetSystemClock()            140000000UL
        #define GetPeripheralClock()        (GetSystemClock())
        #define GetInstructionClock()       (GetSystemClock())
        #define FCY (GetSystemClock()/2)		// per LibPic30.h e delay

    
    #endif

    /** LED ************************************************************/

#if defined (__dsPIC33EP256MU806__) 	// forgetIvreaAna pcb 2020

    #define mInitAllLEDs()      LATD &= 0b1111011111111111; TRISD &= 0b1111011111111111; LATF &= 0b1111111111001101; TRISF &= 0b1111111111001101; 
    
#define LED2_TRIS			(TRISDbits.TRISD11)	// 
#define LED2_IO				(LATDbits.LATD11)
#define LED0_TRIS			(TRISFbits.TRISF4)	// 
#define LED0_IO				(LATFbits.LATF4)
#define LED1_TRIS			(TRISFbits.TRISF5)	// 
#define LED1_IO				(LATFbits.LATF5)

		// 
    #define mLED_1              LATFbits.LATF4
    #define mLED_2              LATFbits.LATF5
    #define mLED_3              LATDbits.LATD11
    #define mLED_4              LATFbits.LATF1

    #define mGetLED_1()         mLED_1
    #define mGetLED_2()         mLED_2
    #define mGetLED_3()         mLED_3
    #define mGetLED_4()         mLED_4
    
    #define mLED_1_On()         mLED_1 = 1;
    #define mLED_2_On()         mLED_2 = 1;
    #define mLED_3_On()         mLED_3 = 1;
    #define mLED_4_On()         mLED_4 = 1;
    
    #define mLED_1_Off()        mLED_1 = 0;
    #define mLED_2_Off()        mLED_2 = 0;
    #define mLED_3_Off()        mLED_3 = 0;
    #define mLED_4_Off()        mLED_4 = 0;
    
    #define mLED_1_Toggle()     mLED_1 = !mLED_1;
    #define mLED_2_Toggle()     mLED_2 = !mLED_2;
    #define mLED_3_Toggle()     mLED_3 = !mLED_3;
    #define mLED_4_Toggle()     mLED_4 = !mLED_4;

    #define mSetLED_1(in)         mLED_1 = in;
    #define mSetLED_2(in)         mLED_2 = in;
    #define mSetLED_3(in)         mLED_3 = in;
    #define mSetLED_4(in)         mLED_4 = in;
    

    
		// OSCI e OSCO, come pure SOSCI e SOSCO .. meglio lasciarli liberi...

    /** SWITCH *********************************************************/
    #define mInitSwitch1()      TRISBbits.TRISB15=1;CNPUBbits.CNPUB15=1;
    #define mInitSwitch2()      TRISDbits.TRISD0=1;CNPUDbits.CNPUD0=1;
    #define mInitSwitch3()      TRISGbits.TRISG7=1;CNPUGbits.CNPUG7=1;
    #define mInitSwitch4()      
    #define mInitAllSwitches()  mInitSwitch1();mInitSwitch2();mInitSwitch3();mInitSwitch4();
    #define sw1                 PORTBbits.RB15
    #define sw2                 PORTDbits.RD0		// IO0
    #define sw3                 PORTGbits.RG7		// DTR/boot
    #define sw4                 



		#define m_I2CClkBit           LATDbits.LATD10
		#define m_I2CDataBit          LATDbits.LATD9
		#define m_I2CDataBitI         PORTDbits.RD9
		#define m_I2CClkBitI          PORTDbits.RD10
		#define I2CDataTris      TRISDbits.TRISD9
		#define I2CClkTris       TRISDbits.TRISD10
		#define SPI_232_I 			PORTDbits.RD9
		#define SPI_232_IO 			LATDbits.LATD9
		#define SPI_232_I_TRIS 			TRISDbits.TRISD9
		#define SPI_232_O 			LATDbits.LATD10
		#define SPI_232_OI 			PORTDbits.RD10
		#define SPI_232_O_TRIS 			TRISDbits.TRISD10



#else

		#define mInitAllLEDs()      LATB &= 0b1111111111111111; TRISB &= 0b1111101111111111; 
    
#define LED0_TRIS			(TRISBbits.TRISB10)	//
#define LED0_IO				(LATBbits.LATB10)
//#define LED1_TRIS			(TRISBbits.TRISB7)	// 
//#define LED1_IO				(LATBbits.LATB7)
//#define LED2_TRIS			(TRISBbits.TRISB5)	// 
//#define LED2_IO				(LATBbits.LATB5)

		// 
    #define mLED_1              LATBbits.LATB10
    #define mGetLED_1()         mLED_1
    #define mLED_1_On()         mLED_1 = 1
    #define mLED_1_Off()        mLED_1 = 0
    #define mLED_1_Toggle()     mLED_1 ^= 1
//    #define mLED_2              LATBbits.LATB7
    #define mGetLED_2()         mLED_2
    #define mLED_2_On()         mLED_2 = 1
    #define mLED_2_Off()        mLED_2 = 0
    #define mLED_2_Toggle()     mLED_2 ^= 1
//    #define mLED_3              LATBbits.LATB5

    
		// OSCI e OSCO, come pure SOSCI e SOSCO .. meglio lasciarli liberi...

    /** SWITCH *********************************************************/
    #define mInitSwitch1()      TRISBbits.TRISB7=1;
    #define mInitSwitch2()      
    #define mInitSwitch3()      
    #define mInitSwitch4()      
    #define mInitAllSwitches()  mInitSwitch1();mInitSwitch2();mInitSwitch3();mInitSwitch4();
    #define sw1                 PORTBbits.RB7
//    #define sw2                 PORTBbits.RB4
//    #define sw4                 PORTBbits.RB3		// invertiti? boh
//    #define sw3                 PORTBbits.RB2

		#define mSwitch1 LATDbits.LATB7
//		#define mSwitch2 LATDbits.LATD15


		#define m_I2CClkBit           LATBbits.LATB10
		#define m_I2CDataBit          LATBbits.LATB9
		#define m_I2CDataBitI         PORTBbits.RB9
		#define m_I2CClkBitI          PORTBbits.RB10
		#define I2CDataTris      TRISBbits.TRISB9
		#define I2CClkTris       TRISBbits.TRISB10
		#define SPI_232_I 			PORTBbits.RB9
		#define SPI_232_IO 			LATBbits.LATB9
		#define SPI_232_I_TRIS 			TRISBbits.TRISB9
		#define SPI_232_O 			LATBbits.LATB10
		#define SPI_232_OI 			PORTBbits.RB10
		#define SPI_232_O_TRIS 			TRISBbits.TRISB10

#define m_SyncPin LATBbits.LATB14
#define m_VideoPin LATBbits.LATB13
// alt-video ev. = RB9/PWM OC1

#endif

    /** POT ************************************************************/
//    #define mInitPOT()  {AD1PCFGLbits.PCFG5 = 0;    AD1CON2bits.VCFG = 0x0;    AD1CON3bits.ADCS = 0xFF;    AD1CON1bits.SSRC = 0x0;    AD1CON3bits.SAMC = 0b10000;    AD1CON1bits.FORM = 0b00;    AD1CON2bits.SMPI = 0x0;    AD1CON1bits.ADON = 1;}
// 		RB13


    /** I/O pin definitions ********************************************/
    #define INPUT_PIN 1
    #define OUTPUT_PIN 0


#endif  //HARDWARE_PROFILE_PIC24FJ256GB110_PIM_H

