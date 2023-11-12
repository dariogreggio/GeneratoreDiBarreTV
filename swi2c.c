/* sw_i2c solite 
per PIC24 ossia no pointer dispari

*/


#include "GenericTypedefs.h"

#include "swi2c.h"

unsigned char I2CBuffer[64];

/**************************************************************
       Start Bit Subroutine
       this routine generates a start bit
       (Low going data line while clock is high)          
**************************************************************/
void I2CSTART(void) {

//	SWStartI2C();
	m_I2CClkBit=0;					// make sure clock is low
	m_I2CDataBit=1;					// make sure data is high
	I2CDataTris=0;					// set data and clock lines for output
	I2CDelay();
	setClockHigh();					// CLK=1
	I2CDelay();
	m_I2CDataBit=0;					// data line goes low during high clock for start bit
	I2CDelay();
	m_I2CClkBit=0;					// start clock train
	I2CDelay();
	}

/************************************************************
       Stop Bit Subroutine
       This routine generates a stop bit
       (High going data line while clock is high)
************************************************************/
void I2CSTOP(void) {

//	SWStopI2C();
	m_I2CDataBit=0;						// make sure data line is low
	m_I2CClkBit=0;						// make sure clock is low
	I2CDataTris=0;						// set data/clock lines as outputs
	m_I2CClkBit=1;						// set clock high
	I2CDelay();
	m_I2CDataBit=1;						// data goes high while clock high for stop bit
	I2CDelay();
	m_I2CClkBit=0;						// set clock low again
	I2CDelay();
	I2CDataTris=1;						// set data line as input
//	m_I2CClkBit=1;						// set clock idle

	setClockHigh();						// CLK=1
	// questa e' un'altra attesa, per i dispos. (tipo PICBELL, PICWDI2C) che il WAIT lo fanno DOPO lo STOP! 

//	m_I2CClkBit=0;
	I2CDelay();

	}

/*************************************************************
       BITOUT routine takes the bit of data in C and
       transmits it to the serial EE device
*************************************************************/
void I2CBITOUT(unsigned char n) {

	I2CDataTris=0;					// set data,clock as outputs
	if(n)										// check for state of data bit to xmit
		m_I2CDataBit=1;					// output a low bit     
	else
		m_I2CDataBit=0;					// high? set data line high 
	Nop();										// dev'esserci un min. ritardo tra il cambio in DATA e la salita di CLK
	Nop();
	m_I2CClkBit=1;							// set clock line high
	I2CDelay();
	m_I2CClkBit=0;							// return clock line low
	I2CDelay();
//	retlw   0
	}
                                                             
/**************************************************************
       BITIN routine reads one bit of data from the 
       serial EE device and stores it in C
**************************************************************/
unsigned char I2CBITIN(void) {
	/*overlay*/ unsigned char i;

	I2CDataTris=1;						// make sdata an input line
	I2CDelay();
	m_I2CClkBit=1;							// set clock line high
	I2CDelay();		            // just sit here a sec
	i = m_I2CDataBitI ? 1 : 0;		  // read the data bit
	m_I2CClkBit=0;					    // set clock line low
	return i;
	}

void setClockHigh(void) {
// Gestisce WAIT_STATE dello slave (CLK basso)... che puo' succedere SEMPRE!
	
	m_I2CClkBit=1;

	I2CClkTris=1;						// CLK e' input

	I2CDelay();

	do {
		ClrWdt();
		} while(!m_I2CClkBitI);

	I2CClkTris=0;				// CLK e' output

	}

/****************************************************************
       Transmit Data Subroutine
       This routine takes the unsigned char of data stored in the
       'temp' register and transmits it to the serial EE device.
       It will then send 1 more clock to the serial EE for the
       acknowledge bit.  If the ack bit from the part was low
       then the transmission was sucessful.  If it is high, then
       the device did not send a proper ack bit and the ack
       fail LED will be turned on.
****************************************************************/
#define I2CTXSlaveAddrW() I2CTXByte(0xA0)
#define I2CTXSlaveAddrR() I2CTXByte(0xA1)


unsigned char I2CTXByte(unsigned char n) {
	unsigned char I2CData,I2CCnt;

	I2CData=n;

	for(I2CCnt=0; I2CCnt<8; I2CCnt++) {		// set the #bits to 8
		I2CBITOUT(I2CData & 0x80);		      // send the bit to serial EE
		I2CData <<= 1;											// rotate I2CData/txbuf left, bit in CARRY
		}																		// 8 bits done?
										        // no - go again

	// read ack bit
	I2CDataTris=1;
	I2CDelay();
	setClockHigh();					// aspetto ACK
//		Per WAIT_STATE dello slave (CLK basso)... NON e' chiaro se PRIMA o DOPO ACK... (PicBell lo fa dopo!)
//			questo lo controlla SEMPRE!
	I2CDelay();

	// dopo Delay, qua arrivo con W=0 e Z=1
	I2CCnt=m_I2CDataBitI;						// Z=1 se ACK=OK (basso), altrimenti Z=0 e W!=0

	m_I2CClkBit=0;

	if(I2CCnt) {						// check ack bit
//		m_MemCardLedBit=1;				// spegne! set acknowledge fail LED if the device did not pull data low
		return 0;								// 0=ERR
		}
	return 1;								// 1=OK
	}
                   
                    
/****************************************************************
       Receive data routine
       This routine reads one unsigned char of data from the part
       into the 'I2CData' register.
****************************************************************/
unsigned char I2CRXByte(void) {
	unsigned char I2CData,I2CCnt;

//	SWReadI2C();

//	I2CData=0;							// clear input buffer  non serve!
	      
	for(I2CCnt=0; I2CCnt<8; I2CCnt++) {		// set the #bits to 8
		I2CData <<= 1;				// rotate I2CData 1 bit left
		I2CData |= I2CBITIN();							// read a bit
//		STATUSbits.C = 1;			// la presenza di ASM disabilita l'ottimizzazione, quindi PEGGIORA!
//		Rlcf(input);            // Rotate the carry into the data unsigned char
		}										   // 8 bits done?
													// no, do another

	return I2CData;
	}

/****************************************************************
       Receive data routine
       This routine reads one unsigned char of data from the part
       into the 'I2CData' register.  It then sends a high 
       ACK bit to indicate that no more data is to be read
****************************************************************/
unsigned char I2CRX1Byte(void) {
	unsigned char I2CData,I2CCnt;

//	I2CData=0;							// clear input buffer  non serve!
	      
	for(I2CCnt=0; I2CCnt<8; I2CCnt++) {		// set the #bits to 8
//		STATUSbits.C = 1;			// la presenza di ASM disabilita l'ottimizzazione, quindi PEGGIORA!
//		Rlcf(input);            // Rotate the carry into the data unsigned char
		I2CData <<= 1;		       // rotate I2CData 1 bit left
		I2CData |= I2CBITIN();						// read a bit
		}									  // 8 bits done?
													// no, do another
	// set ack bit = 1
	I2CBITOUT(1);						// to finish transmission

	return I2CData;
	}

/**************************************************************
       READ (read routine)
       This routine reads 8 consecutive addresses of the
       serial EE device starting at address 00 in the
       random access mode (non-sequential read). Reading 
       the device using the random access mode
       requires that the address pointer be set for every 
       unsigned char before the read takes place. The address pointer
       is set by sending a 'write mode' control unsigned char with the
       address of the location to read.  
***************************************************************/
unsigned char I2CReadRandom(unsigned char n) {
	unsigned char I2CData;

//	bcf     port_a,ackf		; clear the ack fail LED if on

  I2CSTART();						// generate start bit
												//
												// now send the write control unsigned char and
												// address to set the pointer
												//
	I2CTXSlaveAddrW();		// get slave address and write mode
//	movfw   I2CAddr				; get unsigned int address
	I2CTXByte(n);		// and send it
											 
											 // now read one unsigned char from the part
											 
	I2CSTART();							// generate start bit
	I2CTXSlaveAddrR();			// get slave address and read mode
	I2CData=I2CRX1Byte();		// read 1 unsigned char from serial EE
	I2CSTOP();							// send stop bit to end transmission

	return I2CData;
	}

unsigned char I2CReadRandom16(unsigned int n) {
	unsigned char I2CData;

//	bcf     port_a,ackf		; clear the ack fail LED if on

  I2CSTART();						// generate start bit
												//
												// now send the write control unsigned char and
												// address to set the pointer
												//
	I2CTXSlaveAddrW();		// get slave address and write mode
							//	move unsigned int address (HIGH)
	I2CTXByte(*(((unsigned char *)&n)+1));						// and send it        
							// move unsigned int address (LOW)
	I2CTXByte(*((unsigned char *)(&n)));						// and send it        
											 
											 // now read one unsigned char from the part
											 
	I2CSTART();							// generate start bit
	I2CTXSlaveAddrR();			// get slave address and read mode
	I2CData=I2CRX1Byte();		// read 1 unsigned char from serial EE
	I2CSTOP();							// send stop bit to end transmission

	return I2CData;
	}

/**************************************************************
       READ (sequential read routine)

       This routine reads 8 consecutive addresses of the
       serial EE device starting at address 00 in the
       sequential read mode. Reading in this mode is more
       efficient than the random read mode as the control unsigned char
       and address have to be sent only once at the beginning
       of the sequence.  As many consecutive addresses as
       needed can then be read from the part until a stop bit is
       sent.  In the read mode, the PIC 16C54 must send the acknowledge
       bit after every 8 data bits from the device.  When the
       last unsigned char needed has been read, then the controller will
       send a high acknowledge bit and then a stop bit to halt
       transmission from the device. 
***************************************************************/
unsigned char I2CRead8Seq(unsigned char n,unsigned char I2CCntB) {			// entra W=indirizzo part. lettura, FSR punta al buffer
	unsigned char *p;

	//	bcf     port_a,ackf		; clear the ack fail LED if on
	p=I2CBuffer;						// bank1, ISP=0!

	I2CSTART();						// generate start bit
	I2CTXSlaveAddrW();		// send slave address and write mode
													// get unsigned int address
	I2CTXByte(n);			// and send it        
	I2CSTART();						// generate start bit
	I2CTXSlaveAddrR();		// send slave address and read mode

I2CRead8Seq2:
	*p++=I2CRXByte();				// read 1 unsigned char from device
	if(!--I2CCntB) {				// are all 8 unsigned chars read?
												// no, send low ack and do another
	// yes, send high ack bit 
		I2CBITOUT(1);					// to stop tranxmission
		I2CSTOP();						// and send a stop bit 
		return 8;
		}

	//  send low ack bit 
	I2CBITOUT(0);				// to continue transmission
	goto I2CRead8Seq2;		//and read another unsigned char
	}

/**************************************************************
       READ16 (sequential read routine)

       This routine reads 8 consecutive addresses of the
       serial EE device starting at given 16bit address in the
       sequential read mode. Reading in this mode is more
       efficient than the random read mode as the control unsigned char
       and address have to be sent only once at the beginning
       of the sequence.  As many consecutive addresses as
       needed can then be read from the part until a stop bit is
       sent.  In the read mode, the PIC 16C54 must send the acknowledge
       bit after every 8 data bits from the device.  When the
       last unsigned char needed has been read, then the controller will
       send a high acknowledge bit and then a stop bit to halt
       transmission from the device. 
***************************************************************/
unsigned char I2CRead16Seq(unsigned int n,unsigned char I2CCntB) {			// entra indirizzo part. lettura, FSR punta al buffer
	unsigned char *p;

//	bcf     port_a,ackf		; clear the ack fail LED if on
	
	I2CSTART();						// generate start bit
	I2CTXSlaveAddrW();		// send slave address and write mode

							//	move unsigned int address (HIGH)
	I2CTXByte(*(((unsigned char *)&n)+1));						// and send it        
							// move unsigned int address (LOW)
	I2CTXByte(*((unsigned char *)(&n)));						// and send it        

	I2CSTART();							// generate start bit
	I2CTXSlaveAddrR();			// send slave address and read mode

	p=I2CBuffer;
	
I2CRead16Seq2:
	*p++=I2CRXByte();				// read 1 unsigned char from device
	if(!--I2CCntB) {					// are all n unsigned chars read?
													// no, send low ack and do another
		// yes, send high ack bit 
		I2CBITOUT(1);					// to stop tranxmission
		I2CSTOP();						// and send a stop bit 
		return 16;
		}

  // send low ack bit 
	I2CBITOUT(0);		      // to continue transmission
	goto I2CRead16Seq2;		// and read another unsigned char
	}

/****************************************************************
       Byte Write Routine
       This routine writes the data in "temp" to 
       8 consecutive unsigned chars in the serial EE device starting
       at address 00.  This routine waits 10mS after every
       unsigned char to give the device time to do the write.  This 
       program repeats forever. 
*****************************************************************/
void I2CWriteByte(unsigned char n, unsigned char b) {
	unsigned char I2CAddr,I2CCnt;

//	SWWriteI2C();

//	clrf    port_a				; clear all LEDs
	I2CAddr=n;						// set starting address to W
//	temp=0x55;						// set data to write as 55h

	// set number of unsigned chars
	for(I2CCnt=0; I2CCnt<8; I2CCnt++) {		// set the #bits to 8

	  I2CSTART();						// generate start bit
		I2CTXSlaveAddrW();		// send slave address and write mode
		// move unsigned int address
		I2CTXByte(I2CAddr);		// and send it        
		I2CTXByte(b);					// move data unsigned char and transmit it
		I2CSTOP();						// generate stop bit
	
	//	movlw   10
	//	movwf   loops					; set delay time to give
	//	call    WAIT					; 10 ms wait after every unsigned char
		I2CWritePagePoll();
	
		I2CAddr++;						// add 1 to address counter
		}											// all 8 unsigned chars written?
													// no, do another unsigned char
	}

/****************************************************************
       Page Write Routine

       This routine uses page mode to write the data in "temp" to 
       8 consecutive unsigned chars in the serial EE device starting
       at address 00. This routine waits 10mS after every
       page to give the device time to do the write.  This
       routine executes forever 
*****************************************************************/

unsigned char I2CWritePage(unsigned char n,unsigned char I2CCntB) {					// entra W=indirizzo part. lettura, (FSR usato per buffer)
	unsigned char *p;

//	clrf    port_a					; clear all LEDs
	p=I2CBuffer;

	I2CSTART();							// generate start bit
	I2CTXSlaveAddrW();			// send slave address and write mode

	// move unsigned int address
	I2CTXByte(n);						// and send it        

	do {
		I2CTXByte(*p++);						// and transmit it
		} while(--I2CCntB);						// all n unsigned chars written?
																// no, do another
	I2CSTOP();							// yes,generate stop bit

//	movlw   10
//	movwf   loops						; set delay time to give
//	call    WAIT						; 10 ms wait after every unsigned char
	__delay_ms(20);							// 50mS

	return 1;
	}

/****************************************************************
       Page Write 16 Routine (8 unsigned chars, address a 16 bit)

       This routine uses page mode to write the data in "temp" to 
       8 consecutive unsigned chars in the serial EE device starting
       at given address . This routine waits 10mS after every
       page to give the device time to do the write.  This
       routine executes forever 
*****************************************************************/
#define I2CWritePage16Default() I2CWritePage16((unsigned int)&I2CAddr)


unsigned char I2CWritePage16(unsigned int n,unsigned char I2CCntB) {		// entra W=puntat. all'indirizzo part. lettura, (FSR usato per buffer)
	unsigned char *p;

// clrf    port_a					; clear all LEDs

	I2CSTART();							// generate start bit
	I2CTXSlaveAddrW();			// send slave address and write mode

							//	move unsigned int address (HIGH)
	I2CTXByte(*(((unsigned char *)&n)+1));						// and send it        
							// move unsigned int address (LOW)
	I2CTXByte(*((unsigned char *)(&n)));						// and send it        

	p=I2CBuffer;

	do {
		I2CTXByte(*p++);				// and transmit it
		} while(--I2CCntB);					// move data unsigned char 
														// all n unsigned chars written?
														// no, do another
	I2CSTOP();							// yes,generate stop bit

//	movlw   10
//	movwf   loops						; set delay time to give
//	call    WAIT						; 10 ms wait after every unsigned char
	__delay_ms(5);							// 20mS no 5, 2015!
	ClrWdt();			// su C30 questa non pulisce WDT
	return 16;
	}


unsigned char I2CWritePagePoll(void) {
	unsigned char I2Cpollcnt,i;

	I2Cpollcnt=100;						// set max number of times to poll as 100
	do {
		I2CSTART();								// generate start bit
		i=I2CTXSlaveAddrW();				// send slave address and write mode
		if(i)										// was the ack bit low?
			goto exitpoll;				// yes, do another unsigned char
		} while(--I2Cpollcnt);		 // is poll counter down to zero?
															 // no, poll again.  Otherwise the part is
//	bsf     port_a,timeout	; not responding in time so set timeout LED and continue on 

exitpoll:
		;
	}


void I2CDelay(void) {			// circa 2,5uSec (400KHz operation)
	unsigned char uSec;			// 1.25 ca.

	uSec=1;
	do {
		ClrWdt();							// Clear the WDT
		__delay_us(1);			// circa
		} while(--uSec);
	}

