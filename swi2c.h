#ifndef __XC16__
#include "Compiler.h"
#else
#include <xc.h>
#endif
#include "hardwareprofile.h"
#include <libpic30.h>

unsigned char I2CRX1Byte(void);
unsigned char I2CRXByte(void);
unsigned char I2CTXByte(unsigned char );
unsigned char I2CWritePage(unsigned char,unsigned char);
unsigned char I2CWritePage16(unsigned int,unsigned char);
unsigned char I2CWritePagePoll(void);
unsigned char I2CReadRandom(unsigned char);
unsigned char I2CReadRandom16(unsigned int);
unsigned char I2CRead8Seq(unsigned char,unsigned char);
unsigned char I2CRead16Seq(unsigned int,unsigned char);
void I2CWriteRTC(unsigned char ,unsigned char );
unsigned char I2CReadRTC(unsigned char );
void I2CSTART(void);
void I2CSTOP(void);
unsigned char I2CBITIN(void);
void I2CBITOUT(unsigned char);
void I2CDelay(void);
void setClockHigh(void);

extern unsigned char I2CBuffer[64];

