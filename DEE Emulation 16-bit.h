/************************************************************************
*
*   Emulating Data EEPROM for PIC24 Microcontrollers and
*           dsPIC Digital Signal Controllers
*
*************************************************************************
* FileName:     DEE Emulation 16-bit.h
* Compiler:     MPLAB XC16, v1.30 or higher
* Company:      Microchip Technology, Inc.
*
* Software License Agreement
*
* Copyright © 2016 Microchip Technology Inc. All rights reserved.
*
* Microchip licenses to you the right to use, modify, copy and distribute
* Software only when embedded on a Microchip microcontroller or digital
* signal controller, which is integrated into your product or third party
* product (pursuant to the sublicense terms in the accompanying license
* agreement).
*
* You should refer to the license agreement accompanying this Software for
* additional information regarding your rights and obligations.
*
*
* Author            Date        Comment
*************************************************************************
* D. Otten          2007/05/01  Version 1.0.0 - Initial Release
* D. Otten          2007/05/15  Version 1.0.1 - First publication release
* Pradeep Budagutta 2008/04/02  Version 1.1.0 - Multi EEPROM banks included
* Priyabrata Sinha  2011/01/20  Version 2.0.0 - Added dsPIC33E/PIC24E support
* Priyabrata Sinha  2016/12/20  Version 3.0.0 - Added support for new devices
 * GD/C             2019/12/15  per dsPIC33CH
* GD/C              2021/08/10  per dsPIC33E
************************************************************************/

#ifndef _DEE_EMULATION_H
#define _DEE_EMULATION_H

// User defined constants
#if defined(__dsPIC33C__) 
#define DATA_EE_BANKS       4       // con 8 dà errore in dsPIC
#else
#define DATA_EE_BANKS       2           //0x1000 word ossia 6Kbytes... 2*2 * 1024 (NUMBER_OF_INSTRUCTIONS_IN_PAGE) POSSO USARE fino a 2*255 word (e in softmodem me ne servono 514 byte)
#endif
#define DATA_EE_SIZE        255
#define DATA_EE_TOTAL_SIZE  (DATA_EE_BANKS * DATA_EE_SIZE)
#define NUM_DATA_EE_PAGES   2       // sono 0x800 ogni pagina con EE_BANKS=2!! (su PIC24F)
// in pratica si hanno circa 1/4 dei byte allocati, come usabili... ossia 1Kword con 8Kbyte... (su PIC24)

// Direi che sta cagata funziona così: il byte alto della word-24 è "l'indirizzo", ossia il byte basso dell'indirizzo da usare:
//   il byte alto sceglie la Pagina, e il primo byte della pagina è lo Stato della pagina (per cui rimangono 255 byte WORD usabili, v.sopra)
//   e poi credo che vengano messe a disposizione 2 (NUM_DATA_) pagine per endurance
//   Indirizzi da 0 a 255 vengono scritti tra 3000 e 37ff, 256 lo mette a 3800... 11/8/21

// Modify the following constants based on the specific device being used
#if defined(__dsPIC33E__) || defined(__PIC24E__)
//#define	NUMBER_OF_INSTRUCTIONS_IN_PAGE  512 PORCAMADONNA NON è COSì pic24ep512gp202 (v. DEE del 2014, Larry)); anche CAN GU810
//#define	NUMBER_OF_INSTRUCTIONS_IN_ROW   64
#define	_FLASH_PAGE         1024            // in alcuni xc c'è non c'è fa cacare #cancromicrochip

#define	NUMBER_OF_INSTRUCTIONS_IN_PAGE  _FLASH_PAGE         //1024
#define	NUMBER_OF_INSTRUCTIONS_IN_ROW   128         // _FLASH_ROW ??
#elif defined(__dsPIC33C__) 
#define	NUMBER_OF_INSTRUCTIONS_IN_PAGE  _FLASH_PAGE         //1024
//VERIFICARE!!!
#define	NUMBER_OF_INSTRUCTIONS_IN_ROW   128         // _FLASH_ROW ??
#elif defined(__PIC24F__)
#define	NUMBER_OF_INSTRUCTIONS_IN_PAGE  512 //_FLASH_PAGE  //1024
#define	NUMBER_OF_INSTRUCTIONS_IN_ROW   64
// sul PIC24FV32KA302, la riga è di 32 e l'erase può andare da 32 a 96...
#elif defined(__dsPIC33F__) || defined(__PIC24H__)
#define	NUMBER_OF_INSTRUCTIONS_IN_PAGE  512
#define	NUMBER_OF_INSTRUCTIONS_IN_ROW   64
#endif

// Uncomment the following line if using a device with Flash ECC feature
#if defined(__dsPIC33C__)           // mah credo... 2020
#define __HAS_ECC	1
#endif

// Uncomment the following line if using Auxiliary Flash for EEPROM Emulation
#define __AUXFLASH	1       // sembra andare su dsPIC33E softmodem 11/8/21

#if defined(__dsPIC33E__) || defined(__PIC24E__) || defined(__dsPIC33C__)

#define ERASE_PAGE          0x4003
#define PROGRAM_ROW         0x4002      // sul GP202 non c'è, ma pare che il codice non la usi
#define PROGRAM_WORD        0x4001

#else

#if defined(__HAS_ECC)
#define ERASE_PAGE          0x4003
#define PROGRAM_ROW         0x4002
#define PROGRAM_WORD        0x4001
#else
#define ERASE_PAGE          0x4042
#define PROGRAM_ROW         0x4001
#define PROGRAM_WORD        0x4003
#endif

#endif

#define ERASE_WRITE_CYCLE_MAX           5
//#define NUMBER_OF_ROWS_IN_PAGE          (_FLASH_PAGE \ _FLASH_ROW)      // non è usata ma cmq non andrebbe "£$%
#define NUMBER_OF_ROWS_IN_PAGE          (NUMBER_OF_INSTRUCTIONS_IN_PAGE \ NUMBER_OF_INSTRUCTIONS_IN_ROW)
#define PAGE_AVAILABLE                  1
#define PAGE_CURRENT                    0
#define PAGE_EXPIRED                    0
#define PAGE_NOT_AVAILABLE              0
#define PAGE_NOT_CURRENT                1
#define PAGE_NOT_EXPIRED                1
#define STATUS_AVAILABLE                18
#define STATUS_CURRENT                  19
#define STATUS_EXPIRED                  20

#define GetaddrNotFound() dataEEFlags.addrNotFound
#define SetaddrNotFound(x) dataEEFlags.addrNotFound = x

#define GetPageCorruptStatus() dataEEFlags.pageCorrupt
#define SetPageCorruptStatus(x) dataEEFlags.pageCorrupt = x

#define GetPageExpiredPage() dataEEFlags.expiredPage
#define SetPageExpiredPage(x) dataEEFlags.expiredPage = x

#define GetPageIllegalAddress() dataEEFlags.IllegalAddress
#define SetPageIllegalAddress(x) dataEEFlags.IllegalAddress = x

#define GetPagePackBeforeInit() dataEEFlags.packBeforeInit
#define SetPagePackBeforeInit(x) dataEEFlags.packBeforeInit = x

#define GetPagePackBeforePageFull() dataEEFlags.packBeforePageFull
#define SetPagePackBeforePageFull(x) dataEEFlags.packBeforePageFull = x

#define GetPagePackSkipped() dataEEFlags.packSkipped
#define SetPagePackSkipped(x) dataEEFlags.packSkipped = x

#define GetPageWriteError() dataEEFlags.writeError
#define SetPageWriteError(x) dataEEFlags.writeError = x

typedef union {
    unsigned char val;
    struct {
        unsigned addrNotFound:1;	    // Return 0xFFFF
        unsigned expiredPage:1;	   	    // Return 0x1
        unsigned packBeforePageFull:1;	// Not a return condition
        unsigned packBeforeInit:1;		// Return 0x3
        unsigned packSkipped:1;		    // Return 0x4
        unsigned IllegalAddress:1;	    // Return 0x5
        unsigned pageCorrupt:1;		    // Return 0x6
        unsigned writeError:1;		    // Return 0x7
        };
    } DATA_EE_FLAGS;

extern DATA_EE_FLAGS dataEEFlags;

extern int  ReadPMHigh(int);
extern int  ReadPMLow(int);
extern void UnlockPM(void);
extern int  WritePMHigh(int, int);
extern int  WritePMLow(int, int);

void            UnlockWrite         (void);
int             GetPageStatus       (unsigned char bank, unsigned volatile char page, unsigned volatile char field);
void            ErasePage           (unsigned char bank, unsigned char page);
char            IncEWCount          (unsigned char *index);
unsigned int    GetNextAvailCount   (unsigned char bank);
int             PackEE              (unsigned char bank);
unsigned char   DataEEInit          (void);
unsigned int    DataEERead          (unsigned int addr);
unsigned char   DataEEWrite         (unsigned int data, unsigned int addr);

#endif
