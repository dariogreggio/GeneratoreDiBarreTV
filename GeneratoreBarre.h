

#define SERNUM      1000
#define VERNUMH     1
#define VERNUML     1

// https://github.com/Picatout/vpc-32/blob/master/hardware/tvout/ntsc.c
// https://www.gamedev.net/blog/161/entry-2165446-dspic33-vdc-with-glcd-or-pal-tv-output/


#define USA_SW_RTC 1

#define CR        0xd
#define LF        0xa

#define DMA_READY __attribute__((/*space(dma),*/aligned(256)))    // 

#ifdef TTY_CGA
#define SCREENSIZE_X 320
#define SCREENSIZE_Y 240

#define USA_DMA 1
#define USA_DMA_8BIT 1


enum __attribute((packed)) VSYNC_STATE {
    FRONT_PORCH=0,
    VSYNC,
    BACK_PORCH,
    TOP_BORDER,
    VFRAME,
    BOTTOM_BORDER
    };


#ifdef USA_DMA
#define VIDEO_BUFSIZE (((SCREENSIZE_X)/8+HORIZ_PORCH_COMP*2)* (SCREENSIZE_Y /*+VERT_PORCH_COMP+VERT_SYNC_COMP*/)) 	// in WORD, v.

//#define VERT_SYNC_COMP 15       // in effetti sarebbero 5+5+5 (o modif. x interlace) con short e long pulses..
//#define VERT_PORCH_COMP 10
#define VERT_SYNC_COMP 20       // in effetti sarebbero 5+5+5 (o modif. x interlace) con short e long pulses..
#define VERT_PORCH_COMP 6
  // occhio DMA conta fino a 0 compreso! v. interrupt
#ifdef USA_DMA_8BIT
#define HORIZ_SYNC_COMP 2  //2     // 0=3.5uS, 1=6; (4.7uS => 8% su 64uS totale @16bit
#define HORIZ_PORCH_COMP 6      // 8uS => 12.5%
// 15.625Hz, 52uS image = 160nS/pixel (320H)
#define HORIZ_SHIFT_COMP 3  //((HORIZ_PORCH_COMP*6)/8)
#else
#define HORIZ_SYNC_COMP 1  //2     // 4.7uS => 8% su 64uS totale @16bit
#define HORIZ_PORCH_COMP 3      // 8uS => 12.5%
// 15.625Hz, 52uS image = 160nS/pixel (320H)
#define HORIZ_SHIFT_COMP 2  //((HORIZ_PORCH_COMP*6)/8)
#endif
#else
#define VIDEO_BUFSIZE (SCREENSIZE_X/8*SCREENSIZE_Y) 	// 
#endif

extern WORD DMA_READY videoRAM[VIDEO_BUFSIZE/2];

int drawPixel(WORD x, WORD y, int c);
int drawLine(WORD x1, WORD y1, WORD x2, WORD y2, int c);
int drawRectangle(WORD x1, WORD y1, WORD x2, WORD y2, int c);
int drawBar(WORD x1, WORD y1, WORD x2, WORD y2, int c);
int drawCircle(WORD x1, WORD y1, WORD s, int c);
int drawCircleFilled(WORD x1, WORD y1, WORD s, int c);
int writeCharAt(WORD x, WORD y, char ch, int c);
int cwrite(char ch);
int writeString(const char *s);
int writeStringAt(WORD x, WORD y, const char *s, int c);
int screenCLS(void);
void setCursor(int x,int y);
int drawImage(const WORD *w);

#endif



// il Timer0 conta ogni 14.28nSec*prescaler... (@140MHz CPUCLK => 70MHz) su dsPIC33

#define TMR2BASE (100000000L/(1000000000.0/(GetPeripheralClock())*256))		//   10Hz per timer
#ifdef TTY_CGA
#ifdef USA_DMA
#ifdef USA_DMA_8BIT
#define TMR3BASE ((156*8)/(1000000000.0/(GetPeripheralClock())*1))	//   160nS/pixel @320x240
//anche qua c'è sempre un ritardino minimo di 100nS... ossia di un SPI cycle, non è chiaro se si può eliminare
#define widthDMA (SCREENSIZE_X/8 +HORIZ_PORCH_COMP)
#else
#define TMR3BASE ((150*16)/(1000000000.0/(GetPeripheralClock())*1) /*11.36*/)	//   160nS/pixel @320x240
#define widthDMA (SCREENSIZE_X/16 +HORIZ_PORCH_COMP)
#endif
#else
#define TMR3BASE (64000/(1000000000.0/(GetPeripheralClock())*1) /*4480*/)	//   64uS @320x240
#endif
#else
#define TMR3BASE (3200/(1000000000.0/(GetPeripheralClock())*1) /*224*/ /*223 overclock*/) // 3.2uS  (279)		//   4uS 
#define TMR4BASE (3)		//   .25uS 
#endif

enum __attribute((packed)) TIPO_FIGURA {
	NESSUNA,
	RIGHE_VERTICALI,
	RIGHE_ORIZZONTALI,
	RETICOLO,
	PUNTI,
	CROCE,
	SCACCHIERA,
	CERCHIO,
	BIANCO,
	NERO,
	MEZZO_ORIZZ,
	MEZZO_VERT,
	RUMORE
	};

struct __attribute((packed)) SAVED_PARAMETERS {
	WORD signature;
	WORD Frequenza;			// num reticolo
	enum TIPO_FIGURA tipoFigura;
	BYTE Thickness;
	BYTE Luminosita;
	BYTE Reverse;
	};

extern struct SAVED_PARAMETERS configParms;
#define PARMS_FIELD(field) configParms.field
BYTE EEleggi(WORD);
void EEscrivi_(WORD,BYTE);



#define BEEP_STD_FREQ 4000

#define EEcopiaARAM(p) { *p=EEleggi(p); }
#define EEcopiaAEEPROM(p) EEscrivi_(p,*p)

void saveSettings(void);
void loadSettings(void);
void resetSettings(void);



	
#define DELAY_SPI_FAST 5
#define Delay08()	__delay_us(TIME_GRANULARITY)			// 1 bit-time 
#define Delay_SPI() __delay_us(DELAY_SPI_FAST /*DelaySPIVal*/)
#define Delay_1uS() __delay_us(1)
#define Delay_uS(x) __delay_us(x)
#define Delay_mS(x) __delay_ms(x)
#define Delay_1mS() Delay_mS(1)
void Delay_mS_LP(BYTE);
void I2CDelay(void);




void Beep(void);


void UserTasks(void);



    // PIC24 RTCC Structure
typedef union {
  struct {
    unsigned char   mday;       // BCD codification for day of the month, 01-31
    unsigned char   mon;        // BCD codification for month, 01-12
    unsigned char   year;       // BCD codification for years, 00-99
    unsigned char   reserved;   // reserved for future use. should be 0
  	};                              // field access	
  unsigned char       b[4];       // byte access
  unsigned short      w[2];       // 16 bits access
  unsigned long       l;          // 32 bits access
	} PIC24_RTCC_DATE;

// PIC24 RTCC Structure
typedef union {
  struct {
    unsigned char   sec;        // BCD codification for seconds, 00-59
    unsigned char   min;        // BCD codification for minutes, 00-59
    unsigned char   hour;       // BCD codification for hours, 00-24
    unsigned char   weekday;    // BCD codification for day of the week, 00-06
  	};                              // field access
  unsigned char       b[4];       // byte access
  unsigned short      w[2];       // 16 bits access
  unsigned long       l;          // 32 bits access
	} PIC24_RTCC_TIME;



DWORD   PIC24RTCCGetTime( void );
DWORD   PIC24RTCCGetDate( void );
void    PIC24RTCCSetTime( WORD weekDay_hours, WORD minutes_seconds );
void    PIC24RTCCSetDate( WORD xx_year, WORD month_day );
void    UnlockRTCC( void );


BYTE to_bcd(BYTE );
BYTE from_bcd(BYTE );



