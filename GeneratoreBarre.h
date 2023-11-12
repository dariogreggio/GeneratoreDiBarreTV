


#define SERNUM      1000
#define VERNUMH     0
#define VERNUML     1


//#define FLASH_TIME 7
#define USA_SW_RTC 1



// il Timer0 conta ogni 62.5nSec*prescaler... (@32MHz CPUCLK => 16MHz) su PIC24
#define TMR2BASE (27350 /*6260*/-10)		//   10Hz per timer
#define TMR3BASE (223) // 3.2uS  (279)		//   4uS 
#define TMR4BASE (18)		//   .25uS 

enum TIPO_FIGURA {
	NESSUNA,
	RIGHE_VERTICALI,
	RIGHE_ORIZZONTALI,
	RETICOLO,
	PUNTI,
	SCACCHIERA,
	CERCHIO,
	RUMORE
	};

struct SAVED_PARAMETERS {
	WORD signature;
	enum TIPO_FIGURA tipoFigura;
	WORD Frequenza;			// num reticolo
	BYTE Thickness;
	BYTE Luminosita;
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




void prepOutBuffer(BYTE );
void clearOutBuffer(void);
void prepStatusBuffer(BYTE);


	
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



