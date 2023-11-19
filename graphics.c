#ifndef __XC16__
#include "compiler.h"
#else
#include <xc.h>
#endif
#include "generictypedefs.h"
#include "generatoreBarre.h"

extern const char font8x8_basic[128][8];
#ifdef USA_DMA_8BIT
const WORD bitLUT[16]= {128,64,32,16,8,4,2,1, 32768,16384,8192,4096,2048,1024,512,256};
#else
const WORD bitLUT[16]= {32768,16384,8192,4096,2048,1024,512,256, 128,64,32,16,8,4,2,1};
#endif
extern BYTE cursor_x,cursor_y;

#define max(a,b) ((a)>(b) ? (a) : (b))
#define abs(a) ((a)>=0 ? (a) : (-a))

int drawPixel(WORD x, WORD y, int c) {
	WORD i,j;

	ClrWdt();

	if(y>=SCREENSIZE_Y || x>=SCREENSIZE_X)	
		return 0;

#ifdef USA_DMA
  i=(((WORD)SCREENSIZE_X)/16  +HORIZ_PORCH_COMP)*y + x/16   +HORIZ_SHIFT_COMP;
#else
	i=(SCREENSIZE_X/16)*y+(x/16);
#endif
	j=bitLUT[x & 15];
  if(!configParms.Reverse) {
    if(c & 1)
      videoRAM[i] |= j;
    else
      videoRAM[i] &= ~j;
    }
  else {
    if(c & 1)
      videoRAM[i] &= ~j;
    else
      videoRAM[i] |= j;
    }

	return 1;
	}

int drawLine(WORD x1, WORD y1, WORD x2, WORD y2, int c) {
	WORD x,y;
	int d1,d2,t,t2;

	if(y1==y2) {
// ottimizzo linee orizzontali con 0xffff ...
		WORD i;
		if(x1>x2) {
			t=x1; x1=x2; x2=t;
			}
		t=(x1+15) & 0xfff0;			// plotto i pixel prima del modulo-16...
		for(x=x1; x<t; x++) 
			drawPixel(x,y1,c);

		t=x2 & 0xfff0;
#ifdef USA_DMA
    i=(((WORD)SCREENSIZE_X)/16  +HORIZ_PORCH_COMP)*y1 + x/16   +HORIZ_SHIFT_COMP;
#else
		i=(SCREENSIZE_X/16)*y1+(x/16);
#endif	
    for(; x<t; x+=16) {			// ora, i blocchi da 16pixel
			if(c & 1)
				videoRAM[i] = configParms.Reverse ? 0x0000 : 0xffff;
      else
				videoRAM[i] = configParms.Reverse ? 0xffff : 0x0000;
      i++;
			}
		for(; x<x2; x++) 				// ...infine plotto i pixel dopo il modulo-16!
			drawPixel(x,y1,c);
		}
	else {
    d1=abs(x2-x1);
    d2=abs(y2-y1);
    if(d1>d2) {
      if(x1>x2) {
        t=x1; x1=x2; x2=t;
        }
      t=0;
      y=y1;
      t2=y2>y1 ? +1 : -1;
      for(x=x1; x<x2; x++) {
        drawPixel(x,y,c);
  // Bresenham http://c.happycodings.com/games-and-graphics/code18.html
        t+=d2;
        if(t>d1) {
          y+=t2;
          t=0;
          }
        }
      }
    else {
      if(y1>y2) {
        t=y1; y1=y2; y2=t;
        }
      t=0;
      x=x1;
      t2=x2>x1 ? +1 : -1;
      for(y=y1; y<y2; y++) {
        drawPixel(x,y,c);
        t+=d1;
        if(t>d2) {
          x+=t2;
          t=0;
          }
        }
      }
    }

	return 1;
	}

int drawRectangle(WORD x1, WORD y1, WORD x2, WORD y2, int c) {
	WORD x,y;

	if(x1>x2 || y1>y2)
		return 0;
	drawLine(x1,y1,x2,y1,c);
	drawLine(x1,y1,x1,y2,c);
	drawLine(x2,y1,x2,y2,c);
	drawLine(x1,y2,x2,y2,c);

	return 1;	
	}

int drawBar(WORD x1, WORD y1, WORD x2, WORD y2, int c) {
	WORD x,y;

	if(x1>x2 || y1>y2)
		return 0;
	for(y=y1; y<y2; y++) {
		drawLine(x1,y,x2,y,c);
		}

	return 1;	
	}

int screenCLS(void) {
	WORD i;

	for(i=0; i<VIDEO_BUFSIZE/2; i++) {
  	videoRAM[i]=configParms.Reverse ? 0xffff : 0x0000;
		ClrWdt();
		}
  cursor_x=cursor_y=0;

	return 1;	
	}

int drawCircle(WORD x0, WORD y0, WORD r, int color) {   // v. anche PC_PIC cmq
  int f = 1 - r;
  int ddF_x = 1;
  int ddF_y = -2 * r;
  int x=0;
  int y=r;

    drawPixel(x0  , y0+r, color);
    drawPixel(x0  , y0-r, color);
    drawPixel(x0+r, y0  , color);
    drawPixel(x0-r, y0  , color);

  while(x<y) {
    if(f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
      }
    x++;
    ddF_x += 2;
    f += ddF_x;

      drawPixel(x0 + x, y0 + y, color);
      drawPixel(x0 - x, y0 + y, color);
      drawPixel(x0 + x, y0 - y, color);
      drawPixel(x0 - x, y0 - y, color);
      drawPixel(x0 + y, y0 + x, color);
      drawPixel(x0 - y, y0 + x, color);
      drawPixel(x0 + y, y0 - x, color);
      drawPixel(x0 - y, y0 - x, color);
    }
  return 1;
	}
int drawCircleFilled(WORD x0, WORD y0, WORD r, int color) {
  int x,y;
	unsigned int largestX = r;
	unsigned long radius2=r*r;

//https://stackoverflow.com/questions/1201200/fast-algorithm-for-drawing-filled-circles
	for(y=0; y <= r; ++y) {
		uint32_t y2=y*y;
    for(x=largestX; x >= 0; --x) {
			if((x * x) + y2 <= radius2) {
        int xn,yn;
        xn=max((int)x0-x,0);     // drawLine si schianta (credo loop infinito) se x negativo (che non può accettare!)
        yn=max((int)y0-y,0);
				drawLine(xn, y0 + y, x0 + x, y0 + y, color);		// VERIFICARE! manca il 4° parm (coglione)
				drawLine(xn, yn, x0 + x, yn, color);
				largestX = x;
				break; // go to next y coordinate
				}
			}
		}

  return 1;
	}


int writeCharAt(WORD x, WORD y, char ch, int c) {
	WORD xc,yc;
	const BYTE *p=&font8x8_basic[(BYTE)ch];
	BYTE cmask;

	for(yc=0; yc<8; yc++,p++) {
		cmask = 0b00000001;
		for(xc=0; xc<8; xc++, cmask <<= 1) {
			if(*p & cmask)
				drawPixel(xc+x,yc+y,c);
			else
				drawPixel(xc+x,yc+y,0);
// gestire colore di sfondo?
			}
		}
	return 1;	
	}

int writeStringAt(WORD x, WORD y, const char *s, int c) {

	while(*s) {
// gestire CR,LF ecc; limiti schermo...
		switch(*s) {
			case CR:
				x=0;
				break;
			case LF:
				y+=8;
				break;
			default:
				writeCharAt(x,y,*s,c);
				x+=8;
				break;
			}
		s++;
		}
	return 1;	
	}

int cwrite(char ch) {
  
  writeCharAt(cursor_x*8,cursor_y*8,ch,1);
  cursor_x++;
  if(cursor_x>=SCREENSIZE_X/8) {
    cursor_x=0;
    cursor_y++;
    if(cursor_y>=SCREENSIZE_Y/8) {    
      
      cursor_y=0;   // finire scroll ecc
      }
    }
  }


int drawImage(const WORD w[]) {
	WORD i;
#define SIZEIMAGE VIDEO_BUFSIZE/2  /* (((320*240/4)/2)/2) */

	for(i=0; i<SIZEIMAGE; i++) {
		videoRAM[i]=w[i];
		ClrWdt();
		}
	
	}

