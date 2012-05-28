#include "debug/debug.h"
#include "util.h"
#include "llautm2.h"
#include "hardware/estop.h"
#include "hardware/gps.h"
#include "debug/uart.h"
#include "control/odometry.h"
#include "control/magfollow.h"

#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/pgmspace.h>
#include <math.h>

// gps uart
static PORT_t &uartport_gps = PORTD;
#define RXVEC_GPS USARTD1_RXC_vect
#define TXVEC_GPS USARTD1_DRE_vect
static USART_t &uart_gps = USARTD1;
static const int bsel_gps = 3333;
static const int bscale_gps = 0xC;
static const int txpin_gps = 7;
static const int rxpin_gps = 6;

volatile int iNumOfCommas = 0;
volatile int index = 0;
volatile char buffer = 0;
char DataIn[41];
volatile int NewPacket = 0;
double fLatitude = 0;
double fLongitude = 0;

char *ptrLongitude   = &DataIn[15];
char *ptrLatitude    = &DataIn[2];
char *ptrLat_Std_Dev = &DataIn[28];
char *ptrLon_Std_Dev = &DataIn[33];
int ptrSats		 	 = 1;
int ptrSolStatus	 = 38;
int ptrRt20Status	 = 39;
int ptrFixStatus	 = 40;

utmCoordinates utmCoordinatesOut;
double X_Coord_Dec, Y_Coord_Dec, X_utm, Y_utm;
float x_offset = 0;
float y_offset = 0;

bool enabled = false;	//Enabled will decide whether or not we update odometry with our new gps position (set in initial calibration)

GPSHealth gpsHealth;
GPSPos gpsPos;

struct UARTData {
	char outbuf[64];
	volatile uint8_t outbuf_pos;
	char inbuf[8];
	volatile uint8_t inbuf_pos;
};

static UARTData gpsdata;
static USART_t *const uarts = &USARTD1;

// Startup Commands
static const char gps_fullReset[] = "CRESET\n";
static const char gps_rt20Reset[] = "RESETRT20\n";
static const char gps_com1[]	  = "COM1 9600,n,8,1,n,off\n";
static const char gps_com2[]	  = "COM2 9600,n,8,1,n,off\n";
static const char gps_rtcm[]	  = "RTCMRULE 6CR\n";
static const char gps_comIn[]	  = "ACCEPT com2,rt20\n";
static const char gps_comOut[]	  = "LOG com1,P20A,ontime,10\n";
static const char gps_save[]	  = "SAVECONFIG\n";

void gps_init() {
	uartport_gps.OUTSET = _BV(txpin_gps);
	uartport_gps.DIRSET = _BV(txpin_gps);

	uart_gps.CTRLA = USART_RXCINTLVL_LO_gc;	// Set to low priority
	uart_gps.CTRLB = USART_RXEN_bm | USART_TXEN_bm;
	uart_gps.CTRLC = USART_CHSIZE_8BIT_gc;
	uart_gps.BAUDCTRLA = bsel_gps & 0xFF;
	uart_gps.BAUDCTRLB = (bscale_gps << USART_BSCALE_gp) | (bsel_gps >> 8);

	// Send startup commands to GPS (may not have to do every time?)
/*	gps_puts(&gps_fullReset[0]);
	_delay_ms(10);					// Delays are just to be sure we have plenty of time to clock everything out without filling up the buffer
	gps_puts(&gps_rt20Reset[0]);
	_delay_ms(10);
	gps_puts(&gps_com1[0]);
	_delay_ms(10);
	gps_puts(&gps_com2[0]);
	_delay_ms(10);
	gps_puts(&gps_rtcm[0]);
	_delay_ms(10);
	gps_puts(&gps_comIn[0]);
	_delay_ms(10);
	gps_puts(&gps_comOut[0]);
	_delay_ms(10);
	gps_puts(&gps_save[0]);*/
}

void gps_setEnabled(bool new_enabled) {
	enabled = new_enabled;
}

bool gps_getEnabled() {
	return enabled;
}

bool gps_put(char ch) {
	UARTData &data = gpsdata;
	USART_t &usart = *uarts;

	if (data.outbuf_pos >= sizeof(data.outbuf))
		return false;

	usart.CTRLA &= ~USART_DREINTLVL_gm;
	data.outbuf[data.outbuf_pos++] = ch;
	usart.CTRLA |= USART_DREINTLVL_LO_gc; // enable transmit interrupt at low priority
	return true;
}

int gps_puts(const char *buf) {
	int ctr=0;
	while (*buf) {
		if (!gps_put(*buf++))
			break;
		ctr++;
	}
	return ctr;
}

int gps_get() {
	UARTData &data = gpsdata;
	USART_t &usart = *uarts;

	if (data.inbuf_pos == 0)
		return -1;

	usart.CTRLA &= ~USART_RXCINTLVL_gm;
	char ch = data.inbuf[0];
	data.inbuf_pos--;
	memmove(data.inbuf, data.inbuf+1, data.inbuf_pos);
	usart.CTRLA |= USART_RXCINTLVL_gm;

	return ch;
}

void gps_putch(char ch) {
	UARTData &data = gpsdata;

	if (data.inbuf_pos >= sizeof(data.inbuf))
		return;

	data.inbuf[data.inbuf_pos++] = ch;	
}

void gps_tick() {
	if(NewPacket) {									// Has a new GPS packet been recieved?
		NewPacket = 0;

		/*DataIn[14] = ',';
	
		DataIn[27] = ',';
		
		DataIn[32] = ',';
		
		DataIn[37] = ',';*/

		gpsHealth.satellites = DataIn[ptrSats] - 48;		// Convert all these into single digit ints from ascii chars
		gpsHealth.solStatus  = DataIn[ptrSolStatus] - 48;
		gpsHealth.rt20Status = DataIn[ptrRt20Status] - 48;
		gpsHealth.fixStatus  = DataIn[ptrFixStatus] - 48;

		if (gpsHealth.solStatus == 6) {								// Booting from a cold start
			printf_P(PSTR("GPS not yet converged from cold start?\n"));
		}
/*		if (gpsHealth.rt20Status == 5 || gpsHealth.rt20Status == 6 || gpsHealth.rt20Status == 7) {	// Requires Reset
			gps_puts(&gps_rt20Reset[0]);
		}*/
		if (gpsHealth.solStatus == 0 && (gpsHealth.rt20Status == 0 || gpsHealth.rt20Status == 1) && gpsHealth.fixStatus == 2/* || gpsHealth.fixStatus == 1*/) {		// GPS data can be used to update odometry
			if (!gpsHealth.valid) {
				debug_buzzerBeep(2);
			}
			gpsHealth.valid = true;
			fLongitude  = atof(ptrLongitude);						// Convert String into double
			fLatitude   = atof(ptrLatitude);						// Convert String into double

			gpsHealth.Lat_Std_Dev	= (float)(atof(ptrLat_Std_Dev));				// Convert String into float
			gpsHealth.Lon_Std_Dev	= (float)(atof(ptrLon_Std_Dev));				// Convert String into float

			utmCoordinatesOut = *LLA_UTM2(fLatitude, fLongitude);	// Convert Lat/Lon to UTM

			X_utm = (int)utmCoordinatesOut.xCoordinate%100;			// We only want up to tens place (field no more than 99 meters)
			Y_utm = (int)utmCoordinatesOut.yCoordinate%100;			// We only want up to tens place (field no more than 99 meters)
			X_Coord_Dec = utmCoordinatesOut.xCoordinate - (int)utmCoordinatesOut.xCoordinate;   // Now get only the decimal places
			Y_Coord_Dec = utmCoordinatesOut.yCoordinate - (int)utmCoordinatesOut.yCoordinate;   // Now get only the decimal places

			float raw_heading = anglewrap(magfollow_getRawHeading() + M_PI/2);
			X_utm = X_utm + X_Coord_Dec;	// Combine the two previous to get UTM in ##.###### format
			Y_utm = Y_utm + Y_Coord_Dec;	// Combine the two previous to get UTM in ##.###### format
			gpsPos.X_Raw = X_utm;			// Save raw values so that we can calibrate out GPS to 0
			gpsPos.Y_Raw = Y_utm;
			X_utm = X_utm - x_offset - gps_base_offset*cos(raw_heading);
			Y_utm = Y_utm - y_offset - gps_base_offset*sin(raw_heading);

			float heading_offset = anglewrap(magfollow_getOffset());
			gpsPos.X_Actual = (float)(X_utm * cos(heading_offset) - Y_utm * sin(heading_offset));		// Transform x
			gpsPos.Y_Actual = (float)(X_utm * sin(heading_offset) + Y_utm * cos(heading_offset));		// Transform y
			if (enabled) {
				odometry_setPos(mtocm(gpsPos.X_Actual), mtocm(gpsPos.Y_Actual));
			}
		} else {
			if (gpsHealth.valid) {
				debug_buzzerSolid(5);
			}
			gpsHealth.valid = false;
		}
	}
}

GPSHealth gps_getHealth() {
	return gpsHealth;
}

GPSPos gps_getPos() {
	return gpsPos;
}

void gps_setOffset(float x, float y) {
	x_offset = x;
	y_offset = y;
}

void gps_printRaw() {
	for (int i = 0; i < 41; i++) {
		printf("%c", DataIn[i]);
	}
	printf("\n");
}

void gps_printStatus() {
	printf("sats: %d, sol: %d, rt: %d, fix: %d\n", gpsHealth.satellites, gpsHealth.solStatus, gpsHealth.rt20Status, gpsHealth.fixStatus);
}

void gps_printPos() {
	printf("x: %f, y: %f\n", gpsPos.X_Actual, gpsPos.Y_Actual);
}

void gps_printOffset() {
	printf_P(PSTR("X: %f, Y: %f\n"), x_offset, y_offset);
}

// GCC doesn't want to inline these, but its a huge win because num is eliminated and
// all the memory addresses get calculated at compile time.
#pragma GCC optimize("3")
static void receive() __attribute__((always_inline));
static void transmit() __attribute__((always_inline));

static void receive() {
	UARTData &data = gpsdata;
	uint8_t byte = uarts->DATA;

	unsigned char temp;
	unsigned int c;
	temp = byte;

	if (index > 40)
		index = 0;

	if (temp == '$') {
			if (buffer < 4)
				buffer++;
			iNumOfCommas = 0;
			index = 0;
			for (c = 0; c <= 40; c++);
				DataIn[c] = '0';
	}
	if (buffer >= 3) {
		if (temp == ',') {
		 	iNumOfCommas++;

		 	switch (iNumOfCommas) {
			  case 4: //parse # of sattelites
			  	   DataIn[0] = '0';
			  	   index = 1;//this case the index will be decrememented rather than incremeneted
				   break;
			  case 5://parse latitude	 		
				   index = 2;
				   break;
			  case 6: //parse longitude 			
				   index = 15;
				   break; 
			  case 10://parse latitude std dev
				   index = 28;
				   break;
			  case 11: //parse longitude std dev
				   index = 33;
				   break;
			  case 13:	//parse solution status
			  	   index = 38;
				   break;
			  case 14: //parse rt20 status
				   index = 39;
				   break;
			  case 15: //parse rt20 status
				   index = 40;
				   break;
			}
		} else {
			// LCD_8bit_Out(index); 
			 switch (iNumOfCommas) {
				  case 4: //parse # of sattelites
				  	if (index <= 1) {
						DataIn[index] = temp;
						index--;
					}
					break;		
				  case 5:	//parse latitude
				  	if (index <= 14) {
						DataIn[index] = temp;
						index++;
					}
					break;
				  case 6:	//parse longitude
					if (index <= 27) {
						DataIn[index] = temp;
						index++;
					}
					break;
				  case 10:	//parse latitude std dev
					if (index <= 32) {
						DataIn[index] = temp;
						index++;
				  	}
					break;
				  case 11:	 //parse longitude std dev
					if (index <= 37) {
						DataIn[index] = temp;
						index++;
				  	}
					break;
				  case 13:	//parse solution status
				  	DataIn[index] = temp;
					break;
				  case 14:	//parse rt20 status
					DataIn[index] = temp;
					break;
				  case 15:	//parse fix status
					DataIn[index] = temp;
					NewPacket = 1;
					break;
				}
			} // End Else
	   	}

	if (data.inbuf_pos >= sizeof(data.inbuf)) {
		return;
	}

	data.inbuf[data.inbuf_pos++] = byte;
}

static void transmit() {
	UARTData &data = gpsdata;

	if (data.outbuf_pos > 0) {
		uarts->DATA = data.outbuf[0];
		data.outbuf_pos--;

		if (data.outbuf_pos > 0)
			memmove(data.outbuf, data.outbuf+1, data.outbuf_pos);
	} else {
		uarts->CTRLA &= ~USART_DREINTLVL_gm; // disable transmit interrupt
	}
}

ISR(TXVEC_GPS) {
	transmit();
}

ISR(RXVEC_GPS) {
	receive();
}
