#include "control/tick.h"
#include "debug/debug.h"
#include "debug/uart.h"
#include "hardware/adc.h"
#include "hardware/motor.h"
#include "util.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdint.h>

static PORT_t &debugled_port = PORTJ;
static const int debugled_mask = _BV(0) | _BV(1) | _BV(2) | _BV(3) | _BV(4);

static PORT_t &buzzer_port = PORTH;
static const int buzzer_mask = _BV(7);

static bool buzzerEnable = false;
static int buzzerCounter = 0;
static int buzzer_beepCount = 0;
static int buzzer_beepDesired = 0;
static int buzzer_lengthCount = 0;
static int buzzer_lengthDesired = 0;

BuzzerFunction buzzerFunction;

// debug timer
static TC1_t &tim = TCC1;
#define TIMOVFVEC TCC1_OVF_vect

// output flags
static bool echo_enabled = true;

// battery monitor
static uint8_t batterycnt;

// stdio stuff
static int put(char ch, FILE* file);
static int get(FILE* file);
static FILE stdinout;

void debug_init() {
	debugled_port.DIRSET = debugled_mask;
	buzzer_port.OUTCLR = buzzer_mask;
	buzzer_port.DIRSET = buzzer_mask;
	
	tim.CTRLA = TC_CLKSEL_DIV64_gc; // 32Mhz / 64 = .5 Mhz timer
	tim.PER = 0xFFFF; // .5Mhz / 65536 = 131ms
	tim.INTCTRLA = TC_OVFINTLVL_LO_gc;	// set overflow interrupt to low level

	fdev_setup_stream(&stdinout, put, get, _FDEV_SETUP_RW);
	stdin = &stdinout;
	stdout = &stdinout;
}

void debug_initCheck() {
/*	if ((RST.STATUS & RST_BORF_bm) == RST_BORF_bm) {
		printf_P(PSTR("Reset caused by Brownout!!\n"));
//		RST.STATUS = RST_BORF_bm;	// Clear flag
	}
	if ((RST.STATUS & RST_SRF_bm) == RST_SRF_bm) {
		printf_P(PSTR("Reset caused by Software Reset.\n"));
//		RST.STATUS = RST_SRF_bm;	// Clear flag
	}*/
/*	if ((RST.STATUS & RST_EXTRF_bm) == RST_EXTRF_bm) {
		printf_P(PSTR("Board was Externally Reset.\n"));
//		RST.STATUS = RST_EXTRF_bm;	// Clear flag
	}
	if ((RST.STATUS & RST_PORF_bm) == RST_PORF_bm) {
		printf_P(PSTR("Board was Power Cycled.\n"));
//		RST.STATUS = RST_PORF_bm;	// Clear flag
	}*/
}

void debug_setLED(LED led, bool on) {
	if (on) {
		debugled_port.OUTSET = _BV(led);
	} else {
		debugled_port.OUTCLR = _BV(led);
	}
}

static void debug_setBuzzer(bool on) {
	if (on) {
		buzzer_port.OUTSET = buzzer_mask;
	} else {
		buzzer_port.OUTCLR = buzzer_mask;
	}
}

static void debug_setBuzzerEnable(bool new_enable) {
	if (buzzerEnable) {
		debug_setBuzzer(false);
	}
	buzzerEnable = new_enable;
}

void debug_buzzerBeep(int new_buzzer_beepCount) {
	buzzer_beepCount = 0;
	buzzerFunction = BEEP_BUZZER;
	buzzer_beepDesired = new_buzzer_beepCount;
	debug_setBuzzerEnable(true);
}

void debug_buzzerSolid(int new_buzzerLength) {
	buzzer_lengthCount = 0;
	buzzerFunction = SOLID_BUZZER;
	buzzer_lengthDesired = new_buzzerLength;
	debug_setBuzzerEnable(true);
}

void debug_tick() {
	if (adc_sampleBattery() < 10.5) {
		batterycnt++;
		if (batterycnt >= 100) {
			debug_halt("LOW BATTERY");
		}
	} else {
		batterycnt = 0;
	}
}

static int put(char ch, FILE* file) {
	if(ch == '\n')
		put('\r', file);

	while (!uart_put(UART_USB, ch)) { }
	while (!uart_put(UART_XBEE, ch)) { }

	return 1;
}

static int get(FILE* file) {
	int ch;
	do {
		ch = uart_get(UART_USB);
		if (ch == -1)
			ch = uart_get(UART_XBEE);
	} while (ch == -1);

	if (ch == '\r')
		ch = '\n';

	if (echo_enabled)
		put(ch, NULL); // echo character

	return ch;
}

void debug_resetTimer() {
	tim.CNT = 0;
}

uint16_t debug_getTimer() {
	return tim.CNT * 2;
}

void debug_println(const char *fmt, ...) {
	va_list ap;
	va_start(ap, fmt);

	char buf[64];
	unsigned int amt = vsnprintf(buf, sizeof(buf), fmt, ap);
	if (amt > sizeof(buf)-4)
		amt = sizeof(buf)-4;
	buf[amt++] = '\r';
	buf[amt++] = '\n';
	buf[amt++] = 0;

	uart_puts(UART_USB, buf); // no resending logic, we drop bytes if buffer fills up

	va_end(ap);
}

void debug_setEchoEnabled(bool enabled) {
	echo_enabled = enabled;
}

void debug_halt(const char *reason) {
	tick_suspend();
	motor_allOff();

	debug_buzzerBeep(10);

	bool led=false;
	while (true) {
		printf_P(PSTR("Halted: %s\n"), reason);
		debug_setLED(ERROR_LED, led);
		led = !led;
		_delay_ms(1000);
	}
}

ISR(TIMOVFVEC) {
	if (buzzerCounter == 5) {	// 5*131ms = 655ms
		if (buzzerEnable) {
			if (buzzerFunction == BEEP_BUZZER) {
				debug_setBuzzer(true);
				buzzer_beepCount++;
			} else if (buzzerFunction == SOLID_BUZZER) {
				debug_setBuzzer(true);
			}
		}
	} else if (buzzerCounter >= 10) {
		if (buzzerEnable) {
			if (buzzerFunction == BEEP_BUZZER) {
				debug_setBuzzer(false);
				if (buzzer_beepCount >= buzzer_beepDesired) {
					debug_setBuzzerEnable(false);
				}
			} else if (buzzerFunction == SOLID_BUZZER) {
				buzzer_lengthCount++;
				if (buzzer_lengthCount >= buzzer_lengthDesired) {
					debug_setBuzzerEnable(false);
				}
			}
		}
		buzzerCounter = 0;
	}
	buzzerCounter++;
}
