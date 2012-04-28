#include "debug.h"
#include "uart.h"
//#include "adc.h"
#include "hardware/motor.h"
#include "tick.h"
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

// debug timer
static TC1_t &tim = TCC1;

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
	
	tim.CTRLA = TC_CLKSEL_DIV64_gc; // 32Mhz / 64 = .5 Mhz timer
	tim.PER = 0xFFFF; // 1Mhz / 65536 = 65ms

	fdev_setup_stream(&stdinout, put, get, _FDEV_SETUP_RW);
	stdin = &stdinout;
	stdout = &stdinout;
}

void debug_setLED(LED led, bool on) {
	if (on) {
		debugled_port.OUTSET = _BV(led);
	} else {
		debugled_port.OUTCLR = _BV(led);
	}
}

void debug_tick() {
/*	if (adc_getBattery() < 10.5) {
		if (++batterycnt >= 100)
			debug_halt("LOW BATTERY");
	} else {
		batterycnt = 0;
	}*/
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

	bool led=false;
	while (true) {
		printf_P("Halted: %s\n", reason);
		debug_setLED(ERROR_LED, led);
		led = !led;
		_delay_ms(1000);
	}
}
