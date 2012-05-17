#ifndef DEBUG_H_
#define DEBUG_H_

#include <stdint.h>

enum LED {
	OTHER2_LED,		// Blue
	OTHER3_LED,		// Green
	TICK_LED,		// Blue
	ESTOP_LED,		// Orange
	ERROR_LED		// Red
};

void debug_init();
void debug_initCheck();

void debug_setLED(LED led, bool on);

void debug_resetTimer();
uint16_t debug_getTimer(); // in us

void debug_println(const char *fmt, ...) __attribute__ ((format (printf, 1, 2)));
void debug_setEchoEnabled(bool enabled);

void debug_halt(const char *reason);

void debug_tick();

#endif
