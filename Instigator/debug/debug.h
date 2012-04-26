#ifndef DEBUG_H_
#define DEBUG_H_

#include <stdint.h>

enum LED {
	LED_ERROR,
	LED_ESTOP,
	LED_BATTERY,
	LED_OTHER3,
	LED_OTHER4
};

void debug_init();
void debug_setLED(LED led, bool on);

void debug_resetTimer();
uint16_t debug_getTimer(); // in us

void debug_println(const char *fmt, ...) __attribute__ ((format (printf, 1, 2)));
void debug_setEchoEnabled(bool enabled);

void debug_halt(const char *reason);

void debug_tick();

#endif
