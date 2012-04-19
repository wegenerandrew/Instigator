#ifndef DEBUG_H_
#define DEBUG_H_

#include <stdint.h>

void debug_init();
void debug_setErrorLED();
void debug_killErrorLED();

void debug_resetTimer();
uint16_t debug_getTimer(); // in us

void debug_println(const char *fmt, ...) __attribute__ ((format (printf, 1, 2)));
void debug_setEchoEnabled(bool enabled);

void debug_halt(const char *reason);

void debug_tick();

#endif
