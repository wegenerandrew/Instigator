#include "hardware/solenoid.h"

#include <avr/io.h>
#include <util/delay.h>

static PORT_t &solenoid_port = PORTH;
static const int solenoid_mask = _BV(0);

void solenoid_init() {
	solenoid_port.OUTCLR = solenoid_mask;
	solenoid_port.DIRSET = solenoid_mask;
}

void solenoid_kill() {
	solenoid_port.OUTSET = solenoid_mask;
	_delay_ms(500);
	solenoid_port.OUTCLR = solenoid_mask;
}
