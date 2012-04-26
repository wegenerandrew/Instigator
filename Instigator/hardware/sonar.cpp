#include "debug/debug.h"
#include "hardware/sonar.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>

static PORT_t &sonar1_port = PORTD;
static PORT_t &sonar2_port = PORTE;
static PORT_t &sonar3_port = PORTF;
static const int echo1_mask = _BV(4);
static const int echo2_mask = _BV(4);
static const int echo3_mask = _BV(4);
static const int trigger1_mask = _BV(5);
static const int trigger2_mask = _BV(5);
static const int trigger3_mask = _BV(5);

static TC1_t &sonar_timer = TCE1;
#define TIMOVFVEC TCE1_OVF_vect
#define SIGINT0VECT PORTD_INT0_vect
#define SIGINT0VECT PORTE_INT0_vect
#define SIGINT0VECT PORTF_INT0_vect

volatile bool rising = true;

struct SonarData {
	float dist;
	float pulseStart;
};

static SonarData sonarData[sonar_count];

volatile Sonar sonar = LEFT_SONAR;

void sonar_init() {
	sonar1_port.DIRSET = trigger1_mask;		// set trigger pin as output
	sonar2_port.DIRSET = trigger2_mask;
	sonar3_port.DIRSET = trigger3_mask;
	sonar1_port.DIRCLR = echo1_mask;			// set echo pin as input
	sonar2_port.DIRCLR = echo2_mask;
	sonar3_port.DIRCLR = echo3_mask;
	sonar_timer.CTRLA = TC_CLKSEL_DIV64_gc;	// set divider to 64: 32 MHz / 64 = 500 kHz
	sonar_timer.PER   = 40000;				// 500 kHz / 40000 = 12.5 Hz, 1/12.5 Hz = 80 ms to overflow
	sonar_timer.INTCTRLA = TC_OVFINTLVL_LO_gc;	// set overflow interrupt to low level
	sonar1_port.PIN4CTRL = PORT_ISC_BOTHEDGES_gc;		//Set pin interrupts to occur on the both edges
	sonar2_port.PIN4CTRL = PORT_ISC_BOTHEDGES_gc;
	sonar3_port.PIN4CTRL = PORT_ISC_BOTHEDGES_gc;
	sonar1_port.INT0MASK = echo1_mask;  //Set echo pins in port to be part of an interrupt
	sonar2_port.INT0MASK = echo2_mask;
	sonar3_port.INT0MASK = echo3_mask;
	sonar1_port.INTCTRL = TC_OVFINTLVL_MED_gc;		// EStop set to Medium Priority
	sonar2_port.INTCTRL = TC_OVFINTLVL_MED_gc;
	sonar3_port.INTCTRL = TC_OVFINTLVL_MED_gc;
}

float sonar_getDist(Sonar sonar) {
	return sonarData[sonar].dist;
}

float sonar_getTimer() {
	return sonar_timer.CNT;
}

float sonar_getStart(Sonar sonar) {
	return sonarData[sonar].pulseStart;
}

ISR(TIMOVFVEC) {		// Sends trigger pulse for sonar to transmit
	if (sonar == 0) {
		sonar1_port.OUTSET = _BV(4);			// set trigger high
		_delay_us(20);
		sonar1_port.OUTCLR = _BV(4);			// pull trigger back low
	} else if (sonar == 1) {
		sonar2_port.OUTSET = _BV(4);			// set trigger high
		_delay_us(20);
		sonar2_port.OUTCLR = _BV(4);			// pull trigger back low
	} else if (sonar == 2) {
		sonar3_port.OUTSET = _BV(4);			// set trigger high
		_delay_us(20);
		sonar3_port.OUTCLR = _BV(4);			// pull trigger back low
	}
	if (sonar == LEFT_SONAR) {
		sonar = MIDDLE_SONAR;
	} else if (sonar == MIDDLE_SONAR) {
		sonar = RIGHT_SONAR;
	} else {
		sonar = LEFT_SONAR;
	}
	rising = true;
}

ISR(SIGINT0VECT){		// Interrupt for rising/falling edges of pulse returned by sonar
	if (sonar1_port.INTFLAGS == 0x01) {
		sonar1_port.INTFLAGS = 0x01; //Clear the flag by writing a one to it
	} else if (sonar2_port.INTFLAGS == 0x01) {
		sonar2_port.INTFLAGS = 0x01;
	} else if (sonar3_port.INTFLAGS == 0x01) {
		sonar3_port.INTFLAGS = 0x01;
	}
	if (rising) {
		sonarData[sonar].pulseStart = sonar_timer.CNT;
		rising = false;
	} else {
		sonarData[sonar].dist = (sonar_timer.CNT - sonarData[sonar].pulseStart)/29; // Converts length of pulse to distance in Centimeters
		rising = true;
	}
}
