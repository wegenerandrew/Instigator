#include "debug/debug.h"
#include "hardware/sonar.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/pgmspace.h>

static PORT_t &sonar1_port = PORTD;
static PORT_t &sonar2_port = PORTE;
static const int echo1_mask = _BV(4);
static const int echo2_mask = _BV(4);
static const int trigger_mask = _BV(5);

static TC1_t &sonar_timer = TCE1;
#define TIMOVFVEC TCE1_OVF_vect
#define SIGINT0VECTD PORTD_INT0_vect
#define SIGINT0VECTE PORTE_INT0_vect

static const float sonar_scaler = 0.0345578232;

struct SonarData {
	float dist;
	float pulseStart;
};

static SonarData sonarData[sonar_count];

static int l_counter = 0;
static int m_counter = 0;
static int r_counter = 0;
static float l_avg = 0;
static float m_avg = 0;
static float r_avg = 0;
static bool pulsing = false;

static void sonar_trigger() {		// Sends trigger pulse to first sonar
	sonar1_port.OUTSET = trigger_mask;
	_delay_us(20);
	sonar1_port.OUTCLR = trigger_mask;
}

void sonar_init() {
	sonar1_port.DIRCLR = echo1_mask;			// set echo pin as input
	sonar2_port.DIRCLR = echo2_mask;
	sonar1_port.DIRSET = trigger_mask;
	sonar_timer.CTRLA = TC_CLKSEL_DIV64_gc;	// set divider to 64: 32 MHz / 64 = 500 kHz
	sonar_timer.PER   = 40000;				// 500 kHz / 40000 = 12.5 Hz, 1/12.5 Hz = 80 ms to overflow
	sonar_timer.INTCTRLA = TC_OVFINTLVL_LO_gc;	// set overflow interrupt to low level
	sonar1_port.PIN4CTRL = PORT_ISC_BOTHEDGES_gc;		//Set pin interrupts to occur on the both edges
	sonar2_port.PIN4CTRL = PORT_ISC_BOTHEDGES_gc;
	sonar1_port.INT0MASK = echo1_mask;  //Set echo pins in port to be part of an interrupt
	sonar2_port.INT0MASK = echo2_mask;
	sonar1_port.INTCTRL = TC_OVFINTLVL_LO_gc;		// echo set to Medium Priority
	sonar2_port.INTCTRL = TC_OVFINTLVL_LO_gc;
	sonar_trigger();
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

static int16_t sonar_diff(uint16_t a, uint16_t b) {
	int16_t diff = (int16_t)a - (int16_t)b;
	if (diff < 0) {	// if sonar timer wrapped around
		diff  = (sonar_timer.PER - b) + (a);
	}
	return diff;
}

ISR(TIMOVFVEC) {		// Overflow interrupt acts as a watchdog type thing, if sonars stop triggering, will restart a cycle
	if (!pulsing) {		// If we don't have sonars go off for a cycle, trigger again
		sonar_trigger();
	}
	pulsing = false;
}

ISR(SIGINT0VECTD) {
	if ((sonar1_port.IN | 0xEF) == 0xFF) {		// if the echo is high
		sonarData[LEFT_SONAR].pulseStart = sonar_timer.CNT;
		pulsing = true;
	} else {		// echo is done, sonar will auto-trigger next sonar
		sonarData[LEFT_SONAR].dist = sonar_diff(sonar_timer.CNT, sonarData[LEFT_SONAR].pulseStart)*sonar_scaler; // Converts length of pulse to distance in Centimeters
		pulsing = true;
	}
}

ISR(SIGINT0VECTE) {
	if ((sonar2_port.IN | 0xEF) == 0xFF) {		// if the echo is high
		sonarData[FRONT_SONAR].pulseStart = sonar_timer.CNT;
		pulsing = true;
	} else {		// echo is done, so trigger first sonar again
		sonarData[FRONT_SONAR].dist = sonar_diff(sonar_timer.CNT, sonarData[FRONT_SONAR].pulseStart)*sonar_scaler; // Converts length of pulse to distance in Centimeters
		pulsing = true;
		sonar_trigger();
	}
}
