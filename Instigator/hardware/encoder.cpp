#include "hardware/encoder.h"

#include <avr/io.h>
#include <stdint.h>

static PORT_t &lencoderport = PORTD;
static PORT_t &rencoderport = PORTE;
static const int lencoderpins_mask = 0x03;
static const int rencoderpins_mask = 0x03;

static const int chan0mux = EVSYS_CHMUX_PORTD_PIN0_gc;
static const int chan2mux = EVSYS_CHMUX_PORTE_PIN0_gc; // event channels are used in pairs for qdec

static TC0_t &encodertim0 = TCD0;
static TC1_t &encodertim1 = TCD1;

void encoder_init() {
	PORTCFG.MPCMASK = lencoderpins_mask; // configure all encoder pins
	lencoderport.PIN0CTRL = PORT_ISC_LEVEL_gc | PORT_OPC_PULLUP_gc; // set them to level sensing, required for qdec hardware

	PORTCFG.MPCMASK = rencoderpins_mask;
	rencoderport.PIN0CTRL = PORT_ISC_LEVEL_gc | PORT_OPC_PULLUP_gc;

	EVSYS.CH0MUX = chan0mux; // configure the event channel muxes to the correct pins
	EVSYS.CH2MUX = chan2mux;
	EVSYS.CH0CTRL = EVSYS.CH2CTRL = EVSYS_QDEN_bm | EVSYS_DIGFILT_8SAMPLES_gc; // turn on quadrature decoding, and digital filtering

	encodertim0.CTRLD = TC_EVACT_QDEC_gc | TC_EVSEL_CH0_gc; // set up timers for quadrature decoding from the correct event channels
	encodertim1.CTRLD = TC_EVACT_QDEC_gc | TC_EVSEL_CH2_gc;
	encodertim0.PER = encodertim1.PER = 0xFFFF; // max out the period so we use all 16 bits before overflowing
	encodertim0.CTRLA = encodertim1.CTRLA = TC_CLKSEL_DIV1_gc; // div1 clock selection required for qdec to work
}

uint16_t encoder_get(EncoderNum num) {
	if (num == LEFT_ENCODER)
		return -encodertim0.CNT;
	else
		return -encodertim1.CNT;
}

void encoder_reset(EncoderNum num) {
	if (num == LEFT_ENCODER)
		encodertim0.CNT = 0;
	else
		encodertim1.CNT = 0;
}

int16_t encoder_diff(uint16_t a, uint16_t b) {
	int16_t diff = (int16_t)a - (int16_t)b;
	if (diff > INT16_MAX/2) {	// if encoder wrapped around
		diff -= INT16_MAX;
	} else if (diff < INT16_MIN/2) {
		diff += INT16_MAX;
	}
	return diff;
}
