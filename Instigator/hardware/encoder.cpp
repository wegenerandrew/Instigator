#include "hardware/encoder.h"
#include "debug/debug.h"
#include "util.h"
#include "hardware/estop.h"
#include "control/odometry.h"

#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdint.h>

// Encoder Pin Masks
static const int cspin_mask = _BV(0);
static const int clkpin_mask = _BV(1);
static const int datapin_mask = _BV(2);
static PORT_t encoder_port;

// lEncoder
static PORT_t &lEncoder_port = PORTD;

// rEncoder
static PORT_t &rEncoder_port = PORTE;

static int16_t encoderVal[2] = {0x00, 0x00};
static int16_t previousVal[2] = {0x00, 0x00};
static int16_t encoderTemp[2] = {0x00, 0x00};
static int16_t encoderOffset[2] = {0x00, 0x00};

void encoder_init() {
	lEncoder_port.OUTSET = cspin_mask;	// set chip selects to high so both encoders are disabled
	rEncoder_port.OUTSET = cspin_mask;
	lEncoder_port.OUTSET = clkpin_mask;	// Set clock pins to high for both encoders
	rEncoder_port.OUTSET = clkpin_mask;
	lEncoder_port.DIRCLR = datapin_mask;	// Set data pins to input
	rEncoder_port.DIRCLR = datapin_mask;
	lEncoder_port.DIRSET = cspin_mask;		// Set both chip select pins as output
	rEncoder_port.DIRSET = cspin_mask;
	lEncoder_port.DIRSET = clkpin_mask;	// Set both clock pins as output
	rEncoder_port.DIRSET = clkpin_mask;
	lEncoder_port.DIRSET = 0xFF;
	lEncoder_port.OUTSET = 0xFF;
}

int encoder_get(ENCODERNum num) {		// Gives value encoder is at minus offset (so you can reset encoders to 0)
	return encoderVal[num] - encoderOffset[num];
}

void encoder_reset(ENCODERNum num) {	// Sets offset to current encoder position, encoder_get() will then return 0 at this position
	encoderOffset[num] = encoderVal[num];
}

void encoder_resetAll() {				// does encoder_reset() but for both encoders
	for (int i = 0; i < encoder_count; i++) {
		encoderOffset[i] = encoderVal[i];
	}
}

int16_t encoder_diff(uint16_t a, uint16_t b) {		// Returns difference between two positions of encoders, deals with wraparound
	int16_t diff = (int16_t)a - (int16_t)b;
	if (diff > INT16_MAX/2) {	// if encoder wrapped around
		diff -= INT16_MAX;
	} else if (diff < INT16_MIN/2) {
		diff += INT16_MAX;
	}
	return diff;
}

static void receive(ENCODERNum num) {
	previousVal[num] = encoderVal[num];
	encoderTemp[num] = 0x00;		// Reset temp for new reading
	if (num == LEFT_ENCODER) {		// Set up which encoder port we're working with
		PORT_t &encoder_port = lEncoder_port;
	} else {
		PORT_t &encoder_port = rEncoder_port;
	}
	encoder_port.OUTCLR = cspin_mask;	// Set chipselect low to enable encoder
	_delay_us(1);		// min 500 ns
	encoder_port.OUTCLR = clkpin_mask;	// Set clock low
	for (int i = 0; i < 11; i++) {		// up, down, read, repeat
		_delay_us(1);
		encoder_port.OUTSET = clkpin_mask;	// up
		_delay_us(1);
		encoder_port.OUTCLR = clkpin_mask;	// down
		encoderTemp[num] = encoderTemp[num] | ((encoder_port.IN >> datapin_mask) & 0x01);	// read
		encoderTemp[num] = encoderTemp[num] << 1;		// get ready for next bit
	}	// Repeat
	_delay_us(1);
	encoder_port.OUTSET = clkpin_mask;
	_delay_us(1);
	encoderVal[num] = encoderTemp[num] | ((encoder_port.IN >> datapin_mask) & 0x01);	// final read
	encoder_port.OUTSET = cspin_mask;	// Deselect encoder by setting chip select high
}

void encoder_tick() {
//	receive(LEFT_ENCODER);
//	receive(RIGHT_ENCODER);
//	odometry_update(encoder_diff(encoderVal[LEFT_ENCODER], previousVal[LEFT_ENCODER]), encoder_diff(encoderVal[RIGHT_ENCODER], previousVal[RIGHT_ENCODER]));
}
