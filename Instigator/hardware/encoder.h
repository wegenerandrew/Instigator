#ifndef ENCODER_H_
#define ENCODER_H_

#include <stdint.h>

static const float ticks_per_rotation = 1920;
const int encoder_count = 2;

enum ENCODERNum {
	LEFT_ENCODER,
	RIGHT_ENCODER
};

void encoder_init();
void encoder_reset(ENCODERNum num);
void encoder_resetAll();

int16_t encoder_diff(uint16_t a, uint16_t b);

int encoder_get(ENCODERNum num);
void encoder_tick();

#endif
