#ifndef ENCODER_H_
#define ENCODER_H_

#include <stdint.h>

enum EncoderNum {
	LEFT_ENCODER,
	RIGHT_ENCODER
};

static const float ticks_per_rotation = 1920;

void encoder_init();
uint16_t encoder_get(EncoderNum num);
void encoder_reset(EncoderNum num);

int16_t encoder_diff(uint16_t a, uint16_t b);

#endif /* ENCODER_H_ */
