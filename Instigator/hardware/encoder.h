#ifndef ENCODER_H_
#define ENCODER_H_

#include <stdint.h>

enum Encoder {
	RIGHT_ENCODER,
	LEFT_ENCODER
};

static const float ticks_per_rotation = 25500;

void encoder_init();
uint16_t encoder_get(Encoder num);
void encoder_reset(Encoder num);

int16_t encoder_diff(uint16_t a, uint16_t b);

#endif /* ENCODER_H_ */
