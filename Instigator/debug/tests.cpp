#include "debug/tests.h"
#include "hardware/motor.h"
#include "hardware/encoder.h"

#include <util/delay.h>
#include <stdint.h>
#include <avr/pgmspace.h>
#include <stdio.h>

void tests_feedforward() {
	for (int16_t PWM = 0; PWM < motor_maxPWM; PWM++)
		tests_PWM_single(PWM);

	motor_setPWM(MOTOR_LEFT, 0);
	motor_setPWM(MOTOR_RIGHT, 0);
	_delay_ms(2000);

	for (int16_t PWM = 0; PWM > -motor_maxPWM; PWM--)
		tests_PWM_single(PWM);

	motor_setPWM(MOTOR_LEFT, 0);
	motor_setPWM(MOTOR_RIGHT, 0);
}

void tests_PWM_single(int16_t PWM) {
	encoder_reset((Encoder)MOTOR_LEFT);
	encoder_reset((Encoder)MOTOR_RIGHT);
	motor_setPWM(MOTOR_LEFT, PWM);
	motor_setPWM(MOTOR_RIGHT, PWM);
	_delay_ms(10);
	printf_P(PSTR("%d %d %d\n"), PWM, encoder_get((Encoder)MOTOR_LEFT), encoder_get((Encoder)MOTOR_RIGHT));
}
