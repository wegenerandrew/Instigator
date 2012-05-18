#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdint.h>

enum Motor {
	MOTOR_RIGHT,
	MOTOR_LEFT
};

static const int motor_count = 2;
static const int16_t motor_maxPWM = 1024;

void motor_init();

void motor_setPWM(Motor mot, int16_t PWM);
void motor_goPWM(int16_t newPWML, int16_t newPWMR);
int16_t motor_getPWM(Motor mot);

void motor_ramp(int16_t newPWML, int16_t newPWMR);
void motor_rampMotor(Motor mot, int16_t newPWM);

void motor_off(Motor mot);
void motor_allOff();
void motor_killMotor(Motor mot);
void motor_estop();

void motor_tick();

#endif /* MOTOR_H_ */
