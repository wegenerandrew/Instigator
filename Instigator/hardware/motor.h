#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdint.h>

enum Motor {
	MOTOR_SOLENOID,
	MOTOR_WEEDWHACKER,
	MOTOR_LEFT,
	MOTOR_RIGHT
};

static const int motor_count = 4;
static const int16_t motor_maxPWM = 1024;

void motor_init();
void motor_ramp(int16_t newPWML, int16_t newPWMR);
void motor_goPWM(int16_t newPWML, int16_t newPWMR);
void motor_setPWM(uint8_t mot, int16_t PWM);
int16_t motor_getPWM(uint8_t mot);
void motor_off(uint8_t mot);
void motor_allOff();
void motor_killMotor(uint8_t mot);
void motor_estop();
void motor_tick();

#endif /* MOTOR_H_ */
