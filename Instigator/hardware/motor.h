#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdint.h>

enum Motor {
	MOTOR_LEFT,
	MOTOR_RIGHT,
	MOTOR_SOLENOID
};

static const int motor_count = 4;
static const int16_t motor_maxPWM = 1024;

void motor_init();
void motor_setPWM(uint8_t mot, int16_t PWM);
int16_t motor_getPWM(uint8_t mot);
inline void motor_off(uint8_t mot) { motor_setPWM(mot, 0); }
void motor_allOff();
void motor_killMotor(uint8_t mot);
void motor_estop();
void motor_tick();

#endif /* MOTOR_H_ */
