#include "control/solenoidcontrol.h"
#include "hardware/motor.h"
#include <util/delay.h>

void solenoidcontrol_kill() {
	motor_setPWM(MOTOR_SOLENOID, 1000);
	_delay_ms(500);
	motor_killMotor(MOTOR_SOLENOID);
}

