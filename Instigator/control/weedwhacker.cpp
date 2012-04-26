#include "control/weedwhacker.h"
#include "hardware/motor.h"

void weedwhacker_start() {
	motor_startWhacker(2000);
}

void weedwhacker_kill() {
	motor_killMotor(MOTOR_WEEDWHACKER);
}
