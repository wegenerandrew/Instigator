#ifndef ESTOP_H_
#define ESTOP_H_

void estop_init();
void estop_setLED();
bool estop_check();
void estop_restart();

void estop_killall();

#endif

