#ifndef ESTOP_H_
#define ESTOP_H_

void estop_init();
void estop_initCheck();
void estop_setLED();
bool estop_check();
void estop_restart();

bool estop_checkPin();
void estop_killall();
void estop_reboot();

#endif

