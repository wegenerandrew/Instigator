#ifndef CONTROLPANEL_H_
#define CONTROLPANEL_H_

#include <stdint.h>
#include "control/pid.h"

void controlpanel_init();

bool controlpanel();

void controlpanel_drive();
void controlpanel_autonomy();
void controlpanel_sensor();
void controlpanel_encoder();
void controlpanel_magnetometer();
void controlpanel_calibrate();
void controlpanel_sonar();

int controlpanel_prompt(const char *prompt, const char *fmt, ...);
char controlpanel_promptChar(const char *prompt);
bool controlpanel_promptGains(const char *name, const PIDGains &curgains, PIDGains &gains);

#endif
