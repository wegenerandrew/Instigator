#ifndef CONTROLPANEL_H_
#define CONTROLPANEL_H_

#include <stdint.h>

void controlpanel_init();

bool controlpanel();

void controlpanel_drive();
void controlpanel_autonomy();
void controlpanel_sensor();
void controlpanel_encoder();
void controlpanel_magnetometer();

int controlpanel_prompt(const char *prompt, const char *fmt, ...);
char controlpanel_promptChar(const char *prompt);

#endif
