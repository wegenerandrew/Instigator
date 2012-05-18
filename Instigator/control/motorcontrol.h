#ifndef MOTORCONTROL_H_
#define MOTORCONTROL_H_

#include "hardware/motor.h"
#include "control/pid.h"

static const int motorcontrol_count = 2;	// Number of motors under control of motorcontrol

void motorcontrol_init();

float motorcontrol_getRPS(Motor motnum);
float motorcontrol_getRPSDesired(Motor motnum);
void motorcontrol_setRPS(Motor motnum, float RPS);
inline void motorcontrol_stop(Motor mot) { motorcontrol_setRPS(mot, 0); }

void motorcontrol_setEnabled(bool enabled=true);
void motorcontrol_setGains(const PIDGains &newpidgains);
PIDGains motorcontrol_getGains();
void motorcontrol_setDebug(bool debug);

void motorcontrol_tick();

#endif
