#ifndef DRIVE_H_
#define DRIVE_H_

/*************************
**   All distances in   **
**     Centimeters      **
** All times in Seconds **
*************************/

/**
\addtogroup control
@{
*/

/**
\brief Drive Libraries for simple control of motion.

Drive libraries handle basic movements such as drive forward, drive backwards a distance, turn a set amount of degrees, etc.
*/

const float wheel_circumference = 81.28;		// In centimeters.
const float wheelbase_radius = 52.71;		// In centimeters.

/**
\brief Powers motors at given velocities.
*/
void drive(float lvel, float rvel);

/**
\brief Powers motors at given velocities for given distance.
*/
void drive_dist(float lvel, float rvel, float dist);
void drive_stop();
void drive_off();
void drive_fd(float vel);
void drive_fdDist(float vel, float dist);
void drive_bk(float vel);
void drive_bkDist(float vel, float dist);
void drive_turn(float vel, float deg, bool right);
void drive_lturn(float vel);
void drive_lturnDeg(float vel, float deg);
void drive_rturn(float vel);
void drive_rturnDeg(float vel, float deg);
void drive_steer(float steer, float vel);

#endif
