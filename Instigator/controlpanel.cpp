#include "controlpanel.h"
#include "control/drive.h"
#include "control/magfollow.h"
#include "debug.h"
#include "hardware/motor.h"
#include "hardware/mag.h"
#include "hardware/encoder.h"
#include "tick.h"
#include "util.h"
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <math.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

static const char unknown_str[] PROGMEM = "Unknown. ? for help.";
static const float turn_diff = 100;

static float speed = 700;


void controlpanel_init() {
	printf_P(PSTR("Starting up\n"));
}

bool controlpanel() {
	while (true) {
		switch (controlpanel_promptChar("Main")) {
			case 'd':
				controlpanel_drive();
				break;
			case 's':
				controlpanel_sensor();
				break;
			case 'a':
				controlpanel_autonomy();
				break;
			default:
				puts_P(unknown_str);
				break;
			case '?':
				static const char msg[] PROGMEM =
					"Control Panels:\n"
					"  d - Drive"
					"  s - Sensors"
					"  a - Autonomy"
					"  q - Quit";
				puts_P(msg);
				break;
		}
	}
}

void controlpanel_autonomy() {
	while (true) {
		char ch = controlpanel_promptChar("Autonomy");
		switch (ch) {
			case 'H': {			// Sets current heading of robot to prompted heading from user
				float newheading;
				controlpanel_prompt("Heading (deg)", "%f", &newheading);
				magfollow_setHeading(degtorad(newheading));
				break;
			}
			case 'M': {			// Prints out magnetometer calibrated heading
				float heading = magfollow_getHeading();
				heading = radtodeg(heading);
				printf_P(PSTR("Mag Heading (deg): %f\n"), heading);
				break;
			}
			case 'g': {
				float dir;
				controlpanel_prompt("Heading (deg)", "%f", &dir);
				magfollow_start(speed, degtorad(dir));
				break;
			}
			case ' ':
				magfollow_stop();
				break;
			case 'q':
				return;
			case '?':
				static const char msg[] PROGMEM =
					"Control Panels:\n"
					"  H - Set Heading\n"
					"  M - Current Heading\n"
					"  g - Magfollow\n"
					" ' '- Stop\n"
					"  q - Quit";
				puts_P(msg);
				break;
		}
	}
}
				

void controlpanel_drive() {
	while (true) {
		char ch=controlpanel_promptChar("Drive");

		switch (ch) {
			case ' ':
				motor_ramp(0, 0);
				//drive_stop();
				break;
			case 'x':
				//drive_stop(DM_TRAJ);
				break;
			case 'w':
				motor_ramp(speed, speed);
				//drive_fd(speed);
				break;
			case 'a':
				motor_ramp(-(speed - turn_diff), (speed - turn_diff));
				//drive_lturn(speed);
				break;
			case 's':
				motor_ramp(-speed, -speed);
				//drive_bk(speed);
				break;
			case 'd':
				motor_ramp((speed - turn_diff), -(speed - turn_diff));
				//drive_rturn(speed);
				break;
			case 'W':
				motor_setPWM(MOTOR_LEFT, speed);
				motor_setPWM(MOTOR_RIGHT, speed);
				//drive_fdDist(speed, 50);
				break;
			case 'A':
				motor_setPWM(MOTOR_LEFT, -(speed - turn_diff));
				motor_setPWM(MOTOR_RIGHT, (speed - turn_diff));
				//drive_lturnDeg(speed, 90);
				break;
			case 'S':
				motor_setPWM(MOTOR_LEFT, -speed);
				motor_setPWM(MOTOR_RIGHT, -speed);
				//drive_bkDist(speed, 50);
				break;
			case 'D':
				motor_setPWM(MOTOR_LEFT, (speed - turn_diff));
				motor_setPWM(MOTOR_RIGHT, -(speed - turn_diff));
				//drive_rturnDeg(speed, 90);
				break;
			case '=':
				speed += 100;
				printf_P(PSTR("Speed: %f\n"), speed);
				break;
			case '-':
				speed -= 100;
				printf_P(PSTR("Speed: %f\n"), speed);
				break;
			case '+':
				speed += 10;
				printf_P(PSTR("Speed: %f\n"), speed);
				break;
			case '_':
				speed -= 10;
				printf_P(PSTR("Speed: %f\n"), speed);
				break;

			case 'p':
				//motorcontrol_setDebug(false);
				printf_P(PSTR("Debug disabled\n"));
				break;

			case 'c':
				//motorcontrol_setEnabled(false);
				printf_P(PSTR("Motor control disabled\n"));
				break;

			case 'P':
				//motorcontrol_setDebug(true);
				break;

			case 'q':	
				//motorcontrol_setEnabled(false);
				return;

			case 'm': {
				/*float amax = drive_getTrajAmax();
				printf_P(PSTR("Current amax: %.2f\n"), amax);
				if (controlpanel_prompt("amax", "%f", &amax) != 1) {
					printf_P(PSTR("Cancelled.\n"));
					continue;
				}

				drive_setTrajAmax(amax);
				printf_P(PSTR("Amax set to: %.2f\n"), amax);*/
				break;
			}

			default:
				puts_P(unknown_str);
				//drive_stop();
				break;
			case '?':
				static const char msg[] PROGMEM =
					"Drive commands:\n"
					"  wasd  - Control robot\n"
					"  space - Stop\n"
					"  -=_+  - Adjust speed\n"
					"  WASD  - Full speed movements\n"
					"  c	 - Disable motor control\n"
					"  Pp	 - Enable/Disable motor control debug\n"
					"  q	 - Back";
				puts_P(msg);
				break;
		}
	}
}

void controlpanel_sensor() {
	while (true) {
		char ch = controlpanel_promptChar("Sensors");
		switch(ch) {
			case 'e':
				controlpanel_encoder();
				break;
			case 'm':
				controlpanel_magnetometer();
				break;
			case 'q':
				return;
			case '?':
				static const char msg[] PROGMEM =
					"Sensor menu:\n"
					"  e - Encoders\n"
					"  m - Magnetometer\n"
					"  q - Back\n";
				puts_P(msg);
				break;
		}
	}
}

void controlpanel_encoder() {
	int ticks = 0;
	while (true) {
		char ch = controlpanel_promptChar("Encoders");
		switch (ch) {
			case 'p':
				ticks = enc_get(0);
				printf_P(PSTR("Ticks: %d "), ticks);
				ticks = enc_get(1);
				printf_P(PSTR("Ticks: %d\n"), ticks);
				break;
			case 'r':
				enc_reset(0);
				enc_reset(1);
				break;
			case 'q':
				return;
			case '?':
				static const char msg[] PROGMEM =
					"Encoder menu:\n"
					"  p - position (ticks)\n"
					"  r - Reset counter\n"
					"  q - Back\n";
				puts_P(msg);
				break;
		}
	}
}

void controlpanel_magnetometer() {
	MagReading dir;
	float heading;
	float first_heading;
	while (true) {
		char ch = controlpanel_promptChar("Magnetometer");
		switch (ch) {
			case 'x':
				dir = mag_getReading();
				printf_P(PSTR("X: %d\n"), dir.x);
				break;
			case 'y':
				dir = mag_getReading();
				printf_P(PSTR("Y: %d\n"), dir.y);
				break;
			case 'z':
				dir = mag_getReading();
				printf_P(PSTR("Z: %d\n"), dir.z);
				break;
			case 'h':
				heading = magfollow_getHeading();
				printf_P(PSTR("Heading: %f\n"), heading);
				break;
			case 'l':
				motor_setPWM(MOTOR_LEFT, -600);
				motor_setPWM(MOTOR_RIGHT, 600);
				first_heading = magfollow_getHeading();
				heading = magfollow_getHeading();
				while (sign(heading - first_heading)*(heading - first_heading) < 1) {
					dir = mag_getReading();
					printf_P(PSTR("%d %d %d\n"), dir.x, dir.y, dir.z);
					heading = magfollow_getHeading();
				}
				while (heading != first_heading) {
					heading = magfollow_getHeading();
					dir = mag_getReading();
					printf_P(PSTR("%d %d %d\n"), dir.x, dir.y, dir.z);
				}
				while (sign(heading - first_heading)*(heading - first_heading) < 1) {
					dir = mag_getReading();
					printf_P(PSTR("%d %d %d\n"), dir.x, dir.y, dir.z);
					heading = magfollow_getHeading();
				}
				motor_setPWM(MOTOR_LEFT, 0);
				motor_setPWM(MOTOR_RIGHT, 0);
				break;
			case 'q':
				return;
			case '?':
				static const char msg[] PROGMEM =
					"Encoder menu:\n"
					"  x - x value\n"
					"  y - y value\n"
					"  z - z value\n"
					"  h - heading\n"
					"  l - calibrate\n"
					"  q - Back\n";
				puts_P(msg);
				break;
		}
	}
}

int controlpanel_prompt(const char *prompt, const char *fmt, ...) {
	va_list argp;
	va_start(argp, fmt);

	printf_P(PSTR("%s# "), prompt);

	char buf[32];
	fgets(buf, sizeof(buf), stdin);
	return vsscanf(buf, fmt, argp);
}

char controlpanel_promptChar(const char *prompt) {
	printf_P(PSTR("%s> "), prompt);

	char ch = getchar();
	putchar('\n');
	return ch;
}
