#include "controlpanel.h"
#include "debug.h"
#include "control/motor.h"
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

void controlpanel_init() {
	printf_P(PSTR("Starting up\n"));
}

bool controlpanel() {
	while (true) {
		switch (controlpanel_promptChar("Main")) {
			case 'd':
				controlpanel_drive();
				break;
			default:
				puts_P(unknown_str);
				break;
			case '?':
				static const char msg[] PROGMEM =
					"Control Panels:\n"
					"  d - Drive";
				puts_P(msg);
				break;
		}
	}
}

void controlpanel_drive() {
	float speed=800;
	while (true) {
		char ch=controlpanel_promptChar("Drive");

		switch (ch) {
			case ' ':
				motor_setPWM(MOTOR_LEFT, 0);
				motor_setPWM(MOTOR_RIGHT, 0);
				//drive_stop();
				break;
			case 'x':
				//drive_stop(DM_TRAJ);
				break;

			case 'w':
				motor_setPWM(MOTOR_LEFT, speed);
				motor_setPWM(MOTOR_RIGHT, speed);
				//drive_fd(speed);
				break;
			case 'a':
				motor_setPWM(MOTOR_LEFT, -speed);
				motor_setPWM(MOTOR_RIGHT, speed);
				//drive_lturn(speed);
				break;
			case 's':
				motor_setPWM(MOTOR_LEFT, -speed);
				motor_setPWM(MOTOR_RIGHT, -speed);
				//drive_bk(speed);
				break;
			case 'd':
				motor_setPWM(MOTOR_LEFT, speed);
				motor_setPWM(MOTOR_RIGHT, -speed);
				//drive_rturn(speed);
				break;

			case 'W':
				//drive_fdDist(speed, 50);
				break;
			case 'A':
				//drive_lturnDeg(speed, 90);
				break;
			case 'S':
				//drive_bkDist(speed, 50);
				break;
			case 'D':
				//drive_rturnDeg(speed, 90);
				break;	
			case '=':
				speed += 2;
				printf_P(PSTR("Speed: %f\n"), speed);
				break;
			case '-':
				speed -= 2;
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

			case 'z':			// Does moonwalk forwards
				speed = 40;
				for (int i = 0; i < 20; i++) {
					//drive_fd(speed);
					_delay_ms(25);
					//drive_rturn(speed);
					_delay_ms(50);
					//drive_fd(speed);
					_delay_ms(25);
					//drive_lturn(speed);
					_delay_ms(50);
				}
				//drive_stop();
				speed = 20;
				break;

			case 'Z':			// Does moonwalk backwards
				speed = 100;
				for (int i = 0; i < 20; i++) {
					//drive_bk(speed);
					_delay_ms(25);
					//drive_rturn(speed);
					_delay_ms(50);
					//drive_bk(speed);
					_delay_ms(25);
					//drive_lturn(speed);
					_delay_ms(50);
				}
				speed = 20;
				break;

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
					"  WASD  - Execute distance moves\n"
					"  c	 - Disable motor control\n"
					"  Pp	 - Enable/Disable motor control debug\n"
					"  zZ	 - Moonwalk (WIP)\n"
					"  q	 - Back";
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
