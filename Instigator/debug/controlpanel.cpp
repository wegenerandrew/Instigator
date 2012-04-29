#include "control/drive.h"
#include "control/magfollow.h"
#include "control/weedwhacker.h"
#include "control/tick.h"
#include "debug/debug.h"
#include "debug/controlpanel.h"
#include "hardware/motor.h"
#include "hardware/mag.h"
#include "hardware/encoder.h"
#include "hardware/sonar.h"
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
			case 'l':
				controlpanel_LED();
				break;
			case 'a':
				controlpanel_autonomy();
				break;
			case 'c':
				controlpanel_calibrate();
				break;
			default:
				puts_P(unknown_str);
				break;
			case '?':
				static const char msg[] PROGMEM =
					"Control Panels:\n"
					"  d - Drive\n"
					"  s - Sensors\n"
					"  l - LED\n"
					"  a - Autonomy\n"
					"  c - Calibrate\n"
					"  q - Quit";
				puts_P(msg);
				break;
		}
	}
}

void controlpanel_autonomy() {
	float follow = 0;
	while (true) {
		char ch = controlpanel_promptChar("Autonomy");
		switch (ch) {
			case 'c': {			// Sets current heading of robot to prompted heading from user
				float newheading;
				controlpanel_prompt("Heading (deg)", "%f", &newheading);
				magfollow_setHeading(degtorad(newheading));
				break;
			}
			case 'h': {			// Prints out magnetometer calibrated heading
				float heading = magfollow_getHeading();
				heading = radtodeg(heading);
				printf_P(PSTR("Mag Heading (deg): %f\n"), heading);
				break;
			}
			case 'w': {
				controlpanel_prompt("Heading (deg)", "%f", &follow);
				magfollow_start(speed, anglewrap(degtorad(follow)));
				printf_P(PSTR("Following at: %f\n"), follow);
				break;
			}
			case 'a':
				follow = follow + 5;
				if (magfollow_enabled()) {
					magfollow_start(speed, anglewrap(degtorad(follow)));
					printf_P(PSTR("Following at: %f\n"), follow);
				} else {
					printf_P(PSTR("Not following, but heading set to: %f\n"), follow);
				}
				break;
			case 'd':
				follow = follow - 5;
				if (magfollow_enabled()) {
					magfollow_start(speed, anglewrap(degtorad(follow)));
					printf_P(PSTR("Following at: %f\n"), follow);
				} else {
					printf_P(PSTR("Not following, but heading set to: %f\n"), follow);
				}
				break;
			case 't':
				controlpanel_prompt("Heading (deg)", "%f", &follow);
				printf_P(PSTR("Currently at: %f, Turning to: %f\n"), magfollow_getHeading(), follow);
				magfollow_turn(speed, anglewrap(degtorad(follow)));
				break;
			case ' ':
				magfollow_stop();
				break;
			case 'q':
				magfollow_stop();
				return;
			case '?':
				static const char msg[] PROGMEM =
					"Control Panels:\n"
					"  c - Set Heading\n"
					"  h - Current Heading\n"
					"  w - Magfollow\n"
					"  a - Shift following left\n"
					"  d - Shift following right\n"
					"  t - MagTurn\n"
					" ' '- Stop\n"
					"  q - Quit";
				puts_P(msg);
				break;
			default:
				puts_P(unknown_str);
				break;
		}
	}
}
				

void controlpanel_drive() {
	int16_t steer = 0;
	bool fwd = true;
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
				fwd = true;
				steer = 0;
				motor_ramp(speed, speed);
				break;
			case 'a':
				steer = steer - 50;
				if (fwd) {
					motor_ramp(speed + steer, speed - steer);
				} else {
					motor_ramp(-speed + steer, -speed - steer);
				}
				break;
			case 's':
				fwd = false;
				steer = 0;
				motor_ramp(-speed, -speed);
				break;
			case 'd':
				steer = steer + 50;
				if (fwd) {
					motor_ramp(speed + steer, speed - steer);
				} else {
					motor_ramp(-speed + steer, -speed - steer);
				}
				break;
			case 'W':
				motor_ramp(speed, speed);
				//drive_fd(speed);
				break;
			case 'A':
				motor_ramp(-(speed - turn_diff), (speed - turn_diff));
				//drive_lturn(speed);
				break;
			case 'S':
				motor_ramp(-speed, -speed);
				//drive_bk(speed);
				break;
			case 'D':
				motor_ramp((speed - turn_diff), -(speed - turn_diff));
				//drive_rturn(speed);
				break;
			case 'k':
				weedwhacker_kill();
				break;
			case 'K':
				weedwhacker_start();
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
					"  k	 - Weedwhacker Kill\n"
					"  K	 - Weedwhacker Start\n"
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

void controlpanel_LED() {
	bool error_led = false;
	bool estop_led = false;
	bool led2 = false;
	bool led3 = false;
	bool led4 = false;
	while (true) {
		char ch = controlpanel_promptChar("LED");
		switch (ch) {
			case 'e':
				debug_setLED(ERROR_LED, !error_led);
				error_led = !error_led;
				break;
			case 's':
				debug_setLED(ESTOP_LED, !estop_led);
				estop_led = !estop_led;
				break;
			case '2':
				debug_setLED(OTHER2_LED, !led2);
				led2 = !led2;
				break;
			case '3':
				debug_setLED(OTHER3_LED, !led3);
				led3 = !led3;
				break;
			case '4':
				debug_setLED(OTHER4_LED, !led4);
				led4 = !led4;
				break;
			case 'q':
				debug_setLED(ERROR_LED, false);
				debug_setLED(ESTOP_LED, false);
				debug_setLED(OTHER2_LED, false);
				debug_setLED(OTHER3_LED, false);
				debug_setLED(OTHER4_LED, false);
				return;
			case '?':
				static const char msg[] PROGMEM =
					"LED commands:\n"
					"  e  - Error LED\n"
					"  s  - Estop LED\n"
					"  2  - LED 2\n"
					"  3  - LED 3\n"
					"  4  - LED 4\n"
					"  q  - Back";
				puts_P(msg);
				break;
			default:
				puts_P(unknown_str);
				break;
		}
	}
}

void controlpanel_sensor() {
	while (true) {
		char ch = controlpanel_promptChar("Sensors");
		switch (ch) {
			case 'e':
				controlpanel_encoder();
				break;
			case 'm':
				controlpanel_magnetometer();
				break;
			case 's':
				controlpanel_sonar();
				break;
			case 'q':
				return;
			case '?':
				static const char msg[] PROGMEM =
					"Sensor menu:\n"
					"  e - Encoders\n"
					"  m - Magnetometer\n"
					"  s - Sonar\n"
					"  q - Back\n";
				puts_P(msg);
				break;
			default:
				puts_P(unknown_str);
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
			default:
				puts_P(unknown_str);
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
			default:
				puts_P(unknown_str);
				break;
		}
	}
}

void controlpanel_sonar() {
	while (true) {
		char ch = controlpanel_promptChar("Sonars");
		switch (ch) {
			case 'd':
				printf_P(PSTR("LDist: %f, MDist: %f, RDist: %f\n"), sonar_getDist(LEFT_SONAR), sonar_getDist(MIDDLE_SONAR), sonar_getDist(RIGHT_SONAR));
				break;
			case 't':
				printf_P(PSTR("Timer: %f\n"), sonar_getTimer());
				break;
			case 'q':
				return;
			case '?':
				static const char msg[] PROGMEM =
					"Encoder menu:\n"
					"  d - Distance (cm)\n"
					"  t - Timer\n"
					"  q - Back\n";
				puts_P(msg);
				break;
			default:
				puts_P(unknown_str);
				break;
		}
	}
}

void controlpanel_calibrate() {
	PIDGains newgains;
	while (true) {
		char ch = controlpanel_promptChar("Calibrate");
		switch (ch) {
			case 'm':
				if (controlpanel_promptGains("magfollow", magfollow_getGains(), newgains)) {
					magfollow_setGains(newgains);
					printf_P(PSTR("Gains set!\n"));
				} else {
					printf_P(PSTR("Cancelled.\n"));
				}
				break;
			case 'q':
				return;
			case '?':
				static const char msg[] PROGMEM =
					"Calibrate Menu:\n"
					"  m - Magnetometer PID\n"
					"  q - Back\n";
				puts_P(msg);
				break;
			default:
				puts_P(unknown_str);
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

bool controlpanel_promptGains(const char *name, const PIDGains &curgains, PIDGains &gains) {
	printf_P(PSTR("Setting gains for %s\n"), name);
	printf_P(PSTR("Current gains: P %.4f I %.4f D %.4f MaxI %.4f\n"), curgains.p, curgains.i, curgains.d, curgains.maxi);
	
	if (controlpanel_prompt("P", "%f", &gains.p) != 1)
		gains.p = curgains.p;
	if (controlpanel_prompt("I", "%f", &gains.i) != 1)
		gains.i = curgains.i;
	if (controlpanel_prompt("D", "%f", &gains.d) != 1)
		gains.d = curgains.d;
	if (controlpanel_prompt("MaxI", "%f", &gains.maxi) != 1)
		gains.maxi = curgains.maxi;
		
	printf_P(PSTR("New gains: P %.4f I %.4f D %.4f MaxI %.4f\n"), gains.p, gains.i, gains.d, gains.maxi);
	return controlpanel_promptChar("Ok? [y/n]") == 'y';
}
