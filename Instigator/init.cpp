#include "init.h"
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "controlpanel.h"
#include "control/motorcontrol.h"
#include "debug.h"
#include "estop.h"
#include "hardware/encoder.h"
#include "hardware/motor.h"
#include "hardware/mag.h"
#include "tick.h"
#include "uart.h"

void init() {
	init_clocks();
	init_modules();
	init_interrupts();
}

void init_interrupts(){
	PMIC.CTRL = PMIC_HILVLEN_bm;	//Enable High level interrupts
	PMIC.CTRL |= PMIC_MEDLVLEN_bm;	//Enable Medium level interrupts
	PMIC.CTRL |= PMIC_LOLVLEN_bm;	//Enable Low level interrupts
	
	sei();							//Clears global mask
}

void init_clocks() {
	OSC.CTRL = OSC_RC32MEN_bm | OSC_RC2MEN_bm; // enable 32 Mhz clock (while leaving the current 2 Mhz clock enabled)
	while (!(OSC.STATUS & OSC_RC32MRDY_bm)) { } // wait for it to stabilize
	CPU_CCP = CCP_IOREG_gc; // enable access to protected registers
	CLK.CTRL = CLK_SCLKSEL_RC32M_gc; // switch to the 32 Mhz clock
}

void init_modules() {
	estop_init();
	uart_init();
	debug_init();
	tick_init();
	enc_init();
	motor_init();
	motorcontrol_init();
	mag_init();
	controlpanel_init();
	estop_initCheck();
}
