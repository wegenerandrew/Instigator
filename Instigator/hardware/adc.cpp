#include "hardware/adc.h"

#include <avr/io.h>
#include <util/delay.h>

void adc_init() {
	ADCA.CTRLA = ADC_ENABLE_bm;
	ADCA.REFCTRL = ADC_REFSEL_VCC_gc | ADC_BANDGAP_bm | ADC_TEMPREF_bm;
	ADCA.PRESCALER = ADC_PRESCALER_DIV32_gc;
	
	ADCB.CTRLA = ADC_ENABLE_bm;
	ADCB.REFCTRL = ADC_REFSEL_VCC_gc | ADC_BANDGAP_bm | ADC_TEMPREF_bm;
	ADCB.PRESCALER = ADC_PRESCALER_DIV32_gc;
}

uint16_t adc_sample(uint8_t pin) {
	ADC_t *adc;
	if (pin < 8) {	// pins 0-7 on ADCA, pins 8-13 on ADCB
		adc = &ADCA;
	} else {
		adc = &ADCB;
		pin = pin - 8;
	}

	adc->CH0.MUXCTRL = pin << ADC_CH_MUXPOS_gp; // select the desired pin
	adc->CH0.CTRL = ADC_CH_START_bm | ADC_CH_INPUTMODE_SINGLEENDED_gc; // start a single ended conversion
	while (!(adc->CH0.INTFLAGS & ADC_CH_CHIF_bm)) { } // wait for it to complete
	adc->CH0.INTFLAGS = ADC_CH_CHIF_bm; // clear completion flag
	return adc->CH0.RES;
}

uint16_t adc_sampleAverage(uint8_t pin, uint8_t samples) {
	uint32_t total = 0;
	for (int i = 0; i < samples; i++) {
		total = total + adc_sample(pin);
		_delay_us(100);
	}
	return (uint16_t)(total/(uint32_t)samples);
}

float adc_sampleFloat(uint8_t pin) {
	return adc_sample(pin)/4096.0f;
}

float adc_sampleBattery() {
	return adc_sample(ADC_BATTERY)*0.003;
}


