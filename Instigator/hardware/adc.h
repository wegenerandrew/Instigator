#ifndef ADC_H_
#define ADC_H_

#include <stdint.h>

enum {
	ADC_BATTERY = 0
};

void adc_init();
uint16_t adc_sample(uint8_t pin);
uint16_t adc_sampleAverage(uint8_t pin, uint8_t samples);
float adc_sampleFloat(uint8_t pin);
float adc_sampleBattery();

#endif
