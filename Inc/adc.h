#ifndef ADC_H
#define ADC_H

#include <stdint.h>
#include "main.h"

uint16_t readADC(ADC_HandleTypeDef *ADCx, uint8_t channel);
float batteryVoltage(void);


#endif /* ADC_H */ 
