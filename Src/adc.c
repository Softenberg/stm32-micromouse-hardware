#include <stdint.h>
#include "main.h"
#include "adc.h"

/**
	* @brief Function that read a channel on the ADC and returns the value.
	* @param ADCx	(ADC_HandleTypeDef)	Pointer to the ADC. (e.g &hadc1)
	* @param channel (uint8_t) 				What channel to read. A pin is usually assigned to a channel on the ADC.
	* @return (uint16_t)							Since the ADC on the STM32F103C8 is 12bit, we will return a 12 bit value.
*/
uint16_t readADC(ADC_HandleTypeDef *ADCx, uint8_t channel){
	HAL_ADC_Start(ADCx);
	while(HAL_ADC_PollForConversion(ADCx, channel) != HAL_OK);
	return HAL_ADC_GetValue(ADCx);
}

/**
	* @brief Function that reads the LiPo battery voltage via a voltage divider. R1 = 1k, R2 = 470;
* @return (float)	The abattery voltage. V_Batt = V_Sense * (R1 + R2) / R2 (Voltage divider=
*/
float batteryVoltage(void){
	int adcVal = readADC(&hadc1, 9);
	return ((float)adcVal / 4096) * 3.3 * (1000 + 470) / 470.0 ;
}


