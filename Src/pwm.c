#include "pwm.h"
#include "main.h"
#include "stm32f1xx_hal.h"

void setRightPWM(int32_t speed){
	//Saturation of the speed signal
	if(speed > 999)
		speed = 999;
	if(speed < -999)
		speed = -999;
	
	if(speed >= 0){
		/* Right motor forward -> CW */
		HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
		R_PWM = speed;
	}else{
		/* Right motor reverse -> CCW */
		HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
		R_PWM = -speed;
	}
}

void setLeftPWM(int32_t speed){
	if(speed > 999)
		speed = 999;
	if(speed < -999)
		speed = -999;
	
	if(speed >= 0){
		/* Left motor Forward -> CCW */
		HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
		L_PWM = speed;
	}else{
		/* Left motor -> Reverse -> CW */
		HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
		L_PWM = -speed;
	}
}
