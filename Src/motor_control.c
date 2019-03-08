#include <stdint.h>
#include "main.h"
#include "stm32f1xx_hal.h"
#include "motor_control.h"
#include "pwm.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;

int distanceLeft;
int leftBaseSpeed;
int rightBaseSpeed;

int encoderChange;
int encoderCount;

int leftEncoder;
int rightEncoder;

int leftEncoderChange;
int rightEncoderChange;

int leftEncoderOld;
int rightEncoderOld;

int leftEncoderCount;
int rightEncoderCount;

float curSpeedX = 0;
float curSpeedW = 0;
int targetSpeedX = 0;
int targetSpeedW = 0;
int encoderFeedbackX = 0;
int encoderFeedbackW = 0;
float pidInputX = 0;
float pidInputW = 0;
float posErrorX = 0;
float posErrorW = 0;
float oldPosErrorX = 0;
float oldPosErrorW = 0;
int posPwmX = 0;
int posPwmW = 0;
float kpX = 30, kdX = 10;  //original is 2 and 4
float kpW = 10, kdW = 12;//used in straight, original is 1 and 12
float accX = 1000;//0.6m/s/s  => 600mm/s, The unit i use is cm/s or cm/s^2, should be mm/s instead I think
float decX = 1000; 
float accW = 1; //cm/s^2 
float decW = 1;

int forwardCorrection = 0;
int reverseCorrection = 0;
int lRevCorrVal;
int rRevCorrVal;
void motorSetup(void){
	/* Start the encoder timer */
		HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
		HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
		
		/*Enable the motor*/
		HAL_GPIO_WritePin(H_Bridge_Enable_GPIO_Port, H_Bridge_Enable_Pin, GPIO_PIN_SET);
		/* Orientation Right motor -> CW */
		HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
		/* Orientation Left motor -> CCW */
		HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
}


void speedProfile(void){	
	getEncoderStatus();
	updateCurrentSpeed();
	calculateMotorPwm();      
}

void getEncoderStatus(void){
	leftEncoder = TIM4->CNT;//read current encoder ticks from register of 16 bit general purpose timer 4
	rightEncoder = TIM2->CNT;//read current encoder ticks from register of 16 bit general purpose timer 2

	leftEncoderChange = leftEncoder - leftEncoderOld;
	rightEncoderChange = rightEncoder - rightEncoderOld;
	
	if(leftEncoderChange > 0xF000){
		leftEncoderChange = leftEncoder - 0xFFFF - leftEncoderOld;
	}else if(leftEncoderChange < - 0xF000){
		leftEncoderChange = 0xFFFF - leftEncoderOld + leftEncoder;
	}
	
	if(rightEncoderChange > 0xF000){
		rightEncoderChange = rightEncoder - 0xFFFF - rightEncoderOld;
	}else if(rightEncoderChange < - 0xF000){
		rightEncoderChange = 0xFFFF - rightEncoderOld + rightEncoder;
	}
	
	
	encoderChange = (leftEncoderChange + rightEncoderChange)/2;	 

	leftEncoderOld = leftEncoder;
	rightEncoderOld = rightEncoder;
					
	leftEncoderCount += leftEncoderChange;
	rightEncoderCount += rightEncoderChange;
	encoderCount =  (leftEncoderCount+rightEncoderCount)/2;	
	
	//distanceLeft -= encoderChange;// update distanceLeft	
}


void updateCurrentSpeed(void){
	if(curSpeedX < targetSpeedX){
		curSpeedX += (float)speedToCounts(accX*2)/1000; //Original was 100, not 1000, don't know what is correct
		if(curSpeedX > targetSpeedX)		// 100 was original since accW was in cm/s^2 and not mm/s^2
			curSpeedX = targetSpeedX;
	}else if(curSpeedX > targetSpeedX){
		curSpeedX -= (float)speedToCounts(decX*2)/1000;
		if(curSpeedX < targetSpeedX)
			curSpeedX = targetSpeedX;
	}
	if(curSpeedW < targetSpeedW){
		curSpeedW += accW;
		if(curSpeedW > targetSpeedW)
			curSpeedW = targetSpeedW;
	}else if(curSpeedW > targetSpeedW){
		curSpeedW -= decW;
		if(curSpeedW < targetSpeedW)
			curSpeedW = targetSpeedW;
	}	
}

void calculateMotorPwm(void){ // encoder PD controller	
	//int gyroFeedback;
	int rotationalFeedback;
	//int sensorFeedback;
	
  /* simple PD loop to generate base speed for both motors */	
	encoderFeedbackX = rightEncoderChange + leftEncoderChange;
	encoderFeedbackW = rightEncoderChange - leftEncoderChange;	
	
	//gyroFeedback = aSpeed/gyroFeedbackRatio; //gyroFeedbackRatio mentioned in curve turn lecture	
	//sensorFeedback = sensorError/a_scale;//have sensor error properly scale to fit the system
	
	rotationalFeedback = encoderFeedbackW;

	
	//Removed += to = sign to only do velocity control.
	posErrorX = curSpeedX - encoderFeedbackX;
	posErrorW = curSpeedW - rotationalFeedback;
	
	posPwmX = kpX * posErrorX + kdX * (posErrorX - oldPosErrorX);
	posPwmW = kpW * posErrorW + kdW * (posErrorW - oldPosErrorW);
	
	oldPosErrorX = posErrorX;
	oldPosErrorW = posErrorW;
	
	leftBaseSpeed = posPwmX - posPwmW;
	rightBaseSpeed = posPwmX + posPwmW;

	setLeftPWM(leftBaseSpeed);
	setRightPWM(rightBaseSpeed);
}



/**

	* @brief This function converts a speed/acceleration to counts/time
	* @param Speed or Acceleration in cm/s or cm/s^2
	* @return Speed or Acceleration in counts/ms
*/
/*
int speed_to_counts(int speed){
	// / (3.141592*4) - cm/s => varv/s (omkrets = PI * diameter (4cm))
	// / 1000 - varv/s => varv/ms
	// * 2096 * 19 - varv/ms => counts/ms (2096 counts per motor varv, 19:1 i utväxling.)
	return  speed / (3.141592 * 4.0) / 1000.0 * 2096.0 * 19.0;
}
*/

