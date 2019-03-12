#include <stdint.h>
#include <stdlib.h>
#include "main.h"
#include "stm32f1xx_hal.h"
#include "motor_control.h"
#include "pwm.h"



int distanceLeft;
int leftBaseSpeed;
int rightBaseSpeed;

int encoderChange;
int encoderCount;
int oldEncoderCount;

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
float kpX = 5, kdX = 4;  //original is 2 and 4, when using Lipo on 8.4V you need to lower Kp.
float kpW = 2, kdW = 12;//used in straight, original is 1 and 12
float accX = 1000;//0.6m/s/s  => 600mm/s, The unit i use is cm/s or cm/s^2, should be mm/s instead I think
float decX = 1000; 
float accW = 1; //cm/s^2 
float decW = 1;

int moveSpeed = speedToCounts(100*2); // max 0.1 m/s
int maxSpeed = speedToCounts(500*2); // max 0.5 m/s, absolute max is around 1m/s.

int oneCellDistance = distanceToCounts(180); //One cell is 180mm

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
	
	// Overflow correction
	if(leftEncoderChange > 0xF000){
		leftEncoderChange = leftEncoder - 0xFFFF - leftEncoderOld;
	}else if(leftEncoderChange < - 0xF000){
		leftEncoderChange = 0xFFFF - leftEncoderOld + leftEncoder;
	}
	
	// Overflow correction
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
	
	distanceLeft -= encoderChange;
}


void updateCurrentSpeed(void){
	if(curSpeedX < targetSpeedX){
		curSpeedX += (float)speedToCounts(accX*2)/1000;// /100 was original since accW was in cm/s^2 and not mm/s^2
		if(curSpeedX > targetSpeedX)		
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
	posErrorX += curSpeedX - encoderFeedbackX;
	posErrorW += curSpeedW - rotationalFeedback;
	
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
	* @brief Function that determines at what rate you need to decelerate 
	*        in order to reach the goal speed within a given distance and current speed.
	* @param dist		(encoder counts)	The distance until the goal speed should be reached. 
	* @param curSpd	(counts/ms)				The current speed	
	* @param endSpd	(counts/ms)				The goal speed		
	* @return The deceleration
*/
int needToDecelerate(int32_t dist, int16_t curSpd, int16_t endSpd){
	if (curSpd<0) curSpd = -curSpd;
	if (endSpd<0) endSpd = -endSpd;
	if (dist<0) dist = 1;			//-dist;
	if (dist == 0) dist = 1;  //prevent divide by 0
	
	// The / 1000 is needed because we are converting from counts/(ms^2) to m/s. 
	// countsToSpeed takes care of one *1000 since it converts speed, not acceleration.
	int acc = countsToSpeed((curSpd*curSpd - endSpd*endSpd)*1000/dist/4/2);
		
	if(acc < 0)
		acc = -acc;
	
	return acc;
	
	// 2as = V
	//*a*S = Vt^2 - V0^2  ==>  a = (Vt^2-V0^2)/2/S
	//because the speed is the sum of left and right wheels(which means it's doubled), that's why there is a "/4" in equation since the square of 2 is 4
}

/**
	* @brief Function that rotates the robot a given amount of degrees. (NOT IMPLEMENTED)
	* @param angle	(degrees)	The angle you want to turn. Positive means right turn. 
*/
void rotate(int angle){

}

