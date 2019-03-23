#ifndef motor_control_h
#define motor_control_h

#include <stdint.h>

#define WHEEL_TO_WHEEL 137.5 // Center of wheel to the center of the other wheel. 
#define WHEEL_D 31.0
#define CPR 2096.0
#define PI 3.141592
#define GEARING (19.0 * 56.0/12.0)

//NEW
extern float currPos;
extern float setPos;
extern float currAngle;
extern float setAngle;
extern float integralError;


//OLD
extern int distanceLeft;
extern volatile int rotationLeft;
extern float curSpeedX;
extern float curSpeedW;
extern int targetSpeedX;
extern int targetSpeedW;
extern int moveSpeed;
extern int maxSpeed;
extern int turnSpeed;
extern float decX; 
extern float decW;
extern int encoderCount;
extern int rightEncoderCount;
extern int oldEncoderCount;
extern int oldRightEncoderCount;
extern int oneCellDistance;
extern int encoderChange;

void motorSetup(void);
void speedProfile(void);
void getEncoderStatus(void);
void updateCurrentSpeed(void);
void calculateMotorPwm(void);
float needToDecelerate(int32_t, int16_t, int16_t);
void rotate(int);

// 1000 is for s => ms
#define speedToCounts(speed) (speed / (PI * WHEEL_D) / 1000.0 * CPR * GEARING)
#define countsToSpeed(counts) (counts * (PI * WHEEL_D) * 1000.0 / CPR / GEARING)
#define distanceToCounts(distance) (distance / (PI * WHEEL_D) * CPR * GEARING)
#define countsToDistance(distance) (distance * (PI * WHEEL_D) / CPR / GEARING)

#define degreesToRadians(deg) (deg / 360.0 * 2 * PI)
#define rotSpeedToCounts(rotSpeed) ( speedToCounts(degreesToRadians(rotSpeed) * WHEEL_D) )
#define rotToCounts(deg) ( distanceToCounts(deg / 360.0 * (PI * WHEEL_TO_WHEEL)) ) 
#define countsToRot(counts) ( countsToDistance(counts) / (PI * WHEEL_TO_WHEEL) * 360.0  ) 
#endif 
