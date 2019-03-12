#ifndef motor_control_h
#define motor_control_h

#include <stdint.h>

extern int distanceLeft;
extern float curSpeedX;
extern int targetSpeedX;
extern int moveSpeed;
extern int maxSpeed;
extern float decX; 
extern int encoderCount;
extern int oldEncoderCount;
extern int oneCellDistance; 

void motorSetup(void);
void speedProfile(void);
void getEncoderStatus(void);
void updateCurrentSpeed(void);
void calculateMotorPwm(void);
int needToDecelerate(int32_t, int16_t, int16_t);
void rotate(int);

// 40mm is wheel diameter
// 1000 is for s => ms
// 2096 is counts per rev on the motor
// 19 is gearing form wheel to motor.
#define speedToCounts(speed) (speed / (3.141592 * 40) / 1000.0 * 2096.0 * 19.0)
#define countsToSpeed(counts) (counts * (3.141592 * 40) * 1000.0 / 2096.0 / 19.0)
#define distanceToCounts(distance) (distance / (3.141592 * 40) * 2096.0 * 19)

#endif 

