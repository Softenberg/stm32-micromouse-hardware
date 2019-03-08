#ifndef motor_control_h
#define motor_control_h

void motorSetup(void);
void speedProfile(void);
void getEncoderStatus(void);
void updateCurrentSpeed(void);
void calculateMotorPwm(void);

// 40mm is wheel diameter
// 1000 is for s => ms
// 2096 is counts per rev on the motor
// 19 is gearing form wheel to motor.
#define speedToCounts(speed) (speed / (3.141592 * 40) / 1000.0 * 2096.0 * 19.0)


#endif 

