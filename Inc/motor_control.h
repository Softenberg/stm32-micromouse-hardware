#ifndef motor_control_h
#define motor_control_h

void motorSetup(void);
void speedProfile(void);
void getEncoderStatus(void);
void updateCurrentSpeed(void);
void calculateMotorPwm(void);


#define speedToCounts(speed) (speed / (3.141592 * 4.0) / 1000.0 * 2096.0 * 19.0)


#endif 

