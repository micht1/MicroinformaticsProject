#ifndef DRIVEMOTORS_H
#define DRIVEMOTORS_H

#define MAXWHEELSPEED 500
#define MAXBEARING M_PI

float wrapAngle(float angle);
void setDesiredBearing(float desiredBearing);
void setDesiredSpeed(int16_t speed);
float getCurrentBearing(void);
void startMotors(void);
float getXPosition(void);
float getYPosition(void);
void isAllowedToDrive(bool doDrive);
void limitWheelSpeed(int16_t speedLimit);


#endif /* DRIVEMOTORS_H */
