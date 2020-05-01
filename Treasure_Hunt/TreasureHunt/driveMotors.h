#ifndef DRIVEMOTORS_H
#define DRIVEMOTORS_H

#define MAXWHEELSPEED 500
#define MAXBEARING M_PI

float wrapAngle(float angle);
void setDesiredSpeed(int16_t speed);
void startMotors(void);
void isAllowedToDrive(bool doDrive);
void setDesiredBearing(float desiredBearing);


float getXPosition(void);
float getYPosition(void);
float getBearing(void);
void limitWheelSpeed(int16_t speedLimit);

#endif /* DRIVEMOTORS_H */
