#ifndef DRIVEMOTORS_H
#define DRIVEMOTORS_H


float wrapAngle(float angle);
void setDesiredBearing(float desiredBearing);
void setDesiredSpeed(int16_t speed);
float getCurrentBearing(void);
void startMotors(void);
float getXPosition(void);
float getYPosition(void);
void isAllowedToDrive(bool doDrive);


#endif /* DRIVEMOTORS_H */
