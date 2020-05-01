#ifndef DRIVEMOTORS_H
#define DRIVEMOTORS_H


float wrapAngle(float angle);
void setDesiredSpeed(int16_t speed);
void startMotors(void);
void isAllowedToDrive(bool doDrive);
void setDesiredBearing(float desiredBearing);


float getXPosition(void);
float getYPosition(void);

float getBearing(void);




#endif /* DRIVEMOTORS_H */
