#ifndef DRIVEMOTORS_H
#define DRIVEMOTORS_H

#define MAXWHEELSPEED 500
#define MAXBEARING M_PI
/**
 * @brief: starts motorthread with high priority
 */
void startMotors(void);
/**
 * @brief: function which wraps angle into the range of -pi to +pi
 *
 * @param[in] angle: angle which should be wrapped
 * @return: returns the result of the wrapping
 */
float wrapAngle(float angle);
/**
 * @brief: sets desired forward speed
 * @param[in] speed: desired speed in cm/s
 */
void setDesiredSpeed(int16_t speed);

/**
 * @brief: function with which forward/backward travel can be blocked, rotating on point is still possible
 * if set once the status is retained until this function is called with an other value
 * @param[in]: bool , true if robot can drive forward, false if it shouldn't.
 */
void isAllowedToDrive(bool doDrive);
/**
 * @brief: sets the desired orientation of the robot.
 * it is the orientation in relation to the orientation of the robot, when it started up. The angle is wrapped onto -pi to pi
 *
 * @param[in] desiredBearing: the desired robot orientation in radiant
 */
void setDesiredBearing(float desiredBearing);
/*
 * @brief: returns the x-coordinate
 * @return: the x-coordinate in cm
 */
float getXPosition(void);
/*
 * @brief: returns the y-coordinate
 * @return: the y-coordinate in cm
 */
float getYPosition(void);
/*
 * @brief: returns the orientation of the robot.
 * the orientation is in relation to the orientation of the robot at startup
 * @return: the orientation in radiant
 */
float getBearing(void);
/**
 * @brief: function which returns ture if the robot is rotating
 * @return: true if the robot is rotating, false if he is not rotating
 */
bool isRotating(void);
/**
 * @brief: lets external functions limit the speed at which the wheels are allowed to rotate
 *
 * @param[in] speedLimit: the maximal speed at which the wheels are allowed to spin,in steps/s,
 * if the speed is higher than MAXWHEELSPEED, then it will be set to MAXWHEELSPEED
 */
void limitWheelSpeed(int16_t speedLimit);

#endif /* DRIVEMOTORS_H */
