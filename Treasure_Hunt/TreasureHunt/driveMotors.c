#include <main.h>
#include <math.h>
#include <driveMotors.h>
#include <motors.h>
#include <stdio.h>
#include <string.h>
#include "ch.h"
#include "hal.h"
#include <stdlib.h>
#include <chprintf.h>



#define WHEEL_DISTANCE 0.053f			// distance between the 2 wheels of the robot
#define WHEEL_RADIUS 0.041f/2			// radius of the wheels of the robot
#define REGULATORPERIOD 10				// in ms
#define KP 10.0f						// proportional controller value. used to correct the rotation to align the robot rotation with the desired rotation
#define KI 0.0f							// Integral value of the controller. not in use
#define MAXERROR 2.0f					// value at which the integration is stopped


#define MINSPEED 5						// wheel speed under which the wheel is considered as not rotating
#define COUNTSPERTURN 1000				// constant which describes how many counts/steps per turn of the wheel the motors make

#define FILTERCOEFFICIENT 0.3f			// filter coefficient for filtering the wheelspeed



static float xPosition=0;				//tracks current x position
static float yPosition=0;				//tracks current y position
static float desiredBearingPhi=0;		// tracks orientation which the robot should have
static float bearingPhi=0;				//tracks current orientation
static int16_t desiredSpeed=0;			//tracks desired forwardspeed
static bool stop=false;					// variable used to track if robot is allowed to drive forward or not
static bool rotating=false;				// tracks if the robot is rotating or not
static int16_t maxWheelSpeed=MAXWHEELSPEED;	 // maximal allowed wheel speed

static THD_WORKING_AREA(driveMotor_thd_was, 256);
static THD_FUNCTION(driveMotor_thd, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);
    systime_t time;
    float errorSum=0;
    float oldVelocityLeft=0;
    float oldVelocityRight=0;
    int32_t previousStepsLeft=0;
    int32_t previousStepsRight=0;
    int16_t filtredSpeedLeft=0;
    int16_t filtredSpeedRight=0;


    while(1)
    {
    	time=chVTGetSystemTime();
    	int16_t forwardSpeed=0;							//speed in cm/s
    	float rotationalSpeed=0;

    	if(fabs(desiredBearingPhi-bearingPhi)<0.05f)
    	{
    		if(stop==false)
    		{
    			forwardSpeed=desiredSpeed;
    		}
    		rotating=false;
    	}
    	else
    	{
    		rotating=true;
    		forwardSpeed=0;
    	}

    	//calculate rotational speed nessecary to rotate  the robot towards thedesired direction
    	errorSum += (desiredBearingPhi-bearingPhi);
    	if(errorSum>MAXERROR)
    	{
    		errorSum=MAXERROR;
    	}
    	else if(errorSum<(-MAXERROR))
    	{
    		errorSum=-MAXERROR;
    	}
    	rotationalSpeed = KP*(desiredBearingPhi-bearingPhi)+ KI*errorSum;
    	// calculate the wheelspeed to generate the desired rotation and forward speed of the robot
    	int16_t desiredSpeedLeft = (((forwardSpeed/100.0f)-WHEEL_DISTANCE/2*rotationalSpeed)/(WHEEL_RADIUS*2*M_PI))*COUNTSPERTURN;
    	int16_t desiredSpeedRight = (((forwardSpeed/100.0f)+WHEEL_DISTANCE/2*rotationalSpeed)/(WHEEL_RADIUS*2*M_PI))*COUNTSPERTURN;
    	if(fabs(desiredSpeedLeft)<MINSPEED)				//trying to prevent overheating, since the left/right motor speed function only sets the H-bridge to 0 when 0 speed is given
    	{
    		desiredSpeedLeft=0;
    	}
    	else if(desiredSpeedLeft>maxWheelSpeed)			//limit wheel speed to the maximal or the one set by limitWheelSpeed
    	{
    		desiredSpeedLeft=maxWheelSpeed;
    	}
    	else if(desiredSpeedLeft<-maxWheelSpeed)
    	{
    		desiredSpeedLeft=-maxWheelSpeed;
    	}

    	if(fabs(desiredSpeedRight)<MINSPEED)				//trying to prevent overheating
		{
    		desiredSpeedRight=0;
		}
    	else if(desiredSpeedRight>maxWheelSpeed)
		{
			desiredSpeedRight=maxWheelSpeed;
		}
		else if(desiredSpeedRight<-maxWheelSpeed)
		{
			desiredSpeedRight=-maxWheelSpeed;
		}
    	if(fabs(bearingPhi-desiredBearingPhi)>M_PI)	//rotate the shorter way, also lets it rotate over the singularity
    	{
    		desiredSpeedLeft=-desiredSpeedLeft;
    		desiredSpeedRight=-desiredSpeedRight;
    	}

    	left_motor_set_speed(desiredSpeedLeft);
    	right_motor_set_speed(desiredSpeedRight);

    	//calculate actual speed of the wheels
    	int32_t stepsLeft	=	left_motor_get_pos();
    	int32_t stepsRight	=	right_motor_get_pos();
    	int16_t actualSpeedLeft = (stepsLeft-previousStepsLeft)*1000.0f/REGULATORPERIOD;			//1000 ms per second
    	int16_t actualSpeedRight = (stepsRight-previousStepsRight)*1000.0f/REGULATORPERIOD;
    	previousStepsLeft = stepsLeft;
    	previousStepsRight = stepsRight;
    	//catching overflow of stepscounter and filter the measurement
    	if(abs(actualSpeedLeft)>MAXWHEELSPEED)
    	{
    		actualSpeedLeft=oldVelocityLeft;
    	}
    	if(abs(actualSpeedRight)>MAXWHEELSPEED)
    	{
    		actualSpeedRight=oldVelocityRight;
    	}
    	oldVelocityLeft=actualSpeedLeft;
    	oldVelocityRight=actualSpeedRight;
    	filtredSpeedLeft	= actualSpeedLeft*FILTERCOEFFICIENT+(1-FILTERCOEFFICIENT)*filtredSpeedLeft;
    	filtredSpeedRight	= actualSpeedRight*FILTERCOEFFICIENT+(1-FILTERCOEFFICIENT)*filtredSpeedRight;
    	//convert wheelspeed into robot movement
    	float estimatedForwardVelocity		=	((filtredSpeedLeft+filtredSpeedRight)*2*M_PI*WHEEL_RADIUS)*50/COUNTSPERTURN;
    	float estimatedRotationalVelocity	= 	(filtredSpeedRight-filtredSpeedLeft)*2*M_PI*WHEEL_RADIUS/WHEEL_DISTANCE/COUNTSPERTURN;
    	//use calculated movement to estimate the orientation and position
    	bearingPhi+=REGULATORPERIOD*estimatedRotationalVelocity/1000.0f;
    	xPosition = xPosition+cosf(bearingPhi)*estimatedForwardVelocity*REGULATORPERIOD/1000.0f;
    	yPosition = yPosition+sinf(bearingPhi)*estimatedForwardVelocity*REGULATORPERIOD/1000.0f;
    	bearingPhi=wrapAngle(bearingPhi);
    	chThdSleepUntilWindowed(time,time+MS2ST(REGULATORPERIOD));
    }
}

float wrapAngle(float angle)
{
	angle =fmod(angle+M_PI,2*M_PI);
	if(angle<0)
	{
		angle +=2*M_PI;
	}
	return angle-M_PI;
}


void setDesiredSpeed(int16_t speed)
{
	desiredSpeed=speed;
}
void startMotors(void)
{
	motors_init();
	chThdCreateStatic(driveMotor_thd_was, sizeof(driveMotor_thd_was), HIGHPRIO, driveMotor_thd, NULL);
}
void isAllowedToDrive(bool doDrive)
{
	stop=!doDrive;
}
void setDesiredBearing(float desiredBearing)
{
	desiredBearingPhi=wrapAngle(desiredBearing);
	rotating=true;
}
float getXPosition(void)
{
	return xPosition;
}
float getYPosition(void)
{
	return yPosition;
}
float getBearing(void)
{
	return bearingPhi;
}
void limitWheelSpeed(int16_t speedLimit)
{
	if(speedLimit<MAXWHEELSPEED)
	{
		maxWheelSpeed=speedLimit;
	}
	else
	{
		maxWheelSpeed=MAXWHEELSPEED;
	}
}
bool isRotating(void)
{
	return rotating;
}
