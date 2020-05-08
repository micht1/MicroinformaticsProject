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


#define ROTATIONALSPEED 2.0f			//rad/s
#define WHEEL_DISTANCE 0.053f
#define WHEEL_RADIUS 0.041f/2
#define REGULATORPERIOD 10				// in ms
#define KP 10.0f
#define KI 0.0f
#define MAXERROR 2.0f


#define PI M_PI
#define MINSPEED 5
#define COUNTSPERTURN 1000

#define FILTERCOEFFICIENT 0.3f



static float xPosition=0;
static float yPosition=0;
static float desiredBearingPhi=0;
static float bearingPhi=0;
static int16_t desiredSpeed=0;
static bool stop=false;
static bool rotating=false;
static int16_t maxWheelSpeed=MAXWHEELSPEED;

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

    	int16_t desiredSpeedLeft = (((forwardSpeed/100.0f)-WHEEL_DISTANCE/2*rotationalSpeed)/(WHEEL_RADIUS*2*M_PI))*COUNTSPERTURN;
    	int16_t desiredSpeedRight = (((forwardSpeed/100.0f)+WHEEL_DISTANCE/2*rotationalSpeed)/(WHEEL_RADIUS*2*M_PI))*COUNTSPERTURN;
    	if(fabs(desiredSpeedLeft)<MINSPEED)				//trying to prevent overheating
    	{
    		desiredSpeedLeft=0;
    	}
    	else if(desiredSpeedLeft>maxWheelSpeed)
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
    	if(fabs(bearingPhi-desiredBearingPhi)>M_PI)
    	{
    		desiredSpeedLeft=-desiredSpeedLeft;
    		desiredSpeedRight=-desiredSpeedRight;
    	}
    	if(forwardSpeed<MAXWHEELSPEED && rotating==true)
		{
			//chprintf((BaseSequentialStream *) &SD3,"DSpeedL: %d, DSpeedR: %d\n\r",desiredSpeedLeft,desiredSpeedRight);
		}
    	left_motor_set_speed(desiredSpeedLeft);
    	right_motor_set_speed(desiredSpeedRight);

    	//calculate actual speed
    	int32_t stepsLeft	=	left_motor_get_pos();
    	int32_t stepsRight	=	right_motor_get_pos();
    	int16_t actualSpeedLeft = (stepsLeft-previousStepsLeft)*1000.0f/REGULATORPERIOD;			//1000 ms per second
    	int16_t actualSpeedRight = (stepsRight-previousStepsRight)*1000.0f/REGULATORPERIOD;
    	previousStepsLeft = stepsLeft;
    	previousStepsRight = stepsRight;
    	//catching overflow of stepscounter
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
    	float estimatedForwardVelocity		=	((filtredSpeedLeft+filtredSpeedRight)*2*PI*WHEEL_RADIUS)*50/COUNTSPERTURN;
    	float estimatedRotationalVelocity	= 	(filtredSpeedRight-filtredSpeedLeft)*2*PI*WHEEL_RADIUS/WHEEL_DISTANCE/COUNTSPERTURN;
    	bearingPhi+=REGULATORPERIOD*estimatedRotationalVelocity/1000.0f;

    	xPosition = xPosition+cosf(bearingPhi)*estimatedForwardVelocity*REGULATORPERIOD/1000.0f;
    	yPosition = yPosition+sinf(bearingPhi)*estimatedForwardVelocity*REGULATORPERIOD/1000.0f;
    	bearingPhi=wrapAngle(bearingPhi);
    	//chprintf((BaseSequentialStream *) &SD3,"vel:%f filteredLeft:%d filteredRight:%d rawLeft %d rawRight %d \n\r",estimatedForwardVelocity,filtredSpeedLeft,filtredSpeedRight, actualSpeedLeft,actualSpeedRight);
    	chThdSleepUntilWindowed(time,time+MS2ST(REGULATORPERIOD));

    }
}

float wrapAngle(float angle)
{
	angle =fmod(angle+PI,2*PI);
	if(angle<0)
	{
		angle +=2*PI;
	}
	return angle-PI;
}




//**********public functions****************

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
