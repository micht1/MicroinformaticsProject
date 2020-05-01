#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>
#include "math.h"
#include "driveMotors.h"
#include <arm_math.h>
#include "sensors/VL53L0X/VL53L0X.h"
#include "freeDirectionDetection.h"


static bool doScan=false;
static scanningStep_t scanStatus=0;
static float startingBearing=0;
static float endingBearing=0;

static float bearingFC=0;
static float bearingCF=0;



static THD_WORKING_AREA(freeDirection_thd_was, 256);
static THD_FUNCTION(freeDirection_thd, arg)
{
	(void) arg;
	chRegSetThreadName(__FUNCTION__);
	uint16_t currentDistance=0;
	uint16_t oldDistance=0;
	int16_t minDelta=0;
	float minBearing=0;
	int16_t maxDelta=0;
	float maxBearing=0;
	while(1)
	{
		switch(scanStatus)
		{
		case IDLE:
			if(doScan==true)
			{
				setDesiredBearing(startingBearing);
				scanStatus=ROTATINGTOSTART;
				bearingCF=0;
				bearingFC=0;
			}
			break;
		case ROTATINGTOSTART:
			if(fabs(getCurrentBearing()-startingBearing)<STARTINGTOLERANCE && doScan==true)
			{
				scanStatus=SCANNINGROTATION;
				limitWheelSpeed(SCANWHEELSPEED);
				setDesiredBearing(endingBearing);
			}
			else if(doScan==false)
			{
				scanStatus=IDLE;
			}
			break;
		case SCANNINGROTATION:
			currentDistance=VL53L0X_get_dist_mm();
			if((oldDistance -currentDistance)>minDelta && oldDistance!=0)
			{
				minDelta=oldDistance - currentDistance;
				minBearing=getCurrentBearing();
			}
			if((oldDistance -currentDistance)<maxDelta && oldDistance!=0)
			{
				maxDelta=oldDistance - currentDistance;
				maxBearing=getCurrentBearing();
			}
			oldDistance=currentDistance;
			chprintf((BaseSequentialStream *) &SD3,"Delta: %d, current %u, ol %u\n\r",oldDistance-currentDistance,currentDistance,oldDistance);
			if((fabs(getCurrentBearing()-endingBearing)<ENDINGTOLERANCE))
			{
				scanStatus=FINISHEDSCANNING;
				setDesiredBearing(getCurrentBearing());
				bearingFC=maxBearing;
				bearingCF=minBearing;
				limitWheelSpeed(MAXWHEELSPEED);
			}
			break;
		case FINISHEDSCANNING:
			doScan=false;
			break;

		}
		//chprintf((BaseSequentialStream *) &SD3,"status: %u\n\r",scanStatus);
		//chprintf((BaseSequentialStream *) &SD3,"status :%u, bearing: %f, currentdistance %u\n\r",scanStatus, getCurrentBearing(),VL53L0X_get_dist_mm());
		chThdSleep(MS2ST(100));
	}


}
void doScanning(bool shouldScan)
{

	doScan=shouldScan;
	if(shouldScan==true)
	{
		scanStatus=IDLE;
	}
}
scanningStep_t getScanStatus(void)
{
	return scanStatus;
}
void setScanningRange(float beginningBearing, float endBearing)
{
	if(doScan==false)
	{
		startingBearing=beginningBearing;
		endingBearing=endBearing;
		//scanStatus=IDLE;
	}
}
void getFreeBearing(float *freeBearings,uint8_t bearingSize)
{
	if(bearingSize==NBOFBEARINGS)
	{
		freeBearings[0]=bearingFC;
		freeBearings[1]=bearingCF;
	}
}
void startDirectionDetectionThread(void)
{
	VL53L0X_start();
	chThdCreateStatic(freeDirection_thd_was, sizeof(freeDirection_thd_was), NORMALPRIO, freeDirection_thd, NULL);
}
