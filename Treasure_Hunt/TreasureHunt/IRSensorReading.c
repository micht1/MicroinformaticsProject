#include <stdio.h>
#include <string.h>
#include "ch.h"
#include "hal.h"
#include <stdlib.h>
#include "main.h"
#include <chprintf.h>
#include "driveMotors.h"
#include "sensors/proximity.h"
#include "IRSensorReading.h"

//***defines
#define SENSORTRIGGERHIGHVALUE 200		// value at which the sensor is considered as  triggered
#define SENSORTRIGGERLOWVALUE 100		// value under which the sensor is considered not triggered anymore
#define OBSTACLECLOSE 700				// value over which the obstacle is considered, very close
#define SENSORCORRECTION 0			//value used if different behaviour is desired from the front side and the 2 back side sensor
#define IRFILTER 0.5				// value for the simple lowpass filter y = IRFILTER*x+(1-IRFILTER)*y
#define SENSORMAXVALUE 3000			// value over which the data from the sensor is considered nonsense
#define BEHINDAREA 180				//value in degress when both backside senors are triggered

// some defines to align the IR-sensor number on the robot with the number in the programm
#define IR1 0
#define IR2 1
#define IR4 3
#define IR5 4
#define IR7 6
#define IR8 7


//*****constants
// array containing the angles with which the sensors are monted on the robot. They are in degrees
const int8_t IRSensorDegree[PROXIMITY_NB_CHANNELS]={-17,-49,-90,-120,120,90,49,17};




//********static variables
static IRData processingValues={0};

/**
 * beginning of the thread
 */
static THD_WORKING_AREA(IRSensorProcessing_wa, 256);
static THD_FUNCTION(IRSensorProcessing, arg) {
     (void) arg;
     chRegSetThreadName(__FUNCTION__);

     messagebus_topic_t *IRValuesTopic;
     IRValuesTopic=messagebus_find_topic(&bus, "/proximity");
     proximity_msg_t irValues;

     int sensorValues[PROXIMITY_NB_CHANNELS]={0};					// array used to save filtered values of the sensors
     presenceOfObstacle_t triggeredSensors[PROXIMITY_NB_CHANNELS]={0};	//array used to save the trigger status of the sensor
     while(true)
     {
    	 messagebus_topic_wait(IRValuesTopic, &irValues, sizeof(irValues));		//wait for new data

    	 // go through all sensors and check which one is triggered, which one is not after the values where somewhat filtered
    	 for(uint8_t sensorCounter=0;sensorCounter<PROXIMITY_NB_CHANNELS;sensorCounter++)
		 {
    		 if(get_calibrated_prox(sensorCounter)<SENSORMAXVALUE)
    		 {
				 sensorValues[sensorCounter]=get_calibrated_prox(sensorCounter)*IRFILTER+(1-IRFILTER)*sensorValues[sensorCounter];
				 int16_t triggerCorrection=0;
				 if(sensorCounter==IR1 || sensorCounter==IR8 || sensorCounter==IR5 || sensorCounter==IR4 )	//since not all sensor behave the same, notably the frontsensors(1,8) are much more sensitive, than the rest
				 {
					 triggerCorrection=SENSORCORRECTION;
				 }
				 if(sensorValues[sensorCounter]>(OBSTACLECLOSE+triggerCorrection))
				 {
					 triggeredSensors[sensorCounter]=DANGERCLOSE;
				 }
				 else if(sensorValues[sensorCounter]>(SENSORTRIGGERHIGHVALUE+triggerCorrection))
				 {

					 triggeredSensors[sensorCounter]=OBSTACLEDETECTED;

				 }
				 else if(sensorValues[sensorCounter]<(SENSORTRIGGERLOWVALUE+triggerCorrection))
				 {
					 triggeredSensors[sensorCounter]=NOOBSTACLE;
				 }
				 if(triggeredSensors[sensorCounter]>NOOBSTACLE)
				 {
					 processingValues.directionOfObstacle=0;
				 }
    		 }
    		 else
    		 {
    			 processingValues.directionOfObstacle=0;
    		 }
		 }
    	 // go through all sensors and check which are triggered. save the status and calculate the angle of the obstacle
    	 uint8_t nbOfTriggeredSensors=0;
    	 for(uint8_t sensorCounter=0;sensorCounter<PROXIMITY_NB_CHANNELS;sensorCounter++)
		 {
    		 if(triggeredSensors[sensorCounter]>NOOBSTACLE)
    		 {

    			 nbOfTriggeredSensors++;


				 processingValues.directionOfObstacle+=IRSensorDegree[sensorCounter];


    			 if(triggeredSensors[sensorCounter]>processingValues.obstaclePresence)
    			 {
    				 processingValues.obstaclePresence=triggeredSensors[sensorCounter];
    			 }
    		 }
		 }
    	 //catch an exception when +120° and -120° are triggered
    	 if((triggeredSensors[IR4]>NOOBSTACLE && triggeredSensors[IR5]>NOOBSTACLE))
		 {
			 processingValues.directionOfObstacle=(BEHINDAREA);
			 nbOfTriggeredSensors=1;
		 }
    	 //make sure if no sensors are triggered the status is NOOBSTACLE
    	 if(nbOfTriggeredSensors==0)
    	 {
    		 processingValues.obstaclePresence=NOOBSTACLE;
    	 }
    	 else
    	 {
    		 //stop the robot if any obstalce is detected
    		 if(processingValues.ignoringObstacle==false)
    		 {
    			 isAllowedToDrive(false);
    		 }
    	 }
    	 // provide additional information
    	 if(triggeredSensors[IR1]>NOOBSTACLE || triggeredSensors[IR2]>NOOBSTACLE ||triggeredSensors[IR7]>NOOBSTACLE ||triggeredSensors[IR8]>NOOBSTACLE)
    	 {
    		 processingValues.aheadIsOK=false;
    	 }
    	 else
    	 {
    		 processingValues.aheadIsOK=true;
    	 }
    	 processingValues.directionOfObstacle=processingValues.directionOfObstacle/nbOfTriggeredSensors;
     }
}






//***************Public functions *****************************************
void IRProcessingStart(void)
{
	proximity_start();
	calibrate_ir();
	chThdCreateStatic(IRSensorProcessing_wa, sizeof(IRSensorProcessing_wa), NORMALPRIO, IRSensorProcessing, NULL);

}
presenceOfObstacle_t presenceOfObstacle(void)
{

	return processingValues.obstaclePresence;
}

void ignoreObstacle(bool doIgnoreObstacle)
{
	processingValues.ignoringObstacle=doIgnoreObstacle;
}

float getObstacleDirection(void)
{

	return  (float)processingValues.directionOfObstacle;
}
bool aheadIsOk(void)
{

	return processingValues.aheadIsOK;
}
