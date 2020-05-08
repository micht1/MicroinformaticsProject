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
bool doSendDebugData=false;

//***defines
#define SENSORTRIGGERHIGHVALUE 200
#define SENSORTRIGGERLOWVALUE 100
#define OBSTACLECLOSE 700
#define SENSORCORRECTION 0
#define IRFILTER 0.5
#define SENSORMAXVALUE 3000
#define BEHINDAREA 180


#define IR1 0
#define IR2 1
#define IR4 3
#define IR5 4
#define IR7 6
#define IR8 7


//*****constants
const int8_t IRSensorDegree[PROXIMITY_NB_CHANNELS]={-17,-49,-90,-120,120,90,49,17};




//********static variables
static IRData processingValues={0};

static THD_WORKING_AREA(IRSensorProcessing_wa, 1024);
static THD_FUNCTION(IRSensorProcessing, arg) {
     (void) arg;
     chRegSetThreadName(__FUNCTION__);

     messagebus_topic_t *IRValuesTopic;
     IRValuesTopic=messagebus_find_topic(&bus, "/proximity");
     proximity_msg_t irValues;
     int sensorValues[PROXIMITY_NB_CHANNELS]={0};
     presenceOfObstacle_t triggeredSensors[PROXIMITY_NB_CHANNELS]={0};
     while(true)
     {
    	 messagebus_topic_wait(IRValuesTopic, &irValues, sizeof(irValues));

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

					 //chprintf((BaseSequentialStream *)&SD3,"sensor%d: %d",sensorCounter,sensorValues[sensorCounter]);
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
    	 uint8_t nbOfTriggeredSensors=0;

    	 //chprintf((BaseSequentialStream *)&SD3,"value: %d\n\r",processingValues.directionOfObstacle);
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
    	 if((triggeredSensors[IR4]>NOOBSTACLE && triggeredSensors[IR5]>NOOBSTACLE))
		 {
			 processingValues.directionOfObstacle=(BEHINDAREA);
			 nbOfTriggeredSensors=1;
		 }

    	// chprintf((BaseSequentialStream *)&SD3,"n: %u,sV: %d,presence %d\n\r",nbOfTriggeredSensors,sensorValues[2],processingValues.obstaclePresence);
    	 if(nbOfTriggeredSensors==0)
    	 {
    		 processingValues.obstaclePresence=NOOBSTACLE;
    	 }
    	 else
    	 {
    		 if(processingValues.ignoringObstacle==false)
    		 {
    			 isAllowedToDrive(false);
    		 }
    	 }
    	 if(triggeredSensors[IR1]>NOOBSTACLE || triggeredSensors[IR2]>NOOBSTACLE ||triggeredSensors[IR7]>NOOBSTACLE ||triggeredSensors[IR8]>NOOBSTACLE)
    	 {
    		 processingValues.aheadIsOK=false;
    		 //chprintf((BaseSequentialStream *)&SD3,"0: %u,1: %u,6: %u,7: %u,\n\r",irValues.delta[0],irValues.delta[1],irValues.delta[6],irValues.delta[7]);
    	 }
    	 else
    	 {
    		 processingValues.aheadIsOK=true;
    	 }
    	 processingValues.directionOfObstacle=processingValues.directionOfObstacle/nbOfTriggeredSensors;


		 /*if(nbOfTriggeredSensors>0)
    	 {
    		 chprintf((BaseSequentialStream *)&SD3,"direction: %d\n\r",processingValues.directionOfObstacle);
    	 }*/
    	 //chprintf((BaseSequentialStream *)&SD3,"sensorVlue: %u\n\r",irValues.delta[0]);
    	// chprintf((BaseSequentialStream *)&SD3,"angle: %d\n\r",processingValues.directionOfObstacle);
    	// chprintf((BaseSequentialStream *)&SD3,"angle: %d, obstaclePresence: %d,ahead : %d\n\r",processingValues.directionOfObstacle,processingValues.obstaclePresence,processingValues.aheadIsOK);

    	 //chprintf((BaseSequentialStream *)&SD3,"Published present: %s, Value: %d\n\r",processingValues.obstaclePresent ? "true" : "false",processingValues.directionOfObstacle);
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
