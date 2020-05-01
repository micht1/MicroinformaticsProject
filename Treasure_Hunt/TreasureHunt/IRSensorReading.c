#include <stdio.h>
#include <string.h>
#include "ch.h"
#include "hal.h"
#include <stdlib.h>
#include "main.h"
#include <chprintf.h>

#include "sensors/proximity.h"

#include "IRSensorReading.h"


//***defines
#define SENSORTRIGGERVALUE 500
#define SENSORISTIRGGERED 1


//*****constants
const int8_t IRSensorDegree[PROXIMITY_NB_CHANNELS]={15,45,90,110,-110,-90,-45,-15};




//********static variables


static IRData processingValues;
static float objectDirection=0;

static THD_WORKING_AREA(IRSensorProcessing_wa, 512);
static THD_FUNCTION(IRSensorProcessing, arg) {
     (void) arg;
     chRegSetThreadName(__FUNCTION__);

     messagebus_topic_t *IRValuesTopic;
     IRValuesTopic=messagebus_find_topic(&bus, "/proximity");
     proximity_msg_t irValues;
     while(true)
     {
    	 messagebus_topic_wait(IRValuesTopic, &irValues, sizeof(irValues));
    	 uint8_t nbOfTriggeredSensors=0;
    	 processingValues.directionOfObstacle=0;
    	 for(uint8_t sensorCounter=0;sensorCounter<PROXIMITY_NB_CHANNELS;sensorCounter++)
    	 {
    		 if(irValues.delta[sensorCounter]>SENSORTRIGGERVALUE)
    		 {
    			 processingValues.directionOfObstacle+=IRSensorDegree[sensorCounter];
    			 processingValues.obstaclePresent=true;
    			 nbOfTriggeredSensors++;
    		 }

    	 }
    	 if(nbOfTriggeredSensors==0)
    	 {
    		 processingValues.obstaclePresent=false;
    	 }

    	 processingValues.directionOfObstacle= processingValues.directionOfObstacle/nbOfTriggeredSensors;


    	 //chprintf((BaseSequentialStream *)&SD3,"Published present: %s, Value: %d\n\r",processingValues.obstaclePresent ? "true" : "false",processingValues.directionOfObstacle);

     }



}






//***************Public functions *****************************************
void IRProcessingStart(void)
{
	proximity_start();
	chThdCreateStatic(IRSensorProcessing_wa, sizeof(IRSensorProcessing_wa), NORMALPRIO, IRSensorProcessing, NULL);

}
bool isObstaclePresent(void)
{
	return processingValues.obstaclePresent;
}

void ignoreObstacle(bool doIgnoreObstacle)
{
	processingValues.ignoringObstacle=doIgnoreObstacle;
}

float getObjectDirection(void)
{
	return objectDirection;
}
