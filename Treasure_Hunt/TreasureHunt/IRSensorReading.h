#ifndef IRSENSOREADING_H
#define IRSENSOREADING_H
#include "msgbus/messagebus.h"

typedef enum {NOOBSTACLE,OBSTACLEDETECTED,DANGERCLOSE}presenceOfObstacle_t;	//enumeration with which the status of the obstacle detection is described
/**
 * The struct in which the data of the Obstacle direction detection is saved
 */

typedef struct {
	presenceOfObstacle_t obstaclePresence;
	bool ignoringObstacle;
	bool aheadIsOK;					//
	int32_t directionOfObstacle;	//direction in degrees
}IRData;

#define BEHINDAREA 180
extern messagebus_t bus;

/**
 * @brief: starts the thread used for IR-Sensor detection with Normal Priority and the proximity-thread
 * The thread is directly linked to the proximity-thread. It only runs when new data is aviable.
 *  		 *0*
 * 		   *	 *
 * 		 90		 -90
 * 		   *	 *
 * 		    *180*
 */
void IRProcessingStart(void);

/**
 * @brief
 * @return NOOBSTACLE if no obstacle detected, OBSTACLE detected if an obstacle is detected and DANGERCLOSE if an obstacle is very close
 */
presenceOfObstacle_t presenceOfObstacle(void);
/**
 * @brief: The thread stops the forward motion if an obstacle is detected, this function disables this,
 * which means the robot can move even when an obstacle is detected
 * @param[in]: bool set to true if obstacle should be ignored
 */
void ignoreObstacle(bool doIgnoreObstacle);
/**
 * @brief: function that return true if one of the 4 front sensors are triggered.
 *
 * @return: true if IR1,IR2,IR7,IR8 detects an obstacle, false otherwise
 */
bool aheadIsOk(void);
/**
 * @brief: returns the direction of the obstacle with the data provided by the proximity-thread.
 *  		 -0-
 * 		   *	 *
 * 		 90		 -90
 * 		   *	 *
 * 		    -180-
 * @return direction in degrees, directly to the front is 0°
 *
 */
float getObstacleDirection(void);

#endif

