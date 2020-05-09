#ifndef FREEDIRECTIONDETECTION_H
#define FREEDIRECTIONDETECION_H

#define MINDETECTIONDELTA 40		//minimum difference between 2 measurement points to be considered as a possible direction to travel
#define STARTINGTOLERANCE 0.02f		// allignement tolerance to start the scan
#define ENDINGTOLERANCE	0.02f		// alignement tolerance to consider the scan to be at the end
#define NBOFBEARINGS 2				// number of direction which should be saved as possible travel directions
#define MAXDISTANCE 2000			// any distance measurement which gives more than 2000 mm is considered faulty

#define SCANWHEELSPEED 200			// speed of the wheels which should be used for scanning rotation

typedef enum {IDLE=0,ROTATINGTOSTART,SCANNINGROTATION,FINISHEDSCANNING}scanningStep_t;	//Enumeration used to track the status where in the scanning process the robot is

/**
 * @brief: setter function, which between the robot should scan, the robot scan always counter clock wise
 *
 * @param[in] beginningAngle: starting angle in radiants. Angle where the scan should start
 * @param[in] endAngle: angle where the scan should end
 */
void setScanningRange(float beginningAngle, float endAngle);
/**
 * @brief: function used to start/stop the scanning process
 *
 * @param[in] shouldScan: set to true when the robot should scan, set to false if he should stop
 */
void doScanning(bool shouldScan);

/**
 * @brief: function that returns the status of the scan
 *
 * @return: status of the scan. FINISHEDSCANNING when the robot finisheed scanning
 */
scanningStep_t getScanStatus(void);
/**
 * @brief: returns directions in which travel should be possible
 *
 * @param[out] freeBearing: array in which the directions should be saved
 * @param[in] bearingSize: the size of the array in which the directions should be saved
 */
void getFreeBearing(float *freeBearings,uint8_t bearingSize);
/**
 * @brief starts the thread with normal prio. thread does nothing until the doScan function is called with true
 */
void startDirectionDetectionThread(void);
#endif
