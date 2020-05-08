#ifndef FREEDIRECTIONDETECTION_H
#define FREEDIRECTIONDETECION_H

#define MINDETECTIONDELTA 40
#define STARTINGTOLERANCE 0.02f
#define ENDINGTOLERANCE	0.02f
#define NBOFBEARINGS 2
#define MAXDISTANCE 2000

#define SCANWHEELSPEED 200

typedef enum {IDLE=0,ROTATINGTOSTART,SCANNINGROTATION,FINISHEDSCANNING}scanningStep_t;

void setScanningRange(float beginningAngle, float endAngle);
void doScanning(bool shouldScan);
scanningStep_t getScanStatus(void);
void getFreeBearing(float *freeBearings,uint8_t bearingSize);
void startDirectionDetectionThread(void);
#endif
