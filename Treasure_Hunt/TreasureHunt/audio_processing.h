#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H


#define FFT_SIZE 	1024

#define FREQUENCYCOEFFIZIENT 15.23
#define MINSOUNDLEVEL 6000

#define FREQUENCYTOFIND 2000	//
#define MAXPHASEANGLE 3.1f		//might need checking
#define SPEEDOFSOUND 343.2f
#define MICDISTANCEFRONTBACK 0.056f
#define MICDISTANCELEFTRIGHT 0.062f
#define NBOFPHASES 2
#define FILTERCOEFFIZIENT 0.06f
#define MAXANGLEDIFF M_PI/2
#define WEAKWEIGHT 0.2f
#define STRONGWEIGHT (1-WEAKWEIGHT)
#define LEFTRIGHTMICCORRECTION 0.15f
#define FRONTBACKMICCORRECTION 0.15f



//#define NBOFAVERGINGPOINTS 12
typedef enum
{
	LEFTRIGHT1=0,					//chose Solution for -90� <alpha< 90
	LEFTRIGHT2,					// chose Solution for alpha =[180...90] or alpha = [-180 ... -90]
	FRONTBACKP,					//chose solution for 0<beta<180�
	FRONTBACKN,					//Chose solution for 0>beta>-180
	SOLIUTIONERROR=500					//error. random number
}correctSolution_t;



typedef struct{
	float realPart;
	float imaginaryPart;
} complexNumber_t;

void processAudioData(int16_t *data, uint16_t num_samples);

/*
*	put the invoking thread into sleep until it can process the audio datas
*/
void wait_send_to_computer(void);


float calculateMaxFrequency(float *frequencyBuffer,uint16_t bufferSize);
float getToneFrequency(void);
uint32_t getSoundLevel(void);
float getRelativeAngle(void);


#endif /* AUDIO_PROCESSING_H */
