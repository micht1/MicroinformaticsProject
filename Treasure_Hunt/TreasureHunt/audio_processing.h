#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H


#define FFT_SIZE 	1024

#define FREQUENCYCOEFFIZIENT 15.23			// coeffizient which links the position in the FFT result to a frequency
#define MINSOUNDLEVEL 6000					// sound level which the searched for frequency has to reach in order to search for it.

#define FREQUENCYTOFIND 2000	// frequency which will be searched for
#define MAXPHASEANGLE 3.1f		//maximum possible Phasedifference between Microphone signals
#define SPEEDOFSOUND 343.2f
#define MICDISTANCEFRONTBACK 0.056f
#define MICDISTANCELEFTRIGHT 0.062f
#define NBOFPHASES 2			// number of phase differences calculated
#define FILTERCOEFFIZIENT 0.06f	// filter coeffizient used to filter the phasedifferences
#define WEAKWEIGHT 0.2f			// weak weight used in the calculation of an weighted average
#define STRONGWEIGHT (1-WEAKWEIGHT)




//#define NBOFAVERGINGPOINTS 12
typedef enum
{
	LEFTRIGHT1=0,					//chose Solution for -90� <alpha< 90
	LEFTRIGHT2,					// chose Solution for alpha =[180...90] or alpha = [-180 ... -90]
	FRONTBACKP,					//chose solution for 0<beta<180�
	FRONTBACKN,					//Chose solution for 0>beta>-180
	SOLIUTIONERROR=500					//error. random number
}correctSolution_t;



typedef struct{					//struct used to save a complex number
	float realPart;
	float imaginaryPart;
} complexNumber_t;
/**
 * @brief: callback function, which is called everytime new data is available from the microphones.
 * Calculates the angle from where the sound comes
 * @author: EPFL, Fanny Borza, Tobias Michel
 */
void processAudioData(int16_t *data, uint16_t num_samples);
/**
 * @brief: getter Function for the sound level of the searched for frequency
 * @return averaged value of the sound level of the searched for frequency
 */
uint32_t getSoundLevel(void);
/**
 * @brief: getter function for the angle with which the sound arrives at the robot.
 * The angle is relative to the front of the robot. So if the sound comes from the front an angle of 0 is calculated
 * @return: the angle of the sound in radiants
 */
float getRelativeAngle(void);


#endif /* AUDIO_PROCESSING_H */
