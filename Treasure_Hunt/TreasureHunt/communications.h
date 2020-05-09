#ifndef COMMUNICATIONS_H
#define COMMUNICATIONS_H
typedef struct{
	float xPosition;
	float yPosition;
	float directionOfSound;
	float avoidanceDirection;
	float obstacleDirection;
	bool updated;

}PCMessage_t;
/**
 * @brief: function which sends float to the computer
 * @author: epfl
 */
void SendFloatToComputer(BaseSequentialStream* out, float* data, uint16_t size);


/**
 * @brief: signals that the message is ready to be sent.
 * @param[in] isReady set to true if message should be send
 */
void messageReady(bool isReady);
/**
 * @brief: sets the buffer of data which should be send
 *
 * @param[in] messageToSend pointer to the buffer containing the data to be send
 */
void setMessage (PCMessage_t *messageToSend);

/**
 * @brief: sends the data in the buffer specified by setMessage to the computer
 *
 * @return 1 if successfuly send,  0 otherwise
 */
uint8_t sendEventDataToComputer(void);

#endif /* COMMUNICATIONS_H */
