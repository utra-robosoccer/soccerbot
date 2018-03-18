/*
 * MX28.h
 *
 *  Created on: Mar 17, 2018
 *      Author: Admin
 */

#ifndef MX28_MX28_H_
#define MX28_MX28_H_

class MX28: public Dynamixel {
public:
	/********** Properties **********/
	uint16_t 			lastTick;					/*!< Time tick (ms) read from motor					*/
	float	 			lastPWM;					/*!< PWM output as a percentage 					*/
	uint32_t 			lastVelocityTrajectory;		/*!< --												*/
	uint32_t 			lastPositionTrajecctory;	/*!< --												*/


	/********** Methods **********/
	// Constructor and destructor
	MX28();
	virtual ~MX28();

	// Low-level transmission and reception
	void dataWriter(uint8_t arrSize, uint8_t writeAddr, uint8_t param1, uint8_t param2);
	uint16_t dataReader(uint8_t readAddr, uint8_t readLength);
	void computeChecksum(uint8_t *arr, int length);


	// Setters (use the WRITE DATA instruction)
	void setDriveMode(); // (EEPROM)
	void setOperatingMode(); // (EEPROM)
	void setShadowID(); // (EEPROM)
	void setProtocolVersion(); // (EEPROM)
	void setHomingOffset(); // (EEPROM)
	void setMovingThreshold(); // (EEPROM)
	void setPWMLimit(); // (EEPROM)
	void setAccelerationLimit(); // (EEPROM)
	void setVelocityLimit(); // (EEPROM)
	void setShutdownConditions(); // (EEPROM)

	void setHardwareErrorStatus(); // (RAM)
	void setVelocityIGain(); // (RAM)
	void setVelocityPGain(); // (RAM)
	void setPositionDGain(); // (RAM)
	void setPositionIGain(); // (RAM)
	void setPositionPGain(); // (RAM)
	void setFeedforwardGain2(); // (RAM)
	void setFeedforwardGain1(); // (RAM)
	void setBusWatchdog(); // (RAM)
	void setGoalPWM(); // (RAM)
	void setAccelerationprofile(); // (RAM)
	void setVelocityProfile(); // (RAM)


	// Getters (use the READ DATA instruction)
	uint16_t getTick();
	uint8_t getMovingStatus();
	void getPresentPWM();
	void getVelocityTrajectory();
	void getPositionTrajectory();
};

#endif /* MX28_MX28_H_ */
