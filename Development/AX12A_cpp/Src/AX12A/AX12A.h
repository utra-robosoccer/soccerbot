/*
 * AX12A.h
 *
 *  Created on: Mar 16, 2018
 *      Author: Admin
 */




/************************* Prevent recursive inclusion ************************/
#ifndef AX12A_H_
#define AX12A_H_




/********************************** Includes **********************************/
#include "../Dynamixel/Dynamixel.h"

/* Instruction set definitions. */
extern const uint8_t INST_PING;
extern const uint8_t INST_READ_DATA;
extern const uint8_t INST_WRITE_DATA;
extern const uint8_t INST_REG_WRITE;
extern const uint8_t INST_ACTION;
extern const uint8_t INST_RESET;
extern const uint8_t INST_SYNC_WRITE;

/* Default register value definitions. */
extern const uint8_t BROADCAST_ID;
extern const uint8_t DEFAULT_ID;
extern const uint8_t DEFAULT_BAUD_RATE;
extern const uint8_t DEFAULT_RETURN_DELAY;
extern const uint16_t DEFAULT_CW_ANGLE_LIMIT;
extern const uint16_t DEFAULT_CCW_ANGLE_LIMIT;
extern const uint8_t DEFAULT_HIGH_VOLTAGE_LIMIT;
extern const uint8_t DEFAULT_LOW_VOLTAGE_LIMIT;
extern const uint16_t DEFAULT_MAXIMUM_TORQUE;
extern const uint8_t DEFAULT_STATUS_RETURN_LEVEL;
extern const uint8_t DEFAULT_ALARM_LED;
extern const uint8_t DEFAULT_ALARM_SHUTDOWN;
extern const uint8_t DEFAULT_TORQUE_ENABLE;
extern const uint8_t DEFAULT_LED_ENABLE;
extern const uint8_t DEFAULT_CW_COMPLIANCE_MARGIN;
extern const uint8_t DEFAULT_CCW_COMPLIANCE_MARGIN;
extern const uint8_t DEFAULT_CW_COMPLIANCE_SLOPE;
extern const uint8_t DEFAULT_CCW_COMPLIANCE_SLOPE;
extern const uint8_t DEFAULT_EEPROM_LOCK;
extern const uint16_t DEFAULT_PUNCH;




/********************************** Classes ***********************************/
class AX12A: public Dynamixel {
public:
	/********** Methods **********/
	// Constructor and destructor
	AX12A(MotorInitData* motorInitData);
	virtual ~AX12A();
	int Init();

	// Low-level transmission and reception
	void dataWriter(uint8_t arrSize, uint8_t writeAddr, uint8_t param1, uint8_t param2);
	uint16_t dataReader(uint8_t readAddr, uint8_t readLength);
	uint8_t computeChecksum(uint8_t *arr, int length);

	// Setters (use the WRITE DATA instruction)
	void setGoalTorque(double goalTorque); // (RAM)
	void setMaxTorque(double maxTorque); // (EEPROM)
	void setAlarmLED(uint8_t alarm_LED_data); // (EEPROM)
	void setAlarmShutdown(uint8_t alarm_shutdown_data); // (EEPROM)
	void setCWComplianceMargin(uint8_t CWcomplianceMargin); // (RAM)
	void setCCWComplianceMargin(uint8_t CCWcomplianceMargin); // (RAM)
	void setCWComplianceSlope(uint8_t CWcomplianceSlope); // (RAM)
	void setCCWComplianceSlope(uint8_t CCWcomplianceSlope); // (RAM)
	void setEEPROMLock(); // (RAM)
	void setPunch(double punch); // (RAM)
	void reset();

	void setStatusReturnLevel(uint8_t status_data); // (EEPROM in AX-12A)

	// Getters (use the READ DATA instruction)
	bool isInJointMode();

	// Interfaces for previously-defined functions
	void revive(uint8_t ID); // TODO: Check this over for correctness in the context of several motors
	void broadcastRevive(uint8_t ID); // TODO: Check this over for correctness in the context of several motors
	void enterWheelMode(double goalVelocity);
	void enterJointMode();
	void setComplianceSlope(uint8_t complianceSlope);
	void setComplianceMargin(uint8_t complianceMargin);
};

#endif /* AX12A_H_ */
