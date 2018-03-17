
#ifndef MPU_REGS_H_
#define MPU_REGS_H_

#include "stm32h7xx_hal.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>
#include <stdlib.h>

class MPU6050 {
    public:
        MPU6050();
        MPU6050(uint8_t address);

        void initialize();
        bool testConnection();

        // AUX_VDDIO register
        uint8_t getAuxVDDIOLevel();
        void setAuxVDDIOLevel(uint8_t level);

        // SMPLRT_DIV register
        uint8_t getRate();
        void setRate(uint8_t rate);

        // CONFIG register
        uint8_t getExternalFrameSync();
        void setExternalFrameSync(uint8_t sync);
        uint8_t getDLPFMode();
        void setDLPFMode(uint8_t bandwidth);

        // GYRO_CONFIG register
        uint8_t getFullScaleGyroRange();
        void setFullScaleGyroRange(uint8_t range);

        // ACCEL_CONFIG register
        bool getAccelXSelfTest();
        void setAccelXSelfTest(bool enabled);
        bool getAccelYSelfTest();
        void setAccelYSelfTest(bool enabled);
        bool getAccelZSelfTest();
        void setAccelZSelfTest(bool enabled);
        uint8_t getFullScaleAccelRange();
        void setFullScaleAccelRange(uint8_t range);
        uint8_t getDHPFMode();
        void setDHPFMode(uint8_t mode);

        // FF_THR register
        uint8_t getFreefallDetectionThreshold();
        void setFreefallDetectionThreshold(uint8_t threshold);

        // FF_DUR register
        uint8_t getFreefallDetectionDuration();
        void setFreefallDetectionDuration(uint8_t duration);

        // MOT_THR register
        uint8_t getMotionDetectionThreshold();
        void setMotionDetectionThreshold(uint8_t threshold);

        // MOT_DUR register
        uint8_t getMotionDetectionDuration();
        void setMotionDetectionDuration(uint8_t duration);

        // ZRMOT_THR register
        uint8_t getZeroMotionDetectionThreshold();
        void setZeroMotionDetectionThreshold(uint8_t threshold);

        // ZRMOT_DUR register
        uint8_t getZeroMotionDetectionDuration();
        void setZeroMotionDetectionDuration(uint8_t duration);

        // FIFO_EN register
        bool getTempFIFOEnabled();
        void setTempFIFOEnabled(bool enabled);
        bool getXGyroFIFOEnabled();
        void setXGyroFIFOEnabled(bool enabled);
        bool getYGyroFIFOEnabled();
        void setYGyroFIFOEnabled(bool enabled);
        bool getZGyroFIFOEnabled();
        void setZGyroFIFOEnabled(bool enabled);
        bool getAccelFIFOEnabled();
        void setAccelFIFOEnabled(bool enabled);
        bool getSlave2FIFOEnabled();
        void setSlave2FIFOEnabled(bool enabled);
        bool getSlave1FIFOEnabled();
        void setSlave1FIFOEnabled(bool enabled);
        bool getSlave0FIFOEnabled();
        void setSlave0FIFOEnabled(bool enabled);

        // I2C_MST_CTRL register
        bool getMultiMasterEnabled();
        void setMultiMasterEnabled(bool enabled);
        bool getWaitForExternalSensorEnabled();
        void setWaitForExternalSensorEnabled(bool enabled);
        bool getSlave3FIFOEnabled();
        void setSlave3FIFOEnabled(bool enabled);
        bool getSlaveReadWriteTransitionEnabled();
        void setSlaveReadWriteTransitionEnabled(bool enabled);
        uint8_t getMasterClockSpeed();
        void setMasterClockSpeed(uint8_t speed);

        // I2C_SLV* registers (Slave 0-3)
        uint8_t getSlaveAddress(uint8_t num);
        void setSlaveAddress(uint8_t num, uint8_t address);
        uint8_t getSlaveRegister(uint8_t num);
        void setSlaveRegister(uint8_t num, uint8_t reg);
        bool getSlaveEnabled(uint8_t num);
        void setSlaveEnabled(uint8_t num, bool enabled);
        bool getSlaveWordByteSwap(uint8_t num);
        void setSlaveWordByteSwap(uint8_t num, bool enabled);
        bool getSlaveWriteMode(uint8_t num);
        void setSlaveWriteMode(uint8_t num, bool mode);
        bool getSlaveWordGroupOffset(uint8_t num);
        void setSlaveWordGroupOffset(uint8_t num, bool enabled);
        uint8_t getSlaveDataLength(uint8_t num);
        void setSlaveDataLength(uint8_t num, uint8_t length);

        // I2C_SLV* registers (Slave 4)
        uint8_t getSlave4Address();
        void setSlave4Address(uint8_t address);
        uint8_t getSlave4Register();
        void setSlave4Register(uint8_t reg);
        void setSlave4OutputByte(uint8_t data);
        bool getSlave4Enabled();
        void setSlave4Enabled(bool enabled);
        bool getSlave4InterruptEnabled();
        void setSlave4InterruptEnabled(bool enabled);
        bool getSlave4WriteMode();
        void setSlave4WriteMode(bool mode);
        uint8_t getSlave4MasterDelay();
        void setSlave4MasterDelay(uint8_t delay);
        uint8_t getSlate4InputByte();

        // I2C_MST_STATUS register
        bool getPassthroughStatus();
        bool getSlave4IsDone();
        bool getLostArbitration();
        bool getSlave4Nack();
        bool getSlave3Nack();
        bool getSlave2Nack();
        bool getSlave1Nack();
        bool getSlave0Nack();

        // INT_PIN_CFG register
        bool getInterruptMode();
        void setInterruptMode(bool mode);
        bool getInterruptDrive();
        void setInterruptDrive(bool drive);
        bool getInterruptLatch();
        void setInterruptLatch(bool latch);
        bool getInterruptLatchClear();
        void setInterruptLatchClear(bool clear);
        bool getFSyncInterruptLevel();
        void setFSyncInterruptLevel(bool level);
        bool getFSyncInterruptEnabled();
        void setFSyncInterruptEnabled(bool enabled);
        bool getI2CBypassEnabled();
        void setI2CBypassEnabled(bool enabled);
        bool getClockOutputEnabled();
        void setClockOutputEnabled(bool enabled);

        // INT_ENABLE register
        uint8_t getIntEnabled();
        void setIntEnabled(uint8_t enabled);
        bool getIntFreefallEnabled();
        void setIntFreefallEnabled(bool enabled);
        bool getIntMotionEnabled();
        void setIntMotionEnabled(bool enabled);
        bool getIntZeroMotionEnabled();
        void setIntZeroMotionEnabled(bool enabled);
        bool getIntFIFOBufferOverflowEnabled();
        void setIntFIFOBufferOverflowEnabled(bool enabled);
        bool getIntI2CMasterEnabled();
        void setIntI2CMasterEnabled(bool enabled);
        bool getIntDataReadyEnabled();
        void setIntDataReadyEnabled(bool enabled);

        // INT_STATUS register
        uint8_t getIntStatus();
        bool getIntFreefallStatus();
        bool getIntMotionStatus();
        bool getIntZeroMotionStatus();
        bool getIntFIFOBufferOverflowStatus();
        bool getIntI2CMasterStatus();
        bool getIntDataReadyStatus();

        // ACCEL_*OUT_* registers
        void getMotion9(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* mx, int16_t* my, int16_t* mz);
        void getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
        void getAcceleration(int16_t* x, int16_t* y, int16_t* z);
        int16_t getAccelerationX();
        int16_t getAccelerationY();
        int16_t getAccelerationZ();

        // TEMP_OUT_* registers
        int16_t getTemperature();

        // GYRO_*OUT_* registers
        void getRotation(int16_t* x, int16_t* y, int16_t* z);
        int16_t getRotationX();
        int16_t getRotationY();
        int16_t getRotationZ();

        // EXT_SENS_DATA_* registers
        uint8_t getExternalSensorByte(int position);
        uint16_t getExternalSensorWord(int position);
        uint32_t getExternalSensorDWord(int position);

        // MOT_DETECT_STATUS register
        bool getXNegMotionDetected();
        bool getXPosMotionDetected();
        bool getYNegMotionDetected();
        bool getYPosMotionDetected();
        bool getZNegMotionDetected();
        bool getZPosMotionDetected();
        bool getZeroMotionDetected();

        // I2C_SLV*_DO register
        void setSlaveOutputByte(uint8_t num, uint8_t data);

        // I2C_MST_DELAY_CTRL register
        bool getExternalShadowDelayEnabled();
        void setExternalShadowDelayEnabled(bool enabled);
        bool getSlaveDelayEnabled(uint8_t num);
        void setSlaveDelayEnabled(uint8_t num, bool enabled);

        // SIGNAL_PATH_RESET register
        void resetGyroscopePath();
        void resetAccelerometerPath();
        void resetTemperaturePath();

        // MOT_DETECT_CTRL register
        uint8_t getAccelerometerPowerOnDelay();
        void setAccelerometerPowerOnDelay(uint8_t delay);
        uint8_t getFreefallDetectionCounterDecrement();
        void setFreefallDetectionCounterDecrement(uint8_t decrement);
        uint8_t getMotionDetectionCounterDecrement();
        void setMotionDetectionCounterDecrement(uint8_t decrement);

        // USER_CTRL register
        bool getFIFOEnabled();
        void setFIFOEnabled(bool enabled);
        bool getI2CMasterModeEnabled();
        void setI2CMasterModeEnabled(bool enabled);
        void switchSPIEnabled(bool enabled);
        void resetFIFO();
        void resetI2CMaster();
        void resetSensors();

        // PWR_MGMT_1 register
        void reset();
        bool getSleepEnabled();
        void setSleepEnabled(bool enabled);
        bool getWakeCycleEnabled();
        void setWakeCycleEnabled(bool enabled);
        bool getTempSensorEnabled();
        void setTempSensorEnabled(bool enabled);
        uint8_t getClockSource();
        void setClockSource(uint8_t source);

        // PWR_MGMT_2 register
        uint8_t getWakeFrequency();
        void setWakeFrequency(uint8_t frequency);
        bool getStandbyXAccelEnabled();
        void setStandbyXAccelEnabled(bool enabled);
        bool getStandbyYAccelEnabled();
        void setStandbyYAccelEnabled(bool enabled);
        bool getStandbyZAccelEnabled();
        void setStandbyZAccelEnabled(bool enabled);
        bool getStandbyXGyroEnabled();
        void setStandbyXGyroEnabled(bool enabled);
        bool getStandbyYGyroEnabled();
        void setStandbyYGyroEnabled(bool enabled);
        bool getStandbyZGyroEnabled();
        void setStandbyZGyroEnabled(bool enabled);

        // FIFO_COUNT_* registers
        uint16_t getFIFOCount();

        // FIFO_R_W register
        uint8_t getFIFOByte();
        void setFIFOByte(uint8_t data);
        void getFIFOBytes(uint8_t *data, uint8_t length);

        // WHO_AM_I register
        uint8_t getDeviceID();
        void setDeviceID(uint8_t id);

        // ======== UNDOCUMENTED/DMP REGISTERS/METHODS ========

        // XG_OFFS_TC register
        uint8_t getOTPBankValid();
        void setOTPBankValid(bool enabled);
        int8_t getXGyroOffsetTC();
        void setXGyroOffsetTC(int8_t offset);

        // YG_OFFS_TC register
        int8_t getYGyroOffsetTC();
        void setYGyroOffsetTC(int8_t offset);

        // ZG_OFFS_TC register
        int8_t getZGyroOffsetTC();
        void setZGyroOffsetTC(int8_t offset);

        // X_FINE_GAIN register
        int8_t getXFineGain();
        void setXFineGain(int8_t gain);

        // Y_FINE_GAIN register
        int8_t getYFineGain();
        void setYFineGain(int8_t gain);

        // Z_FINE_GAIN register
        int8_t getZFineGain();
        void setZFineGain(int8_t gain);

        // XA_OFFS_* registers
        int16_t getXAccelOffset();
        void setXAccelOffset(int16_t offset);

        // YA_OFFS_* register
        int16_t getYAccelOffset();
        void setYAccelOffset(int16_t offset);

        // ZA_OFFS_* register
        int16_t getZAccelOffset();
        void setZAccelOffset(int16_t offset);

        // XG_OFFS_USR* registers
        int16_t getXGyroOffset();
        void setXGyroOffset(int16_t offset);

        // YG_OFFS_USR* register
        int16_t getYGyroOffset();
        void setYGyroOffset(int16_t offset);

        // ZG_OFFS_USR* register
        int16_t getZGyroOffset();
        void setZGyroOffset(int16_t offset);

        // INT_ENABLE register (DMP functions)
        bool getIntPLLReadyEnabled();
        void setIntPLLReadyEnabled(bool enabled);
        bool getIntDMPEnabled();
        void setIntDMPEnabled(bool enabled);

        // DMP_INT_STATUS
        bool getDMPInt5Status();
        bool getDMPInt4Status();
        bool getDMPInt3Status();
        bool getDMPInt2Status();
        bool getDMPInt1Status();
        bool getDMPInt0Status();

        // INT_STATUS register (DMP functions)
        bool getIntPLLReadyStatus();
        bool getIntDMPStatus();

        // USER_CTRL register (DMP functions)
        bool getDMPEnabled();
        void setDMPEnabled(bool enabled);
        void resetDMP();

        // BANK_SEL register
        void setMemoryBank(uint8_t bank, bool prefetchEnabled=false, bool userBank=false);

        // MEM_START_ADDR register
        void setMemoryStartAddress(uint8_t address);

        // MEM_R_W register
        uint8_t readMemoryByte();
        void writeMemoryByte(uint8_t data);
        void readMemoryBlock(uint8_t *data, uint16_t dataSize, uint8_t bank=0, uint8_t address=0);
        bool writeMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank=0, uint8_t address=0, bool verify=true, bool useProgMem=false);
        bool writeProgMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank=0, uint8_t address=0, bool verify=true);

        bool writeDMPConfigurationSet(const uint8_t *data, uint16_t dataSize, bool useProgMem=false);
        bool writeProgDMPConfigurationSet(const uint8_t *data, uint16_t dataSize);

        // DMP_CFG_1 register
        uint8_t getDMPConfig1();
        void setDMPConfig1(uint8_t config);

        // DMP_CFG_2 register
        uint8_t getDMPConfig2();
        void setDMPConfig2(uint8_t config);

        // special methods for MotionApps 2.0 implementation
        #ifdef MPU6050_INCLUDE_DMP_MOTIONAPPS20
            uint8_t *dmpPacketBuffer;
            uint16_t dmpPacketSize;

            uint8_t dmpInitialize();
            bool dmpPacketAvailable();

            uint8_t dmpSetFIFORate(uint8_t fifoRate);
            uint8_t dmpGetFIFORate();
            uint8_t dmpGetSampleStepSizeMS();
            uint8_t dmpGetSampleFrequency();
            int32_t dmpDecodeTemperature(int8_t tempReg);

            // Register callbacks after a packet of FIFO data is processed
            //uint8_t dmpRegisterFIFORateProcess(inv_obj_func func, int16_t priority);
            //uint8_t dmpUnregisterFIFORateProcess(inv_obj_func func);
            uint8_t dmpRunFIFORateProcesses();

            // Setup FIFO for various output
            uint8_t dmpSendQuaternion(uint_fast16_t accuracy);
            uint8_t dmpSendGyro(uint_fast16_t elements, uint_fast16_t accuracy);
            uint8_t dmpSendAccel(uint_fast16_t elements, uint_fast16_t accuracy);
            uint8_t dmpSendLinearAccel(uint_fast16_t elements, uint_fast16_t accuracy);
            uint8_t dmpSendLinearAccelInWorld(uint_fast16_t elements, uint_fast16_t accuracy);
            uint8_t dmpSendControlData(uint_fast16_t elements, uint_fast16_t accuracy);
            uint8_t dmpSendSensorData(uint_fast16_t elements, uint_fast16_t accuracy);
            uint8_t dmpSendExternalSensorData(uint_fast16_t elements, uint_fast16_t accuracy);
            uint8_t dmpSendGravity(uint_fast16_t elements, uint_fast16_t accuracy);
            uint8_t dmpSendPacketNumber(uint_fast16_t accuracy);
            uint8_t dmpSendQuantizedAccel(uint_fast16_t elements, uint_fast16_t accuracy);
            uint8_t dmpSendEIS(uint_fast16_t elements, uint_fast16_t accuracy);

            // Get Fixed Point data from FIFO
            uint8_t dmpGetAccel(int32_t *data, const uint8_t* packet=0);
            uint8_t dmpGetAccel(int16_t *data, const uint8_t* packet=0);
            uint8_t dmpGetAccel(VectorInt16 *v, const uint8_t* packet=0);
            uint8_t dmpGetQuaternion(int32_t *data, const uint8_t* packet=0);
            uint8_t dmpGetQuaternion(int16_t *data, const uint8_t* packet=0);
            uint8_t dmpGetQuaternion(Quaternion *q, const uint8_t* packet=0);
            uint8_t dmpGet6AxisQuaternion(int32_t *data, const uint8_t* packet=0);
            uint8_t dmpGet6AxisQuaternion(int16_t *data, const uint8_t* packet=0);
            uint8_t dmpGet6AxisQuaternion(Quaternion *q, const uint8_t* packet=0);
            uint8_t dmpGetRelativeQuaternion(int32_t *data, const uint8_t* packet=0);
            uint8_t dmpGetRelativeQuaternion(int16_t *data, const uint8_t* packet=0);
            uint8_t dmpGetRelativeQuaternion(Quaternion *data, const uint8_t* packet=0);
            uint8_t dmpGetGyro(int32_t *data, const uint8_t* packet=0);
            uint8_t dmpGetGyro(int16_t *data, const uint8_t* packet=0);
            uint8_t dmpGetGyro(VectorInt16 *v, const uint8_t* packet=0);
            uint8_t dmpSetLinearAccelFilterCoefficient(float coef);
            uint8_t dmpGetLinearAccel(int32_t *data, const uint8_t* packet=0);
            uint8_t dmpGetLinearAccel(int16_t *data, const uint8_t* packet=0);
            uint8_t dmpGetLinearAccel(VectorInt16 *v, const uint8_t* packet=0);
            uint8_t dmpGetLinearAccel(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity);
            uint8_t dmpGetLinearAccelInWorld(int32_t *data, const uint8_t* packet=0);
            uint8_t dmpGetLinearAccelInWorld(int16_t *data, const uint8_t* packet=0);
            uint8_t dmpGetLinearAccelInWorld(VectorInt16 *v, const uint8_t* packet=0);
            uint8_t dmpGetLinearAccelInWorld(VectorInt16 *v, VectorInt16 *vReal, Quaternion *q);
            uint8_t dmpGetGyroAndAccelSensor(int32_t *data, const uint8_t* packet=0);
            uint8_t dmpGetGyroAndAccelSensor(int16_t *data, const uint8_t* packet=0);
            uint8_t dmpGetGyroAndAccelSensor(VectorInt16 *g, VectorInt16 *a, const uint8_t* packet=0);
            uint8_t dmpGetGyroSensor(int32_t *data, const uint8_t* packet=0);
            uint8_t dmpGetGyroSensor(int16_t *data, const uint8_t* packet=0);
            uint8_t dmpGetGyroSensor(VectorInt16 *v, const uint8_t* packet=0);
            uint8_t dmpGetControlData(int32_t *data, const uint8_t* packet=0);
            uint8_t dmpGetTemperature(int32_t *data, const uint8_t* packet=0);
            uint8_t dmpGetGravity(int32_t *data, const uint8_t* packet=0);
            uint8_t dmpGetGravity(int16_t *data, const uint8_t* packet=0);
            uint8_t dmpGetGravity(VectorInt16 *v, const uint8_t* packet=0);
            uint8_t dmpGetGravity(VectorFloat *v, Quaternion *q);
            uint8_t dmpGetUnquantizedAccel(int32_t *data, const uint8_t* packet=0);
            uint8_t dmpGetUnquantizedAccel(int16_t *data, const uint8_t* packet=0);
            uint8_t dmpGetUnquantizedAccel(VectorInt16 *v, const uint8_t* packet=0);
            uint8_t dmpGetQuantizedAccel(int32_t *data, const uint8_t* packet=0);
            uint8_t dmpGetQuantizedAccel(int16_t *data, const uint8_t* packet=0);
            uint8_t dmpGetQuantizedAccel(VectorInt16 *v, const uint8_t* packet=0);
            uint8_t dmpGetExternalSensorData(int32_t *data, uint16_t size, const uint8_t* packet=0);
            uint8_t dmpGetEIS(int32_t *data, const uint8_t* packet=0);

            uint8_t dmpGetEuler(float *data, Quaternion *q);
            uint8_t dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity);

            // Get Floating Point data from FIFO
            uint8_t dmpGetAccelFloat(float *data, const uint8_t* packet=0);
            uint8_t dmpGetQuaternionFloat(float *data, const uint8_t* packet=0);

            uint8_t dmpProcessFIFOPacket(const unsigned char *dmpData);
            uint8_t dmpReadAndProcessFIFOPacket(uint8_t numPackets, uint8_t *processed=NULL);

            uint8_t dmpSetFIFOProcessedCallback(void (*func) (void));

            uint8_t dmpInitFIFOParam();
            uint8_t dmpCloseFIFO();
            uint8_t dmpSetGyroDataSource(uint8_t source);
            uint8_t dmpDecodeQuantizedAccel();
            uint32_t dmpGetGyroSumOfSquare();
            uint32_t dmpGetAccelSumOfSquare();
            void dmpOverrideQuaternion(long *q);
            uint16_t dmpGetFIFOPacketSize();
        #endif

        // special methods for MotionApps 4.1 implementation
        #ifdef MPU6050_INCLUDE_DMP_MOTIONAPPS41
            uint8_t *dmpPacketBuffer;
            uint16_t dmpPacketSize;

            uint8_t dmpInitialize();
            bool dmpPacketAvailable();

            uint8_t dmpSetFIFORate(uint8_t fifoRate);
            uint8_t dmpGetFIFORate();
            uint8_t dmpGetSampleStepSizeMS();
            uint8_t dmpGetSampleFrequency();
            int32_t dmpDecodeTemperature(int8_t tempReg);

            // Register callbacks after a packet of FIFO data is processed
            //uint8_t dmpRegisterFIFORateProcess(inv_obj_func func, int16_t priority);
            //uint8_t dmpUnregisterFIFORateProcess(inv_obj_func func);
            uint8_t dmpRunFIFORateProcesses();

            // Setup FIFO for various output
            uint8_t dmpSendQuaternion(uint_fast16_t accuracy);
            uint8_t dmpSendGyro(uint_fast16_t elements, uint_fast16_t accuracy);
            uint8_t dmpSendAccel(uint_fast16_t elements, uint_fast16_t accuracy);
            uint8_t dmpSendLinearAccel(uint_fast16_t elements, uint_fast16_t accuracy);
            uint8_t dmpSendLinearAccelInWorld(uint_fast16_t elements, uint_fast16_t accuracy);
            uint8_t dmpSendControlData(uint_fast16_t elements, uint_fast16_t accuracy);
            uint8_t dmpSendSensorData(uint_fast16_t elements, uint_fast16_t accuracy);
            uint8_t dmpSendExternalSensorData(uint_fast16_t elements, uint_fast16_t accuracy);
            uint8_t dmpSendGravity(uint_fast16_t elements, uint_fast16_t accuracy);
            uint8_t dmpSendPacketNumber(uint_fast16_t accuracy);
            uint8_t dmpSendQuantizedAccel(uint_fast16_t elements, uint_fast16_t accuracy);
            uint8_t dmpSendEIS(uint_fast16_t elements, uint_fast16_t accuracy);

            // Get Fixed Point data from FIFO
            uint8_t dmpGetAccel(int32_t *data, const uint8_t* packet=0);
            uint8_t dmpGetAccel(int16_t *data, const uint8_t* packet=0);
            uint8_t dmpGetAccel(VectorInt16 *v, const uint8_t* packet=0);
            uint8_t dmpGetQuaternion(int32_t *data, const uint8_t* packet=0);
            uint8_t dmpGetQuaternion(int16_t *data, const uint8_t* packet=0);
            uint8_t dmpGetQuaternion(Quaternion *q, const uint8_t* packet=0);
            uint8_t dmpGet6AxisQuaternion(int32_t *data, const uint8_t* packet=0);
            uint8_t dmpGet6AxisQuaternion(int16_t *data, const uint8_t* packet=0);
            uint8_t dmpGet6AxisQuaternion(Quaternion *q, const uint8_t* packet=0);
            uint8_t dmpGetRelativeQuaternion(int32_t *data, const uint8_t* packet=0);
            uint8_t dmpGetRelativeQuaternion(int16_t *data, const uint8_t* packet=0);
            uint8_t dmpGetRelativeQuaternion(Quaternion *data, const uint8_t* packet=0);
            uint8_t dmpGetGyro(int32_t *data, const uint8_t* packet=0);
            uint8_t dmpGetGyro(int16_t *data, const uint8_t* packet=0);
            uint8_t dmpGetGyro(VectorInt16 *v, const uint8_t* packet=0);
            uint8_t dmpGetMag(int16_t *data, const uint8_t* packet=0);
            uint8_t dmpSetLinearAccelFilterCoefficient(float coef);
            uint8_t dmpGetLinearAccel(int32_t *data, const uint8_t* packet=0);
            uint8_t dmpGetLinearAccel(int16_t *data, const uint8_t* packet=0);
            uint8_t dmpGetLinearAccel(VectorInt16 *v, const uint8_t* packet=0);
            uint8_t dmpGetLinearAccel(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity);
            uint8_t dmpGetLinearAccelInWorld(int32_t *data, const uint8_t* packet=0);
            uint8_t dmpGetLinearAccelInWorld(int16_t *data, const uint8_t* packet=0);
            uint8_t dmpGetLinearAccelInWorld(VectorInt16 *v, const uint8_t* packet=0);
            uint8_t dmpGetLinearAccelInWorld(VectorInt16 *v, VectorInt16 *vReal, Quaternion *q);
            uint8_t dmpGetGyroAndAccelSensor(int32_t *data, const uint8_t* packet=0);
            uint8_t dmpGetGyroAndAccelSensor(int16_t *data, const uint8_t* packet=0);
            uint8_t dmpGetGyroAndAccelSensor(VectorInt16 *g, VectorInt16 *a, const uint8_t* packet=0);
            uint8_t dmpGetGyroSensor(int32_t *data, const uint8_t* packet=0);
            uint8_t dmpGetGyroSensor(int16_t *data, const uint8_t* packet=0);
            uint8_t dmpGetGyroSensor(VectorInt16 *v, const uint8_t* packet=0);
            uint8_t dmpGetControlData(int32_t *data, const uint8_t* packet=0);
            uint8_t dmpGetTemperature(int32_t *data, const uint8_t* packet=0);
            uint8_t dmpGetGravity(int32_t *data, const uint8_t* packet=0);
            uint8_t dmpGetGravity(int16_t *data, const uint8_t* packet=0);
            uint8_t dmpGetGravity(VectorInt16 *v, const uint8_t* packet=0);
            uint8_t dmpGetGravity(VectorFloat *v, Quaternion *q);
            uint8_t dmpGetUnquantizedAccel(int32_t *data, const uint8_t* packet=0);
            uint8_t dmpGetUnquantizedAccel(int16_t *data, const uint8_t* packet=0);
            uint8_t dmpGetUnquantizedAccel(VectorInt16 *v, const uint8_t* packet=0);
            uint8_t dmpGetQuantizedAccel(int32_t *data, const uint8_t* packet=0);
            uint8_t dmpGetQuantizedAccel(int16_t *data, const uint8_t* packet=0);
            uint8_t dmpGetQuantizedAccel(VectorInt16 *v, const uint8_t* packet=0);
            uint8_t dmpGetExternalSensorData(int32_t *data, uint16_t size, const uint8_t* packet=0);
            uint8_t dmpGetEIS(int32_t *data, const uint8_t* packet=0);

            uint8_t dmpGetEuler(float *data, Quaternion *q);
            uint8_t dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity);

            // Get Floating Point data from FIFO
            uint8_t dmpGetAccelFloat(float *data, const uint8_t* packet=0);
            uint8_t dmpGetQuaternionFloat(float *data, const uint8_t* packet=0);

            uint8_t dmpProcessFIFOPacket(const unsigned char *dmpData);
            uint8_t dmpReadAndProcessFIFOPacket(uint8_t numPackets, uint8_t *processed=NULL);

            uint8_t dmpSetFIFOProcessedCallback(void (*func) (void));

            uint8_t dmpInitFIFOParam();
            uint8_t dmpCloseFIFO();
            uint8_t dmpSetGyroDataSource(uint8_t source);
            uint8_t dmpDecodeQuantizedAccel();
            uint32_t dmpGetGyroSumOfSquare();
            uint32_t dmpGetAccelSumOfSquare();
            void dmpOverrideQuaternion(long *q);
            uint16_t dmpGetFIFOPacketSize();
        #endif

    private:
        uint8_t devAddr;
        uint8_t buffer[14];
};


//Below this line is what already existed:
uint16_t TOTAL_COUNT;
uint8_t Acc_X, Acc_Y, Acc_Z;
float acc_X;
int Gyro_X, Gyro_Y, Gyro_Z;
int Rem_X_Accel,Rem_Y_Accel,Rem_Z_Accel;
int Rem_X_Gyro,Rem_Y_Gyro,Rem_Z_Gyro;
char Sign_X_Accel, Sign_Y_Accel, Sign_Z_Accel;
char Sign_X_Gyro, Sign_Y_Gyro, Sign_Z_Gyro;

#define INT_COEF 16384.0f
#define REM_COEF 16384

/**********************************Communication********************************/
#define I2C	3
#define UART_Handle	2
/*********************************** Types ************************************/
typedef struct{
	uint8_t					_ID;					/*!< Sensor identification (0-252)					*/
	uint32_t				_BaudRate;				/*!< UART communication baud rate*/
	uint8_t					_Sample_Rate;
	UART_HandleTypeDef*		_UART_Handle;
	I2C_HandleTypeDef* 		_I2C_Handle;
	int16_t				_X_GYRO;  			/*!< x-axis angular velocity read from sensor*/
	int16_t				_Y_GYRO;  			/*!< y-axis angular velocity read from sensor*/
	int16_t				_Z_GYRO;  			/*!< z-axis angular velocity read from sensor*/
	int16_t				_X_ACCEL;  			/*!< x-axis acceleration read from sensor*/
	int16_t				_Y_ACCEL;  			/*!< y-axis acceleration read from sensor*/
	int16_t				_Z_ACCEL;  			/*!< z-axis acceleration read from sensor*/
}MPU6050_HandleTypeDef;
/****************Function Definition********************************/
void MPU6050_READ_DATA(MPU6050_HandleTypeDef *sMPU6050, uint8_t Reg_addr, uint8_t* sensor_buffer);
void MPU6050_WRITE_REG(MPU6050_HandleTypeDef *sMPU6050,uint8_t reg_addr, uint8_t data);
void MPU6050_READ_REG(MPU6050_HandleTypeDef *sMPU6050, uint8_t reg_addr);
void MPU6050_init(MPU6050_HandleTypeDef *sMPU6050);
void MPU6050_RESET_SENSOR_REG();
void MPU6050_Clear_Int();
void MPU6050_Data_Ready_Int();
void MPU6050_Get_Val_Gyro();
void MPU6050_Get_Val_Accel();
void MPU6050_print_Angular_Velocity(MPU6050_HandleTypeDef *sMPU6050);
void MPU6050_print_Acceleration(MPU6050_HandleTypeDef *sMPU6050);
void MPU6050_Read_Gyroscope(MPU6050_HandleTypeDef *sMPU6050);
void MPU6050_Read_Accelerometer(MPU6050_HandleTypeDef *sMPU6050);

#define MPU6050_RA_WHO_AM_I         0x75
#define MPU6050_ADDR    	    0b11010000	// ID
#define MPU6050_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW
#define MPU6050_ADDR    	    0b11010000
#define MPU6050_RA_XG_OFFS_TC       0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_YG_OFFS_TC       0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_ZG_OFFS_TC       0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_X_FINE_GAIN      0x03 //[7:0] X_FINE_GAIN
#define MPU6050_RA_Y_FINE_GAIN      0x04 //[7:0] Y_FINE_GAIN
#define MPU6050_RA_Z_FINE_GAIN      0x05 //[7:0] Z_FINE_GAIN
#define MPU6050_RA_XA_OFFS_H        0x06 //[15:0] XA_OFFS
#define MPU6050_RA_XA_OFFS_L_TC     0x07
#define MPU6050_RA_YA_OFFS_H        0x08 //[15:0] YA_OFFS
#define MPU6050_RA_YA_OFFS_L_TC     0x09
#define MPU6050_RA_ZA_OFFS_H        0x0A //[15:0] ZA_OFFS
#define MPU6050_RA_ZA_OFFS_L_TC     0x0B
#define MPU6050_RA_XG_OFFS_USRH     0x13 //[15:0] XG_OFFS_USR
#define MPU6050_RA_XG_OFFS_USRL     0x14
#define MPU6050_RA_YG_OFFS_USRH     0x15 //[15:0] YG_OFFS_USR
#define MPU6050_RA_YG_OFFS_USRL     0x16
#define MPU6050_RA_ZG_OFFS_USRH     0x17 //[15:0] ZG_OFFS_USR
#define MPU6050_RA_ZG_OFFS_USRL     0x18
#define MPU6050_RA_SMPLRT_DIV       0x19
#define MPU6050_RA_CONFIG           0x1A
#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_RA_ACCEL_CONFIG     0x1C
#define MPU6050_RA_FIFO_EN          0x23
#define MPU6050_RA_I2C_MST_CTRL     0x24
#define MPU6050_RA_I2C_MST_STATUS   0x36
#define MPU6050_RA_INT_PIN_CFG      0x37
#define MPU6050_RA_INT_ENABLE       0x38
#define MPU6050_RA_DMP_INT_STATUS   0x39
#define MPU6050_RA_INT_STATUS       0x3A

/*************Output****************************/
#define MPU6050_RA_ACCEL_XOUT_H     0x3B
#define MPU6050_RA_ACCEL_XOUT_L     0x3C
#define MPU6050_RA_ACCEL_YOUT_H     0x3D
#define MPU6050_RA_ACCEL_YOUT_L     0x3E
#define MPU6050_RA_ACCEL_ZOUT_H     0x3F
#define MPU6050_RA_ACCEL_ZOUT_L     0x40
#define MPU6050_RA_TEMP_OUT_H       0x41
#define MPU6050_RA_TEMP_OUT_L       0x42
#define MPU6050_RA_GYRO_XOUT_H      0x43
#define MPU6050_RA_GYRO_XOUT_L      0x44
#define MPU6050_RA_GYRO_YOUT_H      0x45
#define MPU6050_RA_GYRO_YOUT_L      0x46
#define MPU6050_RA_GYRO_ZOUT_H      0x47
#define MPU6050_RA_GYRO_ZOUT_L      0x48

#define MPU6050_RA_USER_CTRL		0x6A

#define MPU6050_RA_PWR_MGMT_1		0x6B
#define MPU6050_RA_PWR_MGMT_2		0x6C

#define MPU6050_RA_FIFO_COUNTH      0x72
#define MPU6050_RA_FIFO_COUNTL      0x73
#define MPU6050_RA_FIFO_R_W         0x74
#define MPU6050_RA_WHO_AM_I         0x75

#define MPU6050_GCONFIG_FS_SEL_BIT      4
#define MPU6050_GCONFIG_FS_SEL_LENGTH   2

/***********Full scale selection*********************/
#define MPU6050_GYRO_FS_250         0x00
#define MPU6050_GYRO_FS_500         0x01
#define MPU6050_GYRO_FS_1000        0x02
#define MPU6050_GYRO_FS_2000        0x03

#define MPU6050_ACCEL_FS_2          0x00
#define MPU6050_ACCEL_FS_4          0x01
#define MPU6050_ACCEL_FS_8          0x02
#define MPU6050_ACCEL_FS_16         0x03

/* FIFO enable bit***********************************/
#define MPU6050_TEMP_FIFO_EN_BIT    7
#define MPU6050_XG_FIFO_EN_BIT      6
#define MPU6050_YG_FIFO_EN_BIT      5
#define MPU6050_ZG_FIFO_EN_BIT      4
#define MPU6050_ACCEL_FIFO_EN_BIT   3
#define MPU6050_SLV2_FIFO_EN_BIT    2
#define MPU6050_SLV1_FIFO_EN_BIT    1
#define MPU6050_SLV0_FIFO_EN_BIT    0

/*****************Sample Rate DIV*******************/
#define MPU6050_CLOCK_DIV_348       0x0
#define MPU6050_CLOCK_DIV_333       0x1
#define MPU6050_CLOCK_DIV_320       0x2
#define MPU6050_CLOCK_DIV_308       0x3
#define MPU6050_CLOCK_DIV_296       0x4
#define MPU6050_CLOCK_DIV_286       0x5
#define MPU6050_CLOCK_DIV_276       0x6
#define MPU6050_CLOCK_DIV_267       0x7
#define MPU6050_CLOCK_DIV_258       0x8
#define MPU6050_CLOCK_DIV_500       0x9
#define MPU6050_CLOCK_DIV_471       0xA
#define MPU6050_CLOCK_DIV_444       0xB
#define MPU6050_CLOCK_DIV_421       0xC
#define MPU6050_CLOCK_DIV_400       0xD
#define MPU6050_CLOCK_DIV_381       0xE
#define MPU6050_CLOCK_DIV_364       0xF

/***********interrupt*************************/
#define MPU6050_INTERRUPT_FF_BIT            7
#define MPU6050_INTERRUPT_MOT_BIT           6
#define MPU6050_INTERRUPT_ZMOT_BIT          5
#define MPU6050_INTERRUPT_FIFO_OFLOW_BIT    4
#define MPU6050_INTERRUPT_I2C_MST_INT_BIT   3
#define MPU6050_INTERRUPT_PLL_RDY_INT_BIT   2
#define MPU6050_INTERRUPT_DMP_INT_BIT       1
#define MPU6050_INTERRUPT_DATA_RDY_BIT      0

// TODO: figure out what these actually do
// UMPL source code is not very obivous
#define MPU6050_DMPINT_5_BIT            5
#define MPU6050_DMPINT_4_BIT            4
#define MPU6050_DMPINT_3_BIT            3
#define MPU6050_DMPINT_2_BIT            2
#define MPU6050_DMPINT_1_BIT            1
#define MPU6050_DMPINT_0_BIT            0

#define MPU6050_MOTION_MOT_XNEG_BIT     7
#define MPU6050_MOTION_MOT_XPOS_BIT     6
#define MPU6050_MOTION_MOT_YNEG_BIT     5
#define MPU6050_MOTION_MOT_YPOS_BIT     4
#define MPU6050_MOTION_MOT_ZNEG_BIT     3
#define MPU6050_MOTION_MOT_ZPOS_BIT     2
#define MPU6050_MOTION_MOT_ZRMOT_BIT    0

/****************Path Rest************************/
#define MPU6050_PATHRESET_GYRO_RESET_BIT    2
#define MPU6050_PATHRESET_ACCEL_RESET_BIT   1
#define MPU6050_PATHRESET_TEMP_RESET_BIT    0

#define MPU6050_USERCTRL_DMP_EN_BIT             7
#define MPU6050_USERCTRL_FIFO_EN_BIT            6
#define MPU6050_USERCTRL_I2C_MST_EN_BIT         5
#define MPU6050_USERCTRL_I2C_IF_DIS_BIT         4
#define MPU6050_USERCTRL_DMP_RESET_BIT          3
#define MPU6050_USERCTRL_FIFO_RESET_BIT         2
#define MPU6050_USERCTRL_I2C_MST_RESET_BIT      1
#define MPU6050_USERCTRL_SIG_COND_RESET_BIT     0

/************Power Management************************/
#define MPU6050_PWR1_DEVICE_RESET_BIT   7
#define MPU6050_PWR1_SLEEP_BIT          6
#define MPU6050_PWR1_CYCLE_BIT          5
#define MPU6050_PWR1_TEMP_DIS_BIT       3
#define MPU6050_PWR1_CLKSEL_BIT         2
#define MPU6050_PWR1_CLKSEL_LENGTH      3

#define MPU6050_PWR2_LP_WAKE_CTRL_BIT       7
#define MPU6050_PWR2_LP_WAKE_CTRL_LENGTH    2
#define MPU6050_PWR2_STBY_XA_BIT            5
#define MPU6050_PWR2_STBY_YA_BIT            4
#define MPU6050_PWR2_STBY_ZA_BIT            3
#define MPU6050_PWR2_STBY_XG_BIT            2
#define MPU6050_PWR2_STBY_YG_BIT            1
#define MPU6050_PWR2_STBY_ZG_BIT            0

#define MPU6050_WAKE_FREQ_1P25      0x0
#define MPU6050_WAKE_FREQ_2P5       0x1
#define MPU6050_WAKE_FREQ_5         0x2

#endif /* MPU6050_H_ */
