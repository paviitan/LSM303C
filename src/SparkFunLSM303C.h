// Test derrived class for base class SparkFunIMU
#ifndef __SPARKFUN_LSM303C_H__
#define __SPARKFUN_LSM303C_H__

#include "SparkFunIMU.h"
#include "LSM303CTypes.h"
#include "mbed.h"

#define SENSITIVITY_ACC   0.06103515625   // LSB/mg
#define SENSITIVITY_MAG   0.00048828125   // LSB/Ga

class LSM303C : public SparkFunIMU {
public:
    // These are the only methods are the only methods the user can use w/o mods
    LSM303C(I2C &i2c);
    ~LSM303C()  =  default;
    SparkFunIMU_status_t begin(void);
    // Begin contains hardware specific code (Pro Mini)
    SparkFunIMU_status_t begin(LSM303C_InterfaceMode_t, LSM303C_MAG_DO_t, LSM303C_MAG_FS_t, LSM303C_MAG_BDU_t, LSM303C_MAG_OMXY_t,
                               LSM303C_MAG_OMZ_t, LSM303C_MAG_MD_t, LSM303C_ACC_FS_t, LSM303C_ACC_BDU_t, uint8_t, LSM303C_ACC_ODR_t);
    float readAccelX(void);
    float readAccelY(void);
    float readAccelZ(void);
    float   readMagX(void);
    float   readMagY(void);
    float   readMagZ(void);
    float  readTempC(void);
    float  readTempF(void);

    //Magnetometer Interrupts
    SparkFunIMU_status_t MAG_EnableInterrupt(bool, LSM303C_AXIS_t);
    SparkFunIMU_status_t MAG_SetInterruptThreshold(float);
    SparkFunIMU_status_t MAG_ResetInterruptConfig();
    SparkFunIMU_status_t MAG_ResetInterruptLatch();
    SparkFunIMU_status_t MAG_ReadIntReg();

protected:
    // Variables to store the most recently read raw data from sensor
    LSM303C_AxesRaw_t accelData = {NAN, NAN, NAN};
    LSM303C_AxesRaw_t   magData = {NAN, NAN, NAN};

    // The LSM303C functions over both I2C or SPI. This library supports both.
    // Interface mode used must be set!
    LSM303C_InterfaceMode_t interfaceMode = MODE_I2C;  // Set a default...

    // Hardware abstraction functions
    // @Note: SPI not supported
    uint8_t  SPI_ReadByte(LSM303C_CHIP_t, uint8_t);
    SparkFunIMU_status_t SPI_WriteByte(LSM303C_CHIP_t, uint8_t, uint8_t);
    // I2C
    I2C &_i2c;
    uint8_t  I2C_ByteWrite(LSM303C_I2C_ADDR_t, uint8_t, uint8_t);
    SparkFunIMU_status_t I2C_ByteRead(LSM303C_I2C_ADDR_t, uint8_t, uint8_t &);

    // Methods required to get device up and running
    SparkFunIMU_status_t MAG_SetODR(LSM303C_MAG_DO_t);
    SparkFunIMU_status_t MAG_SetFullScale(LSM303C_MAG_FS_t);
    SparkFunIMU_status_t MAG_BlockDataUpdate(LSM303C_MAG_BDU_t);
    SparkFunIMU_status_t MAG_XY_AxOperativeMode(LSM303C_MAG_OMXY_t);
    SparkFunIMU_status_t MAG_Z_AxOperativeMode(LSM303C_MAG_OMZ_t);
    SparkFunIMU_status_t MAG_SetMode(LSM303C_MAG_MD_t);
    SparkFunIMU_status_t ACC_SetFullScale(LSM303C_ACC_FS_t);
    SparkFunIMU_status_t ACC_BlockDataUpdate(LSM303C_ACC_BDU_t);
    SparkFunIMU_status_t ACC_EnableAxis(uint8_t);
    SparkFunIMU_status_t ACC_SetODR(LSM303C_ACC_ODR_t);

    SparkFunIMU_status_t ACC_Status_Flags(uint8_t &);
    SparkFunIMU_status_t ACC_GetAccRaw(LSM303C_AxesRaw_t &);
    float    readAccel(LSM303C_AXIS_t); // Reads the accelerometer data from IC

    SparkFunIMU_status_t MAG_GetMagRaw(LSM303C_AxesRaw_t &);
    SparkFunIMU_status_t MAG_TemperatureEN(LSM303C_MAG_TEMP_EN_t);
    SparkFunIMU_status_t MAG_XYZ_AxDataAvailable(LSM303C_MAG_XYZDA_t &);
    float    readMag(LSM303C_AXIS_t);   // Reads the magnetometer data from IC

    SparkFunIMU_status_t MAG_ReadReg(LSM303C_MAG_REG_t, uint8_t &);
    uint8_t  MAG_WriteReg(LSM303C_MAG_REG_t, uint8_t);
    SparkFunIMU_status_t ACC_ReadReg(LSM303C_ACC_REG_t, uint8_t &);
    uint8_t  ACC_WriteReg(LSM303C_ACC_REG_t, uint8_t);
};

#endif
