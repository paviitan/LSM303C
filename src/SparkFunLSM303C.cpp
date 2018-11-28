#include "SparkFunLSM303C.h"

// Public methods
LSM303C::LSM303C(I2C &i2c) : _i2c(i2c)
{
}

SparkFunIMU_status_t LSM303C::begin()
{
    return
        begin(// Default to I2C bus
            MODE_I2C,
            // Initialize magnetometer output data rate to 0.625 Hz (turn on device)
            MAG_DO_40_Hz,
            // Initialize magnetic field full scale to +/-16 gauss
            MAG_FS_16_Ga,
            // Enabling block data updating
            MAG_BDU_ENABLE,
            // Initialize magnetometer X/Y axes ouput data rate to high-perf mode
            MAG_OMXY_HIGH_PERFORMANCE,
            // Initialize magnetometer Z axis performance mode
            MAG_OMZ_HIGH_PERFORMANCE,
            // Initialize magnetometer run mode. Also enables I2C (bit 7 = 0)
            MAG_MD_CONTINUOUS,
            // Initialize acceleration full scale to +/-2g
            ACC_FS_2g,
            // Enable block data updating
            ACC_BDU_ENABLE,
            // Enable X, Y, and Z accelerometer axes
            ACC_X_ENABLE | ACC_Y_ENABLE | ACC_Z_ENABLE,
            // Initialize accelerometer output data rate to 100 Hz (turn on device)
            ACC_ODR_100_Hz
        );
}

SparkFunIMU_status_t LSM303C::begin(LSM303C_InterfaceMode_t im, LSM303C_MAG_DO_t modr, LSM303C_MAG_FS_t mfs,
                                    LSM303C_MAG_BDU_t mbu, LSM303C_MAG_OMXY_t mxyodr, LSM303C_MAG_OMZ_t mzodr, LSM303C_MAG_MD_t mm,
                                    LSM303C_ACC_FS_t afs, LSM303C_ACC_BDU_t abu, uint8_t aea, LSM303C_ACC_ODR_t aodr)
{
    uint8_t successes = 0;
    // Select I2C or SPI
    interfaceMode = im;

    if (interfaceMode == MODE_SPI) {
        return IMU_NOT_SUPPORTED;
    }
    ////////// Initialize Magnetometer //////////
    // Initialize magnetometer output data rate
    successes += MAG_SetODR(modr);
    // Initialize magnetic field full scale
    successes += MAG_SetFullScale(mfs);
    // Enabling block data updating
    successes += MAG_BlockDataUpdate(mbu);
    // Initialize magnetometer X/Y axes ouput data rate
    successes += MAG_XY_AxOperativeMode(mxyodr);
    // Initialize magnetometer Z axis performance mode
    successes += MAG_Z_AxOperativeMode(mzodr);
    // Initialize magnetometer run mode.
    successes += MAG_SetMode(mm);

    ////////// Initialize Accelerometer //////////
    // Initialize acceleration full scale
    successes += ACC_SetFullScale(afs);
    // Enable block data updating
    successes += ACC_BlockDataUpdate(abu);
    // Enable X, Y, and Z accelerometer axes
    successes += ACC_EnableAxis(aea);
    // Initialize accelerometer output data rate
    successes += ACC_SetODR(aodr);

    return (successes == IMU_SUCCESS) ? IMU_SUCCESS : IMU_HW_ERROR;
}

float LSM303C::readMagX()
{
    return readMag(xAxis);
}

float LSM303C::readMagY()
{
    return readMag(yAxis);
}

float LSM303C::readMagZ()
{
    return readMag(zAxis);
}

float LSM303C::readAccelX()
{
    return readAccel(xAxis);
}

float LSM303C::readAccelY()
{
    return readAccel(yAxis);
}

float LSM303C::readAccelZ()
{
    return readAccel(zAxis);
}


float LSM303C::readTempC()
{
    uint8_t valueL;
    uint8_t valueH;
    float temperature;

    // Make sure temperature sensor is enabled
    if (MAG_TemperatureEN(MAG_TEMP_EN_ENABLE)) {
        return NAN;
    }

    if (MAG_ReadReg(MAG_TEMP_OUT_L, valueL)) {
        return NAN;
    }

    if (MAG_ReadReg(MAG_TEMP_OUT_H, valueH)) {
        return NAN;
    }

    temperature = (float)(int16_t)((valueH << 8) | valueL);
    temperature /= 8; // 8 digits/˚C
    temperature += 25;// Reads 0 @ 25˚C

    return temperature;
}

float LSM303C::readTempF()
{
    return ((readTempC() * 9.0 / 5.0) + 32.0);
}



////////////////////////////////////////////////////////////////////////////////
////// Protected methods

float LSM303C::readAccel(LSM303C_AXIS_t dir)
{
    uint8_t flag_ACC_STATUS_FLAGS;
    SparkFunIMU_status_t response = ACC_Status_Flags(flag_ACC_STATUS_FLAGS);

    if (response != IMU_SUCCESS) {
        return NAN;
    }

    // Check for new data in the status flags with a mask
    // If there isn't new data use the last data read.
    // There are valid cases for this, like reading faster than refresh rate.
    if (flag_ACC_STATUS_FLAGS & ACC_ZYX_NEW_DATA_AVAILABLE) {
        response = ACC_GetAccRaw(accelData);
    }
    //convert from LSB to mg
    switch (dir) {
        case xAxis:
            return accelData.xAxis * SENSITIVITY_ACC;
            break;
        case yAxis:
            return accelData.yAxis * SENSITIVITY_ACC;
            break;
        case zAxis:
            return accelData.zAxis * SENSITIVITY_ACC;
            break;
        default:
            return NAN;
    }

    // Should never get here
    return NAN;
}

float LSM303C::readMag(LSM303C_AXIS_t dir)
{
    LSM303C_MAG_XYZDA_t flag_MAG_XYZDA;
    SparkFunIMU_status_t response = MAG_XYZ_AxDataAvailable(flag_MAG_XYZDA);

    if (response != IMU_SUCCESS) {
        return NAN;
    }

    // Check for new data in the status flags with a mask
    if (flag_MAG_XYZDA & MAG_XYZDA_YES) {
        response = MAG_GetMagRaw(magData);
    }
    //convert from LSB to Gauss
    switch (dir) {
        case xAxis:
            return magData.xAxis * SENSITIVITY_MAG;
            break;
        case yAxis:
            return magData.yAxis * SENSITIVITY_MAG;
            break;
        case zAxis:
            return magData.zAxis * SENSITIVITY_MAG;
            break;
        default:
            return NAN;
    }

    // Should never get here
    return NAN;
}

SparkFunIMU_status_t LSM303C::MAG_GetMagRaw(LSM303C_AxesRaw_t &buff)
{
    uint8_t valueL;
    uint8_t valueH;

    if (MAG_ReadReg(MAG_OUTX_L, valueL)) {
        return IMU_HW_ERROR;
    }

    if (MAG_ReadReg(MAG_OUTX_H, valueH)) {
        return IMU_HW_ERROR;
    }

    buff.xAxis = (int16_t)((valueH << 8) | valueL);

    if (MAG_ReadReg(MAG_OUTY_L, valueL)) {
        return IMU_HW_ERROR;
    }

    if (MAG_ReadReg(MAG_OUTY_H, valueH)) {
        return IMU_HW_ERROR;
    }

    buff.yAxis = (int16_t)((valueH << 8) | valueL);

    if (MAG_ReadReg(MAG_OUTZ_L, valueL)) {
        return IMU_HW_ERROR;
    }

    if (MAG_ReadReg(MAG_OUTZ_H, valueH)) {
        return IMU_HW_ERROR;
    }

    buff.zAxis = (int16_t)((valueH << 8) | valueL);

    return IMU_SUCCESS;
}

// Methods required to get device up and running
SparkFunIMU_status_t LSM303C::MAG_SetODR(LSM303C_MAG_DO_t val)
{
    uint8_t value;

    if (MAG_ReadReg(MAG_CTRL_REG1, value)) {
        return IMU_HW_ERROR;
    }

    // Mask and only change DO0 bits (4:2) of MAG_CTRL_REG1
    value &= ~MAG_DO_80_Hz;
    value |= val;

    if (MAG_WriteReg(MAG_CTRL_REG1, value)) {
        return IMU_HW_ERROR;
    }

    return IMU_SUCCESS;
}

SparkFunIMU_status_t LSM303C::MAG_SetFullScale(LSM303C_MAG_FS_t val)
{
    uint8_t value;

    if (MAG_ReadReg(MAG_CTRL_REG2, value)) {
        return IMU_HW_ERROR;
    }

    value &= ~MAG_FS_16_Ga; //mask
    value |= val;

    if (MAG_WriteReg(MAG_CTRL_REG2, value)) {
        return IMU_HW_ERROR;
    }

    return IMU_SUCCESS;
}

SparkFunIMU_status_t LSM303C::MAG_BlockDataUpdate(LSM303C_MAG_BDU_t val)
{
    uint8_t value;

    if (MAG_ReadReg(MAG_CTRL_REG5, value)) {
        return IMU_HW_ERROR;
    }


    value &= ~MAG_BDU_ENABLE; //mask
    value |= val;

    if (MAG_WriteReg(MAG_CTRL_REG5, value)) {
        return IMU_HW_ERROR;
    }

    return IMU_SUCCESS;
}

SparkFunIMU_status_t LSM303C::MAG_XYZ_AxDataAvailable(LSM303C_MAG_XYZDA_t &value)
{
    if (MAG_ReadReg(MAG_STATUS_REG, (uint8_t &)value)) {
        return IMU_HW_ERROR;
    }

    value = (LSM303C_MAG_XYZDA_t)((int8_t)value & (int8_t)MAG_XYZDA_YES);

    return IMU_SUCCESS;
}

SparkFunIMU_status_t LSM303C::MAG_XY_AxOperativeMode(LSM303C_MAG_OMXY_t val)
{

    uint8_t value;

    if (MAG_ReadReg(MAG_CTRL_REG1, value)) {
        return IMU_HW_ERROR;
    }

    value &= ~MAG_OMXY_ULTRA_HIGH_PERFORMANCE; //mask
    value |= val;

    if (MAG_WriteReg(MAG_CTRL_REG1, value)) {
        return IMU_HW_ERROR;
    }

    return IMU_SUCCESS;
}

SparkFunIMU_status_t LSM303C::MAG_Z_AxOperativeMode(LSM303C_MAG_OMZ_t val)
{
    uint8_t value;

    if (MAG_ReadReg(MAG_CTRL_REG4, value)) {
        return IMU_HW_ERROR;
    }

    value &= ~MAG_OMZ_ULTRA_HIGH_PERFORMANCE; //mask
    value |= val;

    if (MAG_WriteReg(MAG_CTRL_REG4, value)) {
        return IMU_HW_ERROR;
    }

    return IMU_SUCCESS;
}

SparkFunIMU_status_t LSM303C::MAG_SetMode(LSM303C_MAG_MD_t val)
{
    uint8_t value;

    if (MAG_ReadReg(MAG_CTRL_REG3, value)) {
        return IMU_HW_ERROR;
    }

    value &= ~MAG_MD_POWER_DOWN_2;
    value |= val;

    if (MAG_WriteReg(MAG_CTRL_REG3, value)) {
        return IMU_HW_ERROR;
    }

    return IMU_SUCCESS;
}

SparkFunIMU_status_t LSM303C::ACC_SetFullScale(LSM303C_ACC_FS_t val)
{
    uint8_t value;

    if (ACC_ReadReg(ACC_CTRL4, value)) {
        return IMU_HW_ERROR;
    }

    value &= ~ACC_FS_8g;
    value |= val;


    if (ACC_WriteReg(ACC_CTRL4, value)) {
        return IMU_HW_ERROR;
    }

    return IMU_SUCCESS;
}

SparkFunIMU_status_t LSM303C::ACC_BlockDataUpdate(LSM303C_ACC_BDU_t val)
{
    uint8_t value;

    if (ACC_ReadReg(ACC_CTRL1, value)) {
        return IMU_HW_ERROR;
    }

    value &= ~ACC_BDU_ENABLE;
    value |= val;

    if (ACC_WriteReg(ACC_CTRL1, value)) {
        return IMU_HW_ERROR;
    }

    return IMU_SUCCESS;
}

SparkFunIMU_status_t LSM303C::ACC_EnableAxis(uint8_t val)
{
    uint8_t value;

    if (ACC_ReadReg(ACC_CTRL1, value)) {
        return IMU_HW_ERROR;
    }

    value &= ~0x07;
    value |= val;

    if (ACC_WriteReg(ACC_CTRL1, value)) {
        return IMU_HW_ERROR;
    }

    return IMU_SUCCESS;
}

SparkFunIMU_status_t LSM303C::ACC_SetODR(LSM303C_ACC_ODR_t val)
{
    uint8_t value;

    if (ACC_ReadReg(ACC_CTRL1, value)) {
        return IMU_HW_ERROR;
    }

    value &= ~ACC_ODR_MASK;
    value |= val;

    if (ACC_WriteReg(ACC_CTRL1, value)) {
        return IMU_HW_ERROR;
    }

    return IMU_SUCCESS;
}

SparkFunIMU_status_t LSM303C::MAG_TemperatureEN(LSM303C_MAG_TEMP_EN_t val)
{
    uint8_t value;

    if (MAG_ReadReg(MAG_CTRL_REG1, value)) {
        return IMU_HW_ERROR;
    }

    value &= ~MAG_TEMP_EN_ENABLE; //mask
    value |= val;

    if (MAG_WriteReg(MAG_CTRL_REG1, value)) {
        return IMU_HW_ERROR;
    }

    return IMU_SUCCESS;
}

SparkFunIMU_status_t LSM303C::MAG_ReadReg(LSM303C_MAG_REG_t reg, uint8_t &data)
{
    SparkFunIMU_status_t ret = IMU_GENERIC_ERROR;

    if (interfaceMode == MODE_I2C) {
        ret = I2C_ByteRead(MAG_I2C_ADDR, reg, data);
    } else if (interfaceMode == MODE_SPI) {
        data = SPI_ReadByte(MAG, reg);
        ret = IMU_SUCCESS;
    } else {
        ret = IMU_GENERIC_ERROR; // Shouldn't get here
    }

    return ret;
}

uint8_t  LSM303C::MAG_WriteReg(LSM303C_MAG_REG_t reg, uint8_t data)
{
    uint8_t ret;

    if (interfaceMode == MODE_I2C) {
        ret = I2C_ByteWrite(MAG_I2C_ADDR, reg, data);
    } else if (interfaceMode == MODE_SPI) {
        ret = SPI_WriteByte(MAG, reg, data);
    } else {
        ret = IMU_GENERIC_ERROR;
    }

    return ret;
}

SparkFunIMU_status_t LSM303C::ACC_ReadReg(LSM303C_ACC_REG_t reg, uint8_t &data)
{
    SparkFunIMU_status_t ret;

    if (interfaceMode == MODE_I2C) {
        ret = I2C_ByteRead(ACC_I2C_ADDR, reg, data);
    } else if (interfaceMode == MODE_SPI) {
        data = SPI_ReadByte(ACC, reg);
        ret = IMU_SUCCESS;
    } else {
        ret = IMU_HW_ERROR;
    }

    return ret;
}





uint8_t  LSM303C::ACC_WriteReg(LSM303C_ACC_REG_t reg, uint8_t data)
{
    uint8_t ret;

    if (interfaceMode == MODE_I2C) {
        ret = I2C_ByteWrite(ACC_I2C_ADDR, reg, data);
    } else if (interfaceMode == MODE_SPI) {
        ret = SPI_WriteByte(ACC, reg, data);
    } else {
        ret = IMU_GENERIC_ERROR;
    }

    return ret;
}

// This function uses bit manibulation for higher speed & smaller code
uint8_t LSM303C::SPI_ReadByte(LSM303C_CHIP_t chip, uint8_t data)
{
    uint8_t ret = IMU_NOT_SUPPORTED;
    return ret;
}





// This function uses bit manibulation for higher speed & smaller code
SparkFunIMU_status_t LSM303C::SPI_WriteByte(LSM303C_CHIP_t chip, uint8_t reg, uint8_t data)
{
    SparkFunIMU_status_t ret = IMU_NOT_SUPPORTED;
    return ret;
}




uint8_t  LSM303C::I2C_ByteWrite(LSM303C_I2C_ADDR_t slaveAddress, uint8_t reg,
                                uint8_t data)
{
    uint8_t ret = IMU_GENERIC_ERROR;
    char cmd[2];
    cmd[0] = reg;
    cmd[1] = data;
    if (_i2c.write(slaveAddress, cmd, 2)) {
        ret = IMU_HW_ERROR;
    } else {
        ret = IMU_SUCCESS;
    }
    return ret;
}

SparkFunIMU_status_t LSM303C::I2C_ByteRead(LSM303C_I2C_ADDR_t slaveAddress, uint8_t reg,
                                           uint8_t &data)
{
    SparkFunIMU_status_t ret = IMU_GENERIC_ERROR;
    char dataByte[1];
    char cmd[1];
    cmd[0] = reg;
    if (_i2c.write(slaveAddress, cmd, 1, true)) {
        ret = IMU_HW_ERROR;
    }
    if (_i2c.read(slaveAddress, dataByte, 1)) {
        ret = IMU_HW_ERROR;
    } else {
        data = dataByte[0];
        ret = IMU_SUCCESS;
    }
    return ret;
}

SparkFunIMU_status_t LSM303C::ACC_Status_Flags(uint8_t &val)
{
    if (ACC_ReadReg(ACC_STATUS, val)) {
        return IMU_HW_ERROR;
    }

    return IMU_SUCCESS;
}

SparkFunIMU_status_t LSM303C::ACC_GetAccRaw(LSM303C_AxesRaw_t &buff)
{
    uint8_t valueL;
    uint8_t valueH;

    if (ACC_ReadReg(ACC_OUT_X_H, valueH)) {
        return IMU_HW_ERROR;
    }

    if (ACC_ReadReg(ACC_OUT_X_L, valueL)) {
        return IMU_HW_ERROR;
    }

    buff.xAxis = (int16_t)((valueH << 8) | valueL);

    if (ACC_ReadReg(ACC_OUT_Y_H, valueH)) {
        return IMU_HW_ERROR;
    }

    if (ACC_ReadReg(ACC_OUT_Y_L, valueL)) {
        return IMU_HW_ERROR;
    }

    buff.yAxis = (int16_t)((valueH << 8) | valueL);

    if (ACC_ReadReg(ACC_OUT_Z_H, valueH)) {
        return IMU_HW_ERROR;
    }

    if (ACC_ReadReg(ACC_OUT_Z_L, valueL)) {
        return IMU_HW_ERROR;
    }

    buff.zAxis = (int16_t)((valueH << 8) | valueL);

    return IMU_SUCCESS;
}
