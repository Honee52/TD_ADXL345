/**
 * ----------------------------------------------------------------------------
 * @file TD_ADXL345.cpp
 * @brief Arduino I2C library for ADXL345 device (Digital Accelerometer).
 * @details Written by Honee52 for Technode Design (info@technode.fi).
 * You may use this library as it is or change it without limitations.
 * Beerware license.
 * @version 1.0.0
 * @note 'Simple is beatiful'
 * @todo .
 * Version history:
 * Version 1.0.0    Initial version. Only very basic functions are implemented.
 * ----------------------------------------------------------------------------
*/

#include "TD_ADXL345.h"

/**
 * ----------------------------------------------------------------------------
 * @brief Initialize TD_ADXL345 Class.
 * ----------------------------------------------------------------------------
*/
TD_ADXL345::TD_ADXL345(uint8_t i2c_device_address)
{
    _i2c_device_address = i2c_device_address;    
    _error_code = NO_ERROR;
}

/**
 * ----------------------------------------------------------------------------
 * @brief Functions bool begin().
 * ----------------------------------------------------------------------------
*/
bool TD_ADXL345::begin()
{
     return begin(&Wire);
}

bool TD_ADXL345::begin(TwoWire *wire)
{
    _i2c = wire;
    _i2c->begin();
    _i2c->setClock(100000); // 100kHz
    /* Put here other initialized values */
    return true;
}

/**
 * ----------------------------------------------------------------------------
 * @brief Function readDeviceID()
 * ----------------------------------------------------------------------------
*/
uint8_t TD_ADXL345::readDeviceID() 
{
    uint8_t value = 0;

    if(selectRegister(DEVID) == false)
    {
        return 0;
    }

    if (readBytes(&value, 1) == false)
    {
        return 0;
    }
    return value;
}

/**
 * ----------------------------------------------------------------------------
 * @brief Function readData(float *x, float *y, float *z).
 * ----------------------------------------------------------------------------
*/
bool TD_ADXL345::readData(float *x, float *y, float *z)
{
    int16_t u16X, u16Y, u16Z;
    uint8_t buffer[6] = { 0, 0, 0, 0, 0, 0 };
    
    if (selectRegister(DATAX0) == false)
    {
        return false;
    }

    /* Read data bytes */
    if (readBytes((uint8_t*) &buffer[0], 6) == false)
    {
        return false;
    }

    /* Get data from private variables */
    u16X = (buffer[1] << 8) | buffer[0];
    u16Y = (buffer[3] << 8) | buffer[2];
    u16Z = (buffer[5] << 8) | buffer[4];   

    /* Convert */
    *x = u16X * _ratio;
    *y = u16Y * _ratio;
    *z = u16Z * _ratio;

    return true;
}

/**
 * ----------------------------------------------------------------------------
 * @brief Function readRegister(uint8_t u8_register).
 * ----------------------------------------------------------------------------
*/
uint8_t TD_ADXL345::readRegister(uint8_t u8_register) 
{
    uint8_t value = 0;

    if(selectRegister(u8_register) == false)
    {
        return 0;
    }

    if (readBytes(&value, 1) == false)
    {
        return 0;
    }
    return value;
}

/**
 * ----------------------------------------------------------------------------
 *  @brief Function bool writeRegister(uint8_t u8register, uint8_t u8_value).
 * ----------------------------------------------------------------------------
*/
bool TD_ADXL345::writeRegister(uint8_t u8_register, uint8_t u8_value)
{
    byte buffer[2];
    buffer[0] = u8_register;
    buffer[1] = u8_value;

    //uint8_t format = readRegister(u8_register); ???

    _i2c->beginTransmission(_i2c_device_address);
    if (_i2c->write(buffer, 2) != 0x02)
    {
        _error_code |= ERROR_WRITE_LEN;
        return false;
    }
    if (_i2c->endTransmission() != 0)
    {
        _error_code |= ERROR_END_TRANSMISSION;
        return false;
    }

    if (u8_register == BW_RATE)     { return true; }
    if (u8_register == POWER_CTL)   { return true; }

    /* Set data ratio */
    if (u8_register == DATA_FORMAT)
    {
        switch (u8_value)
        {
            case RANGE_2G:  { _ratio = (float) (4) / 1024.0f;  break; }
            case RANGE_4G:  { _ratio = (float) (8) / 1024.0f;  break; }
            case RANGE_8G:  { _ratio = (float) (16) / 1024.0f; break; }
            case RANGE_16G: { _ratio = (float) (32) / 1024.0f; break; }
        }
        return true;
    }
    return false;
}

/**
 * ----------------------------------------------------------------------------
 * @brief Function int getLastError().
 * ----------------------------------------------------------------------------
*/
int TD_ADXL345::getLastError()
{
  int retval = _error_code;
  _error_code = NO_ERROR;
  return retval;
}

/**
 * >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
 * Private functions.
 * <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
*/

/**
 * ----------------------------------------------------------------------------
 * @brief Function bool readBytes(uint8_t *buffer, uint8_t len).
 * ----------------------------------------------------------------------------
*/
bool TD_ADXL345::readBytes(uint8_t *buffer, uint8_t len)
{
    int retval = _i2c->requestFrom(_i2c_device_address, (uint8_t) len);
    if (retval == len)
    {
        for (uint8_t i = 0; i < len; i++)
        {
            buffer[i] = _i2c->read();
        }
        return true;
    }
    _error_code |= ERROR_REQUEST_LEN;
    return false;
}

/**
 * ----------------------------------------------------------------------------
 *  @brief Function bool selectRegister(uint8_t u8register).
 * ----------------------------------------------------------------------------
*/
bool TD_ADXL345::selectRegister(uint8_t u8_register)
{
    byte buffer[2];
    buffer[0] = u8_register;
    _i2c->beginTransmission(_i2c_device_address);
    if (_i2c->write(buffer, 1) != 0x01)
    {
        _error_code |= ERROR_WRITE_LEN;
        return false;
    }
    if (_i2c->endTransmission() != 0)
    {
        _error_code |= ERROR_END_TRANSMISSION;
        return false;
    }
    return true;
}

