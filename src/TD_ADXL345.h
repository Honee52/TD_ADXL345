/**
 * ----------------------------------------------------------------------------
 * @file TD_ADXL345.h
 * @brief Arduino I2C library for ADXL345 device (Digital Accelerometer).
 * @details Written by Honee52 for Technode Design (info@technode.fi).
 * You may use this library as it is or change it without limitations.
 * Beerware license.
 * @version 1.0.1
 * @note 'Simple is beatiful'
 * @todo .
 * Version history:
 * Version 1.0.0    Initial version. Only very basic functions are implemented.
 * Version 1.0.1    Function readRawData added.
 * ----------------------------------------------------------------------------
*/
#ifndef TD_ADXL345_H
#define TD_ADXL345_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Wire.h"
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define TD_ADXL345_VERSION "1.0.1"

/**
 * ----------------------------------------------------------------------------
 * @brief REGISTER MAP
 * 
 * analog.com Rev. G | 23 of 36
 * Table 19.
 * Address
 * Hex Dec Name            Type    Reset Value     Description
 * 0x00 0  DEVID           R       11100101        Device ID
 * 0x01 to 0x1C 1 to 28 Reserved Reserved; do not access
 * 0x1D 29 THRESH_TAP      R/W     00000000        Tap threshold
 * 0x1E 30 OFSX            R/W     00000000        X-axis offset
 * 0x1F 31 OFSY            R/W     00000000        Y-axis offset
 * 0x20 32 OFSZ            R/W     00000000        Z-axis offset
 * 0x21 33 DUR             R/W     00000000        Tap duration
 * 0x22 34 Latent          R/W     00000000        Tap latency
 * 0x23 35 Window          R/W     00000000        Tap window
 * 0x24 36 THRESH_ACT      R/W     00000000        Activity threshold
 * 0x25 37 THRESH_INACT    R/W     00000000        Inactivity threshold
 * 0x26 38 TIME_INACT      R/W     00000000        Inactivity time
 * 0x27 39 ACT_INACT_CTL   R/W     00000000        Axis enable control for activity and inactivity detection
 * 0x28 40 THRESH_FF       R/W     00000000        Free-fall threshold
 * 0x29 41 TIME_FF         R/W     00000000        Free-fall time
 * 0x2A 42 TAP_AXES        R/W     00000000        Axis control for single tap/double tap
 * 0x2B 43 ACT_TAP_STATUS  R       00000000        Source of single tap/double tap
 * 0x2C 44 BW_RATE         R/W     00001010        Data rate and power mode control
 * 0x2D 45 POWER_CTL       R/W     00000000        Power-saving features control
 * 0x2E 46 INT_ENABLE      R/W     00000000        Interrupt enable control
 * 0x2F 47 INT_MAP         R/W     00000000        Interrupt mapping control
 * 0x30 48 INT_SOURCE      R       00000010        Source of interrupts
 * 0x31 49 DATA_FORMAT     R/W     00000000        Data format control
 * 0x32 50 DATAX0          R       00000000        X-Axis Data 0
 * 0x33 51 DATAX1          R       00000000        X-Axis Data 1
 * 0x34 52 DATAY0          R       00000000        Y-Axis Data 0
 * 0x35 53 DATAY1          R       00000000        Y-Axis Data 1
 * 0x36 54 DATAZ0          R       00000000        Z-Axis Data 0
 * 0x37 55 DATAZ1          R       00000000        Z-Axis Data 1
 * 0x38 56 FIFO_CTL        R/W     00000000        FIFO control
 * 0x39 57 FIFO_STATUS     R       00000000        FIFO status
 * ----------------------------------------------------------------------------
*/

/**
 * --------------------------------------------------------
 * @brief Registers.
 * --------------------------------------------------------
*/
#define DEVID	        0x00
#define THRESH_TAP	    0x1D
#define OFSX	        0x1E
#define OFSY	        0x1F
#define OFSZ	        0x20
#define DUR	            0x21
#define Latent	        0x22
#define Window	        0x23
#define THRESH_ACT	    0x24
#define THRESH_INACT	0x25
#define TIME_INACT	    0x26
#define ACT_INACT_CTL	0x27
#define THRESH_FF	    0x28
#define TIME_FF	        0x29
#define TAP_AXES	    0x2A
#define ACT_TAP_STATUS	0x2B
#define BW_RATE	        0x2C
#define POWER_CTL	    0x2D
#define INT_ENABLE	    0x2E
#define INT_MAP	        0x2F
#define INT_SOURCE	    0x30
#define DATA_FORMAT	    0x31
#define DATAX0	        0x32
#define DATAX1	        0x33
#define DATAY0	        0x34
#define DATAY1	        0x35
#define DATAZ0	        0x36
#define DATAZ1	        0x37
#define FIFO_CTL	    0x38

/**
 * --------------------------------------------------------
 * Register 0x27—ACT_INACT_CTL
 * --------------------------------------------------------
*/
#define INACT_Z_ENABLE  (1<<0)
#define INACT_Y_ENABLE  (1<<1)
#define INACT_X_ENABLE  (1<<2)
#define INACT_AC_DC     (1<<3)
#define ACT_Z_ENABLE    (1<<4)
#define ACT_Y_ENABLE    (1<<5)
#define ACT_X_ENABLE    (1<<6)
#define ACT_AD_DC       (1<<7)

/**
 * --------------------------------------------------------
 * @brief Register 0x2A—TAP_AXES
 * --------------------------------------------------------
*/
#define TAP_Z_ENABLE    (1<<0)
#define TAP_Y_ENABLE    (1<<1)
#define TAP_X_ENABLE    (1<<2)
#define SUPPRESS        (1<<3)

/**
 * --------------------------------------------------------
 * @brief Register 0x2B—ACT_TAP_STATUS
 * --------------------------------------------------------
*/
#define TAP_Z_SOURCE    (1<<0)
#define TAP_Y_SOURCE    (1<<1)
#define TAP_X_SOURCE    (1<<2)
#define ASLEEP          (1<<3)
#define ACT_Z_SOURCE    (1<<4)
#define ACT_Y_SOURCE    (1<<5)
#define ACT_X_SOURCE    (1<<6)

/**
 * ----------------------------------------------------------------------------
 * @brief Output Data Rate Table
 * Table 7
 * (Hz)     Bandwidth (Hz)  Rate Code   IDD (µA)
 * 3200     1600            1111        140
 * 1600     800             1110        90
 * 800      400             1101        140
 * 400      200             1100        140
 * 200      100             1011        140
 * 100      50              1010        140
 * 50       25              1001        90
 * 25       12.5            1000        60
 * 12.5     6.25            0111        50
 * 6.25     3.13            0110        45
 * 3.13     1.56            0101        40
 * 1.56     0.78            0100        34
 * 0.78     0.39            0011        23
 * 0.39     0.20            0010        23
 * 0.20     0.10            0001        23
 * 0.10     0.05            0000        23
 * ----------------------------------------------------------------------------
*/

/**
 * --------------------------------------------------------
 * @brief Register 0x2C—BW_RATE
 * @note ODR = Output Data Rates
 * --------------------------------------------------------
*/
#define ODR_3200_HZ	    0b1111
#define ODR_1600_HZ	    0b1110
#define ODR_800_HZ	    0b1101
#define ODR_400_HZ	    0b1100
#define ODR_200_HZ	    0b1011
#define ODR_100_HZ	    0b1010
#define ODR_50_HZ	    0b1001
#define ODR_25_HZ	    0b1000
#define ODR_12P5_HZ	    0b0111
#define ODR_6P25_HZ	    0b0110
#define ODR_3P13_HZ	    0b0101
#define ODR_1P56_HZ	    0b0100
#define ODR_0P78_HZ	    0b0011
#define ODR_0P39_HZ	    0b0010
#define ODR_0P20_HZ	    0b0001
#define ODR_0P10_HZ	    0b0000
#define LOW_POWER       (1<<4)

/**
 * --------------------------------------------------------
 * @brief Register 0x2D—POWER_CTL
 * --------------------------------------------------------
*/
#define WAKEUP_8Hz      0x00
#define WAKEUP_4Hz      0x01
#define WAKEUP_2Hz      0x10
#define WAKEUP_1Hz      0x11
#define SLEEP           (1<<2)
#define MEASURE         (1<<3)
#define AUTO_SLEEP      (1<<4)
#define LINK            (1<<5)

/**
 * --------------------------------------------------------
 * @brief Rgister bits for registers:
 * Register 0x2E—INT_ENABLE 
 * Register 0x2F—INT_MAP
 * Register 0x30—INT_SOURCE 
 * --------------------------------------------------------
*/
#define OVERRUN         (1<<0)
#define WATERMARK       (1<<1)
#define FREE_FALL       (1<<2)
#define INACTIVITY      (1<<3)
#define ACTIVITY        (1<<4)
#define DOUBLE_TAP      (1<<5)
#define SINGLE_TAP      (1<<6)
#define DATA_READY      (1<<7)

/**
 * --------------------------------------------------------
 * @brief Register 0x31—DATA_FORMAT bits
 * --------------------------------------------------------
 * Range D1 D0
 * 0 0 ±2 g
 * 0 1 ±4 g
 * 1 0 ±8 g
 * 1 1 ±16 g
 * --------------------------------------------------------
*/
#define RANGE_2G        0x00
#define RANGE_4G        0x01
#define RANGE_8G        0x02
#define RANGE_16G       0x03
#define JUSTIFY         (1<<2)
#define FULL_RES        (1<<3)
#define INT_INVERT      (1<<5)
#define SPI             (1<<6)
#define SELF_TEST       (1<<7)

/**
 * --------------------------------------------------------
 * @brief Register 0x38—FIFO_CTL
 * --------------------------------------------------------
*/
#define SAMPLES_MASK    0b11111
#define TRIGGER         (1<<5)
#define MODE_BYPASS     (0x00 << 6)
#define MODE_FIFO       (0x01 << 6)
#define MODE_STREAM     (0x02 << 6)
#define MODE_TRIGGER    (0x03 << 6)

/**
 * --------------------------------------------------------
 * @brief Register 0x39—FIFO_STATUS
 * --------------------------------------------------------
*/
#define ENTRIES_MASK    0b111111
#define FIFO_TRIG       (1<<7)

/**
 * --------------------------------------------------------
 * @brief Error codes & error masks.
 * --------------------------------------------------------
*/
#define NO_ERROR                    0b0000000000000000
#define ERROR_TRANSMISSION_LEN      0b0000000000000001
#define ERROR_END_TRANSMISSION      0b0000000000000010
#define ERROR_REQUEST_LEN           0b0000000000000100
#define ERROR_WRITE_LEN             0b0000000000001000
#define ERROR_WRONG_SENSOR_ID       0b0000000000010000
#define ERROR_FM_TIMEOUT            0b0000000000100000
#define ERROR_NOT_CONNECTED         0b0000000001000000
#define ERROR_CRC_CHECK             0b0000000010000000
#define ERROR_WRONG_COMMAND         0b0000000100000001

/**
 * @class TD_ADXL345.
 * @brief TD_ADXL345 Class definition.
*/
class TD_ADXL345
{
    public:
    /**
     * @brief TD_ADXL345 Class forward declaration.
     * @param[in] I2C address of the SHT31 device
    */
    TD_ADXL345(uint8_t i2c_device_address);

    /**
     * @brief Function begin.
     * @return boolean result
    */
    bool begin();

    /**
     * @brief Function begin.
     * @param *wire
     * @return boolean result
    */    
    bool begin(TwoWire *wire); 

    /**
     * @brief Read Device ID
     * @param void
     * @return DeviceID (uint8_t)
     * @note Read error if return value == 0
    */
    uint8_t readDeviceID();

    /**
     * @brief Read data (x,y,z) from device.
     * @param x [out] float *x
     * @param y [out] float *y
     * @param z [out] float *z
     * @return boolean result
    */
    bool readData(float *x, float *y, float *z);

    /**
     * @brief Read raw data (x,y,z) from device.
     * @param x [out] int16_t *x
     * @param y [out] int16_t *y
     * @param z [out] int16_t *z
     * @return boolean result
    */
    bool readRawData(int16_t *x, int16_t *y, int16_t *z);

    /**
     * @brief Read from device register.
     * @param u8_register (uint8_t)
     * @return Register value (uint8_t)
    */
    uint8_t readRegister(uint8_t u8_register);

    /**
     * @brief Write to device register.
     * @param u8_register (uint8_t)
     * @param u8_value (uint8_t)
     * @return boolean result
    */
    bool writeRegister(uint8_t u8_register, uint8_t u8_value);   

    /**
     * @brief Return last error.
     * @param void
     * @return error code (int _error_code)
     * @note When reading _error_code is cleared.
    */
    int getLastError();    

    /**
     * @brief TD_SHT31 Class private declarations.
    */
    private:
    TwoWire* _i2c;
    uint8_t _sdaPIN;
    uint8_t _slcPIN;
    uint8_t _i2c_device_address;
    float _ratio;
    int _error_code;

    /**
     * @brief Read bytes to buffer.
     * @param *buffer [out] (uint8_t *buffer)
     * @param data length (uint8_t len)
     * @return boolean result
    */
    bool readBytes(uint8_t *buffer, uint8_t len);

    /**
     * @brief Select device register.
     * @param u8_register (uint8_t)
     * @return boolean result
    */
    bool selectRegister(uint8_t u8_register);  

};

#endif
