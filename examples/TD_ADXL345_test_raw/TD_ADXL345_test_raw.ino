/**
* @file TD_ADXL345_test_raw.ino
* @brief
* This simple code show how to use TD_ADXL345 library with ADXL345 digital
* accelerometer device. Function readRawData is used.

* Interface (typical minimum):
* ADXL345         Arduino Uno Board
* -----------------------------------------------------------------------------
* Gnd             Gnd
* VCC (3.3V)      3.3V
* CS -> Connect to VCC (3.3V) if not connected internally on ADXL345 board.
* INT1            Not connected
* INT2            Not connected
* SDO -> Connect to GND if not connected internally on ADXL345 board.
* SDA             A4
* SCL             A5
* -----------------------------------------------------------------------------
*
* Written by Honee52.
 */

#include <TD_ADXL345.h>

/**
 * ----------------------------------------------------------------------------
 * Define ADXL345 and variables.
 * ----------------------------------------------------------------------------
 */
 TD_ADXL345 accel(0x53);
 int16_t x, y, z;

/**
 * ----------------------------------------------------------------------------
 * Setup
 * ----------------------------------------------------------------------------
*/
void setup() {
  /* Initialize serial port */
  Serial.begin(9600);
  while (!Serial) {
    ; // Wait for serial port. Remove wait if not native USB port.
  }

  delay(500);
  Serial.println(" ");
  Serial.println("Testing TD_ADXL345 Library");
  Serial.println(" ");
  delay(500);

  accel.begin();

  byte deviceID = accel.readDeviceID();
  if (deviceID != 0) {
    Serial.print("DeviceID: 0x");
    Serial.println(deviceID, HEX);
    Serial.println(" ");
  } else {
    Serial.println("read device id: failed");
    while(1) {
      delay(100);
    }
  }

  uint8_t regValue;

  /* BW_RATE: LOW_POWER Bit = 0 => normal operation */
  regValue = ODR_200_HZ;
  if (accel.writeRegister(BW_RATE, regValue) == false)
  {
    Serial.println("writeRegister/BW_RATE failed");
    while(1) {
      delay(100);
    }
  }

  /* DATA_FORMAT */
  regValue = RANGE_16G;
  if (accel.writeRegister(DATA_FORMAT, regValue) == false)
  {
    Serial.println("writeRegister/DATA_FORMAT failed");
    while(1) {
        delay(100);
    }
  }

  /* Start measuring */
  regValue = MEASURE;
  if (accel.writeRegister(POWER_CTL, regValue) == false)
  {
    Serial.println("writeRegister/POWER_CTL failed");
    while(1) {
      delay(100);
    }
  }
  delay(500);
}

/**
 * ----------------------------------------------------------------------------
 * Main loop.
 * ----------------------------------------------------------------------------
*/
void loop() {
  /* Read x,y and z */
  if (accel.readRawData(&x, &y, &z) == false)
  {      
    Serial.println("readRegister/DATAX0 failed");
    while(1) {
      delay(100);
    }
  }
  Serial.print("X: "); Serial.print(x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(z); Serial.println("  ");
  delay(5000);
}
