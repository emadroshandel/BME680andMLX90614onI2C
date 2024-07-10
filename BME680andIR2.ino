#include <Wire.h>              // I2C library, required for MLX90614
#include "BME680mixIR.h"       // Include the BME680 and MLX90614 Sensor library
/**************************************************************************************************
** Declare all program constants                                                                 **
**************************************************************************************************/
const uint32_t SERIAL_SPEED{115200};  ///< Set the baud rate for Serial I/O
/**************************************************************************************************
** Declare global variables and instantiate classes                                              **
**************************************************************************************************/
mixTempSens_Class BME680;  ///< Create an instance of the BME680 class
mixTempSens_Class mlx   ;   // Create an IRTherm object to interact with throughout

// Forward function declaration with default value for sea level
float altitude(const int32_t press, const float seaLevel = 1013.25);
float altitude(const int32_t press, const float seaLevel) {
  static float Altitude;
  Altitude =
    44330.0 * (1.0 - pow(((float)press / 100.0) / seaLevel, 0.1903));  // Convert into meters
  return (Altitude);
}  // of method altitude()

void setup() {
  Serial.begin(SERIAL_SPEED);  // Start serial port at Baud rate

#ifdef __AVR_ATmega32U4__  // If this is a 32U4 processor, then wait 3 seconds to init USB port
  delay(3000);
#endif
  BMESetup();
  IRSetp();
}  
void loop() {
  BMEoPR();
  IRopr();
}  