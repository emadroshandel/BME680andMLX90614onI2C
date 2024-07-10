void BMESetup(){
  while (!BME680.begin(I2C_STANDARD_MODE, 0x77)) {  // Start BME680 using I2C, use first device found
    Serial.print(F("-  Unable to find BME680. Trying again in 5 seconds.\n"));
    delay(5000);
  }  // of loop until device is located
  Serial.print(F("- Setting 16x oversampling for all sensors\n"));
  BME680.setOversampling(TemperatureSensor, Oversample16);  // Use enumerated type values
  BME680.setOversampling(HumiditySensor, Oversample16);     // Use enumerated type values
  BME680.setOversampling(PressureSensor, Oversample16);     // Use enumerated type values
  Serial.print(F("- Setting IIR filter to a value of 4 samples\n"));
  BME680.setIIRFilter(IIR4);                                                    // Use enumerated type values
  Serial.print(F("- Setting gas measurement to 320\xC2\xB0\x43 for 150ms\n"));  // "�C" symbols
  BME680.setGas(320, 150);         
                                               // 320�c for 150 milliseconds
  Serial.print(F("\n- Initializing BME680 sensor 2 at address MLX90614\n"));
}

void BMEoPR(){
  //serachAddress();
  static int32_t temp, humidity, pressure, gas;  // BME readings
  static char buf[16];                           // sprintf text buffer
  static float alt;                              // Temporary variable
  static uint16_t loopCounter = 0;               // Display iterations
  if (loopCounter % 25 == 0) {                   // Show header @25 loops
    Serial.print(F("\nLoop Temp\xC2\xB0\x43 Humid% Press hPa   Alt m Air m"));
    Serial.print(F("\xE2\x84\xA6\n==== ====== ====== ========= ======= ======\n"));  // "�C" symbol
  }                                                                                  // if-then time to show headers
  BME680.getSensorData(temp, humidity, pressure, gas);                               // Get readings
  if (loopCounter++ != 0) {                                                          // Ignore first reading, might be incorrect
    sprintf(buf, "%4d %3d.%02d", (loopCounter - 1) % 9999,                           // Clamp to 9999,
            (int8_t)(temp / 100), (uint8_t)(temp % 100));                            // Temp in decidegrees
    Serial.print(buf);
    sprintf(buf, "%3d.%03d", (int8_t)(humidity / 1000),
            (uint16_t)(humidity % 1000));  // Humidity milli-pct
    Serial.print(buf);
    sprintf(buf, "%7d.%02d", (int16_t)(pressure / 100),
            (uint8_t)(pressure % 100));  // Pressure Pascals
    Serial.print(buf);
    alt = altitude(pressure);                                                // temp altitude
    sprintf(buf, "%5d.%02d", (int16_t)(alt), ((uint8_t)(alt * 100) % 100));  // Altitude meters
    Serial.print(buf);
    sprintf(buf, "%4d.%02d\n", (int16_t)(gas / 100), (uint8_t)(gas % 100));  // Resistance milliohms
    Serial.print(buf);
    delay(5000);  //5 sec waiting 
  }                
}