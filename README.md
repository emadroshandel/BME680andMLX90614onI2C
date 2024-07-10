# BME680andMLX90614onI2C
 This repository contains an example and a library for reading data simultaneously from both the BME680 and MLX90614 sensors over an I2C bus.
 The library is called BME680mixIR. The header and the source code of this library are available in this repo. It should be highlighted that the library has been developed based on the "Zanshin_BME680" and "Adafruit_MLX90614" libraries which can be found in the following links:
- Zanshin_BME680: https://github.com/Zanduino/BME680/tree/master
- Adafruit_MLX90614: https://github.com/adafruit/Adafruit-MLX90614-Library

In the example, the setup and loop functions for each sensor are presented in separate files for better clarity. The "IRsetup.ino" file contains the setup and loop functions for the MLX90614, while the "BME680Setup.ino" file includes those for the BME680.
