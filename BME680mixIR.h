#include <SPI.h>   // Standard SPI library
#include <Wire.h>  // Standard I2C "Wire" library
#include <Adafruit_I2CDevice.h>
#include <Arduino.h>
/////////////////////////////////////
//IR thermometer
typedef enum {
	TEMP_RAW,
	TEMP_K,
	TEMP_C,
	TEMP_F
} temperature_units;
//////////////////////////////////
// MLX90614 Default I2C Address //
//////////////////////////////////
//#define MLX90614_DEFAULT_ADDRESS 0x5A
#define MLX90614_I2CADDR 0x5A
// RAM
#define MLX90614_RAWIR1 0x04
#define MLX90614_RAWIR2 0x05
#define MLX90614_TA 0x06
#define MLX90614_TOBJ1 0x07
#define MLX90614_TOBJ2 0x08
// EEPROM
#define MLX90614_TOMAX 0x20
#define MLX90614_TOMIN 0x21
#define MLX90614_PWMCTRL 0x22
#define MLX90614_TARANGE 0x23
#define MLX90614_EMISS 0x24
#define MLX90614_CONFIG 0x25
#define MLX90614_ADDR 0x2E
#define MLX90614_ID1 0x3C
#define MLX90614_ID2 0x3D
#define MLX90614_ID3 0x3E
#define MLX90614_ID4 0x3F
/////////////////////////////////////
#ifndef BME680mixIR_H
#define BME680mixIR_H
  #define CONCAT_BYTES(msb, lsb) \
    (((uint16_t)msb << 8) | (uint16_t)lsb)  ///< combine msb & lsb bytes
  #ifndef _BV
    #define _BV(bit) (1 << (bit))  ///< This macro isn't pre-defined on all platforms
  #endif
  /***************************************************************************************************
  ** Declare publically visible constants used in the class **
  ***************************************************************************************************/
  #ifndef I2C_MODES                           //   If the I2C_Modes haven't been declared yet
    #define I2C_MODES                         ///< Guard code definition for the I2C modes
const uint32_t I2C_STANDARD_MODE{100000};     ///< Default normal I2C 100KHz speed
const uint32_t I2C_FAST_MODE{400000};         ///< Fast mode
const uint32_t I2C_FAST_MODE_PLUS{1000000};   ///< Really fast mode
const uint32_t I2C_HIGH_SPEED_MODE{3400000};  ///< Turbo mode
  #endif
const uint32_t SPI_HERZ{500000};  ///< SPI speed in Hz
/***************************************************************************************************
** Declare enumerated types used in the class                                                     **
***************************************************************************************************/
enum sensorTypes { TemperatureSensor, HumiditySensor, PressureSensor, GasSensor, UnknownSensor };
enum oversamplingTypes {
  SensorOff,
  Oversample1,
  Oversample2,
  Oversample4,
  Oversample8,
  Oversample16,
  UnknownOversample
};

enum iirFilterTypes { IIROff, IIR2, IIR4, IIR8, IIR16, IIR32, IIR64, IIR128, UnknownIIR };
class mixTempSens_Class {
 public:
////////////////////////////////////////////////////////////////////////////////////////////////
  // mlx functions
  bool beginIR(uint8_t addr = MLX90614_I2CADDR, TwoWire *wire = &Wire);

  double readObjectTempC(void);
  double readAmbientTempC(void);
  double readObjectTempF(void);
  double readAmbientTempF(void);
  uint16_t readEmissivityReg(void);
  void writeEmissivityReg(uint16_t ereg);
  double readEmissivity(void);
  void writeEmissivity(double emissivity);
////////////////////////////////////////////////////////////////////////////////////////////////
  mixTempSens_Class();                           // Class constructor (unused)
  ~mixTempSens_Class();                          // Class destructor (unused)
  // BME functions
  bool    begin();                          // Start using I2C Communications
  bool    begin(const uint32_t i2cSpeed);   // I2C with a non-default speed
  bool    begin(const uint8_t chipSelect);  // Start using either I2C or HW-SPI
  bool    begin(const uint32_t i2cSpeed, const uint8_t i2cAddress);  // Set speed and I2C Addr.
  bool    begin(const uint8_t chipSelect, const uint8_t mosi,        // Start using software SPI
                const uint8_t miso, const uint8_t sck);
  uint8_t setOversampling(const uint8_t sensor,  // Set enum sensorType Oversampling
                          const uint8_t sampling = UINT8_MAX) const;  // and return current value
  bool    setGas(uint16_t GasTemp, uint16_t GasMillis) const;  // Gas heating temperature and time
  uint8_t setIIRFilter(const uint8_t iirFilterSetting = UINT8_MAX) const;  // Set IIR Filter
  uint8_t getSensorData(int32_t &temp, int32_t &hum,    // get most recent readings
                        int32_t &press, int32_t &gas,   //
                        const bool waitSwitch = true);  //
  uint8_t getI2CAddress() const;                        // Return the I2C Address of the BME680
  void    reset();                                      // Reset the BME680
  bool    measuring() const;                            ///< true if currently measuring
  void    triggerMeasurement() const;                   ///< trigger a measurement
  ////////////////////////////////////////////////////////////////////////////////////////////////
 private:
 // mlx functions
  Adafruit_I2CDevice *i2c_dev = NULL; ///< Pointer to I2C bus interface
  float readTemp(uint8_t reg);
  uint16_t read16(uint8_t addr);
  void write16(uint8_t addr, uint16_t data);
  byte crc8(byte *addr, byte len);
  uint8_t _addr;
////////////////////////////////////////////////////////////////////////////////////////////////
// BME680 functions
  bool     commonInitialization();                      ///< Common initialization code
  uint8_t  readByte(const uint8_t addr) const;          ///< Read byte from register address
  uint8_t  readSensors(const bool waitSwitch);          ///< read the registers in one burst
  void     waitForReadings() const;                     ///< Wait for readings to finish
  void     getCalibration();                            ///< Load calibration from registers
  uint8_t  _I2CAddress = 0;                             ///< Default is I2C address is unknown
  uint16_t _I2CSpeed   = 0;                             ///< Default is I2C speed is unknown
  uint8_t  _cs, _sck, _mosi, _miso;                     ///< Hardware and software SPI pins
  uint8_t  _H6, _P10, _res_heat_range;                  ///< unsigned configuration vars
  int8_t   _H3, _H4, _H5, _H7, _G1, _G3, _T3, _P3, _P6, _P7, _res_heat,
      _rng_sw_err;                                            ///< signed configuration vars
  uint16_t _H1, _H2, _T1, _P1;                                ///< unsigned 16bit configuration vars
  int16_t  _G2, _T2, _P2, _P4, _P5, _P8, _P9;                 ///< signed 16bit configuration vars
  int32_t  _tfine, _Temperature, _Pressure, _Humidity, _Gas;  ///< signed 32bit configuration vars

  template <typename T>
  uint8_t &getData(const uint8_t addr, T &value) const {
    /*!
      @brief     Template for reading from I2C or SPI using any data type
      @details   As a template it can support compile-time data type definitions
      @param[in] addr Memory address
      @param[in] value Data Type "T" to read
      @return    Size of data read in bytes
    */
    uint8_t *      bytePtr    = (uint8_t *)&value;  // Pointer to structure beginning
    static uint8_t structSize = sizeof(T);          // Number of bytes in structure
    if (_I2CAddress)                                // Using I2C if address is non-zero
    {                                               //
      Wire.beginTransmission(_I2CAddress);          // Address the I2C device
      Wire.write(addr);                             // Send register address to read
      Wire.endTransmission();                       // Close transmission
      Wire.requestFrom(_I2CAddress, sizeof(T));     // Request 1 byte of data
      structSize = Wire.available();                // Use the actual number of bytes
      for (uint8_t i = 0; i < structSize; i++)
        *bytePtr++ = Wire.read();  // loop for each byte to be read
    }                              //
    else                           //
    {                              //
      if (_sck == 0)               // if sck is zero then hardware SPI
      {                            //
        SPI.beginTransaction(
            SPISettings(SPI_HERZ, MSBFIRST, SPI_MODE0));  // Start the SPI transaction
        digitalWrite(_cs, LOW);                           // Tell BME680 to listen up
        SPI.transfer(addr | 0x80);                        // bit 7 is high, so read a byte
        for (uint8_t i = 0; i < structSize; i++)
          *bytePtr++ = SPI.transfer(0);  // loop for each byte to be read
        digitalWrite(_cs, HIGH);         // Tell BME680 to stop listening
        SPI.endTransaction();            // End the transaction
      } else {                           // otherwise we are using soft SPI
        int8_t  i, j;                    // Loop variables
        uint8_t reply;                   // return byte for soft SPI transfer
        digitalWrite(_cs, LOW);          // Tell BME680 to listen up
        for (j = 7; j >= 0; j--)         // First send the address byte
        {
          digitalWrite(_sck, LOW);                          // set the clock signal
          digitalWrite(_mosi, ((addr) | 0x80) & (1 << j));  // set the MOSI pin state
          digitalWrite(_sck, HIGH);                         // reset the clock signal
        }                                                   // of for-next each bit
        for (i = 0; i < structSize; i++)                    // Loop for each byte to read
        {                                                   //
          reply = 0;                                        // reset our return byte
          for (j = 7; j >= 0; j--)                          // Now read the data at that byte
          {                                                 //
            reply <<= 1;                                    // shift buffer one bit left
            digitalWrite(_sck, LOW);                        // set and reset the clock signal
            digitalWrite(_sck, HIGH);                       // pin to get the next MISO bit
            if (digitalRead(_miso)) reply |= 1;             // read the MISO bit, add to reply
          }                                                 // of for-next each bit
          *bytePtr++ = reply;                               // Add byte just read to return data
        }                                                   // of for-next each byte to be read
        digitalWrite(_cs, HIGH);                            // Tell BME680 to stop listening
      }  // of  if-then-else we are using hardware SPI
    }    // of if-then-else we are using I2C
    return (structSize);
  }  // of method getData()
  template <typename T>
  uint8_t &putData(const uint8_t addr, const T &value) const {
    /*!
      @brief     Template for writing to I2C or SPI using any data type
      @details   As a template it can support compile-time data type definitions
      @param[in] addr Memory address
      @param[in] value Data Type "T" to write
      @return    Size of data written in bytes
    */
    const uint8_t *bytePtr    = (const uint8_t *)&value;  // Pointer to structure beginning
    static uint8_t structSize = sizeof(T);                // Number of bytes in structure
    if (_I2CAddress)                                      // Using I2C if address is non-zero
    {                                                     //
      Wire.beginTransmission(_I2CAddress);                // Address the I2C device
      Wire.write(addr);                                   // Send register address to write
      for (uint8_t i = 0; i < sizeof(T); i++)
        Wire.write(*bytePtr++);  // loop for each byte to be written
      Wire.endTransmission();    // Close transmission
    } else {
      if (_sck == 0)  // if sck is zero then hardware SPI
      {
        SPI.beginTransaction(
            SPISettings(SPI_HERZ, MSBFIRST, SPI_MODE0));  // start the SPI transaction
        digitalWrite(_cs, LOW);                           // Tell BME680 to listen up
        SPI.transfer(addr & ~0x80);                       // bit 7 is low, so write a byte
        for (uint8_t i = 0; i < structSize; i++) {
          SPI.transfer(*bytePtr++);
        }                         // loop for each byte to be written
        digitalWrite(_cs, HIGH);  // Tell BME680 to stop listening
        SPI.endTransaction();     // End the transaction
      } else                      // Otherwise soft SPI is used
      {
        int8_t  i, j;                     // Loop variables
        uint8_t reply;                    // return byte for soft SPI transfer
        for (i = 0; i < structSize; i++)  // Loop for each byte to read
        {
          reply = 0;                // reset our return byte
          digitalWrite(_cs, LOW);   // Tell BME680 to listen up
          for (j = 7; j >= 0; j--)  // First send the address byte
          {
            digitalWrite(_sck, LOW);                         // set the clock signal
            digitalWrite(_mosi, (addr & ~0x80) & (1 << j));  // set the MOSI pin state
            digitalWrite(_sck, HIGH);                        // reset the clock signal
          }                                                  // of for-next each bit
          for (j = 7; j >= 0; j--)                           // Now read the data at that byte
          {
            reply <<= 1;                               // shift buffer one bit left
            digitalWrite(_sck, LOW);                   // set the clock signal
            digitalWrite(_mosi, *bytePtr & (1 << j));  // set the MOSI pin state
            digitalWrite(_sck, HIGH);                  // reset the clock signal
          }                                            // of for-next each bit
          digitalWrite(_cs, HIGH);                     // Tell BME680 to stop listening
          bytePtr++;                                   // go to next byte to write
        }                                              // of for-next each byte to be read
      }                                                // of  if-then-else we are using hardware SPI
    }                                                  // of if-then-else we are using I2C
    return (structSize);
  }  // of method putData()
};   // of BME680 class definition
#endif
