#ifndef DFRobot_BMP280_h
#define DFRobot_BMP280_h

#if(ARDUINO >= 100)
    #include "Arduino.h"
#else
    #include "WProgram.h"
#endif

#include "Wire.h"

/* Default Values */
#define BMP280_DEFAULT_ADRESS   0x76
#define BMP280_CHIP_ID          0x58
#define BMP280_DEFAULT_MODE     0x03
#define BMP280_DEFAULT_OSRS_P   0x07
#define BMP280_DEFAULT_OSRS_T   0x02
#define BMP280_DEFAULT_T_SB     0x00
#define BMP280_DEFAULT_FILTER   0x07

#define SEA_LEVEL_HPA           101325

/* Calibration Data Structure */
typedef struct
{
  uint16_t dig_T1;
  int16_t  dig_T2;
  int16_t  dig_T3;

  uint16_t dig_P1;
  int16_t  dig_P2;
  int16_t  dig_P3;
  int16_t  dig_P4;
  int16_t  dig_P5;
  int16_t  dig_P6;
  int16_t  dig_P7;
  int16_t  dig_P8;
  int16_t  dig_P9;

  uint8_t  dig_H1;
  int16_t  dig_H2;
  uint8_t  dig_H3;
  int16_t  dig_H4;
  int16_t  dig_H5;
  int8_t   dig_H6;
} bmp280_calib_data;

class DFRobot_BMP280
{

public:

  DFRobot_BMP280(uint8_t i2caddr = BMP280_DEFAULT_ADRESS);

  bool begin(uint8_t mode = BMP280_DEFAULT_MODE,
                uint8_t osrs_p = BMP280_DEFAULT_OSRS_P,
                uint8_t osrs_t = BMP280_DEFAULT_OSRS_T,
                uint8_t t_sb = BMP280_DEFAULT_T_SB,
                uint8_t filter = BMP280_DEFAULT_FILTER);
  void setPowerMode(uint8_t);
  void setPressureOverSampling(uint8_t);
  void setTemperatureOverSampling(uint8_t);
  void setStandbyTime(uint8_t);
  void setIIRcoeff(uint8_t);

  uint8_t getPowerMode(void);
  uint8_t getPressureOverSampling(void);
  uint8_t getTemperatureOverSampling(void);
  uint8_t getStandbyTime(void);
  uint8_t getIIRcoeff(void);

  float readTemperature(void);
  float readPressure(void);
  float readAltitude(float seaLevel = SEA_LEVEL_HPA);

private:
  boolean _ready;
  uint8_t _addr;
  uint8_t _rwError;
  int32_t t_fine;

  bmp280_calib_data _bmp280_calib;

  void readCoefficients(void);

  uint32_t read24(byte);
  uint16_t read16(byte);
  uint8_t read8(byte);
  int16_t  readS16(byte);
  uint16_t read16_LE(byte); // little endian
  int16_t  readS16_LE(byte); // little endian
  void write8(byte, byte);

};

#endif
