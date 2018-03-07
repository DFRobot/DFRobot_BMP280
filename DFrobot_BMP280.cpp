#include "Arduino.h"
#include "DFRobot_BMP280.h"
#include "Wire.h"

/* Register adresses */
#define BMP280_TEMP_XLSB    0xFC
#define BMP280_TEMP_LSB     0xFB
#define BMP280_TEMP_MSB     0xFA
#define BMP280_PRESS_XLSB   0xF9
#define BMP280_PRESS_LSB    0xF8
#define BMP280_PRESS_MSB    0xF7
#define BMP280_CONFIG       0xF5
#define BMP280_CTRL_MEAS    0xF4
#define BMP280_STATUS       0xF3
#define BMP280_RESET        0xE0
#define BMP280_ID           0xD0

/* Temperature and Pressure Calculation Coefficients */
/* Registers */
#define BMP280_REGISTER_DIG_T1    0x88
#define BMP280_REGISTER_DIG_T2    0x8A
#define BMP280_REGISTER_DIG_T3    0x8C
#define BMP280_REGISTER_DIG_P1    0x8E
#define BMP280_REGISTER_DIG_P2    0x90
#define BMP280_REGISTER_DIG_P3    0x92
#define BMP280_REGISTER_DIG_P4    0x94
#define BMP280_REGISTER_DIG_P5    0x96
#define BMP280_REGISTER_DIG_P6    0x98
#define BMP280_REGISTER_DIG_P7    0x9A
#define BMP280_REGISTER_DIG_P8    0x9C
#define BMP280_REGISTER_DIG_P9    0x9E

// ===================================================================== PUBLIC

/* -- BMP280 class constructor --------------------------------------------- */
DFRobot_BMP280::DFRobot_BMP280(uint8_t i2caddr) {

  _addr = i2caddr;
  _ready = false;
}

/* -- Initializes sensor ------------------------------------------------------
 * POWER MODE                 (default = normal mode)
 * PRESSURE OVERSAMPLING      (default = x16)
 * TEMPERATURE OVERSAMPLING   (default = x2)
 * STANDBY TIME               (default = 0.5ms)
 * IIR FILTER COEFFICIENTS    (default = 16)

 * readCoefficients will read the calibration coefficients to use in temperature
 and pressure calculations.
 --------------------------------------------------------------------------- */
bool DFRobot_BMP280::begin(uint8_t mode, uint8_t osrs_p, uint8_t osrs_t,
                      uint8_t t_sb, uint8_t filter) {

  _rwError = 0;

  Wire.begin();

  readCoefficients();

  setPowerMode(mode);
  setPressureOverSampling(osrs_p);
  setTemperatureOverSampling(osrs_t);
  setStandbyTime(t_sb);
  setIIRcoeff(filter);

  if (_rwError) return _ready = false;

  return _ready = true;
}

/* -- Defines power mode -----------------------------------------------------
 * 00           sleep mode
 * 01 and 10    force mode
 * 11           normal mode

 * sends one of the combinations to BMP280_CTRL_MEAS register bits 1, 0.
 --------------------------------------------------------------------------- */
void DFRobot_BMP280::setPowerMode(uint8_t mode) {

  mode &= 0x03;

  uint8_t reg_data = read8(BMP280_CTRL_MEAS);
  reg_data &= 0xFC;
  reg_data |= mode;

  if (mode != getPowerMode() && !_rwError){
    write8(BMP280_CTRL_MEAS, reg_data);
  }
}
/* -- Gets Power Mode ---------------------------------------------------------
---------------------------------------------------------------------------- */
uint8_t DFRobot_BMP280::getPowerMode(void) {
  uint8_t mode = read8(BMP280_CTRL_MEAS) & 3;
  return mode;
}

/* -- Defines pressure oversampling configuration -----------------------------
 * 000            Skipped
 * 001            Oversampling *1
 * 010            Oversampling *2
 * 011            Oversampling *4
 * 100            Oversampling *8
 * 101, others    Oversampling *16

 * sends one of the combinations to BMP280_CTRL_MEAS register bits 4, 3, 2
 --------------------------------------------------------------------------- */
void DFRobot_BMP280::setPressureOverSampling(uint8_t osrs_p) {

  osrs_p &= 7;

  uint8_t reg_data = read8(BMP280_CTRL_MEAS);
  reg_data &= 0xE3;
  reg_data |= (osrs_p << 2);

  if (osrs_p != getPressureOverSampling() && !_rwError){
    write8(BMP280_CTRL_MEAS, reg_data);
  }
}
/* -- Gets pressure oversampling configuration --------------------------------
---------------------------------------------------------------------------- */
uint8_t DFRobot_BMP280::getPressureOverSampling(void){
  uint8_t orsrs_p = (read8(BMP280_CTRL_MEAS) >> 2) & 7;
  return orsrs_p;
}

/* Defines temperature oversampling configuration
 * 000            Skipped
 * 001            Oversampling *1
 * 010            Oversampling *2
 * 011            Oversampling *4
 * 100            Oversampling *8
 * 101, others    Oversampling *16

 * sends one of the combinations to BMP280_CTRL_MEAS register bits 7, 6, 5
 --------------------------------------------------------------------------- */
void DFRobot_BMP280::setTemperatureOverSampling(uint8_t osrs_t) {

  osrs_t &= 7;

  uint8_t reg_data = read8(BMP280_CTRL_MEAS);
  reg_data &= 0x1F;
  reg_data |= (osrs_t << 5);

  if (osrs_t != getTemperatureOverSampling() && !_rwError){
    write8(BMP280_CTRL_MEAS, reg_data);
  }
}
/* -- Gets temperature oversampling configuration --------------------------------
---------------------------------------------------------------------------- */
uint8_t DFRobot_BMP280::getTemperatureOverSampling(void){
  uint8_t orsrs_t = (read8(BMP280_CTRL_MEAS) >> 5) & 7;
  return orsrs_t;
}

/* -- Defines standby time ----------------------------------------------------
 * 000      0.5ms
 * 001      62.5ms
 * 010      125ms
 * 011      250ms
 * 100      500ms
 * 101      1000ms
 * 110      2000ms
 * 111      4000ms

 * sends one of the configurations to BMP280_CONFIG register bits 7, 6, 5
 --------------------------------------------------------------------------- */
void DFRobot_BMP280::setStandbyTime(uint8_t t_sb) {

  t_sb &= 7;

  uint8_t reg_data = read8(BMP280_CONFIG);
  reg_data &= 0x1F;
  reg_data |= (t_sb << 5);

  if (t_sb != getStandbyTime() && !_rwError){
    write8(BMP280_CONFIG, reg_data);
  }
}
/* -- Gets Standby Time -------------------------------------------------------
---------------------------------------------------------------------------- */
uint8_t DFRobot_BMP280::getStandbyTime(void){
  uint8_t t_sb = (read8(BMP280_CONFIG) >> 5) & 7;
  return t_sb;
}

/* -- Defines IIR filter coefficients -----------------------------------------
 * off
 * 2
 * 4
 * 8
 * 16

 * sends to BMP280_CONFIG register bits 4, 3, 2
 --------------------------------------------------------------------------- */
void DFRobot_BMP280::setIIRcoeff(uint8_t filter) {

  filter &= 7;

  uint8_t reg_data = read8(BMP280_CONFIG);
  reg_data &= 0xE3;
  reg_data |= (filter << 2);

  if (filter != getPressureOverSampling() && !_rwError){
    write8(BMP280_CONFIG, reg_data);
  }
}
/* -- Gets IIR filter coefficients --------------------------------------------
---------------------------------------------------------------------------- */
uint8_t DFRobot_BMP280::getIIRcoeff(void){
  uint8_t filter = (read8(BMP280_CONFIG) >> 2) & 7;
  return filter;
}

/* -- Temperature -------------------------------------------------------------
 * algorithm based on Adafruit BMP280 Library
 * use burst reads
 * calibration data used to calculate temperature
 --------------------------------------------------------------------------- */
float DFRobot_BMP280::readTemperature(void) {

  int32_t var1, var2;

  int32_t adc_T = read24(BMP280_TEMP_MSB);
  adc_T >>= 4;

  var1  = ((((adc_T>>3) - ((int32_t)_bmp280_calib.dig_T1 <<1))) *
	   ((int32_t)_bmp280_calib.dig_T2)) >> 11;

  var2  = (((((adc_T>>4) - ((int32_t)_bmp280_calib.dig_T1)) *
	     ((adc_T>>4) - ((int32_t)_bmp280_calib.dig_T1))) >> 12) *
	   ((int32_t)_bmp280_calib.dig_T3)) >> 14;

  t_fine = var1 + var2;

  float T  = (t_fine * 5 + 128) >> 8;
  return T/100;
}

/* -- Pressure ----------------------------------------------------------------
 * algorithm based on Adafruit BMP280 Library
 * use burst reads
 * calibration data and temperature used to calculate pressure
 --------------------------------------------------------------------------- */
float DFRobot_BMP280::readPressure(void) {
  int64_t var1, var2, p;

  // Must be done first to get the t_fine variable set up
  readTemperature();

  int32_t adc_P = read24(BMP280_PRESS_MSB);
  adc_P >>= 4;

  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)_bmp280_calib.dig_P6;
  var2 = var2 + ((var1*(int64_t)_bmp280_calib.dig_P5)<<17);
  var2 = var2 + (((int64_t)_bmp280_calib.dig_P4)<<35);
  var1 = ((var1 * var1 * (int64_t)_bmp280_calib.dig_P3)>>8) +
    ((var1 * (int64_t)_bmp280_calib.dig_P2)<<12);
  var1 = (((((int64_t)1)<<47)+var1))*((int64_t)_bmp280_calib.dig_P1)>>33;

  if (var1 == 0) {
    return 0;  // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p<<31) - var2)*3125) / var1;
  var1 = (((int64_t)_bmp280_calib.dig_P9) * (p>>13) * (p>>13)) >> 25;
  var2 = (((int64_t)_bmp280_calib.dig_P8) * p) >> 19;

  p = ((p + var1 + var2) >> 8) + (((int64_t)_bmp280_calib.dig_P7)<<4);
  return (float)p/256;
}

/* -- Altitude -------------------------------------------------------
 * temperature is calculated based on temperature and sea level pressure
 * p = 101325 * (1-(0.0065*h/288.15))^5.25588; p=local pressure; h=altitude
 * (invert equation!)
 ---------------------------------------------------------------------------- */
float DFRobot_BMP280::readAltitude(float seaLevel) {
  float altitude;

  float pressure = readPressure(); // in Si units for Pascal
  pressure /= 100;
  seaLevel /= 100;

  altitude = 44330.769 * (1.0 - pow(pressure / seaLevel, 0.1903));

  return altitude;
}

// =================================================================== PRIVATE

/* -- Reads the factory set coefficients --------------------------------------
* algorithm based on Adafruit BMP280 Library
---------------------------------------------------------------------------- */
void DFRobot_BMP280::readCoefficients(void)
{
    _bmp280_calib.dig_T1 = read16_LE(BMP280_REGISTER_DIG_T1);
    _bmp280_calib.dig_T2 = readS16_LE(BMP280_REGISTER_DIG_T2);
    _bmp280_calib.dig_T3 = readS16_LE(BMP280_REGISTER_DIG_T3);

    _bmp280_calib.dig_P1 = read16_LE(BMP280_REGISTER_DIG_P1);
    _bmp280_calib.dig_P2 = readS16_LE(BMP280_REGISTER_DIG_P2);
    _bmp280_calib.dig_P3 = readS16_LE(BMP280_REGISTER_DIG_P3);
    _bmp280_calib.dig_P4 = readS16_LE(BMP280_REGISTER_DIG_P4);
    _bmp280_calib.dig_P5 = readS16_LE(BMP280_REGISTER_DIG_P5);
    _bmp280_calib.dig_P6 = readS16_LE(BMP280_REGISTER_DIG_P6);
    _bmp280_calib.dig_P7 = readS16_LE(BMP280_REGISTER_DIG_P7);
    _bmp280_calib.dig_P8 = readS16_LE(BMP280_REGISTER_DIG_P8);
    _bmp280_calib.dig_P9 = readS16_LE(BMP280_REGISTER_DIG_P9);
}

/* -- Reads 24 bits -----------------------------------------------------------
 * unsigned
---------------------------------------------------------------------------- */
uint32_t DFRobot_BMP280::read24(byte reg) {

  // read 16 bits from register reg
  Wire.beginTransmission((uint8_t) _addr);
  Wire.write((uint8_t)reg);
  _rwError |= (1 << Wire.endTransmission(true)) >> 1;
  Wire.requestFrom((uint8_t)_addr, (byte)3);

  uint32_t data = Wire.read();
  data <<= 8;
  data |= Wire.read();
  data <<= 8;
  data |= Wire.read();

  return data;
}

/* -- Reads 16 bits -----------------------------------------------------------
 * unsigned
---------------------------------------------------------------------------- */
uint16_t DFRobot_BMP280::read16(byte reg) {

  // read 16 bits from register reg
  Wire.beginTransmission((uint8_t) _addr);
  Wire.write((uint8_t)reg);
  _rwError |= (1 << Wire.endTransmission(true)) >> 1;
  Wire.requestFrom((uint8_t)_addr, (byte) 2);
  uint16_t data = Wire.read() << 8 | Wire.read();

  return data;
}

/* -- Reads 16 bits -----------------------------------------------------------
 * unsigned
 * little endian
---------------------------------------------------------------------------- */
uint16_t DFRobot_BMP280::read16_LE(byte reg) {
  uint16_t temp = read16(reg);

  return (temp >> 8) | (temp << 8);
}

/* -- Reads 16 bits -----------------------------------------------------------
 * signed
---------------------------------------------------------------------------- */
int16_t DFRobot_BMP280::readS16(byte reg)
{return (int16_t)read16(reg);}

/* -- Reads 16 bits -----------------------------------------------------------
 * signed
 * little endian
---------------------------------------------------------------------------- */
int16_t DFRobot_BMP280::readS16_LE(byte reg)
{return (int16_t)read16_LE(reg);}

/* -- Reads 8 bits ------------------------------------------------------------
 * unsigned
---------------------------------------------------------------------------- */
uint8_t DFRobot_BMP280::read8(byte reg) {

  // read 16 bits from register reg
  Wire.beginTransmission((uint8_t)_addr);
  Wire.write((uint8_t)reg);
  _rwError |= (1 << Wire.endTransmission(true)) >> 1;
  Wire.requestFrom((uint8_t)_addr, (byte)1);
  uint8_t data = Wire.read();

  return data;
}

/* -- Writes 8 bits -----------------------------------------------------------
 * unsigned
---------------------------------------------------------------------------- */
void DFRobot_BMP280::write8(byte reg, byte data) {
  // writes 8 bits of data to register reg
  Wire.beginTransmission((uint8_t)_addr);
  Wire.write((uint8_t)reg);
  Wire.write((uint8_t)data);
  _rwError |= (1 << Wire.endTransmission(true)) >> 1;
}
