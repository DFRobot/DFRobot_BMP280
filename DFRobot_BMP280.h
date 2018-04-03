/*!
 * @file DFRobot_BMP280.h
 * @brief DFRobot's DFRobot_BMP280
 * @n DFRobot's Temperature、Pressure and Approx altitude
 *
 * @copyright	[DFRobot](http://www.dfrobot.com), 2016
 * @copyright	GNU Lesser General Public License
 *
 * @author [yuxiang](1137717512@qq.com)
 * @version  V1.0
 * @date  2016-12-6
 */
#ifndef __DFRobot_BMP280_H__
#define __DFRobot_BMP280_H__

#include "Arduino.h"
#include "Wire.h"

/*I2C ADDRESS/BITS/SETTINGS*/
#define BMP280_ADDRESS                (0x76)
#define BMP280_ADDRESS1                (0x77)
#define BMP280_CHIPID                 (0x58)

/*REGISTERS*/
    enum
    {
      BMP280_REGISTER_DIG_T1              = 0x88,
      BMP280_REGISTER_DIG_T2              = 0x8A,
      BMP280_REGISTER_DIG_T3              = 0x8C,

      BMP280_REGISTER_DIG_P1              = 0x8E,
      BMP280_REGISTER_DIG_P2              = 0x90,
      BMP280_REGISTER_DIG_P3              = 0x92,
      BMP280_REGISTER_DIG_P4              = 0x94,
      BMP280_REGISTER_DIG_P5              = 0x96,
      BMP280_REGISTER_DIG_P6              = 0x98,
      BMP280_REGISTER_DIG_P7              = 0x9A,
      BMP280_REGISTER_DIG_P8              = 0x9C,
      BMP280_REGISTER_DIG_P9              = 0x9E,

      BMP280_REGISTER_CHIPID             = 0xD0,
      BMP280_REGISTER_VERSION            = 0xD1,
      BMP280_REGISTER_SOFTRESET          = 0xE0,

      BMP280_REGISTER_CAL26              = 0xE1,  // R calibration stored in 0xE1-0xF0

      BMP280_REGISTER_CONTROL            = 0xF4,
      BMP280_REGISTER_CONFIG             = 0xF5,
      BMP280_REGISTER_PRESSUREDATA       = 0xF7,
      BMP280_REGISTER_TEMPDATA           = 0xFA,
    };

/*CALIBRATION DATA*/
    typedef struct
    {
      uint16_t digT1;
      int16_t  digT2;
      int16_t  digT3;

      uint16_t digP1;
      int16_t  digP2;
      int16_t  digP3;
      int16_t  digP4;
      int16_t  digP5;
      int16_t  digP6;
      int16_t  digP7;
      int16_t  digP8;
      int16_t  digP9;

      uint8_t  digH1;
      int16_t  digH2;
      uint8_t  digH3;
      int16_t  digH4;
      int16_t  digH5;
      int8_t   digH6;
    } tBmp280CalibData;
/*=========================================================================*/

class DFRobot_BMP280
{
  public:
    DFRobot_BMP280();

    bool  begin(uint8_t addr = BMP280_ADDRESS, uint8_t chipid = BMP280_CHIPID);
	/**************************************************************************/
    /*!
        @brief  Reads the temperature
    */
    /**************************************************************************/
    float readTemperatureValue(void);
	/**************************************************************************/
    /*!
        @brief Reads the pressue
    */
    /**************************************************************************/
    float readPressureValue(void);
	/**************************************************************************/
    /*!
        @brief Reads the altitude
    */
    /**************************************************************************/
    float readAltitudeValue(float seaLevelhPa = 1013.25);
	
  private:
    /**************************************************************************/
    /*!
        @brief  Reads the factory-set coefficients
    */
    /**************************************************************************/
    void readCoefficients(void);
    /*************************************************************************/
    /*!
        @brief  Writes an 8 bit value over I2C
    */
    /*************************************************************************/
    void      write8(byte reg, byte value);
	/************************************************************************/
   /*!
       @brief  Reads an 8 bit value over I2C
   */
   /**************************************************************************/
    uint8_t   read8(byte reg);
	 /************************************************************************/
   /*!
       @brief  Reads a 16 bit value over I2C
   */
   /**************************************************************************/
    uint16_t  read16(byte reg);
	
    uint32_t  read24(byte reg);
    int16_t   readS16(byte reg);
    uint16_t  read16_LE(byte reg); // little endian
    int16_t   readS16_LE(byte reg); // little endian

    uint8_t   _i2caddr;
    int32_t t_fine;

    tBmp280CalibData _bmp280Calib;

};

#endif
