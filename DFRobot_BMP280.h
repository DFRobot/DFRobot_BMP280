/*!
 * @file DFRobot_BMP280.h
 * @brief Provides an Arduino library for reading and interpreting Bosch BMP280 data over I2C. 
 * @n Used to read current temperature, air pressure and calculate altitude.
 *
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author [Frank](jiehan.guo@dfrobot.com)
 * @version  V1.0
 * @date  2019-03-12
 * @url https://github.com/DFRobot/DFRobot_BMP280
 */

#ifndef DFROBOT_BMP280_H
#define DFROBOT_BMP280_H

#include "Arduino.h"
#include "Wire.h"

#ifndef PROGMEM
# define PROGMEM
#endif

class DFRobot_BMP280 {

public:
  /**
   * @enum eStatus_t
   * @brief Enum global status
   */
  typedef enum {
    eStatusOK,
    eStatusErr,
    eStatusErrDeviceNotDetected,
    eStatusErrParameter
  } eStatus_t;

  typedef struct {
    uint16_t    t1;
    int16_t     t2, t3;
    uint16_t    p1;
    int16_t     p2, p3, p4, p5, p6, p7, p8, p9;
    uint16_t    reserved0;
  } sCalibrateDig_t;

  typedef struct {
    uint8_t   im_update: 1;
    uint8_t   reserved: 2;
    uint8_t   measuring: 1;
  } sRegStatus_t;

  /**
   * @enum eCtrlMeasMode_t
   * @brief Enum control measurement mode (power)
   */
  typedef enum {
    eCtrlMeasModeSleep,
    eCtrlMeasModeForced,
    eCtrlMeasModeNormal = 0x03
  } eCtrlMeasMode_t;

  /**
   * @enum  eSampling_t
   * @brief Enum sampling
   */
  typedef enum {
    eSampling_no,
    eSampling_X1,
    eSampling_X2,
    eSampling_X4,
    eSampling_X8,
    eSampling_X16
  } eSampling_t;

  typedef struct {
    uint8_t   mode: 2;
    uint8_t   osrs_p: 3;
    uint8_t   osrs_t: 3;
  } sRegCtrlMeas_t;

  typedef enum {
    eConfigSpi3w_en_disable,
    eConfigSpi3w_en_enable
  } eConfigSpi3w_en_t;

  /**
   * @enum eConfigFilter_t
   * @brief Enum config filter
   */
  typedef enum {
    eConfigFilter_off,
    eConfigFilter_X2,
    eConfigFilter_X4,
    eConfigFilter_X8,
    eConfigFilter_X16
  } eConfigFilter_t;

  /**
   * @enum eConfigTStandby_t
   * @brief Enum config standby time, unit ms
   */
  typedef enum {
    eConfigTStandby_0_5,    /**< 0.5 ms */
    eConfigTStandby_62_5,
    eConfigTStandby_125,
    eConfigTStandby_250,
    eConfigTStandby_500,
    eConfigTStandby_1000,
    eConfigTStandby_2000,
    eConfigTStandby_4000
  } eConfigTStandby_t;

  typedef struct {
    uint8_t   spi3w_en: 1;
    uint8_t   reserved1: 1;
    uint8_t   filter: 3;
    uint8_t   t_sb: 3;
  } sRegConfig_t;

  typedef struct {
    uint8_t   msb, lsb;
    uint8_t   reserved: 4;
    uint8_t   xlsb: 4;
  } sRegPress_t;

  typedef struct {
    uint8_t   msb, lsb;
    uint8_t   reserved: 4;
    uint8_t   xlsb: 4;
  } sRegTemp_t;

  #define BMP280_REG_START    0x88
  typedef struct {
    sCalibrateDig_t   calib;
    uint8_t   reserved0[(0xd0 - 0xa1 - 1)];
    uint8_t   chip_id;
    #define BMP280_REG_CHIP_ID_DEFAULT    0x58
    uint8_t   reserved1[(0xe0 - 0xd0 - 1)];
    uint8_t   reset;
    uint8_t   reserved2[(0xf3 - 0xe0 - 1)];
    sRegStatus_t    status;
    sRegCtrlMeas_t    ctrlMeas;
    sRegConfig_t      config;
    uint8_t   reserved3;
    sRegPress_t   press;
    sRegTemp_t    temp;
  } sRegs_t;

public:
  DFRobot_BMP280();

  /**
   * @fn begin
   * @brief begin Sensor begin
   * @return Enum of eStatus_t
   */
  eStatus_t begin();

  /**
   * @fn getTemperature
   * @brief getTemperature Get temperature
   * @return Temprature in Celsius
   */
  float  getTemperature();

  /**
   * @fn getPressure
   * @brief getPressure Get pressure
   * @return Pressure in pa
   */
  uint32_t getPressure();

  /**
   * @fn calAltitude
   * @brief calAltitude Calculate altitude
   * @param seaLevelPressure Sea level pressure
   * @param pressure Pressure in pa
   * @return Altitude in meter
   */
  float calAltitude(float seaLevelPressure, uint32_t pressure);

  /**
   * @fn reset
   * @brief reset Reset sensor
   */
  void reset();

  /**
   * @fn setCtrlMeasMode
   * @brief setCtrlMeasMode Set control measure mode
   * @param eMode One enum of eCtrlMeasMode_t
   */
  void setCtrlMeasMode(eCtrlMeasMode_t eMode);

  /**
   * @fn setCtrlMeasSamplingTemp
   * @brief setCtrlMeasSamplingTemp Set control measure temperature oversampling
   * @param eSampling One enum of eSampling_t
   */
  void setCtrlMeasSamplingTemp(eSampling_t eSampling);

  /**
   * @fn setCtrlMeasSamplingPress
   * @brief setCtrlMeasSamplingPress Set control measure pressure oversampling
   * @param eSampling One enum of eSampling_t
   */
  void setCtrlMeasSamplingPress(eSampling_t eSampling);

  /**
   * @fn setConfigFilter
   * @brief setConfigFilter Set config filter
   * @param eFilter One enum of eConfigFilter_t
   */
  void setConfigFilter(eConfigFilter_t eFilter);

  /**
   * @fn setConfigTStandby
   * @brief setConfigTStandby Set config standby time
   * @param eT One enum of eConfigTStandby_t
   */
  void setConfigTStandby(eConfigTStandby_t eT);

protected:
  void    getCalibrate();

  int32_t   getTemperatureRaw();
  int32_t   getPressureRaw();

  uint8_t   getReg(uint8_t reg);
  void      writeRegBits(uint8_t reg, uint8_t field, uint8_t val);

  virtual void    writeReg(uint8_t reg, uint8_t *pBuf, uint16_t len) = 0;
  virtual void    readReg(uint8_t reg, uint8_t *pBuf, uint16_t len) = 0;


public:
  eStatus_t   lastOperateStatus; /**< lastOperateStatus Last operate status*/

protected:
  int32_t   _t_fine;

  sCalibrateDig_t   _sCalib;
};

class DFRobot_BMP280_IIC : public DFRobot_BMP280 {
public:
  /**
   * @enum eSdo_t
   * @brief Enum pin sdo states
   */
  typedef enum {
    eSdoLow,
    eSdoHigh
  } eSdo_t;

  /**
   * @fn DFRobot_BMP280_IIC
   * @brief DFRobot_BMP280_IIC
   * @param pWire Which TwoWire peripheral to operate
   * @param eSdo Pin sdo status
   */
  DFRobot_BMP280_IIC(TwoWire *pWire, eSdo_t eSdo);

protected:
  void    writeReg(uint8_t reg, uint8_t *pBuf, uint16_t len);
  void    readReg(uint8_t reg, uint8_t *pBuf, uint16_t len);

protected:
  TwoWire   *_pWire;

  uint8_t   _addr;

};

#endif
