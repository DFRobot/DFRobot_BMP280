/*!
 * config.ino
 *
 * Download this demo to test config to bmp280, connect sensor through IIC interface
 * Data will print on your serial monitor
 *
 * Copyright   [DFRobot](http://www.dfrobot.com), 2016
 * Copyright   GNU Lesser General Public License
 *
 * version  V1.0
 * date  12/03/2019
 */

#include "DFRobot_BMP280.h"
#include "Wire.h"

typedef DFRobot_BMP280_IIC    BME;    // ******** use abbreviations instead of full names ********

BME   bme(&Wire, BME::eSdo_low);

#define SEA_LEVEL_PRESSURE    1015.0f   // sea level pressure

// show last sensor operate status
void printLastOperateStatus(BME::eStatus_t eStatus)
{
  switch(eStatus) {
  case BME::eStatusOK:    Serial.println("everything ok"); break;
  case BME::eStatusErr:   Serial.println("unknow error"); break;
  case BME::eStatusErrDeviceNotDetected:    Serial.println("device not detected"); break;
  case BME::eStatusErrParameter:    Serial.println("parameter error"); break;
  default: Serial.println("unknow status"); break;
  }
}

void setup()
{
  Serial.begin(115200);
  bme.reset();
  Serial.println("bmp config test");
  while(bme.begin() != BME::eStatusOK) {
    Serial.println("bmp begin faild");
    printLastOperateStatus(bme.lastOperateStatus);
    delay(2000);
  }
  Serial.println("bmp begin success");

  bme.setConfigFilter(BME::eConfigFilter_off);        // set config filter
  bme.setConfigTStandby(BME::eConfigTStandby_125);    // set standby time
  bme.setCtrlMeasSamplingTemp(BME::eSampling_X8);     // set temperature over sampling
  bme.setCtrlMeasSamplingPress(BME::eSampling_X8);    // set pressure over sampling
  bme.setCtrlMeasMode(BME::eCtrlMeasMode_normal);     // set control measurement mode to make these settings effective

  delay(100);
}

void loop()
{
  float   temp = bme.getTemperature();
  uint32_t    press = bme.getPressure();
  float   alti = bme.calAltitude(SEA_LEVEL_PRESSURE, press);

  Serial.println();
  Serial.println("======== start print ========");
  Serial.print("temperature (unit Celsius): "); Serial.println(temp);
  Serial.print("pressure (unit pa):         "); Serial.println(press);
  Serial.print("altitude (unit meter):      "); Serial.println(alti);
  Serial.println("========  end print  ========");

  delay(1000);
}
