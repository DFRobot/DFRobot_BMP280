/*!
 * @file bmp280test.ino
 * @brief DFRobot's Temperature、Pressure and Approx altitude
 * @n [Get the module here](等上架后添加商品购买链接)
 * @n This example read the Temperature、Pressure and Altitude from BMP280, and then print them
 * @n [Connection and Diagram](等上架后添加wiki链接)
 *
 * @copyright	[DFRobot](http://www.dfrobot.com), 2016
 * @copyright	GNU Lesser General Public License
 *
 * @author [yuxiang](1137717512@qq.com)
 * @version  V1.0
 * @date  2016-12-06
 */

#include <Wire.h>
#include "DFRobot_BMP280.h"

DFRobot_BMP280 bmp280; 

void setup() {
  Serial.begin(9600);
  Serial.println("BMP280 demo");
  
  if (!bmp280.begin()) {  
    Serial.println("Could not find a valid BMP280 sensor!");
    while (1);
  }
}

void loop() {
    Serial.print("Temperature = ");
    Serial.print(bmp280.readTemperatureValue());
    Serial.println(" *C");
    
    Serial.print("Pressure = ");
    Serial.print(bmp280.readPressureValue());
    Serial.println(" Pa");

    Serial.print("Altitude = ");
    Serial.print(bmp280.readAltitudeValue(1013.25)); // this should be adjusted to your local forcase
    Serial.println(" m");
    
    Serial.println();
    delay(2000);
}
