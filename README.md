# BMP280
DFRobot's Temperature、Pressure and Approx altitude

## DFRobot_BMP280 Library for Arduino
---------------------------------------------------------
Provides an Arduino library for reading and interpreting Bosch BMP280 data over I2C.Used to read current temperature and air pressure and calculate altitude.

## Table of Contents

* [Installation](#installation)
* [Methods](#methods)
* [Compatibility](#compatibility)
* [History](#history)
* [Credits](#credits)

<snippet>
<content>

## Installation

To use this library download the zip file, uncompress it to a folder named DFRobot_BMP280. 
Download the zip file first to use this library and uncompress it to a folder named DFRobot_BMP280. 

## Methods

```C++

/*
 * @brief init BMP280 device
 *
 * @return result
 *    ture : falid
 *    false : succussful
 */
    bool begin();

/*
 * @brief  Read temperature value.
 */
    float readTemperatureValue();
/*
 * @brief  Read Pressure value.
 */
    float readPressureValue();
/*
 * @brief  Read Altitude value.
 *         Altitude is calculated based on temperature and sea level pressure
 *         p = 101325 * (1-(0.0065*h/288.15))^5.25588; p=local pressure; h=altitude
 */
    float readAltitudeValue();

```

## Compatibility

MCU                | Work Well | Work Wrong | Untested  | Remarks
------------------ | :----------: | :----------: | :---------: | -----
FireBeetle-ESP32  |      √       |             |            | 
FireBeetle-ESP8266  |      √       |             |            | 
Arduino uno |       √      |             |            | 

## History

- Nov 31, 2018 - Version 0.1 released.

## Credits

Written by yuxiang, 2018. (Welcome to our [website](https://www.dfrobot.com/))
