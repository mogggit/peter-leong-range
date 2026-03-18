// original info:
/*!
 * @file DFRobot_GNSS.h
 * @brief Define the basic structure of the DFRobot_GNSS class, the implementation of the basic methods
 * @copyright	Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license The MIT License (MIT)
 * @author [ZhixinLiu](zhixin.liu@dfrobot.com)
 * @version V1.0
 * @date 2023-03-07
 * @url https://github.com/DFRobot/DFRobot_GNSS
 */
 //modified to be compatible with DFRobot_GNSS.c
 
#ifndef __DFRobot_GNSS_H__
#define __DFRobot_GNSS_H__

#include "main.h"

	#define GNSS_DEVICE_ADDR (0x66 << 1)
  #define I2C_YEAR_H 0
  #define I2C_YEAR_L 1
  #define I2C_MONTH 2
  #define I2C_DATE  3
  #define I2C_HOUR  4
  #define I2C_MINUTE 5
  #define I2C_SECOND 6
  #define I2C_LAT_1 7
  #define I2C_LAT_2 8
  #define I2C_LAT_X_24 9
  #define I2C_LAT_X_16 10
  #define I2C_LAT_X_8  11
  #define I2C_LON_DIS  12
  #define I2C_LON_1 13
  #define I2C_LON_2 14
  #define I2C_LON_X_24 15
  #define I2C_LON_X_16 16
  #define I2C_LON_X_8  17
  #define I2C_LAT_DIS  18
  #define I2C_USE_STAR 19
  #define I2C_ALT_H 20
  #define I2C_ALT_L 21
  #define I2C_ALT_X 22

  #define I2C_SOG_H 23
  #define I2C_SOG_L 24
  #define I2C_SOG_X 25
  #define I2C_COG_H 26
  #define I2C_COG_L 27
  #define I2C_COG_X 28

  #define I2C_START_GET 29
  #define I2C_ID 30
  #define I2C_DATA_LEN_H 31
  #define I2C_DATA_LEN_L 32
  #define I2C_ALL_DATA 33

  #define I2C_GNSS_MODE 34
  #define I2C_SLEEP_MODE 35
  #define I2C_RGB_MODE 36

  #define I2C_FLAG  1
  #define UART_FLAG 2
  #define TIME_OUT  500            ///< time out

  #define ENABLE_POWER 0
  #define DISABLE_POWER 1
  
  #define RGB_ON 0x05
  #define RGB_OFF 0x02

/**
 * @struct sTim_t
 * @brief Store the time and date information obtained from GPS 
 */
typedef struct {
  uint16_t year;
  uint8_t month;
  uint8_t date;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
}sTim_t;

/**
 * @struct sLonLat_t
 * @brief Store latitude, longitude and direction information obtained from GPS 
 */
typedef struct {
  uint8_t lonDDD;
  uint8_t lonMM;
  uint32_t lonMMMMM;
  char lonDirection;
  uint8_t latDD;
  uint8_t latMM;
  uint32_t latMMMMM;
  char latDirection;
  double latitude;
  double latitudeDegree;
  double lonitude;
  double lonitudeDegree;
}sLonLat_t;


/**
 * @brief Set GNSS 
 */
typedef enum {
  eGPS=1,
  eBeiDou,
  eGPS_BeiDou,
  eGLONASS,
  eGPS_GLONASS,
  eBeiDou_GLONASS,
  eGPS_BeiDou_GLONASS,
}eGnssMode_t;

typedef struct {
    I2C_HandleTypeDef *hi2c;  // Pointer to your CubeMX I2C handle
    uint8_t addr;             // Device address
    uint8_t uartI2CFlag;
    void (*callback)(char *data, uint8_t len);
} DFRobot_GNSS_t;

void DFRobot_GNSS_Init(DFRobot_GNSS_t *self, I2C_HandleTypeDef *hi2c, uint8_t addr);

/**
 * @fn getUTC
 * @brief Get UTC, standard time 
 * @return sTim_t type, represents the returned hour, minute and second 
 * @retval sTim_t.hour hour 
 * @retval sTim_t.minute minute 
 * @retval sTim_t.second second 
 */
  sTim_t getUTC(DFRobot_GNSS_t *self);

/**
 * @fn getDate
 * @brief Get date information, year, month, day 
 * @return sTim_t type, represents the returned year, month, day 
 * @retval sTim_t.year year
 * @retval sTim_t.month month 
 * @retval sTim_t.day day 
 */
  sTim_t getDate(DFRobot_GNSS_t *self);

/**
 * @fn getLat
 * @brief Get latitude 
 * @return sLonLat_t type, represents the returned latitude  
 * @retval sLonLat_t.latDD   Latitude degree(0-90)
 * @retval sLonLat_t.latMM   The first and second digits behind the decimal point 
 * @retval sLonLat_t.latMMMMM Latitude  The third and seventh digits behind the decimal point 
 * @retval sLonLat_t.latitude Latitude value with 7 decimal digits
 * @retval sLonLat_t.latDirection Direction of latitude
 */
  sLonLat_t getLat(DFRobot_GNSS_t *self);

/**
 * @fn getLon
 * @brief Get longitude 
 * @return sLonLat_t Type, represents the returned longitude
 * @retval sLonLat_t.lonDDD  Longitude degree(0-90)
 * @retval sLonLat_t.lonMM   Longitude  The first and second digits behind the decimal point
 * @retval sLonLat_t.lonMMMMM Longitude The third and seventh digits behind the decimal point
 * @retval sLonLat_t.lonitude Longitude value with 7 decimal digits
 * @retval sLonLat_t.lonDirection Direction of longitude 
 */
  sLonLat_t getLon(DFRobot_GNSS_t *self);

/**
 * @fn getNumSatUsed
 * @brief Get the number of the used satellite used
 * @return uint8_t type, represents the number of the used satellite
 */
  uint8_t getNumSatUsed(DFRobot_GNSS_t *self);

/**
 * @fn getAlt
 * @brief Get altitude
 * @return double type, represents altitude 
 */
  double getAlt(DFRobot_GNSS_t *self);

/**
 * @fn getSog
 * @brief Get speed over ground 
 * @return speed Float data(unit: knot)
 */
  double getSog(DFRobot_GNSS_t *self);

/**
 * @fn getCog
 * @brief Get course over ground
 * @return Float data(unit: degree) 
 */
  double getCog(DFRobot_GNSS_t *self);

/**
 * @fn setGnss
 * @brief Set GNSS to be used
 * @param mode
 * @n   eGPS              use gps
 * @n   eBeiDou           use beidou
 * @n   eGPS_BeiDou       use gps + beidou
 * @n   eGLONASS          use glonass
 * @n   eGPS_GLONASS      use gps + glonass
 * @n   eBeiDou_GLONASS   use beidou +glonass
 * @n   eGPS_BeiDou_GLONASS use gps + beidou + glonass
 * @return NULL
 */
  void setGnss(DFRobot_GNSS_t *self, eGnssMode_t mode);

/**
 * @fn getGnssMode
 * @brief Get the used gnss mode
 * @return mode
 * @retval 1 gps
 * @retval 2 beidou
 * @retval 3 gps + beidou
 * @retval 4 glonass
 * @retval 5 gps + glonass
 * @retval 6 beidou +glonass
 * @retval 7 gps + beidou + glonass
 */
  uint8_t getGnssMode(DFRobot_GNSS_t *self);

/**
 * @fn getAllGnss
 * @brief Get GNSS data, call back and receive
 * @return null
 */
  void getAllGnss(DFRobot_GNSS_t *self);

/**
 * @fn enablePower
 * @brief Enable gnss power
 * @return null
 */
void enablePower(DFRobot_GNSS_t *self);

/**
 * @fn disablePower
 * @brief Disable gnss power
 * @return null
 */
void disablePower(DFRobot_GNSS_t *self);

/**
 * @fn setRgbOn
 * @brief Turn rgb on
 * @return null
 */
void setRgbOn(DFRobot_GNSS_t *self);

/**
 * @fn setRgbOn
 * @brief Turn rgb off
 * @return null
 */
void setRgbOff(DFRobot_GNSS_t *self);


/**
 * @fn setCallback
 * @brief Set callback function type
 * @param  call function name 
 * @return null
 */
void setCallback(DFRobot_GNSS_t *self, void (*call)(char *, uint8_t));

#endif