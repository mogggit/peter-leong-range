#ifndef __DFROBOT_GNSS_ONLY_H__
#define __DFROBOT_GNSS_ONLY_H__

#include "main.h" // For HAL_I2C and standard types
#include <stdint.h>
#include <stdbool.h>

#define GNSS_DEVICE_ADDR      (0x66<<1)

/* Register Map for GNSS Data */
#define I2C_YEAR_H            0
#define I2C_MONTH             2
#define I2C_DATE              3
#define I2C_HOUR              4
#define I2C_MINUTE            5
#define I2C_SECOND            6
#define I2C_LAT_1             7
#define I2C_LON_DIS           12
#define I2C_LON_1             13
#define I2C_LAT_DIS           18
#define I2C_USE_STAR          19
#define I2C_ALT_H             20
#define I2C_SOG_H             23
#define I2C_COG_H             26
#define I2C_START_GET         29
#define I2C_DATA_LEN_H        31
#define I2C_ALL_DATA          33
#define I2C_GNSS_MODE         34
#define I2C_SLEEP_MODE        35
#define I2C_RGB_MODE          36

/* Configuration Constants */
#define ENABLE_POWER          0
#define DISABLE_POWER         1
#define RGB_ON                0x05
#define RGB_OFF               0x02

typedef struct {
    uint16_t year;
    uint8_t month, date, hour, minute, second;
} sGNSS_Time_t;

typedef struct {
    double latitude;       // Decimal degrees (DD.DDDDDD)
    double longitude;      // Decimal degrees (DD.DDDDDD)
    char latDirection;     // 'N' or 'S'
    char lonDirection;     // 'E' or 'W'
    double altitude;       // Meters
    double speedKnot;      // Knots
    double courseDegree;   // Degrees
    uint8_t satellites;    // Number of satellites used
} sGNSS_Data_t;

typedef enum {
    eGPS = 1,
    eBeiDou,
    eGPS_BeiDou,
    eGLONASS,
    eGPS_GLONASS,
    eBeiDou_GLONASS,
    eGPS_BeiDou_GLONASS,
} eGnssMode_t;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t addr;
    void (*callback)(char *data, uint16_t len);
} DFRobot_GNSS_t;

/* Public API */
void GNSS_Init(DFRobot_GNSS_t *self, I2C_HandleTypeDef *hi2c);
void GNSS_SetMode(DFRobot_GNSS_t *self, eGnssMode_t mode);
void GNSS_PowerControl(DFRobot_GNSS_t *self, bool enable);
void GNSS_SetRGB(DFRobot_GNSS_t *self, bool on);

sGNSS_Time_t GNSS_GetTime(DFRobot_GNSS_t *self);
sGNSS_Data_t GNSS_GetAllData(DFRobot_GNSS_t *self);

/* NMEA Raw Data Retrieval */
void GNSS_ProcessRawNMEA(DFRobot_GNSS_t *self);
void GNSS_SetCallback(DFRobot_GNSS_t *self, void (*call)(char *, uint16_t));

#endif