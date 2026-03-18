#include "DFRobot_GNSS.h"
#include <string.h>

/* --- Private Internal Helper Functions --- */

/**
 * @brief Private helper to write to I2C registers
 */
static void writeReg(DFRobot_GNSS_t *self, uint8_t reg, uint8_t *data, uint8_t len) {
    HAL_I2C_Mem_Write(self->hi2c, self->addr, reg, I2C_MEMADD_SIZE_8BIT, data, len, 100);
}

/**
 * @brief Private helper to read from I2C registers
 */
static int16_t readReg(DFRobot_GNSS_t *self, uint8_t reg, uint8_t *data, uint8_t len) {
    if (HAL_I2C_Mem_Read(self->hi2c, self->addr, reg, I2C_MEMADD_SIZE_8BIT, data, len, 100) == HAL_OK) {
        return 0;
    }
    return -1;
}

/* --- Initialization --- */

void DFRobot_GNSS_Init(DFRobot_GNSS_t *self, I2C_HandleTypeDef *hi2c, uint8_t addr) {
    self->hi2c = hi2c;
    self->addr = addr;
    self->uartI2CFlag = I2C_FLAG;
    self->callback = NULL;
}

/* --- Basic Data Getters --- */

sTim_t getDate(DFRobot_GNSS_t *self) {
    sTim_t data = {0};
    uint8_t buf[4] = {0};
    if (readReg(self, I2C_YEAR_H, buf, 4) == 0) {
        data.year = ((uint16_t)buf[0] << 8) | buf[1];
        data.month = buf[2];
        data.date = buf[3];
    }
    return data;
}

sTim_t getUTC(DFRobot_GNSS_t *self) {
    sTim_t data = {0};
    uint8_t buf[3] = {0};
    if (readReg(self, I2C_HOUR, buf, 3) == 0) {
        data.hour   = buf[0];
        data.minute = buf[1];
        data.second = buf[2];
    }
    return data;
}

sLonLat_t getLat(DFRobot_GNSS_t *self) {
    sLonLat_t data = {0};
    uint8_t buf[6] = {0};
    if (readReg(self, I2C_LAT_1, buf, 6) == 0) {
        data.latDD  = buf[0];
        data.latMM  = buf[1];
        data.latMMMMM = ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 8) | ((uint32_t)buf[4]);
        data.latitude = (double)data.latDD * 100.0 + ((double)data.latMM) + ((double)data.latMMMMM / 100000.0);
        data.latitudeDegree = (double)data.latDD + (double)data.latMM / 60.0 + (double)data.latMMMMM / 100000.0 / 60.0;
    }
    readReg(self, I2C_LAT_DIS, buf, 1);
    data.latDirection = buf[0];
    return data;
}

sLonLat_t getLon(DFRobot_GNSS_t *self) {
    sLonLat_t data = {0};
    uint8_t buf[6] = {0};
    if (readReg(self, I2C_LON_1, buf, 6) == 0) {
        data.lonDDD  = buf[0];
        data.lonMM   = buf[1];
        data.lonMMMMM = ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 8) | ((uint32_t)buf[4]);
        data.lonitude = (double)data.lonDDD * 100.0 + ((double)data.lonMM) + ((double)data.lonMMMMM / 100000.0);
        data.lonitudeDegree = (double)data.lonDDD + (double)data.lonMM / 60.0 + (double)data.lonMMMMM / 100000.0 / 60.0;
    }
    readReg(self, I2C_LON_DIS, buf, 1);
    data.lonDirection = buf[0];
    return data;
}

/* --- Environment & Status --- */

uint8_t getNumSatUsed(DFRobot_GNSS_t *self) {
    uint8_t buf[1] = {0};
    readReg(self, I2C_USE_STAR, buf, 1);
    return buf[0];
}

double getAlt(DFRobot_GNSS_t *self) {
    uint8_t buf[3] = {0};
    readReg(self, I2C_ALT_H, buf, 3);
    return (double)((uint16_t)(buf[0] & 0x7F) << 8 | buf[1]) + (double)buf[2] / 100.0;
}

double getSog(DFRobot_GNSS_t *self) {
    uint8_t buf[3] = {0};
    readReg(self, I2C_SOG_H, buf, 3);
    return (double)((uint16_t)(buf[0] & 0x7F) << 8 | buf[1]) + (double)buf[2] / 100.0;
}

double getCog(DFRobot_GNSS_t *self) {
    uint8_t buf[3] = {0};
    readReg(self, I2C_COG_H, buf, 3);
    return (double)((uint16_t)(buf[0] & 0x7F) << 8 | buf[1]) + (double)buf[2] / 100.0;
}

/* --- Configuration --- */

void setRgbOn(DFRobot_GNSS_t *self) {
    uint8_t val = RGB_ON;
    writeReg(self, I2C_RGB_MODE, &val, 1);
    HAL_Delay(50);
}

void setRgbOff(DFRobot_GNSS_t *self) {
    uint8_t val = RGB_OFF;
    writeReg(self, I2C_RGB_MODE, &val, 1);
    HAL_Delay(50);
}

void enablePower(DFRobot_GNSS_t *self) {
    uint8_t val = ENABLE_POWER;
    writeReg(self, I2C_SLEEP_MODE, &val, 1);
    HAL_Delay(50);
}

void disablePower(DFRobot_GNSS_t *self) {
    uint8_t val = DISABLE_POWER;
    writeReg(self, I2C_SLEEP_MODE, &val, 1);
    HAL_Delay(50);
}

void setGnss(DFRobot_GNSS_t *self, eGnssMode_t mode) {
    uint8_t val = (uint8_t)mode;
    writeReg(self, I2C_GNSS_MODE, &val, 1);
    HAL_Delay(50);
}

/* --- Advanced Data Handlers --- */

uint16_t getGnssLen(DFRobot_GNSS_t *self) {
    uint8_t val = 0x55;
    writeReg(self, I2C_START_GET, &val, 1);
    HAL_Delay(100);
    uint8_t buf[2];
    readReg(self, I2C_DATA_LEN_H, buf, 2);
    return (uint16_t)buf[0] << 8 | buf[1];
}

void getAllGnss(DFRobot_GNSS_t *self) {
    uint8_t buf[260] = {0};
    uint16_t len = getGnssLen(self);
    if (len > 1224 || len == 0) return;

    uint16_t remaining = len;
    while (remaining > 0) {
        uint8_t chunk = (remaining > 32) ? 32 : remaining;
        readReg(self, I2C_ALL_DATA, buf, chunk);
        
        for (uint8_t i = 0; i < chunk; i++) {
            if (buf[i] == '\0') buf[i] = '\n';
        }

        if (self->callback) {
            self->callback((char *)buf, chunk);
        }
        remaining -= chunk;
    }
}

void setCallback(DFRobot_GNSS_t *self, void (*call)(char *, uint8_t)) {
    self->callback = call;
}