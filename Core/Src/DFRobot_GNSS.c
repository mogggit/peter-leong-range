#include "DFRobot_GNSS.h"

static HAL_StatusTypeDef writeReg(DFRobot_GNSS_t *self, uint8_t reg, uint8_t *data, uint16_t len) {
    return HAL_I2C_Mem_Write(self->hi2c, self->addr, reg, I2C_MEMADD_SIZE_8BIT, data, len, 100);
}

static HAL_StatusTypeDef readReg(DFRobot_GNSS_t *self, uint8_t reg, uint8_t *data, uint16_t len) {
    return HAL_I2C_Mem_Read(self->hi2c, self->addr, reg, I2C_MEMADD_SIZE_8BIT, data, len, 100);
}

void GNSS_Init(DFRobot_GNSS_t *self, I2C_HandleTypeDef *hi2c) {
    self->hi2c = hi2c;
    self->addr = GNSS_DEVICE_ADDR;
    self->callback = NULL;
}

void GNSS_SetMode(DFRobot_GNSS_t *self, eGnssMode_t mode) {
    uint8_t m = (uint8_t)mode;
    writeReg(self, I2C_GNSS_MODE, &m, 1);
    HAL_Delay(50);
}

void GNSS_PowerControl(DFRobot_GNSS_t *self, bool enable) {
    uint8_t val = enable ? ENABLE_POWER : DISABLE_POWER;
    writeReg(self, I2C_SLEEP_MODE, &val, 1);
    HAL_Delay(50);
}

void GNSS_SetRGB(DFRobot_GNSS_t *self, bool on) {
    uint8_t val = on ? RGB_ON : RGB_OFF;
    writeReg(self, I2C_RGB_MODE, &val, 1);
}

sGNSS_Time_t GNSS_GetTime(DFRobot_GNSS_t *self) {
    sGNSS_Time_t t = {0};
    uint8_t buf[7];
    if (readReg(self, I2C_YEAR_H, buf, 7) == HAL_OK) {
        t.year   = ((uint16_t)buf[0] << 8) | buf[1];
        t.month  = buf[2];
        t.date   = buf[3];
        t.hour   = buf[4];
        t.minute = buf[5];
        t.second = buf[6];
    }
    return t;
}

sGNSS_Data_t GNSS_GetAllData(DFRobot_GNSS_t *self) {
    sGNSS_Data_t data = {0};
    uint8_t buf[6];

    // Latitude
    if (readReg(self, I2C_LAT_1, buf, 6) == HAL_OK) {
        uint32_t latMMMMM = ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 8) | (uint32_t)buf[4];
        data.latitude = (double)buf[0] + (double)buf[1] / 60.0 + (double)latMMMMM / 6000000.0;
    }
    readReg(self, I2C_LAT_DIS, (uint8_t*)&data.latDirection, 1);

    // Longitude
    if (readReg(self, I2C_LON_1, buf, 6) == HAL_OK) {
        uint32_t lonMMMMM = ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 8) | (uint32_t)buf[4];
        data.longitude = (double)buf[0] + (double)buf[1] / 60.0 + (double)lonMMMMM / 6000000.0;
    }
    readReg(self, I2C_LON_DIS, (uint8_t*)&data.lonDirection, 1);

    // Stats: Sats, Alt, SOG, COG
    data.satellites = 0;
    readReg(self, I2C_USE_STAR, &data.satellites, 1);

    if (readReg(self, I2C_ALT_H, buf, 3) == HAL_OK)
        data.altitude = (double)((uint16_t)(buf[0] & 0x7F) << 8 | buf[1]) + (double)buf[2] / 100.0;
    
    if (readReg(self, I2C_SOG_H, buf, 3) == HAL_OK)
        data.speedKnot = (double)((uint16_t)(buf[0] & 0x7F) << 8 | buf[1]) + (double)buf[2] / 100.0;

    if (readReg(self, I2C_COG_H, buf, 3) == HAL_OK)
        data.courseDegree = (double)((uint16_t)(buf[0] & 0x7F) << 8 | buf[1]) + (double)buf[2] / 100.0;

    return data;
}

void GNSS_ProcessRawNMEA(DFRobot_GNSS_t *self) {
    uint8_t start = 0x55;
    writeReg(self, I2C_START_GET, &start, 1);
    HAL_Delay(100);

    uint8_t lenBuf[2];
    if (readReg(self, I2C_DATA_LEN_H, lenBuf, 2) != HAL_OK) return;
    uint16_t len = ((uint16_t)lenBuf[0] << 8) | lenBuf[1];

    if (len == 0 || len > 1224) return;

    uint8_t dataBuf[32];
    uint16_t remaining = len;
    while (remaining > 0) {
        uint8_t chunk = (remaining > 32) ? 32 : (uint8_t)remaining;
        if (readReg(self, I2C_ALL_DATA, dataBuf, chunk) == HAL_OK) {
            if (self->callback) self->callback((char*)dataBuf, chunk);
        }
        remaining -= chunk;
    }
}

void GNSS_SetCallback(DFRobot_GNSS_t *self, void (*call)(char *, uint16_t)) {
    self->callback = call;
}