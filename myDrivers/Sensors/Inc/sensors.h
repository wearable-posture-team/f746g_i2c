#ifndef SENSOR_H
#define SENSOR_H

#include "stm32f7xx_hal.h"
#include "BMP180_api.h"
#include <stdbool.h>  // Add this line to define bool type
#include <stdio.h>

// addresses of sensors
// #define BMP180_ADDRESS     0x77 already in BMP180_api.h
#define TH09C_ADDRESS      0x43

// BMP180
#define BMP180_PRESSURE_MSB_REG     0xF6
#define BMP180_PRESSURE_LSB_REG     0xF5
#define BMP180_PRESSURE_XLSB_REG    0xF4
#define BMP180_OSS                  BMP180_OSS_LOWPOWER

// TH09C
#define TH09C_REG_PART_ID    0x00    // Part ID register (2 bytes)
#define TH09C_REG_DIE_REV    0x02    // Die revision register (2 bytes)
#define TH09C_REG_UID        0x04    // Unique identifier register (8 bytes)
#define TH09C_REG_SYS_CTRL   0x10    // System control register
#define TH09C_REG_SYS_STAT   0x11    // System status register
#define TH09C_REG_SENS_RUN   0x21    // Sensor run mode register
#define TH09C_REG_SENS_START 0x22    // Sensor start register
#define TH09C_REG_SENS_STOP  0x23    // Sensor stop register
#define TH09C_REG_SENS_STAT  0x24    // Sensor status register
#define TH09C_REG_T_VAL      0x30    // Temperature value register
#define TH09C_REG_H_VAL      0x33    // Humidity value register

// Bit masks
#define TH09C_TEMP_BIT       0x01    // Temperature bit for control registers
#define TH09C_HUM_BIT        0x02    // Humidity bit for control registers
#define TH09C_VALID_BIT      0x10    // Valid data bit (bit 16 in the 3-byte format)
#define TH09C_LOW_POWER_BIT  0x01    // Low power bit in SYS_CTRL register
#define TH09C_ACTIVE_BIT     0x01    // System active bit in SYS_STAT register

// System constants
#define TH09C_CONVERSION_TIME_MS 130  // Max time for temp+humidity measurement

// CRC-7 polynomial constants
#define CRC7WIDTH  7                  // 7 bits CRC has polynomial of 7th order
#define CRC7POLY   0x89               // Polynomial coefficients (x^7+x^3+x^0)
#define CRC7IVEC   0x7F               // Initial vector (all 7 bits high)
#define DATA7WIDTH 17                 // Payload data width
#define DATA7MASK  ((1UL<<DATA7WIDTH)-1) // 0b0 1111 1111 1111 1111
#define DATA7MSB   (1UL<<(DATA7WIDTH-1)) // 0b1 0000 0000 0000 0000

// Struct to store temperature and humidity readings
typedef struct {
    float temperature;    // Temperature in degrees Celsius
    float humidity;       // Relative humidity in percentage
    bool temp_valid;      // Flag indicating if temperature reading is valid
    bool hum_valid;       // Flag indicating if humidity reading is valid
    HAL_StatusTypeDef  status;
} th09c_reading_t;

typedef struct {
    long temperature;
    long pressure;  // Include status to know if the reading was successful
    HAL_StatusTypeDef  status;
} bmp180_reading_t;

// function prototypes
void sensor_init(I2C_HandleTypeDef *hi2c1, I2C_HandleTypeDef *hi2c2);
bmp180_reading_t sensor_read_bmp180(I2C_HandleTypeDef *hi2c);
th09c_reading_t sensor_read_th09c(I2C_HandleTypeDef *hi2c);
uint32_t crc7(uint32_t val);

#endif
