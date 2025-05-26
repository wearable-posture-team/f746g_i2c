#include "sensors.h"

static const char *TAG = "sensor";

BMP180_CAL_TypeDef bmp180_calibration_data;
bool bmp180_initialized = false;

void sensor_init(I2C_HandleTypeDef *hi2c1, I2C_HandleTypeDef *hi2c2) {
    HAL_StatusTypeDef ret = HAL_OK;
    uint8_t data;

    // Reading calibration data from BMP180 sensor's eeprom initializes the sensor
    ret = BMP180_READ_CALDATA(hi2c1, &bmp180_calibration_data);
    if(ret == HAL_OK) {
        printf("%s: BMP180 initialized : %d\r\n", TAG, ret);
        bmp180_initialized = true;
    }
    else {
        printf("%s: BMP180 initialization failed: %d\r\n", TAG, ret);
    }

    // Check if we can communicate with the sensor
    ret = HAL_I2C_Mem_Read(hi2c2, TH09C_ADDRESS << 1, TH09C_REG_SENS_STAT, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
    if (ret != HAL_OK) {
        printf("%s: Failed to communicate with TH09C sensor: %d\r\n", TAG, ret);
    }
    else {
        printf("%s: TH09C sensor found, status: 0x%02x\r\n", TAG, data);
    }

    // Configure sensor for single shot mode (0x00 = single shot mode for both sensors)
    data = 0x00;
    ret = HAL_I2C_Mem_Write(hi2c2, TH09C_ADDRESS << 1, TH09C_REG_SENS_RUN, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
    if (ret != HAL_OK) {
        printf("%s: Failed to set TH09C run mode: %d\r\n", TAG, ret);
    }

    // Configure system control register - enable low power mode
    data = TH09C_LOW_POWER_BIT;  // Set low power bit
    ret = HAL_I2C_Mem_Write(hi2c2, TH09C_ADDRESS << 1, TH09C_REG_SYS_CTRL, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
    if (ret != HAL_OK) {
        printf("%s: Failed to configure TH09C system control: %d\r\n", TAG, ret);
    }

    printf("%s: TH09C sensor initialized successfully\r\n", TAG);
}

bmp180_reading_t sensor_read_bmp180(I2C_HandleTypeDef *hi2c) {
    BMP180_CTRL_TypeDef temp_result, press_result;
    uint32_t ut, up;
    bmp180_reading_t reading = {0, 0, HAL_OK};
    
    if (!bmp180_initialized) {
        printf("%s: BMP180: Sensor not initialized\r\n", TAG);
        reading.status = HAL_ERROR;
        return reading;
    }
    
    // Step 1: Read uncompensated(raw) temp data
    reading.status = BMP180_READ_UT(hi2c, &temp_result);
    if (reading.status != HAL_OK) {
        printf("%s: BMP180: Failed to read temperature\r\n", TAG);
        return reading;
    }
    
    // Step 2: Calculate temperature to get B5 value
    ut = (((uint32_t)temp_result.MSB) << 8) | temp_result.LSB;
    reading.temperature = BMP180_CALC_TEMP(ut, &bmp180_calibration_data);
    
    // Step 3: Read uncompensated pressure
    reading.status = BMP180_READ_UP(hi2c, &press_result, BMP180_OSS);
    if (reading.status != HAL_OK) {
        printf("%s: Failed to read BMP180 pressure\r\n", TAG);
        return reading;
    }
    
    // Step 4: Calculate pressure with correct bit assembly
    up = (((uint32_t)press_result.MSB) << 16 | press_result.LSB << 8 | press_result.XLSB) >> (8-BMP180_OSS);
    reading.pressure = BMP180_CALC_PRESS(up, &bmp180_calibration_data, BMP180_OSS);
    
    return reading;
}

uint32_t crc7(uint32_t val) {
    // Setup polynomial
    uint32_t pol = CRC7POLY;

    // Align polynomial with data
    pol = pol << (DATA7WIDTH - CRC7WIDTH - 1);

    // Loop variable (indicates which bit to test, start with highest)
    uint32_t bit = DATA7MSB;

    // Make room for CRC value
    val = val << CRC7WIDTH;
    bit = bit << CRC7WIDTH;
    pol = pol << CRC7WIDTH;

    // Insert initial vector
    val |= CRC7IVEC;

    // Apply division until all bits done
    while (bit & (DATA7MASK << CRC7WIDTH)) {
        if (bit & val) val ^= pol;
        bit >>= 1;
        pol >>= 1;
    }

    return val;
}

// Read temperature and humidity from TH09C sensor
th09c_reading_t sensor_read_th09c(I2C_HandleTypeDef *hi2c) {
    HAL_StatusTypeDef ret;
    uint8_t data;

    th09c_reading_t reading = {0, 0, 0, 0, HAL_OK};

    // Start both temperature and humidity measurements
    data = TH09C_TEMP_BIT | TH09C_HUM_BIT;  // Set bits for both sensors
    reading.status = HAL_I2C_Mem_Write(hi2c, TH09C_ADDRESS << 1, TH09C_REG_SENS_START, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
    if (reading.status != HAL_OK) {
        printf("%s: Failed to start TH09C measurements: %d\r\n", TAG, reading.status);
        return reading;
    }

    // Wait for measurements to complete
    HAL_Delay(TH09C_CONVERSION_TIME_MS);

    // Check if measurements are complete
    reading.status = HAL_I2C_Mem_Read(hi2c, TH09C_ADDRESS << 1, TH09C_REG_SENS_STAT, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
    if (reading.status != HAL_OK) {
        printf("%s: Failed to read sensor status: %d\r\n", TAG, reading.status);
        return reading;
    }

    if (data & (TH09C_TEMP_BIT | TH09C_HUM_BIT)) {
        printf("%s: TH09C measurements still in progress, status: 0x%02x\r\n", TAG, data);
        // We'll try to read anyway
    }

    // Read temperature value (3 bytes)
    uint8_t t_val_buf[3];
    reading.status = HAL_I2C_Mem_Read(hi2c, TH09C_ADDRESS << 1, TH09C_REG_T_VAL, I2C_MEMADD_SIZE_8BIT, t_val_buf, sizeof(t_val_buf), HAL_MAX_DELAY);
    if (reading.status == HAL_OK) {
        // Extract the fields according to datasheet (little endian)
        uint32_t t_val = (t_val_buf[2] << 16) | (t_val_buf[1] << 8) | t_val_buf[0];
        uint32_t t_data = (t_val >> 0) & 0xFFFF;          // 16-bit data (bits 0-15)
        uint32_t t_valid = (t_val >> 16) & 0x1;           // Valid flag (bit 16)
        uint32_t t_crc = (t_val >> 17) & 0x7F;            // CRC (bits 17-23)

        // Check CRC
        uint32_t t_payload = (t_val >> 0) & 0x1FFFF;      // CRC is calculated over 17 bits
        bool t_crc_ok = (crc7(t_payload) == t_crc);

        if (t_valid && t_crc_ok) {
            // Convert to temperature according to datasheet
            float temp_kelvin = (float)t_data / 64.0f;    // Temperature in Kelvin
            reading.temperature = temp_kelvin - 273.15f;  // Convert to Celsius
            reading.temp_valid = true;

            printf("%s: Temperature: %.1fK, %.1fC\r\n", TAG, temp_kelvin, reading.temperature);
        } else {
            printf("%s: Temperature data not valid or CRC error (valid=%lu, crc_ok=%d)\r\n", TAG,
                   (unsigned long)t_valid, t_crc_ok);
        }
    } else {
        printf("%s: Failed to read temperature data: %d\r\n", TAG, reading.status);
    }

    // Read humidity value (3 bytes)
    uint8_t h_val_buf[3];
    ret = HAL_I2C_Mem_Read(hi2c, TH09C_ADDRESS << 1, TH09C_REG_H_VAL, I2C_MEMADD_SIZE_8BIT, h_val_buf, sizeof(h_val_buf), HAL_MAX_DELAY);
    if (ret == HAL_OK) {
        // Extract the fields according to datasheet (little endian)
        uint32_t h_val = (h_val_buf[2] << 16) | (h_val_buf[1] << 8) | h_val_buf[0];
        uint32_t h_data = (h_val >> 0) & 0xFFFF;          // 16-bit data (bits 0-15)
        uint32_t h_valid = (h_val >> 16) & 0x1;           // Valid flag (bit 16)
        uint32_t h_crc = (h_val >> 17) & 0x7F;            // CRC (bits 17-23)

        // Check CRC
        uint32_t h_payload = (h_val >> 0) & 0x1FFFF;      // CRC is calculated over 17 bits
        bool h_crc_ok = (crc7(h_payload) == h_crc);

        if (h_valid && h_crc_ok) {
            // Convert to humidity according to datasheet
            reading.humidity = (float)h_data / 512.0f;   // Humidity in %RH
            reading.hum_valid = true;

            printf("%s: Humidity: %.1f%%\r\n", TAG, reading.humidity);
        } else {
            printf("%s: Humidity data not valid or CRC error (valid=%lu, crc_ok=%d)\r\n", TAG,
                   (unsigned long)h_valid, h_crc_ok);
        }
    } else {
        printf("%s: Failed to read humidity data: %d\r\n", TAG, ret);
    }

    return reading;
}
