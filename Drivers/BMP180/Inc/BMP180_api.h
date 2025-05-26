#ifndef BMP180_API_H
#define BMP180_API_H

#include "stm32f7xx_hal.h"

#define BMP180_DEV_ADDR                0xEE
#define BMP180_WAIT_UT                 45 //waiting time in 0.1ms
#define BMP180_WAIT_UP_LOWPOWER        45
#define BMP180_WAIT_UP_STANDARD        75
#define BMP180_WAIT_UP_HIGHRES         135
#define BMP180_WAIT_UP_ULTRAHIGHRES    255

//read-write-wait functions
#define BMP_READ(hi2c, register_addr, register_data, read_length) \
        HAL_I2C_Mem_Read(hi2c, BMP180_DEV_ADDR, register_addr, I2C_MEMADD_SIZE_8BIT, register_data, read_length, HAL_MAX_DELAY)
#define BMP_WRITE(hi2c, register_addr, register_data, len) \
        HAL_I2C_Mem_Write(hi2c, BMP180_DEV_ADDR, register_addr, I2C_MEMADD_SIZE_8BIT, register_data, len, HAL_MAX_DELAY)

#define Delay(msec) HAL_Delay(msec)

//oversampling settings
#define BMP180_OSS_LOWPOWER            0
#define BMP180_OSS_STANDARD            1
#define BMP180_OSS_HIGHRES             2
#define BMP180_OSS_ULTRAHIGHRES        3

//commands
#define BMP180_COM_CTRL_MEAS_TEMP      0x2E
#define BMP180_COM_CTRL_MEAS_PRESS     0x34
#define BMP180_COM_SOFT_RESET          0xB6

//control reg bits
#define BMP180_BIT_CTRL_MEAS_SCO       (1<<5)
#define BMP180_BIT_CTRL_MEAS_SCO_SHIFT 5
#define BMP180_BIT_CTRL_MEAS_OSS       (0x3<<6)
#define BMP180_BIT_CTRL_MEAS_OSS_SHIFT 6

//register values
#define BMP180_VALUE_ID                0x55
#define BMP180_CALREGS_BASE_ADDRESS    0xAA
#define BMP180_CTRLREGS_BASE_ADDRESS   0xF4

//addresses of control registers
#define BMP180_ADDR_OUT_XLSB           0xF8
#define BMP180_ADDR_OUT_LSB            0xF7
#define BMP180_ADDR_OUT_MSB            0xF6
#define BMP180_ADDR_CTRL_MEAS          0xF4
#define BMP180_ADDR_SOFT_RESET         0xE0
#define BMP180_ADDR_ID                 0xD0

//calibration registers
typedef struct{
	//integrated registers
	volatile short AC1;
	volatile short AC2;
	volatile short AC3;
	volatile unsigned short AC4;
	volatile unsigned short AC5;
	volatile unsigned short AC6;
	volatile short B1;
	volatile short B2;
	volatile short MB;
	volatile short MC;
	volatile short MD;
	//calculated parameters
	long B5;
}BMP180_CAL_TypeDef;

typedef struct{
	volatile unsigned char CTRL;
	volatile unsigned char DUMP;
	volatile unsigned char MSB;
	volatile unsigned char LSB;
	volatile unsigned char XLSB;
}BMP180_CTRL_TypeDef;

#define BMP180_CALREGS                 ((BMP180_CAL_TypeDef*) BMP180_CALREGS_BASE_ADDRESS)
#define BMP180_CTRLREGS                ((BMP180_CTRL_TypeDef*) BMP180_CTRLREGS_BASE_ADDRESS)

HAL_StatusTypeDef BMP180_READ_CALDATA(I2C_HandleTypeDef *hi2c, BMP180_CAL_TypeDef* reg_data);
HAL_StatusTypeDef BMP180_READ_UT(I2C_HandleTypeDef *hi2c, BMP180_CTRL_TypeDef* result);
HAL_StatusTypeDef BMP180_READ_UP(I2C_HandleTypeDef *hi2c, BMP180_CTRL_TypeDef* result, uint8_t oss);
short BMP180_CALC_TEMP(unsigned long ut,BMP180_CAL_TypeDef *reg_data);
long BMP180_CALC_PRESS(unsigned long up,BMP180_CAL_TypeDef *reg_data, unsigned char oss);

#endif
