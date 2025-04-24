#ifndef INC_INA219_H_
#define INC_INA219_H_

#include "stdint.h"
#include "stm32l1xx_hal.h" // Thay đổi theo dòng vi điều khiển của bạn

// Địa chỉ mặc định của INA219
#define INA219_ADDRESS (0x40)

// Các thanh ghi của INA219
#define INA219_REG_CONFIG (0x00)
#define INA219_REG_SHUNTVOLTAGE (0x01)
#define INA219_REG_BUSVOLTAGE (0x02)
#define INA219_REG_POWER (0x03)
#define INA219_REG_CURRENT (0x04)
#define INA219_REG_CALIBRATION (0x05)

// Cấu hình reset
#define INA219_CONFIG_RESET (0x8000)

// Định nghĩa cấu hình
#define INA219_CONFIG_BVOLTAGERANGE_16V (0x0000)
#define INA219_CONFIG_BVOLTAGERANGE_32V (0x2000)
#define INA219_CONFIG_GAIN_1_40MV (0x0000)
#define INA219_CONFIG_GAIN_8_320MV (0x1800)
#define INA219_CONFIG_BADCRES_12BIT (0x0180)
#define INA219_CONFIG_SADCRES_12BIT_1S_532US (0x0018)
#define INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS (0x07)

// Struct cho INA219
typedef struct {
    I2C_HandleTypeDef *ina219_i2c;
    uint8_t Address;
} INA219_t;

// Biến toàn cục
extern uint16_t ina219_calibrationValue;
extern int16_t ina219_currentDivider_mA;
extern int16_t ina219_powerMultiplier_mW;

// Hàm giao tiếp INA219
uint8_t INA219_Init(INA219_t *ina219, I2C_HandleTypeDef *i2c, uint8_t Address);
uint16_t INA219_ReadBusVoltage(INA219_t *ina219);
uint16_t INA219_ReadShuntVoltage(INA219_t *ina219);
int16_t INA219_ReadCurrent(INA219_t *ina219);
void INA219_Reset(INA219_t *ina219);
void INA219_setCalibration_32V_2A(INA219_t *ina219);
void INA219_setCalibration_32V_1A(INA219_t *ina219);

#endif /* INC_INA219_H_ */
