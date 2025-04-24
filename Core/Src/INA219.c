#include "INA219.h"

// Biến toàn cục
uint16_t ina219_calibrationValue = 0;
int16_t ina219_currentDivider_mA = 0;
int16_t ina219_powerMultiplier_mW = 0;

// Đọc 16 bit từ thanh ghi
uint16_t Read16(INA219_t *ina219, uint8_t Register) {
    uint8_t Value[2];
    HAL_I2C_Mem_Read(ina219->ina219_i2c, (INA219_ADDRESS << 1), Register, 1, Value, 2, 1000);
    return ((Value[0] << 8) | Value[1]);
}

// Ghi 16 bit vào thanh ghi
void Write16(INA219_t *ina219, uint8_t Register, uint16_t Value) {
    uint8_t addr[2];
    addr[0] = (Value >> 8) & 0xff; // Byte cao
    addr[1] = (Value >> 0) & 0xff; // Byte thấp
    HAL_I2C_Mem_Write(ina219->ina219_i2c, (INA219_ADDRESS << 1), Register, 1, addr, 2, 1000);
}

// Hàm khởi tạo INA219
uint8_t INA219_Init(INA219_t *ina219, I2C_HandleTypeDef *i2c, uint8_t Address) {
    ina219->ina219_i2c = i2c;
    ina219->Address = Address;

    ina219_currentDivider_mA = 0;
    ina219_powerMultiplier_mW = 0;

    uint8_t ina219_isReady = HAL_I2C_IsDeviceReady(i2c, (Address << 1), 3, 2);

    if (ina219_isReady == HAL_OK) {
        INA219_Reset(ina219);
        INA219_setCalibration_32V_2A(ina219);
        return 1;
    } else {
        return 0;
    }
}

// Hàm reset INA219
void INA219_Reset(INA219_t *ina219) {
    Write16(ina219, INA219_REG_CONFIG, INA219_CONFIG_RESET);
    HAL_Delay(1);
}

// Hàm hiệu chuẩn cho dải 32V, 2A
void INA219_setCalibration_32V_2A(INA219_t *ina219) {
    uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
                      INA219_CONFIG_GAIN_8_320MV |
                      INA219_CONFIG_BADCRES_12BIT |
                      INA219_CONFIG_SADCRES_12BIT_1S_532US |
                      INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

    ina219_calibrationValue = 4096;
    ina219_currentDivider_mA = 10;
    ina219_powerMultiplier_mW = 2;

    Write16(ina219, INA219_REG_CALIBRATION, ina219_calibrationValue);
    Write16(ina219, INA219_REG_CONFIG, config);
}

// Đọc điện áp Bus (Vbus)
uint16_t INA219_ReadBusVoltage(INA219_t *ina219) {
    uint16_t rawValue = Read16(ina219, INA219_REG_BUSVOLTAGE);
    return (rawValue >> 3) * 4; // Dịch phải 3 bit và nhân 4 để ra mV
}

// Đọc điện áp Shunt (Vshunt)
uint16_t INA219_ReadShuntVoltage(INA219_t *ina219) {
    int16_t rawValue = (int16_t)Read16(ina219, INA219_REG_SHUNTVOLTAGE);
    return rawValue; // Giá trị trả về là uV
}

// Đọc dòng điện (Current)
int16_t INA219_ReadCurrent(INA219_t *ina219) {
    Write16(ina219, INA219_REG_CALIBRATION, ina219_calibrationValue);
    int16_t rawValue = (int16_t)Read16(ina219, INA219_REG_CURRENT);
    return rawValue / ina219_currentDivider_mA; // Trả về mA
}
