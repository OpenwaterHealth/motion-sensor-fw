/*
 * ICM20948.c
 *
 *  Created on: Apr 20, 2025
 *      Author: GeorgeVigelette
 */

#include "main.h"
#include "ICM20948.h"
#include "utils.h"

#include <string.h>
#include <stdio.h>

volatile uint8_t current_bank = 0;

static inline HAL_StatusTypeDef ICM_readBytes(uint8_t reg, uint8_t *pData, uint16_t size)
{
    HAL_StatusTypeDef result;
    int retries = 3;

    for (int attempt = 0; attempt < retries; attempt++)
    {
        result = HAL_I2C_Master_Transmit(&ICM_I2C, ICM20948_ADDR << 1, &reg, 1, 100);
        if (result != HAL_BUSY) break;
    }
    if (result != HAL_OK) return result;

    for (int attempt = 0; attempt < retries; attempt++)
    {
        result = HAL_I2C_Master_Receive(&ICM_I2C, ICM20948_ADDR << 1, pData, size, 100);
        if (result != HAL_BUSY) break;
    }
    return result;
}

static HAL_StatusTypeDef ICM_WriteBytes(uint8_t reg, uint8_t *pData, uint16_t size)
{
    HAL_StatusTypeDef result = HAL_OK;
    int retries = 3;
    uint8_t buffer[16];

    if (size > sizeof(buffer) - 1) return HAL_ERROR; // prevent buffer overflow

    buffer[0] = reg;
    memcpy(&buffer[1], pData, size);

    for (int attempt = 0; attempt < retries; attempt++)
    {
    	result = HAL_I2C_Master_Transmit(&ICM_I2C, ICM20948_ADDR << 1, buffer, size + 1, 100);
        if (result != HAL_BUSY) break;
    }
    return result;
}


static inline void ICM_SelectBank(uint8_t bank)
{
	if (current_bank == bank) return;
    uint8_t val = bank;
    if (HAL_I2C_Mem_Write(&ICM_I2C, ICM20948_ADDR << 1, ICM20948_REG_BANK_SEL, I2C_MEMADD_SIZE_8BIT, &val, 1, 5) == HAL_OK)
    {
    	current_bank = bank;
    }
    else
    {
    	printf("SelectBank Err\r\n");
    }

}

uint8_t ICM_WHOAMI(void) {
	uint8_t data = 0x01;
	if(ICM_readBytes(ICM20948_WHO_AM_I_REG, &data, 1) != HAL_OK)
	{
		return 0x00;
	}
	return data;
}

uint8_t ICM_Init(void)
{
    HAL_StatusTypeDef status;
    uint8_t whoami = 0;
    current_bank = 0xFF;
    // 1. Read WHO_AM_I
    ICM_SelectBank(ICM20948_USER_BANK_0);
    status = ICM_readBytes(ICM20948_WHO_AM_I_REG, &whoami, 1);
    if (status != HAL_OK || whoami != 0xEA)
    {
        printf("ICM20948 not found. WHOAMI: 0x%02X\r\n", whoami);
        return HAL_ERROR;
    }
    printf("ICM20948 WHOAMI OK: 0x%02X\r\n", whoami);

    // 2. Reset device (set DEVICE_RESET bit in PWR_MGMT_1)
    uint8_t reset_cmd = 0x80;
    status = ICM_WriteBytes(ICM20948_PWR_MGMT_1, &reset_cmd, 1);
    HAL_Delay(100);  // Wait for reset
    if (status != HAL_OK) return status;

    // 3. Wake up and set clock source
    uint8_t pwr_mgmt_1 = 0x01;  // sleep=0, clock=auto
    status = ICM_WriteBytes(ICM20948_PWR_MGMT_1, &pwr_mgmt_1, 1);
    if (status != HAL_OK) return status;
    HAL_Delay(10);

    // 4. Enable all sensors (Accel, Gyro, Temp)
    uint8_t pwr_mgmt_2 = 0x00; // All on
    status = ICM_WriteBytes(ICM20948_PWR_MGMT_2, &pwr_mgmt_2, 1);
    if (status != HAL_OK) return status;

    // 4.5 Disable Low Power Mode (LP_EN = 0)
    uint8_t lp_config = 0x00;
    status = ICM_WriteBytes(ICM20948_LP_CONFIG, &lp_config, 1);  // LP_CONFIG register (bank 0)
    if (status != HAL_OK) return status;

    // 5. Enable I2C master interface (to use I2C slave interface)
    uint8_t user_ctrl = 0x20;
    status = ICM_WriteBytes(ICM20948_USER_CTRL, &user_ctrl, 1);
    if (status != HAL_OK) return status;

    // Switch to USER BANK 3
    ICM_SelectBank(ICM20948_USER_BANK_3);

    // Set I2C Master Clock Speed (400kHz)
    uint8_t i2c_mst_ctrl = 0x07;  // I2C_MST_CLK = 7 = 345.6 kHz (closest to 400kHz)
    status = ICM_WriteBytes(0x01, &i2c_mst_ctrl, 1);
    if (status != HAL_OK) return status;

    // 6. Select USER BANK 2 for gyro and accel config
    ICM_SelectBank(ICM20948_USER_BANK_2);

    // 7. Configure gyroscope (±2000 dps, 17Hz BW)
    uint8_t gyro_config_1 = 0x06; // FCHOICE=0, DLPFCFG=6 (17Hz), FS_SEL=3 (±2000dps)
    status = ICM_WriteBytes(ICM20948_GYRO_CONFIG_1, &gyro_config_1, 1);
    if (status != HAL_OK) return status;

    // 8. Configure accelerometer (±16g, 17Hz BW)
    uint8_t accel_config = 0x06; // FCHOICE=1, DLPFCFG=6, FS_SEL=3 (±16g)
    status = ICM_WriteBytes(ICM20948_ACCEL_CONFIG, &accel_config, 1);
    if (status != HAL_OK) return status;

    // Set I2C_SLV0 to write to AK09916 CNTL2 (0x31) to set continuous mode 2 (100Hz)
    uint8_t slv0_addr = 0x0C;  // AK09916 I2C addr (write)
    uint8_t reg = 0x31;
    uint8_t data = 0x08; // Continuous measurement mode 2 (100Hz)

    // Set register to write to (CNTL2)
    status = ICM_WriteBytes(ICM20948_I2C_SLV0_ADDR, &slv0_addr, 1);
    if (status != HAL_OK) return status;
    status = ICM_WriteBytes(ICM20948_I2C_SLV0_REG, &reg, 1);
    if (status != HAL_OK) return status;
    status = ICM_WriteBytes(ICM20948_I2C_SLV0_DO, &data, 1);
    if (status != HAL_OK) return status;

    // Enable I2C_SLV0 for one write
    uint8_t ctrl = 0x81;  // Enable, 1 byte
    status = ICM_WriteBytes(ICM20948_I2C_SLV0_CTRL, &ctrl, 1);
    if (status != HAL_OK) return status;

    HAL_Delay(10);  // Let mag config complete

    // 9. Return to USER BANK 0
    ICM_SelectBank(ICM20948_USER_BANK_0);

    HAL_Delay(50);
    // printf("ICM20948 initialization complete.\r\n");

    return HAL_OK;

}

float ICM_ReadTemperature(void) {
    uint8_t rawData[2];
    float temperatureC = 0.0;

    ICM_SelectBank(ICM20948_USER_BANK_0);  // Ensure correct register bank!

    if (ICM_readBytes(ICM20948_TEMP_OUT_H, rawData, 2) != HAL_OK)
    {
        printf("Failed to read temperature registers.\r\n");
        return temperatureC;
    }

    int16_t temp_raw = ((int16_t)rawData[0] << 8) | rawData[1];
    temperatureC = ((float)temp_raw / 333.87f) + 21.0f;  // per datasheet Page 14

    return temperatureC;
}

uint8_t ICM_ReadAccel(ICM_Axis3D *accel)
{
    uint8_t rawData[6];
    ICM_SelectBank(ICM20948_USER_BANK_0);

    if (ICM_readBytes(ICM20948_ACCEL_XOUT_H, rawData, 6) != HAL_OK)
        return HAL_ERROR;

    accel->x = (int16_t)((rawData[0] << 8) | rawData[1]);
    accel->y = (int16_t)((rawData[2] << 8) | rawData[3]);
    accel->z = (int16_t)((rawData[4] << 8) | rawData[5]);

    return HAL_OK;
}

uint8_t ICM_ReadGyro(ICM_Axis3D *gyro)
{
    uint8_t rawData[6];
    ICM_SelectBank(ICM20948_USER_BANK_0);

    if (ICM_readBytes(ICM20948_GYRO_XOUT_H, rawData, 6) != HAL_OK)
        return HAL_ERROR;

    gyro->x = (int16_t)((rawData[0] << 8) | rawData[1]);
    gyro->y = (int16_t)((rawData[2] << 8) | rawData[3]);
    gyro->z = (int16_t)((rawData[4] << 8) | rawData[5]);

    return HAL_OK;
}

uint8_t ICM_ReadMag(ICM_Axis3D *mag)
{
	HAL_StatusTypeDef status;
    uint8_t mag_raw[6];


    // Configure I2C_SLV0 to auto-read 6 bytes from AK09916 starting at 0x11 (HXL)
    ICM_SelectBank(ICM20948_USER_BANK_3);

    uint8_t slv0_addr = 0x8C; // AK09916 read address (0x0C << 1 | 1)
    uint8_t start_reg = 0x11; // HXL
    uint8_t ctrl = 0x86;      // Enable, read 6 bytes

    status = ICM_WriteBytes(ICM20948_I2C_SLV0_ADDR, &slv0_addr, 1);
    if (status != HAL_OK) return status;
    status = ICM_WriteBytes(ICM20948_I2C_SLV0_REG, &start_reg, 1);
    if (status != HAL_OK) return status;
    status = ICM_WriteBytes(ICM20948_I2C_SLV0_CTRL, &ctrl, 1);
    if (status != HAL_OK) return status;

    HAL_Delay(10);  // Wait for data to populate

    ICM_SelectBank(ICM20948_USER_BANK_0);

    if (ICM_readBytes(ICM20948_EXT_SENS_DATA_00, mag_raw, 6) != HAL_OK)
        return HAL_ERROR;

    mag->x = (int16_t)((mag_raw[1] << 8) | mag_raw[0]);
    mag->y = (int16_t)((mag_raw[3] << 8) | mag_raw[2]);
    mag->z = (int16_t)((mag_raw[5] << 8) | mag_raw[4]);

    return HAL_OK;
}

uint8_t ICM_GetAllRawData(ICM_Axis3D *accel, float * pTemp, ICM_Axis3D *gyro, ICM_Axis3D *mag)
{
    uint8_t rawData[21];
    int16_t temp_raw = 0;

    // 1.41362 ms
    ICM_SelectBank(ICM20948_USER_BANK_0);

    if (ICM_readBytes(ICM20948_ACCEL_XOUT_H, rawData, 21) != HAL_OK)
        return HAL_ERROR;
    //

    
    accel->x = (int16_t)((rawData[0] << 8) | rawData[1]);
    accel->y = (int16_t)((rawData[2] << 8) | rawData[3]);
    accel->z = (int16_t)((rawData[4] << 8) | rawData[5]);

    gyro->x = (int16_t)((rawData[6] << 8) | rawData[7]);
    gyro->y = (int16_t)((rawData[8] << 8) | rawData[9]);
    gyro->z = (int16_t)((rawData[10] << 8) | rawData[11]);

    temp_raw = ((int16_t)rawData[12] << 8) | rawData[13];
    *pTemp = ((float)temp_raw / 333.87f) + 21.0f;  // per datasheet Page 14

    mag->x = (int16_t)((rawData[15] << 8) | rawData[14]);
    mag->y = (int16_t)((rawData[17] << 8) | rawData[16]);
    mag->z = (int16_t)((rawData[19] << 8) | rawData[18]);

    if ((rawData[20] & 0x8) != 0)
    {
      printf("ICM OVERFLOW.\r\n");
      return HAL_ERROR;
    }

    return HAL_OK;
}

void ICM_DumpRegisters(void)
{
    uint8_t val;

    printf("\r\n=== ICM20948 REGISTER DUMP ===\r\n");

    // --- USER BANK 0 ---
    ICM_SelectBank(ICM20948_USER_BANK_0);
    printf("USER BANK 0:\r\n");

    ICM_readBytes(ICM20948_WHO_AM_I_REG, &val, 1);
    printf("WHO_AM_I        (0x00): 0x%02X\r\n", val);

    ICM_readBytes(ICM20948_PWR_MGMT_1, &val, 1);
    printf("PWR_MGMT_1      (0x06): 0x%02X\r\n", val);

    ICM_readBytes(ICM20948_PWR_MGMT_2, &val, 1);
    printf("PWR_MGMT_2      (0x07): 0x%02X\r\n", val);

    ICM_readBytes(0x03, &val, 1);
    printf("USER_CTRL       (0x03): 0x%02X\r\n", val);

    ICM_readBytes(0x05, &val, 1);
    printf("LP_CONFIG       (0x05): 0x%02X\r\n", val);

    ICM_readBytes(ICM20948_TEMP_OUT_H, &val, 1);
    printf("TEMP_OUT_H      (0x39): 0x%02X\r\n", val);
    ICM_readBytes(ICM20948_TEMP_OUT_L, &val, 1);
    printf("TEMP_OUT_L      (0x3A): 0x%02X\r\n", val);

    // --- USER BANK 2 ---
    ICM_SelectBank(ICM20948_USER_BANK_2);
    printf("\r\nUSER BANK 2:\r\n");

    ICM_readBytes(0x01, &val, 1);
    printf("GYRO_CONFIG_1   (0x01): 0x%02X\r\n", val);

    ICM_readBytes(0x14, &val, 1);
    printf("ACCEL_CONFIG    (0x14): 0x%02X\r\n", val);

    // Return to bank 0
    ICM_SelectBank(ICM20948_USER_BANK_0);
    printf("=== END DUMP ===\r\n\r\n");
}

