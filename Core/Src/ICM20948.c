/*
 * ICM20948.c
 *
 *  Created on: Apr 20, 2025
 *      Author: GeorgeVigelette
 */

#include "main.h"
#include "ICM20948.h"

#include <string.h>
#include <stdio.h>

static HAL_StatusTypeDef ICM_readBytes(uint8_t reg, uint8_t *pData, uint16_t size)
{
	HAL_StatusTypeDef result = HAL_OK;

    // Send the register address to read from
    result = HAL_I2C_Master_Transmit(&ICM_I2C, ICM20948_ADDR << 1, &reg, 1, 100);
    if (result != HAL_OK)
    {
        return result;  // Return if there's an error transmitting the register address
    }

    // Read bytes
    result = HAL_I2C_Master_Receive(&ICM_I2C, ICM20948_ADDR << 1, pData, size, 50*size);
    return result;
}

static HAL_StatusTypeDef ICM_WriteBytes(uint8_t reg, uint8_t *pData, uint16_t size)
{
    HAL_StatusTypeDef result = HAL_OK;
    uint8_t buffer[16];

    if (size > sizeof(buffer) - 1) return HAL_ERROR; // prevent buffer overflow

    buffer[0] = reg;
    memcpy(&buffer[1], pData, size);

    result = HAL_I2C_Master_Transmit(&ICM_I2C, ICM20948_ADDR << 1, buffer, size + 1, 100);
    return result;
}


static void ICM_SelectBank(uint8_t bank)
{
    uint8_t val = bank;
    HAL_I2C_Mem_Write(&ICM_I2C, ICM20948_ADDR << 1, ICM20948_REG_BANK_SEL, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
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

    // 5. Disable I2C Master (to use I2C slave interface)
    uint8_t user_ctrl = 0x00;
    status = ICM_WriteBytes(ICM20948_USER_CTRL, &user_ctrl, 1);
    if (status != HAL_OK) return status;

    // 6. Select USER BANK 2 for gyro and accel config
    ICM_SelectBank(ICM20948_USER_BANK_2);

    // 7. Configure gyroscope (±2000 dps, 17Hz BW)
    uint8_t gyro_config_1 = 0x06; // FCHOICE=0, DLPFCFG=6 (17Hz), FS_SEL=3 (±2000dps)
    status = ICM_WriteBytes(0x01, &gyro_config_1, 1);
    if (status != HAL_OK) return status;

    // 8. Configure accelerometer (±16g, 17Hz BW)
    uint8_t accel_config = 0x06; // FCHOICE=1, DLPFCFG=6, FS_SEL=3 (±16g)
    status = ICM_WriteBytes(0x14, &accel_config, 1);
    if (status != HAL_OK) return status;

    // 9. Return to USER BANK 0
    ICM_SelectBank(ICM20948_USER_BANK_0);

    HAL_Delay(50);
    printf("ICM20948 initialization complete.\r\n");

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

