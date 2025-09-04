/*
 * crosslink.c
 *
 *  Created on: Aug 6, 2024
 *      Author: gvigelet
 */

#include "crosslink.h"
#include "common.h"
#include "main.h"
#include "i2c_master.h"
#include "utils.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#define I2C_ADDRESS 0x40  // Replace with your FPGA's I2C address
#define CMD_INITIATE 0xC0
#define CMD_ISSUE 0x74
#define CMD_ENABLE_SRAM 0xC6
#define CMD_ERASE_SRAM 0x26
#define CMD_READ_STATUS 0x3C
#define CMD_PROGRAM_SRAM 0x46
#define CMD_VERIFY_USERCODE 0xC8
#define CMD_DISABLE 0x26

extern uint8_t txComplete;
extern uint8_t rxComplete;
extern uint8_t i2cError;

unsigned char activation_key[5] = {0xFF, 0xA4, 0xC6, 0xF4, 0x8A};
uint8_t expected_idcode[] = {0x01, 0x2C, 0x00, 0x43};
unsigned char write_buf[4];
unsigned char read_buf[4];

const uint8_t max_attempts = 3;
extern uint8_t bitstream_buffer[];
extern uint32_t bitstream_len;

int fpga_send_activation(I2C_HandleTypeDef *hi2c, uint16_t DevAddress)
{
	// Step 1: Initialize
	if(verbose_on) printf("Step 1: Send Activation Key\r\n");
	if (xi2c_write_bytes(hi2c, DevAddress, activation_key, 5) != HAL_OK) {
		if(verbose_on) printf("failed to send activation key\r\n");
	    return 1;  // Exit if writing activation key fails
	}

	return 0;
}

int fpga_checkid(I2C_HandleTypeDef *hi2c, uint16_t DevAddress)
{
    // Step 2: Check IDCODE (Optional)
    if(verbose_on) printf("Step 2: Check IDCODE (Optional)\r\n");
    write_buf[0] = 0xE0; write_buf[1] = 0x00; write_buf[2] = 0x00; write_buf[3] = 0x00;
    if (xi2c_write_and_read(hi2c, DevAddress, write_buf, 4, read_buf, 4) != HAL_OK) {
        if(verbose_on) printf("failed to send IDCODE Command\r\n");
        return 1;  // Exit if write/read fails
    }

    if(verbose_on) print_hex_buf("IDCODE", read_buf, 4);
	return 0;
}

int fpga_enter_sram_prog_mode(I2C_HandleTypeDef *hi2c, uint16_t DevAddress)
{
    // Step 3: Enable SRAM Programming Mode
    if(verbose_on) printf("Step 3: Enable SRAM Programming Mode\r\n");
    write_buf[0] = 0xC6; write_buf[1] = 0x00; write_buf[2] = 0x00; write_buf[3] = 0x00;
    if (xi2c_write_bytes(hi2c, DevAddress, write_buf, 4) != HAL_OK) {
        if(verbose_on) printf("failed to send SRAM Command\r\n");
        return 1;  // Exit if writing fails
    }
    HAL_Delay(1);
    return 0;
}

int fpga_exit_prog_mode(I2C_HandleTypeDef *hi2c, uint16_t DevAddress)
{

    // Step 11: Exit Programming Mode
    if(verbose_on) printf("Step 10: Exit Programming Mode\r\n");
    write_buf[0] = 0x26; write_buf[1] = 0x00; write_buf[2] = 0x00; write_buf[3] = 0x00;
    if (xi2c_write_bytes(hi2c, DevAddress, write_buf, 4) != HAL_OK) {
        if(verbose_on) printf("failed to send Exit Command\r\n");
        return 1;  // Exit if writing fails
    }

    return 0;
}

uint32_t fpga_read_status(I2C_HandleTypeDef *hi2c, uint16_t DevAddress)
{
    // Step 5: Read Status Register
    if(verbose_on) printf("Read Status Register\r\n");
    write_buf[0] = 0x3C; write_buf[1] = 0x00; write_buf[2] = 0x00; write_buf[3] = 0x00;
    if (xi2c_write_and_read(hi2c, DevAddress, write_buf, 4, read_buf, 4) != HAL_OK) {
        if(verbose_on) printf("failed to send READ Status Command\r\n");
        return 1;  // Exit if write/read fails
    }

    if(verbose_on) print_hex_buf("Erase Status", read_buf, 4);
    return 0;

}

uint32_t fpga_read_usercode(I2C_HandleTypeDef *hi2c, uint16_t DevAddress)
{
    // Step 9: Read USERCODE (Optional)
    if(verbose_on) printf("Step 9: Verify USERCODE (Optional)\r\n");
    write_buf[0] = 0xC0; write_buf[1] = 0x00; write_buf[2] = 0x00; write_buf[3] = 0x00;
    if (xi2c_write_and_read(hi2c, DevAddress, write_buf, 4, read_buf, 4) != HAL_OK) {
        if(verbose_on) printf("failed to send USERCODE Command\r\n");
        return 1;  // Exit if write/read fails
    }

    if(verbose_on) print_hex_buf("User Register", read_buf, 4);
    return 0;
}

int fpga_erase_sram(I2C_HandleTypeDef *hi2c, uint16_t DevAddress)
{
    // Step 4: Erase SRAM
    if(verbose_on) printf("Step 4: Erase SRAM...");
    write_buf[0] = 0x0E; write_buf[1] = 0x00; write_buf[2] = 0x00; write_buf[3] = 0x00;
    if (xi2c_write_bytes(hi2c, DevAddress, write_buf, 4) != HAL_OK) {
        if(verbose_on) printf("\r\nFAILED to send SRAM Erase Command\r\n");
        return 1;  // Exit if writing fails
    }
    if(verbose_on) printf("COMPLETED\r\n");
    return 0;
}

int fpga_program_sram(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, bool rom_bitstream, uint8_t* pData, uint32_t Data_Len)
{
    if(verbose_on) printf("Program SRAM\r\n");
    write_buf[0] = 0x46; write_buf[1] = 0x00; write_buf[2] = 0x00; write_buf[3] = 0x00;
    if (xi2c_write_bytes(hi2c, DevAddress, write_buf, 4)!= HAL_OK) {
        if(verbose_on) printf("failed to send Exit Command\r\n");
        return 1;  // Exit if writing fails
    }

    if(rom_bitstream)
    {
    	write_buf[0] = 0x7A; write_buf[1] = 0x00; write_buf[2] = 0x00; write_buf[3] = 0x00;
    	if(xi2c_write_long(hi2c, DevAddress, write_buf, 4, (uint8_t *)bitstream_buffer, bitstream_len) != HAL_OK)
    	{
    		return 1; // program sram failed
    	}
    }
    else
    {
    	return 1; // ROM Bitstream not enabled
    }

    return 0;
}


int fpga_configure(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	int ret_status = 0;
	uint8_t attempt = 0;
	_Bool idcode_match = false;

    // Set GPIO HIGH
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
    HAL_Delay(250);

    // Set GPIO LOW
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
    HAL_Delay(250);

    while (attempt < max_attempts && !idcode_match) {
        attempt++;
		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
		HAL_Delay(100);

		// Set GPIO LOW
		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
		HAL_Delay(1000);

		// Activation Key
        uint8_t activation_key[] = {0xFF, 0xA4, 0xC6, 0xF4, 0x8A};
        if(verbose_on) printf("Sending Activation Key, Attempt %d...\r\n", attempt);
        HAL_Delay(5);
	    int ret = xi2c_write_bytes(hi2c, DevAddress, activation_key, 5);
        if(ret != HAL_OK){
            printf("ERROR: Failed to send Activation Key on attempt %d\r\n", attempt);
            continue;
        }
        printf("xi2c_write_bytes returned: %d\r\n", ret);
        HAL_Delay(15); // remove this once you can guarantee that i2c_write_bytes works correctly
        HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
		HAL_Delay(10);
        if(verbose_on) printf("Activation Key sent.\r\n");

		// IDCODE
        if(verbose_on) printf("Checking IDCODE...\r\n");
		memset(read_buf, 0, 4);
		memcpy(write_buf, (uint8_t[]){0xE0,0x00,0x00,0x00}, 4);
		xi2c_write_and_read(hi2c, DevAddress, write_buf, 4, read_buf, 4);
		if(verbose_on) print_hex_buf("IDCODE", read_buf, 4);

	    // Check if IDCODE matches expected
	    if (memcmp(read_buf, expected_idcode, 4) == 0) {
	        idcode_match = true;
	        break;
	    }
    }

    if (!idcode_match) {
        printf("ERROR: Failed to match IDCODE after %d attempts.\r\n", max_attempts);
        // Optionally return or handle error
        return HAL_ERROR;
    } else {
        printf("IDCODE matched successfully.\r\n");
        // Proceed with the next steps
    }

    // Enable SRAM
    memcpy(write_buf, (uint8_t[]){0xC6,0x00,0x00,0x00}, 4);
    xi2c_write_bytes(hi2c, DevAddress, write_buf, 4);
    HAL_Delay(1);

    // Erase SRAM
    memcpy(write_buf, (uint8_t[]){0x0E,0x00,0x00,0x00}, 4);
    xi2c_write_bytes(hi2c, DevAddress, write_buf, 4);
    HAL_Delay(5000);

    // Read Status
    memset(read_buf, 0, 4);
    memcpy(write_buf, (uint8_t[]){0x3C,0x00,0x00,0x00}, 4);
    xi2c_write_and_read(hi2c, DevAddress, write_buf, 4, read_buf, 4);
    if(verbose_on) print_hex_buf("Erase Status", read_buf, 4);

    // Program Command
    memcpy(write_buf, (uint8_t[]){0x46,0x00,0x00,0x00}, 4);
    xi2c_write_bytes(hi2c, DevAddress, write_buf, 4);
    HAL_Delay(1);

    // Send Bitstream
    memcpy(write_buf, (uint8_t[]){0x7A,0x00,0x00,0x00}, 4);
    xi2c_write_long(hi2c, DevAddress, write_buf, 4, (uint8_t*)0x081A0000, (size_t)163489);
    HAL_Delay(1);

    // USERCODE (optional)
    memset(read_buf, 0, 4);
    memcpy(write_buf, (uint8_t[]){0xC0,0x00,0x00,0x00}, 4);
    xi2c_write_and_read(hi2c, DevAddress, write_buf, 4, read_buf, 4);
    if(verbose_on) print_hex_buf("User Register", read_buf, 4);

    // Final Status
    memset(read_buf, 0, 4);
    memcpy(write_buf, (uint8_t[]){0x3C,0x00,0x00,0x00}, 4);
    xi2c_write_and_read(hi2c, DevAddress, write_buf, 4, read_buf, 4);
    if(verbose_on) print_hex_buf("Program Status", read_buf, 4);

    if(read_buf[2] != 0x0F) ret_status = 1;

    // Exit Program Mode
    memcpy(write_buf, (uint8_t[]){0x26,0x00,0x00,0x00}, 4);
    xi2c_write_bytes(hi2c, DevAddress, write_buf, 4);

    if(verbose_on) printf("FPGA configuration complete.\r\n");
    return ret_status;
}
