/*
 * crosslink.c
 *
 *  Created on: Aug 6, 2024
 *      Author: gvigelet
 */

#include "crosslink.h"
#include "common.h"
#include "main.h"
#include "utils.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#define I2C_ADDRESS 0x40  // Replace with your FPGA's I2C address
#define BITSTREAM_CHUNK_SIZE 8192

// SRAM Programming Commands
#define CMD_INITIATE 0xC0
#define CMD_ISSUE 0x74
#define CMD_ENABLE_SRAM 0xC6
#define CMD_ERASE_SRAM 0x26
#define CMD_READ_STATUS 0x3C
#define CMD_PROGRAM_SRAM 0x46
#define CMD_VERIFY_USERCODE 0xC8
#define CMD_DISABLE 0x26

// NVCM Programming Commands
#define CMD_NVCM_ENABLE 0x74
#define CMD_NVCM_ERASE 0x0E
#define CMD_NVCM_INIT_ADDR 0x46
#define CMD_NVCM_PROG_INC 0x70
#define CMD_NVCM_READ_INC 0x73
#define CMD_NVCM_PROG_USERCODE 0xC2
#define CMD_NVCM_PROG_FEATURE 0xE4
#define CMD_NVCM_PROG_DONE 0x5E
#define NVCM_ISC_DISABLE 0x26

// NVCM Timing Parameters (in milliseconds)
#define NVCM_ERASE_TIME 200
#define NVCM_PROG_TIME 1
#define NVCM_VERIFY_TIME 1


volatile uint8_t txComplete = 0;
volatile uint8_t rxComplete = 0;
volatile uint8_t i2cError = 0;

uint8_t trim_params[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char activation_key[5] = {0xFF, 0xA4, 0xC6, 0xF4, 0x8A};
uint8_t expected_idcode[] = {0x01, 0x2C, 0x00, 0x43};
unsigned char write_buf[4];
unsigned char read_buf[4];

const uint8_t max_attempts = 3;
extern uint8_t bitstream_buffer[];
extern uint32_t bitstream_len;

// ============================================================================
// General Programming Functions
// ============================================================================

static int xi2c_write_bytes(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *data, uint16_t length) {
    return HAL_I2C_Master_Transmit(hi2c, DevAddress << 1, data, length, HAL_MAX_DELAY);
}

static int xi2c_write_and_read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *wbuf, uint16_t wlen, uint8_t *rbuf, uint16_t rlen) {
    txComplete = 0;
    rxComplete = 0;
    i2cError = 0;

    if (HAL_I2C_Master_Seq_Transmit_IT(hi2c, DevAddress << 1, wbuf, wlen, I2C_FIRST_FRAME) != HAL_OK)
        return -1;

    // Wait for the transmission to complete
    while (!txComplete && !i2cError) {}

    if (i2cError)
    {
        return HAL_ERROR;
    }


    if (HAL_I2C_Master_Seq_Receive_IT(hi2c, DevAddress << 1, rbuf, rlen, I2C_LAST_FRAME) != HAL_OK)
        return -1;

    // Wait for the reception to complete
    while (!rxComplete && !i2cError) {}

    if (i2cError)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

static HAL_StatusTypeDef xi2c_write_long(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *cmd, int cmd_len, uint8_t *data, size_t data_len) {
	HAL_StatusTypeDef ret;
    size_t offset = 0;
    uint32_t frame_flag;
    size_t total_len = data_len+cmd_len;
    int num_chunks = (total_len + BITSTREAM_CHUNK_SIZE - 1) / BITSTREAM_CHUNK_SIZE;  // Calculate number of chunks
    uint8_t *pData;
	uint16_t datalen;

	memset(bitstream_buffer, 0, MAX_BITSTREAM_SIZE);
    memcpy(bitstream_buffer, cmd, cmd_len);  // copy the long write command in
    memcpy(bitstream_buffer + cmd_len, data, data_len);

    for (int i = 0; i < num_chunks; i++) {
        size_t current_chunk_size = (total_len - offset > BITSTREAM_CHUNK_SIZE)
                                     ? BITSTREAM_CHUNK_SIZE
                                     : (total_len - offset);

        // Determine frame flags
        if (i == 0 && num_chunks == 1) {
            frame_flag = I2C_FIRST_AND_LAST_FRAME;
        } else if (i == 0) {
            frame_flag = I2C_FIRST_AND_NEXT_FRAME;
        } else if (i == num_chunks - 1) {
            frame_flag = I2C_LAST_FRAME;
        } else {
            frame_flag = I2C_NEXT_FRAME;
        }

        // Check if the I2C peripheral is ready
        while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
            //if(verbose_on) printf("I2C busy, waiting...\r\n");
        	delay_us(1000);  // Add a small delay to avoid busy looping
        }

        pData = (uint8_t*)&bitstream_buffer[offset];
        datalen = (uint16_t)current_chunk_size;

        // Reset completion flags
        txComplete = 0;
        i2cError = 0;

        // Transmit the current chunk
        ret = HAL_I2C_Master_Seq_Transmit_IT(hi2c, DevAddress << 1, pData, datalen, frame_flag);
        if (ret != HAL_OK) {
        	printf("++++> i2c_write_long HAL TX HAL_StatusTypeDef: 0%04X I2C_ERROR: 0%04lX\r\n", ret, hi2c1.ErrorCode);
            return ret; // Return if any transmission fails
        }

        // Wait for the transmission to complete
        while (!txComplete && !i2cError) {}

        if (i2cError)
        {
            return HAL_ERROR;
        }

        offset += current_chunk_size;
    }

    return HAL_OK;
}

// ============================================================================
// SRAM Programming Functions
// ============================================================================

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
    delay_us(1000);
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

// ============================================================================
// NVCM Programming Functions
// ============================================================================

int fpga_nvcm_check_busy(I2C_HandleTypeDef *hi2c, uint16_t DevAddress)
{
    uint8_t status_buf[4];
    uint32_t timeout = 1000; // 1 second timeout

    while (timeout > 0) {
        write_buf[0] = 0x3C; write_buf[1] = 0x00; write_buf[2] = 0x00; write_buf[3] = 0x00;
        if (xi2c_write_and_read(hi2c, DevAddress, write_buf, 4, status_buf, 4) != HAL_OK) {
            if(verbose_on) printf("Failed to read status register\r\n");
            return HAL_ERROR;
        }

        // Check bit 12 (busy bit) - status_buf[1] bit 4
        if ((status_buf[1] & 0x10) == 0) {
            return HAL_OK; // Not busy
        }

        delay_us(1000);
        timeout--;
    }

    if(verbose_on) printf("Timeout waiting for device to be ready\r\n");
    return HAL_TIMEOUT;
}

int fpga_nvcm_program_trim(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *trim_data)
{
    if(verbose_on) printf("NVCM: Programming Trim Parameters\r\n");

    // LSC_PROG_CNTRL0 command (0x22) with trim data
    write_buf[0] = 0x22;
    if (trim_data != NULL) {
        memcpy(&write_buf[1], trim_data, 3);
    } else {
        write_buf[1] = 0x00; write_buf[2] = 0x00; write_buf[3] = 0x00;
    }

    if (xi2c_write_bytes(hi2c, DevAddress, write_buf, 4) != HAL_OK) {
        if(verbose_on) printf("Failed to program trim parameters\r\n");
        return HAL_ERROR;
    }

    delay_us(1000 * NVCM_PROG_TIME);
    return HAL_OK;
}

int fpga_nvcm_enter_prog_mode(I2C_HandleTypeDef *hi2c, uint16_t DevAddress)
{
    if(verbose_on) printf("NVCM: Enter ISC Access Mode\r\n");

    // ISC ENABLE command (0xC6) with operand 0x08 for NVCM mode
    write_buf[0] = 0xC6; write_buf[1] = 0x08; write_buf[2] = 0x00; write_buf[3] = 0x00;
    if (xi2c_write_bytes(hi2c, DevAddress, write_buf, 4) != HAL_OK) {
        if(verbose_on) printf("Failed to enter NVCM programming mode\r\n");
        return HAL_ERROR;
    }

    delay_us(1000);
    return HAL_OK;
}

int fpga_nvcm_erase(I2C_HandleTypeDef *hi2c, uint16_t DevAddress)
{
    if(verbose_on) printf("NVCM: Erasing NVCM array...");

    // ISC ERASE command (0x0E) with operand 0x04 for NVCM erase
    write_buf[0] = 0x0E; write_buf[1] = 0x04; write_buf[2] = 0x00; write_buf[3] = 0x00;
    if (xi2c_write_bytes(hi2c, DevAddress, write_buf, 4) != HAL_OK) {
        if(verbose_on) printf("\r\nFailed to send NVCM erase command\r\n");
        return HAL_ERROR;
    }

    delay_us(1000 * NVCM_ERASE_TIME);

    // Check busy status
    if (fpga_nvcm_check_busy(hi2c, DevAddress) != HAL_OK) {
        if(verbose_on) printf("\r\nNVCM erase timeout\r\n");
        return HAL_TIMEOUT;
    }

    if(verbose_on) printf("COMPLETED\r\n");
    return HAL_OK;
}

int fpga_nvcm_init_address(I2C_HandleTypeDef *hi2c, uint16_t DevAddress)
{
    if(verbose_on) printf("NVCM: Initialize address\r\n");

    // LSC_INIT_ADDRESS command (0x46)
    write_buf[0] = 0x46; write_buf[1] = 0x00; write_buf[2] = 0x00; write_buf[3] = 0x00;
    if (xi2c_write_bytes(hi2c, DevAddress, write_buf, 4) != HAL_OK) {
        if(verbose_on) printf("Failed to initialize address\r\n");
        return HAL_ERROR;
    }

    delay_us(1000);
    return HAL_OK;
}

int fpga_nvcm_program_array(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t* bitstream, uint32_t bitstream_len)
{
    if(verbose_on) printf("NVCM: Programming array (%lu bytes)\r\n", bitstream_len);

    // LSC_PROG_INCR_NV command (0x70) followed by bitstream data
    write_buf[0] = 0x70; write_buf[1] = 0x00; write_buf[2] = 0x00; write_buf[3] = 0x00;

    if (xi2c_write_long(hi2c, DevAddress, write_buf, 4, bitstream, bitstream_len) != HAL_OK) {
        if(verbose_on) printf("Failed to program NVCM array\r\n");
        return HAL_ERROR;
    }

    delay_us(1000 * NVCM_PROG_TIME);

    // Check busy status
    if (fpga_nvcm_check_busy(hi2c, DevAddress) != HAL_OK) {
        if(verbose_on) printf("NVCM programming timeout\r\n");
        return HAL_TIMEOUT;
    }

    if(verbose_on) printf("NVCM array programming complete\r\n");
    return HAL_OK;
}

int fpga_nvcm_verify_array(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t* bitstream, uint32_t bitstream_len)
{
    if(verbose_on) printf("NVCM: Verifying array (%lu bytes)\r\n", bitstream_len);

    // Initialize address for reading
    if (fpga_nvcm_init_address(hi2c, DevAddress) != HAL_OK) {
        return HAL_ERROR;
    }

    // LSC_READ_INCR_NV command (0x73)
    write_buf[0] = 0x73; write_buf[1] = 0x00; write_buf[2] = 0x00; write_buf[3] = 0x00;

    uint32_t verify_errors = 0;
    uint32_t bytes_to_verify = bitstream_len;
    uint32_t offset = 0;
    uint8_t verify_buf[16];

    // Verify in chunks
    while (bytes_to_verify > 0) {
        uint32_t chunk_size = (bytes_to_verify > 16) ? 16 : bytes_to_verify;

        if (xi2c_write_and_read(hi2c, DevAddress, write_buf, 4, verify_buf, chunk_size) != HAL_OK) {
            if(verbose_on) printf("Failed to read NVCM data for verification\r\n");
            return HAL_ERROR;
        }

        // Compare with original bitstream
        if (memcmp(&bitstream[offset], verify_buf, chunk_size) != 0) {
            verify_errors++;
            if(verbose_on) printf("Verification error at offset %lu\r\n", offset);
        }

        offset += chunk_size;
        bytes_to_verify -= chunk_size;
    }

    if (verify_errors > 0) {
        if(verbose_on) printf("NVCM verification failed with %lu errors\r\n", verify_errors);
        return HAL_ERROR;
    }

    if(verbose_on) printf("NVCM array verification successful\r\n");
    return HAL_OK;
}

int fpga_nvcm_program_usercode(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint32_t usercode)
{
    if(verbose_on) printf("NVCM: Programming USERCODE (0x%08lX)\r\n", usercode);

    // ISC_PROGRAM_USERCODE command (0xC2)
    write_buf[0] = 0xC2;
    write_buf[1] = (usercode >> 24) & 0xFF;
    write_buf[2] = (usercode >> 16) & 0xFF;
    write_buf[3] = (usercode >> 8) & 0xFF;

    uint8_t usercode_data[1];
    usercode_data[0] = usercode & 0xFF;

    if (xi2c_write_long(hi2c, DevAddress, write_buf, 4, usercode_data, 1) != HAL_OK) {
        if(verbose_on) printf("Failed to program USERCODE\r\n");
        return HAL_ERROR;
    }

    delay_us(1000 * NVCM_PROG_TIME);

    if (fpga_nvcm_check_busy(hi2c, DevAddress) != HAL_OK) {
        if(verbose_on) printf("USERCODE programming timeout\r\n");
        return HAL_TIMEOUT;
    }

    if(verbose_on) printf("USERCODE programming complete\r\n");
    return HAL_OK;
}

int fpga_nvcm_verify_usercode(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint32_t expected_usercode)
{
    if(verbose_on) printf("NVCM: Verifying USERCODE\r\n");

    // USERCODE command (0xC0)
    write_buf[0] = 0xC0; write_buf[1] = 0x00; write_buf[2] = 0x00; write_buf[3] = 0x00;

    uint8_t usercode_buf[4];
    if (xi2c_write_and_read(hi2c, DevAddress, write_buf, 4, usercode_buf, 4) != HAL_OK) {
        if(verbose_on) printf("Failed to read USERCODE\r\n");
        return HAL_ERROR;
    }

    uint32_t read_usercode = (usercode_buf[0] << 24) | (usercode_buf[1] << 16) |
                             (usercode_buf[2] << 8) | usercode_buf[3];

    if(verbose_on) printf("Read USERCODE: 0x%08lX, Expected: 0x%08lX\r\n", read_usercode, expected_usercode);

    if (read_usercode != expected_usercode) {
        if(verbose_on) printf("USERCODE verification failed\r\n");
        return HAL_ERROR;
    }

    if(verbose_on) printf("USERCODE verification successful\r\n");
    return HAL_OK;
}

int fpga_nvcm_program_feature_bits(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t feature_bits)
{
    if(verbose_on) printf("NVCM: Programming Feature Bits (0x%04X)\r\n", feature_bits);

    // ISC_PROGRAM_FEATURE command (0xE4)
    write_buf[0] = 0xE4;
    write_buf[1] = (feature_bits >> 8) & 0xFF;
    write_buf[2] = feature_bits & 0xFF;
    write_buf[3] = 0x00;

    if (xi2c_write_bytes(hi2c, DevAddress, write_buf, 4) != HAL_OK) {
        if(verbose_on) printf("Failed to program feature bits\r\n");
        return HAL_ERROR;
    }

    delay_us(1000 * NVCM_PROG_TIME);

    if (fpga_nvcm_check_busy(hi2c, DevAddress) != HAL_OK) {
        if(verbose_on) printf("Feature bits programming timeout\r\n");
        return HAL_TIMEOUT;
    }

    if(verbose_on) printf("Feature bits programming complete\r\n");
    return HAL_OK;
}

int fpga_nvcm_verify_feature_bits(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t expected_bits)
{
    if(verbose_on) printf("NVCM: Verifying Feature Bits\r\n");

    // LSC_READ_FEATURE command (0xE7)
    write_buf[0] = 0xE7; write_buf[1] = 0x00; write_buf[2] = 0x00; write_buf[3] = 0x00;

    uint8_t feature_buf[4];
    if (xi2c_write_and_read(hi2c, DevAddress, write_buf, 4, feature_buf, 4) != HAL_OK) {
        if(verbose_on) printf("Failed to read feature bits\r\n");
        return HAL_ERROR;
    }

    uint16_t read_features = (feature_buf[0] << 8) | feature_buf[1];

    if(verbose_on) printf("Read Feature Bits: 0x%04X, Expected: 0x%04X\r\n", read_features, expected_bits);

    if (read_features != expected_bits) {
        if(verbose_on) printf("Feature bits verification failed\r\n");
        return HAL_ERROR;
    }

    if(verbose_on) printf("Feature bits verification successful\r\n");
    return HAL_OK;
}

int fpga_nvcm_program_done_bit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress)
{
    if(verbose_on) printf("NVCM: Programming DONE bit\r\n");

    // ISC_PROGRAM_DONE command (0x5E)
    write_buf[0] = 0x5E; write_buf[1] = 0x00; write_buf[2] = 0x00; write_buf[3] = 0x00;

    if (xi2c_write_bytes(hi2c, DevAddress, write_buf, 4) != HAL_OK) {
        if(verbose_on) printf("Failed to program DONE bit\r\n");
        return HAL_ERROR;
    }

    delay_us(1000 * NVCM_PROG_TIME);

    if (fpga_nvcm_check_busy(hi2c, DevAddress) != HAL_OK) {
        if(verbose_on) printf("DONE bit programming timeout\r\n");
        return HAL_TIMEOUT;
    }

    if(verbose_on) printf("DONE bit programming complete\r\n");
    return HAL_OK;
}

int fpga_nvcm_verify_done_bit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress)
{
    if(verbose_on) printf("NVCM: Verifying DONE bit\r\n");

    // Read status register
    write_buf[0] = 0x3C; write_buf[1] = 0x00; write_buf[2] = 0x00; write_buf[3] = 0x00;

    uint8_t status_buf[4];
    if (xi2c_write_and_read(hi2c, DevAddress, write_buf, 4, status_buf, 4) != HAL_OK) {
        if(verbose_on) printf("Failed to read status register\r\n");
        return HAL_ERROR;
    }

    // Check bit 8 (DONE bit) - status_buf[1] bit 0
    if ((status_buf[1] & 0x01) == 0) {
        if(verbose_on) printf("DONE bit verification failed\r\n");
        return HAL_ERROR;
    }

    if(verbose_on) printf("DONE bit verification successful\r\n");
    return HAL_OK;
}

int fpga_verify_idcode(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	uint8_t attempt = 0;
	_Bool idcode_match = false;

	if(verbose_on) printf("Starting FPGA configuration...\r\n");

    // Set GPIO HIGH
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
    delay_us(1000);

    // Set GPIO LOW
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
    delay_us(1000);

    while (attempt < max_attempts && !idcode_match) {
        attempt++;
		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
		delay_us(500);

		// Set GPIO LOW
		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
		delay_us(500);

		// Activation Key
		uint8_t activation_key[] = {0xFF, 0xA4, 0xC6, 0xF4, 0x8A};
		xi2c_write_bytes(hi2c, DevAddress, activation_key, 5);
		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
		delay_us(5000);

		// IDCODE
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
    }

    if(verbose_on) printf("Verified IDCODE successful\r\n");
    return HAL_OK;
}


int fpga_nvcm_exit_prog_mode(I2C_HandleTypeDef *hi2c, uint16_t DevAddress)
{
    if(verbose_on) printf("NVCM: Exit Prog Mode\r\n");

    // ISC DISABLE command (0x26)
    write_buf[0] = 0x26; write_buf[1] = 0x00; write_buf[2] = 0x00; write_buf[3] = 0x00;

    if (xi2c_write_bytes(hi2c, DevAddress, write_buf, 4) != HAL_OK) {
        if(verbose_on) printf("Failed to Exit NVCM programming mode\r\n");
        return HAL_ERROR;
    }

    delay_us(250);

    // NVCM_NOOP (0xFF)
    write_buf[0] = 0xFF; write_buf[1] = 0x00; write_buf[2] = 0x00; write_buf[3] = 0x00;

    if (xi2c_write_bytes(hi2c, DevAddress, write_buf, 4) != HAL_OK) {
        if(verbose_on) printf("Failed to Exit NVCM programming mode\r\n");
        return HAL_ERROR;
    }

    delay_us(1000);

    if(verbose_on) printf("DONE Exit NVCM programming mode successful\r\n");
    return HAL_OK;
}

// ============================================================================
// FPGA COnfiguration Functions
// ============================================================================

int fpga_configure(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	int ret_status = 0;
	uint8_t attempt = 0;
	_Bool idcode_match = false;

	if(verbose_on) printf("Starting FPGA configuration...\r\n");

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
		xi2c_write_bytes(hi2c, DevAddress, activation_key, 5);
		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
		HAL_Delay(10);

		// IDCODE
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

int fpga_nvcm_configure(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, const uint8_t *config_data, uint32_t data_size, uint32_t usercode, uint8_t feature_bits)
{
    int ret;

    if (!config_data || data_size == 0) {
        return -1;
    }

    printf("Starting NVCM programming sequence...\n");

    // Step 1: Verify IDCODE
    printf("Step 1: Verifying IDCODE...\n");
    ret = fpga_verify_idcode(hi2c, DevAddress, GPIOx, GPIO_Pin);
    if (ret != 0) {
        printf("ERROR: IDCODE verification failed\n");
        return ret;
    }

    // Step 2: Program Trim Parameters
    printf("Step 2: Programming trim parameters...\n");
    ret = fpga_nvcm_program_trim(hi2c, DevAddress, trim_params);
    if (ret != 0) {
        printf("ERROR: Trim programming failed\n");
        return ret;
    }

    // Step 3: Enter ISC Access Mode
    printf("Step 3: Entering ISC access mode...\n");
    ret = fpga_nvcm_enter_prog_mode(hi2c, DevAddress);
    if (ret != 0) {
        printf("ERROR: Failed to enter programming mode\n");
        return ret;
    }

    // Step 4: Erase NVCM Array
    printf("Step 4: Erasing NVCM array...\n");
    ret = fpga_nvcm_erase(hi2c, DevAddress);
    if (ret != 0) {
        printf("ERROR: NVCM erase failed\n");
        goto exit_prog_mode;
    }

    // Step 5: Initialize Address
    printf("Step 5: Initializing address...\n");
    ret = fpga_nvcm_init_address(hi2c, DevAddress);
    if (ret != 0) {
        printf("ERROR: Address initialization failed\n");
        goto exit_prog_mode;
    }

    // Step 6: Program NVCM Array
    printf("Step 6: Programming NVCM array (%lu bytes)...\n", data_size);
    ret = fpga_nvcm_program_array(hi2c, DevAddress, (uint8_t*)config_data, data_size);
    if (ret != 0) {
        printf("ERROR: NVCM array programming failed\n");
        goto exit_prog_mode;
    }

    // Step 7: Verify NVCM Array
    printf("Step 7: Verifying NVCM array...\n");
    ret = fpga_nvcm_verify_array(hi2c, DevAddress, (uint8_t*)config_data, data_size);
    if (ret != 0) {
        printf("ERROR: NVCM array verification failed\n");
        goto exit_prog_mode;
    }

    // Step 8: Program Usercode
    printf("Step 8: Programming usercode (0x%08lX)...\n", usercode);
    ret = fpga_nvcm_program_usercode(hi2c, DevAddress, usercode);
    if (ret != 0) {
        printf("ERROR: Usercode programming failed\n");
        goto exit_prog_mode;
    }

    // Step 9: Verify Usercode
    printf("Step 9: Verifying usercode...\n");
    ret = fpga_nvcm_verify_usercode(hi2c, DevAddress, usercode);
    if (ret != 0) {
        printf("ERROR: Usercode verification failed\n");
        goto exit_prog_mode;
    }

    // Step 10: Program Feature Bits
    printf("Step 10: Programming feature bits (0x%02X)...\n", feature_bits);
    ret = fpga_nvcm_program_feature_bits(hi2c, DevAddress, feature_bits);
    if (ret != 0) {
        printf("ERROR: Feature bits programming failed\n");
        goto exit_prog_mode;
    }

    // Step 11: Verify Feature Bits
    printf("Step 11: Verifying feature bits...\n");
    ret = fpga_nvcm_verify_feature_bits(hi2c, DevAddress, feature_bits);
    if (ret != 0) {
        printf("ERROR: Feature bits verification failed\n");
        goto exit_prog_mode;
    }

    // Step 12: Program Done Bit
    printf("Step 12: Programming done bit...\n");
    ret = fpga_nvcm_program_done_bit(hi2c, DevAddress);
    if (ret != 0) {
        printf("ERROR: Done bit programming failed\n");
        goto exit_prog_mode;
    }

    // Step 13: Verify Done Bit
    printf("Step 13: Verifying done bit...\n");
    ret = fpga_nvcm_verify_done_bit(hi2c, DevAddress);
    if (ret != 0) {
        printf("ERROR: Done bit verification failed\n");
        goto exit_prog_mode;
    }

exit_prog_mode:
    // Step 14: Exit ISC Access Mode
    printf("Step 14: Exiting ISC access mode...\n");
    fpga_nvcm_exit_prog_mode(hi2c, DevAddress);

    if (ret == 0) {
        printf("NVCM programming completed successfully!\n");
    } else {
        printf("NVCM programming failed with error code: %d\n", ret);
    }

    return ret;
}

// ============================================================================
// Callback Functions
// ============================================================================

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    txComplete = 1;
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    rxComplete = 1;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    i2cError = 1;
}
