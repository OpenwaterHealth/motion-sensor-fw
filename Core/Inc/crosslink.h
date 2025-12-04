/*
 * crosslink.h
 *
 *  Created on: Aug 6, 2024
 *      Author: gvigelet
 */

#ifndef INC_CROSSLINK_H_
#define INC_CROSSLINK_H_
#include "main.h"
#include <stdbool.h>

// SRAM Configuration Functions
int fpga_send_activation(I2C_HandleTypeDef *hi2c, uint16_t DevAddress);
int fpga_checkid(I2C_HandleTypeDef *hi2c, uint16_t DevAddress);
int fpga_enter_sram_prog_mode(I2C_HandleTypeDef *hi2c, uint16_t DevAddress);
int fpga_exit_prog_mode(I2C_HandleTypeDef *hi2c, uint16_t DevAddress);
int fpga_erase_sram(I2C_HandleTypeDef *hi2c, uint16_t DevAddress);
uint32_t fpga_read_status(I2C_HandleTypeDef *hi2c, uint16_t DevAddress);
uint32_t fpga_read_usercode(I2C_HandleTypeDef *hi2c, uint16_t DevAddress);
int fpga_program_sram(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, bool rom_bitstream, uint8_t* pData, uint32_t Data_Len);
int fpga_configure(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
int fpga_verify_idcode(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

// NVCM Programming Functions
int fpga_nvcm_program_trim(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *trim_data);
int fpga_nvcm_enter_prog_mode(I2C_HandleTypeDef *hi2c, uint16_t DevAddress);
int fpga_nvcm_erase(I2C_HandleTypeDef *hi2c, uint16_t DevAddress);
int fpga_nvcm_init_address(I2C_HandleTypeDef *hi2c, uint16_t DevAddress);
int fpga_nvcm_program_array(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t* bitstream, uint32_t bitstream_len);
int fpga_nvcm_verify_array(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t* bitstream, uint32_t bitstream_len);
int fpga_nvcm_program_usercode(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint32_t usercode);
int fpga_nvcm_verify_usercode(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint32_t expected_usercode);
int fpga_nvcm_program_feature_bits(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t feature_bits);
int fpga_nvcm_verify_feature_bits(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t expected_bits);
int fpga_nvcm_program_done_bit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress);
int fpga_nvcm_verify_done_bit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress);
int fpga_nvcm_check_busy(I2C_HandleTypeDef *hi2c, uint16_t DevAddress);
int fpga_nvcm_exit_prog_mode(I2C_HandleTypeDef *hi2c, uint16_t DevAddress);
int fpga_nvcm_configure(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, const uint8_t *config_data, uint32_t data_size, uint32_t usercode, uint16_t feature_bits);

#endif /* INC_CROSSLINK_H_ */
