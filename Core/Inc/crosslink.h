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

int fpga_send_activation(I2C_HandleTypeDef *hi2c, uint16_t DevAddress);
int fpga_checkid(I2C_HandleTypeDef *hi2c, uint16_t DevAddress);
int fpga_enter_sram_prog_mode(I2C_HandleTypeDef *hi2c, uint16_t DevAddress);
int fpga_exit_prog_mode(I2C_HandleTypeDef *hi2c, uint16_t DevAddress);
int fpga_erase_sram(I2C_HandleTypeDef *hi2c, uint16_t DevAddress);
uint32_t fpga_read_status(I2C_HandleTypeDef *hi2c, uint16_t DevAddress);
uint32_t fpga_read_usercode(I2C_HandleTypeDef *hi2c, uint16_t DevAddress);
int fpga_program_sram(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, bool rom_bitstream, uint8_t* pData, uint32_t Data_Len);
int fpga_configure(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

#endif /* INC_CROSSLINK_H_ */
