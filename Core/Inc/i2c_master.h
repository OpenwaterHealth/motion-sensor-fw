/*
 * i2c_master.h
 *
 *  Created on: Mar 31, 2024
 *      Author: gvigelet
 */

#ifndef INC_I2C_MASTER_H_
#define INC_I2C_MASTER_H_

#include "main.h"  // This should contain your HAL includes and other basic includes.
#include <stdio.h>
#include <stdbool.h>


uint8_t I2C_scan(I2C_HandleTypeDef * pI2c, uint8_t* addr_list, size_t list_size, bool display) ;
uint8_t send_buffer_to_slave(I2C_HandleTypeDef * pI2c, uint8_t slave_addr, uint8_t* pBuffer, uint16_t buf_len);
uint8_t read_status_register_of_slave(I2C_HandleTypeDef * pI2c, uint8_t slave_addr, uint8_t* pBuffer, uint16_t max_len);
uint8_t read_data_register_of_slave(I2C_HandleTypeDef * pI2c, uint8_t slave_addr, uint8_t* pBuffer, size_t rx_len);
void reset_slaves(I2C_HandleTypeDef * pI2c);
HAL_StatusTypeDef TCA9548A_SelectChannel(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t channel);
HAL_StatusTypeDef TCA9548A_SelectBroadcast(I2C_HandleTypeDef *hi2c, uint8_t address);
HAL_StatusTypeDef ICM20948_ReadID(I2C_HandleTypeDef *hi2c, uint8_t *id);
HAL_StatusTypeDef ICM20948_IsAlive(I2C_HandleTypeDef *hi2c,   uint8_t id);

#endif /* INC_I2C_MASTER_H_ */
