/*
 * i2c_master.c
 *
 *  Created on: Mar 30, 2024
 *      Author: gvigelet
 */

#include "main.h"
#include "i2c_protocol.h"
#include "i2c_master.h"
#include "utils.h"
#include <stdio.h>
#include <string.h>

// I2C Expander specific
#define TCA9548_ADDR 0x70
#define TIMEOUT_MS 5000

uint8_t I2C_current_target = 0x00; // Default target address

volatile uint8_t txComplete = 0;
volatile uint8_t rxComplete = 0;
volatile uint8_t i2cError = 0;


uint8_t I2C_scan(I2C_HandleTypeDef * pI2c, uint8_t* addr_list, size_t list_size, bool display) {

	uint8_t found = 0;

    // Iterate through all possible 7-bit addresses
    for (uint8_t address = 0x00; address <= 0x7F; address++) {
        HAL_StatusTypeDef status;
        status = HAL_I2C_IsDeviceReady(pI2c, address << 1, 2, 50); // Address shift left by 1 for read/write bit
        if (status == HAL_OK) {
        	if(addr_list != NULL && found < list_size) {
        		addr_list[found] = address;
        	}
        	found++;
        	if(display)
        	{
        		printf("%2x ", address);
        	}
        }else{
        	if(display)
        	{
        		printf("-- ");
        	}
        }
        if (display && (address + 1) % 16 == 0)  printf("\r\n");
    }

	if(display)
	{
		printf("\r\n");
	    fflush(stdout);
	}

    return found;
}

uint8_t send_buffer_to_slave(I2C_HandleTypeDef * pI2c, uint8_t slave_addr, uint8_t* pBuffer, uint16_t buf_len)
{
	// Check if the I2C handle is valid
    if (HAL_I2C_GetState(pI2c) != HAL_I2C_STATE_READY) {
    	printf("===> ERROR I2C Not in ready state\r\n");
        return 1; // I2C is not in a ready state
    }

	// printf("===> Sending Packet %d Bytes\r\n", buf_len);
    if(HAL_I2C_Master_Transmit(pI2c, (uint16_t)(slave_addr << 1), pBuffer, buf_len, HAL_MAX_DELAY)!= HAL_OK)
	{
        /* Error_Handler() function is called when error occurs. */
        Error_Handler();
	}


	return 0;
}

uint8_t read_status_register_of_slave(I2C_HandleTypeDef * pI2c, uint8_t slave_addr, uint8_t* pBuffer, uint16_t max_len)
{
	uint8_t rx_len = sizeof(I2C_STATUS_Packet);
	// Check if the I2C handle is valid
    if (HAL_I2C_GetState(pI2c) != HAL_I2C_STATE_READY) {
    	printf("===> ERROR I2C Not in ready state\r\n");
        return 1; // I2C is not in a ready state
    }

	// printf("===> Receive Status Packet %d Bytes\r\n", rx_len);

#if 0

	if(HAL_I2C_Master_Receive(&I2C_DEVICE, (uint16_t)(slave_addr << 1), pBuffer, rx_len, HAL_MAX_DELAY)!= HAL_OK)
	{
        /* Error_Handler() function is called when error occurs. */
        Error_Handler();
	}

#else

    if(HAL_I2C_Mem_Read(pI2c, (uint16_t)(slave_addr << 1), 0x00, I2C_MEMADD_SIZE_8BIT, pBuffer, rx_len, HAL_MAX_DELAY)!= HAL_OK)
    {
        /* Error_Handler() function is called when error occurs. */
        Error_Handler();
    }

#endif

	return rx_len;
}

uint8_t read_data_register_of_slave(I2C_HandleTypeDef * pI2c, uint8_t slave_addr, uint8_t* pBuffer, size_t rx_len)
{
	// Check if the I2C handle is valid
    if (HAL_I2C_GetState(pI2c) != HAL_I2C_STATE_READY) {
    	printf("===> ERROR I2C Not in ready state\r\n");
        return 1; // I2C is not in a ready state
    }

	// printf("===> Receive Data Packet %d Bytes\r\n", rx_len);

    if(HAL_I2C_Mem_Read(pI2c, (uint16_t)(slave_addr << 1), 0x01, I2C_MEMADD_SIZE_8BIT, pBuffer, rx_len, HAL_MAX_DELAY)!= HAL_OK)
    {
        /* Error_Handler() function is called when error occurs. */
        Error_Handler();
    }

	return rx_len;
}

#if 0
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
	printf("TX Completed\r\n");
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
	printf("RX Completed\r\n");

}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	printf("HAL_I2C_MemTxCpltCallback\r\n");
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	printf("HAL_I2C_MemRxCpltCallback\r\n");
}
#endif

void reset_slaves(I2C_HandleTypeDef * pI2c)
{
    printf("Reset Slaves..\r\n");
}


HAL_StatusTypeDef TCA9548A_SelectChannel(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t channel)
{
    if (channel > 7) {
        return HAL_ERROR;  // Invalid channel
    }

    if(verbose_on) printf("TCA9548A_SelectChannel: Switching to channel %d\r\n", channel);
    
    //TODO(remove this check later)
    // Check if after the switch happened, the I2C became busy
    uint32_t isr = hi2c->Instance->ISR;     
    int busy = (isr & I2C_ISR_BUSY) != 0;
    if(busy){
        printf("I2C bus is busy, ISR=0x%08lx\r\n", isr);
    }

    uint8_t data = (1 << channel);  // Set the corresponding bit for the channel

    if(I2C_current_target == data) return HAL_OK; // no need to change
    
    I2C_current_target = data;
    int ret = HAL_I2C_Master_Transmit(hi2c, address << 1, &data, 1, HAL_MAX_DELAY);
    if(ret ==HAL_OK) {
        HAL_Delay(20);
    } else {
        printf("TCA9548A_SelectChannel Error: %d\r\n", ret);
    }

    //TODO(remove this check later)
    // Check if after the switch happened, the I2C became busy
    isr = hi2c->Instance->ISR;     
    busy = (isr & I2C_ISR_BUSY) != 0;
    if(busy){
        printf("TCA: after: I2C bus is busy, ISR=0x%08lx\r\n", isr);
    }

    return ret;
}

HAL_StatusTypeDef TCA9548A_SelectBroadcast(I2C_HandleTypeDef *hi2c, uint8_t address)
{
    uint8_t data = 0xFF;  // Set the corresponding bit for the channel
    if(I2C_current_target == data) return HAL_OK; // no need to change

    I2C_current_target = data;
    return HAL_I2C_Master_Transmit(hi2c, address << 1, &data, 1, HAL_MAX_DELAY);
}

int xi2c_write_bytes(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *data, uint16_t length) {

    uint32_t isr = hi2c->Instance->ISR;     // H7/I2C v2
    int busy = (isr & I2C_ISR_BUSY) != 0;
    if(busy){
        printf("I2C bus is busy, ISR=0x%08lx\r\n", isr);
        return HAL_ERROR;
    }
    return HAL_I2C_Master_Transmit(hi2c, DevAddress << 1, data, length, HAL_MAX_DELAY);
}

int xi2c_write_and_read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *wbuf, uint16_t wlen, uint8_t *rbuf, uint16_t rlen) {
    txComplete = 0;
    rxComplete = 0;
    i2cError = 0;

    if (HAL_I2C_Master_Seq_Transmit_IT(hi2c, DevAddress << 1, wbuf, wlen, I2C_FIRST_FRAME) != HAL_OK)
        return -1;

    // Wait for the transmission to complete
    uint32_t t0 = HAL_GetTick();
    while (!txComplete && !i2cError) {
        // printf("~");
        if ((HAL_GetTick() - t0) >= TIMEOUT_MS){
            printf("I2C receive timeout\r\n");
            uint32_t t3 = HAL_GetTick();
            printf("I2C receive took %lu ms\r\n", t3 - t0);
            printf("txcomplete: %d, i2cError: %d\r\n", txComplete, i2cError);
            return HAL_ERROR;
        }
    }
    uint32_t t_diff = HAL_GetTick() - t0;
    if(t_diff > 0)
        printf("                           I2C write took %lu ms\r\n", t_diff); // ????? why is this always 0??????? 
    // printf("T0: %lu\r\n", t0);
    // printf("T1: %lu\r\n", t1);
    if (i2cError)
    {
        return HAL_ERROR;
    }


    if (HAL_I2C_Master_Seq_Receive_IT(hi2c, DevAddress << 1, rbuf, rlen, I2C_LAST_FRAME) != HAL_OK)
        return -1;

    // Wait for the reception to complete
    t0 = HAL_GetTick();
    while (!rxComplete && !i2cError) {
    	if ((HAL_GetTick() - t0) >= TIMEOUT_MS) return HAL_ERROR;
    }

    if (i2cError)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}


// Callback implementations
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

