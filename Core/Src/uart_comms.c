/*
 * uart_comms.c
 *
 *  Created on: Sep 30, 2024
 *      Author: GeorgeVigelette
 */

#include "if_commands.h"
#include "main.h"
#include "uart_comms.h"
#include "utils.h"
#include <string.h>
#include "usbd_cdc_if.h"

// Private variables
extern uint8_t rxBuffer[COMMAND_MAX_SIZE];
extern uint8_t txBuffer[COMMAND_MAX_SIZE];
extern DMA_HandleTypeDef hdma_memtomem_dma2_stream1;

volatile uint32_t ptrReceive;
volatile uint8_t rx_flag = 0;
volatile uint8_t tx_flag = 0;
const uint32_t zero_val = 0;

#define TX_TIMEOUT 5

void ClearBuffer_DMA(void)
{
    // Clear using 32-bit writes (COMMAND_MAX_SIZE must be divisible by 4)
    HAL_DMA_Start(&hdma_memtomem_dma2_stream1,
                  (uint32_t)&zero_val,
                  (uint32_t)rxBuffer,
                  COMMAND_MAX_SIZE / 4);

    // Wait until transfer completes (blocking version)
    HAL_DMA_PollForTransfer(&hdma_memtomem_dma2_stream1,
                            HAL_DMA_FULL_TRANSFER,
                            HAL_MAX_DELAY);
}

void comms_interface_send(UartPacket *pResp) {
	tx_flag = 0;  // Clear the flag before starting transmission

//    memset(txBuffer, 0, sizeof(txBuffer));
	int bufferIndex = 0;

	// Build the packet header
	txBuffer[bufferIndex++] = OW_START_BYTE;
	txBuffer[bufferIndex++] = pResp->id >> 8;
	txBuffer[bufferIndex++] = pResp->id & 0xFF;
	txBuffer[bufferIndex++] = OW_RESP;
	txBuffer[bufferIndex++] = pResp->command;
	txBuffer[bufferIndex++] = pResp->addr;
	txBuffer[bufferIndex++] = pResp->reserved;
	txBuffer[bufferIndex++] = (pResp->data_len) >> 8;
	txBuffer[bufferIndex++] = (pResp->data_len) & 0xFF;

	// Check for possible buffer overflow (optional)
	if ((bufferIndex + pResp->data_len + 4) > sizeof(txBuffer)) {
		printf("Packet too large to send\r\n");
		// Handle error: packet too large for txBuffer
		return;
	}

	// Add data payload if any
	if (pResp->data_len > 0) {
		memcpy(&txBuffer[bufferIndex], pResp->data, pResp->data_len);
		bufferIndex += pResp->data_len;
	}

	// Compute CRC over the packet from index 1 for (pResp->data_len + 8) bytes
	uint16_t crc = util_crc16(&txBuffer[1], pResp->data_len + 8);
	txBuffer[bufferIndex++] = crc >> 8;
	txBuffer[bufferIndex++] = crc & 0xFF;

	// Add the end byte
	txBuffer[bufferIndex++] = OW_END_BYTE;

	// Initiate transmission via USB CDC
	CDC_Transmit_HS(txBuffer, bufferIndex);

	// Wait for the transmit complete flag with a timeout to avoid infinite loop.
	uint32_t start_time = HAL_GetTick();

	while (!tx_flag) {
		if ((HAL_GetTick() - start_time) >= TX_TIMEOUT) {
			// Timeout handling: Log error and break out or reset the flag.
			printf("TX Timeout\r\n");
			break;
		}
	}
}

void comms_host_start(void) {

	memset(rxBuffer, 0, sizeof(rxBuffer));
	ptrReceive = 0;

	CDC_FlushRxBuffer_HS();

	rx_flag = 0;
	tx_flag = 0;

	CDC_ReceiveToIdle(rxBuffer, COMMAND_MAX_SIZE);

}

// This is the FreeRTOS task
void comms_host_check_received(void) {
	UartPacket cmd;
	UartPacket resp;
	uint16_t calculated_crc;

	if (!rx_flag)
		return;
//	printf("Packet recieved\r\n");
	int bufferIndex = 0;

	if (rxBuffer[bufferIndex++] != OW_START_BYTE) {
		// Send NACK doesn't have the correct start byte
		resp.id = 0xFFFF;
		resp.data_len = 0;
		resp.packet_type = OW_NAK;
		goto NextDataPacket;
	}

	cmd.id = (rxBuffer[bufferIndex] << 8 | (rxBuffer[bufferIndex + 1] & 0xFF));
	bufferIndex += 2;
	cmd.packet_type = rxBuffer[bufferIndex++];
	cmd.command = rxBuffer[bufferIndex++];
	cmd.addr = rxBuffer[bufferIndex++];
	cmd.reserved = rxBuffer[bufferIndex++];

	// Extract payload length
	cmd.data_len = (rxBuffer[bufferIndex] << 8
			| (rxBuffer[bufferIndex + 1] & 0xFF));
	bufferIndex += 2;

	// Check if data length is valid
	if (cmd.data_len > COMMAND_MAX_SIZE - bufferIndex
			&& rxBuffer[COMMAND_MAX_SIZE - 1] != OW_END_BYTE) {
		// Send NACK response due to no end byte
		// data can exceed buffersize but every buffer must have a start and end packet
		// command that will send more data than one buffer will follow with data packets to complete the request
		resp.id = cmd.id;
		resp.addr = 0;
		resp.reserved = 0;
		resp.data_len = 0;
		resp.packet_type = OW_ERROR;
		goto NextDataPacket;
	}

	// Extract data pointer
	cmd.data = &rxBuffer[bufferIndex];
	if (cmd.data_len > COMMAND_MAX_SIZE) {
		bufferIndex = COMMAND_MAX_SIZE - 3; // [3 bytes from the end should be the crc for a continuation packet]
	} else {
		bufferIndex += cmd.data_len; // move pointer to end of data
	}

	// Extract received CRC
	cmd.crc = (rxBuffer[bufferIndex] << 8 | (rxBuffer[bufferIndex + 1] & 0xFF));
	bufferIndex += 2;

	// Calculate CRC for received data

	if (cmd.data_len > COMMAND_MAX_SIZE) {
		calculated_crc = util_crc16(&rxBuffer[1], COMMAND_MAX_SIZE - 3);
	} else {
		calculated_crc = util_crc16(&rxBuffer[1], cmd.data_len + 8);
	}

	// Check CRC
	if (cmd.crc != calculated_crc) {
		// Send NACK response due to bad CRC
		resp.id = cmd.id;
		resp.addr = 0;
		resp.reserved = OW_BAD_CRC;
		resp.data_len = 0;
		resp.packet_type = OW_ERROR;
		goto NextDataPacket;
	}

	// Check end byte
	if (rxBuffer[bufferIndex++] != OW_END_BYTE) {
		resp.id = cmd.id;
		resp.data_len = 0;
		resp.addr = 0;
		resp.reserved = 0;
		resp.packet_type = OW_ERROR;
		goto NextDataPacket;
	}

	resp = process_if_command(cmd);
	// printf("CMD Packet:\r\n");
	// print_uart_packet(&cmd);

NextDataPacket:
	comms_interface_send(&resp);
	memset(rxBuffer, 0, sizeof(rxBuffer));
	// ClearBuffer_DMA();
	ptrReceive = 0;
	rx_flag = 0;
	CDC_ReceiveToIdle(rxBuffer, COMMAND_MAX_SIZE);
}

// Callback functions
void CDC_handle_RxCpltCallback(uint16_t len) {
	rx_flag = 1;
}

void CDC_handle_TxCpltCallback() {
	tx_flag = 1;
}
