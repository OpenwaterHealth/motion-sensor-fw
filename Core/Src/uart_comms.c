/*
 * uart_comms.c
 *
 *  Created on: Sep 30, 2024
 *      Author: GeorgeVigelette
 */

#include "main.h"
#include "uart_comms.h"
#include "if_commands.h"
#include "usbd_comms.h"
#include "usbd_def.h"
#include "utils.h"

#include <string.h>

#include <stdio.h>
#include <inttypes.h>
#include <string.h>

// Private variables
extern uint8_t rxBuffer[COMMAND_MAX_SIZE];
extern uint8_t txBuffer[COMMAND_MAX_SIZE];
extern DMA_HandleTypeDef hdma_memtomem_dma2_stream1;

volatile uint32_t ptrReceive;
volatile uint8_t rx_flag = 0;
volatile uint8_t tx_flag = 1; // Start in idle state
const uint32_t zero_val = 0;

extern USBD_HandleTypeDef hUsbDeviceHS;

#define TX_TIMEOUT 500

__attribute__((unused))
static void print_uart_packet(const UartPacket *pkt)
{
    if (!pkt) {
        printf("UartPacket: NULL\r\n");
        return;
    }

    printf("UartPacket {\r\n");
    printf("  id           : 0x%04" PRIX16 " (%" PRIu16 ")\r\n", pkt->id, pkt->id);
    printf("  packet_type  : 0x%02X (%u)\r\n", pkt->packet_type, pkt->packet_type);
    printf("  command      : 0x%02X (%u)\r\n", pkt->command, pkt->command);
    printf("  addr         : 0x%02X (%u)\r\n", pkt->addr, pkt->addr);
    printf("  reserved     : 0x%02X\r\n", pkt->reserved);
    printf("  data_len     : %" PRIu16 "\r\n", pkt->data_len);

    // print data bytes if available
    if (pkt->data && pkt->data_len > 0) {
        printf("  data         : ");
        for (uint16_t i = 0; i < pkt->data_len; i++) {
            printf("%02X ", pkt->data[i]);
        }
        printf("\r\n");
    } else {
        printf("  data         : <none>\r\n");
    }

    printf("  crc          : 0x%04" PRIX16 " (%" PRIu16 ")\r\n", pkt->crc, pkt->crc);
    printf("}\r\n");
}

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

_Bool comms_interface_send(UartPacket *pResp) {
	if(!tx_flag){
		printf("Comm tx not complete from last time");
		return false;
	}

//    memset(txBuffer, 0, sizeof(txBuffer));
	int bufferIndex = 0;

	// Build the packet header
	txBuffer[bufferIndex++] = OW_START_BYTE;
	txBuffer[bufferIndex++] = pResp->id >> 8;
	txBuffer[bufferIndex++] = pResp->id & 0xFF;
	txBuffer[bufferIndex++] = pResp->packet_type;
	txBuffer[bufferIndex++] = pResp->command;
	txBuffer[bufferIndex++] = pResp->addr;
	txBuffer[bufferIndex++] = pResp->reserved;
	txBuffer[bufferIndex++] = (pResp->data_len) >> 8;
	txBuffer[bufferIndex++] = (pResp->data_len) & 0xFF;

	// Check for possible buffer overflow (optional)
	if ((bufferIndex + pResp->data_len + 4) > sizeof(txBuffer)) {
		printf("Packet too large to send\r\n");
		// Handle error: packet too large for txBuffer
		return false;
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
	tx_flag = 0;  // Set the flag before starting transmission
	uint8_t tx_status = USBD_COMMS_Transmit(&hUsbDeviceHS, txBuffer, bufferIndex);
	if (tx_status != USBD_OK) {
		// Transmission not started (e.g., endpoint busy); don't block on tx_flag
		printf("COMM USB TX failed: id=0x%04X cmd=0x%02X type=0x%02X len=%d dev_state=0x%02X\r\n",
			   pResp->id, pResp->command, pResp->packet_type, bufferIndex, hUsbDeviceHS.dev_state);
		if (tx_status == USBD_BUSY) {
			printf("COMM USB TX failed: endpoint busy\r\n");
		} else if (tx_status == USBD_FAIL) {
			printf("COMM USB TX failed: USBD_FAIL\r\n");
		} else {
			printf("COMM USB TX failed: status=0x%02X\r\n", tx_status);
		}
		tx_flag = 1; // reset to idle on failure
		return false;
	}

	if(pResp->command == OW_CAMERA_GET_HISTOGRAM && verbose_on)
	{
		printf("F:%d C: 0x%02X V: 0x%02X S:%d\r\n", pResp->id, pResp->addr, pResp->reserved, bufferIndex);
	}

	// Wait for the transmit complete flag with a timeout to avoid infinite loop.
	uint32_t start_time = get_timestamp_ms();

	while (!tx_flag) {
		if ((get_timestamp_ms() - start_time) >= TX_TIMEOUT) {
			// Timeout handling: Log error and break out or reset the flag.
			printf("COMM USB TX Timeout\r\n");
			tx_flag = 1; // reset to idle on timeout
			return false;
		}
	}
	return true;
}

void comms_host_start(void) {

	memset(rxBuffer, 0, sizeof(rxBuffer));
	ptrReceive = 0;

	USBD_COMMS_FlushRxBuffer();

	rx_flag = 0;
	tx_flag = 1;

}

// This is the FreeRTOS task
void comms_host_check_received(void) {
	UartPacket cmd;
	UartPacket resp;
	uint16_t calculated_crc;
	if (!rx_flag)
		return;
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
	printf("0x%04X\r\n", cmd.id);
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

NextDataPacket:
	if(comms_interface_send(&resp)){
		printf(".\r\n");
	}
	else {
		printf("!\r\n");
	}
	// printf("[RESP] ID:0x%04X Cmd:0x%02X Type:0x%02X -> Resp:0x%02X Len:%d\r\n",
	// 	   cmd.id, cmd.command, cmd.packet_type, resp.packet_type, resp.data_len);
	memset(rxBuffer, 0, sizeof(rxBuffer));
	// ClearBuffer_DMA();
	ptrReceive = 0;
	rx_flag = 0;
}


// Callback functions
void USBD_COMMS_RxCpltCallback(uint8_t *Buf, uint32_t Len, uint8_t epnum) {
	static volatile uint32_t rx_overrun_count = 0;

	if (rx_flag) {
		rx_overrun_count++;
		printf("COMM RX OVERRUN: count=%lu len=%lu\r\n",
			   (unsigned long)rx_overrun_count,
			   (unsigned long)Len);
	}

	memcpy(rxBuffer, Buf, Len);
	rx_flag = 1;
}

void USBD_COMMS_TxCpltCallback(uint8_t *Buf, uint32_t Len, uint8_t epnum) {
	tx_flag = 1;
}

