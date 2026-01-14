#include "main.h"
#include "logging.h"
#include "lwrb.h"
#include "uart_comms.h"
#include "common.h"
#include "usbd_comms.h"
#include "utils.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

static bool bInit_dma = false;
volatile bool bPrintfTransferComplete = false;

static uint8_t usart_start_tx_dma_transfer(void);

/* Ring buffer for TX data */
lwrb_t usart_tx_buff;
uint8_t usart_tx_buff_data[1512];
volatile size_t usart_tx_dma_current_len;

/* Buffer for accumulating printf messages to send over command and control endpoint */
#define LOG_MSG_BUFFER_SIZE 512
#define TX_TIMEOUT 500  // Timeout in milliseconds for USB transmission
static uint8_t log_msg_buffer[LOG_MSG_BUFFER_SIZE];
static size_t log_msg_buffer_index = 0;
extern USBD_HandleTypeDef hUsbDeviceHS;
extern volatile uint8_t tx_flag;  // Transmit flag from uart_comms.c

/* Forward declarations */
static void send_log_message_over_comms(void);
static void add_char_to_log_buffer(uint8_t ch);

#ifdef __GNUC__
	/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
		 set to 'Yes') calls __io_putchar() */
	#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
	#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */


#ifdef __GNUC__
int _write(int fd, const void *buf, size_t count){
	UNUSED(fd);
	uint8_t * src = (uint8_t *)buf;
	
	// Send to UART (existing functionality)
	if(bInit_dma)
	{
	    if (lwrb_get_free(&usart_tx_buff) >= count) {
	        lwrb_write(&usart_tx_buff, buf, count);
	        usart_start_tx_dma_transfer();
	    }
	}
	else
	{
		HAL_StatusTypeDef com_tx_status = HAL_UART_Transmit(&DEBUG_UART, src, count, 10);
		if(com_tx_status != HAL_OK)
		{
			Error_Handler();
		}
	}

	// Also send to command and control endpoint
	for (size_t i = 0; i < count; i++) {
		add_char_to_log_buffer(src[i]);
	}

	return count;
}
#else
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
	//HAL_UART_Transmit(&DEBUG_UART, (uint8_t *)&ch, 1,10);
	UART_putChar( (uint8_t) ch);
  return ch;
}
#endif

void init_dma_logging()
{
    /* Initialize ringbuff */
    lwrb_init(&usart_tx_buff, usart_tx_buff_data, sizeof(usart_tx_buff_data));

    bInit_dma = true;
	bPrintfTransferComplete = true;
}

bool is_using_dma(){
	return bInit_dma;
}

static uint8_t usart_start_tx_dma_transfer(void) {
    if (usart_tx_dma_current_len == 0 && (usart_tx_dma_current_len = lwrb_get_linear_block_read_length(&usart_tx_buff)) > 0) {

        /* Limit maximal size to transmit at a time */
        if (usart_tx_dma_current_len > 32) {
            usart_tx_dma_current_len = 32;
        }
    	bPrintfTransferComplete = false;
		if(HAL_UART_Transmit_DMA(&DEBUG_UART, (uint8_t*)lwrb_get_linear_block_read_address(&usart_tx_buff), usart_tx_dma_current_len)!= HAL_OK)
		{
			Error_Handler();
		}
    }
    return 1;
}

void logging_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart)
{

}


void logging_UART_ErrorCallback(UART_HandleTypeDef *huart)
{

}


void logging_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	bPrintfTransferComplete = true;
    lwrb_skip(&usart_tx_buff, usart_tx_dma_current_len);/* Data sent, ignore these */
    usart_tx_dma_current_len = 0;
    usart_start_tx_dma_transfer();          /* Try to send more data */
}

/* Send accumulated log message over command and control endpoint */
static void send_log_message_over_comms(void) {
	if (log_msg_buffer_index == 0) {
		return; // Nothing to send
	}

	// Prepare packet
	UartPacket log_packet;
	log_packet.id = 0; // ensure that every packet like this has a zero id to avoid confusion with the packet id
	log_packet.packet_type = OW_DATA;
	log_packet.command = OW_CMD_ECHO; // Use ECHO command for log messages
	log_packet.addr = 0;
	log_packet.reserved = 0;
	log_packet.data_len = log_msg_buffer_index;
	log_packet.data = log_msg_buffer;

	// Send via comms interface (non-blocking, will use existing txBuffer)
	// We need to build the packet manually to avoid using the shared txBuffer
	// which might be in use. Instead, we'll use a local buffer.
	static uint8_t log_tx_buffer[COMMAND_MAX_SIZE];
	int bufferIndex = 0;

	// Build the packet header
	log_tx_buffer[bufferIndex++] = OW_START_BYTE;
	log_tx_buffer[bufferIndex++] = log_packet.id >> 8;
	log_tx_buffer[bufferIndex++] = log_packet.id & 0xFF;
	log_tx_buffer[bufferIndex++] = log_packet.packet_type;
	log_tx_buffer[bufferIndex++] = log_packet.command;
	log_tx_buffer[bufferIndex++] = log_packet.addr;
	log_tx_buffer[bufferIndex++] = log_packet.reserved;
	log_tx_buffer[bufferIndex++] = (log_packet.data_len) >> 8;
	log_tx_buffer[bufferIndex++] = (log_packet.data_len) & 0xFF;

	// Add data payload
	if (log_packet.data_len > 0) {
		memcpy(&log_tx_buffer[bufferIndex], log_packet.data, log_packet.data_len);
		bufferIndex += log_packet.data_len;
	}

	// Compute CRC over the packet from index 1 for (data_len + 8) bytes
	uint16_t crc = util_crc16(&log_tx_buffer[1], log_packet.data_len + 8);
	log_tx_buffer[bufferIndex++] = crc >> 8;
	log_tx_buffer[bufferIndex++] = crc & 0xFF;

	// Add the end byte
	log_tx_buffer[bufferIndex++] = OW_END_BYTE;

	// Clear the transmit flag before starting transmission
	tx_flag = 0;

	// Send via USB CDC (non-blocking)
	// Note: This may fail silently if USB is not initialized, which is acceptable
	USBD_COMMS_Transmit(&hUsbDeviceHS, log_tx_buffer, bufferIndex);
	
	// Wait for the transmit complete flag with a timeout to avoid infinite loop.
	uint32_t start_time = get_timestamp_ms();
	while (!tx_flag) {
		if ((get_timestamp_ms() - start_time) >= TX_TIMEOUT) {
			// Timeout handling: Log error and break out
			// Note: Don't reset tx_flag here as it may be set asynchronously
			break;
		}
	}
	
	// Clear the buffer
	log_msg_buffer_index = 0;
}

/* Add character to log message buffer and send if newline encountered */
static void add_char_to_log_buffer(uint8_t ch) {
	// If buffer is full, send what we have and reset
	if (log_msg_buffer_index >= (LOG_MSG_BUFFER_SIZE - 1)) {
		send_log_message_over_comms();
	}

	// Add character to buffer
	log_msg_buffer[log_msg_buffer_index++] = ch;

	// Send message when we encounter a newline
	if (ch == '\n') {
		send_log_message_over_comms();
	}
}
