/*
 * camera_manager.c
 *
 *  Created on: Mar 5, 2025
 *      Author: GeorgeVigelette
 */

#include "main.h"
#include "crosslink.h"
#include "0X02C1B.h"
#include "i2c_master.h"
#include "uart_comms.h"
#include "utils.h"
#include "usbd_histo.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#define FPGA_I2C_ADDRESS 0x40  // Replace with your FPGA's I2C address
#define HISTO_JSON_BUFFER_SIZE 34000
#define HISTO_SIZE_32B 1024
CameraDevice cam_array[CAMERA_COUNT];	// array of all the cameras

static int _active_cam_idx = 0;

volatile uint8_t frame_buffer[2][CAMERA_COUNT * HISTOGRAM_DATA_SIZE]; // Double buffer
uint8_t packet_buffer[HISTO_JSON_BUFFER_SIZE];

static uint8_t _active_buffer = 0; // Index of the buffer currently being written to
uint8_t frame_id = 0;
extern uint8_t event_bits_enabled; // holds the event bits for the cameras to be enabled
extern uint8_t event_bits;
extern bool fake_data_gen;
extern USBD_HandleTypeDef hUsbDeviceHS;

 __attribute__((section(".sram4"))) volatile uint8_t spi6_buffer[SPI_PACKET_LENGTH];


static void generate_fake_histogram(uint8_t *histogram_data) {
    // Cast the byte buffer to uint32_t pointer to store histogram data
    uint32_t *histogram = (uint32_t *)histogram_data;

    // Initialize histogram bins to zero
    memset(histogram, 0, HISTOGRAM_DATA_SIZE/4);

    // Generate random 10-bit grayscale image and compute histogram
    switch(HISTO_TEST_PATTERN){
    	case 0: // this one will run VERY slow but look like an actual histo. run once at startup.
			for (int i = 0; i < WIDTH * HEIGHT; i++) {
					uint32_t pixel_value = rand() % HISTOGRAM_BINS; // Random 10-bit value (0-1023)
					histogram[pixel_value]++;
				}
			break;
    	case 1:
    		for(int i=0;i<HISTOGRAM_BINS;i++){
    			histogram[i] = (uint32_t) (i + frame_id);
    		}
			break;
		case 2:
    		for(int i=0;i<HISTOGRAM_BINS;i++){
    			histogram[i] =  (uint32_t) 0xAAAAAAAA;
    		}
			break;
		case 3:
			for(int i=0;i<HISTOGRAM_BINS;i++){
    			histogram[i] =  i;//(i>HISTOGRAM_BINS) ? (uint32_t) 1024: 2048;
    		}
			break;
		
    }

	histogram_data[0]+= 0x06;
    histogram[HISTOGRAM_BINS-1] |= ((uint32_t) frame_id)<<24; // fill in the frame_id to the last bin's spacer
}

static void init_camera(CameraDevice *cam){
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	// Reconfigure CRESETB Pin
	HAL_GPIO_DeInit(cam->cresetb_port, cam->cresetb_pin);
	GPIO_InitStruct.Pin = cam->cresetb_pin; // Same pin
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Push-pull output
	GPIO_InitStruct.Pull = GPIO_NOPULL;         // No pull-up or pull-down
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // Set the speed
	HAL_GPIO_Init(cam->cresetb_port, &GPIO_InitStruct);

	// Reconfigure GPIO0 Pin
	HAL_GPIO_DeInit(cam->gpio0_port, cam->gpio0_pin);
	GPIO_InitStruct.Pin = cam->gpio0_pin; // Same pin
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Push-pull output
	GPIO_InitStruct.Pull = GPIO_NOPULL;         // No pull-up or pull-down
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // Set the speed
	HAL_GPIO_Init(cam->gpio0_port, &GPIO_InitStruct);

	cam->streaming_enabled = false;
}

void init_camera_sensors() {
	int i = 0;

	//Configure, initialize, and set default camera
	cam_array[0].id = 0;
	cam_array[0].cresetb_port = CRESET_1_GPIO_Port;
	cam_array[0].cresetb_pin = CRESET_1_Pin;
	cam_array[0].gpio0_port = GPIO0_1_GPIO_Port;
	cam_array[0].gpio0_pin = GPIO0_1_Pin;
	cam_array[0].useUsart = true;
	cam_array[0].useDma = true;
	cam_array[0].pI2c = &hi2c1;
	cam_array[0].device_address = FPGA_I2C_ADDRESS;
	cam_array[0].pSpi = NULL;
	cam_array[0].pUart = &husart2;
	cam_array[0].i2c_target = 0;
	cam_array[0].pRecieveHistoBuffer = NULL;

	cam_array[1].id = 1;
	cam_array[1].cresetb_port = CRESET_2_GPIO_Port;
	cam_array[1].cresetb_pin = CRESET_2_Pin;
	cam_array[1].gpio0_port = GPIO0_2_GPIO_Port;
	cam_array[1].gpio0_pin = GPIO0_2_Pin;
	cam_array[1].useUsart = false;
	cam_array[1].useDma = true;
	cam_array[1].pI2c = &hi2c1;
	cam_array[1].device_address = FPGA_I2C_ADDRESS;
	cam_array[1].pSpi = &hspi6;
	cam_array[1].pUart = NULL;
	cam_array[1].i2c_target = 1;
	cam_array[1].pRecieveHistoBuffer = NULL;

	cam_array[2].id = 2;
	cam_array[2].cresetb_port = CRESET_3_GPIO_Port;
	cam_array[2].cresetb_pin = CRESET_3_Pin;
	cam_array[2].gpio0_port = GPIO0_3_GPIO_Port;
	cam_array[2].gpio0_pin = GPIO0_3_Pin;
	cam_array[2].useUsart = true;
	cam_array[2].useDma = true;
	cam_array[2].pI2c = &hi2c1;
	cam_array[2].device_address = FPGA_I2C_ADDRESS;
	cam_array[2].pSpi = NULL;
	cam_array[2].pUart = &husart3;
	cam_array[2].i2c_target = 2;
	cam_array[2].pRecieveHistoBuffer = NULL;

	cam_array[3].id = 3;
	cam_array[3].cresetb_port = CRESET_4_GPIO_Port;
	cam_array[3].cresetb_pin = CRESET_4_Pin;
	cam_array[3].gpio0_port = GPIO0_4_GPIO_Port;
	cam_array[3].gpio0_pin = GPIO0_4_Pin;
	cam_array[3].useUsart = true;
	cam_array[3].useDma = true;
	cam_array[3].pI2c = &hi2c1;
	cam_array[3].device_address = FPGA_I2C_ADDRESS;
	cam_array[3].pSpi = NULL;
	cam_array[3].pUart = &husart6;
	cam_array[3].i2c_target = 3;
	cam_array[3].pRecieveHistoBuffer = NULL;

	cam_array[4].id = 4;
	cam_array[4].cresetb_port = CRESET_5_GPIO_Port;
	cam_array[4].cresetb_pin = CRESET_5_Pin;
	cam_array[4].gpio0_port = GPIO0_5_GPIO_Port;
	cam_array[4].gpio0_pin = GPIO0_5_Pin;
	cam_array[4].useUsart = true;
	cam_array[4].useDma = true;
	cam_array[4].pI2c = &hi2c1;
	cam_array[4].device_address = FPGA_I2C_ADDRESS;
	cam_array[4].pSpi = NULL;
	cam_array[4].pUart = &husart1;
	cam_array[4].i2c_target = 4;
	cam_array[4].pRecieveHistoBuffer = NULL;

	cam_array[5].id = 5;
	cam_array[5].cresetb_port = CRESET_6_GPIO_Port;
	cam_array[5].cresetb_pin = CRESET_6_Pin;
	cam_array[5].gpio0_port = GPIO0_6_GPIO_Port;
	cam_array[5].gpio0_pin = GPIO0_6_Pin;
	cam_array[5].useUsart = false;
	cam_array[5].useDma = true;
	cam_array[5].pI2c = &hi2c1;
	cam_array[5].device_address = FPGA_I2C_ADDRESS;
	cam_array[5].pSpi = &hspi3;
	cam_array[5].pUart = NULL;
	cam_array[5].i2c_target = 5;
	cam_array[5].pRecieveHistoBuffer = NULL;

	cam_array[6].id = 6;
	cam_array[6].cresetb_port = CRESET_7_GPIO_Port;
	cam_array[6].cresetb_pin = CRESET_7_Pin;
	cam_array[6].gpio0_port = GPIO0_7_GPIO_Port;
	cam_array[6].gpio0_pin = GPIO0_7_Pin;
	cam_array[6].useUsart = false;
	cam_array[6].useDma = true;
	cam_array[6].pI2c = &hi2c1;
	cam_array[6].device_address = FPGA_I2C_ADDRESS;
	cam_array[6].pSpi = &hspi2;
	cam_array[6].pUart = NULL;
	cam_array[6].i2c_target = 6;
	cam_array[6].pRecieveHistoBuffer = NULL;

	cam_array[7].id = 7;
	cam_array[7].cresetb_port = CRESET_8_GPIO_Port;
	cam_array[7].cresetb_pin = CRESET_8_Pin;
	cam_array[7].gpio0_port = GPIO0_8_GPIO_Port;
	cam_array[7].gpio0_pin = GPIO0_8_Pin;
	cam_array[7].useUsart = false;
	cam_array[7].useDma = true;
	cam_array[7].pI2c = &hi2c1;
	cam_array[7].device_address = FPGA_I2C_ADDRESS;
	cam_array[7].pSpi = &hspi4;
	cam_array[7].pUart = NULL;
	cam_array[7].i2c_target = 7;
	cam_array[7].pRecieveHistoBuffer = NULL;

	for(i=0; i<CAMERA_COUNT; i++){
		cam_array[i].pRecieveHistoBuffer =(uint8_t *)&frame_buffer[_active_buffer][i * HISTOGRAM_DATA_SIZE];
		init_camera(&cam_array[i]);
	}

	cam_array[1].pRecieveHistoBuffer = (uint8_t *)spi6_buffer;

	event_bits = 0x00;
	event_bits_enabled = 0x00;

	fill_frame_buffers();
}

CameraDevice* get_active_cam(void) {
	return &cam_array[_active_cam_idx];
}

CameraDevice* set_active_camera(int id) {
	if(id < 0 || id >= CAMERA_COUNT) return NULL;

	_active_cam_idx = id;
	return &cam_array[_active_cam_idx];
}

CameraDevice* get_camera_byID(int id) {
	if(id < 0 || id >= CAMERA_COUNT)
		return NULL;
	return &cam_array[id];
}

_Bool reset_camera(uint8_t cam_id)
{
	if(cam_id < 0 || cam_id >= CAMERA_COUNT)
	{
		printf("Hard Reset Camera %d Failed\r\n", cam_id+1);
		return false;
	}

	printf("Hard Reset Camera %d Started\r\n", cam_id+1);
	_active_cam_idx = cam_id;
	CameraDevice *cam = &cam_array[_active_cam_idx];

    HAL_GPIO_WritePin(cam->cresetb_port, cam->cresetb_pin, GPIO_PIN_SET);
    HAL_Delay(5);

    HAL_GPIO_WritePin(cam->cresetb_port, cam->cresetb_pin, GPIO_PIN_RESET);
    HAL_Delay(1000);

	return true;
}

_Bool enable_fpga(uint8_t cam_id)
{
	if(cam_id < 0 || cam_id >= CAMERA_COUNT)
	{
		printf("Enable FPGA Camera %d Failed\r\n", cam_id+1);
		return false;
	}

	printf("Enable FPGA Camera %d Started\r\n", cam_id+1);
	_active_cam_idx = cam_id;
	CameraDevice *cam = &cam_array[_active_cam_idx];

    HAL_GPIO_WritePin(cam->cresetb_port, cam->cresetb_pin, GPIO_PIN_SET);
    HAL_Delay(2);
	return true;
}

_Bool disable_fpga(uint8_t cam_id)
{
	if(cam_id < 0 || cam_id >= CAMERA_COUNT)
	{
		printf("Disable FPGA Camera %d Failed\r\n", cam_id+1);
		return false;
	}

	printf("Disable FPGA Camera %d Started\r\n", cam_id+1);
	_active_cam_idx = cam_id;
	CameraDevice *cam = &cam_array[_active_cam_idx];

    HAL_GPIO_WritePin(cam->cresetb_port, cam->cresetb_pin, GPIO_PIN_RESET);
    HAL_Delay(2);

	return true;
}

_Bool activate_fpga(uint8_t cam_id)
{
	if(cam_id < 0 || cam_id >= CAMERA_COUNT)
	{
		printf("Activate FPGA Camera %d Failed\r\n", cam_id+1);
		return false;
	}

	printf("Activate FPGA Camera %d Started\r\n", cam_id+1);
	_active_cam_idx = cam_id;
	CameraDevice *cam = &cam_array[_active_cam_idx];

	if(TCA9548A_SelectChannel(&hi2c1, 0x70, cam->i2c_target) != HAL_OK)
	{
		printf("failed to select Camera %d channel\r\n", cam_id+1);
		return false;
	}

	if(fpga_send_activation(cam->pI2c, cam->device_address) == 1)
	{
		printf("Activate FPGA Camera %d Failed\r\n", cam_id+1);
		return false;
	}
	return true;
}

_Bool verify_fpga(uint8_t cam_id)
{
	if(cam_id < 0 || cam_id >= CAMERA_COUNT)
	{
		printf("Activate FPGA Camera %d Failed\r\n", cam_id+1);
		return false;
	}

	printf("Activate FPGA Camera %d Started\r\n", cam_id+1);
	_active_cam_idx = cam_id;
	CameraDevice *cam = &cam_array[_active_cam_idx];

	if(TCA9548A_SelectChannel(&hi2c1, 0x70, cam->i2c_target) != HAL_OK)
	{
		printf("failed to select Camera %d channel\r\n", cam_id+1);
		return false;
	}

	if(fpga_checkid(cam->pI2c, cam->device_address) == 1)
	{
		printf("Activate FPGA Camera %d Failed\r\n", cam_id+1);
		return false;
	}
	return true;
}

_Bool enter_sram_prog_fpga(uint8_t cam_id)
{
	if(cam_id < 0 || cam_id >= CAMERA_COUNT)
	{
		printf("Activate FPGA Camera %d Failed\r\n", cam_id+1);
		return false;
	}

	printf("Activate FPGA Camera %d Started\r\n", cam_id+1);
	_active_cam_idx = cam_id;
	CameraDevice *cam = &cam_array[_active_cam_idx];

	if(TCA9548A_SelectChannel(&hi2c1, 0x70, cam->i2c_target) != HAL_OK)
	{
		printf("failed to select Camera %d channel\r\n", cam_id+1);
		return false;
	}

	if(fpga_enter_sram_prog_mode(cam->pI2c, cam->device_address) == 1)
	{
		printf("Activate FPGA Camera %d Failed\r\n", cam_id+1);
		return false;
	}
	return true;
}

_Bool exit_sram_prog_fpga(uint8_t cam_id)
{
	if(cam_id < 0 || cam_id >= CAMERA_COUNT)
	{
		printf("Activate FPGA Camera %d Failed\r\n", cam_id+1);
		return false;
	}

	printf("Activate FPGA Camera %d Started\r\n", cam_id+1);
	_active_cam_idx = cam_id;
	CameraDevice *cam = &cam_array[_active_cam_idx];

	if(TCA9548A_SelectChannel(&hi2c1, 0x70, cam->i2c_target) != HAL_OK)
	{
		printf("failed to select Camera %d channel\r\n", cam_id+1);
		return false;
	}

	if(fpga_enter_sram_prog_mode(cam->pI2c, cam->device_address) == 1)
	{
		printf("Activate FPGA Camera %d Failed\r\n", cam_id+1);
		return false;
	}
	return true;
}

_Bool erase_sram_fpga(uint8_t cam_id)
{
	if(cam_id < 0 || cam_id >= CAMERA_COUNT)
	{
		printf("Activate FPGA Camera %d Failed\r\n", cam_id+1);
		return false;
	}

	printf("Activate FPGA Camera %d Started\r\n", cam_id+1);
	_active_cam_idx = cam_id;
	CameraDevice *cam = &cam_array[_active_cam_idx];

	if(TCA9548A_SelectChannel(&hi2c1, 0x70, cam->i2c_target) != HAL_OK)
	{
		printf("failed to select Camera %d channel\r\n", cam_id+1);
		return false;
	}

	if(fpga_erase_sram(cam->pI2c, cam->device_address) == 1)
	{
		printf("Activate FPGA Camera %d Failed\r\n", cam_id+1);
		return false;
	}
	return true;
}

uint32_t read_status_fpga(uint8_t cam_id)
{
	uint32_t ret_val = 0;
	if(cam_id < 0 || cam_id >= CAMERA_COUNT)
	{
		printf("Activate FPGA Camera %d Failed\r\n", cam_id+1);
		return ret_val;
	}

	printf("Activate FPGA Camera %d Started\r\n", cam_id+1);
	_active_cam_idx = cam_id;
	CameraDevice *cam = &cam_array[_active_cam_idx];

	if(TCA9548A_SelectChannel(&hi2c1, 0x70, cam->i2c_target) != HAL_OK)
	{
		printf("failed to select Camera %d channel\r\n", cam_id+1);
		return false;
	}

	ret_val = fpga_read_status(cam->pI2c, cam->device_address);

	return ret_val;
}

uint32_t read_usercode_fpga(uint8_t cam_id)
{
	uint32_t ret_val = 0;
	if(cam_id < 0 || cam_id >= CAMERA_COUNT)
	{
		printf("Activate FPGA Camera %d Failed\r\n", cam_id+1);
		return ret_val;
	}

	printf("Activate FPGA Camera %d Started\r\n", cam_id+1);
	_active_cam_idx = cam_id;
	CameraDevice *cam = &cam_array[_active_cam_idx];

	if(TCA9548A_SelectChannel(&hi2c1, 0x70, cam->i2c_target) != HAL_OK)
	{
		printf("failed to select Camera %d channel\r\n", cam_id+1);
		return false;
	}

	ret_val = fpga_read_usercode(cam->pI2c, cam->device_address);

	return ret_val;
}

_Bool program_sram_fpga(uint8_t cam_id, bool rom_bitstream, uint8_t* pData, uint32_t Data_Len)
{
	if(cam_id < 0 || cam_id >= CAMERA_COUNT)
	{
		printf("Program FPGA Camera %d Failed\r\n", cam_id+1);
		return false;
	}

	printf("Program FPGA Camera %d Started\r\n", cam_id+1);
	_active_cam_idx = cam_id;
	CameraDevice *cam = &cam_array[_active_cam_idx];

	if(TCA9548A_SelectChannel(&hi2c1, 0x70, cam->i2c_target) != HAL_OK)
	{
		printf("failed to select Camera %d channel\r\n", cam_id+1);
		return false;
	}

	if(fpga_program_sram(cam->pI2c, cam->device_address, rom_bitstream, pData, Data_Len) == 1)
	{
		printf("Program FPGA Camera %d Failed\r\n", cam_id+1);
		return false;
	}

	return true;
}

_Bool program_fpga(uint8_t cam_id)
{
	if(cam_id < 0 || cam_id >= CAMERA_COUNT)
	{
		printf("Program FPGA Camera %d Failed\r\n", cam_id+1);
		return false;
	}

	printf("Program FPGA Camera %d Started\r\n", cam_id+1);
	_active_cam_idx = cam_id;
	CameraDevice *cam = &cam_array[_active_cam_idx];

	if(TCA9548A_SelectChannel(&hi2c1, 0x70, cam->i2c_target) != HAL_OK)
	{
		printf("failed to select Camera %d channel\r\n", cam_id+1);
		return false;
	}

	if(fpga_configure(cam->pI2c, cam->device_address, cam->cresetb_port, cam->cresetb_pin) == 1)
	{
		printf("Program FPGA Camera %d Failed\r\n", cam_id+1);
		return false;
	}

	// If the selected camera is one that uses USART, 
	// reset the USART to ensure it is in a known state
	// !! This is required for the USART to work properly after FPGA programming !!
	if(cam->useUsart)
	{
		cam->pUart->Instance->CR1 &= ~USART_CR1_UE; // Disable USART
		cam->pUart->Instance->CR1 |= USART_CR1_UE;
	}

	return true;
}

_Bool configure_camera_sensor(uint8_t cam_id)
{
	if(cam_id < 0 || cam_id >= CAMERA_COUNT)
	{
		printf("Configure Camera %d Registers Failed\r\n", cam_id+1);
		return false;
	}

	printf("Configure Camera %d Registers Started\r\n", cam_id+1);
	_active_cam_idx = cam_id;
	CameraDevice *cam = &cam_array[_active_cam_idx];

	if(TCA9548A_SelectChannel(&hi2c1, 0x70, cam->i2c_target) != HAL_OK)
	{
		printf("failed to select Camera %d channel\r\n", cam_id+1);
		return false;
	}

	if(X02C1B_configure_sensor(cam) == 1)
	{
		printf("Configure Camera %d Registers Failed\r\n", cam_id+1);
		return false;
	}

	return true;
}

_Bool configure_camera_testpattern(uint8_t cam_id, uint8_t test_pattern)
{
	if(cam_id < 0 || cam_id >= CAMERA_COUNT)
	{
		printf("Configure Camera %d Registers Failed\r\n", cam_id+1);
		return false;
	}

	printf("Configure Camera %d Test Pattern Started\r\n", cam_id+1);
	_active_cam_idx = cam_id;
	CameraDevice *cam = &cam_array[_active_cam_idx];

	if(TCA9548A_SelectChannel(&hi2c1, 0x70, cam->i2c_target) != HAL_OK)
	{
		printf("failed to select Camera %d channel\r\n", cam_id+1);
		return false;
	}

	if(X02C1B_set_test_pattern(cam, test_pattern) == 1)
	{
		printf("Configure Camera %d Test Pattern Failed\r\n", cam_id+1);
		return false;
	}

	return true;
}

_Bool get_single_histogram(uint8_t cam_id, uint8_t* data, uint16_t* data_len)
{
	if(cam_id < 0 || cam_id >= CAMERA_COUNT)
	{
		printf("Capture HISTO for Camera %d Failed\r\n", cam_id+1);
		return false;
	}

	printf("Get HISTO for Camera %d Registers Started\r\n", cam_id+1);
	_active_cam_idx = cam_id;
	CameraDevice *cam = &cam_array[_active_cam_idx];

    if (!cam->pRecieveHistoBuffer) {
        printf("No histogram buffer for camera %d\r\n", cam_id+1);
        return false;
    }

    // Copy data into the provided buffer
    memcpy(data, cam->pRecieveHistoBuffer, HISTOGRAM_DATA_SIZE);
    *data_len = HISTOGRAM_DATA_SIZE;

    return true;
}

_Bool capture_single_histogram(uint8_t cam_id)
{
	_Bool ret = true;
	HAL_StatusTypeDef status = HAL_OK;
	if(cam_id < 0 || cam_id >= CAMERA_COUNT)
	{
		printf("Capture HISTO for Camera %d Failed\r\n", cam_id+1);
		return false;
	}

	printf("Capture HISTO for Camera %d Registers Started\r\n", cam_id+1);
	_active_cam_idx = cam_id;
	CameraDevice *cam = &cam_array[_active_cam_idx];

	if(TCA9548A_SelectChannel(&hi2c1, 0x70, cam->i2c_target) != HAL_OK)
	{
		printf("failed to select Camera %d channel\r\n", cam_id+1);
		return false;
	}

	GPIO_SetOutput(FSIN_GPIO_Port, FSIN_Pin, GPIO_PIN_RESET);
	HAL_Delay(25);

	memset((uint8_t*)cam->pRecieveHistoBuffer, 0, HISTOGRAM_DATA_SIZE);

	start_data_reception(cam_id);

	if(status != HAL_OK)
	{
		printf("failed to setup receive for Camera %d channel\r\n", cam_id+1);
		return false;
	}

	X02C1B_stream_on(cam);
	HAL_Delay(10);

	HAL_GPIO_WritePin(FSIN_GPIO_Port, FSIN_Pin, GPIO_PIN_SET);
	HAL_Delay(25);
	HAL_GPIO_WritePin(FSIN_GPIO_Port, FSIN_Pin, GPIO_PIN_RESET);
	HAL_Delay(25);
//	HAL_GPIO_WritePin(FSIN_GPIO_Port, FSIN_Pin, GPIO_PIN_SET);
//	HAL_Delay(25);
//	HAL_GPIO_WritePin(FSIN_GPIO_Port, FSIN_Pin, GPIO_PIN_RESET);

	uint32_t timeout = HAL_GetTick() + 5000; // 100ms timeout example

	uint8_t event_bit_single = 0x01 << cam_id;
	while(event_bits != event_bit_single) {
	    if (HAL_GetTick() > timeout) {
	        printf("USART receive timeout!\r\n");
	        if(cam->useUsart) {
	            HAL_DMA_Abort(cam->pUart->hdmarx); // safely abort DMA
	            __HAL_USART_DISABLE(cam->pUart);   // disable USART
	            cam->pUart->RxXferCount = 0;       // force clear counters
	            __HAL_USART_ENABLE(cam->pUart);    // re-enable USART
	        } else {
	            HAL_DMA_Abort(cam->pSpi->hdmarx);  // safely abort DMA
	            __HAL_SPI_DISABLE(cam->pSpi);      // disable SPI
	            cam->pSpi->RxXferCount = 0;
	            __HAL_SPI_ENABLE(cam->pSpi);       // re-enable SPI
	        }
	    	ret = false;
	        break;
	    }
	    HAL_Delay(1);
	}

	HAL_Delay(10);
	X02C1B_stream_off(cam);
	printf("Received Frame\r\n");

	return ret;
}

void switch_frame_buffer(void) {
    _active_buffer = 1 - _active_buffer; // Toggle between 0 and 1
    // Reassign each camera’s buffer pointer
    for (int i = 0; i < CAMERA_COUNT; i++) {
        cam_array[i].pRecieveHistoBuffer =(uint8_t *)&frame_buffer[_active_buffer][i * HISTOGRAM_DATA_SIZE];
    }
}

uint8_t* get_active_frame_buffer(void) {
    return (uint8_t*)frame_buffer[_active_buffer];
}

uint8_t* get_inactive_frame_buffer(void) {
    return (uint8_t*)frame_buffer[1 - _active_buffer];
}

void fill_frame_buffers(void) {
    for (int i = 0; i < CAMERA_COUNT; i++) {
    	generate_fake_histogram(cam_array[i].pRecieveHistoBuffer);
    }
}

_Bool send_fake_data(void) {

	//TODO(update this to work similar to the other packetizer, refactor)
	uint8_t *fb = get_active_frame_buffer();

	fill_frame_buffers();
	//uint8_t packet_buffer[HISTO_JSON_BUFFER_SIZE];

    // do the work of copying the data into from the camera buffers into the usb buffer
    int offset = 0;
    size_t buf_size = HISTO_JSON_BUFFER_SIZE;

    offset += snprintf(packet_buffer + offset, buf_size - offset, "{\n");
    
    for (int cam = 0; cam < CAMERA_COUNT; ++cam) {
        offset += snprintf(packet_buffer + offset, buf_size - offset, "  \"H%d\": [", cam);
		
		uint32_t *histo_ptr = cam_array[cam].pRecieveHistoBuffer;
        for (int i = 0; i < HISTO_SIZE_32B - 1; ++i) {
            offset += snprintf(packet_buffer + offset, buf_size - offset, "%lu,", histo_ptr[i]);
        }
		uint32_t cam_frame_id = ((histo_ptr[HISTO_SIZE_32B - 1] & 0xFF000000) >> 24);
		offset += snprintf(packet_buffer + offset, buf_size - offset, "%lu", 
							(histo_ptr[HISTO_SIZE_32B - 1] & 0x00FFFFFF));

		offset += snprintf(packet_buffer + offset, buf_size - offset,
                           (cam < CAMERA_COUNT - 1) ? "],\n" : "]\n");
    }

    offset += snprintf(packet_buffer + offset, buf_size - offset,
                       "  ,\"META\": {\n    \"frame_id\": %u\n  }\n", frame_id);

    offset += snprintf(packet_buffer + offset, buf_size - offset, "}\n");

	uint8_t status = USBD_HISTO_SetTxBuffer(&hUsbDeviceHS, packet_buffer, offset);

	//TODO( get prev packet completed send, handle if not completed)
	if(status != USBD_OK)
		printf("failed to send\r\n");

	frame_id++;

	return true;
}

_Bool start_data_reception(uint8_t cam_id){
	HAL_StatusTypeDef status;
	CameraDevice cam = cam_array[cam_id];

	if (cam.useUsart) {
		if (cam.useDma) {
			status = HAL_USART_Receive_DMA(cam.pUart,
					cam.pRecieveHistoBuffer, USART_PACKET_LENGTH);
		} else {
			status = HAL_USART_Receive_IT(cam.pUart,
					cam.pRecieveHistoBuffer, USART_PACKET_LENGTH);
		}
	} else {
		if (cam.useDma) {
			status = HAL_SPI_Receive_DMA(cam.pSpi,
					cam.pRecieveHistoBuffer, SPI_PACKET_LENGTH);
		} else {
			status = HAL_SPI_Receive_IT(cam.pSpi,
					cam.pRecieveHistoBuffer, SPI_PACKET_LENGTH);
		}
	}
	if (status != HAL_OK) {
		printf("failed to setup receive for Camera %d channel\r\n", cam_id+1);
		abort_data_reception(cam_id);
		return false;
	}
	return true;
}

_Bool abort_data_reception(uint8_t cam_id){
	HAL_StatusTypeDef status;
	// disable the reception
	if(get_camera_byID(cam_id)->useUsart) {
		if(get_camera_byID(cam_id)->useDma)
			status = HAL_USART_Abort(get_camera_byID(cam_id)->pUart);
		else
			status = HAL_USART_Abort_IT(get_camera_byID(cam_id)->pUart);
	}
	else{
		if(get_camera_byID(cam_id)->useDma)
			status = HAL_SPI_Abort(get_camera_byID(cam_id)->pSpi);
		else
			status = HAL_SPI_Abort_IT(get_camera_byID(cam_id)->pSpi);
	}
	if (status != HAL_OK) {
		return false;
	}
	return true;
}

_Bool enable_camera_stream(uint8_t cam_id){
	bool status = false;
	bool enabled = (event_bits_enabled & (1 << cam_id)) != 0;
	if(enabled){
		printf("Camera %d already enabled\r\n", cam_id+1);
		return true;
	}
	CameraDevice *cam = get_camera_byID(cam_id);

	if(TCA9548A_SelectChannel(&hi2c1, 0x70, cam->i2c_target) != HAL_OK)
		{
			printf("failed to select Camera %d channel\r\n", cam_id+1);
			return false;
		}

	bool data_recp_status= start_data_reception(cam_id);
	bool stream_on_status= (X02C1B_stream_on(cam) < 0); // returns -1 if failed

	status |= data_recp_status | stream_on_status;
	if(!status)
	{
		printf("Failed to start camera %d stream\r\n", cam_id +1);
		return false;
	}
	event_bits_enabled |= (1 << cam_id);
	cam->streaming_enabled = true;
	printf("Enabled cam %d stream (%02X)\r\n", cam_id+1, event_bits_enabled);
	return true;
}

_Bool disable_camera_stream(uint8_t cam_id){
//	printf("Disable C: %d\r\n",cam_id);
	bool status = false;
	bool enabled = (event_bits_enabled & (1 << cam_id)) != 0;
	if(!enabled){
		printf("Camera %d already disabled\r\n", cam_id+1);
		return true;
	}
	CameraDevice *cam = get_camera_byID(cam_id);

	if(TCA9548A_SelectChannel(&hi2c1, 0x70, cam->i2c_target) != HAL_OK)
		{
			printf("failed to select Camera %d channel\r\n", cam_id+1);
			return false;
		}

	status |= abort_data_reception(cam_id);
	status |= (X02C1B_stream_off(cam) < 0); // returns -1 if failed

	if(!status)
	{
		printf("Failed to stop camera %d stream\r\n", cam_id+1);
		return false;
	}
	event_bits_enabled &= ~(1 << cam_id);
	cam->streaming_enabled = false;
//	printf("Disabled cam %d stream (%02X)\r\n", cam_id+1, event_bits_enabled);
	return true;
}

_Bool toggle_camera_stream(uint8_t cam_id){
	_Bool status = false;
    bool enabled = (event_bits_enabled & (1 << cam_id)) != 0;
	if(enabled) status = disable_camera_stream(cam_id);
	else status = enable_camera_stream(cam_id);

	return status;
}

_Bool send_histogram_data(void) {
	_Bool status = true;
	int offset = 0;

	uint8_t count = 0;
	for (int i = 0; i < CAMERA_COUNT ; ++i) {
		if (event_bits_enabled & (1 << i)) {
			count++;
		}
	}
	uint32_t payload_size = count*(HISTO_SIZE_32B*4+3);
    uint32_t total_size = HISTO_HEADER_SIZE + payload_size + HISTO_TRAILER_SIZE;
    if (HISTO_JSON_BUFFER_SIZE < total_size) {
        return false;  // Buffer too small
    }

	HAL_GPIO_TogglePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin);
	
	// --- Header ---
    packet_buffer[offset++] = HISTO_SOF;
    packet_buffer[offset++] = TYPE_HISTO;
    packet_buffer[offset++] = (uint8_t)(total_size & 0xFF);
    packet_buffer[offset++] = (uint8_t)((total_size >> 8) & 0xFF);
    packet_buffer[offset++] = (uint8_t)((total_size >> 16) & 0xFF);
    packet_buffer[offset++] = (uint8_t)((total_size >> 24) & 0xFF);

	// --- Data ---
	for (uint8_t cam_id = 0; cam_id < CAMERA_COUNT; ++cam_id) {
		if((event_bits_enabled & (0x01 << cam_id)) != 0) {
			uint32_t *histo_ptr = cam_array[cam_id].pRecieveHistoBuffer;
		    packet_buffer[offset++] = HISTO_SOH;
			packet_buffer[offset++] = cam_id;
			memcpy(packet_buffer+offset,histo_ptr,HISTO_SIZE_32B*4);
			offset += HISTO_SIZE_32B*4;
			packet_buffer[offset++] = HISTO_EOH;
		}
	}

	// --- Footer --- 
	uint16_t crc = util_crc16(packet_buffer, offset - 1);  // From 'type' to EOH
    packet_buffer[offset++] = crc & 0xFF;
    packet_buffer[offset++] = (crc >> 8) & 0xFF;
    packet_buffer[offset++] = HISTO_EOF;
		
	uint8_t tx_status = USBD_HISTO_SetTxBuffer(&hUsbDeviceHS, packet_buffer, offset);

	//TODO( handle the case where the packet fails to send better)
	if(tx_status != USBD_OK){
		printf("failed to send\r\n");
		status = false;
	}
	HAL_GPIO_TogglePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin);

	// kick off the next frame reception
	for(int i = 0;i<CAMERA_COUNT;i++){
		if((event_bits_enabled & (0x01 << i)) != 0)
			start_data_reception(i);
	}
	frame_id++;

	return status;
}

//Get SPI/usart status for the camera
_Bool get_camera_status(uint8_t cam_id) {
	if (cam_id < 0 || cam_id >= CAMERA_COUNT) {
		printf("Get Camera %d Status Failed\r\n", cam_id + 1);
		return false;
	}

	CameraDevice *cam = get_camera_byID(cam_id);
	if (cam->useUsart) {
		HAL_USART_StateTypeDef usart_state;
		usart_state = HAL_USART_GetState(cam->pUart);
		
		if(usart_state == HAL_USART_STATE_RESET){
			printf("USART state: HAL_USART_STATE_RESET\r\n");
		}
		else if(usart_state == HAL_USART_STATE_READY){
			printf("USART state: HAL_USART_STATE_READY\r\n");
		}
		else if(usart_state == HAL_USART_STATE_BUSY){
			printf("USART state: HAL_USART_STATE_BUSY\r\n");
		}
		else if(usart_state == HAL_USART_STATE_BUSY_TX){
			printf("USART state: HAL_USART_STATE_BUSY_TX\r\n");
		}
		else if(usart_state == HAL_USART_STATE_BUSY_RX){
			printf("USART state: HAL_USART_STATE_BUSY_RX\r\n");
		}
		else if(usart_state == HAL_USART_STATE_BUSY_TX_RX){
			printf("USART state: HAL_USART_STATE_BUSY_TX_RX\r\n");
		}
		else if(usart_state == HAL_USART_STATE_TIMEOUT){
			printf("USART state: HAL_USART_STATE_TIMEOUT\r\n");
		}
		else if(usart_state == HAL_USART_STATE_ERROR){
			printf("USART state: HAL_USART_STATE_ERROR\r\n");
		}
		else{
			printf("USART state: Unknown\r\n");
		}
		return HAL_USART_GetState(cam->pUart) == HAL_USART_STATE_READY;
	} else {
		HAL_SPI_StateTypeDef spi_state;
		spi_state = HAL_SPI_GetState(cam->pSpi);

		if(spi_state == HAL_SPI_STATE_RESET){
			printf("SPI state: HAL_SPI_STATE_RESET\r\n");
		}
		else if(spi_state == HAL_SPI_STATE_READY){
			printf("SPI state: HAL_SPI_STATE_READY\r\n");
		}
		else if(spi_state == HAL_SPI_STATE_BUSY){
			printf("SPI state: HAL_SPI_STATE_BUSY\r\n");
		}
		else if(spi_state == HAL_SPI_STATE_BUSY_TX){
			printf("SPI state: HAL_SPI_STATE_BUSY_TX\r\n");
		}
		else if(spi_state == HAL_SPI_STATE_BUSY_RX){
			printf("SPI state: HAL_SPI_STATE_BUSY_RX\r\n");
		}
		else if(spi_state == HAL_SPI_STATE_BUSY_TX_RX){
			printf("SPI state: HAL_SPI_STATE_BUSY_TX_RX\r\n");
		}
		else if(spi_state == HAL_SPI_STATE_ERROR){
			printf("SPI state: HAL_SPI_STATE_ERROR\r\n");
		}
		else if(spi_state == HAL_SPI_STATE_ABORT){
			printf("SPI state: HAL_SPI_STATE_ABORT\r\n");
		}
		else{
			printf("SPI state: Unknown\r\n");
		}
		return HAL_SPI_GetState(cam->pSpi) == HAL_SPI_STATE_READY;
	}
}
