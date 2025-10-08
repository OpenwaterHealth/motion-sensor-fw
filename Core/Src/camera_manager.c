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
#include <stdbool.h>

#define CAM_TEMP_TOTAL_CAMS   8
#define CAM_TEMP_IDLE_EXTRA_MS 200 // idle after 8 cameras
#define CAM_TEMP_CYCLE_MS ((CAM_TEMP_INTERVAL_MS * CAM_TEMP_TOTAL_CAMS) + CAM_TEMP_IDLE_EXTRA_MS)

volatile float cam_temp[CAMERA_COUNT] = {0};   // °C ×100
static uint32_t next_temp_ms = 0;
static uint8_t  next_cam_idx = 0;

#define FPGA_I2C_ADDRESS 0x40  // Replace with your FPGA's I2C address
#define HISTO_JSON_BUFFER_SIZE 34000
#define HISTO_SIZE_32B 1024
CameraDevice cam_array[CAMERA_COUNT];	// array of all the cameras

static int _active_cam_idx = 0;
static uint8_t cameras_present = 0x00;

volatile bool usb_failed = false;

__ALIGN_BEGIN volatile uint8_t frame_buffer[2][CAMERA_COUNT * HISTOGRAM_DATA_SIZE] __ALIGN_END; // Double buffer
__ALIGN_BEGIN uint8_t packet_buffer[HISTO_JSON_BUFFER_SIZE] __ALIGN_END; 

static uint8_t _active_buffer = 0; // Index of the buffer currently being written to
volatile uint8_t frame_id = 0;
extern uint8_t event_bits_enabled; // holds the event bits for the cameras to be enabled
extern uint8_t event_bits;
extern bool fake_data_gen;
extern USBD_HandleTypeDef hUsbDeviceHS;


// Variables for keeping track of statisticss
uint32_t total_frames_sent = 0;
uint32_t total_frames_failed = 0;

#define STREAMING_TIMEOUT_MS 75
uint32_t most_recent_frame_time = 0;
uint32_t streaming_start_time = 0;
bool streaming_active = false;

 __ALIGN_BEGIN __attribute__((section(".sram4"))) volatile uint8_t spi6_buffer[SPI_PACKET_LENGTH] __ALIGN_END;


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

	// Reconfigure GPIO1 Pin
	HAL_GPIO_DeInit(cam->gpio1_port, cam->gpio1_pin);
    GPIO_InitStruct.Pin = cam->gpio1_pin;            // PA9 = USART1_TX
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // Push-pull output
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(cam->gpio1_port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(cam->gpio1_port, cam->gpio1_pin, GPIO_PIN_RESET);  // Set GPIO1 low

	cam->streaming_enabled = false;
	cam->isConfigured = false;
	cam->isProgrammed = false;
}

void init_camera_sensors() {
	int i = 0;

	//Configure, initialize, and set default camera
	cam_array[0].id = 0;
	cam_array[0].cresetb_port = CRESET_1_GPIO_Port;
	cam_array[0].cresetb_pin = CRESET_1_Pin;
	cam_array[0].gpio0_port = GPIO0_1_GPIO_Port;
	cam_array[0].gpio0_pin = GPIO0_1_Pin;
	cam_array[0].gpio1_port = GPIOA;
	cam_array[0].gpio1_pin = GPIO_PIN_2;
	cam_array[0].power_port = CAM_PWR_1_GPIO_Port;
	cam_array[0].power_pin = CAM_PWR_1_Pin;
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
	cam_array[1].gpio1_port = GPIOA;
	cam_array[1].gpio1_pin = GPIO_PIN_6;
	cam_array[1].power_port = CAM_PWR_2_GPIO_Port;
	cam_array[1].power_pin = CAM_PWR_2_Pin;
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
	cam_array[2].gpio1_port = GPIOD;
	cam_array[2].gpio1_pin = GPIO_PIN_8;
	cam_array[2].power_port = CAM_PWR_3_GPIO_Port;
	cam_array[2].power_pin = CAM_PWR_3_Pin;
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
	cam_array[3].gpio1_port = GPIOC;
	cam_array[3].gpio1_pin = GPIO_PIN_6;
	cam_array[3].power_port = CAM_PWR_4_GPIO_Port;
	cam_array[3].power_pin = CAM_PWR_4_Pin;
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
	cam_array[4].gpio1_port = GPIOB;
	cam_array[4].gpio1_pin = GPIO_PIN_6;
	cam_array[4].power_port = CAM_PWR_5_GPIO_Port;
	cam_array[4].power_pin = CAM_PWR_5_Pin;
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
	cam_array[5].gpio1_port = GPIOC;
	cam_array[5].gpio1_pin = GPIO_PIN_11;
	cam_array[5].power_port = CAM_PWR_6_GPIO_Port;
	cam_array[5].power_pin = CAM_PWR_6_Pin;
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
	cam_array[6].gpio1_port = GPIOB;
	cam_array[6].gpio1_pin = GPIO_PIN_14;
	cam_array[6].power_port = CAM_PWR_7_GPIO_Port;
	cam_array[6].power_pin = CAM_PWR_7_Pin;
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
	cam_array[7].gpio1_port = GPIOE;
	cam_array[7].gpio1_pin = GPIO_PIN_5;
	cam_array[7].power_port = CAM_PWR_8_GPIO_Port;
	cam_array[7].power_pin = CAM_PWR_8_Pin;
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
		cam_array[i].isPowered = true; // Initialize all cameras as powered on
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

// Get SPI/USART status for the specified camera ID
// Returns a bitfield where each bit indicates a specific status:
//
// Bit 0: SPI or USART state is READY
// Bit 1: Camera firmware is programmed
// Bit 2: Camera is configured
// Bit 7: Streaming is enabled
//
// Bits 3–6: Reserved (unused)
//
uint8_t get_camera_status(uint8_t cam_id) {
	
	uint8_t status_flags = 0x00;

	if (cam_id < 0 || cam_id >= CAMERA_COUNT) {
		printf("Get Camera %d Status Failed\r\n", cam_id + 1);
		return false;
	}

	CameraDevice *cam = get_camera_byID(cam_id);
	if (cam->useUsart) {
		HAL_USART_StateTypeDef usart_state;
		usart_state = HAL_USART_GetState(cam->pUart);
		
		if(usart_state == HAL_USART_STATE_READY) {			
			status_flags |= (1 << 0);  // Set bit 0
		} else {
			// Print the USART state for debugging
			if(usart_state == HAL_USART_STATE_RESET){
				printf("USART state: HAL_USART_STATE_RESET\r\n");
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
		}
	} else {
		HAL_SPI_StateTypeDef spi_state;
		spi_state = HAL_SPI_GetState(cam->pSpi);
		if(spi_state == HAL_SPI_STATE_READY) {			
			status_flags |= (1 << 0);  // Set bit 0
		} else {
			// Print the SPI state for debugging
			if(spi_state == HAL_SPI_STATE_RESET){
				printf("SPI state: HAL_SPI_STATE_RESET\r\n");
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
		}
	}

	if(cam->isProgrammed) {
		status_flags |= (1 << 1);  // Set bit 1
	}

	if(cam->isConfigured) {
		status_flags |= (1 << 2);  // Set bit 2		
	}

	if(cam->streaming_enabled) {
		status_flags |= (1 << 7);  // Set bit 7		
	}

	return status_flags;
}

void print_active_cameras(uint8_t cameras_present)
{
    printf("Active cameras: ");
    for (int i = 0; i < 8; ++i)
    {
        if (cameras_present & (1 << i))
        {
            printf("%d ", i + 1);  // Cameras are 1-based
        }
    }
    printf("\r\n");
}


/* -------- START FPGA FUNCTIONS -------- */
_Bool reset_camera(uint8_t cam_id)
{
	if(cam_id < 0 || cam_id >= CAMERA_COUNT)
	{
		printf("Hard Reset Camera %d Failed\r\n", cam_id+1);
		return false;
	}

	// printf("Hard Reset Camera %d Started\r\n", cam_id+1);
	_active_cam_idx = cam_id;
	CameraDevice *cam = &cam_array[_active_cam_idx];

    HAL_GPIO_WritePin(cam->cresetb_port, cam->cresetb_pin, GPIO_PIN_SET);
    HAL_Delay(5);

    HAL_GPIO_WritePin(cam->cresetb_port, cam->cresetb_pin, GPIO_PIN_RESET);
    HAL_Delay(1000);

    cam->isConfigured = false;
    cam->isProgrammed = false;

	return true;
}

_Bool enable_fpga(uint8_t cam_id)
{
	if(cam_id < 0 || cam_id >= CAMERA_COUNT)
	{
		printf("Enable FPGA Camera %d Failed\r\n", cam_id+1);
		return false;
	}

	// printf("Enable FPGA Camera %d Started\r\n", cam_id+1);
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

	// printf("Disable FPGA Camera %d Started\r\n", cam_id+1);
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

	// printf("Activate FPGA Camera %d Started\r\n", cam_id+1);
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

	// printf("Activate FPGA Camera %d Started\r\n", cam_id+1);
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

	// printf("Activate FPGA Camera %d Started\r\n", cam_id+1);
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

	// printf("Activate FPGA Camera %d Started\r\n", cam_id+1);
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

	// printf("Activate FPGA Camera %d Started\r\n", cam_id+1);
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
	}else{
		cam->isProgrammed = false;
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

	// printf("Activate FPGA Camera %d Started\r\n", cam_id+1);
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

	// printf("Activate FPGA Camera %d Started\r\n", cam_id+1);
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

_Bool program_sram_fpga(uint8_t cam_id, bool rom_bitstream, uint8_t* pData, uint32_t Data_Len, _Bool force_update)
{
	printf("\r\nProgramming FPGA Camera %d...", cam_id+1);
	if(cam_id < 0 || cam_id >= CAMERA_COUNT)
	{
		printf("Program FPGA Camera %d Failed\r\n", cam_id+1);
		return false;
	}

	// printf("Program FPGA Camera %d Started\r\n", cam_id+1);
	_active_cam_idx = cam_id;
	CameraDevice *cam = &cam_array[_active_cam_idx];

	if(!force_update)
	{
		if(cam->isProgrammed) return true;
	} else {
		cam->isProgrammed = false; // set programmed to false and Program FPGA
	}

	if(TCA9548A_SelectChannel(&hi2c1, 0x70, cam->i2c_target) != HAL_OK)
	{
		printf("failed to select Camera %d channel\r\n", cam_id+1);
		return false;
	}

	if(fpga_program_sram(cam->pI2c, cam->device_address, rom_bitstream, pData, Data_Len) == 1)
	{
		printf("Program FPGA Camera %d Failed\r\n", cam_id+1);
		return false;
	}else{

	}
	printf("done\r\n");
	return true;
}

_Bool program_fpga(uint8_t cam_id, _Bool force_update)
{
	printf("\r\nProgramming FPGA Camera %d...", cam_id+1);
	if(cam_id < 0 || cam_id >= CAMERA_COUNT)
	{
		printf("Program FPGA Camera %d Failed\r\n", cam_id+1);
		return false;
	}

	// printf("Program FPGA Camera %d Started\r\n", cam_id+1);
	_active_cam_idx = cam_id;
	CameraDevice *cam = &cam_array[_active_cam_idx];

	// Check if camera is powered on before attempting to program
	if(!cam->isPowered)
	{
		printf("Cannot program FPGA - Camera %d is powered off\r\n", cam_id+1);
		cam->isProgrammed = false;
		return false;
	}

	if(!force_update)
	{
		if(cam->isProgrammed){
			printf("already programmed\r\n");
			return true;
		} 
	} else {
		cam->isProgrammed = false; // set isProgrammed to false and program FPGA
	}

	if(TCA9548A_SelectChannel(&hi2c1, 0x70, cam->i2c_target) != HAL_OK)
	{
		printf("failed to select Camera %d channel\r\n", cam_id+1);
		return false;
	}

	if(fpga_configure(cam->pI2c, cam->device_address, cam->cresetb_port, cam->cresetb_pin) == 1)
	{
		printf("Program FPGA Camera %d Failed\r\n", cam_id+1);

		cam->isProgrammed = false;
		return false;
	} else {
		cam->isProgrammed = true;
	}

	// If the selected camera is one that uses USART, 
	// reset the USART to ensure it is in a known state
	// !! This is required for the USART to work properly after FPGA programming !!
	if(cam->useUsart)
	{
		cam->pUart->Instance->CR1 &= ~USART_CR1_UE; // Disable USART
		cam->pUart->Instance->CR1 |= USART_CR1_UE;
	}
	printf("done\r\n");

	return true;
}
/* -------- END FPGA FUNCTIONS -------- */

/* -------- START CAMERA I2C FUNCTIONS -------- */
void scan_camera_sensors(bool scanI2cAtStart){
  // Scan for I2C cameras
  for (int i = 0; i < CAMERA_COUNT; i++)
  {
	uint8_t addresses_found[10];
	uint8_t found;
	bool camera_found = false, fpga_found = false;

	TCA9548A_SelectChannel(&hi2c1, 0x70, i);
	HAL_Delay(10);

	if (scanI2cAtStart)
	  printf("I2C Scanning bus %d\r\n", i + 1);
	found = I2C_scan(&hi2c1, addresses_found, sizeof(addresses_found), scanI2cAtStart);

	for (int j = 0; j < found; j++)
	{
	  if (addresses_found[j] == 0x36)
		camera_found = true;
	  else if (addresses_found[j] == 0x40)
		fpga_found = true;
	}

	if (camera_found && fpga_found)
	  cameras_present |= 0x01 << i;
	else
	  printf("Camera %d not found\r\n", i + 1);
  }

  if (cameras_present == 0xFF)
  {
	printf("All cameras found\r\n");
  }else{
	  print_active_cameras(cameras_present);
  }
}

_Bool configure_camera_sensor(uint8_t cam_id)
{
	printf("Configuring Camera %d...", cam_id+1);
	if(cam_id < 0 || cam_id >= CAMERA_COUNT)
	{
		printf("Configure Camera %d Registers Failed\r\n", cam_id+1);
		return false;
	}

	// printf("Configure Camera %d Registers Started\r\n", cam_id+1);
	_active_cam_idx = cam_id;
	CameraDevice *cam = &cam_array[_active_cam_idx];

	// Check if camera is already configured and powered on
	if(cam->isConfigured && cam->isPowered)
	{
		printf("Camera %d Sensor already configured\r\n", cam_id+1);
		return true;
	}

	// Check if camera is powered on before attempting to configure
	if(!cam->isPowered)
	{
		printf("Cannot configure Camera %d - camera is powered off\r\n", cam_id+1);
		cam->isConfigured = false;
		return false;
	}

	if(TCA9548A_SelectChannel(&hi2c1, 0x70, cam->i2c_target) != HAL_OK)
	{
		printf("failed to select Camera %d channel\r\n", cam_id+1);
		return false;
	}

	if(X02C1B_configure_sensor(cam) == 1)
	{
		printf("Configure Camera %d Registers Failed\r\n", cam_id+1);
		cam->isConfigured = false;
		return false;
	}else{
		cam->isConfigured = true;
		printf("done\r\n");

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

	// printf("Configure Camera %d Test Pattern Started\r\n", cam_id+1);
	_active_cam_idx = cam_id;
	CameraDevice *cam = &cam_array[_active_cam_idx];

	if(!cam->isConfigured)
	{
		printf("Camera %d Register Update Failed it is not configured\r\n", cam_id+1);
		return false;
	}

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

void poll_camera_temperatures(void)
{
    if (HAL_GetTick() >= next_temp_ms)
    {
        next_temp_ms += CAM_TEMP_INTERVAL_MS;

        // Skip to next active camera
        for (uint8_t i = 0; i < CAM_TEMP_TOTAL_CAMS; i++)
        {
            uint8_t cam = next_cam_idx;
            next_cam_idx = (next_cam_idx + 1) % CAM_TEMP_TOTAL_CAMS;

            if (cameras_present & (1 << cam))
            {
                CameraDevice *pCam = get_camera_byID(cam);
                if (pCam != NULL)
                {
                    cam_temp[cam] = X02C1B_read_temp(pCam);
                    break;  // One camera per 100ms interval
                }
            }
        }

        // If we wrapped back to camera 0, add the idle pause
        if (next_cam_idx == 0)
        {
            next_temp_ms += CAM_TEMP_IDLE_EXTRA_MS;
        }
    }
}
/* -------- END CAMERA I2C FUNCTIONS -------- */
/* -------- START FRAME BUFFER FUNCTIONS -------- */
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

void fill_frame_buffers(void) {
    for (int i = 0; i < CAMERA_COUNT; i++) {
    	generate_fake_histogram(cam_array[i].pRecieveHistoBuffer);
    }
}
/* -------- END FRAME BUFFER FUNCTIONS -------- */
/* -------- START HISTOGRAM TRANSFER FUNCTIONS -------- */
_Bool get_single_histogram(uint8_t cam_id, uint8_t* data, uint16_t* data_len)
{
	if(cam_id < 0 || cam_id >= CAMERA_COUNT)
	{
		printf("Capture HISTO for Camera %d Failed\r\n", cam_id+1);
		return false;
	}

	// printf("Get HISTO for Camera %d Registers Started\r\n", cam_id+1);
	_active_cam_idx = cam_id;
	CameraDevice *cam = &cam_array[_active_cam_idx];

	// get camera event bits
	if (!cam->pRecieveHistoBuffer || !(event_bits & (1 << cam_id))) {
        printf("No histogram buffer for camera %d\r\n", cam_id+1);
        return false;
    }

    // Copy data into the provided buffer
    memcpy(data, cam->pRecieveHistoBuffer, cam->useUsart ? USART_PACKET_LENGTH : SPI_PACKET_LENGTH);
    *data_len = cam->useUsart ? USART_PACKET_LENGTH : SPI_PACKET_LENGTH;
    event_bits = 0x00;
    return true;
}

_Bool capture_single_histogram(uint8_t cam_id)
{
	_Bool ret = true;
	if(cam_id < 0 || cam_id >= CAMERA_COUNT)
	{
		printf("Capture HISTO for Camera %d Failed\r\n", cam_id+1);
		return false;
	}

	// printf("Capture HISTO for Camera %d Registers Started\r\n", cam_id+1);
	_active_cam_idx = cam_id;
	CameraDevice *cam = &cam_array[_active_cam_idx];

	if(TCA9548A_SelectChannel(&hi2c1, 0x70, cam->i2c_target) != HAL_OK)
	{
		printf("failed to select Camera %d channel\r\n", cam_id+1);
		return false;
	}

	GPIO_SetOutput(FSIN_GPIO_Port, FSIN_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);

	memset((uint8_t*)cam->pRecieveHistoBuffer, 0, cam->useUsart ? USART_PACKET_LENGTH : SPI_PACKET_LENGTH);

	start_data_reception(cam_id);

	X02C1B_stream_on(cam);
	HAL_Delay(2);
	HAL_GPIO_WritePin(cam->gpio1_port, cam->gpio1_pin, GPIO_PIN_SET); // Set GPIO1 high

	HAL_GPIO_WritePin(FSIN_GPIO_Port, FSIN_Pin, GPIO_PIN_SET);
	HAL_Delay(2);
	HAL_GPIO_WritePin(FSIN_GPIO_Port, FSIN_Pin, GPIO_PIN_RESET);
	HAL_Delay(2);

	uint32_t timeout = HAL_GetTick() + 5000; // 100ms timeout example

	while(!event_bits) {
	    if (HAL_GetTick() > timeout) {
	        printf("HISTO receive timeout!\r\n");
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

	HAL_Delay(1);
	X02C1B_stream_off(cam);
	// printf("Received Frame\r\n");
	HAL_GPIO_WritePin(cam->gpio1_port, cam->gpio1_pin, GPIO_PIN_RESET); // Set GPIO1 low

	return ret;
}

_Bool send_data(void) {

	// Take care of statistics
	if(!streaming_active){
		streaming_start_time = HAL_GetTick();
		streaming_active = true;
	}
	most_recent_frame_time = HAL_GetTick();

	bool success = false;
	if (fake_data_gen) // Call FAKE data sender if enabled)
    {
        success = send_fake_data();
    }
    else {
       success = send_histogram_data();
    }
	event_bits = 0x00;
	poll_camera_temperatures();

    if (success) {
		total_frames_sent++;
	} else {
		total_frames_failed++;
	}
	return success;
}

_Bool check_streaming(void){
	if(streaming_active){
		if(HAL_GetTick() - most_recent_frame_time > STREAMING_TIMEOUT_MS){
			uint32_t elapsed = HAL_GetTick() - streaming_start_time;
			send_data(); // send data one last frame to finish the buffers 
			printf("Cameras have stopped sending data %lu ms\r\n", elapsed);
			printf("Total frames sent: %lu, failed: %lu\r\n", total_frames_sent, total_frames_failed);
			total_frames_sent = 0;
			total_frames_failed = 0;
			streaming_active = false;
		}
	}
	return streaming_active;
}

_Bool send_histogram_data(void) {
	_Bool status = true;
	int offset = 0;

	uint8_t count = 0;
	for (int i = 0; i < CAMERA_COUNT ; ++i) {
		if (event_bits & (1 << i)) {
			count++;
		}
	}
	if(count == 0)
		printf("No cameras have data to send\r\n");
	uint32_t payload_size = count*(HISTO_SIZE_32B*4+7); // 7 = SOH + CAM_ID + TEMPx4 + EOH
    uint32_t total_size = HISTO_HEADER_SIZE + payload_size + HISTO_TRAILER_SIZE;
    if (HISTO_JSON_BUFFER_SIZE < total_size) {
        return false;  // Buffer too small
    }

	// HAL_GPIO_TogglePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin);
	
	// --- Header ---
    packet_buffer[offset++] = HISTO_SOF;
    packet_buffer[offset++] = TYPE_HISTO;
    packet_buffer[offset++] = (uint8_t)(total_size & 0xFF);
    packet_buffer[offset++] = (uint8_t)((total_size >> 8) & 0xFF);
    packet_buffer[offset++] = (uint8_t)((total_size >> 16) & 0xFF);
    packet_buffer[offset++] = (uint8_t)((total_size >> 24) & 0xFF);
	if(total_size>32833){
		printf("Packet too large\r\n");
	}

	// --- Data ---
	for (uint8_t cam_id = 0; cam_id < CAMERA_COUNT; ++cam_id) {
		if((event_bits & (0x01 << cam_id)) != 0) {
			uint32_t *histo_ptr = (uint32_t *)cam_array[cam_id].pRecieveHistoBuffer;
		    packet_buffer[offset++] = HISTO_SOH;
			packet_buffer[offset++] = cam_id;
			memcpy(packet_buffer+offset,histo_ptr,HISTO_SIZE_32B*4);
			offset += HISTO_SIZE_32B*4;
			
			uint32_t temp_bits;
			memcpy(&temp_bits, (uint8_t*)&cam_temp[cam_id],4);

			packet_buffer[offset++] = (uint8_t)(temp_bits & 0xFF);
			packet_buffer[offset++] = (uint8_t)((temp_bits >> 8) & 0xFF);
			packet_buffer[offset++] = (uint8_t)((temp_bits >> 16) & 0xFF);
			packet_buffer[offset++] = (uint8_t)((temp_bits >> 24) & 0xFF);

			packet_buffer[offset++] = HISTO_EOH;
			
		}
	}

	// --- Footer --- 
	uint16_t crc = util_crc16(packet_buffer, offset - 1);  // From 'type' to EOH
    packet_buffer[offset++] = crc & 0xFF;
    packet_buffer[offset++] = (crc >> 8) & 0xFF;
    packet_buffer[offset++] = HISTO_EOF;
	
	// Send data with 3 retries if it fails, print - if retry and F if failed after 3 tries
	uint8_t tx_status = USBD_HISTO_SetTxBuffer(&hUsbDeviceHS, packet_buffer, offset);
	uint8_t timeout_tries = 0;
	while(tx_status != USBD_OK){
		printf("-\r\n");
		HAL_Delay(1);
		tx_status = USBD_HISTO_SetTxBuffer(&hUsbDeviceHS, packet_buffer, offset);
		timeout_tries++;
		if(timeout_tries > 2){
			printf("F\r\n");
			status = false;
			usb_failed = true;
			break;
		}
	}
	// HAL_GPIO_TogglePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin);

	// kick off the next frame reception
	for(int i = 0;i<CAMERA_COUNT;i++){
		if((event_bits & (0x01 << i)) != 0)
			start_data_reception(i);
	}
	frame_id++;

	return status;
}

_Bool send_fake_data(void) {

	fill_frame_buffers();

	_Bool status = true;
	int offset = 0;
	

	uint8_t count = 0;
	for (int i = 0; i < CAMERA_COUNT ; ++i) {
		if (event_bits_enabled & (1 << i)) {
			count++;
		}
	}
	uint32_t payload_size = count*(HISTO_SIZE_32B*4+7); // 7 = SOH + CAM_ID + TEMPx4 + EOH
	uint32_t total_size = HISTO_HEADER_SIZE + payload_size + HISTO_TRAILER_SIZE;
	if (HISTO_JSON_BUFFER_SIZE < total_size) {
		return false;  // Buffer too small
	}

	// HAL_GPIO_TogglePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin);

	// --- Header ---
	packet_buffer[offset++] = HISTO_SOF;
	packet_buffer[offset++] = TYPE_HISTO;
	packet_buffer[offset++] = (uint8_t)(total_size & 0xFF);
	packet_buffer[offset++] = (uint8_t)((total_size >> 8) & 0xFF);
	packet_buffer[offset++] = (uint8_t)((total_size >> 16) & 0xFF);
	packet_buffer[offset++] = (uint8_t)((total_size >> 24) & 0xFF);

	// --- Data ---
	for (uint8_t cam_id = 0; cam_id < count; ++cam_id) {
		if((event_bits_enabled & (0x01 << cam_id)) != 0) {
			uint32_t *histo_ptr = (uint32_t *) cam_array[cam_id].pRecieveHistoBuffer;
			packet_buffer[offset++] = HISTO_SOH;
			packet_buffer[offset++] = cam_id;
			memcpy(packet_buffer+offset,histo_ptr,HISTO_SIZE_32B*4);
			offset += HISTO_SIZE_32B*4;

			uint32_t temp_bits;
			memcpy(&temp_bits, (uint8_t*)&cam_temp[cam_id],4);

			packet_buffer[offset++] = (uint8_t)(temp_bits & 0xFF);
			packet_buffer[offset++] = (uint8_t)((temp_bits >> 8) & 0xFF);
			packet_buffer[offset++] = (uint8_t)((temp_bits >> 16) & 0xFF);
			packet_buffer[offset++] = (uint8_t)((temp_bits >> 24) & 0xFF);


			packet_buffer[offset++] = HISTO_EOH;
		}
	}
	// --- Footer --- 
	uint16_t crc = util_crc16(packet_buffer, offset - 1);  // From 0 to EOH
	packet_buffer[offset++] = crc & 0xFF;
	packet_buffer[offset++] = (crc >> 8) & 0xFF;
	packet_buffer[offset++] = HISTO_EOF;

	// HAL_GPIO_TogglePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin);

	uint8_t tx_status = USBD_HISTO_SetTxBuffer(&hUsbDeviceHS, packet_buffer, offset);

	//TODO( handle the case where the packet fails to send better)
	if(tx_status != USBD_OK){
		printf("failed to send, fid: %d\r\n",frame_id);
		status = false;
		usb_failed = true;
	}
	if(status && usb_failed){
		printf("USB RECOVERED\r\n");
		usb_failed = false;
	}
	frame_id++;

	return true;
}

_Bool start_data_reception(uint8_t cam_id){
	printf("Start data reception on camera: %d... ",cam_id+1);
	HAL_StatusTypeDef status;
	CameraDevice cam = cam_array[cam_id];

    // Check if the device is BUSY
    if (cam.useUsart) {
        if (cam.pUart->State == HAL_USART_STATE_BUSY_RX ||
            cam.pUart->State == HAL_USART_STATE_BUSY_TX_RX) {
				printf("USART busy\r\n");
				return false;  // Device is busy, don't start another reception
        }
    } else {
        if (cam.pSpi->State == HAL_SPI_STATE_BUSY_RX ||
            cam.pSpi->State == HAL_SPI_STATE_BUSY_TX_RX) {
            printf("SPI busy\r\n");
            return false;  // Device is busy, don't start another reception
        }
	}

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
		// Check if the device is BUSY
		if (cam.useUsart) {
			if (cam.pUart->State == HAL_USART_STATE_BUSY_RX ||
				cam.pUart->State == HAL_USART_STATE_BUSY_TX_RX) {
					printf("USART busy\r\n");
				}
		} else {
			if (cam.pSpi->State == HAL_SPI_STATE_BUSY_RX ||
				cam.pSpi->State == HAL_SPI_STATE_BUSY_TX_RX) {
				printf("SPI busy\r\n");
			}
		}
		
		abort_data_reception(cam_id);
		return false;
	}
	printf("done\r\n");
	return true;
}

_Bool abort_data_reception(uint8_t cam_id){
	printf("Abort data reception C: %d... ",cam_id);
	HAL_StatusTypeDef status;
	// disable the reception
	CameraDevice* cam = get_camera_byID(cam_id);
	if(cam->useUsart) {
		if(cam->useDma)
			status = HAL_USART_Abort(cam->pUart);
		else
			status = HAL_USART_Abort_IT(cam->pUart);
	}
	else{
		if(cam->useDma)
			status = HAL_SPI_Abort(cam->pSpi);
		else
			status = HAL_SPI_Abort_IT(cam->pSpi);
	}
	if (status != HAL_OK) {
		return false;
	}
	HAL_Delay(10);
	// Check if the device is BUSY
    if (cam->useUsart) {
        if (cam->pUart->State == HAL_USART_STATE_BUSY_RX ||
            cam->pUart->State == HAL_USART_STATE_BUSY_TX_RX) {
				printf("USART still busy aborting\r\n");
			return false;
		}
    } else {
        if (cam->pSpi->State == HAL_SPI_STATE_BUSY_RX ||
            cam->pSpi->State == HAL_SPI_STATE_BUSY_TX_RX) {
            printf("SPI still busy aborting\r\n");
            return false;  
        }
    }
	printf("done\r\n");
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

	if(!cam->isConfigured) {
		if(configure_camera_sensor(cam_id))
		{
			cam->isConfigured = true;
		}else{
			return false;
		}
	}

	delay_us(200);

	if(TCA9548A_SelectChannel(&hi2c1, 0x70, cam->i2c_target) != HAL_OK)
		{
			printf("failed to select Camera %d channel\r\n", cam_id+1);
			return false;
		}

	bool stream_on_status= (X02C1B_stream_on(cam) < 0); // returns -1 if failed

	bool data_recp_status= start_data_reception(cam_id);

	status |= data_recp_status | stream_on_status;
	if(!status)
	{
		printf("Failed to start camera %d stream\r\n", cam_id +1);
		return false;
	}
	event_bits_enabled |= (1 << cam_id);
	cam->streaming_enabled = true;
	HAL_GPIO_WritePin(cam->gpio1_port, cam->gpio1_pin, GPIO_PIN_SET); // Set GPIO1 high

	printf("Enabled cam %d stream (%02X)\r\n", cam_id+1, event_bits_enabled);
	return true;
}

_Bool disable_camera_stream(uint8_t cam_id){
	printf("Disable C: %d... ",cam_id);
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

	bool status = true;
	status &= (X02C1B_stream_off(cam) == 0); // 0 if successful, -1 if failed
	status &= abort_data_reception(cam_id);
	if(!status)
	{
		printf("Failed to stop camera %d stream\r\n", cam_id+1);
		return false;
	}

	event_bits_enabled &= ~(1 << cam_id);

	cam->streaming_enabled = false;
	HAL_GPIO_WritePin(cam->gpio1_port, cam->gpio1_pin, GPIO_PIN_RESET); // Set GPIO1 low
	printf("done\r\n");
	return true;
}

_Bool toggle_camera_stream(uint8_t cam_id){
	_Bool status = false;
    bool enabled = (event_bits_enabled & (1 << cam_id)) != 0;
	if(enabled) status = disable_camera_stream(cam_id);
	else status = enable_camera_stream(cam_id);

	return status;
}
/* -------- END HISTOGRAM TRANSFER FUNCTIONS -------- */

/* -------- BEGIN CAMERA POWER TOGGLE FUNCTIONS -------- */
_Bool enable_camera_power(uint8_t cam_id){
	if(cam_id < 0 || cam_id >= CAMERA_COUNT)
	{
		printf("Enable Power for Camera %d Failed\r\n", cam_id+1);
		return false;
	}

	CameraDevice *cam = get_camera_byID(cam_id);

	HAL_GPIO_WritePin(cam->power_port, cam->power_pin, GPIO_PIN_SET); // Set power pin high
	cam->isPowered = true;

	printf("Enabled Power for Camera %d\r\n", cam_id+1);
	return true;
}

_Bool disable_camera_power(uint8_t cam_id){
	if(cam_id < 0 || cam_id >= CAMERA_COUNT)
	{
		printf("Disable Power for Camera %d Failed\r\n", cam_id+1);
		return false;
	}

	CameraDevice *cam = get_camera_byID(cam_id);

	HAL_GPIO_WritePin(cam->power_port, cam->power_pin, GPIO_PIN_RESET); // Set power pin low
	cam->isPowered = false;
	cam->isProgrammed = false; // Clear programmed status when power is off
	cam->isConfigured = false; // Clear configured status when power is off
	cam->streaming_enabled = false; // Clear streaming status when power is off

	printf("Disabled Power for Camera %d\r\n", cam_id+1);
	return true;
}

_Bool get_camera_power_status(uint8_t cam_id){
	if(cam_id < 0 || cam_id >= CAMERA_COUNT)
	{
		printf("Get Power Status for Camera %d Failed\r\n", cam_id+1);
		return false;
	}

	CameraDevice *cam = get_camera_byID(cam_id);
	return cam->isPowered;
}
