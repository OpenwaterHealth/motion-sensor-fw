/*
 * camera_manager.h
 *
 *  Created on: Mar 5, 2025
 *      Author: GeorgeVigelette
 */

#ifndef INC_CAMERA_MANAGER_H_
#define INC_CAMERA_MANAGER_H_
#include "main.h"

typedef struct {
	uint16_t id;
	GPIO_TypeDef * 	cresetb_port;
	uint16_t  		cresetb_pin;
	GPIO_TypeDef *	gpio0_port;
	uint16_t  		gpio0_pin;
	I2C_HandleTypeDef * pI2c;
	uint8_t  		device_address;
	bool 			useUsart; // use usart over spi
	bool 			useDma;
	SPI_HandleTypeDef * pSpi;
	USART_HandleTypeDef * pUart;
	uint16_t 		i2c_target;
	bool 			streaming_enabled;
	bool 			isProgrammed;
	bool 			isConfigured;
	uint8_t 		gain;
	uint8_t 		exposure;
	uint8_t *pRecieveHistoBuffer;
    size_t   receiveBufferSize;      // Size of the buffer
} CameraDevice;

#define CAMERA_COUNT	8
#define HISTOGRAM_DATA_SIZE	4100
#define WIDTH 1920
#define HEIGHT 1280
#define HISTOGRAM_BINS 1024
#define HISTO_TEST_PATTERN 3

#define HISTO_SOF  0xAA
#define HISTO_SOH  0xFF
#define HISTO_EOH  0xEE
#define HISTO_EOF  0xDD
#define TYPE_HISTO 0x00
#define HISTO_HEADER_SIZE 6
#define HISTO_TRAILER_SIZE 3


void init_camera_sensors(void);
CameraDevice* get_active_cam(void);
CameraDevice* set_active_camera(int id);
CameraDevice* get_camera_byID(int id);

_Bool reset_camera(uint8_t cam_id);
_Bool enable_fpga(uint8_t cam_id);
_Bool disable_fpga(uint8_t cam_id);
_Bool activate_fpga(uint8_t cam_id);
_Bool verify_fpga(uint8_t cam_id);
_Bool enter_sram_prog_fpga(uint8_t cam_id);
_Bool exit_sram_prog_fpga(uint8_t cam_id);
_Bool erase_sram_fpga(uint8_t cam_id);
_Bool program_fpga(uint8_t cam_id, _Bool force_update);
_Bool configure_camera_sensor(uint8_t cam_id);
_Bool configure_camera_testpattern(uint8_t cam_id, uint8_t test_pattern);
_Bool capture_single_histogram(uint8_t cam_id);
_Bool get_single_histogram(uint8_t cam_id, uint8_t* data, uint16_t* data_len);
_Bool start_data_reception(uint8_t cam_id);
_Bool abort_data_reception(uint8_t cam_id);
_Bool send_fake_data(void);
_Bool send_histogram_data(void);
_Bool enable_camera_stream(uint8_t cam_id);
_Bool disable_camera_stream(uint8_t cam_id);
uint8_t get_camera_status(uint8_t cam_id);


void Camera_USART_RxCpltCallback_Handler(USART_HandleTypeDef *husart);
void Camera_SPI_RxCpltCallback_Handler(SPI_HandleTypeDef *hspi);

uint32_t read_status_fpga(uint8_t cam_id);
uint32_t read_usercode_fpga(uint8_t cam_id);
_Bool program_sram_fpga(uint8_t cam_id, bool rom_bitstream, uint8_t* pData, uint32_t Data_Len, _Bool force_update);

void switch_frame_buffer(void);
uint8_t* get_active_frame_buffer(void);
uint8_t* get_inactive_frame_buffer(void);
void fill_frame_buffers(void);
_Bool toggle_camera_stream(uint8_t cam_id);
void print_active_cameras(uint8_t cameras_present);


void CAM_UART_RxCpltCallback(USART_HandleTypeDef *husart);
void CAM_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);

#endif /* INC_CAMERA_MANAGER_H_ */
