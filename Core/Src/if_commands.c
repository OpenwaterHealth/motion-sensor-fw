/*
 * if_commands.c
 *
 *  Created on: Sep 30, 2024
 *      Author: GeorgeVigelette
 */

#include "main.h"
#include "if_commands.h"
#include "common.h"
#include "jsmn.h"
#include "i2c_master.h"
#include "i2c_protocol.h"
#include "ICM20948.h"
#include "0X02C1B.h"

#include <stdio.h>
#include <string.h>

extern uint8_t FIRMWARE_VERSION_DATA[3];
static uint32_t id_words[3] = {0};
static uint8_t camera_status[8] = {0};
static float cam_temp;
volatile float imu_temp = 0;
static ICM_Axis3D accel;
static ICM_Axis3D gyro;
extern uint32_t imu_frame_counter;

static void process_basic_command(UartPacket *uartResp, UartPacket cmd)
{
	CameraDevice* pCam = get_active_cam();

	switch (cmd.command)
	{
	case OW_CMD_NOP:
		uartResp->command = OW_CMD_NOP;
		uartResp->packet_type = OW_RESP;
		break;
	case OW_CMD_PING:
		uartResp->command = OW_CMD_PING;
		uartResp->packet_type = OW_RESP;
		break;
	case OW_CMD_VERSION:
		uartResp->command = OW_CMD_VERSION;
		uartResp->packet_type = OW_RESP;
		uartResp->data_len = sizeof(FIRMWARE_VERSION_DATA);
		uartResp->data = FIRMWARE_VERSION_DATA;
		break;
	case OW_CMD_HWID:
		uartResp->command = OW_CMD_HWID;
		uartResp->packet_type = OW_RESP;
		id_words[0] = HAL_GetUIDw0();
		id_words[1] = HAL_GetUIDw1();
		id_words[2] = HAL_GetUIDw2();
		uartResp->data_len = 16;
		uartResp->data = (uint8_t *)&id_words;
		break;
	case OW_CMD_ECHO:
		// exact copy
		uartResp->command = OW_CMD_ECHO;
		uartResp->packet_type = OW_RESP;
		uartResp->data_len = cmd.data_len;
		uartResp->data = cmd.data;
		break;
	case OW_CMD_TOGGLE_LED:
		// printf("Toggle LED\r\n");
		uartResp->command = OW_CMD_TOGGLE_LED;
		uartResp->packet_type = OW_RESP;
		HAL_GPIO_TogglePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin);
		break;
	case OW_CMD_I2C_BROADCAST:
		printf("Broadcasting I2C on all channels\r\n");
		uartResp->command = OW_CMD_I2C_BROADCAST;
		uartResp->packet_type = OW_RESP;
		TCA9548A_SelectBroadcast(pCam->pI2c, 0x70);
		break;
	case OW_TOGGLE_CAMERA_STREAM:
		uartResp->command = OW_TOGGLE_CAMERA_STREAM;
		uartResp->packet_type = OW_RESP;
		uint8_t status = 0;

		for (uint8_t i = 0; i < 8; i++) {
	        if ((cmd.addr >> i) & 0x01) {
				if(cmd.reserved == 1)
					status |= (enable_camera_stream(i)<<i);
				else {
					status |= (disable_camera_stream(i)<<i);
				}
	        }
	    }
		if(status != cmd.addr) // if the status bits are not true for all the cameras addressed, error
		{
			uartResp->packet_type = OW_ERROR;
			printf("Failed to %d on mask %02X\r\n", cmd.reserved, status);

		}
		else uartResp->packet_type = OW_ACK;
		break;
	case OW_CMD_RESET:
		uartResp->command = OW_CMD_RESET;
		uartResp->packet_type = OW_RESP;
		// softreset
		break;
	default:
		uartResp->data_len = 0;
		uartResp->packet_type = OW_UNKNOWN;
		break;
	}
}

extern uint8_t bitstream_buffer[];
extern uint32_t bitstream_len;

uint8_t* ptrBitstream;

void I2C_DisableEnableReset(I2C_HandleTypeDef *hi2c)
{
    // Step 1: Disable the I2C peripheral
    __HAL_RCC_I2C1_CLK_DISABLE(); // Replace I2C1 with your I2C instance
    HAL_I2C_DeInit(hi2c);         // De-initialize the I2C to reset its state

    // Step 2: Add a small delay for safety
    HAL_Delay(10);

    // Step 3: Re-enable the I2C peripheral
    __HAL_RCC_I2C1_CLK_ENABLE(); // Re-enable the I2C clock
    HAL_I2C_Init(hi2c);          // Reinitialize the I2C

    // Optional: Verify the I2C is ready
    if (HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY)
    {
        // Handle error, e.g., log a message or reset the microcontroller
        // printf("I2C Reset Failed\n");
    }
}

static void process_fpga_commands(UartPacket *uartResp, UartPacket cmd)
{
	CameraDevice* pCam = get_active_cam();

	switch (cmd.command)
	{
	case OW_FPGA_ON:
		uartResp->command = OW_FPGA_ON;
		uartResp->packet_type = OW_RESP;
	    for (uint8_t i = 0; i < 8; i++) {
	        if ((cmd.addr >> i) & 0x01) {
	    		if(!enable_fpga(i))
	    		{
	    			uartResp->packet_type = OW_ERROR;
	    			printf("Failed to enable FPGA on camera %d\r\n", i);
	    		}
	        }
	    }
		break;
	case OW_FPGA_OFF:
		uartResp->command = OW_FPGA_OFF;
		uartResp->packet_type = OW_RESP;
	    for (uint8_t i = 0; i < 8; i++) {
	        if ((cmd.addr >> i) & 0x01) {
	    		if(!disable_fpga(i))
	    		{
	    			uartResp->packet_type = OW_ERROR;
	    			printf("Failed to disable FPGA on camera %d\r\n", i);
	    		}
	        }
	    }
		break;
	case OW_FPGA_ACTIVATE:
		uartResp->command = OW_FPGA_ACTIVATE;
		uartResp->packet_type = OW_RESP;
	    for (uint8_t i = 0; i < 8; i++) {
	        if ((cmd.addr >> i) & 0x01) {
	    		if(!activate_fpga(i))
	    		{
	    			uartResp->packet_type = OW_ERROR;
	    			printf("Failed to Activate FPGA on camera %d\r\n", i);
	    		}
	        }
	    }
		break;
	case OW_FPGA_ID:
		uartResp->command = OW_FPGA_ID;
		uartResp->packet_type = OW_RESP;
	    for (uint8_t i = 0; i < 8; i++) {
	        if ((cmd.addr >> i) & 0x01) {
	    		if(!verify_fpga(i))
	    		{
	    			uartResp->packet_type = OW_ERROR;
	    			printf("Failed to Activate FPGA on camera %d\r\n", i);
	    		}
	        }
	    }
		break;
	case OW_FPGA_ENTER_SRAM_PROG:
		uartResp->command = OW_FPGA_ENTER_SRAM_PROG;
		uartResp->packet_type = OW_RESP;
	    for (uint8_t i = 0; i < 8; i++) {
	        if ((cmd.addr >> i) & 0x01) {
	    		if(!enter_sram_prog_fpga(i))
	    		{
	    			uartResp->packet_type = OW_ERROR;
	    			printf("Failed to Activate FPGA on camera %d\r\n", i);
	    		}
	        }
	    }
		break;
	case OW_FPGA_EXIT_SRAM_PROG:
		uartResp->command = OW_FPGA_EXIT_SRAM_PROG;
		uartResp->packet_type = OW_RESP;
	    for (uint8_t i = 0; i < 8; i++) {
	        if ((cmd.addr >> i) & 0x01) {
	    		if(!exit_sram_prog_fpga(i))
	    		{
	    			uartResp->packet_type = OW_ERROR;
	    			printf("Failed to Activate FPGA on camera %d\r\n", i);
	    		}
	        }
	    }
		break;
	case OW_FPGA_ERASE_SRAM:
		uartResp->command = OW_FPGA_ERASE_SRAM;
		uartResp->packet_type = OW_RESP;
	    for (uint8_t i = 0; i < 8; i++) {
	        if ((cmd.addr >> i) & 0x01) {
	    		if(!erase_sram_fpga(i))
	    		{
	    			uartResp->packet_type = OW_ERROR;
	    			printf("Failed to Activate FPGA on camera %d\r\n", i);
	    		}
	        }
	    }
		break;
	case OW_FPGA_PROG_SRAM:
		uartResp->command = OW_FPGA_PROG_SRAM;
		uartResp->packet_type = OW_RESP;
		// TODO: Adde parameter to force update currently defaults to false
	    for (uint8_t i = 0; i < 8; i++) {
	        if ((cmd.addr >> i) & 0x01) {
	        	_Bool func_ret = false;

	        	if(cmd.reserved == 1) {
	        		func_ret = program_fpga(i, false);
	        	} else {
	        		func_ret = program_sram_fpga(i, true, 0, 0, false);
	        	}
	    		if(!func_ret)
	    		{
	    			uartResp->packet_type = OW_ERROR;
	    			printf("Failed to Program FPGA on camera %d\r\n", i);
	    		}
	        }
	    }
		break;
	case OW_FPGA_USERCODE:
		uartResp->command = OW_FPGA_USERCODE;
		uartResp->packet_type = OW_RESP;
	    for (uint8_t i = 0; i < 8; i++) {
	        if ((cmd.addr >> i) & 0x01) {
	        	read_usercode_fpga(i);
	        }
	    }
		break;
	case OW_FPGA_BITSTREAM:
		uartResp->command = OW_FPGA_BITSTREAM;
		uartResp->packet_type = OW_RESP;
		if(cmd.reserved == 0){
			if(cmd.addr == 0){
				// first block
				memset(bitstream_buffer, 0, MAX_BITSTREAM_SIZE);
				ptrBitstream = (uint8_t*)(bitstream_buffer + 4);
				bitstream_len = 0;
			}
			memcpy(ptrBitstream, cmd.data, cmd.data_len);  // Copy data
			ptrBitstream += cmd.data_len;
			bitstream_len+=cmd.data_len;
		}else{
			uint16_t calc_crc = util_crc16((uint8_t*)(bitstream_buffer + 4), bitstream_len);
			uint16_t crc_valid = ((uint16_t)cmd.data[0] << 8) | cmd.data[1];
			// printf("BITSTREAM Size: %ld bytes CRC: 0x%04X\r\n",bitstream_len, calc_crc);
			if(crc_valid != calc_crc) {
    			uartResp->packet_type = OW_ERROR;
    			printf("Failed crc check\r\n");
			}
		}
		break;
	case OW_FPGA_STATUS:
		uartResp->command = OW_FPGA_STATUS;
		uartResp->packet_type = OW_RESP;
	    for (uint8_t i = 0; i < 8; i++) {
	        if ((cmd.addr >> i) & 0x01) {
	    		read_status_fpga(i);
	        }
	    }
		break;
	case OW_FPGA_RESET:
		uartResp->command = OW_FPGA_RESET;
		uartResp->packet_type = OW_RESP;
	    for (uint8_t i = 0; i < 8; i++) {
	        if ((cmd.addr >> i) & 0x01) {
	        	if(!reset_camera(i))
	        	{
	    			uartResp->packet_type = OW_ERROR;
	    			printf("Failed to reset camera %d\r\n", i);

	        	}
	        }
	    }

		// I2C_DisableEnableReset(pCam->pI2c);

		break;
	case OW_FPGA_SOFT_RESET:
		uartResp->command = OW_FPGA_SOFT_RESET;
		uartResp->packet_type = OW_RESP;
		//fpga_soft_reset(pCam);
		if(pCam->useUsart){
			// method 1: clear the rxfifo
//			cam.pUart->Instance->RQR |= USART_RQR_RXFRQ;
			// method 2: diable and reenable the usart
			pCam->pUart->Instance->CR1 &= ~USART_CR1_UE; // Disable USART
			pCam->pUart->Instance->CR1 |= USART_CR1_UE;
			printf("Usart buffer reset\r\n");
		}
		break;
	default:
		uartResp->data_len = 0;
		uartResp->packet_type = OW_UNKNOWN;
		// uartResp.data = (uint8_t*)&cmd.tag;
		break;
	}
}


static void process_imu_commands(UartPacket *uartResp, UartPacket cmd)
{
	switch (cmd.command)
	{
	case OW_IMU_GET_TEMP:
		// printf("OW_IMU_GET_TEMP\r\n");
		uartResp->command = OW_IMU_GET_TEMP;
		uartResp->packet_type = OW_RESP;
		imu_temp = ICM_ReadTemperature();
		uartResp->data_len = 4;
		uartResp->data = (uint8_t *)&imu_temp;
		break;
	case OW_IMU_GET_ACCEL:
		uartResp->command = OW_IMU_GET_ACCEL;
		uartResp->packet_type = OW_RESP;
		if(ICM_ReadAccel(&accel) != HAL_OK)
		{
			uartResp->packet_type = OW_ERROR;
		    uartResp->data_len = 0;
		    uartResp->data = NULL;
			printf("Failed to read accelerometer data\r\n");
		}else{
			uartResp->data_len = sizeof(accel);
			uartResp->data = (uint8_t *)&accel;
	        // printf("Accelerometer size: %d data: X=%d, Y=%d, Z=%d\r\n", sizeof(accel), accel.x, accel.y, accel.z);
		}
		break;
	case OW_IMU_GET_GYRO:
		uartResp->command = OW_IMU_GET_GYRO;
		uartResp->packet_type = OW_RESP;
		if(ICM_ReadGyro(&gyro) != HAL_OK)
		{
			uartResp->packet_type = OW_ERROR;
		    uartResp->data_len = 0;
		    uartResp->data = NULL;
			printf("Failed to read gyroscope data\r\n");
		}else{
			uartResp->data_len = sizeof(gyro);
			uartResp->data = (uint8_t *)&gyro;
	        // printf("Gyroscope size: %d data: X=%d, Y=%d, Z=%d\r\n", sizeof(gyro), gyro.x, gyro.y, gyro.z);
		}
		break;
	case OW_IMU_ON:
		uartResp->command = OW_IMU_ON;
		uartResp->packet_type = OW_RESP;
	    uartResp->data_len = 0;
	    uartResp->data = NULL;
	    imu_frame_counter = 0;
		if(HAL_TIM_Base_Start_IT(&IMU_TIMER)!= HAL_OK)
		{
			uartResp->packet_type = OW_ERROR;
			printf("Failed to turn on IMU data\r\n");
		}
		break;
	case OW_IMU_OFF:
		uartResp->command = OW_IMU_OFF;
		uartResp->packet_type = OW_RESP;
	    uartResp->data_len = 0;
	    uartResp->data = NULL;
		if(HAL_TIM_Base_Stop_IT(&IMU_TIMER)!= HAL_OK)
		{
			uartResp->packet_type = OW_ERROR;
			printf("Failed to turn off IMU data\r\n");
		}
		break;
	default:
		uartResp->data_len = 0;
		uartResp->packet_type = OW_UNKNOWN;
		break;
	}
}

static void process_camera_commands(UartPacket *uartResp, UartPacket cmd)
{
	CameraDevice* pCam = get_active_cam();
	int result = 0;
	switch (cmd.command)
	{
	case OW_CAMERA_SCAN:
		// printf("Reading Camera %d ID\r\n",pCam->id+1);
		uartResp->command = OW_CAMERA_SCAN;
		uartResp->packet_type = OW_RESP;
		if(X02C1B_detect(pCam)){
			// error
			printf("Failed Reading Camera %d ID\r\n",pCam->id+1);
			uartResp->packet_type = OW_ERROR;
		}
		break;
	case OW_CAMERA_ON:
		// printf("Setting Camera %d Stream on\r\n",pCam->id+1);
		uartResp->command = OW_CAMERA_ON;
		uartResp->packet_type = OW_RESP;
		if(X02C1B_stream_on(pCam)){
			// error
			printf("Failed Setting Camera %d Stream on\r\n",pCam->id+1);
			uartResp->packet_type = OW_ERROR;
		}
		break;
	case OW_CAMERA_SET_CONFIG:
		uartResp->command = OW_CAMERA_SET_CONFIG;
		uartResp->packet_type = OW_RESP;
	    for (uint8_t i = 0; i < 8; i++) {
	        if ((cmd.addr >> i) & 0x01) {
	        	if(!configure_camera_sensor(i))
	        	{
	    			uartResp->packet_type = OW_ERROR;
	    			printf("Failed set registers for camera %d\r\n", i);

	        	}
	        }
	    }
		break;
	case OW_CAMERA_SINGLE_HISTOGRAM:
		// printf("Capture single histogram frame\r\n");
		uartResp->command = OW_CAMERA_SINGLE_HISTOGRAM;
		uartResp->packet_type = OW_RESP;
	    for (uint8_t i = 0; i < 8; i++) {
	        if ((cmd.addr >> i) & 0x01) {
	        	if(!capture_single_histogram(i))
	        	{
	    			uartResp->packet_type = OW_ERROR;
	    			printf("Failed capture histo for camera %d\r\n", i);

	        	}
	        }
	    }
		break;
	case OW_CAMERA_GET_HISTOGRAM:
		// printf("Capture single histogram frame\r\n");
		uartResp->command = OW_CAMERA_GET_HISTOGRAM;
		uartResp->packet_type = OW_RESP;
		uartResp->addr = cmd.addr;
		uartResp->reserved = 0;
	    for (uint8_t i = 0; i < 8; i++) {
	        if ((cmd.addr >> i) & 0x01) {
	        	if(!get_single_histogram(i, uartResp->data, &uartResp->data_len))
	        	{
	        		uartResp->reserved &= ~(1 << i);
	    			uartResp->packet_type = OW_ERROR;
	    			printf("Failed capture histo for camera %d\r\n", i);

	        	} else {
	        		uartResp->reserved |= (1 << i);
	        	}
	        	// printf("C: %d F:%d L:%d\r\n", i, uartResp->id, uartResp->data_len);
	        }
	    }
		break;
	case OW_CAMERA_SET_TESTPATTERN:
		if(cmd.data_len != 1){
			uartResp->packet_type = OW_ERROR;
			printf("Invalid data length for test pattern\r\n");
			break;
		}
		uint8_t test_pattern = cmd.data[0];
		uartResp->command = OW_CAMERA_SET_TESTPATTERN;
		uartResp->packet_type = OW_RESP;
	    for (uint8_t i = 0; i < 8; i++) {
	        if ((cmd.addr >> i) & 0x01) {
	        	if(!configure_camera_testpattern(i,test_pattern))
	        	{
	    			uartResp->packet_type = OW_ERROR;
	    			printf("Failed set camera test pattern for camera %d\r\n", i);

	        	}
	        }
	    }
		break;
	case OW_CAMERA_OFF:
		// printf("Setting Camera %d Stream off\r\n",pCam->id+1);
		uartResp->command = OW_CAMERA_OFF;
		uartResp->packet_type = OW_RESP;
		if(X02C1B_stream_off(pCam)<0){
			// error
			printf("Failed Setting Camera %d Stream off\r\n",pCam->id+1);
			uartResp->packet_type = OW_ERROR;
		}
		break;
	case OW_CAMERA_STATUS:
		// printf("Camera status 0x%02X\r\n", cmd.addr);
		uartResp->command = OW_CAMERA_STATUS;
		uartResp->packet_type = OW_RESP;


	    for (uint8_t i = 0; i < 8; i++) {
	        if ((cmd.addr >> i) & 0x01) {
	        	// update camera status requested
	        	camera_status[i] = get_camera_status(i);
	        }
	    }

	    uartResp->data = camera_status;
	    uartResp->data_len = 8;
		break;
	case OW_CAMERA_RESET:
		// printf("Camera %d Sensor Reset\r\n",pCam->id+1);
		uartResp->command = OW_CAMERA_RESET;
		uartResp->packet_type = OW_RESP;
		if(X02C1B_soft_reset(pCam)<0){
			// error
			printf("Failed Camera %d Sensor Reset\r\n",pCam->id+1);
			uartResp->packet_type = OW_ERROR;
		}
		break;
	case OW_CAMERA_FSIN:
		// printf("Camera FSIN %s\r\n", cmd.reserved?"Enable":"Disable");
		uartResp->command = OW_CAMERA_FSIN;
		uartResp->packet_type = OW_RESP;
		if(cmd.reserved == 0){
			result = X02C1B_fsin_off();

			//TODO(fix this garbage, this resets the usart at the finish of a frame. this should be done more gracefully))
			for(int i = 0;i<CAMERA_COUNT; i++){
				CameraDevice *pCam = get_camera_byID(i);
				if(pCam->useUsart){
					pCam->pUart->Instance->CR1 &= ~USART_CR1_UE; // Disable USART
					pCam->pUart->Instance->CR1 |= USART_CR1_UE;
				}
			}
		} else {
			result = X02C1B_fsin_on();
		}

		if(result != 0){
			// error
			printf("Failed Camera FSIN %s\r\n", cmd.reserved?"Enable":"Disable");
			uartResp->packet_type = OW_ERROR;
		}
		break;
	case OW_CAMERA_SWITCH:
		uint8_t channel = cmd.data[0];
		uartResp->command = OW_CAMERA_SWITCH;
		uartResp->packet_type = OW_RESP;
		// printf("Switching to camera %d\r\n",channel+1);
        TCA9548A_SelectChannel(pCam->pI2c, 0x70, channel);
        set_active_camera(channel);
		break;
	case OW_CAMERA_READ_TEMP:
		// printf("Reading Camera %d Temp\r\n",pCam->id+1);
		uartResp->command = OW_CAMERA_READ_TEMP;
		uartResp->packet_type = OW_RESP;

		if(TCA9548A_SelectChannel(&hi2c1, 0x70, pCam->i2c_target) != HAL_OK)
		{
			printf("failed to select Camera %d channel\r\n", pCam->id + 1);
			uartResp->packet_type = OW_ERROR;
	        uartResp->data_len = 0;
	        uartResp->data = NULL; // No valid data to send
		} else {
			cam_temp = X02C1B_read_temp(pCam);
			if(cam_temp<0){
				// error
				printf("Failed Reading Camera %d Temp\r\n",pCam->id+1);
				uartResp->packet_type = OW_ERROR;
				uartResp->data_len = 0;
				uartResp->data = NULL; // No valid data to send
			}else{
				uartResp->data_len = sizeof(cam_temp);
				uartResp->data = (uint8_t *)&cam_temp; // Point to the static temp variable
			}
		}
		break;
	case OW_CAMERA_FSIN_EXTERNAL:
		// printf("Enabling FSIN_EXT...\r\n");
		uartResp->command = OW_CAMERA_FSIN_EXTERNAL;
		uartResp->packet_type = OW_RESP;
		if(cmd.reserved == 0){
			result = X02C1B_FSIN_EXT_disable();
		} else {
			result = X02C1B_FSIN_EXT_enable();
		}

		if(result != 0){
			// error
			printf("Failed Enabling FSIN_EXT...\r\n");
			uartResp->packet_type = OW_ERROR;
		}
		X02C1B_FSIN_EXT_enable();
		break;
	default:
		uartResp->data_len = 0;
		uartResp->command = cmd.command;
		uartResp->packet_type = OW_UNKNOWN;
		break;
	}

}

#if 0
static void JSON_ProcessCommand(UartPacket *uartResp, UartPacket cmd)
{
	// json parser
    jsmn_parser parser;
    parser.size = sizeof(parser);
    jsmn_init(&parser, NULL);
    jsmntok_t t[16];
    jsmnerr_t ret = jsmn_parse(&parser, (char *)cmd.data, cmd.data_len, t,
				 sizeof(t) / sizeof(t[0]), NULL);
    // printf("Found %d Tokens\r\n", ret);
	switch (cmd.command)
	{
	case OW_CMD_NOP:
		uartResp->command = OW_CMD_NOP;
		break;
	case OW_CMD_ECHO:
		// exact copy
		uartResp->id = cmd.id;
		uartResp->packet_type = cmd.packet_type;
		uartResp->command = cmd.command;
		uartResp->data_len = cmd.data_len;
		uartResp->data = cmd.data;
		break;
	default:
		uartResp->data_len = 0;
		uartResp->packet_type = OW_UNKNOWN;
		break;
	}
}
#endif

static void print_uart_packet(const UartPacket* packet) __attribute__((unused));
static void print_uart_packet(const UartPacket* packet) {
    printf("ID: 0x%04X\r\n", packet->id);
    printf("Packet Type: 0x%02X\r\n", packet->packet_type);
    printf("Command: 0x%02X\r\n", packet->command);
    printf("Data Length: %d\r\n", packet->data_len);
    printf("CRC: 0x%04X\r\n", packet->crc);
    printf("Data: ");
    for (int i = 0; i < packet->data_len; i++) {
        printf("0x%02X ", packet->data[i]);
    }
    printf("\r\n");
}

UartPacket process_if_command(UartPacket cmd)
{
	UartPacket uartResp;
	I2C_TX_Packet i2c_packet;
	CameraDevice* pCam = get_active_cam();
	uartResp.id = cmd.id;
	uartResp.packet_type = OW_RESP;
	uartResp.data_len = 0;
	uartResp.data = 0;
	switch (cmd.packet_type)
	{
	case OW_JSON:
		// JSON_ProcessCommand(&uartResp, cmd);
		break;
	case OW_CMD:
		process_basic_command(&uartResp, cmd);
		break;
	case OW_FPGA:
		process_fpga_commands(&uartResp, cmd);
		break;
	case OW_CAMERA:
		process_camera_commands(&uartResp, cmd);
		break;
	case OW_IMU:
		process_imu_commands(&uartResp, cmd);
		break;
	case OW_I2C_PASSTHRU:

//		print_uart_packet(&cmd);

        // printBuffer(cmd.data, 10);
		i2c_packet_fromBuffer(cmd.data, &i2c_packet);
		// i2c_tx_packet_print(&i2c_packet);

		HAL_Delay(20);

		send_buffer_to_slave(pCam->pI2c, cmd.command, cmd.data, cmd.data_len);

		break;
	default:
		uartResp.data_len = 0;
		uartResp.packet_type = OW_UNKNOWN;
		// uartResp.data = (uint8_t*)&cmd.tag;
		break;
	}

	return uartResp;

}
