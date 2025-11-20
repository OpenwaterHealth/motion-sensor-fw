/*
 * 0X02C1B.h
 *
 *  Created on: Oct 15, 2024
 *      Author: GeorgeVigelette
 */

#ifndef INC_0X02C1B_H_
#define INC_0X02C1B_H_

#include "main.h"
/*
 * From the datasheet, "20ms after PWDN goes low or 20ms after RESETB goes
 * high if reset is inserted after PWDN goes high, host can access sensor's
 * SCCB to initialize sensor."
 */
#define PWDN_ACTIVE_DELAY_MS	20

#define X02C1B_ADDRESS			0x36

#define X02C1B_SW_RESET			0x0103

#define X02C1B_EC_A_REG03		0x3503

#define X02C1B_TEMP_UPPER 		0x4d2a
#define X02C1B_TEMP_LOWER 		0x4d2b


#define ARRAY_SIZE(array) \
    (sizeof(array) / sizeof(*array))

struct regval_list {
	uint16_t addr;
	uint8_t data;
};

int X02C1B_soft_reset(CameraDevice *cam);
int X02C1B_stream_on(CameraDevice *cam);
int X02C1B_stream_off(CameraDevice *cam);
int X02C1B_configure_sensor(CameraDevice *cam);
int X02C1B_set_test_pattern(CameraDevice *cam, uint8_t test_pattern);
int X02C1B_detect(CameraDevice *cam);
int X02C1B_fsin_on();
int X02C1B_fsin_off();
float X02C1B_read_temp(CameraDevice *cam);
int X02C1B_FSIN_EXT_enable();
int X02C1B_FSIN_EXT_disable();

// Lattice CrossLink TraceID reading functions
// Lattice CrossLink configuration I2C address (7-bit = 0x40)
// STM32 HAL expects this left-shifted by 1 bit.
#define LATTICE_CFG_I2C_ADDR   (0x40U << 1)

// Size of the TraceID in bytes
#define LATTICE_TRACE_ID_LEN   8U

HAL_StatusTypeDef X02C1B_ReadLatticeTraceID(CameraDevice *cam, uint8_t trace_id[LATTICE_TRACE_ID_LEN]);
uint64_t X02C1B_LatticeTraceID_ToU64(const uint8_t trace_id[LATTICE_TRACE_ID_LEN]);

#endif /* INC_0X02C1B_H_ */
