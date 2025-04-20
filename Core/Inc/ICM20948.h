#ifndef ICM20948_H_
#define ICM20948_H_


#define ICM20948_ADDR 				(0x68)

#define ICM20948_REG_BANK_SEL       (0x7F)
#define ICM20948_USER_BANK_0        (0x00)
#define ICM20948_USER_BANK_2        (0x20)  // for accel/gyro
#define ICM20948_USER_BANK_3        (0x30)

#define ICM20948_LP_CONFIG 			(0x05)
#define ICM20948_PWR_MGMT_1 		(0x06)
#define ICM20948_PWR_MGMT_2			(0x07)
#define ICM20948_USER_CTRL        	(0x03)

// IMU Sensor Specific
#define ICM20948_WHO_AM_I_REG 		(0x00)  // WHO_AM_I register address
#define ICM20948_EXPECTED_ID 		(0xEA)   // Expected device ID

#define ICM20948_TEMP_OUT_H         (0x39)
#define ICM20948_TEMP_OUT_L         (0x3A)

uint8_t ICM_WHOAMI(void);
uint8_t ICM_Init(void);
float ICM_ReadTemperature(void);
void ICM_DumpRegisters(void);

#endif /* ICM20948_H_ */
