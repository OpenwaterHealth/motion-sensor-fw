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

// Accelerometer Data Registers
#define ICM20948_ACCEL_XOUT_H       (0x2D)
#define ICM20948_ACCEL_XOUT_L       (0x2E)
#define ICM20948_ACCEL_YOUT_H       (0x2F)
#define ICM20948_ACCEL_YOUT_L       (0x30)
#define ICM20948_ACCEL_ZOUT_H       (0x31)
#define ICM20948_ACCEL_ZOUT_L       (0x32)

// Gyroscope Data Registers
#define ICM20948_GYRO_XOUT_H        (0x33)
#define ICM20948_GYRO_XOUT_L        (0x34)
#define ICM20948_GYRO_YOUT_H        (0x35)
#define ICM20948_GYRO_YOUT_L        (0x36)
#define ICM20948_GYRO_ZOUT_H        (0x37)
#define ICM20948_GYRO_ZOUT_L        (0x38)

// Gyro/Accel Config (Bank 2)
#define ICM20948_GYRO_CONFIG_1      (0x01)
#define ICM20948_ACCEL_CONFIG       (0x14)

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} ICM_Axis3D;

uint8_t ICM_WHOAMI(void);
uint8_t ICM_Init(void);
float ICM_ReadTemperature(void);
uint8_t ICM_ReadAccel(ICM_Axis3D *accel);
uint8_t ICM_ReadGyro(ICM_Axis3D *gyro);
void ICM_DumpRegisters(void);

#endif /* ICM20948_H_ */
