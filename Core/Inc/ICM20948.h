#ifndef ICM20948_H_
#define ICM20948_H_


#define ICM20948_ADDR 				(0x68)

#define ICM20948_USER_BANK_0        (0x00)
#define ICM20948_USER_BANK_2        (0x20)  // for accel/gyro
#define ICM20948_USER_BANK_3        (0x30)

#define ICM20948_USER_CTRL          (0x03)  // Bit 7 enable DMP, bit 3 reset DMP
#define ICM20948_LP_CONFIG 			(0x05)
#define ICM20948_PWR_MGMT_1 		(0x06)
#define ICM20948_PWR_MGMT_2			(0x07)
#define ICM20948_INT_PIN_CFG        (0x0F)
#define ICM20948_INT_ENABLE         (0x10)
#define ICM20948_INT_ENABLE_1	    (0x11) // Not found in MPU-9250
#define ICM20948_INT_ENABLE_2	    (0x12) // Not found in MPU-9250
#define ICM20948_INT_ENABLE_3	    (0x13) // Not found in MPU-9250
#define ICM20948_I2C_MST_STATUS     (0x17)
#define ICM20948_INT_STATUS         (0x19)
#define ICM20948_INT_STATUS_1	    (0x1A) // Not found in MPU-9250
#define ICM20948_INT_STATUS_2	    (0x1B) // Not found in MPU-9250
#define ICM20948_INT_STATUS_3	    (0x1C) // Not found in MPU-9250
#define ICM20948_DELAY_TIMEH		(0x28) // Not found in MPU-9250
#define ICM20948_DELAY_TIMEL		(0x29) // Not found in MPU-9250

// IMU Sensor Specific
#define ICM20948_WHO_AM_I_REG 		(0x00)  // WHO_AM_I register address
#define ICM20948_EXPECTED_ID 		(0xEA)   // Expected device ID

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

// Temperature Data Registers
#define ICM20948_TEMP_OUT_H         (0x39)
#define ICM20948_TEMP_OUT_L         (0x3A)

// Magnetometer Registers
#define AK09916_ADDRESS  			(0x0C)
#define WHO_AM_I_AK09916 			(0x01) // (AKA WIA2) should return 0x09
#define AK09916_EXPECTED_ID 		(0x09)   // Expected device ID
#define AK09916_ST1      			(0x10)  // data ready status bit 0
#define AK09916_XOUT_L   			(0x11)  // data
#define AK09916_XOUT_H   			(0x12)
#define AK09916_YOUT_L   			(0x13)
#define AK09916_YOUT_H   			(0x14)
#define AK09916_ZOUT_L   			(0x15)
#define AK09916_ZOUT_H   			(0x16)
#define AK09916_ST2      			(0x18)  // Data overflow bit 3 and data read error status bit 2
#define AK09916_CNTL     			(0x30)  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK09916_CNTL2    			(0x31)  // Normal (0), Reset (1)

// Gyro/Accel Config (Bank 2)
#define ICM20948_GYRO_CONFIG_1      (0x01)
#define ICM20948_ACCEL_CONFIG       (0x14)

#define ICM20948_EXT_SENS_DATA_00   (0x3B)
#define ICM20948_EXT_SENS_DATA_01   (0x3C)
#define ICM20948_EXT_SENS_DATA_02   (0x3D)
#define ICM20948_EXT_SENS_DATA_03   (0x3E)
#define ICM20948_EXT_SENS_DATA_04   (0x3F)
#define ICM20948_EXT_SENS_DATA_05   (0x40)
#define ICM20948_EXT_SENS_DATA_06   (0x41)
#define ICM20948_EXT_SENS_DATA_07   (0x42)
#define ICM20948_EXT_SENS_DATA_08   (0x43)
#define ICM20948_EXT_SENS_DATA_09   (0x44)
#define ICM20948_EXT_SENS_DATA_10   (0x45)
#define ICM20948_EXT_SENS_DATA_11   (0x46)
#define ICM20948_EXT_SENS_DATA_12   (0x47)
#define ICM20948_EXT_SENS_DATA_13   (0x48)
#define ICM20948_EXT_SENS_DATA_14   (0x49)
#define ICM20948_EXT_SENS_DATA_15   (0x4A)
#define ICM20948_EXT_SENS_DATA_16   (0x4B)
#define ICM20948_EXT_SENS_DATA_17   (0x4C)
#define ICM20948_EXT_SENS_DATA_18   (0x4D)
#define ICM20948_EXT_SENS_DATA_19   (0x4E)
#define ICM20948_EXT_SENS_DATA_20   (0x4F)
#define ICM20948_EXT_SENS_DATA_21   (0x50)
#define ICM20948_EXT_SENS_DATA_22   (0x51)
#define ICM20948_EXT_SENS_DATA_23   (0x52)
#define ICM20948_FIFO_EN_1          (0x66)
#define ICM20948_FIFO_EN_2          (0x67) // Not found in MPU-9250
#define ICM20948_FIFO_RST		    (0x68) // Not found in MPU-9250
#define ICM20948_FIFO_MODE		    (0x69) // Not found in MPU-9250
#define ICM20948_FIFO_COUNTH        (0x70)
#define ICM20948_FIFO_COUNTL        (0x71)
#define ICM20948_FIFO_R_W           (0x72)
#define ICM20948_DATA_RDY_STATUS	(0x74) // Not found in MPU-9250
#define ICM20948_FIFO_CFG		    (0x76) // Not found in MPU-9250
#define ICM20948_REG_BANK_SEL	    (0x7F) // Not found in MPU-9250

// USER BANK 3 REGISTER MAP
#define ICM20948_I2C_MST_ODR_CONFIG	(0x00) // Not found in MPU-9250
#define ICM20948_I2C_MST_CTRL       (0x01)
#define ICM20948_I2C_MST_DELAY_CTRL (0x02)
#define ICM20948_I2C_SLV0_ADDR      (0x03)
#define ICM20948_I2C_SLV0_REG       (0x04)
#define ICM20948_I2C_SLV0_CTRL      (0x05)
#define ICM20948_I2C_SLV0_DO        (0x06)
#define ICM20948_I2C_SLV1_ADDR      (0x07)
#define ICM20948_I2C_SLV1_REG       (0x08)
#define ICM20948_I2C_SLV1_CTRL      (0x09)
#define ICM20948_I2C_SLV1_DO        (0x0A)
#define ICM20948_I2C_SLV2_ADDR      (0x0B)
#define ICM20948_I2C_SLV2_REG       (0x0C)
#define ICM20948_I2C_SLV2_CTRL      (0x0D)
#define ICM20948_I2C_SLV2_DO        (0x0E)
#define ICM20948_I2C_SLV3_ADDR      (0x0F)
#define ICM20948_I2C_SLV3_REG       (0x10)
#define ICM20948_I2C_SLV3_CTRL      (0x11)
#define ICM20948_I2C_SLV3_DO        (0x12)
#define ICM20948_I2C_SLV4_ADDR      (0x13)
#define ICM20948_I2C_SLV4_REG       (0x14)
#define ICM20948_I2C_SLV4_CTRL      (0x15)
#define ICM20948_I2C_SLV4_DO        (0x16)
#define ICM20948_I2C_SLV4_DI        (0x17)

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
uint8_t ICM_ReadMag(ICM_Axis3D *mag);
void ICM_DumpRegisters(void);

#endif /* ICM20948_H_ */
