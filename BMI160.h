#ifndef BMI160_H_
#define BMI160_H_

/* Includes */
#include "my_RTOS_I2C_driver.h"


/* Address */
#define BMI160_I2C_SLAVE_ADDR		(0x68)

/* Registers */
#define BMI160_CMD_REG_ADDR			(0x7E)
#define BMI160_PMU_STATUS_REG_ADDR	(0x03)

/* Configuration values */
#define BMI160_CMD_ACC_INIT			(0x11)
#define BMI160_CMD_GYR_INIT			(0x15)
#define BMI160_CMD_DELAY			(100U)

#define BMI160_PMU_MASK				(0x3C)
#define BMI160_PMU_NORMAL_STATUS	(0x14)




#endif /* BMI160_H_ */
