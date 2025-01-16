/*
 * lps22hh.h
 *
 *  Created on: Jan 15, 2025
 *      Author: marek
 */

#ifndef INC_LPS22HH_H_
#define INC_LPS22HH_H_

#include "stdio.h"
#include "i2c.h"


#define MAX30101_INT_STATUS_1            0x00
#define MAX30101_INT_STATUS_2            0x01
#define MAX30101_INT_ENABLE_1            0x02
#define MAX30101_INT_ENABLE_2            0x03
#define MAX30101_FIFO_WR_PTR             0x04
#define MAX30101_FIFO_OVF_CNT            0x05
#define MAX30101_FIFO_RD_PTR             0x06
#define MAX30101_FIFO_DATA               0x07
#define MAX30101_FIFO_CFG                0x08
#define MAX30101_MODE_CFG                0x09
#define MAX30101_SPO2_CFG                0x0A
#define MAX30101_LED1_PA                 0x0C
#define MAX30101_LED2_PA                 0x0D
#define MAX30101_LED3_PA                 0x0E
#define MAX30101_LED4_PA                 0x0F
#define MAX30101_MULTILED_MODE_1         0x11
#define MAX30101_MULTILED_MODE_2         0x12
#define MAX30101_DIE_TEMP_INT            0x1F
#define MAX30101_DIE_TEMP_FRACTION       0x1F
#define MAX30101_DIE_TEMP_CFG            0x20

#define MAX30101_REV_ID                  0xFE
#define MAX30101_REG_PART_ID             0xFF

#define MAX30101_PART_ID                 0x15

#define MAX30101_SET_DEV_ADDR            0x57
#define MAX30101_SET_DEV_ADDR_W          0xAE
#define MAX30101_SET_DEV_ADDR_R          0xAF



#define TIMEOUT                 100





#define LPS25HB_ADDR			0xBA
#define LPS_PART_ID				0xBD

#define LPS25HB_WHO_AM_I 		0x0F
#define LPS25HB_CTRL_REG1 		0x20
#define LPS25HB_CTRL_REG2 		0x21
#define LPS25HB_CTRL_REG3 		0x22
#define LPS25HB_CTRL_REG4 		0x23
#define LPS25HB_PRESS_OUT_XL 	0x28
#define LPS25HB_PRESS_OUT_L 	0x29
#define LPS25HB_PRESS_OUT_H 	0x2A
#define LPS25HB_TEMP_OUT_L 		0x2B
#define LPS25HB_TEMP_OUT_H 		0x2C
#define LPS25HB_FIFO_CTRL		0x2E



struct values_lps {
    float temp;
    float pressure;
};


uint8_t lps_read_reg(uint8_t reg);
void lps_write_reg(uint8_t reg, uint8_t value);
struct values_lps lps_read_val(void);
void LPS22HH_Init(void);
float apply_moving_average(float *values, int *index, int size, float new_value);

typedef struct {
    float x;  // Aktualny stan (wysokość)
    float P;  // Niepewność stanu
    float Q;  // Wariancja procesu
    float R;  // Wariancja szumu pomiaru
    float K;  // Wzmocnienie Kalmana
} KalmanFilter;

void kalman_init(KalmanFilter *kf, float initial_value, float process_variance, float measurement_variance);
float kalman_update(KalmanFilter *kf, float measurement);

#endif /* INC_LPS22HH_H_ */
