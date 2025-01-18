/*
 * lps22hh.c
 *
 *  Created on: Jan 15, 2025
 *      Author: marek
 */

#include "lps22hh.h"



uint8_t lps_read_reg(uint8_t reg)
{
	uint8_t value = 0;
	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Read(&hi2c1, LPS25HB_ADDR, reg, 1, &value, sizeof(value), HAL_MAX_DELAY);
	return value;
}

void lps_write_reg(uint8_t reg, uint8_t value)
{
	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Write(&hi2c1, LPS25HB_ADDR, reg, 1, &value, sizeof(value), HAL_MAX_DELAY);
}
struct values_lps lps_read_val(void)
{
	struct values_lps result_lps;

	int16_t temp = 0;
	int32_t pressure = 0;

    result_lps.temp = 0.0;
    result_lps.pressure = 0.0;

    if (HAL_I2C_Mem_Read(&hi2c1, LPS25HB_ADDR, LPS25HB_TEMP_OUT_L | 0x80, 1, (uint8_t*)&temp, sizeof(temp), TIMEOUT) != HAL_OK)
        Error_Handler();
    if (HAL_I2C_Mem_Read(&hi2c1, LPS25HB_ADDR, LPS25HB_PRESS_OUT_XL | 0x80, 1, (uint8_t*)&pressure, 3, TIMEOUT) != HAL_OK)
         Error_Handler();

    result_lps.temp =  (float)(42.5f + temp / 480.0f);
    result_lps.pressure =  (float)(pressure / 4096.0f);


    return result_lps;
}

void LPS22HH_Init(void) {
	  printf("Searching...\r\n");
	   uint8_t who_am_i = lps_read_reg(LPS25HB_WHO_AM_I);
	   if (who_am_i == LPS_PART_ID) {
	    printf("Found: LPS\r\n");

	    lps_write_reg(LPS25HB_CTRL_REG1,  0xC0);

		   //uśrednianie itp
			//lps_write_reg(LPS25HB_CTRL_REG2,  0x40);
			//lps_write_reg(LPS25HB_FIFO_CTRL,  0xDF);

	   } else {
	    printf("Error: (0x%02X)\r\n", who_am_i);
	   }
}

/**
 * Funkcja oblicza średnią ruchomą z pomiarów ciśnienia.
 *
 * @param values Tablica wartości ciśnienia.
 * @param index Wskaźnik na aktualny indeks w tablicy.
 * @param size Rozmiar tablicy (FILTER_SIZE).
 * @param new_value Nowa wartość ciśnienia do dodania.
 * @return Średnia ruchoma z wartości w tablicy.
 */
float apply_moving_average(float *values, int *index, int size, float new_value) {
    values[*index] = new_value;          // Dodanie nowej wartości do tablicy
    *index = (*index + 1) % size;        // Przesunięcie indeksu z zawijaniem

    float sum = 0.0f;
    for (int i = 0; i < size; i++) {     // Obliczenie sumy elementów w tablicy
        sum += values[i];
    }
    return sum / size;                   // Zwrot średniej
}

void kalman_init(KalmanFilter *kf, float initial_value, float process_variance, float measurement_variance) {
    kf->x = initial_value;   // Ustaw początkowy stan
    kf->P = 1.0f;            // Niepewność początkowa
    kf->Q = process_variance; // Wariancja procesu
    kf->R = measurement_variance; // Wariancja szumu pomiaru
    kf->K = 0.0f;            // Wzmocnienie Kalmana początkowe
}



float kalman_update(KalmanFilter *kf, float measurement) {
    // Predykcja
    kf->P += kf->Q;

    // Obliczanie wzmocnienia Kalmana
    kf->K = kf->P / (kf->P + kf->R);

    // Aktualizacja stanu
    kf->x += kf->K * (measurement - kf->x);

    // Aktualizacja niepewności
    kf->P *= (1.0f - kf->K);

    return kf->x;
}
