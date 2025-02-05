/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "eth.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "lps22hh.h"
#include <math.h>
#include "bno055_stm32.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch) {
	if (ch == '\n') {
		__io_putchar('\r');
	}
	HAL_UART_Transmit(&huart3, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	return 1;
}

#define FILTER_SIZE 10
float pressure_values[FILTER_SIZE] = { 0 };
int filter_index = 0;

float t = 0.0; //tempratura
float p = 0.0; //cisnienie poczatkowe - punktu zero
float p0 = 0.0; //cisnienie do obliczen
float h = 0.0; //wysokosc
float h_over_filter = 0.0;
float h_over_Kalman = 0.0;

float stable_altitude = 0.0f;
float changing_altitude = 0.0f;

#define NO_MOTION_THRESHOLD  1.0f

void I2C_Scanner(void) {
	printf("Scanning I2C bus...\r\n");
	for (uint16_t i = 0; i < 128; i++) {
		if (HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t) (i << 1), 1, HAL_MAX_DELAY)
				== HAL_OK) {
			printf("Device found at 0x%02X\r\n", i);
		}
	}
	printf("Scan complete.\r\n");
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */
	/* USER CODE BEGIN Boot_Mode_Sequence_0 */
	int32_t timeout;
	/* USER CODE END Boot_Mode_Sequence_0 */

	/* USER CODE BEGIN Boot_Mode_Sequence_1 */
	/* Wait until CPU2 boots and enters in stop mode or timeout*/
	timeout = 0xFFFF;
	while ((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0))
		;
	if (timeout < 0) {
		Error_Handler();
	}
	/* USER CODE END Boot_Mode_Sequence_1 */
	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();
	/* USER CODE BEGIN Boot_Mode_Sequence_2 */
	/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
	 HSEM notification */
	/*HW semaphore Clock enable*/
	__HAL_RCC_HSEM_CLK_ENABLE();
	/*Take HSEM */
	HAL_HSEM_FastTake(HSEM_ID_0);
	/*Release HSEM in order to notify the CPU2(CM4)*/
	HAL_HSEM_Release(HSEM_ID_0, 0);
	/* wait until CPU2 wakes up from stop mode */
	timeout = 0xFFFF;
	while ((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0))
		;
	if (timeout < 0) {
		Error_Handler();
	}
	/* USER CODE END Boot_Mode_Sequence_2 */

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_ETH_Init();
	MX_USART3_UART_Init();
	MX_USB_OTG_FS_PCD_Init();
	MX_I2C1_Init();
	MX_TIM6_Init();
	MX_I2C2_Init();
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	LPS22HH_Init();
	HAL_Delay(3000); // dla inicjalizacji, wazne jak korzystam z usredniania w ukladzie - by sam mogl przeliczy inicjalne probki i wpisac do fifo itp

	struct values_lps result_lps = lps_read_val();
	p0 = result_lps.pressure;

	// wypełnienie tablicy filtra frdniej kraczacej poczatkowymi danymi
	for (int i = 0; i < FILTER_SIZE; i++) {
		pressure_values[i] = p0; // Wypełnienie tablicy pierwszym pomiarem
	}
	filter_index = 0; // Reset indeksu
	KalmanFilter kf;
	// Inicjalizacja filtru Kalmana
	//pierwsza stała w tym przypadku to przybliżenie niepewności dotyczącej modelu procesu, czyli tego, jak zmienia się stan między kolejnymi iteracjami.
	//druga stała w tym przypadku to szacowanie niepewności w odczytach czujnika (np. szumu w pomiarze ciśnienia atmosferycznego).
	kalman_init(&kf, p0, 1.0f, 1.0f);// Wariancja procesu i pomiaru (dopasuj do aplikacji)

	//	//ACC init
	bno055_assignI2C(&hi2c2);
	bno055_setup();
	bno055_setOperationModeNDOF();
	HAL_Delay(1000);

	while (1) {
		// wersja podstawowa -------------------------------------------------------------------same odczyty itp-----------------------------------

//	  struct values_lps result_lps = lps_read_val();
//	  t = result_lps.temp;
//	  p = result_lps.pressure;
//	  //h = -29.271769 * t * log(p / p0);
//	  float filtered_pressure = apply_moving_average(pressure_values, &filter_index, FILTER_SIZE, p);
//
//	//	 Filtracja za pomocą Kalmana
//      float filtered_pressure_Kalman = kalman_update(&kf, p);
//
//	//	ponoc dokładniejszy sposob bazujacy na innym wzorze - lepiej reaguje na zmiany, większe wahania
//	  h = 44330.0 * (1.0 - pow(p / p0, 0.1903));
//	  h_over_filter = 44330.0 * (1.0 - pow(filtered_pressure / p0, 0.1903));
//	  h_over_Kalman = 44330.0 * (1.0 - pow(filtered_pressure_Kalman / p0, 0.1903));
//	  printf("T = %.1f*C\n", result_lps.temp);
//	  printf("p = %.1f hPa\n", result_lps.pressure);
//	  printf("h przed filtracją           = %.2f m\n", h);
//	  printf("h po filtracji (krocząca)   = %.2f m\n", h_over_filter);
//	  printf("h po filtracji (Kalman)     = %.2f m\n", h_over_Kalman);
//
//	  // Dane surowe z akcelerometru
//	      bno055_vector_t accel = bno055_getVectorAccelerometer();
//	      printf("Accel Raw - X: %.2f Y: %.2f Z: %.2f (m/s²)\r\n", accel.x, accel.y, accel.z);
//
//	      // Dane surowe z magnetometru
//	      bno055_vector_t mag = bno055_getVectorMagnetometer();
//	      printf("Magnetometer Raw - X: %.2f Y: %.2f Z: %.2f (µT)\r\n", mag.x, mag.y, mag.z);
//
//	      // Dane surowe z żyroskopu
//	      bno055_vector_t gyro = bno055_getVectorGyroscope();
//	      printf("Gyro Raw - X: %.2f Y: %.2f Z: %.2f (°/s)\r\n", gyro.x, gyro.y, gyro.z);
//
//	      // Kąty Eulera po fuzji danych
//	      bno055_vector_t euler = bno055_getVectorEuler();
//	      printf("Euler - Heading: %.2f Roll: %.2f Pitch: %.2f\r\n", euler.x, euler.y, euler.z);
//
//	      // Kwaterniony
//	      bno055_vector_t quat = bno055_getVectorQuaternion();
//	      printf("Quaternion - W: %.2f X: %.2f Y: %.2f Z: %.2f\r\n", quat.w, quat.x, quat.y, quat.z);
//
//
//		HAL_Delay(100);

		//Wersja z dodaniem warunków i semi fuzja danych -----------------------------------------------------------------------------------------------------------
		// 1) Odczyt barometru i obliczenie wysokości
		struct values_lps result_lps = lps_read_val();
		t = result_lps.temp;
		p = result_lps.pressure;

		float filtered_pressure = apply_moving_average(pressure_values,
				&filter_index, FILTER_SIZE, p);
		float filtered_pressure_Kalman = kalman_update(&kf, p);

		// liczenie
		h = 44330.0 * (1.0 - pow(p / p0, 0.1903));
		h_over_filter = 44330.0 * (1.0 - pow(filtered_pressure / p0, 0.1903));
		h_over_Kalman = 44330.0
				* (1.0 - pow(filtered_pressure_Kalman / p0, 0.1903));

		// 2) Sprawdzam przyspieszenie liniowe z BNO055
		bno055_vector_t linAccel = bno055_getVectorLinearAccel();
		//tu jest to przeliczenie - ktore pozniej jest brane pod uwage przy sprawdzenu ruchu - czuli oblicznay jest wypadkowy wektor
		float linAccelMagnitude = sqrtf(
				linAccel.x * linAccel.x + linAccel.y * linAccel.y
						+ linAccel.z * linAccel.z);

		// 3) Jeśli czujnik prawie w ogóle się nie porusza, to zamrażam wysokość:
		// treshold wyzwalania to #define NO_MOTION_THRESHOLD  0.05f - czyli 0.05 m/s²

		printf("Wypadkowy wektor: %f\n", linAccelMagnitude);

		if (linAccelMagnitude > NO_MOTION_THRESHOLD) {
			//h_over_Kalman = stable_altitude;
			changing_altitude = h_over_Kalman;
		} else {
			//stable_altitude = h_over_Kalman;
		}

		// 4)  wyniki
		printf("T = %.1f*C\n", t);
		printf("p = %.1f hPa\n", p);
		printf("h przed filtracją           = %.2f m\n", h);
		printf("h po filtracji (krocząca)   = %.2f m\n", h_over_filter);
		printf("h po filtracji (Kalman)     = %.2f m\n", h_over_Kalman);
		printf("h po zlaczeniu danych       = %.2f m\n", changing_altitude);
		// Dane z akcelerometru "raw" (dla podglądu)
		bno055_vector_t accel = bno055_getVectorAccelerometer();
		printf("Accel Raw - X: %.2f Y: %.2f Z: %.2f (m/s²)\r\n", accel.x,
				accel.y, accel.z);

		// Kąty Eulera po fuzji danych
		bno055_vector_t euler = bno055_getVectorEuler();
		printf("Euler - Heading: %.2f Roll: %.2f Pitch: %.2f\r\n", euler.x,
				euler.y, euler.z);

		// Kwaterniony
		bno055_vector_t quat = bno055_getVectorQuaternion();
		printf("Quaternion - W: %.2f X: %.2f Y: %.2f Z: %.2f\r\n", quat.w,
				quat.x, quat.y, quat.z);


		// Dane surowe z magnetometru
		bno055_vector_t mag = bno055_getVectorMagnetometer();
		printf("Magnetometer Raw - X: %.2f Y: %.2f Z: %.2f (µT)\r\n", mag.x,
				mag.y, mag.z);

		// Dane surowe z żyroskopu
		bno055_vector_t gyro = bno055_getVectorGyroscope();
		printf("Gyro Raw - X: %.2f Y: %.2f Z: %.2f (°/s)\r\n", gyro.x, gyro.y,
				gyro.z);

		//dla serial plota
//		printf("%.2f %.2f %.2f\r\n", accel.x, accel.y, accel.z);
		HAL_Delay(100);
	}

	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */
}
/* USER CODE END 3 */

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Supply configuration update enable
	 */
	HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
	}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 100;
	RCC_OscInitStruct.PLL.PLLP = 2;
	RCC_OscInitStruct.PLL.PLLQ = 13;
	RCC_OscInitStruct.PLL.PLLR = 2;
	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
	RCC_OscInitStruct.PLL.PLLFRACN = 0;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1
			| RCC_CLOCKTYPE_D1PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
