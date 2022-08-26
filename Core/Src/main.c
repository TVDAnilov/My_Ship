/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"

#define min_PWM 2596
#define max_PWM 4096 - 1

#define min_PWM_angle 100
#define max_PWM_angle 250
#define middle_PWM_angle 168//175
#define min_PWM_angle 100

#define oneProcent  15
#define oneStep  0.25//0.75

#define adress_master_device 0x20
#define adress_pult 0x20
#define adress_ship 0x60

#define adress_hc_12 0xAA
#define adress_LED 0xB4
#define adress_engine 0xBE
#define adress_Servo 0xC8

#define adress_slave_index 0
#define adress_master_index 1
#define crc_adress_devices 2
#define end_array_index 50

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
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

struct {
	volatile uint8_t timerEvent;
	volatile uint8_t dataReady;
	volatile uint8_t rxReady;
	volatile uint8_t txReady;
	volatile uint8_t rxSize;
	volatile uint8_t contact;
	volatile uint8_t counterError;
	volatile uint8_t Led;
	volatile uint8_t txSize;
	volatile uint8_t isCommand;

} flagEvent;

uint8_t StickPos[4] = { };
uint8_t transmitBuff[end_array_index] = { };
uint8_t reciveBuff[end_array_index] = { };
uint8_t adress_slave_device = 0;

void cleanTxArr() {
	for (uint8_t i = 0; i < end_array_index; i++) {
		transmitBuff[i] = 0xFF; // код стоп байта
	}
	transmitBuff[adress_slave_index] = adress_ship;
	transmitBuff[adress_master_index] = adress_master_device;

	transmitBuff[crc_adress_devices] = crc(transmitBuff, 2);    //crc
																// добавить проверку на правильность адреса и устройства
}

void cleanRxArr() {
	for (uint8_t i = 0; i < end_array_index; i++) {
		reciveBuff[i] = 0xFF; // код стоп байта
	}
	flagEvent.rxSize = 0;
}

uint8_t pushArrTX(uint8_t addres_module, uint8_t *pData, uint8_t size,
		uint8_t typeOperation) {
	uint8_t i = 0;
	while (transmitBuff[i] != 0xFF) { // найти конец полезных данных, не выходя за пределы массива.
		if (i < (end_array_index - (size + 5))) { //5 = адрес модуля, тип операции, crc, байт конца команды, стоп.
			i++;
		} else {
			return 1;
		}
	}
	transmitBuff[i] = addres_module;
	i++;
	transmitBuff[i] = typeOperation;
	i++;

	for (uint8_t index = i; index < (i + size); index++) {
		transmitBuff[index] = pData[index - i];
	}

	transmitBuff[i + size] = crc(transmitBuff + (i - 2), size + 2); // crc адреса и команды   2 = смещение назад для адреса модуля и типа команды.
	transmitBuff[i + size + 1] = 0xFC;  // конец команды
	transmitBuff[i + size + 2] = crc(transmitBuff, i + size + 2); // хеш отправки
	transmitBuff[i + size + 3] = 0xFF;  // стоп байт

	i = i + (size + 3) + 1;
	return i;

}

uint8_t changeSpeed(uint8_t speed, uint8_t direct) {

	if (direct == 0) {      //на месте
		TIM2->CCR1 = 0;
		TIM2->CCR2 = 0;
	}
	if (direct == 1) {		//вперед
		TIM2->CCR1 = 0;
		if (speed < 70)
			TIM2->CCR2 = min_PWM + (oneProcent * speed); // так должно быть в идеале

		if (speed >= 90) {
			speed = 80;						//ограничение:( срабатывает защита.
			if (TIM2->CCR2 <= min_PWM) {
				TIM2->CCR2 = min_PWM;
			}
			for (uint16_t i = TIM2->CCR2; i < (min_PWM + (oneProcent * speed));
					i = i + 10) {
				TIM2->CCR2 = i;
				HAL_Delay(2);
			}
		}
	}
	if (direct == 2) {           //назад
		TIM2->CCR1 = min_PWM + (oneProcent * speed);
		TIM2->CCR2 = 0;
	}
return direct;
}

uint8_t changeAngle(uint8_t angle_pwm, uint8_t direct) {
if (direct == 0) {
	TIM3->CCR1 = middle_PWM_angle;   //90 градусов

}
if (direct == 1) {				//направо
	TIM3->CCR1 = middle_PWM_angle - (oneStep * angle_pwm * 0.9); //    0 градусов.

}
if (direct == 2) {				//налево
	TIM3->CCR1 = (oneStep * angle_pwm) + middle_PWM_angle; //  180 градусов

}
return direct;
}

void read_air(void) {
HAL_UART_Receive(&huart1, StickPos, 4, HAL_MAX_DELAY);

changeSpeed(StickPos[0], StickPos[1]);
changeAngle(StickPos[2], StickPos[3]);
}

uint8_t crc(uint8_t *pData, uint8_t size) {
uint16_t crc = 0;
for (uint8_t i = 0; i < size; i++) {
	crc = crc + (pData[i] * 44111);
}
return (uint8_t) crc;

}

void direct_Led_HELLO() {
for (uint8_t i = 0; i < 6; i++) {
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
	HAL_Delay(300);
}
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, RESET);
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, RESET);
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, RESET);
}

void direct_Led(uint8_t Red, uint8_t Green, uint8_t Blue) {
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, Red);
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, Green);
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, Blue);
}

void connect_ok(void) {
direct_Led_HELLO();
}

void parseArrRX(uint8_t *pData, uint8_t size) {

if (reciveBuff[flagEvent.rxSize - 2] == crc(reciveBuff, flagEvent.rxSize - 2)) { //хеш сумма совпала

	if (flagEvent.contact == 1) {		//если уже поздоровались
		command_work();
	}

	if (flagEvent.contact == 0) { //если это приветствие
		if (reciveBuff[3] == adress_hc_12) {
			if (reciveBuff[5] == 0xc8) {
				flagEvent.contact = 1;
				connect_ok();
			}
		}
	}

}

if (reciveBuff[flagEvent.rxSize - 2] != crc(reciveBuff, flagEvent.rxSize - 2)) {

}   //хеш сумма не совпала

}

void initShip(void) {
flagEvent.timerEvent = 0;
flagEvent.dataReady = 0;
flagEvent.rxReady = 0;
flagEvent.txReady = 0;
flagEvent.rxSize = 0;
flagEvent.contact = 0;
flagEvent.counterError = 0;
flagEvent.Led = 0;
flagEvent.txSize = 0;
flagEvent.isCommand = 0;

cleanTxArr();
cleanRxArr();

//HAL_UARTEx_ReceiveToIdle_DMA(&huart1, reciveBuff, sizeof(reciveBuff));

HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}

void command_work(void) {
uint8_t i = 0;
while (reciveBuff[i] != 0xFF) {
	if (i > end_array_index)
		break;

	switch (reciveBuff[i]) {
	case adress_hc_12:
		i++;
		break;
	case adress_LED:
		direct_Led(reciveBuff[i + 2], reciveBuff[i + 3], reciveBuff[i + 4]);
		i = i + 7;
		break;
	case adress_engine:
		changeSpeed(reciveBuff[i + 2], reciveBuff[i + 3]);
		i = i + 5;
		break;
	case adress_Servo:
		changeAngle(reciveBuff[i + 2], reciveBuff[i + 3]);
		i = i + 5;
		break;
	default:
		i++;
		break;
	}

}
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/* MCU Configuration--------------------------------------------------------*/

/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
HAL_Init();

/* USER CODE BEGIN Init */

/* USER CODE END Init */

/* Configure the system clock */
SystemClock_Config();

/* USER CODE BEGIN SysInit */

/* USER CODE END SysInit */

/* Initialize all configured peripherals */
MX_GPIO_Init();
MX_TIM2_Init();
MX_USART1_UART_Init();
MX_DMA_Init();
MX_TIM3_Init();
/* USER CODE BEGIN 2 */

initShip();

/* USER CODE END 2 */

/* Infinite loop */
/* USER CODE BEGIN WHILE */
while (1) {
	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */
	if (flagEvent.contact == 0) {
		uint8_t ok = 0xc8;

		uint8_t sizeBuff = pushArrTX(adress_hc_12, &ok, 1, 1);
		HAL_UART_Transmit(&huart1, transmitBuff, sizeBuff, 100);
		HAL_UARTEx_ReceiveToIdle(&huart1, reciveBuff, (uint16_t) 50,
				(uint8_t*) &flagEvent.rxSize, 500);
		//HAL_Delay(500);
		if (flagEvent.rxSize != 0) {
			flagEvent.rxReady = 1;
		}

		cleanTxArr();
	}
	if (flagEvent.rxReady == 1) {
		flagEvent.rxReady = 0;
		parseArrRX(reciveBuff, flagEvent.rxSize);
		cleanRxArr();
		//HAL_UARTEx_ReceiveToIdle_DMA(&huart1, reciveBuff,					sizeof(reciveBuff));
	}

	if (flagEvent.contact == 1) {
		uint8_t a = flagEvent.contact;
		HAL_UARTEx_ReceiveToIdle(&huart1, reciveBuff, (uint16_t) 50,
				(uint8_t*) &flagEvent.rxSize, 100);
		flagEvent.contact = a;
		flagEvent.rxReady = 1;
		//command_work();
	}
	/* USER CODE END 3 */
}
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

/** Initializes the RCC Oscillators according to the specified parameters
 * in the RCC_OscInitTypeDef structure.
 */
RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
RCC_OscInitStruct.HSIState = RCC_HSI_ON;
RCC_OscInitStruct.HSICalibrationValue =
RCC_HSICALIBRATION_DEFAULT;
RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
	Error_Handler();
}
/** Initializes the CPU, AHB and APB buses clocks
 */
RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
		| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
	Error_Handler();
}
}

/* USER CODE BEGIN 4 */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
//HAL_UART_Transmit(&huart1, reciveBuff, Size, 100);
flagEvent.rxReady = 1;
flagEvent.rxSize = Size;
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
