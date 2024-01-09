/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "pid.h"
#include "math.h"
#include "mpu9250.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MOTOR_2_STOP HAL_GPIO_WritePin(MOTOR_1_A_GPIO_Port, MOTOR_1_A_Pin, GPIO_PIN_SET); HAL_GPIO_WritePin(MOTOR_1_B_GPIO_Port, MOTOR_1_B_Pin, GPIO_PIN_SET)
#define MOTOR_2_TYL HAL_GPIO_WritePin(MOTOR_1_A_GPIO_Port, MOTOR_1_A_Pin, GPIO_PIN_RESET); HAL_GPIO_WritePin(MOTOR_1_B_GPIO_Port, MOTOR_1_B_Pin, GPIO_PIN_SET)
#define MOTOR_2_PRZOD HAL_GPIO_WritePin(MOTOR_1_A_GPIO_Port, MOTOR_1_A_Pin, GPIO_PIN_SET); HAL_GPIO_WritePin(MOTOR_1_B_GPIO_Port, MOTOR_1_B_Pin, GPIO_PIN_RESET)

#define MOTOR_1_STOP HAL_GPIO_WritePin(MOTOR_2_A_GPIO_Port, MOTOR_2_A_Pin, GPIO_PIN_SET); HAL_GPIO_WritePin(MOTOR_2_B_GPIO_Port, MOTOR_2_B_Pin, GPIO_PIN_SET)
#define MOTOR_1_TYL HAL_GPIO_WritePin(MOTOR_2_A_GPIO_Port, MOTOR_2_A_Pin, GPIO_PIN_SET); HAL_GPIO_WritePin(MOTOR_2_B_GPIO_Port, MOTOR_2_B_Pin, GPIO_PIN_RESET)
#define MOTOR_1_PRZOD HAL_GPIO_WritePin(MOTOR_2_A_GPIO_Port, MOTOR_2_A_Pin, GPIO_PIN_RESET); HAL_GPIO_WritePin(MOTOR_2_B_GPIO_Port, MOTOR_2_B_Pin, GPIO_PIN_SET)

#define MOTOR_4_STOP HAL_GPIO_WritePin(MOTOR_3_A_GPIO_Port, MOTOR_3_A_Pin, GPIO_PIN_SET); HAL_GPIO_WritePin(MOTOR_3_B_GPIO_Port, MOTOR_3_B_Pin, GPIO_PIN_SET)
#define MOTOR_4_TYL HAL_GPIO_WritePin(MOTOR_3_A_GPIO_Port, MOTOR_3_A_Pin, GPIO_PIN_SET); HAL_GPIO_WritePin(MOTOR_3_B_GPIO_Port, MOTOR_3_B_Pin, GPIO_PIN_RESET)
#define MOTOR_4_PRZOD HAL_GPIO_WritePin(MOTOR_3_A_GPIO_Port, MOTOR_3_A_Pin, GPIO_PIN_RESET); HAL_GPIO_WritePin(MOTOR_3_B_GPIO_Port, MOTOR_3_B_Pin, GPIO_PIN_SET)

#define MOTOR_3_STOP HAL_GPIO_WritePin(MOTOR_4_A_GPIO_Port, MOTOR_4_A_Pin, GPIO_PIN_SET); HAL_GPIO_WritePin(MOTOR_4_B_GPIO_Port, MOTOR_4_B_Pin, GPIO_PIN_SET)
#define MOTOR_3_TYL HAL_GPIO_WritePin(MOTOR_4_A_GPIO_Port, MOTOR_4_A_Pin, GPIO_PIN_RESET); HAL_GPIO_WritePin(MOTOR_4_B_GPIO_Port, MOTOR_4_B_Pin, GPIO_PIN_SET)
#define MOTOR_3_PRZOD HAL_GPIO_WritePin(MOTOR_4_A_GPIO_Port, MOTOR_4_A_Pin, GPIO_PIN_SET); HAL_GPIO_WritePin(MOTOR_4_B_GPIO_Port, MOTOR_4_B_Pin, GPIO_PIN_RESET)


#define ROZMIAR_RAMKI 10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

uint8_t data[50]; // Tablica przechowujaca wysylana wiadomosc.
uint16_t size = 0; // Rozmiar wysylanej wiadomosci
uint8_t Received;
char Odebrane_dane[ROZMIAR_RAMKI];

PID PID_MOTOR_1;
PID PID_MOTOR_2;
PID PID_MOTOR_3;
PID PID_MOTOR_4;

PID PID_MOTOR_5;
PID PID_MOTOR_6;
PID PID_MOTOR_7;

PID PID_ORIENTATION;



volatile double pwm_motor_1;
volatile double pwm_motor_2;
volatile double pwm_motor_3;
volatile double pwm_motor_4;
volatile double pwm_motor_5;
volatile double pwm_motor_6;
volatile double pwm_motor_7;

volatile int orientation;

volatile int pulse_encoder_1;
volatile int pulse_encoder_2;
volatile int pulse_encoder_3;
volatile int pulse_encoder_4;

volatile int flag_PID;
volatile int flag_rodzaj_ruchu;

volatile int sum_en_1;
volatile int sum_en_2;
volatile int sum_en_3;
volatile int sum_en_4;

volatile int set_speed;

float dt = 0;

uint8_t mpu9250_correct_init_global = 0;

struct MPU9250 mpu1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	char *wsk;

	wsk = strtok(Odebrane_dane, " ");

	if (!strcmp(wsk, "P")) {

		wsk = strtok(NULL, " ");

		if (!strcmp(wsk, "R")) {

			wsk = strtok(NULL, " ");

			switch (atoi(wsk)) {

			case 0: // Jezeli odebrany zostanie znak 0
				PID_MOTOR_1.set_speed = 0;
				PID_MOTOR_2.set_speed = 0;
				PID_MOTOR_3.set_speed = 0;
				PID_MOTOR_4.set_speed = 0;
				PID_ORIENTATION.speed_error_sum = 0;
				MPU9250_Reset_Position(&mpu1);
				flag_rodzaj_ruchu = 0;
				break;

			case 2: // Jezeli odebrany zostanie znak 1
				PID_MOTOR_1.set_speed = set_speed;
				PID_MOTOR_2.set_speed = set_speed;
				PID_MOTOR_3.set_speed = set_speed;
				PID_MOTOR_4.set_speed = set_speed;

				// PID_MOTOR_1.set_speed = set_speed;
				// PID_MOTOR_2.set_speed = -set_speed;

				//.set_speed = set_speed;

				MPU9250_Reset_Position(&mpu1);
				flag_rodzaj_ruchu = 2;
				break;

			case 8: // Jezeli odebrany zostanie znak 1
			 PID_MOTOR_1.set_speed = -set_speed;
			 PID_MOTOR_2.set_speed = -set_speed;
			 PID_MOTOR_3.set_speed = -set_speed;
			 PID_MOTOR_4.set_speed = -set_speed;

			//	PID_MOTOR_1.set_speed = -set_speed;

			//	PID_MOTOR_4.set_speed = -set_speed;

				MPU9250_Reset_Position(&mpu1);
				flag_rodzaj_ruchu = 8;
				break;

			case 3: // Jezeli odebrany zostanie znak 1
				PID_MOTOR_1.set_speed = set_speed;
				PID_MOTOR_2.set_speed = 0;
				PID_MOTOR_3.set_speed = set_speed;
				PID_MOTOR_4.set_speed = 0;
				MPU9250_Reset_Position(&mpu1);
				flag_rodzaj_ruchu = 3;
				break;

			case 7: // Jezeli odebrany zostanie znak 1
				PID_MOTOR_1.set_speed = -set_speed;
				PID_MOTOR_2.set_speed = 0;
				PID_MOTOR_3.set_speed = -set_speed;
				PID_MOTOR_4.set_speed = 0;
				MPU9250_Reset_Position(&mpu1);
				flag_rodzaj_ruchu = 7;
				break;

			case 1: // Jezeli odebrany zostanie znak 1
				PID_MOTOR_1.set_speed = 0;
				PID_MOTOR_2.set_speed = set_speed;
				PID_MOTOR_3.set_speed = 0;
				PID_MOTOR_4.set_speed = set_speed;
				MPU9250_Reset_Position(&mpu1);
				flag_rodzaj_ruchu = 1;
				break;

			case 9: // Jezeli odebrany zostanie znak 1
				PID_MOTOR_1.set_speed = 0;
				PID_MOTOR_2.set_speed = -set_speed;
				PID_MOTOR_3.set_speed = 0;
				PID_MOTOR_4.set_speed = -set_speed;
				MPU9250_Reset_Position(&mpu1);
				flag_rodzaj_ruchu = 9;
				break;

			case 6: // Jezeli odebrany zostanie znak 1
				PID_MOTOR_1.set_speed = set_speed;
				PID_MOTOR_2.set_speed = -set_speed;
				PID_MOTOR_3.set_speed = set_speed;
				PID_MOTOR_4.set_speed = -set_speed;
				MPU9250_Reset_Position(&mpu1);
				flag_rodzaj_ruchu = 6;
				break;

			case 4: // Jezeli odebrany zostanie znak 1
				PID_MOTOR_1.set_speed = -set_speed;
				PID_MOTOR_2.set_speed = set_speed;
				PID_MOTOR_3.set_speed = -set_speed;
				PID_MOTOR_4.set_speed = set_speed;
				MPU9250_Reset_Position(&mpu1);
				flag_rodzaj_ruchu = 4;
				break;

			case 10: // Jezeli odebrany zostanie znak 1
				PID_MOTOR_1.set_speed = set_speed;
				PID_MOTOR_2.set_speed = -set_speed;
				PID_MOTOR_3.set_speed = -set_speed;
				PID_MOTOR_4.set_speed = set_speed;
				MPU9250_Reset_Position(&mpu1);
				flag_rodzaj_ruchu = 10;
				break;

			case 11: // Jezeli odebrany zostanie znak 1
				PID_MOTOR_1.set_speed = -set_speed;
				PID_MOTOR_2.set_speed = set_speed;
				PID_MOTOR_3.set_speed = set_speed;
				PID_MOTOR_4.set_speed = -set_speed;
				MPU9250_Reset_Position(&mpu1);
				flag_rodzaj_ruchu = 11;
				break;

			case 14: // Jezeli odebrany zostanie znak 1
				PID_MOTOR_1.set_speed = -set_speed;
				PID_MOTOR_2.set_speed = set_speed;
				//PID_MOTOR_3.set_speed = set_speed;
				//PID_MOTOR_4.set_speed = -set_speed;
				MPU9250_Reset_Position(&mpu1);
				flag_rodzaj_ruchu = 14;
				break;

			case 15: // Jezeli odebrany zostanie znak 1
				PID_MOTOR_1.set_speed = set_speed;
				PID_MOTOR_2.set_speed = -set_speed;
//				PID_MOTOR_3.set_speed = set_speed;
//				PID_MOTOR_4.set_speed = -set_speed;
				MPU9250_Reset_Position(&mpu1);
				flag_rodzaj_ruchu = 15;
				break;

			case 16: // Jezeli odebrany zostanie znak 1
//				PID_MOTOR_1.set_speed = -set_speed;
//				PID_MOTOR_2.set_speed = set_speed;
				PID_MOTOR_3.set_speed = -set_speed;
				PID_MOTOR_4.set_speed = set_speed;
				MPU9250_Reset_Position(&mpu1);
				flag_rodzaj_ruchu = 16;
				break;

			case 19: // Jezeli odebrany zostanie znak 1
//				PID_MOTOR_1.set_speed = -set_speed;
//				PID_MOTOR_2.set_speed = set_speed;
				PID_MOTOR_3.set_speed = set_speed;
				PID_MOTOR_4.set_speed = -set_speed;
				MPU9250_Reset_Position(&mpu1);
				flag_rodzaj_ruchu = 19;
				break;

			case 17: // Jezeli odebrany zostanie znak 1
//				PID_MOTOR_1.set_speed = -set_speed;
				PID_MOTOR_2.set_speed = -set_speed;
				PID_MOTOR_3.set_speed = -set_speed;
//				PID_MOTOR_4.set_speed = -set_speed;
				MPU9250_Reset_Position(&mpu1);
				flag_rodzaj_ruchu = 17;
				break;

			case 20: // Jezeli odebrany zostanie znak 1
//				PID_MOTOR_1.set_speed = -set_speed;
				PID_MOTOR_2.set_speed = set_speed;
				PID_MOTOR_3.set_speed = set_speed;
//				PID_MOTOR_4.set_speed = set_speed;
				MPU9250_Reset_Position(&mpu1);
				flag_rodzaj_ruchu = 20;
				break;

			case 21: // Jezeli odebrany zostanie znak 1
				PID_MOTOR_1.set_speed = set_speed;
//				PID_MOTOR_2.set_speed = set_speed;
//				PID_MOTOR_3.set_speed = set_speed;
				PID_MOTOR_4.set_speed = set_speed;
				MPU9250_Reset_Position(&mpu1);
				flag_rodzaj_ruchu = 21;
				break;

			case 22: // Jezeli odebrany zostanie znak 1
				PID_MOTOR_1.set_speed = -set_speed;
//				PID_MOTOR_2.set_speed = set_speed;
//				PID_MOTOR_3.set_speed = set_speed;
				PID_MOTOR_4.set_speed = -set_speed;
				MPU9250_Reset_Position(&mpu1);
				flag_rodzaj_ruchu = 22;
				break;

			}

		}else if (!strcmp(wsk, "S")) {

			wsk = strtok(NULL, " ");
			set_speed = atoi(wsk);
		}

	}
	HAL_UART_Receive_IT(&huart3, (uint8_t*) Odebrane_dane, ROZMIAR_RAMKI);

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim) {

	static uint32_t count;

	if (htim == &htim5) {

		flag_PID = 1;
		count++;

		if (count > 199) {
			count = 0;
			//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		}
	}
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM5_Init();
  MX_USART3_UART_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */


	HAL_Delay(1000);


	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);

	HAL_TIM_Base_Start_IT(&htim5);

	MOTOR_1_STOP;
	MOTOR_2_STOP;
	MOTOR_3_STOP;
	MOTOR_4_STOP;

	pwm_motor_1 = 0;
	pwm_motor_2 = 0;
	pwm_motor_3 = 0;
	pwm_motor_4 = 0;
	pwm_motor_5 = 0;
	pwm_motor_6 = 0;
	pwm_motor_7 = 0;

	orientation = 0;

	set_speed = 10;

	TIM4->CCR1 = pwm_motor_1;
	TIM4->CCR2 = pwm_motor_2;
	TIM4->CCR3 = pwm_motor_3;
	TIM4->CCR4 = pwm_motor_4;

	TIM1->CNT = 5000;
	TIM2->CNT = 5000;
	TIM3->CNT = 5000;
	TIM8->CNT = 5000;

	PID_Set_parameters(&PID_MOTOR_1, KP_MOTOR_1, TI_MOTOR_1, TD_MOTOR_1);
	PID_Set_parameters(&PID_MOTOR_2, KP_MOTOR_2, TI_MOTOR_2, TD_MOTOR_2);
	PID_Set_parameters(&PID_MOTOR_3, KP_MOTOR_3, TI_MOTOR_3, TD_MOTOR_3);
	PID_Set_parameters(&PID_MOTOR_4, KP_MOTOR_4, TI_MOTOR_4, TD_MOTOR_4);

	PID_Set_parameters(&PID_ORIENTATION, KP_ORIENTATION, TI_ORIENTATION, TD_ORIENTATION);

	PID_Set_parameters(&PID_MOTOR_5, KP_MOTOR_5, TI_MOTOR_5, TD_MOTOR_5);
	PID_Set_parameters(&PID_MOTOR_6, KP_MOTOR_6, TI_MOTOR_6, TD_MOTOR_6);
	PID_Set_parameters(&PID_MOTOR_7, KP_MOTOR_7, TI_MOTOR_7, TD_MOTOR_7);

	sum_en_1 = 0;
	sum_en_2 = 0;
	sum_en_3 = 0;
	sum_en_4 = 0;

	flag_rodzaj_ruchu = 0;

	HAL_UART_Receive_IT(&huart3, (uint8_t*) Odebrane_dane, ROZMIAR_RAMKI);


	if (MPU9250_Init(&hi2c2, &mpu1, MPU9250_Device_1, MPU9250_Acce_2G, MPU9250_Gyro_2000s) == MPU9250_Init_OK) {

		MPU9250_Set_Offsets(&hi2c2, &mpu1, 2711, 998, -3711, 71.143, -8.411 , 93.567, 0, 0, 0);

		//MPU9250_Calibration_Acce(&hi2c2, &mpu1);
		//MPU9250_Calibration_Gyro(&hi2c2, &mpu1);
		//MPU9250_Calibration_Mag(&hi2c2, &mpu1);

		MPU9250_Calculate_RPY(&hi2c2, &mpu1, dt);
		mpu1.Gyroscope_Roll  = mpu1.Accelerometer_Roll;
		mpu1.Gyroscope_Pitch = mpu1.Accelerometer_Pitch;
		mpu1.Gyroscope_Yaw   = mpu1.Magnetometer_Yaw;

		//a_x_offset_global = mpu1.Accelerometer_X_offset, a_y_offset_global = mpu1.Accelerometer_Y_offset, a_z_offset_global = mpu1.Accelerometer_Z_offset;
		//g_x_offset_global = mpu1.Gyroscope_X_offset, g_y_offset_global = mpu1.Gyroscope_Y_offset, g_z_offset_global = mpu1.Gyroscope_Z_offset;

		for (int i = 0; i < 3; ++i) {

			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
			HAL_Delay(200);
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
			HAL_Delay(200);
		}

		mpu9250_correct_init_global = 1;
	}
	else {

		mpu9250_correct_init_global = 0;
	}


	dt = (float)( 5 ) / 1000;




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

		if (flag_PID == 1) {


			  if( mpu9250_correct_init_global == 1 ) {

				  MPU9250_Calculate_RPY(&hi2c2, &mpu1, dt);
			  }






			flag_PID = 0;

			PID_MOTOR_1.measured_speed = TIM1->CNT - 5000;
			TIM1->CNT -= PID_MOTOR_1.measured_speed;
			pulse_encoder_1 = PID_MOTOR_1.measured_speed;
			pwm_motor_1 = (PID_Calculate(&PID_MOTOR_1)*10);

			pulse_encoder_2 = TIM2->CNT - 5000;
			TIM2->CNT -= pulse_encoder_2;

			PID_MOTOR_2.measured_speed = pulse_encoder_2;
			pwm_motor_2 = (PID_Calculate(&PID_MOTOR_2)*10);

			pulse_encoder_3 = TIM3->CNT - 5000;
			TIM3->CNT -= pulse_encoder_3;

			PID_MOTOR_3.measured_speed = pulse_encoder_3;
			pwm_motor_3 = (PID_Calculate(&PID_MOTOR_3)*10);

			pulse_encoder_4 = TIM8->CNT - 5000;
			TIM8->CNT -= pulse_encoder_4;
			pulse_encoder_4 = -pulse_encoder_4;

			PID_MOTOR_4.measured_speed = pulse_encoder_4;
			pwm_motor_4 = (PID_Calculate(&PID_MOTOR_4)*10);


			if(flag_rodzaj_ruchu != 0){
			PID_ORIENTATION.measured_speed = mpu1.Gyroscope_Yaw;
			orientation = PID_Calculate(&PID_ORIENTATION);

			if(orientation > (set_speed/3)) orientation = (set_speed/3);
			if(orientation < (-set_speed/3)) orientation = (-set_speed/3);


			}

//			cala obsluga kalibracji
			if(mpu1.Gyroscope_Yaw > 0.01 || mpu1.Gyroscope_Yaw < -0.01 ){

				if(flag_rodzaj_ruchu == 3){
					PID_MOTOR_1.set_speed = set_speed + orientation;
					PID_MOTOR_3.set_speed = set_speed - orientation;

				}

				if(flag_rodzaj_ruchu == 7){
					PID_MOTOR_1.set_speed = -set_speed + orientation;
					PID_MOTOR_3.set_speed = -set_speed - orientation;

				}

				if(flag_rodzaj_ruchu == 1){
					PID_MOTOR_2.set_speed = set_speed - orientation;
					PID_MOTOR_4.set_speed = set_speed + orientation;

				}

				if(flag_rodzaj_ruchu == 9){
					PID_MOTOR_2.set_speed = -set_speed - orientation;
					PID_MOTOR_4.set_speed = -set_speed + orientation;

				}

// dla 7 i 8 musz¹ byc oscylacje inaczej bedzie go sciaga³o w druga strone
				// mozna wykrywanie k¹ta 0.2 wykrywac jako 0.1

			/*	if(flag_rodzaj_ruchu == 7){
					// orien >0
					PID_MOTOR_1.set_speed = set_speed + orientation;
					PID_MOTOR_2.set_speed = -set_speed - orientation;
					PID_MOTOR_3.set_speed = set_speed - orientation;
					PID_MOTOR_4.set_speed = -set_speed + orientation;

				}

				if(flag_rodzaj_ruchu == 8){

 // tu sa byki

					PID_MOTOR_1.set_speed = -set_speed + orientation;
					PID_MOTOR_2.set_speed = set_speed - orientation;
					PID_MOTOR_3.set_speed = -set_speed - orientation;
					PID_MOTOR_4.set_speed = set_speed + orientation;

				}

		*/
			} // koniec od niepewnosci


			sum_en_1 += pulse_encoder_1;
			sum_en_2 += pulse_encoder_2;
			sum_en_3 += pulse_encoder_3;
			sum_en_4 += pulse_encoder_4;



	/*		PID_MOTOR_5.measured_speed = abs(sum_en_1) - abs(sum_en_2);
			pwm_motor_5 = PID_Calculate(&PID_MOTOR_5);

			if(PID_MOTOR_2.set_speed > 0 ) pwm_motor_2 = pwm_motor_2 - pwm_motor_5;
			else if(PID_MOTOR_2.set_speed < 0 )pwm_motor_2 = pwm_motor_2 + pwm_motor_5;


			PID_MOTOR_6.measured_speed = abs(sum_en_1) - abs(sum_en_3);
			pwm_motor_6 = PID_Calculate(&PID_MOTOR_6);

			if(PID_MOTOR_3.set_speed > 0 ) pwm_motor_3 = pwm_motor_3 - pwm_motor_6;
			else if(PID_MOTOR_3.set_speed < 0 )pwm_motor_3 = pwm_motor_3 + pwm_motor_6;


			PID_MOTOR_7.measured_speed = abs(sum_en_1) - abs(sum_en_4);
			pwm_motor_7 = PID_Calculate(&PID_MOTOR_7);

			if(PID_MOTOR_4.set_speed > 0 ) pwm_motor_4 = pwm_motor_4 - pwm_motor_7;
			else if(PID_MOTOR_4.set_speed < 0 )pwm_motor_4 = pwm_motor_4 + pwm_motor_7;

*/




			if(pwm_motor_1 > 1000) {

				pwm_motor_1 = 1000;
			}
			else if(pwm_motor_1 < -1000) {

				pwm_motor_1 = -1000;
			}


			if (PID_MOTOR_1.set_speed > 0) {
				MOTOR_1_PRZOD;
				TIM4->CCR2 = pwm_motor_1;
			} else if (PID_MOTOR_1.set_speed < 0) {
				MOTOR_1_TYL;
				TIM4->CCR2 = -pwm_motor_1;
			} else {
				MOTOR_1_STOP;
				PID_MOTOR_1.speed_error_sum = 0;
				sum_en_1 = 0;
				//	TIM4->CCR2 = 0;
			}




			if(pwm_motor_2 > 1000) {

				pwm_motor_2 = 1000;
			}
			else if(pwm_motor_2 < -1000) {

				pwm_motor_2 = -1000;
			}

			if (PID_MOTOR_2.set_speed > 0) {
				MOTOR_2_PRZOD;
				TIM4->CCR1 = pwm_motor_2;
			} else if (PID_MOTOR_2.set_speed < 0) {
				MOTOR_2_TYL;
				TIM4->CCR1 = -pwm_motor_2;
			} else {
				MOTOR_2_STOP;
				PID_MOTOR_2.speed_error_sum = 0;
				PID_MOTOR_5.speed_error_sum = 0;
				sum_en_2 = 0;
				//	TIM4->CCR1 = 0;
			}



			if(pwm_motor_3 > 1000) {

				pwm_motor_3 = 1000;
			}
			else if(pwm_motor_3 < -1000) {

				pwm_motor_3 = -1000;
			}

			if (PID_MOTOR_3.set_speed > 0) {
				MOTOR_3_PRZOD;
				TIM4->CCR4 = pwm_motor_3;
			} else if (PID_MOTOR_3.set_speed < 0) {
				MOTOR_3_TYL;
				TIM4->CCR4 = -pwm_motor_3;
			} else {
				MOTOR_3_STOP;
				PID_MOTOR_3.speed_error_sum = 0;
				PID_MOTOR_6.speed_error_sum = 0;
				sum_en_3 = 0;
				//	TIM4->CCR4 = 0;
			}


			if(pwm_motor_4 > 1000) {

				pwm_motor_4 = 1000;
			}
			else if(pwm_motor_4 < -1000) {

				pwm_motor_4 = -1000;
			}

			if (PID_MOTOR_4.set_speed > 0) {
				MOTOR_4_PRZOD;
				TIM4->CCR3 = pwm_motor_4;
			} else if (PID_MOTOR_4.set_speed < 0) {
				MOTOR_4_TYL;
				TIM4->CCR3 = -pwm_motor_4;
			} else {
				MOTOR_4_STOP;
				PID_MOTOR_4.speed_error_sum = 0;
				PID_MOTOR_7.speed_error_sum = 0;
				sum_en_4 = 0;
				//	TIM4->CCR3 = 0;
			}


					int wynik_z = (int)mpu1.Gyroscope_Yaw;
					size = sprintf(data, "EN_1: %d EN_2: %d EN_3: %d EN_4: %d\n\r"
							"S_1: %d S_2: %d S_3: %d S_4: %d\n\r"
							"MPU.z =  %d PID_od_z: %d \n\r \n\r",
							pulse_encoder_1, pulse_encoder_2, pulse_encoder_3, pulse_encoder_4,
							sum_en_1,sum_en_2,sum_en_3,sum_en_4, wynik_z, orientation );


					HAL_UART_Transmit_IT(&huart3, data, size);

		}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 90;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10003;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10003;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10003;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 899;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 9999;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 44;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 10003;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|MOTOR_2_B_Pin|MOTOR_3_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, MOTOR_3_A_Pin|MOTOR_4_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MOTOR_4_A_GPIO_Port, MOTOR_4_A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MOTOR_2_A_Pin|MOTOR_1_A_Pin|MOTOR_1_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin MOTOR_2_B_Pin MOTOR_3_B_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|MOTOR_2_B_Pin|MOTOR_3_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR_3_A_Pin MOTOR_4_B_Pin */
  GPIO_InitStruct.Pin = MOTOR_3_A_Pin|MOTOR_4_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : MOTOR_4_A_Pin */
  GPIO_InitStruct.Pin = MOTOR_4_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOTOR_4_A_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR_2_A_Pin MOTOR_1_A_Pin MOTOR_1_B_Pin */
  GPIO_InitStruct.Pin = MOTOR_2_A_Pin|MOTOR_1_A_Pin|MOTOR_1_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
