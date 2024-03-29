/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define AVANCE 	GPIO_PIN_SET
#define RECULE  GPIO_PIN_RESET
#define POURCENT 640
#define Seuil_Dist_4 1600 // corespond � 10 cm.
#define Seuil_Dist_3 1600
#define Seuil_Dist_1 1600
#define Seuil_Dist_2 1600
#define V1 38
#define V2 56
#define V3 76
#define Vmax 95
#define T_2_S 1000 //( pwm p�riode = 2 ms )
#define T_200_MS 100
#define T_100_MS 50
#define T_2000_MS 1000
#define CKp_D 120  //80 Robot1
#define CKp_G 100  //80 Robot1
#define CKi_D 80  //50 Robot1
#define CKi_G 80  //50 Robot1
#define CKd_D 0
#define CKd_G 0
#define DELTA 0x50
#define alpha_m90_deg 795 //valeur pour le pilotage de l'angle du servo
#define alpha_0_deg 2000
#define alpha_90_deg 3800
#define T_sonar_11 500
#define T_sonar_1 2000


enum CMDE {
	START,
	STOP,
	AVANT,
	ARRIERE,
	DROITE,
	GAUCHE,
	PARKED,
	ATT_PARK
};
volatile enum CMDE CMDE;
enum MODE {
	SLEEP, ACTIF
};
volatile enum MODE Mode;
volatile unsigned char New_CMDE = 0;
volatile uint16_t Dist_ACS_1, Dist_ACS_2, Dist_ACS_3, Dist_ACS_4;
volatile unsigned int Time = 0;
volatile unsigned int Time_servo = 0;
volatile unsigned int Time_mesure = 0;
volatile unsigned int Tech = 0;
uint16_t adc_buffer[8];
uint16_t Buff_Dist[8];
uint8_t BLUE_RX;

uint16_t _DirG, _DirD, CVitG, CVitD, DirD, DirG;
uint16_t _CVitD = 0;
uint16_t _CVitG = 0;
uint16_t VitD, VitG, DistD, DistG;
uint16_t DistD_old = 0;
uint16_t DistG_old = 0;
int Cmde_VitD = 0;
int Cmde_VitG = 0;
unsigned long Dist_parcours = 0;
volatile uint32_t Dist_Obst;
uint32_t Dist_Obst_;
uint32_t Dist_Obst_cm;
uint32_t Dist;
uint8_t UNE_FOIS = 1;
uint32_t OV = 0;

int8_t choix_xyz = -90; //mesure xyz

volatile uint8_t mesure_0 = 0;
volatile uint8_t mesure_90 = 0;
volatile uint8_t mesure_m90 = 0;

//fonction parked
uint8_t parked_x = 20;
uint8_t parked_y = 150;
uint8_t parked_z = 0;

volatile unsigned char flag_parked = 0;
volatile unsigned char flag_att_park = 0;
volatile int8_t flag_mesure = 0;
volatile unsigned char tempo_sonar = 0;

extern volatile unsigned char flag_awd;

volatile unsigned char Trig_sonar=0;
volatile uint8_t Sonar_last_measure=0; //Flag de calcul de distance possible
//A DELETE�����������������������������������������������������
volatile int8_t flag_mesure_xyz=0;


// ---- Attente parked ----
enum etat_park_list{Avancer50, Tourner90, AvancerAxeYZCons, Tourner902, AvancerAxeXcons, Fin};
volatile enum etat_park_list etat_park = Avancer50;
uint8_t flag_prem_passage = 0;
uint16_t memoire_dist = 0;

// ---- Fin attente park ----

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void Gestion_Commandes(void);
void regulateur(void);
void controle(void);
void Calcul_Vit(void);
void ACS(void);
void mesure_position_robot(void);
void mesure_xyz(int8_t xyz);
void mesure_distance_sonar(void);
void deplacement_to_park(void);
//void rotation_90(char sens_rotation);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  Dist_Obst = 0;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  	HAL_SuspendTick(); // suppresion des Tick interrupt pour le mode sleep.

  	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);  // Start PWM motor
  	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);  // Start input capture motor
  	HAL_TIM_IC_Start(&htim1, TIM_CHANNEL_1);  // Start input capture motor

  	HAL_TIM_IC_Start(&htim1, TIM_CHANNEL_2);  // Start input capture motor
  	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);  // Start input capture motor

  	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);  // Start PWM motor
  	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  	CMDE = STOP;
  	New_CMDE = 1;
  	HAL_TIM_Base_Start_IT(&htim2);  // Start IT sur font montant PWM
  	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  	HAL_UART_Receive_IT(&huart3, &BLUE_RX, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */



  while (1)
  {
	  //batterie faible
	  if (flag_awd == 1){
		  HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_SET);
		  flag_awd = 0;
		  Mode = SLEEP;
	  }
	  Gestion_Commandes();
	  controle();

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  /* USART3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
}

/* USER CODE BEGIN 4 */
void Gestion_Commandes(void) {
	enum ETAT {
		VEILLE,
		ARRET,
		AV1,
		AV2,
		AV3,
		RV1,
		RV2,
		RV3,
		DV1,
		DV2,
		DV3,
		GV1,
		GV2,
		GV3
	};
	static enum ETAT Etat = VEILLE;

if (New_CMDE) {
		New_CMDE = 0;
	switch (CMDE) {
		case STOP: {
			_CVitD = _CVitG = 0;
			// Mise en sommeil: STOP mode , r�veil via IT BP1
			Etat = VEILLE;
			Mode = SLEEP;

			break;
		}
		case START: {
			// r�veil syt�me grace � l'IT BP1
			Etat = ARRET;
			Mode = SLEEP;

			break;
		}
		case AVANT: {
			switch (Etat) {
			case VEILLE: {
				Etat = VEILLE;
				Mode = SLEEP;
				break;
			}
			case ARRET: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = AV1;
				Mode = ACTIF;
				break;
			}
			case AV1: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = AV2;
				Mode = ACTIF;
				break;
			}
			case AV2: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V3;
				_CVitD = V3 ;
				Etat = AV3;
				Mode = ACTIF;
				break;
			}
			case AV3: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V3;
				_CVitD = V3 ;
				Etat = AV3;
				Mode = ACTIF;
				break;
			}
			case RV1: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = 0;
				_CVitD = 0;
				Etat = ARRET;
				Mode = SLEEP;

				break;
			}
			case RV2: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = RV1;
				Mode = ACTIF;
				break;
			}
			case RV3: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = RV2;
				Mode = ACTIF;
				break;
			}
			case DV1: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = AV1;
				Mode = ACTIF;
				break;
			}
			case DV2: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = AV2;
				Mode = ACTIF;
				break;
			}
			case DV3: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = AV3;
				Mode = ACTIF;
				break;
			}
			case GV1: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = AV2;
				Mode = ACTIF;
				break;
			}
			case GV2: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = AV2;
				Mode = ACTIF;
				break;
			}
			case GV3: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = AV3;
				Mode = ACTIF;
				break;
			}
			}
			break;
		}
		case ARRIERE: {
			switch (Etat) {
			case VEILLE: {
				Etat = VEILLE;
				Mode = SLEEP;
				break;
			}
			case ARRET: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = RV1;
				Mode = ACTIF;
				break;
			}
			case AV1: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = 0;
				_CVitD = 0;
				Etat = ARRET;
				Mode = SLEEP;

				break;
			}
			case AV2: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = AV1;
				Mode = ACTIF;
				break;
			}
			case AV3: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = AV2;
				Mode = ACTIF;
				break;
			}
			case RV1: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = RV2;
				Mode = ACTIF;
				break;
			}
			case RV2: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = RV3;
				Mode = ACTIF;
				break;
			}
			case RV3: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = RV3;
				Mode = ACTIF;
				break;
			}
			case DV1: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = RV1;
				Mode = ACTIF;
				break;
			}
			case DV2: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = RV2;
				Mode = ACTIF;
				break;
			}
			case DV3: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = RV3;
				Mode = ACTIF;
				break;
			}
			case GV1: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = RV1;
				Mode = ACTIF;
				break;
			}
			case GV2: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = RV2;
				Mode = ACTIF;
				break;
			}
			case GV3: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = RV3;
				Mode = ACTIF;
				break;
			}
			}
			break;
		}
		case DROITE: {
			switch (Etat) {
			case VEILLE: {
				Etat = VEILLE;
				Mode = SLEEP;
				break;
			}
			case ARRET: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = DV1;
				Mode = ACTIF;
				break;
			}
			case AV1: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = DV1;
				Mode = ACTIF;
				break;
			}
			case AV2: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = DV2;
				Mode = ACTIF;
				break;
			}
			case AV3: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = DV3;
				Mode = ACTIF;
				break;
			}
			case RV1: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = DV1;
				Mode = ACTIF;
				break;
			}
			case RV2: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = DV2;
				Mode = ACTIF;
				break;
			}
			case RV3: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = DV3;
				Mode = ACTIF;
				break;
			}
			case DV1: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = DV2;
				Mode = ACTIF;
				break;
			}
			case DV2: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = DV3;
				Mode = ACTIF;
				break;
			}
			case DV3: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = DV3;
				Mode = ACTIF;
				break;
			}
			case GV1: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = 0;
				_CVitD = 0;
				Etat = ARRET;
				Mode = SLEEP;

				break;
			}
			case GV2: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = GV1;
				Mode = ACTIF;
				break;
			}
			case GV3: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = GV2;
				Mode = ACTIF;
				break;
			}
			}
			break;
		}
		case GAUCHE: {
			switch (Etat) {
			case VEILLE: {
				Etat = VEILLE;
				Mode = SLEEP;
				break;
			}
			case ARRET: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = GV1;
				Mode = ACTIF;
				break;
			}
			case AV1: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = GV1;
				Mode = ACTIF;
				break;
			}
			case AV2: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = GV2;
				Mode = ACTIF;
				break;
			}
			case AV3: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = GV3;
				Mode = ACTIF;
				break;
			}
			case RV1: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = GV1;
				Mode = ACTIF;
				break;
			}
			case RV2: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = GV2;
				Mode = ACTIF;
				break;
			}
			case RV3: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = GV3;
				Mode = ACTIF;
				break;
			}
			case DV1: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = 0;
				_CVitD = 0;
				Etat = ARRET;
				Mode = SLEEP;

				break;
			}
			case DV2: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = DV1;
				Mode = ACTIF;
				break;
			}
			case DV3: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = DV2;
				Mode = ACTIF;
				break;
			}
			case GV1: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = GV2;
				Mode = ACTIF;
				break;
			}
			case GV2: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = GV3;
				Mode = ACTIF;
				break;
			}
			case GV3: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = GV3;
				Mode = ACTIF;
				break;
			}
			}
			break;

		}
		case PARKED: {
			flag_parked = 1;
			break;
		}
		case ATT_PARK: {
			Mode = ACTIF;
			flag_att_park = 1;
			break;
		}
	}
}
}
void controle(void) {

	//Gestion du park
	/*if (rotation_90_test == 1){
		rotation_90(1);
	}*/

	if (flag_parked == 1){
		mesure_position_robot();
	}
	if (flag_mesure == -1){ //cas o� l'on vient de faire une mesure (fin mesure : flag_mesure = -1), on remet � 0 le flag (pr�t pour une nouvelle mesure)
		flag_mesure = 0;
	}
	if (flag_mesure > 0){
		mesure_xyz(choix_xyz);
	}

	if (flag_att_park == 1){
		deplacement_to_park();
	}

	if (Tech >= T_200_MS) {
		Tech = 0;
		ACS();
		Calcul_Vit();
		regulateur();
	}
}

void ACS(void) {
	enum ETAT {
		ARRET, ACTIF
	};
	static enum ETAT Etat = ARRET;
	static uint16_t Delta1 = 0;
	static uint16_t Delta2 = 0;
	static uint16_t Delta3 = 0;
	static uint16_t Delta4 = 0;

	switch (Etat) {
	case ARRET: {
		if (Mode == ACTIF )
			Etat = ACTIF;
		else {
			CVitD = _CVitD;
			CVitG = _CVitG;
			DirD = _DirD;
			DirG = _DirG;
		}
		break;
	}
	case ACTIF: {
		if (Mode == SLEEP)
			Etat = ARRET;
		if (_DirD == AVANCE && _DirG == AVANCE) {
			if ((Dist_ACS_1 < Seuil_Dist_1 - Delta1)
					&& (Dist_ACS_2 < Seuil_Dist_2 - Delta2)) {
				CVitD = _CVitD;
				CVitG = _CVitG;
				DirD = _DirD;
				DirG = _DirG;
				Delta1 = Delta2 = 0;
			} else if ((Dist_ACS_1 < Seuil_Dist_1)
					&& (Dist_ACS_2 > Seuil_Dist_2)) {
				CVitD = V1;
				CVitG = V1;
				DirG = AVANCE;
				DirD = RECULE;
				Delta2 = DELTA;
			} else if ((Dist_ACS_1 > Seuil_Dist_1)
					&& (Dist_ACS_2 < Seuil_Dist_2)) {
				CVitD = V1;
				CVitG = V1;
				DirD = AVANCE;
				DirG = RECULE;
				Delta1 = DELTA;
			} else if ((Dist_ACS_1 > Seuil_Dist_1)
					&& (Dist_ACS_2 > Seuil_Dist_2)) {
				CVitD = 0;
				CVitG = 0;
				DirD = RECULE;
				DirG = RECULE;
			}
		} else if (_DirD == RECULE && _DirG == RECULE) {
			if ((Dist_ACS_3 < Seuil_Dist_3 - Delta3)
					&& (Dist_ACS_4 < Seuil_Dist_4 - Delta4)) {
				CVitD = _CVitD;
				CVitG = _CVitG;
				DirD = _DirD;
				DirG = _DirG;
				Delta3 = Delta4 = 0;
			} else if ((Dist_ACS_3 > Seuil_Dist_3)
					&& (Dist_ACS_4 < Seuil_Dist_4)) {
				CVitD = V1;
				CVitG = V1;
				DirD = AVANCE;
				DirG = RECULE;
				Delta3 = DELTA;
			} else if ((Dist_ACS_3 < Seuil_Dist_3)
					&& (Dist_ACS_4 > Seuil_Dist_4)) {
				CVitD = V1;
				CVitG = V1;
				DirG = AVANCE;
				DirD = RECULE;
				Delta4 = DELTA;
			} else if ((Dist_ACS_3 > Seuil_Dist_3)
					&& (Dist_ACS_4 > Seuil_Dist_4)) {
				CVitD = 0;
				CVitG = 0;
				DirD = RECULE;
				DirG = RECULE;
			}
		} else {
			CVitD = _CVitD;
			CVitG = _CVitG;
			DirD = _DirD;
			DirG = _DirG;
		}
		break;
	}
	}
}

float flag_av_ar = 0;

void deplacement_to_park(void){
	uint16_t current_dist = 0;
	uint16_t dist_to_do = 0;
	switch (etat_park) {
		case Avancer50: {
			if(flag_prem_passage == 0){
				flag_prem_passage = 1;
				memoire_dist = DistG;
			}
			else{
				if(abs(DistG - memoire_dist) < 840){
					_CVitD = V2; _CVitG = V2; _DirD = AVANCE; _DirG = AVANCE;
				}else{
					_CVitD = 0; _CVitG = 0; _DirD = AVANCE; _DirG = AVANCE;
					etat_park = Tourner90;
					flag_prem_passage = 0;
				}
			}
			break;
		}

		case Tourner90:{ //Rotation � 90� dans le sens horaire
			if (flag_prem_passage == 0){
				flag_prem_passage = 1;
				memoire_dist = DistG;
			}
			else{
				if(abs(DistG - memoire_dist) < 480){
					_CVitD = 30; _CVitG = 30; _DirD = RECULE; _DirG = AVANCE;
				}else{
					_CVitD = 0; _CVitG = 0; _DirD = AVANCE; _DirG = AVANCE;
					etat_park = AvancerAxeYZCons;
					flag_prem_passage = 0;
				}
			}
			break;
		}

		case AvancerAxeYZCons:{
			if( flag_prem_passage == 0){ //On effectue la mesure de y
				if (flag_mesure == 0){
					choix_xyz = 0; //Mesure de y
					flag_mesure = 1; //valeur flag pour initialiser mesure_xyz()
				}
				if (flag_mesure == -1){ //quand on a fini la mesure
					flag_prem_passage = 1;
				}
				memoire_dist = DistG;
			}else{
				if(parked_y > 50){
					dist_to_do = abs(mesure_0 - (parked_y - 50));
					if( mesure_0 > parked_y - 50){
						current_dist = (DistG - memoire_dist) * 0.0566;
						_CVitD = V2; _CVitG = V2; _DirD = AVANCE; _DirG = AVANCE;
						if(current_dist > dist_to_do){
							_CVitD = 0; _CVitG = 0; _DirD = AVANCE; _DirG = AVANCE;
							etat_park = Tourner902;
							flag_av_ar = 1;
							flag_prem_passage = 0;
						}
					}else{
						current_dist = (memoire_dist - DistG) * 0.0566;
						_CVitD = V2; _CVitG = V2; _DirD = RECULE; _DirG = RECULE;
						if(current_dist > dist_to_do){
							_CVitD = 0; _CVitG = 0; _DirD = AVANCE; _DirG = AVANCE;
							etat_park = Tourner902;
							flag_av_ar = 0.8;
							flag_prem_passage = 0;
						}
					}
				}else{
					dist_to_do = abs(mesure_0 - (parked_y + 50));
					if( mesure_0 > parked_y + 50){
						current_dist = (DistG - memoire_dist) * 0.0566;
						_CVitD = V2; _CVitG = V2; _DirD = AVANCE; _DirG = AVANCE;
						if(current_dist > dist_to_do){
							_CVitD = 0; _CVitG = 0; _DirD = AVANCE; _DirG = AVANCE;
							etat_park = Tourner902;
							flag_av_ar = 0.7;
							flag_prem_passage = 0;
						}
					}else{
						current_dist = (memoire_dist - DistG) * 0.0566;
						_CVitD = V2; _CVitG = V2; _DirD = RECULE; _DirG = RECULE;
						if(current_dist > dist_to_do){
							_CVitD = 0; _CVitG = 0; _DirD = AVANCE; _DirG = AVANCE;
							etat_park = Tourner902;
							flag_av_ar = 1.3;
							flag_prem_passage = 0;
						}
					}
				}
			}
			break;
		}

		case Tourner902:{ //Rotation � 90� dans le sens antihoraire
			if (flag_prem_passage == 0){
				flag_prem_passage = 1;
				memoire_dist = DistG;
			}
			else{
				if(abs(DistG - memoire_dist) < 480*flag_av_ar){
					_CVitD = 30; _CVitG = 30; _DirD = AVANCE; _DirG = RECULE;
				}else{
					_CVitD = 0; _CVitG = 0; _DirD = AVANCE; _DirG = AVANCE;
					etat_park = AvancerAxeXcons;
					flag_prem_passage = 0;
				}
			}
			break;
		}

		case AvancerAxeXcons:{
			if( flag_prem_passage == 0){ //On effectue la mesure de x
				if (flag_mesure == 0){
					choix_xyz = 0; //Mesure de x
					flag_mesure = 1; //valeur flag pour initialiser mesure_xyz()
				}
				if (flag_mesure == -1){ //quand on a fini la mesure
					flag_prem_passage = 1;
				}
				memoire_dist = DistG;
			}else{
				dist_to_do = mesure_0 - parked_x;
				current_dist = (DistG - memoire_dist) * 0.0566;
				_CVitD = V2; _CVitG = V2; _DirD = AVANCE; _DirG = AVANCE;
				if(current_dist > dist_to_do){
					_CVitD = 0; _CVitG = 0; _DirD = AVANCE; _DirG = AVANCE;
					etat_park = Fin;
					flag_prem_passage = 0;
				}
			}
			break;
		}

		case Fin:{
			flag_att_park = 0;
			Mode = SLEEP;
			break;
		}
	}
}


void Calcul_Vit(void) {

	DistD = __HAL_TIM_GET_COUNTER(&htim3);
	DistG = __HAL_TIM_GET_COUNTER(&htim4);
	VitD = abs(DistD - DistD_old);
	VitG = abs(DistG - DistG_old);
	DistD_old = DistD;
	DistG_old = DistG;
	if (DirD == DirG) {
		Dist_parcours = Dist_parcours + ((VitD + VitG) >> 1);
	}
}

void regulateur(void) {
	enum ETAT {
		ARRET, ACTIF
	};
	static enum ETAT Etat = ARRET;
	uint16_t Kp_D = CKp_D;
	uint16_t Kp_G = CKp_G;
	uint16_t Ki_D = CKi_D;
	uint16_t Ki_G = CKi_G;
	uint16_t Kd_D = CKd_D;
	uint16_t Kd_G = CKd_G;

	static int16_t ErreurD = 0;
	static int16_t ErreurG = 0;
	static int16_t ErreurD_old = 0;
	static int16_t ErreurG_old = 0;
	static int16_t S_erreursD = 0;
	static int16_t S_erreursG = 0;
	static int16_t V_erreurD = 0;
	static int16_t V_erreurG = 0;

	switch (Etat) {
	case ARRET: {
		if (Mode == ACTIF)
			Etat = ACTIF;
		else {
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
			HAL_GPIO_WritePin(IR3_out_GPIO_Port, IR3_out_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(IR4_out_GPIO_Port, IR4_out_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(IR1_out_GPIO_Port, IR1_out_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(IR2_out_GPIO_Port, IR2_out_Pin, GPIO_PIN_RESET);

			HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON,
					PWR_SLEEPENTRY_WFI);

			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
			Time = 0;
		}
		break;
	}
	case ACTIF: {
		if ((CVitD != 0) && (CVitG != 0))
			Time = 0;
		if ((Mode == SLEEP) && (VitD == 0) && (VitG == 0) && Time > T_2_S)
			Etat = ARRET;
		else {
			ErreurD = CVitD - VitD;
			ErreurG = CVitG - VitG;
			S_erreursD += ErreurD;
			S_erreursG += ErreurG;
			V_erreurD = ErreurD - ErreurD_old;
			V_erreurG = ErreurG - ErreurG_old;
			ErreurD_old = ErreurD;
			ErreurG_old = ErreurG;
			Cmde_VitD = (unsigned int) Kp_D * (int) (ErreurD)
					+ (unsigned int) Ki_D * ((int) S_erreursD)
					+ (unsigned int) Kd_D * (int) V_erreurD;
			Cmde_VitG = (unsigned int) Kp_G * (int) (ErreurG)
					+ (unsigned int) Ki_G * ((int) S_erreursG)
					+ (unsigned int) Kd_G * (int) V_erreurG;

			//Cmde_VitD = _CVitD*640;
			//Cmde_VitG = _CVitG*640;
			//	DirD = _DirD;
			//	DirG= _DirG;

			if (Cmde_VitD < 0)
				Cmde_VitD = 0;
			if (Cmde_VitG < 0)
				Cmde_VitG = 0;
			if (Cmde_VitD > 100 * POURCENT)
				Cmde_VitD = 100 * POURCENT;
			if (Cmde_VitG > 100 * POURCENT)
				Cmde_VitG = 100 * POURCENT;
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint16_t ) Cmde_VitG);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, (uint16_t ) Cmde_VitD);
			HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, (GPIO_PinState) DirD);
			HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, (GPIO_PinState) DirG);
		}
		break;
	}
	}
}

/*
void rotation_90(char sens_rotation){ //Si sens_rotation = 1 on tourne de 90� dans le sens horraire sinon dans le sens anti-horraire
	uint32_t nb_top_G;
	uint32_t nb_top_D;

	uint32_t nb_actu_top_D =  __HAL_TIM_GET_COUNTER(&htim3);

	uint32_t d=0;
	uint8_t angle = 0;

	if(flag_debut_rotation == 0){
		Nb_top_init_D =  __HAL_TIM_GET_COUNTER(&htim3); //nb de top au d�but de la rotation
		flag_debut_rotation = 1; //La rotation a commenc�e
	}

	//Gestion de la diff�rence des tops en fonction du sens de rotation
	if (sens_rotation == 0){ //rotation anti-horraire D est en mode avance, les tops s'incr�mentent
		if ( Nb_top_init_D > __HAL_TIM_GET_COUNTER(&htim3) ){ //Il y a eu un overflow du timer
			nb_top_D = 65535 + (nb_actu_top_D - Nb_top_init_D) ; //On compense l'overflow
		} else {
			nb_top_D = nb_actu_top_D - Nb_top_init_D ;
		}
		_CVitD = 20; _CVitG = 20; _DirD = AVANCE; _DirG = RECULE;
	} else{  //rotation horraire D est en mode arri�re, les tops d�cr�mentent
		if ( Nb_top_init_D < __HAL_TIM_GET_COUNTER(&htim3) ){ //Il y a eu un overflow du timer
			nb_top_D = 65535 + Nb_top_init_D - nb_actu_top_D;
		} else {
			nb_top_D = nb_actu_top_D - Nb_top_init_D ; //On compense l'overflow
		}
		_CVitD = 20; _CVitG = 20; _DirD = RECULE; _DirG = AVANCE;
	}
	//angle = 90* d/D
	//DiametreRoue = 6cm
	//d = 360*nb_top_D*DiametreRoue*pi/333*2*180 = 0.0566 *nb_top
	//D = 14.84 cm
	angle = 0.3432 * nb_top_D; //en degr�
	if (angle>110 && angle<114){
		flag_debut_rotation = 0; //Fin de la rotation passage � l'�tat suivant
		rotation_90_test = 0;
		_CVitD = 0; _CVitG = 0;
		_DirD = AVANCE; _DirG = AVANCE;
	}
}*/

void mesure_position_robot(void){
//fonction permettant la mesure en mode commande park des positions x y et z

	if (flag_parked == 1){ //tant qu'on n'a pas fini la mesure des 3 positions

		switch (choix_xyz){
		case -90:
			if (flag_mesure == 0){
				flag_mesure = 1; //valeur flag pour initialiser mesure_xyz()
			}
			if (flag_mesure == -1){ //quand on a fini la mesure, on passe � l'angle suivant
				parked_y = mesure_m90;
				choix_xyz = 0;
			}
			break;
		case 0:
			if (flag_mesure == 0){
				flag_mesure = 1; //valeur flag pour initialiser mesure_xyz()
			}
			if (flag_mesure == -1){ //quand on a fini la mesure, on passe � l'angle suivant
				parked_x = mesure_0;
				choix_xyz = 90;
			}
			break;
		case 90:
			if (flag_mesure == 0){
				flag_mesure = 1; //valeur flag pour initialiser mesure_xyz()
			}
			if (flag_mesure == -1){ //quand on a fini la mesure, on passe � l'angle suivant
				parked_z = mesure_90;
				choix_xyz = -90;
				flag_parked = 0; //on a fini de mesurer x,y,z
			}
			break;
		}
	}
}

void mesure_xyz(int8_t xyz){
//fonction permettant de mesurer la distance x,y ou z du robot � un mur
	Mode = ACTIF ;
	_CVitD = 0; _CVitG = 0;
	_DirD = AVANCE; _DirG = AVANCE;

	if (flag_mesure == 1){
		Time_mesure = 0;
		flag_mesure = 2;
	}

	if (flag_mesure == 2){
		switch (xyz){
		case 0:
			flag_mesure_xyz = 0;
			if(Time_mesure < T_sonar_11){
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, alpha_0_deg);
			} else {
				mesure_distance_sonar(); //on lance la mesure du sonar
				if (tempo_sonar == 0){ //quand on a fini la mesure du sonar
					flag_mesure = -1;
					Mode = SLEEP;
				}
			}
			break;
		case -90:
			flag_mesure_xyz = -90;
			if(Time_mesure < T_sonar_1){
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, alpha_m90_deg);
			} else {
				mesure_distance_sonar(); //on lance la mesure du sonar
				if (tempo_sonar == 0){ //quand on a fini la mesure du sonar
					flag_mesure = -1;
					Mode = SLEEP;
				}
			}
			break;
		case 90:
			flag_mesure_xyz = 90;
			if(Time_mesure < T_sonar_11){
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, alpha_90_deg);
			} else {
				mesure_distance_sonar(); //on lance la mesure du sonar
				if (tempo_sonar == 0){ //quand on a fini la mesure du sonar
					flag_mesure = -1;
					Mode = SLEEP;
				}
			}
			break;
		}
	}
}

void mesure_distance_sonar(void){
	Trig_sonar = 1;
	tempo_sonar = 1;
	HAL_GPIO_WritePin(Trig_sonar_GPIO_Port, Trig_sonar_Pin, (GPIO_PinState) Trig_sonar);

}


//Callback de l'interruption de InputCaptureCompare du timer1
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_1){//Front montant de InputComapre
		Trig_sonar = 0; //On arr�te le signal de demande de mesure sonar
		HAL_GPIO_WritePin(Trig_sonar_GPIO_Port, Trig_sonar_Pin, (GPIO_PinState) Trig_sonar);
	}
	else if (htim->Channel==HAL_TIM_ACTIVE_CHANNEL_2){ //Front descendant de InputComapre
		//On divise la valeur mesur�e par une constante (100) afin d'obtenir une distance
		switch (flag_mesure_xyz){
		case 0:
			mesure_0 = (htim->Instance->CCR2)/100;
			tempo_sonar = 0;
			break;
		case -90:
			mesure_m90 = (htim->Instance->CCR2)/100;
			tempo_sonar = 0;
			break;
		case 90:
			mesure_90 = (htim->Instance->CCR2)/100;
			tempo_sonar = 0;
			break;
		}

	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART3) {

		switch (BLUE_RX) {
		case 'F':
			CMDE = AVANT;
			New_CMDE = 1;
			break;

		case 'B':
			CMDE = ARRIERE;
			New_CMDE = 1;
			break;

		case 'L':
			CMDE = GAUCHE;
			New_CMDE = 1;
			break;

		case 'R':
			CMDE = DROITE;
			New_CMDE = 1;
			break;

		case 'D':
			// disconnect bluetooth
			break;

		case 'W':
			CMDE = PARKED;
			New_CMDE = 1;
			break;

		case 'X':
			CMDE = ATT_PARK;
			etat_park = Avancer50;
			flag_prem_passage = 0;
			New_CMDE = 1;

		case 'V':
			flag_mesure = 1;
			break;


		case 'S':
			break;

		default:
			New_CMDE = 0;
		}

		HAL_UART_Receive_IT(&huart3, &BLUE_RX, 1);

	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {

	Dist_ACS_3 = adc_buffer[0] - adc_buffer[5];
	Dist_ACS_4 = adc_buffer[3] - adc_buffer[8];
	Dist_ACS_1 = adc_buffer[1] - adc_buffer[6];
	Dist_ACS_2 = adc_buffer[2] - adc_buffer[7];
	HAL_ADC_Stop_DMA(hadc);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim) {
	static unsigned char cpt = 0;

	if ( htim->Instance == TIM2) {
		cpt++;
		Time++;
		Tech++;

		if (tempo_sonar == 0){
			Time_mesure++;
		}


		switch (cpt) {
		case 1: {
			HAL_GPIO_WritePin(IR3_out_GPIO_Port, IR3_out_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(IR4_out_GPIO_Port, IR4_out_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(IR1_out_GPIO_Port, IR1_out_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(IR2_out_GPIO_Port, IR2_out_Pin, GPIO_PIN_SET);
			break;
		}
		case 2: {
			HAL_ADC_Start_DMA(&hadc1, (uint32_t *) adc_buffer, 8);
			break;
		}
		case 3: {
			HAL_GPIO_WritePin(IR3_out_GPIO_Port, IR3_out_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(IR4_out_GPIO_Port, IR4_out_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(IR1_out_GPIO_Port, IR1_out_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(IR2_out_GPIO_Port, IR2_out_Pin, GPIO_PIN_RESET);
			break;
		}
		case 4: {
			HAL_ADC_Start_DMA(&hadc1, (uint32_t *) adc_buffer, 8);
			break;
		}
		default:
			cpt = 0;
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	static unsigned char TOGGLE = 0;

	if (TOGGLE)
		CMDE = STOP;
	else
		CMDE = START;
	TOGGLE = ~TOGGLE;
	New_CMDE = 1;

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
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
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
