/* USER CODE BEGIN Header */
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h" 
#include "stdlib.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define pi 3.1415
#define p2r pi/1000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
int32_t PosCnt,Cnttmp,speed;
int16_t CountValue=0,RealVel,DesiredSpeed,HILIM,LOLIM;
uint16_t AngPos1, AngPos0,CntVel;
uint8_t PreviousState,pwm,Speedmode,tick=0;	
bool run=false, dir;
float CurPos=0,DesiredPos,CurVel;	
char Rx_indx, Rx_Buffer[20],Rx_data[2];

// thong so ham picontrol
float DesiredVel = 0;
float err, err_reset= 0, uP, uI,uI_p, uOut;


//thong so PI idue khien van toc
float Kp =0.3582; //0.395
float Ki =9.606;  //8.2983 
float Kb =25.147; //0.1
float Ts = 0.005; // thoi gian lay mau



int32_t uHiLim =100;
int32_t uLoLim = 0;




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
int SetVelLow(float CurrentPos, float Pos);
int SetVelMid(float CurrentPos, float Pos, float CurrentVel);
int SetVelHigh(float CurrentPos, float Pos, float CurrentVel);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ham print	uart */
struct __FILE
{
  int handle;
  /* Whatever you require here. If the only file you are using is */
  /* standard output using printf() for debugging, no file handling */
  /* is required. */
};
/* FILE is typedef’d in stdio.h. */
FILE __stdout;
int fputc(int ch, FILE *f) 
{
	
	char tempch;
  HAL_UART_Transmit(&huart1, (uint8_t*)&ch,1,100);
		return ch;
}

//ham PI dieu khien van toc
void PIcontrol(void)
{

 err = DesiredVel - CurVel;
	uI = uI_p + Ki*Ts*err + Kb*Ts*err_reset;
	uI_p = uI;
	uOut= (int)(uP+uI);
	
	//ham chan
	 if(uOut > uHiLim)
	 {
		 err_reset = uHiLim - uOut;
		 uOut = uHiLim;	 
	 }
	 else if(uOut < uLoLim)
	 {
		 err_reset = uLoLim - uOut;
		 uOut = uLoLim;	 
	 }
	 
	 
	 //ham run dong co
	 if(uOut >0)
		 {
			 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
			 __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,uOut);
			 
		 }
		 else if(uOut <0)
		 {
			 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
			 __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,uOut);
			 
		 }
		 else
		 {
			 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
			 __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,00);
			 
			 
		 }
		 
		 
}	
	
	// Ham ngat Uart
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
	{
		uint8_t i;
		if(huart->Instance == USART1) //uart1
		{
				if(Rx_indx==0) {for (i=0;i<20;i++) Rx_Buffer[i] = 0;}
			
		switch(Rx_data[0]) {
            /* dung dong co */
            case 'e':
                run =false;
                break;
            
            /* dong co chay */
            case 'r':
                run = true;
                break;
            case 'b':
//								reset();
								break;
            case 's':
                DesiredPos = atoi(Rx_Buffer);
                memset(Rx_Buffer, 0, sizeof(Rx_Buffer));
                Rx_indx = 0;
                break;
            case 'v':
                DesiredSpeed = atoi(Rx_Buffer); //nhan tu C#, Pos = 10rpm
								DesiredVel = DesiredSpeed*2*3.1415/60; //tuwf rpm sang rad/s
                memset(Rx_Buffer, 0, sizeof(Rx_Buffer));
                Rx_indx = 0;
                break;    
            case '0':
            case '1':
            case '2':
            case '3':
            case '4':
            case '5':
            case '6':
            case '7':
            case '8':
            case '9':
            case '.':
            case '-':
                Rx_Buffer[Rx_indx++] |= Rx_data[0];
                break; 
            default:
                break;
        }
				HAL_UART_Receive_IT(&huart1,(uint8_t*)Rx_data,1);
		}
	}

void EXTI9_5_IRQHandler(void)	// doc encoder	
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */
unsigned char State0;
	State0 = (State0<<1) | HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
	State0 = (State0<<1) | HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6);
	State0 = State0&0x03;
	switch (State0) {
		case 0:
			if(PreviousState==1) CountValue++;
			else CountValue--;
		break;
		case 1:
			if(PreviousState==3) CountValue++;
			else CountValue--;
		break;
		case 2:
			if(PreviousState==0) CountValue++;
			else CountValue--;
		break;
		case 3:
			if(PreviousState==2) CountValue++;
			else CountValue--;
		break;
		}
	PreviousState = State0;
	CntVel++;
	if (CountValue>=2000) {
		CountValue = 0;
		//PosCnt++;
	}
	else if	(CountValue<=-2000) {
		CountValue = 0;
		//PosCnt--;
	}
  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
  /* USER CODE BEGIN EXTI4_IRQn 1 */
}

/**
* @brief This function handles EXTI line3 interrupt.
*/
void EXTI4_IRQHandler(void)	// doc encoder	
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */
	// CHANNEL A
unsigned char State1;
	State1 = (State1<<1) | HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
	State1 = (State1<<1) | HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6);
	State1 = State1&0x03;
	switch (State1) {
		case 0:
			if(PreviousState==1) CountValue++;
			else CountValue--;
		break;
		case 1:
			if(PreviousState==3) CountValue++;
			else CountValue--;
		break;
		case 2:
			if(PreviousState==0) CountValue++;
			else CountValue--;
		break;
		case 3:
			if(PreviousState==2) CountValue++;
			else CountValue--;
		break;
		}
	PreviousState = State1;
	CntVel++;
	if (CountValue>=2000) {
		CountValue = 0;
		//PosCnt++;
	}
	else if	(CountValue<=-2000) {
		CountValue = 0;
		//PosCnt--;
	}
  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  /* USER CODE BEGIN EXTI3_IRQn 1 */

  /* USER CODE END EXTI3_IRQn 1 */
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {	// ngat timer 4 tinh van toc
	if(htim->Instance==TIM4)	// ngat do timer 4	5ms
	{
		//CurPos = PosCnt*2*pi+CountValue*p2r;	// Position calculation
		Cnttmp = CntVel;
		CntVel = 0;
		RealVel = (Cnttmp*60)/(Ts*2000);										//RPM
		CurVel = Cnttmp*2*pi/10;								//rad/s
		PIcontrol();
		return;
	}
	if(htim->Instance==TIM5)
	{
//		ut=5*pi*time;	// ham chay theo thoi gian u(t)=5*pi(t);	// timer 5
//		time+=0.01;	
			tick++;
//			if (run==1) {
//				if(DesiredSpeed<=340) Speedmode = 1;
//				else if((DesiredSpeed>340)&&(DesiredSpeed<750))	Speedmode = 2;
//				else if(DesiredSpeed>=750) Speedmode = 4;
//				}
//			if (run==0){ 
//					pwm = 0;
//					Speedmode =0;
//			}
			if ((run==1)&&(tick==5)){
				tick=0;
				printf("V%d\r \n",RealVel);
				//printf("P%f\r \n",CurPos);
			}
	}
	
}
int SetVelLow(float CurrentPos, float Pos)
{
	HILIM=10,LOLIM=0;
	float uout;
	
	// STUDENTS ADD CODE FOR SLOW SPEED HERE
	
	if (uout>HILIM) uout=HILIM;
	else if(uout<LOLIM) uout=LOLIM;
	return uout;
}
int SetVelMid(float CurrentPos, float Pos, float CurrentVel)
{
	HILIM=20,LOLIM=0;
	float uout;
	
	// STUDENTS ADD CODES FOR MIDDLE SPEED HERE
	
	if (uout>HILIM) uout=HILIM;
	else if(uout<LOLIM) uout=LOLIM;
	return uout;
}
int SetVelHigh(float CurrentPos, float Pos, float CurrentVel)
{
	HILIM=99,LOLIM=0;
	float uout;
	
	// STUDENTS ADD CODES FOR HIGH SPEED HERE
	
	if (uout>HILIM) uout=HILIM;
	else if( uout<LOLIM) uout=LOLIM;
	
	return uout;
}
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);	// khoi tao timer 2
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);	// khoi tao timer 3
	HAL_TIM_Base_Start_IT(&htim4);						// khoi tao timer 4
	HAL_TIM_Base_Start_IT(&htim5);						// khoi tao timer 5
	HAL_UART_Receive_IT(&huart1,(uint8_t*)Rx_data,1);	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 11;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 11;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 23999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 4;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  htim5.Init.Prescaler = 23999;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 9;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
