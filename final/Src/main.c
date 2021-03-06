
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
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define GET_DISTANCE_TIME 3
#define L_OFFSET 0
#define R_OFFSET 0
#define LRCALI_MINVALUE 0.2
#define LRCALI_MINRANGE 0.025
#define LRCALI_MAXVALUE 12
#define LRCALI_MAX_DIFFERENCE_VALUE 1.5
#define LRCALI_PWMWIDTH 700
#define LRCALI_ULPERIOD 7000 // xK = x us
#define LRCALI_SUCCESS_TIME 2
int AFCALI_PWMWIDTH = 500;
#define AFCALI_EX_PWMWIDTH 100

#define A4988_PWM_GRP GPIOA
#define A4988_PWM_PIN GPIO_PIN_1
#define A4988_DIR_GRP GPIOA  //GPIOG
#define A4988_DIR_PIN GPIO_PIN_2  //GPIO_PIN_12
#define LULT_TRIG_GRP GPIOB
#define LULT_TRIG_PIN GPIO_PIN_5
#define LULT_ECHO_GRP GPIOA
#define LULT_ECHO_PIN GPIO_PIN_5
#define RULT_TRIG_GRP GPIOB
#define RULT_TRIG_PIN GPIO_PIN_13
#define RULT_ECHO_GRP GPIOC
#define RULT_ECHO_PIN GPIO_PIN_7
#define DCMOT_PWM_GRP GPIOB
#define DCMOT_PWM_PIN GPIO_PIN_8
#define DCMOT_DIR_GRP GPIOB
#define DCMOT_DIR_PIN GPIO_PIN_14
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void sendMsg(char*);
void tellWifi(char*);
void wifiInit();
void LRCalibrate(double,double);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

//str to int
int str2int(char* str){
	int ans=0;
	for(int i=1;i<strlen(str)-1;i++){
		ans*=10;
		ans+=(str[i]-'0');
	}
	return ans;
}

// Sending msg to UART3
void sendMsg(char* str){
	HAL_UART_Transmit(&huart3,str,strlen(str),0xffff);
}

// Sending msg to UART6
void tellWifi(char* str){
	HAL_UART_Transmit(&huart6,str,strlen(str),0xffff);
}

void sendMsgThrWifi(char* str){
  char tosend[20]={0};
  sprintf(tosend,"AT+CIPSEND=0,%d\r\n",strlen(str));
  tellWifi(tosend);
  HAL_Delay(500);
  sprintf(tosend,"%s\r\n",str);
  tellWifi(tosend);
}

// Handling UART Receive
char rx6_buf[100],rx6_data,rx3_buf[100],rx3_data,state;
int rx6_index=0,rx3_index=0;
char state=0;
int recvNum=0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance==USART3){
		if(rx3_data=='U'){
			HAL_GPIO_WritePin(GPIOG,GPIO_PIN_12,1);
			HAL_TIM_PWM_Start_IT(&htim5,TIM_CHANNEL_2);
		}else if(rx3_data=='D'){
			HAL_GPIO_WritePin(GPIOG,GPIO_PIN_12,0);
			HAL_TIM_PWM_Start_IT(&htim5,TIM_CHANNEL_2);
		}else{
			if(rx3_index==0){
				for(int i=0;i<100;i++){
					rx3_buf[i]=0;
				}
			}
			rx3_buf[rx3_index++]=rx3_data;
			if(rx3_data=='\n'){
				rx3_index=0;
				char tosend[200]={0};
				sprintf(tosend,"[STM] Send to ESP : %s",rx3_buf);
				sendMsg(tosend);
				tellWifi(rx3_buf);
				sendMsg("[STM] Done\r\n");
			}
			HAL_UART_Receive_IT(&huart3,&rx3_data,1);
		}

	}
  if(huart->Instance==USART6){
	  	  if(rx6_index==0){
	  			for(int i=0;i<100;i++){
	  				rx6_buf[i]=0;
	  			}
	  	  }
	  	  if(rx6_data=='$' || rx6_index>0){
	  		  rx6_buf[rx6_index++]=rx6_data;
	  	  }
	  	  if(rx6_data=='%'){

			    char tosend[10]={0};
			    sprintf(tosend,"[ESP] %s\r\n",rx6_buf);
			    sendMsg(tosend);
			    if(rx6_index==3){
			    	state=rx6_buf[rx6_index-2];
			    }else{
			    	state='R';
			    	recvNum=str2int(rx6_buf);
			    }
			    rx6_index=0;
	  	  }
	  	HAL_UART_Receive_IT(&huart6,&rx6_data,1);
	}
}

// Handling wifi initialize
void wifiInit(){
	tellWifi("AT+CWMODE=2\r\n");
	HAL_Delay(500);
	tellWifi("AT+CIPMUX=1\r\n");
	HAL_Delay(500);

	tellWifi("AT+CIPSERVER=1,66\r\n");
	HAL_Delay(500);
	tellWifi("AT+CIPSTO=0\r\n");
	HAL_Delay(500);
}

// Handling LR calibration
int inta=0,intb=0,floata=0,floatb=0;
double distancea=0,distanceb=0;
double distancea_final=0,distanceb_final=0;
int ultrasonic=0;  // 0 for Ultrasonic1 , 1 for ultrasonic2 , 2 for motor
int LRCALI_STATE=0;
int motorMoveDone=0;
void LRCalibrate(double dl,double dr){
	  char tosend[50]={0};
	  inta=(int)dl;
	  floata=(int)((dl-inta)*100);
	  intb=(int)dr;
	  floatb=(int)((dr-intb)*100);
	  if(dl*LRCALI_MAXVALUE>dr && dr*LRCALI_MAXVALUE>dl){
		  if( (dr-dl)>LRCALI_MINVALUE ||(dl-dr)>LRCALI_MINVALUE ){
			  if(dl>dr){
				  sprintf(tosend,"> %d.%02d , %d.%02d -> Turn R\r\n",inta,floata,intb,floatb);
				  sendMsg(tosend);
				  HAL_GPIO_WritePin(DCMOT_DIR_GRP,DCMOT_DIR_PIN,1);
				  sprintf(tosend,"> > ReadPin: %d\r\n",HAL_GPIO_ReadPin(DCMOT_DIR_GRP,DCMOT_DIR_PIN));
				  sendMsg(tosend);
				  LRCALI_STATE=0;
				  motorMoveDone=0;
				  HAL_TIM_PWM_Start_IT(&htim4,TIM_CHANNEL_3);
			  }else if(dl<dr){
				  sprintf(tosend,"> %d.%02d , %d.%02d -> Turn L\r\n",inta,floata,intb,floatb);
				  sendMsg(tosend);
				  HAL_GPIO_WritePin(DCMOT_DIR_GRP,DCMOT_DIR_PIN,0);
				  sprintf(tosend,"> > ReadPin: %d\r\n",HAL_GPIO_ReadPin(DCMOT_DIR_GRP,DCMOT_DIR_PIN));
				  sendMsg(tosend);
				  motorMoveDone=0;
				  LRCALI_STATE=0;
				  HAL_TIM_PWM_Start_IT(&htim4,TIM_CHANNEL_3);
			  }
		  }else{
			  sprintf(tosend,"> %d.%02d , %d.%02d -> DONE\r\n",inta,floata,intb,floatb);
			  sendMsg(tosend);
			  LRCALI_STATE++;
			  motorMoveDone=1;
			  //return;
		  }
	  }else{
		  sprintf(tosend,"> %d.%02d , %d.%02d -> X\r\n",inta,floata,intb,floatb);
		  sendMsg(tosend);
	  }

	  ultrasonic=0;
}

// Handling Ultrasonic Echo InputCapture
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim){
	if(htim->Instance==TIM2){
		if(HAL_GPIO_ReadPin(LULT_ECHO_GRP,LULT_ECHO_PIN)==1){
			__HAL_TIM_SET_COUNTER(&htim2,0);
		}else{
			int cnt=__HAL_TIM_GET_COUNTER(&htim2);
			distancea=(cnt/(double)116);
			distancea+=L_OFFSET;
			ultrasonic=1;
		}
	}
	if(htim->Instance==TIM3){
		if(HAL_GPIO_ReadPin(RULT_ECHO_GRP,RULT_ECHO_PIN)==1){
			__HAL_TIM_SET_COUNTER(&htim3,0);
		}else{
			int cnt=__HAL_TIM_GET_COUNTER(&htim3);
			distanceb=(cnt/(double)116);
			distanceb+=R_OFFSET;
			ultrasonic=2;
		}
	}
}

// A 10 us timer for ultrasonic interval
int tim1Count=LRCALI_ULPERIOD;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance==TIM1){
		if(tim1Count<LRCALI_ULPERIOD){
			tim1Count++;
		}else{
			HAL_TIM_Base_Stop_IT(&htim1);
			if(ultrasonic==0){
				if(HAL_GPIO_ReadPin(LULT_TRIG_GRP,LULT_TRIG_PIN)==0){
					HAL_GPIO_WritePin(LULT_TRIG_GRP,LULT_TRIG_PIN,1);
					HAL_TIM_Base_Start_IT(&htim1);
				}else{
					tim1Count=0;
					HAL_GPIO_WritePin(LULT_TRIG_GRP,LULT_TRIG_PIN,0);
				}
			}else if(ultrasonic==1){
				if(HAL_GPIO_ReadPin(RULT_TRIG_GRP,RULT_TRIG_PIN)==0){
					HAL_GPIO_WritePin(RULT_TRIG_GRP,RULT_TRIG_PIN,1);
					HAL_TIM_Base_Start_IT(&htim1);
				}else{
					tim1Count=0;
					HAL_GPIO_WritePin(RULT_TRIG_GRP,RULT_TRIG_PIN,0);
				}
			}
		}
	}
}

// Handling PWM Pulse
int pwm5Count=0,pwm4Count=0,a4988MoveDone=0,accumMove=0;
char focusStepState='N'; //N for normal, E for extreme
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){
  //DC MOTOR PWM
  if(htim->Instance==TIM4){
		if(pwm4Count++>=LRCALI_PWMWIDTH){
			HAL_TIM_PWM_Stop_IT(&htim4,TIM_CHANNEL_3);
			pwm4Count=0;
			motorMoveDone=1;
		}
	}
  //A4988 PWM
  if(htim->Instance==TIM5){
	  if(focusStepState=='N'){
		if(pwm5Count++>=AFCALI_PWMWIDTH){
			HAL_TIM_PWM_Stop_IT(&htim5,TIM_CHANNEL_2);
			pwm5Count=0;
			accumMove+=1;
			a4988MoveDone=1;
		}
	  }else{
		if(pwm5Count++>=AFCALI_EX_PWMWIDTH){
			HAL_TIM_PWM_Stop_IT(&htim5,TIM_CHANNEL_2);
			pwm5Count=0;
			a4988MoveDone=1;
			focusStepState='N';
			sendMsg("[STM] Recover Done\r\n");
		}
	  }
	}
}

char pwmMode='N'; //N for normal , I for IT
char focusState='N'; // N for normal, T for top, B for Bottom

// TODO Handling TOP BUTTOM
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin==GPIO_PIN_13){
		//top
		if(pwmMode=='N'){
			HAL_TIM_PWM_Stop_IT(&htim5,TIM_CHANNEL_2);
		}else{
			HAL_TIM_PWM_Stop(&htim5,TIM_CHANNEL_2);
		}
		focusStepState='E';
		HAL_GPIO_WritePin(A4988_DIR_GRP,A4988_DIR_PIN,0);
		HAL_TIM_PWM_Start_IT(&htim5,TIM_CHANNEL_2);
		pwm5Count=0;
		focusState='T';
		a4988MoveDone=1;
	    sendMsg("[STM] <<TOP>>\r\n");

	}else if(GPIO_Pin==GPIO_PIN_15){
		//bottom
		if(pwmMode=='N'){
			HAL_TIM_PWM_Stop_IT(&htim5,TIM_CHANNEL_2);
		}else{
			HAL_TIM_PWM_Stop(&htim5,TIM_CHANNEL_2);
		}
		HAL_GPIO_WritePin(A4988_DIR_GRP,A4988_DIR_PIN,1);
		focusStepState='E';
		HAL_TIM_PWM_Start_IT(&htim5,TIM_CHANNEL_2);
		pwm5Count=0;
		focusState='B';
		a4988MoveDone=1;
		sendMsg("[STM] <<BOTTOM>>\r\n");
	}
}

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART6_UART_Init();
  MX_TIM5_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  sendMsg("[STM] Initializing Board...\r\n");
  HAL_UART_Receive_IT(&huart6,&rx6_data,1);
  HAL_UART_Receive_IT(&huart3,&rx3_data,1);
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,1);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,1);
  TIM4->CCR3=0;
  sendMsg("[STM] Board Ready\r\n");
  HAL_Delay(500);

  sendMsg("[STM] Initializing Wifi...\r\n");
  HAL_Delay(500);

  //wifiInit();

	tellWifi("AT+CWMODE=2\r\n");
	HAL_Delay(500);
	tellWifi("AT+CIPMUX=1\r\n");
	HAL_Delay(500);
	tellWifi("AT+CIPSTO=0\r\n");
	HAL_Delay(500);
	tellWifi("AT+CIPSERVER=1,66\r\n");
	HAL_Delay(500);


  sendMsg("[STM] Wifi Ready\r\n");
  HAL_Delay(500);

  sendMsg("[STM] Wait for APP. (Please connect to AP:Projector)");



  while(state!='S'){
	  HAL_Delay(100);
  }

  //LR Calibration
  HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_2);

  sendMsg("[STM] LR-Calibration START\r\n");
  HAL_Delay(500);
  int LR_done=0;
  while(LRCALI_STATE != LRCALI_SUCCESS_TIME){
	  distancea_final=0;
	  distanceb_final=0;
	  for(int i=0;i<GET_DISTANCE_TIME;i++){
		  char tosend[50]={0};

		  HAL_TIM_Base_Start_IT(&htim1);
		  while(ultrasonic!=1){
			  HAL_Delay(100);
		  }
		  HAL_TIM_Base_Start_IT(&htim1);
		  while(ultrasonic!=2){
			  HAL_Delay(100);
		  }
		  ultrasonic=0;
		  inta=(int)distancea;
		  floata=(int)((distancea-inta)*100);
		  intb=(int)distanceb;
		  floatb=(int)((distanceb-intb)*100);
		  if( distancea-distanceb<LRCALI_MAX_DIFFERENCE_VALUE && distanceb-distancea<LRCALI_MAX_DIFFERENCE_VALUE ){
			  sprintf(tosend,"[ULT] Getting distance... %d.%02d,%d.%02d -> Write\r\n",inta,floata,intb,floatb);
			  //sendMsg(tosend);
			  distancea_final+=distancea;
			  distanceb_final+=distanceb;
		  }else{
			  sprintf(tosend,"[ULT] Getting distance... %d.%02d,%d.%02d -> Pass\r\n",inta,floata,intb,floatb);
			  //sendMsg(tosend);
			  i--;
		  }
	  }
	  ultrasonic=2;
	  distancea_final/=GET_DISTANCE_TIME;
	  distanceb_final/=GET_DISTANCE_TIME;
	  LRCalibrate(distancea_final,distanceb_final);
	  while(!motorMoveDone){
		  HAL_Delay(100);
	  }
	  HAL_Delay(50);
  }

  HAL_Delay(1000);
  sendMsg("[STM] LR-Calibration FINISH\r\n");

  //TODO AF communication and flow
  sendMsg("[STM] Focus-Calibration START\r\n");
  //move to bottom
  HAL_GPIO_WritePin(A4988_DIR_GRP,A4988_DIR_PIN,0);
  pwmMode='N';
  HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);
  while(focusState!='B'){
	  HAL_Delay(100);
  }

  //move upward and send 'N' after moving until TOP (send 'T')
  HAL_Delay(500);
  sendMsgThrWifi("N\n");

  HAL_GPIO_WritePin(A4988_DIR_GRP,A4988_DIR_PIN,1);
  pwmMode='I';
  sendMsg("[STM] Start Cruising\r\n");

  while(1){
	  while(state!='U' && state!='R'){
		  HAL_Delay(10);
	  }
	  if(state=='R'){
		  break;
	  }
	  sendMsg("[STM] Moving UPWARD for one step\r\n");
	  a4988MoveDone=0;
	  HAL_TIM_PWM_Start_IT(&htim5,TIM_CHANNEL_2);
	  while(a4988MoveDone==0){
		  HAL_Delay(10);
	  }
	  sendMsg("[STM] > Done\r\n");
	  state=0;
	  HAL_Delay(500);
	  if(focusState!='T'){
		  sendMsgThrWifi("N\n");
	  }else{
		  sendMsgThrWifi("T\n");
	  }
  }
  //TODO move downward base on recvNum

  char tosend[50]={0};
  sprintf(tosend,"[STM] Start moving %d step downward\r\n",recvNum);
  sendMsg(tosend);
  HAL_GPIO_WritePin(A4988_DIR_GRP,A4988_DIR_PIN,0);
  for(int i=0;i<recvNum;i++){
	  a4988MoveDone=0;
	  HAL_TIM_PWM_Start_IT(&htim5,TIM_CHANNEL_2);
	  while(a4988MoveDone==0){
		  HAL_Delay(10);
	  }
  }
  state=0;
  sprintf(tosend,"[STM] Moving %d step downward Done\r\n",recvNum);
  sendMsgThrWifi("K\n");


  //TODO Fine tune
  AFCALI_PWMWIDTH=100;
  while(1){
	  while(state==0){
		  HAL_Delay(10);
	  }
	  switch(state){
	  case 'U':
		  state=0;
		  a4988MoveDone=0;
		  pwmMode='I';
		  HAL_GPIO_WritePin(A4988_DIR_GRP,A4988_DIR_PIN,1);
		  HAL_TIM_PWM_Start_IT(&htim5,TIM_CHANNEL_2);
		  while(a4988MoveDone==0){
			  HAL_Delay(10);
		  }
		  sendMsg("[STM] FineTune- Upward Done\r\n");
		  HAL_Delay(500);
		  sendMsgThrWifi("K\n");
		  break;
	  case 'D':

		  state=0;
		  a4988MoveDone=0;
		  pwmMode='I';
		  HAL_GPIO_WritePin(A4988_DIR_GRP,A4988_DIR_PIN,0);
		  HAL_TIM_PWM_Start_IT(&htim5,TIM_CHANNEL_2);
		  while(a4988MoveDone==0){
			  HAL_Delay(10);
		  }
		  sendMsg("[STM] FineTune- Downward Done\r\n");
		  HAL_Delay(500);
		  sendMsgThrWifi("K\n");
		  break;
	  //TODO top bottom finish
	  case 'T':

		  state=0;
		  pwmMode='N';
		  a4988MoveDone=0;
		  HAL_GPIO_WritePin(A4988_DIR_GRP,A4988_DIR_PIN,1);
		  HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);
		  while(a4988MoveDone==0){
			  HAL_Delay(10);
		  }
		  sendMsg("[STM] FineTune- Top Done\r\n");
		  sendMsgThrWifi("K\n");
		  break;
	  case 'B':

		  state=0;
		  pwmMode='N';
		  a4988MoveDone=0;
		  HAL_GPIO_WritePin(A4988_DIR_GRP,A4988_DIR_PIN,0);
		  HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);
		  while(a4988MoveDone==0){
			  HAL_Delay(10);
		  }
		  sendMsg("[STM] FineTune- Bottom Done\r\n");
		  sendMsgThrWifi("K\n");

		  break;
	  case 'F':
		  sendMsg("[STM] Auto Focus Fine Tune Finish\r\n");
		  state=0;
		  break;
	  }

  }

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 15;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffffffff;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xffff;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 15;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 90;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim4);

}

/* TIM5 init function */
static void MX_TIM5_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 15;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1000;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim5);

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART6 init function */
static void MX_USART6_UART_Init(void)
{

  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5|GPIO_PIN_9|GPIO_PIN_13|GPIO_PIN_12 
                          |GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB5 PB9 PB13 PB12 
                           PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_9|GPIO_PIN_13|GPIO_PIN_12 
                          |GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

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
