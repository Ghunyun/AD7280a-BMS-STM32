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
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ADS1015_ADS1115.h"
#include "math.h"
#include "ad7280a_stm32.h"
#include "string.h"
#include "eeprom.h"
#include "stdlib.h"
#include "stdio.h"
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
#define		VTh				0.025		//Battery voltage treshold in percentage of battery average voltage to detect cell that have to be in balancing state (charging mode)
#define		VTh_CB				0.05		//Treshold for cell balancing
#define		Vhigh				4100		//High Voltage limit
#define		Vlow				2800		//Low Voltage limit
#define         Vth_high                        4000
#define         Vth_low                         2800
#define         Thigh                           55              //High temp limit
#define		CTh				0.1		//Current treshold in percentage of battery capacity to detect in idle, charge or discharge condition
#define		packCapacity			24		//Battery Capacity in Ah
#define		discharge_current_limit		-300		//Discharge current limit in Ampere
#define		charge_current_limit		8
#define		THERMISTORNOMINAL		10000
#define		TEMPERATURENOMINAL		25
#define		BCOEFFICIENT			3974
#define		SERIESRESISTOR			10000
#define         consecutiveCount                20      //How many consecutive count error to trigger fault
//#define		BATTERY_SERIES_CON		24
//#define 		ARM_MATH_CM4


float					thermistor_resistance[BATTERY_SERIES_CON];
uint8_t 					SPI_Ready = 1, SPI_TX = 1, SPI_RX = 1;
uint8_t 					readBattery = 0;
uint16_t 				ADC_Val = 0;
uint16_t                                ADC_Val_Ref = 0;
float                                   vRef;
uint16_t 				sample_num;
float                                   Vtotal = 0;
float 					out[3]={0,0,0},in[3]={0,0,0};
float 					ADC_Filtered;
float 					current[2], mAh, Ah, accumulator;
uint16_t				capacity = 16200;
float                                   vADC;

float 					Vmax,Vmin,Vavg,Vtresh;
uint8_t					Cell_balancer[BATTERY_SERIES_CON],CB[BATTERY_SERIES_CON/6][12], lastCB[BATTERY_SERIES_CON/6][12], packState=0, lastPackState=0, CBChange;
uint8_t					packUnbalanced = 0,lastPackUnbalanced = 0;
uint8_t					fully_discharged=0,last_fully_discharged=0,fully_charged=0;

extern float         			cellVoltage[BATTERY_SERIES_CON];
extern float         			auxADC[BATTERY_SERIES_CON];
float							temperature[BATTERY_SERIES_CON];

float					VarDataCap = 0;
uint16_t 				VarValue,VarDataTmp = 0;

uint8_t					startUp;
uint16_t 				telo1, telo2;
char					textBuf[511];

uint8_t									DschCtrl, ChCtrl;
uint8_t                                 DisCtrl, CharCtrl;
uint8_t                                 uv_stat, ov_stat;
uint8_t                                 ot_stat;
uint8_t                                 fault1, faultChar,faultall;
int16_t                                 kampret;

uint16_t                                fault0;

ADS1xx5_I2C 							i2c;
int16_t 								adc1;
int 									millivolt;
char 									snum[7];  // Sufficient space for millivolt values (-4096 to 4096)

int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void LPF_current_sensor(void);
void current_calc(void);
void calcVBatCond(void);
void temperature_calc(void);
void printPlot(void);
void chargingMode(void);
void Error_Handler(void);
void calcUV(void);
void calcOV(void);
void calcOT(void);
void faultDet(void);
void faultDetChar(void);
void faultTotal(void);
void q31_to_u16_buffer(int32_t inval, uint16_t* pBuffer);
void calcVtotal(void);
void Get_ADC_Value(void);
void CANTransmit(void);
void ReadADS1115(void);
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_TIM9_Init();
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1,
      CAN_IT_RX_FIFO0_MSG_PENDING |
      CAN_IT_TX_MAILBOX_EMPTY);

  accumulator = capacity;
  AD7280A_Init();

  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim9);
  startUp = 1;
  ADS1115(&i2c, &hi2c1, ADS_ADDR_GND); // Or ADS1015(&i2c, &hi2c1, ADS_ADDR_GND);
  ADSsetGain(&i2c, GAIN_ONE);

  for(int i = 0; i < 20; i++){
      Get_ADC_Value();
      ADC_Val_Ref = ADC_Val_Ref + ADC_Val;
    }
     vRef = ((ADC_Val_Ref/20)/4095.)*3.3;

    if(vRef < 2.6 && vRef > 2.4){
     vRef = vRef;
    }
    else{ vRef = 2.5;}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	    if(readBattery == 1)
	    {
	      AD7280A_Convert_Read_All(BATTERY_SERIES_CON);							// read all battery voltage and auxilary adc

	      /* // buat testing balancing
	      if(ctrcb == 5) {
	        ctrcb = 0;
	        cbcb++;
	      }
	      ctrcb++;

	      if(cbcb < 12){
	        cellVoltage[cbcb] = 3580;
	      }
	      //if(cbcb == 9) cbcb = 8;
	      else if(cbcb >= 12) cbcb = 6;
	      */

	      HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_SET);

	      Get_ADC_Value();
	     // ReadADS1115();
	      Vtotal = 0;
	      calcVtotal();
	      temperature_calc();
	      calcVBatCond();											// calc all Vbat condition include battery cell need to be balanced
	      calcUV();
	      calcOV();
	      calcOT();
	      faultDet();
	      faultDetChar();
	      faultTotal();
	      if(packState ==  1 && lastPackState != 1 ){					//Charging state detected when the state before is Idle state	=>> deteksi kondisi dari idle/discharging ke charging
		if(packUnbalanced == 1)									//unbalanced pack condition detection
		{
		  //printf("Cond 1\n");
		  chargingMode();										//Balancing activation if pack is unbalanced
		}
		else
		{
		   //printf("Cond 2\n");
		   AD7280A_CBOffAll();									//Turn off all balancing output if pack is already in balanced condition
		}
	      }
	      else if(packState == 1 && lastPackState ==1){				//detection in charging state	=>> deteksi kondisi pada saat charging
		if(packUnbalanced == 0 && lastPackUnbalanced == 1)		//when in charging state, the pack goes to balanced condition
		{
	          //printf("Cond 3\n");
		  AD7280A_CBOffAll();									//Turn off all balancing output
		}
		else if(packUnbalanced == 1 && lastPackUnbalanced ==0)
		{
		  //printf("Cond 4\n");
		  //AD7280A_CBxTim(2);
		  //AD7280A_CBOutAll(12);
		  chargingMode();
		}
		else if(packUnbalanced == 1 && lastPackUnbalanced == 1 && CBChange ==1)
		{
		  //printf("Cond 5\n");
		  chargingMode();
		}
	      }
	      else if(packState == 0 && lastPackState == 1){				//when go to Idle state from charging state	=>> deteksi kondisi charging ke idle
		//printf("Cond 6\n");
		AD7280A_CBOffAll();										//turn off all balancing output
	      }

	      //chargingMode();
	      printPlot();
	      CANTransmit();
	      //sendCANMessage();

	      HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_RESET);

	      if(packState == 1)
	      {
		//HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
		//HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
	      }
	      else if(packState ==2)
	      {
		//HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
		//HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
	      }
	      else
	      {
		//HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
		//HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
	      }

	      //if(fully_discharged==1)
		//HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_RESET);
	      //else
		//HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_SET);

	      //if(fully_charged==1)
		//HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
	      //else
		//HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);

	      lastPackState = packState;
	      startUp = 0;
	      fault0 = fault1;


	      if(startUp == 0 && packState !=0)
		{
//			if(EE_WriteVariable(VirtAddVarTab[1], (uint16_t) capacity) != HAL_OK)
//	 		{
//			  Error_Handler();;
//	 		}
		}

	    }

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    // Read the received message
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) {
        if (rxHeader.IDE == CAN_ID_EXT) {
            // Extended ID received
            if (rxHeader.ExtId == 0x18FF50E5) {
                printf("Received Charger Feedback: 0x%X | Data: ", rxHeader.ExtId);
                for (int i = 0; i < rxHeader.DLC; i++) {
                    printf("%02X ", rxData[i]);
                }
                printf("\n");
            }
        } else if (rxHeader.IDE == CAN_ID_STD) {
            // Standard ID received
            if (rxHeader.StdId >= 18 && rxHeader.StdId <= 41) {
                printf("Received Standard ID: 0x%X | Data: ", rxHeader.StdId);
                for (int i = 0; i < rxHeader.DLC; i++) {
                    printf("%02X ", rxData[i]);
                }
                printf("\n");
            }
        }
    }
}


void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
    //printf("CAN Transmission Successful\r\n");
}

void ReadADS1115(void){
    adc1 = ADSreadADC_SingleEnded(&i2c, 1);  // Read ADC channel 1
    millivolt = adc1 * 4.096 * 1000 / 32768; // Convert to millivolts

    itoa(millivolt, snum, 10);  // Convert int to string

    // Example: Send over serial or display
    //printf("ADC1: %d mV\n", millivolt);
}

void CANTransmit(void) {

	// CAN CURRENT
    //char strNumber[20];       // Buffer to hold the converted float as a string
    float currentC = current[0];
    uint8_t TxData[2];  // Data array to store firstByte and secondByte
    // Step 1: Take absolute value and round to the nearest integer
    int currentCAN = (int) round(fabs(currentC));

    // Step 2: Extract tens and ones place
    uint8_t currentfirstByte = (uint8_t) floor(currentCAN / 10.0);
    uint8_t currentsecondByte = (uint8_t) (currentCAN % 10);

//    // Step 3: Extract tens and ones place
//    uint8_t currentfirstByte = (uint8_t)floor((double)currentCAN / 10);
//    uint8_t currentsecondByte = (uint8_t)(currentCAN % 10); // Ones place


//    int scaledData = (int)current[0]; // Scale the data if necessary
//
//    if (scaledData < 255 || scaledData > -255) {
//        TxData[0] = 0;
//        TxData[1] = (uint8_t)floor(scaledData / 100.0); // Integer part with floor rounding
//        TxData[2] = (uint8_t)(scaledData % 100); // Decimal part
//    } else {
//        TxData[0] = (uint8_t)(scaledData / 10000); // First digit
//        TxData[1] = (uint8_t)floor(scaledData / 100.0); // Integer part
//        TxData[2] = (uint8_t)(scaledData % 100); // Decimal part
//    }


	    // Debug output
//	    printf("MilliCurrent: %.6f mA\n", milliCurrent);
//	    printf("First three significant digits: %s\n", firstThreeDigits);
//	    printf("currentCAN: %d\n", currentCAN);
//	    printf("First Byte: %d\n", currentfirstByte);
//	    printf("Second Byte: %d\n", currentsecondByte);

	    CAN_TxHeaderTypeDef TxHeader;

	    uint8_t ChargerData[8];
	    uint32_t TxMailbox;

	    // Set CAN message ID and properties
	    TxHeader.StdId = 56;       // Standard ID 0x56
	    TxHeader.ExtId = 0;          // Not using extended ID
	    TxHeader.IDE = CAN_ID_STD;   // Standard ID mode
	    TxHeader.RTR = CAN_RTR_DATA; // Data frame
	    TxHeader.DLC = 2;            // Sending 2 bytes

	    // Assign data bytes
	    TxData[0] = currentfirstByte;
	    TxData[1] = currentsecondByte;

	    // Send CAN message
	    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
	        // Handle error
	        printf("CAN Transmission Failed!\n");
	    } else {
//	        printf("CAN Message Sent: ID=0x56, Data=[%d, %d]\n", currentfirstByte, currentsecondByte);
	    }


	    TxHeader.StdId = 0x1806E5F4;       // Standard ID 0x56
	    TxHeader.ExtId = 0;          // Not using extended ID
	    TxHeader.IDE = CAN_ID_STD;   // Standard ID mode
	    TxHeader.RTR = CAN_RTR_DATA; // Data frame
	    TxHeader.DLC = 8;            // Sending 2 bytes

	    ChargerData[0] = 0x0F;
	    ChargerData[1] = 0xA0;
	    ChargerData[2] = 0x00;
	    ChargerData[3] = 0x3C;
	    ChargerData[4] = 0;
	    ChargerData[5] = 0;
	    ChargerData[6] = 0;
	    ChargerData[7] = 0;

	    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, ChargerData, &TxMailbox) != HAL_OK) {
	        // Handle error
	        printf("CAN Transmission Failed!\n");
	    } else {
//	        printf("CAN Message Sent: ID=0x56, Data=[%d, %d]\n", currentfirstByte, currentsecondByte);
	    }
}



void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
  SPI_TX = 1;
  SPI_Ready = 1;
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
  SPI_RX = 1;
  SPI_Ready = 1;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
  SPI_TX = 1;
  SPI_RX = 1;
  SPI_Ready = 1;
}

void Get_ADC_Value(void)
{
  HAL_ADC_Start(&hadc1);

  HAL_ADC_PollForConversion(&hadc1, 100);

  ADC_Val = HAL_ADC_GetValue(&hadc1);

  LPF_current_sensor();

  if(startUp == 0)
  	current_calc();

  if(current[0] >= packCapacity*CTh)
    packState = 1;					//Pack in charging condition
  else if (current[0] <= -(packCapacity*CTh))
    packState = 2;					//Pack in discharging condition
  else
    packState = 0;					//Pack in idle condition

  if(packState !=0)
  {
//        uint16_t kampret1 = EE_ReadVariable(VirtAddVarTab[1], &capacity);
//  	mAh = ((current[0])/(3.6));
//  	capacity = capacity + (uint16_t)mAh;
//        if(kampret1 - capacity >= 500 || kampret1 - capacity <= -500){
//        EE_WriteVariable(VirtAddVarTab[1], (uint16_t) capacity);
//        }

  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  ADC_Val = HAL_ADC_GetValue(&hadc1);
  LPF_current_sensor();
  if(startUp ==0)
  	current_calc();

  if(current[0] >= packCapacity*CTh)
    packState = 1;					//Pack in charging condition
  else if (current[0] <= -(packCapacity*CTh))
    packState = 2;					//Pack in discharging condition
  else
    packState = 0;					//Pack in dile condition
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM3)
  {
  	//HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
	readBattery = 1;
	telo2 = telo1;
	telo1 = 0;
  }
  if(htim->Instance == TIM9)
  {
 	 HAL_ADC_Start_IT(&hadc1);
    	 //current[1] = 0;
	 telo1++;
	 //HAL_GPIO_TogglePin(LD6_GPIO_Port, LD6_Pin);
  }
}

void LPF_current_sensor(void){
  //in[0] = ADC_Val*0.020083365564211236;	//1khz
  //in[0] = ADC_Val*0.20657208382614792;		//100hz

  //in[0] = ADC_Val*0.046131802093312926;		//250hz(20)

  //out[0] = in[0] + 2*in[1] + in[2] + 1.5610180758007182*out[1] -  0.64135153805756318*out[2];	//1khz
  //out[0] = in[0] + 2*in[1] + in[2] + 0.36952737735124147*out[1] -   0.19581571265583314*out[2];	//100hz
  //out[0] = in[0] + 2*in[1] + in[2] + 1.3072850288493234*out[1] - 0.49181223722257528*out[2];	//250hz(20)

  out[0] = (in[0]*in[0]) + 2*in[1] + 1 / ( ((in[0]*in[0]) - (1.3072850288493234*in[1]) + 0.49181223722257528));

  in[1] = in[0];
  in[2] = in[1];
  out[1] = out[0];
  out[2] = out[1];
  ADC_Filtered = out[0];
}

void calcVtotal(void){
  for(int i = 0; i < BATTERY_SERIES_CON; i++){
    Vtotal += cellVoltage[i];
  }
  Vtotal = Vtotal/1000;
}
void current_calc(void){
  //current = (ADC_Filtered-2012)*0.00976801;						//perhitungan matematis
  //current[0] = ((0.013083623564625*ADC_Filtered) - 26.2403127904514)*(-1);		//regresi linear
  //current[0] = ((0.013083623564625*ADC_Filtered) - 26.5103127904514)*(-1);		//regresi linear BMS Arjuna
  //current[0] = ((0.012960749*ADC_Filtered) - 26.03771706)*(-1)+0.1;


  //current[0] = (ADC_Filtered - ADC_REFF)*0.375; // tambahan untuk testing

  //currentx = (ADC_Val - ADC_REFF)*0.375; // tambahan untuk testing
  //current[0] = currentx;

  //current[0] = 0.0309*ADC_Filtered - 549.78;

	vADC = (ADC_Val/4095.)*3.3;
  //current[0] = (vADC - 2.788)/0.00666;
  //current[0] = (150*millivolt) - 420;                                                  //regresi linear PIHER HCSP-1BS-0300
	current[0] = (150*vADC) - 420;                                                  //regresi linear PIHER HCSP-1BS-0300
  //current[0] = -10;

  //Ah = ((current[0]-current[1])/2)/900;
  if(current[0] <= discharge_current_limit || fully_discharged == 1)
  {
    //HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_RESET);
    DschCtrl = 0;
  }
  else
  {
    //HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_SET);
    DschCtrl = 1;
  }

  if(current[0] >= charge_current_limit || fully_charged ==1)
  {
    //HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);
    ChCtrl = 0;
  }
  else
  {
    //HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
    ChCtrl = 1;
  }

  current[1] = current[0];
}

void temperature_calc(void){
  float 	Rt;
  int	 	R = 10000;
  float 	logRt;
  float 	c1  = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;
  float		temp_c;

  for(int i=0; i<BATTERY_SERIES_CON; i++){
    temp_c=0;
    Rt = R*(4096 / auxADC[i] - 1);
    //printf("data aux[%d]= %0.f\n",i, auxADC[i]);
    temp_c = (4096/auxADC[i] - 1);
    thermistor_resistance[i] = SERIESRESISTOR /temp_c;
    //printf("resistance[%d]= %0.f\n",i, thermistor_resistance[i]);

    float steinhart=0;
    steinhart=thermistor_resistance[i]/THERMISTORNOMINAL;
    steinhart=log(steinhart);
    steinhart=steinhart/BCOEFFICIENT;
    steinhart=steinhart+1/(TEMPERATURENOMINAL + 273.15);
    steinhart=1/steinhart;
    temperature[i]=steinhart - 273.15 ;
    //logRt = log(Rt);
    //temperature[i] = (1 /  (c1 + c2*logRt + c3*logRt*logRt*logRt)) - 273.15;
  }
}

void calcUV(void) {
  uv_stat=0;
  DisCtrl=0;
  for(int i=0; i<BATTERY_SERIES_CON;i++)
  {
    if(cellVoltage[i] <= Vlow)
    {
      DisCtrl = DisCtrl | 1;
      uv_stat = uv_stat | 1;
    }
    else
    {
      DisCtrl = DisCtrl | 0;
      uv_stat = uv_stat | 0;
    }
  }
  if(uv_stat == 1) {
    HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
  }
  else{
    HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
  }
  //printf("Variable uv_stat = %d\n", uv_stat);

  if(uv_stat == 0)
  {
   for(int i=0; i<BATTERY_SERIES_CON; i++)
   {
    if(cellVoltage[i] > Vth_low)
    {
      //eepromUV = eepromUV | 0;
//      eepromUV = 0;
    }
    else
    {
      //eepromUV = eepromUV | 1;
//      eepromUV = 1;
    }
   }
  }
  else if(uv_stat == 1)
  {
//    eepromUV = 1;
  }
  else if(uv_stat == 0)
  {
   // eepromUV = 0;
  }
  else if (uv_stat == 1)
  {
   // eepromUV = 1;
  }

//  if(EE_WriteVariable(VirtAddVarTab[0], (uint16_t) eepromUV) != HAL_OK)
// {
//   Error_Handler();
// }
}

void calcOV(void) {
  ov_stat = 0;
  CharCtrl = 0;
  for(int i = 0; i<BATTERY_SERIES_CON;i++)
  {
    if(cellVoltage[i] >= Vhigh)
    {
      CharCtrl = CharCtrl | 1;
      ov_stat = ov_stat | 1;
    }
    else
    {
      CharCtrl = CharCtrl | 0;
      ov_stat = ov_stat | 0;
    }
  }
  if(ov_stat == 1) {
    HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);
  }
  else{
    HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
  }
  //printf("Variable ov_stat = %d\n", ov_stat);

  if(ov_stat == 0)
  {
   for(int i=0; i < BATTERY_SERIES_CON;i++)
   {
    if(cellVoltage[i] < Vth_high)
    {
      //eepromOV = 0;
      //eepromOV = eepromOV | 0;
    }
    else
    {
     // eepromOV = 1;
      //eepromOV = eepromOV | 1;
    }
   }
  }
  else if(ov_stat == 1)
  {
    //eepromOV = 1;
  }
  else if(ov_stat == 0)
  {
    //eepromOV = 0;
  }
  else if (ov_stat == 1)
  {
    //eepromOV = 1;
  }

//  if(EE_WriteVariable(VirtAddVarTab[2], (uint16_t) eepromOV) != HAL_OK)
// {
//   Error_Handler();
// }
}

void calcOT(void) {
  ot_stat=0;
  CharCtrl=0;
  DisCtrl=0;
  for(int i=0; i<BATTERY_SERIES_CON; i++){
  if(i!=0||i!=6)
  {
   if(temperature[i] >= Thigh)
    {
      CharCtrl = CharCtrl | 1;
      DisCtrl = DisCtrl | 1;
      ot_stat = ot_stat | 1;
    }
    else
    {
      CharCtrl = CharCtrl | 0;
      DisCtrl = DisCtrl | 0;
      ot_stat = ot_stat | 0;
    }
  }
  }

  if(ot_stat == 1) {
    HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_SET);
  }
  else{
    HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_RESET);
  }
  //printf("Variable ot_stat = %d\n", ot_stat);
}

void faultDet(void){
  fault1 = 0;
  if(packState == 0){
    fault1 = 0;
  }
  else if(packState == 1){
    if(ov_stat == 1)
    {
      fault1 = 1;
    }
    else{
      fault1 = 0;
    }
  }
  else if(packState == 2){
    if(uv_stat == 1)
    {
      fault1 = 1;
    }
    else{
      fault1 = 0;
    }
  }
}

void faultDetChar(void) {
//  faultChar = 0;
//  if(eepromOV==1){
//    faultChar = 1;
//  }
//  else{
//    faultChar = 0;
//  }
  //printf("eeprom ov = %d\n", eepromOV);
}

void faultTotal(void) {


  if(fault1 == 1 && fault0 == 1){
    faultall++;
      if(faultall >= consecutiveCount){
        HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
      }
  }
  else{
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
    faultall = 0;
  }
  //printf("Fault1 = %d\n", fault1);
  //printf("FaultChar = %d\n", faultChar);
}

void calcVBatCond(void){
  Vmax = 0;
  Vmin = 5000;
  Vavg = 0;
  for(int i=0;i<BATTERY_SERIES_CON;i++)
  {
    if(cellVoltage[i] < Vmin)
      Vmin = cellVoltage[i];
    if(cellVoltage[i] > Vmax)
      Vmax = cellVoltage[i];
  }

  if(Vmin <= Vlow)
    fully_discharged=1;
  else if(Vmin+(Vmin*VTh) > Vlow)
    fully_discharged=0;

  if(Vmax >= Vhigh)
    fully_charged=1;
  else if(Vmax-(Vmax*VTh) < Vhigh)
    fully_charged=0;

  for(int i=0; i<BATTERY_SERIES_CON; i++){
    Vavg = Vavg+cellVoltage[i];
  }
  Vavg = Vavg / BATTERY_SERIES_CON;

  Vtresh =  Vavg+(Vavg*VTh);
  CBChange =0;

  for(int i =0; i <(BATTERY_SERIES_CON/6); i++){
    for(int j=0; j<6; j++){
      lastCB[i][j] = CB[i][j];

      if(CB[i][j] == 0)
      {
	if(cellVoltage[i*6+j] >= Vtresh)
	  CB[i][j] = 1;
        else
	  CB[i][j] = 0;
      }
      else if(CB[i][j] == 1 && packState ==1)
      {
	if(cellVoltage[i*6+j]+0.13 >= Vtresh)
	  CB[i][j] = 1;
	else
	  CB[i][j] = 0;
      }
      else if(CB[i][j] == 1 && packState == 0)
      {
	if(cellVoltage[i*6+j] >= Vtresh)
	  CB[i][j] = 1;
        else
	  CB[i][j] = 0;
      }

    Cell_balancer[i*6+j] = CB[i][j];

    if(lastCB[i][j] != CB[i][j])
      CBChange = CBChange | 1;
    else
      CBChange = CBChange | 0;
    }
  }

  lastPackUnbalanced = packUnbalanced;
  packUnbalanced=0;
  for(int i =0; i<BATTERY_SERIES_CON; i++){
    for(int j=0; j<6; j++){
      //packBalanced = CB[i][j] && packBalanced;
      packUnbalanced = (CB[i][j] | packUnbalanced ? 1: 0);
      //printf("packUnbalanced = %d %d\n", packUnbalanced, ((i+1)*(j+1)));
    }
  }
  //CB[1][3] = 1;
}

void chargingMode(void){
  for(int i =0; i <(BATTERY_SERIES_CON/6); i++){
    for(int j=0; j<6; j++){
      if(CB[i][j] == 1)
	AD7280A_CBxTim(i*6+j);
    }
   AD7280A_CBOutAll(i);
   //HAL_Delay(10);
  }
}

//float movingAvg(float *ptrArrNumbers, float *ptrSum, int pos, int len, float nextNum)
//{
  //Subtract the oldest number from the prev sum, add the new number
  //*ptrSum = *ptrSum - ptrArrNumbers[pos] + nextNum;
  //Assign the nextNum to the position in the array
  //ptrArrNumbers[pos] = nextNum;
  //return the average
 // return *ptrSum / len;
//}

void printPlot(void){
  //for(int i=0; i<BATTERY_SERIES_CON; i++)
  //{
    //printf("AUX ADC[%d] = %0.f\n",i,auxADC[i]);
  //}

  //printf("#");

  printf("%.2f,", current[1]);
  sprintf(textBuf + strlen(textBuf), "CuReg:%.2f,", current[1]);

  printf("%d,", capacity);
  printf(textBuf + strlen(textBuf), "%.2f", capacity);

  printf("@");

  for(int i=0; i<BATTERY_SERIES_CON; i++)
  {

    printf("@%.0f,", cellVoltage[i]);
    sprintf(textBuf, "%.0f,",cellVoltage[i]);
    if(i > 1 && (i+1)%6 == 0){
      printf("@");
    }

  }


  //printf("#");
  //for(int i=12; i<24; i++)
  //{
    //printf("%.0f,", cellVoltage[i]);
    //sprintf(textBuf, "%.0f,",cellVoltage[i]);
  //}
  //printf("\n");
  //printf("#");
  /*
  for(int i =0; i <(BATTERY_SERIES_CON/6); i++){
    for(int j=0; j<6; j++){
	printf("%d,", CB[i][j]);
	sprintf(textBuf + strlen(textBuf), "%d,", CB[i][j]);
    }
  }
  //printf("\n");

  for(int i=0; i<BATTERY_SERIES_CON; i++)
  {
    printf("%.0f,",temperature[i]);
    sprintf(textBuf + strlen(textBuf), "%.0f,",temperature[i]);
  }
  */
  //printf(" V: %.0f,", Vtotal);
  //sprintf(textBuf + strlen(textBuf), "Vtot:%.2f\n", Vtotal);

  //printf(" vADC: %.3f,", vADC);
  //sprintf(textBuf + strlen(textBuf), "ADCr:%.1f,", ADC_Filtered);
  /**/

  //printf("\n");
  /*


  printf("%d,", packState);
  sprintf(textBuf + strlen(textBuf), "ps:%d,", packState);

  */
  //printf("%f,", mAh);
  //sprintf(textBuf + strlen(textBuf), "Ah:%f,", Ah);




//  printf("%d,", fault0);
//  printf(textBuf + strlen(textBuf), "%.2f,", fault0);
//
//  printf("%d,", fault1);
//  printf(textBuf + strlen(textBuf), "%.2f,", fault1);
//
//  printf("%d,", faultall);
//  printf(textBuf + strlen(textBuf), "%.2f,", faultall);

  //printf("%f,", vRef);
  //sprintf(textBuf + strlen(textBuf), "%.2f,", vRef);

  //printf("%f,", vADC);
  //sprintf(textBuf + strlen(textBuf), "%.2f,", vADC);

  //printf("eeprom OV:%d,", eepromOV);
  //sprintf(textBuf + strlen(textBuf), "eeprom OV:%d,", eepromOV);

  //printf("status OV:%d,", ov_stat);
  //sprintf(textBuf + strlen(textBuf), "status OV:%d,", ov_stat);

  //printf("eeprom UV:%d,", eepromUV);

  //printf("status UV:%d,", uv_stat);
  //sprintf(textBuf + strlen(textBuf), "status UV:%d,", uv_stat);
  /*


  printf("%d", fault1);
  sprintf(textBuf + strlen(textBuf), "%d", fault1);
  */
  printf("#\n\r");
  /*
  for(int i =0; i <(BATTERY_SERIES_CON/6); i++){
    for(int j=0; j<6; j++){
	printf("%d,", CB[i][j]);
	sprintf(textBuf + strlen(textBuf), "%d,", CB[i][j]);
    }
  }

  printf("%.0f,",Vavg);
  sprintf(textBuf + strlen(textBuf), "%.0f,",Vavg);
  printf("%.0f,",Vtresh);
  sprintf(textBuf + strlen(textBuf), "%.0f,",Vtresh);

  printf("%d,", lastPackState);
  sprintf(textBuf + strlen(textBuf), "%d,", lastPackState);
  printf("%d,", packUnbalanced);
  sprintf(textBuf + strlen(textBuf), "%d,", packUnbalanced);
  printf("%d,", lastPackUnbalanced);
  sprintf(textBuf + strlen(textBuf), "%d,", lastPackUnbalanced);
  */
  //printf("%d,", CBChange);
  //printf("%d,", ChCtrl);
  //printf("%d\n", DschCtrl);
  //sprintf(textBuf + strlen(textBuf), "%d\n", CBChange);
  //HAL_UART_Transmit_DMA(&huart2, (uint8_t *)textBuf, strlen(textBuf));
  //printf("\n");
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
