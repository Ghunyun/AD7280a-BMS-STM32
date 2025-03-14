/**************************************************************************//**
*   @file   ad7280a.c
*   @brief  Driver for the AD7280A Lithium Ion Battery Monitoring System
*   @author Lucian Sin (Lucian.Sin@analog.com)
*
*******************************************************************************
* Copyright 2014(c) Analog Devices, Inc.
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*  - Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*  - Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in
*    the documentation and/or other materials provided with the
*    distribution.
*  - Neither the name of Analog Devices, Inc. nor the names of its
*    contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*  - The use of this software may or may not infringe the patent rights
*    of one or more patent holders.  This license does not release you
*    from the requirement that you obtain separate licenses from these
*    patent holders to use this software.
*  - Use of the software either in source or binary form, must be run
*    on or directly connected to an Analog Devices Inc. component.
*
* THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT, MERCHANTABILITY
* AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
* INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "ad7280a_stm32.h"
#include "stdio.h"

/******************************************************************************/
/************************** Variables Definitions *****************************/
/******************************************************************************/
unsigned long 		readData[BATTERY_SERIES_CON];
float         			cellVoltage[BATTERY_SERIES_CON];
float         			auxADC[BATTERY_SERIES_CON];
extern uint8_t 		SPI_Ready , SPI_TX , SPI_RX ;
extern uint8_t		readBattery;
extern uint8_t		CB[BATTERY_SERIES_CON/6][12];
uint32_t				result;
uint16_t	aTxBuffer[BUFSIZE];
uint16_t 	aRxBuffer[BUFSIZE];
extern  uint16_t        kampret;

/*****************************************************************************/
/************************ Functions Definitions ******************************/
/*****************************************************************************/

/******************************************************************************
 * @brief Initializes the communication with the device.
 *
 * @param None.
 *
 * @return status - Result of the initialization procedure.
 *                  Example:  0 - if initialization was successful;
 *                           -1 - if initialization was unsuccessful.
******************************************************************************/
char AD7280A_Init(void)
{	
	uint32_t		value;
	uint16_t		dataBuf[4];
	
	HAL_GPIO_WritePin(AD7280_CS_Port, AD7280_CS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PD_GPIO_Port, PD_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(CNVST_GPIO_Port, CNVST_Pin, GPIO_PIN_SET);
	__HAL_SPI_ENABLE(&hspi2);
	
	value = AD7280A_CRC_Write((AD7280A_CONTROL_LB << 21) |
                              ((AD7280A_CTRL_LB_MUST_SET |
                              AD7280A_CTRL_LB_LOCK_DEV_ADDR |
                              AD7280A_CTRL_LB_DAISY_CHAIN_RB_EN) << 13) |
                              (1 << 12));
	//printf("val = 0x%x\n", value);	

	AD7280A_Transmit_32bits(value);


	value = AD7280A_CRC_Write((AD7280A_READ << 21) |
                              (AD7280A_CONTROL_LB << 15) |
                              (1 << 12));
	//printf("val = 0x%x\n", value);
	AD7280A_Transmit_32bits(value);


	//AD7280A_Transmit_32bits(AD7280A_READ_TXVAL);
	while(SPI_Ready != 1)
	{}
	SPI_Ready = 0;
	SPI_TX = 0;
	aTxBuffer[0]=0xF800;
	aTxBuffer[1]=0x030A;
	aTxBuffer[2]=0x0;
	aTxBuffer[3]=0x0;
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit_IT(&hspi2, (uint8_t *)aTxBuffer, 2);
        
	//AD7280A_Transmit_32bits(AD7280A_READ_TXVAL);
	
	/*while(SPI_Ready != 1)
  	{}
  
  	aTxBuffer[0] = 0xF800;
  	aTxBuffer[1] = 0x030A;
  	SPI_Ready = 0;
  	SPI_TX = 0;
  	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
  	HAL_SPI_TransmitReceive_IT(&hspi2, (uint8_t *)aTxBuffer, (uint8_t *)aRxBuffer, 4);*/
	       
	return 1;
  
}

/******************************************************************************
* @brief Transmits 32 data bits from/to AD7280A.
 *
 * @param data          - Data to be transmitted (MOSI pin).
 *
 * @return result 		- Status of SPI transmission.
******************************************************************************/
unsigned long AD7280A_Transmit_32bits(uint32_t data)
{
  	uint16_t dataBuf[2];
	
	while(SPI_Ready != 1)
	{}
	
  	SPI_Ready = 0;
	SPI_TX = 0;
	
 	dataBuf[0] = (data >> 16) ;
  	dataBuf[1] = data;
			
	HAL_GPIO_WritePin(AD7280_CS_Port, AD7280_CS_Pin, GPIO_PIN_RESET);
	result = HAL_SPI_Transmit_IT(&hspi2, (uint8_t *)dataBuf, 2);
	HAL_Delay(1);
	
	return result;

}

/******************************************************************************
* @brief Transmit then Receive 32 data bits to/from AD7280A.
 *
 * @param data          - Data to be transmitted (MOSI pin).
 * @param aRxBuffer		- Buffer for storing received data from MISO pin.
 * 
 * @return result 		- Status of SPI transmission.
******************************************************************************/
unsigned long AD7280A_TransmitReceive_32bits(uint32_t data,unsigned int datasize)
{
    	uint32_t dataBuf[128];

        for (int i = 0; i < datasize; i++) {
            dataBuf[i] = i;  // Example data
        }


	while(SPI_Ready != 1)
	{}

  	SPI_Ready = 0;
	SPI_TX = 0;
	HAL_GPIO_WritePin(AD7280_CS_Port, AD7280_CS_Pin, GPIO_PIN_RESET);

	HAL_SPI_TransmitReceive_IT(&hspi2, (uint8_t *)dataBuf, (uint8_t *)aRxBuffer, 4*datasize);
	//HAL_Delay(1000);
	return result;

}

/******************************************************************************
 * @brief Computes the CRC value for a write transmission, and prepares the
 *        complete write codeword
 *
 * @param message : The message that is to be transmitted. The least significant
 *                  11 bits are discarded
 *
 * @return Complete codeword that can be transmitted to AD7280A
******************************************************************************/
unsigned long AD7280A_CRC_Write(unsigned int data)
{	
			int i;
    	int CRC_data;
    	unsigned int CRC_0=0, CRC_1=0, CRC_2=0, CRC_3=0, CRC_4=0, CRC_5=0, CRC_6=0, CRC_7=0;
    	unsigned int xor_1=0, xor_2=0, xor_3=0, xor_4=0, xor_5=0;
			uint32_t data_out = 0;

			data = data >>11;
			
    	for( i = NUMBITS_WRITE; i >= 0; i--)
    	{
        	xor_5 = (CRC_4 ^ CRC_7) ? 1 : 0;
        	xor_4 = (CRC_2 ^ CRC_7) ? 1 : 0;
        	xor_3 = (CRC_1 ^ CRC_7) ? 1 : 0;
        	xor_2 = (CRC_0 ^ CRC_7) ? 1 : 0;
        	xor_1 = ((data&(1u<<i)? 1: 0) ^ CRC_7) ? 1 :0;

        	CRC_7 = CRC_6;
        	CRC_6 = CRC_5;
        	CRC_5 = xor_5;
        	CRC_4 = CRC_3;
        	CRC_3 = xor_4;
        	CRC_2 = xor_3;
        	CRC_1 = xor_2;
        	CRC_0 = xor_1;
		
        	CRC_data = (CRC_0 | (CRC_1 << 1) | (CRC_2 << 2) | (CRC_3 << 3) | (CRC_4 << 4) | (CRC_5 << 5) | (CRC_6 << 6) | (CRC_7 << 7)) ;
    	}

			data_out = (data << 11) | (CRC_data << 3) | 0x2;

    	return data_out;
}

/******************************************************************************
 * @brief Checks the received message if the received CRC and computed CRC are
 *        the same.
 *
 * @param message : The received message
 *
 * @return 1 if the two CRC are identical
 *         0 otherwise
******************************************************************************/
long AD7280A_CRC_Read(uint32_t data)
{
    	int 				i;
	int				CRC_data, CRC_REC;
	uint32_t			data_in;
	unsigned int 	CRC_0=0, CRC_1=0, CRC_2=0, CRC_3=0, CRC_4=0, CRC_5=0, CRC_6=0, CRC_7=0;
	unsigned int 	xor_1, xor_2, xor_3, xor_4, xor_5;
	
	CRC_REC = (data >> 2) & 0x00FF;
	data_in = data >> 10;
	
	for( i = NUMBITS_READ; i >= 0; i--)
	{
	  xor_5 = (CRC_4 ^ CRC_7) ? 1 : 0;
	  xor_4 = (CRC_2 ^ CRC_7) ? 1 : 0;
	  xor_3 = (CRC_1 ^ CRC_7) ? 1 : 0;
	  xor_2 = (CRC_0 ^ CRC_7) ? 1 : 0;
	  xor_1 = ((data_in&(1u<<i)? 1: 0) ^ CRC_7) ? 1 :0;
	  
	  CRC_7 = CRC_6;
	  CRC_6 = CRC_5;
	  CRC_5 = xor_5;
	  CRC_4 = CRC_3;
	  CRC_3 = xor_4;
	  CRC_2 = xor_3;
	  CRC_1 = xor_2;
	  CRC_0 = xor_1;
	  
	  CRC_data = (CRC_0 | (CRC_1 << 1) | (CRC_2 << 2) | (CRC_3 << 3) | (CRC_4 << 4) | (CRC_5 << 5) | (CRC_6 << 6) | (CRC_7 << 7)) ;
	}
	
	if (CRC_REC == CRC_data)
	{
	  return 1;
	}
	else
	{
	  return 0;
	}
}

/******************************************************************************
 * @brief Performs a read from all registers on 2 devices.
 *
 * @param None.
 *
 * @return TRUE.
******************************************************************************/
char AD7280A_Convert_Read_All(unsigned int dataSize)
{
  
  unsigned long value;
  
  //readBattery=0;
  AD7280A_Init();
  /* Configure Control HB register. Read all register, convert all registers,
  average 8 values for all devices */
  value = AD7280A_CRC_Write((unsigned long) (AD7280A_CONTROL_HB << 21) |
			    ((AD7280A_CTRL_HB_CONV_RES_READ_ALL |
			      AD7280A_CTRL_HB_CONV_INPUT_ALL |
				AD7280A_CTRL_HB_CONV_AVG_8) << 13) |
			      (1 << 12));

  /* Configure the Read register for all devices */
  AD7280A_Transmit_32bits(value);

  value = AD7280A_CRC_Write((unsigned long) (AD7280A_READ << 21) |
			    (AD7280A_CELL_VOLTAGE_1 << 15) |
			      (1 << 12));

  AD7280A_Transmit_32bits(value);

  /* Configure the CNVST register, allow single CNVST pulse */
  value = AD7280A_CRC_Write((unsigned long) (AD7280A_CNVST_N_CONTROL << 21) |
			    (1 << 14) |
			      (1 << 12));

  AD7280A_Transmit_32bits(value);

  /* Wait 100us */
  //HAL_Delay(1);
  /* Toggle CNVST pin */
  HAL_GPIO_WritePin(CNVST_GPIO_Port, CNVST_Pin, GPIO_PIN_RESET);
  /* Wait 50us */
  HAL_Delay(1);
  HAL_GPIO_WritePin(CNVST_GPIO_Port, CNVST_Pin, GPIO_PIN_SET);

  /* Wait 300us */
  //HAL_Delay(1);
  
  /* Read data from both devices */
  AD7280A_TransmitReceive_32bits(AD7280A_READ_TXVAL, dataSize);

  HAL_Delay(1);
	
  /* Convert the received data to float values. */
  AD7280A_Convert_Data_All();
  
  return 1;
}

/******************************************************************************
 * @brief Converts acquired data from all channels to float values.
 *
 * @param None.
 *
 * @return TRUE.
******************************************************************************/
char AD7280A_Convert_Data_All(void)
{
  unsigned char i;
  
  for(i =0; i<BATTERY_SERIES_CON/6; i++)
  {
    for(int j=0; j<6; j++)
    {
      cellVoltage[(6*i+j)] 			= (((((aRxBuffer[2*j+24*i] << 16) | aRxBuffer[2*j+24*i+1]) >> 11)  & 0xfff)) * 0.978925389 + 989.4277433 + kampret;
    }
    for(int k =0; k<12; k=k+2)
    {
      auxADC[k/2]				= ((((aRxBuffer[k+12] << 16) | aRxBuffer[k+12+1]) >> 11) & 0xfff) * 1.220703125;
      auxADC[(k+12)/2]				= ((((aRxBuffer[k+36] << 16) | aRxBuffer[k+36+1]) >> 11) & 0xfff) * 1.220703125;
      auxADC[(k+24)/2]				= ((((aRxBuffer[k+60] << 16) | aRxBuffer[k+60+1]) >> 11) & 0xfff) * 1.220703125;
      auxADC[(k+36)/2]				= ((((aRxBuffer[k+84] << 16) | aRxBuffer[k+84+1]) >> 11) & 0xfff) * 1.220703125;
    }
  }
  return 1;
}

unsigned int AD7280A_CBxTim(unsigned int cellNum)
{
  uint8_t temp; 
  uint8_t devAddress,regAddress;
  uint8_t bit[5], temp_bit[5];
  unsigned long value;
  
  temp = (cellNum/6);

  for(int i=0;i<5;i++){
      bit[i] = (temp&(1u<<i)? 1: 0);
      temp_bit[i] = bit[i];
  }
  for(int i=0;i<5;i++){
      bit[i] = 0;
  }
  for(int i=0;i<5;i++){
      bit[4-i] = temp_bit[i];
  }

  devAddress = 0;
  devAddress = bit[4] << 4 | bit[3] << 3 |  bit[2] << 2 |  bit[1] << 1 |  bit[0] << 0 ;
  
  if((cellNum%6) == 0)
    regAddress = AD7280A_CB1_TIMER;
  else if((cellNum%6) == 1)
    regAddress = AD7280A_CB2_TIMER;
  else if((cellNum%6) == 2)
    regAddress = AD7280A_CB3_TIMER;
  else if((cellNum%6) == 3)
    regAddress = AD7280A_CB4_TIMER;
  else if((cellNum%6) == 4)
    regAddress = AD7280A_CB5_TIMER;
  else if((cellNum%6) == 5)
    regAddress = AD7280A_CB6_TIMER;
  
  value = 0;
  value = ((devAddress << 27) | (regAddress << 21) | (0xF8 << 13) | (0x0 << 12) | (0x0 << 11));
  //printf("cellnum = %d\n", (cellNum%6));
  //printf("devAddress = 0x%x \t regAddress = 0x%x\n", devAddress, regAddress);
  //printf("value = 0x%x\n",value>>11);
  
  value = 0;
  value = AD7280A_CRC_Write((unsigned long) (devAddress << 27) | (regAddress << 21) | (0xF8 << 13) | (0x0 << 12) | (0x0 << 11));
  //printf("value CB%dTim= 0x%x\n",cellNum%6,value);
  //while(SPI_Ready != 1)
  //{}
  AD7280A_Transmit_32bits(value);
  
  return 1;
}

void AD7280A_CBOutAll(int stackNum){
  uint8_t devAddress=0, regData=0, status=0;
  unsigned long value;
  
  regData = 0;
  for(int j=0; j<6; j++){
    if(CB[stackNum][j] == 1){
      regData = (regData | (1 <<  j));
      //printf("CB[%d][%d] = 1\n", stackNum, j);
      //printf("regData = 0x%x\n", regData);
    }
  }  
  regData = regData << 2;
  devAddress = AD7280A_addressConv(stackNum);
  value = 0;
  value = ((devAddress << 27) | (AD7280A_CELL_BALANCE << 21) | ( regData << 13) | (0x0 << 12) | (0x0 << 11));
  //printf("value = 0x%x\n",value>>11);
    
  value = 0;
  value = AD7280A_CRC_Write((unsigned long) (devAddress << 27) | (AD7280A_CELL_BALANCE << 21) | ( regData << 13) | (0x0 << 12) | (0x0 << 11));
  //printf("val CB%dOut = 0x%x\n\n",stackNum, value);
  status = AD7280A_Transmit_32bits(value);
  //printf("status %d = %d\n", stackNum, status);
}

void AD7280A_CBOffAll(void){
  unsigned long value=0;
  
  value = AD7280A_CRC_Write((unsigned long) (0x00 << 27) | (AD7280A_CELL_BALANCE << 21) | (0x00 << 13) | (0x1 << 12) | (0x0 << 11));
  AD7280A_Transmit_32bits(value);
}

unsigned int AD7280A_addressConv(unsigned int addressIn)
{
  uint8_t bit[5], temp_bit[5];
  uint8_t devAddress;
  
  for(int i=0;i<5;i++){
      bit[i] = (addressIn&(1u<<i)? 1: 0);
      temp_bit[i] = bit[i];
  }
  for(int i=0;i<5;i++){
      bit[i] = 0;
  }
  for(int i=0;i<5;i++){
      bit[4-i] = temp_bit[i];
  }

  devAddress = 0;
  devAddress = bit[4] << 4 | bit[3] << 3 |  bit[2] << 2 |  bit[1] << 1 |  bit[0] << 0 ;
  
  return devAddress;
}
