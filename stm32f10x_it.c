/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
 #include "stm32f10x_it.h"
 #include <stdio.h>
 #include "upstandingcar.h"
 #include "outputdata.h"
 #include "mpu6050.h"
 #include "UltrasonicWave.h"
 #include "stm32f10x_exti.h"
 #include "bsp_spi_nrf.h" 
 #include "I2C_DEVICE_LIST.h"
 #include "usart.h"
// #include "Anyway_Kalman.h"
	
uint8_t status_TX=0;
uint8_t STM32_DATA_TEMP[32];
u8	BST_u8MainEventCount=0;
unsigned int	TIME_PASED=0;

extern char NRF_CONNECT_USE_ENABLE;
extern char BLE_CONNECT_USE_ENABLE;
extern char UART_CONNECT_USE_ENABLE;
extern char MEMS_DEVICE_NUM;

u8 keystatus=0;
u8 Roller_KEY=0;
u8 keystatus_flag=0;
int doubleclick_time=0,click_num=0;

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */

#if 1
void SysTick_Handler(void)				 //2ms定时器
{  
	u8 status=0;
	BST_u8MainEventCount++;				   //总循环计数值

	if(BST_u8MainEventCount>=10)	//ACCEL+GYRO_3
	{
		BST_u8MainEventCount=0;
		TIME_PASED++;
		if(TIME_PASED==400)//2.4s 
		{
			TIME_PASED=0;
			if(NRF_CONNECT_USE_ENABLE==2)
			{
					status = NRF_Check();
					if(status==0)
					{
							NRF_CONNECT_USE_ENABLE=2;	
							GPIO_SetBits(GPIOB, GPIO_Pin_3);//熄灭NRF LED
					}
					else
					{
							NRF_CONNECT_USE_ENABLE=1;	
							GPIO_ResetBits(GPIOB, GPIO_Pin_3);//点亮NRF LED
					}	
			}
			if(BLE_CONNECT_USE_ENABLE==2)
			{
					status = BLE_Init();//设置密码为123456
					if(status==0)
					{
							BLE_CONNECT_USE_ENABLE=2;	
							GPIO_SetBits(GPIOB, GPIO_Pin_5);//熄灭NRF LED
					}
					else
					{
							BLE_CONNECT_USE_ENABLE=1;	
							GPIO_ResetBits(GPIOB, GPIO_Pin_5);//点亮NRF LED
					}	
			}			
		}
	}
	else if(BST_u8MainEventCount==1)//ACCEL	
	{	  
		////以下内容为FIFO 测试使用，使用前请屏蔽其他功能		

		status_TX=READ_ACCEL();
		if(status_TX==1)
		{
				status_TX=NRF_Tx_Dat(STM32_DATA);//SEND_DATA
				if(status_TX==0)
				{
					//UART => SEND ERROR
				}			
		}
	}

//	else if(BST_u8MainEventCount==2)//MAG
//	{
////		status_TX=READ_MAG();
////		if(status_TX==1)
////		{
////				status_TX=NRF_Tx_Dat(STM32_DATA);//SEND_DATA
////				if(status_TX==0)
////				{
////					//UART => SEND ERROR
////				}			
////		}	
//	}
	else if(BST_u8MainEventCount==2)//ACCEL+GYRO_1
	{
////以下内容为FIFO 测试使用，使用前请屏蔽其他功能		
//		status_TX=READ_SIX_FIFO_TEST();
//		if(status_TX==1)
//		{
//				status_TX=NRF_Tx_Dat(STM32_DATA);//SEND_DATA
//				if(status_TX==0)
//				{
//					//UART => SEND ERROR
//				}			
//		}
//	 GPIOB->ODR^=GPIO_Pin_11;
		status_TX=READ_SIX_1();

		if(status_TX==1)
		{
				status_TX=NRF_Tx_Dat(STM32_DATA);//SEND_DATA
				if(status_TX==0)
				{
					//UART => SEND ERROR
				}			
		}	
	}
	else if(BST_u8MainEventCount==3)//ACCEL+GYRO_2
	{
		status_TX=READ_SIX_2();
		if(status_TX==1)
		{
				status_TX=NRF_Tx_Dat(STM32_DATA);//SEND_DATA
				if(status_TX==0)
				{
					//UART => SEND ERROR
				}	
		}
	}
//	else if(BST_u8MainEventCount==3)////ACCEL+GYRO_3
//	{		
//			status_TX=READ_SIX_3();
//			if(status_TX==1)
//			{
//					status_TX=NRF_Tx_Dat(STM32_DATA);//SEND_DATA
//					if(status_TX==0)
//					{
//						//UART => SEND ERROR
//					}	
//			}
//	 }
	
	else if(BST_u8MainEventCount==4)//气压计
	{		
	static u8 Button_up_flag=1;
  static u8 Button_up_num=0;
	u8 KEY_NUM_TH=3;
	static u8 left_num=0,right_num=0,up_num=0,down_num=0;
		
		if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12)==0||
			 GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13)==0||
			 GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14)==0||		
			 GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15)==0		
		)
		{
					if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12)==0)
					{
						doubleclick_time++;
						left_num++;
						down_num=0;
						right_num=0;
						up_num=0;
					}
					else if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13)==0)
					{
						down_num++;
						left_num=0;
						right_num=0;
						up_num=0;
					}
					else if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14)==0)
					{
						right_num++;
						left_num=0;
						down_num=0;
						up_num=0;				
					}
					else if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15)==0)
					{
						up_num++;
						left_num=0;
						down_num=0;
						right_num=0;	
					}
					
					if(left_num>KEY_NUM_TH)	
					{
						
						keystatus=0X01;
						left_num=0;
						keystatus_flag=keystatus_flag|0x01;
			
					}
					else if(right_num>KEY_NUM_TH)
					{
						keystatus=0X02;
						right_num=0;
					}
					else if(down_num>KEY_NUM_TH)
					{
						Roller_KEY=-1;
						down_num=0;
					}				
					else if(up_num>KEY_NUM_TH)
					{
						Roller_KEY=1;
						up_num=0;
					}	
					Button_up_flag=1;
		}
		else
		{
			  up_num=0;
				left_num=0;
				down_num=0;
				right_num=0;
			if(Button_up_flag==1)
			    Button_up_num++;
			if(Button_up_num>5)
			{
				Button_up_flag=0;
				Button_up_num=0;
				keystatus=0X00;
				Roller_KEY=0;
				
			}				
		}
//			status_TX=READ_SIX_3();
//			if(status_TX==1)
//			{
//					status_TX=NRF_Tx_Dat(STM32_DATA);//SEND_DATA
//					if(status_TX==0)
//					{
//						//UART => SEND ERROR
//					}	
//			}
	 }
	else if(BST_u8MainEventCount==5)//心率计
	{		
//			status_TX=READ_Heart_Rate();
//			if(status_TX==1)
//			{
//					status_TX=NRF_Tx_Dat(STM32_DATA);//SEND_DATA
//					if(status_TX==0)
//					{
//						//UART => SEND ERROR
//					}	
//			}
		
	 }

}


#endif


void	EXTI15_10_IRQHandler(void)
{
		EXTI_ClearITPendingBit(EXTI_Line15);
	
////加速度计 FIFO 中断实验	
//		status_TX=READ_FIFO_ACCEL();
//		if(status_TX==1)
//		{
//				status_TX=NRF_Tx_Dat(STM32_DATA);//SEND_DATA
//				if(status_TX==0)
//				{
//					//UART => SEND ERROR
//				}			
//		}	
//心率计 中断实验		
			status_TX=READ_Heart_Rate();//心率计
			if(status_TX==1)
			{
					status_TX=NRF_Tx_Dat(STM32_DATA);//SEND_DATA
					if(status_TX==0)
					{
						//UART => SEND ERROR
					}	
			}
	
}



void USART3_IRQHandler(void)
{
/*	
	u8 ucBluetoothValue;
	if(USART3->SR&(1<<5))//接收到数据
	{	 
		ucBluetoothValue=USART1->DR; 

    }
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{ 	
	    ucBluetoothValue = USART_ReceiveData(USART3);
	  	switch (ucBluetoothValue)
	{
	  case 0x02 : g_fBluetoothSpeed =   500 ; break;	   //前进
	  case 0x01 : g_fBluetoothSpeed = (-500);  break;	   //后退
	  case 0x03 : g_fBluetoothDirection =   100 ;  break;//左转
	  case 0x04 : g_fBluetoothDirection = (-100);  break;//右转
	  case 0x05 : g_fBluetoothSpeed =   1000 ; break ;
	  case 0x06 : g_fBluetoothSpeed = (-1000); break ;
	  case 0x07 : g_fBluetoothDirection =   500 ;  break;
	  case 0x08 : g_fBluetoothDirection = (-500);  break;
	  default : g_fBluetoothSpeed = 0; g_fBluetoothDirection = 0;break;
	}
	USART_ClearITPendingBit(USART3, USART_IT_RXNE); //清除中断标志
	} 
*/	 
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
