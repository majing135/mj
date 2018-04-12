/***********************************************
** 文件名称：led.c
** 功能描述: led 应用函数库            
** 实验平台：两轮自平衡小车
** 硬件连接：----------------- 
**	   		|   PB3 - LED1     |
**			|   PB4 - LED2     |
**			 ----------------- 
**********************************************************************************/
#include "led.h"
#include "stm32f10x_exti.h"

void delay_1ms (uint16_t i)
{
 uint16_t x, y;
 for(x=0; x < i; x++)
 {
  for(y=0; y<0x3E5; y++)
  {;}
 }
}
/*
 * 函数名：LED_GPIO_Config
 * 描述  ：配置LED用到的I/O口
 * 输入  ：无
 * 输出  ：无
 */
void LED_GPIO_Config(void)
{		
	 /*先开启GPIOB和AFIO的外设时钟*/
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	/*改变指定管脚的映射 GPIO_Remap_SWJ_JTAGDisable ，JTAG-DP 禁用 + SW-DP 使能*/
   GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable , ENABLE); 
   GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE); 
	
		/* Configure the GPIO_LED_KEY input pin */			
		{
	  /*定义一个GPIO_InitTypeDef类型的结构体*/			
			GPIO_InitTypeDef GPIO_InitStructure;

			GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		
			GPIO_InitStructure.GPIO_Pin     = GPIO_Pin_3|
																				GPIO_Pin_4|
																				GPIO_Pin_5|
																				GPIO_Pin_6|
																				GPIO_Pin_11|GPIO_Pin_7;//LED
			GPIO_Init( GPIOB , &GPIO_InitStructure );
		}
		
		{
	  /*定义一个GPIO_InitTypeDef类型的结构体*/			
			GPIO_InitTypeDef GPIO_InitStructure;

			GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		
			GPIO_InitStructure.GPIO_Pin     = GPIO_Pin_10|//NRF
																				//BLE
																				GPIO_Pin_12|//UART
																				GPIO_Pin_13|
																				GPIO_Pin_14;
			GPIO_Init( GPIOB , &GPIO_InitStructure );
		}		                        
		//LED全亮--》全灭	 
		GPIO_ResetBits(GPIOB, GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);	
		delay_1ms(10000);	
		GPIO_SetBits(GPIOB, GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);
		delay_1ms(10);	 
}

//	外部中断		GPIO_Pin_15;
void EXTI0_Config(void)
{
	EXTI_InitTypeDef   EXTI_InitStructure;
	GPIO_InitTypeDef   GPIO_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;

  /* Enable GPIOB clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO,ENABLE);
	
  /* Configure PB15 pin as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource15);
	EXTI_ClearITPendingBit(EXTI_Line15);	
		
  /* Configure EXTI15 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line15;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

  /* Enable and set EXTI0 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0X00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

}



/************************END OF FILE************/
