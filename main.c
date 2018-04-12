/******************** (C) COPYRIGHT (2015)BST BALANCECAR **************************
 * 文件名  ：main.c
**********************************************************************************/
#include "stm32f10x.h"
//#include "stm32f10x_lib.h"
#include "stdio.h"
#include "mpu6050.h"
#include "i2c_mpu6050.h"
#include "motor.h"
#include "upstandingcar.h"
#include "SysTick.h"
#include "led.h"
#include "usart.h"
#include "i2c.h"
#include "usart.h"
#include "timer.h"
#include "I2C_DEVICE_LIST.h"
#include "stm32f10x_exti.h"
#include "bsp_spi_nrf.h"

extern u8 NRF_Check(void); 
extern void SPI_NRF_Init(void);
extern void NRF_TX_Mode(void);

char NRF_CONNECT_USE_ENABLE=0;
char BLE_CONNECT_USE_ENABLE=0;
char UART_CONNECT_USE_ENABLE=0;

//extern void LineFitting( short x[], short y[],int size_D);

//short a_x[20]={1,2,3,4,5,6,7,8,9,10,11,12,0,0,0,0,0,0,1,2};
//short a_y[20]={1,2,3,4,5,6,7,8,9,10,11,12,0,0,0,0,0,0,500,15};

char MEMS_DEVICE_NUM=0;

/*
 * 函数名：main
 * 描述  ：主函数
 */

//#ifdef __GNUC__
//  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
//     set to 'Yes') calls __io_putchar() */
//  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
//#else
//  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
//#endif /* __GNUC__ */



int main(void)
{

  u8 status=0,try_time=0;
	u8 status_TX=0;
//  LineFitting(a_x,a_y,20);//最小二乘拟合	
	
	
//	Timerx_Init(5000,7199);				  //定时器TIM1
	
	i2cInit();							   				//IIC初始化 用于挂靠在总线上的设备使用
	delay_nms(10);						    		//延时10ms

	//SysTick_Init();						    		//SysTick函数初始化
	LED_GPIO_Config();
//	EXTI0_Config();
	delay_nms(10);						    		//延时10ms	
	MEMS_DEVICE_NUM=DEVICE_IIC_INIT();

	 
	//SysTick->CTRL |=  SysTick_CTRL_ENABLE_Msk;	 //使能总算法时钟

//判断需要进行哪部分的数据输出 GPIO 输入
//NRF 1MB      BLE150KB     UART115200bit
//全部数据     三轴数据     三轴数据	
//PB10          PB11          PB12  ==0
//连接并进行测试
	if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10)==0)
	{
			/*检测NRF模块与MCU的连接*/
			SPI_NRF_Init();
			for(try_time=0;try_time<5;try_time++)
			{
					status = NRF_Check();
					if(status!=0)	 break;
			}
			if(try_time==5)
			{
					//在定时器继续查询		2s中去查询一次
					NRF_CONNECT_USE_ENABLE=2;//认为连接了，但是不成功，需要继续尝试连接
					GPIO_SetBits(GPIOB, GPIO_Pin_3);//熄灭NRF LED
			}
			else
			{
					NRF_TX_Mode();	
					GPIO_ResetBits(GPIOB, GPIO_Pin_3);
					NRF_CONNECT_USE_ENABLE=1;
			}
	}
	
	if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11)==0)
	{
			/*UART1*/
			USART1_Config();									//串口1初始化 串口打印
#ifdef USART1_TEST		
		  USART_printf( USART1, "UART1_SEND_DATA_OK!\r\n");
			delay_nms(100);
			USART_printf( USART1, "MEMS_DEVICE_NUM:%D!\r\n",MEMS_DEVICE_NUM);
			delay_nms(100);		
#endif
			GPIO_ResetBits(GPIOB, GPIO_Pin_4);
			UART_CONNECT_USE_ENABLE=1;
	}	
	
	if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12)==0)
	{
			/*UART2 BLE   9600*/
			USART2_Config();									//蓝牙串口配置
			//连接初始化//蓝牙配置初始化
			for(try_time=0;try_time<5;try_time++)
			{
					status = BLE_Init();//设置密码为123456
					if(status!=0)	 break;
			}
			
			if(try_time==5)
			{
					//在定时器继续查询		2s中去查询一次
					BLE_CONNECT_USE_ENABLE=2;//认为连接了，但是不成功，需要继续尝试连接
					GPIO_SetBits(GPIOB, GPIO_Pin_5);//熄灭NRF LED
			}
			else
			{
#ifdef USART2_BLE_TEST		
					USART_printf( USART2, "UART2_SEND_DATA_OK!\r\n");
					delay_nms(100);
#endif				
					GPIO_ResetBits(GPIOB, GPIO_Pin_5);
					BLE_CONNECT_USE_ENABLE=1;
			}
			
	}	

	while(1)
	{		
     
		
		
			//while((status_TX&0x01)!=0x01)bma253
				//while((status_TX&0x10)!=0x10)
					while((status_TX&0x80)!=0x80)//BMA421
			{
				//delay_nms(1);
				//i2cRead(0x1f, 0x13, 1,&status_TX);kx023
				//i2cRead(0x12, 0x0b, 1,&status_TX);//6981
				//i2cRead(0x1f, 0x15, 1,&status_TX);//KX126
				//i2cRead(0x19, 0x27, 1,&status_TX);//bma253	
				i2cRead(0x19, 0x03, 1,&status_TX);//BMA421
			}
	    status_TX=0x00;
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

}


