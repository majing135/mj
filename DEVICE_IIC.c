
#include "stdio.h"
#include "stm32f10x.h"
#include "i2c.h"
#include "math.h"
#include "windows.h"
#include "BMA421.h"
#include "led.h"

// registers of the device
#define MS561101BA_D1 0x40
#define MS561101BA_D2 0x50
#define MS561101BA_RESET 0x1E

// D1 and D2 result size (bytes)
#define MS561101BA_D1D2_SIZE 3

// OSR (Over Sampling Ratio) constants
#define MS561101BA_OSR_256 0x00
#define MS561101BA_OSR_512 0x02
#define MS561101BA_OSR_1024 0x04
#define MS561101BA_OSR_2048 0x06
#define MS561101BA_OSR_4096 0x08

#define MS561101BA_PROM_BASE_ADDR 0xA2 // by adding ints from 0 to 6 we can read all the prom configuration values. 
// C1 will be at 0xA2 and all the subsequent are multiples of 2
#define MS561101BA_PROM_REG_COUNT 6 // number of registers in the PROM
#define MS561101BA_PROM_REG_SIZE 2 // size in bytes of a prom registry.

#define Filter_Num 20//¶¨ÒåÊý×é´óÐ¡
#define Filter_Jnum 4//¶àÏîÊ½ÄâºÏ½×²ãÊý
#define Sensitivity 55//Êó±êÁéÃô¶È


extern float angle_X, angle_dot_X; 		//Íâ²¿ÐèÒªÒýÓÃµÄ±äÁ¿ 
extern float angle_Y, angle_dot_Y; 		//Íâ²¿ÐèÒªÒýÓÃµÄ±äÁ¿
extern float angle_Z, angle_dot_Z; 		//Íâ²¿ÐèÒªÒýÓÃµÄ±äÁ¿
extern float A_D, B_D; //×îÐ¡¶þ³Ë·¨µÄ²ÎÊý



extern void Kalman_Filter_X(float angle_m,float gyro_m);
extern void Kalman_Filter_Y(float angle_m,float gyro_m);																					 
extern void Kalman_Filter_Z(float angle_m,float gyro_m);
extern void LineFitting(short x[], short y[],int size_D);
extern void polyfit(int n, float *x, float *y, int poly_n, float p[]); 

u8 DEVICE_INIT(void);
u8 ERGODIC_DEVICE_LIST(void);
u8 DEVICE_IIC_INIT(void);
u8 DEVICE_LIST_CONNECT[36];
u8	READ_Heart_Rate(void);

extern char MEMS_DEVICE_NUM;
extern u8 keystatus;
extern u8 Roller_KEY;
extern u8 doubleclick_time;
extern u8 click_num;
extern u8 keystatus_flag;
extern char MEMS_DEVICE_NUM;

uint8_t STM32_DATA[32]=
{
	0x00,	0x00,//ÑùÆ·¼ÓËÙ¶È¼Æ
	0x00,	0x00,//¼ÓËÙ¶È¼Æ
	0x00,	0x00,//ÑùÆ·ÍÓÂÝÒÇ Ç°ºó
	0x00,	0x00,//×Ô¼ºÍÓÂÝÒÇ Ç°ºó
	0x00,	0x00,//ÑùÆ·ÍÓÂÝÒÇ ×óÓÒ
	0x00,	0x00,//×Ô¼ºÍÓÂÝÒÇ ×óÓÒ
	0x00,	0x00,//ÑùÆ·ÈÚºÏ½Ç¶È
	0x00,	0x00,//ÈÚºÏ½Ç¶È
	0x00,	0x00,//ÑùÆ·ÈÚºÏ½Ç¶ÈÎ¢·ÖÁ¿
	0x00,	0x00,//ÈÚºÏ½Ç¶ÈÎ¢·ÖÁ¿
	0x00,	0x00,//¿Õ
	0x00,	0x00,//¿Õ
	0x00,	0x00,//¿Õ
	0x00,	0x00,//¿Õ
	0x00,	0x00,//¿Õ  Êý¾Ý°ü ±àºÅ
	0x00,	0x00//¿Õ  Êý¾Ý°ü ±àºÅ
};

u8 IIC_ADDRESS_LIST[36][6]=
{
{0X1C,0x0F,0x11,0xa8,0x00,0x00},//7A30E
{0X1D,0x0F,0x11,0xa8,0x00,0x00},//7A30E
{0x18,0x0F,0x11,0xa8,0x00,0x00},//7A20 0x30
{0x19,0x0F,0x11,0xa8,0x00,0x00},//7A20 0x30
{0x20,0x00,0x01,0x00,0x00,0x00},//7660
{0x21,0x00,0x02,0x00,0x00,0x00},//7660

{0x18,0x00,0xFA,0x02,0x00,0x00},//BMA255//0xF9 //0xFA
{0x19,0x00,0xFA,0x02,0x00,0x00},//BMA255//0xF9 //0xFA//0xF8
{0x24,0x00,0x05,0x00,0x00,0x00},//ST3DH
{0x25,0x00,0x06,0x00,0x00,0x00},//ST3DH
{0x26,0x00,0x07,0x00,0x00,0x00},//ADXL345
{0x27,0x00,0x08,0x00,0x00,0x00},//ADXL345

{0x08,0x0F,0x20,0xA8,0x00,0x00},//7M10
{0x09,0x0F,0x20,0xA8,0x00,0x00},//7M10
{0x30,0x00,0x0B,0x00,0x00,0x00},//AK8975
{0x31,0x00,0x0C,0x00,0x00,0x00},//AK8975

{0X6A,0x0E,0x69,0x28,0x21,0x00},//7I20//Ê¹ÓÃÇ°ÐÞ¸Ä¼Ä´æÆ÷ 0X0E-->0X0F
{0X6B,0x0F,0x69,0x28,0x21,0x00},//7I20//Ê¹ÓÃÇ°ÐÞ¸Ä¼Ä´æÆ÷ 0X0E-->0X0F
{0X67,0x75,0x68,0xBB,0xC3,0x01},//MPU6050
{0X68,0x75,0x68,0xBB,0xC3,0x01},//MPU6050
{0X6A,0x0F,0x69,0x28,0x22,0x00},//ST6DS3//Ê¹ÓÃÇ°ÐÞ¸Ä¼Ä´æÆ÷ 0X0E-->0X0F
{0x6B,0x0E,0x69,0x28,0x22,0x00},//ST6DS3//Ê¹ÓÃÇ°ÐÞ¸Ä¼Ä´æÆ÷ 0X0E-->0X0F	21
{0x68,0x00,0xD1,0x12,0x0C,0x00},//BMI160 0xD0 0xD1 0xD2
{0x69,0x00,0xD1,0x12,0x0C,0x00},//BMI160 0xD0 0xD1 0xD2  23

{0x76,0xD0,0x58,0xF7,0xFA,0x00},//BMP280 FA: TEMP  F7£ºPRESS  24
{0x77,0xD0,0x58,0xF7,0xFA,0x00},//BMP280 FA: TEMP  F7£ºPRESS  25

{0x5C,0x0F,0xBB,0xA8,0x2B,0x00},//SC7P20  26//2B-->TEMPETURE
{0x5D,0x0F,0xBB,0xA8,0x2B,0x00},//SC7P20  27//2B-->TEMPETURE

{0x76,0x0F,0xBB,0xA8,0x2B,0x00},//MSG5611 28//
{0x77,0x0F,0xBB,0xA8,0x2B,0x00},//MSG5611 29//

//{0x19,0x00,0x12,0x00,0x00,0x00},//bma421 30//
{0x19,0x00,0x11,0x12,0x00,0x00},//bma421 30
{0x1f,0x0f,0x15,0x06,0x00,0x00},//Kion 23 31
//{0x08,0x0f,0x33,0x06,0x00,0x00},//Kion  22 31
//{0x4c,0x10,0x00,0x06,0x00,0x00},//mma7660 31
{0x12,0x00,0xB0,0x01,0x00,0x00},//32 6981
{0x1f,0x11,0x38,0x08,0x00,0x00},//KX126 33
{0x19,0x0F,0x44,0x28,0x00,0x00},//LIS2DW  34
};


u8 ACCEL_GYRO_NUM=0;
float SIX_1_NOISE[3][30];


void Delay_ms (uint16_t i)
{
 uint16_t x, y;
 for(x=0; x < i; x++)
 {
  for(y=0; y<0x3E5; y++)
  {;}
 }
}

void choise(int *a,int n)
{
		int i,j,k,temp; 
		for(i=0;i<n-1;i++) 
		{ 
			k=i;
			for(j=i+1;j<n;j++) 
			if(a[k]>a[j]) k=j;
			if(i!=k)
			{
				temp=a[i]; 
				a[i]=a[k]; 
				a[k]=temp; 
			} 
		} 
} 


u8 Accel_Noise_Result[3][3];
u8 Mag_Noise_Result[3][3];

void Accel_Cal_Noise(u8 accel_num,int16_t x,int16_t y,int16_t z)
{
	char i=0,j=0;
	static int16_t ACCEL_Noise[3][3][30];
	int16_t max_value[3][3],min_value[3][3];

	if(accel_num>2)   while(1);//´íÎóÍ£Ö¹
	
	for(j=0;j<3;j++)	
	for(i=0;i<29;i++) 
	{
			ACCEL_Noise[accel_num][j][i]=ACCEL_Noise[accel_num][j][i+1];
	}
	
	ACCEL_Noise[accel_num][0][29]=x;
	ACCEL_Noise[accel_num][1][29]=y;
	ACCEL_Noise[accel_num][2][29]=z;

	for(j=0;j<3;j++)
	{
		min_value[accel_num][j]=ACCEL_Noise[accel_num][j][0];
		max_value[accel_num][j]=min_value[accel_num][j];
	}
	
	for(j=0;j<3;j++)
	for(i=1;i<30;i++)
	{
		if(min_value[accel_num][j]>ACCEL_Noise[accel_num][j][i])
		{
			min_value[accel_num][j]=ACCEL_Noise[accel_num][j][i];
		}
		else if(max_value[accel_num][j]<ACCEL_Noise[accel_num][j][i])
		{
			max_value[accel_num][j]=ACCEL_Noise[accel_num][j][i];		
		}
		else
		{
			//Ö®¼ä¾Ínext
		}
	}	
	for(j=0;j<3;j++)//²»ÄÜ´óÓÚ255	
	if(max_value[accel_num][j]-min_value[accel_num][j]>255)
	{
			Accel_Noise_Result[accel_num][j]=255;
	}
	else
	{
			Accel_Noise_Result[accel_num][j]=max_value[accel_num][j]-min_value[accel_num][j];
	}
	//finsih
}

void Mag_Cal_Noise(u8 mag_num,int16_t x,int16_t y,int16_t z)
{
	char i=0,j=0;
	static int16_t  Mag_Noise[3][3][30];
	int16_t max_value[3][3],min_value[3][3];

	if(mag_num>2)   while(1);//´íÎóÍ£Ö¹
	
	for(j=0;j<3;j++)	
	for(i=0;i<29;i++) 
	{
			 Mag_Noise[mag_num][j][i]= Mag_Noise[mag_num][j][i+1];
	}
	
	 Mag_Noise[mag_num][0][29]=x;
	 Mag_Noise[mag_num][1][29]=y;
	 Mag_Noise[mag_num][2][29]=z;

	for(j=0;j<3;j++)
	{
		min_value[mag_num][j]= Mag_Noise[mag_num][j][0];
		max_value[mag_num][j]= min_value[mag_num][j];
	}
	
	for(j=0;j<3;j++)
	for(i=1;i<30;i++)
	{
		if(min_value[mag_num][j]> Mag_Noise[mag_num][j][i])
		{
			min_value[mag_num][j]= Mag_Noise[mag_num][j][i];
		}
		else if(max_value[mag_num][j]<Mag_Noise[mag_num][j][i])
		{
			max_value[mag_num][j]= Mag_Noise[mag_num][j][i];		
		}
		else
		{
			//Ö®¼ä¾Ínext
		}
	}	
	for(j=0;j<3;j++)//²»ÄÜ´óÓÚ255	
	if(max_value[mag_num][j]-min_value[mag_num][j]>255)
	{
			Mag_Noise_Result[mag_num][j]=255;
	}
	else
	{
			Mag_Noise_Result[mag_num][j]=max_value[mag_num][j]-min_value[mag_num][j];
	}
	//finsih
}







u8 DEVICE_IIC_INIT(void)
{
	u8 value;
	value=ERGODIC_DEVICE_LIST();
	if(value==1)
	{
		value=DEVICE_INIT();
		if(value==1)	return 1;
		else					return 0;
	}
	else
	{
		return 0;
	}
}

//FIND DEVICE
//IIC ADDRESS   CHIP_ID_ADDRESS   CHIP_ID_VALUE
u8 ERGODIC_DEVICE_LIST(void)
{
	u8 i=0,j=0,m=0;
	uint8_t reg_value=0;
	u8 DATA_IIC[255];
	
	for(i=0;i<255;i++)
	{
			
			//i=76;
				i2cRead(0x1d, 0x0f, 1, &DATA_IIC[i]);
			
			//if(DATA_IIC[i]!=0x00)
			//{
			//	m=j;
      //}
	}
		
	
	for(i=0;i<36;i++)
	{
		i2cRead(IIC_ADDRESS_LIST[i][0], IIC_ADDRESS_LIST[i][1], 1, &reg_value);
		if(IIC_ADDRESS_LIST[i][2]==reg_value)
		{
			reg_value=0;
			DEVICE_LIST_CONNECT[i]=1;
		}
		else
		{
			reg_value=0;
			DEVICE_LIST_CONNECT[i]=0;
		}
	}
	
	for(i=0;i<36;i++)
	{
		if(DEVICE_LIST_CONNECT[i]==1) 		return 1;
	}
	if(i==36)			return 0;
	
	return 0;	
}

//************************SC7A30/30E************************//
uint8_t SC7A30_INIT_REG[]=
{
//	0x2E, 0x00, //FIFO_CTRL_REG FM1-FM0=01-->FIFO MODE
	0x20,	0x67,
	0x23,	0x88,//	BDU ¿ªÆô
//	//ÒÔÏÂÄÚÈÝÎªFIFO²âÊÔÊ¹ÓÃ
//	0x22,0x02,//I1_FIFO_overrun
//	0x24,0x40,//FIFO ACC_EN
//	0x25,0x02,//interrupt active low
//	0x2E,0x40 //FIFO_CTRL_REG FM1-FM0=01-->FIFO MODE
//	 0x1e,	0x05//	BDU ¿ªÆô	
	
};

char SC7A30_INIT(u8 pos)
{
	u8 t=0,i=0,DATA[6]={0xff,0xff,0xff,0xff,0xff,0xff};
	
	t=sizeof(SC7A30_INIT_REG)/2;
	for(i=0;i<t;i++)
	{
			i2cWrite(IIC_ADDRESS_LIST[pos][0], SC7A30_INIT_REG[2*i], SC7A30_INIT_REG[2*i+1]);
			SC7A30_INIT_REG[2*i+1]=0X00;
			Delay_ms(10);	//ÅÐ¶ÏÐ´ÈëÊÇ·ñ³É¹¦
	}
	Delay_ms(100);	
	for(i=0;i<t;i++)
	{
			i2cRead(IIC_ADDRESS_LIST[pos][0], SC7A30_INIT_REG[2*i], 1,&SC7A30_INIT_REG[2*i+1]);
	}
	Delay_ms(100);	//ÅÐ¶ÏÐ´ÈëÊÇ·ñ³É¹¦
	
	
	//¶ÁÈ¡Ò»´Î£¬É¾³ý¸Ã´ÎÊý¾Ý¡£	
	i2cRead(IIC_ADDRESS_LIST[pos][0], IIC_ADDRESS_LIST[pos][3], 6,DATA);
	Delay_ms(200);

//	printf("SC7A30_INIT_OK\r\n");
	return 1;
	
}



//************************SC7A20************************//
uint8_t SC7A20_INIT_REG[]=
{
//	0x2E, 0x00, //FIFO_CTRL_REG FM1-FM0=01-->FIFO MODE
	0x20,	0x67,
	0x23,	0x88//	BDU ¿ªÆô
//	//ÒÔÏÂÄÚÈÝÎªFIFO²âÊÔÊ¹ÓÃ
//	0x22,0x02,//I1_FIFO_overrun
//	0x24,0x40,//FIFO ACC_EN
//	0x25,0x02,//interrupt active low
//	0x2E,0x40 //FIFO_CTRL_REG FM1-FM0=01-->FIFO MODE
//	 0x1e,	0x05//	BDU ¿ªÆô	
	
};

uint8_t SC7A20_READ_REG[]=
{
	0x47,	0x00,
	0x48,	0x00,
	0x49,	0x00,
	
	0x4A,	0x00,
	0x4B,	0x00,
	0x4C,	0x00,
	
	0x4D,	0x00,
	0x4E,	0x00,
	0x4F,	0x00,
	
	0x50,	0x00,	
	0x51,	0x00,	
	0x52,	0x00

};

char SC7A20_INIT(u8 pos)
{
	u8 t=0,i=0,DATA[6]={0xff,0xff,0xff,0xff,0xff,0xff};
	
	t=sizeof(SC7A20_INIT_REG)/2;
	for(i=0;i<t;i++)
	{
			i2cWrite(IIC_ADDRESS_LIST[pos][0], SC7A20_INIT_REG[2*i], SC7A20_INIT_REG[2*i+1]);
			SC7A20_INIT_REG[2*i+1]=0X00;
			Delay_ms(10);	//ÅÐ¶ÏÐ´ÈëÊÇ·ñ³É¹¦
	}
	Delay_ms(100);	
	for(i=0;i<t;i++)
	{
			i2cRead(IIC_ADDRESS_LIST[pos][0], SC7A20_INIT_REG[2*i], 1,&SC7A20_INIT_REG[2*i+1]);
	}
	Delay_ms(100);	//ÅÐ¶ÏÐ´ÈëÊÇ·ñ³É¹¦
	
//	//4816G Ð£×¼
//	t=sizeof(SC7A20_READ_REG)/2;
//	for(i=0;i<t;i++)
//	{
//			i2cRead(IIC_ADDRESS_LIST[pos][0], SC7A20_READ_REG[2*i], 1,&SC7A20_READ_REG[2*i+1]);
//	}
//	Delay_ms(100);	//?D??D'è?ê?·?3é1|
////	printf("SC7A20_WR\r\n");
//	
//	for(i=0;i<3;i++)
//	{
//		SC7A20_READ_REG[6*i+7] =(SC7A20_READ_REG[1]&0x80)|(SC7A20_READ_REG[1]>>(i+1));
//		SC7A20_READ_REG[6*i+9] =(SC7A20_READ_REG[3]&0x80)|(SC7A20_READ_REG[3]>>(i+1));
//		SC7A20_READ_REG[6*i+11]=(SC7A20_READ_REG[5]&0x80)|(SC7A20_READ_REG[5]>>(i+1));
//	}
//	
//	t=sizeof(SC7A20_READ_REG)/2;
//	for(i=0;i<t;i++)
//	{
//			i2cWrite(IIC_ADDRESS_LIST[pos][0], SC7A20_READ_REG[2*i], SC7A20_READ_REG[2*i+1]);
//			SC7A20_READ_REG[2*i+1]=0X00;
//	}
//	Delay_ms(100);
//	t=sizeof(SC7A20_READ_REG)/2;
//	for(i=0;i<t;i++)
//	{
//			i2cRead(IIC_ADDRESS_LIST[pos][0], SC7A20_READ_REG[2*i], 1,&SC7A20_READ_REG[2*i+1]);
//	}
//	Delay_ms(100);	//
//	
//	i2cWrite(IIC_ADDRESS_LIST[pos][0], 0x1e, 0x15);
//	Delay_ms(600);	
//	Delay_ms(600);
////	printf("SC7A20_WR\r\n");
	
	//¶ÁÈ¡Ò»´Î£¬É¾³ý¸Ã´ÎÊý¾Ý¡£	
	i2cRead(IIC_ADDRESS_LIST[pos][0], IIC_ADDRESS_LIST[pos][3], 6,DATA);
	Delay_ms(200);

//	printf("SC7A20_INIT_OK\r\n");
	return 1;
	
}

//Ì©ÁèÎ¢
//u8 SC7A20_4G[]=
//{
//	0x47,	0x00,
//	0x48,	0x00,
//	0x49,	0x00,
//	
//	0x4A,	0x00,
//	0x4B,	0x00,
//	0x4C,	0x00,
//	
//	0x4D,	0x00,
//	0x4E,	0x00,
//	0x4F,	0x00,
//	
//	0x50,	0x00,	
//	0x51,	0x00,	
//	0x52,	0x00

//};

//void drv_SC7A20_Init(void)
//{
//	u8  val,i;
//	u8  vaw = 0;
//	u8  var = 0;
//	u8  t=0;

//	i2c_burst_read_midstart(SC7A20_I2C_SLAVE_ADDR, 0x0f,&val,1); //sc
//	if(val!=0x11)	return;

//	 val = 0x00;
//	 i2c_burst_write(SC7A20_I2C_SLAVE_ADDR, 0X2E, &val ,1);

//	 val = 0x47;
//	 i2c_burst_write(SC7A20_I2C_SLAVE_ADDR, 0X20, &val ,1);

//	 //val = 0x98;
//	 val = 0xd8;
//	 i2c_burst_write(SC7A20_I2C_SLAVE_ADDR, 0X23, &val ,1);

//	 val = 0x40;
//	 i2c_burst_write(SC7A20_I2C_SLAVE_ADDR, 0X24, &val ,1);

//	 val = 0x40;
//	 i2c_burst_write(SC7A20_I2C_SLAVE_ADDR, 0X2E, &val ,1);
//	 

//	t=6;
//	for(i=0;i<t;i++)
//	{
//			i2c_burst_read_midstart(SC7A20_I2C_SLAVE_ADDR, SC7A20_4G[2*i],&SC7A20_4G[2*i+1],1); //sc
//	}
//	
//	for(i=0;i<3;i++)
//	{
//		SC7A20_4G[6*i+7] =(SC7A20_4G[1]&0x80)|(SC7A20_4G[1]>>(i+1));
//		SC7A20_4G[6*i+9] =(SC7A20_4G[3]&0x80)|(SC7A20_4G[3]>>(i+1));
//		SC7A20_4G[6*i+11]=(SC7A20_4G[5]&0x80)|(SC7A20_4G[5]>>(i+1));
//	}
//	t=6;
//	for(i=0;i<t;i++)
//	{	
//			i2c_burst_write(SC7A20_I2C_SLAVE_ADDR, SC7A20_4G[2*i], SC7A20_4G[2*i+1] ,1);
//			SC7A20_4G[2*i+1]=0X00;
//	}

//	t=6;
//	for(i=0;i<t;i++)
//	{
//		i2c_burst_read_midstart(SC7A20_I2C_SLAVE_ADDR, SC7A20_4G[2*i],&SC7A20_4G[2*i+1],1); //sc
//	}
//}



//************************SC7A20************************//
uint8_t BMA255_INIT_REG[]=
{
	0x0F,	0x03,//PMU_RANGE:¡À2G
	0x10,	0x0F,//ODR=500HZ
	0x11,	0x0E,//normal mode  &¡¡sleep duration=2MS
	0x13,	0x00
	
};


char BMA255_INIT(u8 pos)
{
	u8 t=0,i=0,DATA[6]={0xff,0xff,0xff,0xff,0xff,0xff};
	
	t=sizeof(BMA255_INIT_REG)/2;
	for(i=0;i<t;i++)
	{
			i2cWrite(IIC_ADDRESS_LIST[pos][0], BMA255_INIT_REG[2*i], BMA255_INIT_REG[2*i+1]);
			BMA255_INIT_REG[2*i+1]=0X00;
	}
	Delay_ms(100);	
	for(i=0;i<t;i++)
	{
			i2cRead(IIC_ADDRESS_LIST[pos][0], BMA255_INIT_REG[2*i], 1,&BMA255_INIT_REG[2*i+1]);
	}
	Delay_ms(100);	//ÅÐ¶ÏÐ´ÈëÊÇ·ñ³É¹¦
	
	//printf("BMA255_WR\r\n");
	
	//¶ÁÈ¡Ò»´Î£¬É¾³ý¸Ã´ÎÊý¾Ý¡£	
	i2cRead(IIC_ADDRESS_LIST[pos][0], IIC_ADDRESS_LIST[pos][3], 6,DATA);
	Delay_ms(200);

	//printf("BMA255_INIT_OK\r\n");
	return 1;
	
}


//************************SC7I20************************//
uint8_t SC7M10_INIT_REG[]=
{
	0x22,0x03,//SC7M10 POWER DOWN
	0x25,0x00,//²»½øÐÐSET/RESET ²Ù×÷
	0x20,0x5c,//0xbc=128,80Hz; 0xb8=128,40Hz; 0xb4=128,20Hz; 0xb0=128,10Hz;0xac,0xa8,0xa4,0xa0
						//AVG=64 0x9c,0x98,0x94,0x90;0x8c,0x88,0x84,0x80; 
						//AVG=32 0x7c,0x78,....
						//AVG=16 0x5c,0x58,....
						//AVG= 8 0x3c,0x38,....
						//AVG= 4 0x1c,0x18,....
	0x21,0x00,//Full scale: 0x00=¡À4Gs£¬0x20=¡À8Gs£¬ 0x40=¡À12Gs£¬ 0x60=¡À16Gs
	0x23,0x01,//BLE=0; b0=BDU=1;
	0x22,0x10 //SC7M10 POWER ON  HR=1
};


char SC7M10_INIT(u8 pos)
{
	u8 t,i,DATA[6],DATA_ONE=0x00;
	
	t=sizeof(SC7M10_INIT_REG)/2;
	for(i=0;i<t;i++)
	{
			i2cWrite(IIC_ADDRESS_LIST[pos][0], SC7M10_INIT_REG[2*i], SC7M10_INIT_REG[2*i+1]);
			SC7M10_INIT_REG[2*i+1]=0X00;	
			Delay_ms(100);
	}
	for(i=0;i<t;i++)
	{
			i2cRead(IIC_ADDRESS_LIST[pos][0], SC7M10_INIT_REG[2*i], 1,&SC7M10_INIT_REG[2*i+1]);
			Delay_ms(20);
	}

//	printf("SC7I20_WR\r\n");
		while(1)
	{
	  i2cRead(IIC_ADDRESS_LIST[pos][0],0x27,1,&DATA_ONE);
	  DATA_ONE=DATA_ONE&0x0f;
	  if(DATA_ONE==0x0f) break;
	}
	//¶ÁÈ¡Ò»´Î£¬É¾³ý¸Ã´ÎÊý¾Ý¡£	
	i2cRead(IIC_ADDRESS_LIST[pos][0], IIC_ADDRESS_LIST[pos][3], 6,DATA);
	Delay_ms(20);
//	printf("SC7I20_INIT_OK\r\n");
	
	return 1;
	
}


//************************SC7I20************************//
uint8_t SC7I20_INIT_REG[]=
{
	0x10,0x67,//ACCEL ODR   200Hz
	0x11,0x87,//GYRO  ODR   1600Hz
	0x12,0x44,//BDU=1 BLE=0 IF_INC=1
	0x13,0x4C//HR=1  ACCEL RANGE ¡À2G  GYRO RANGE ¡À2000dps 16.4
//	0x7D,0x00 //½»»»¼ÓËÙ¶È¼ÆµÄÖá
//ÒÔÏÂÄÚÈÝÎªFIFO²âÊÔÊ¹ÓÃ	
//	0x1A,0x10, //FIFO XL_EN
//	0x1B,0x10, //FIFO G	_EN
//	0x2E,0x4A, //FIFO XL	FIFO MODE SET 10
//	0x1C,0x4A  //FIFO G		FIFO MODE SET 10
};

uint8_t SC7I20_READ_REG[]=
{
	0x6C,	0x00,//2g
	0x6D,	0x00,
	0x6E,	0x00,
	0x6F,	0x00,//4g
	0x70,	0x00,
	0x71,	0x00,
	0x72,	0x00,//8g
	0x73,	0x00,
	0x74,	0x00,
	0x75,	0x00,//16g
	0x76,	0x00,
	0x77,	0x00
	
};



char SC7I20_INIT(u8 pos)
{
	u8 t,i,DATA[6];
	
	t=sizeof(SC7I20_INIT_REG)/2;
	for(i=0;i<t;i++)
	{
			i2cWrite(IIC_ADDRESS_LIST[pos][0], SC7I20_INIT_REG[2*i], SC7I20_INIT_REG[2*i+1]);
			SC7I20_INIT_REG[2*i+1]=0X00;
	}
	Delay_ms(100);
	for(i=0;i<t;i++)
	{
			i2cRead(IIC_ADDRESS_LIST[pos][0], SC7I20_INIT_REG[2*i], 1,&SC7I20_INIT_REG[2*i+1]);
	}
	Delay_ms(10);	//ÅÐ¶ÏÐ´ÈëÊÇ·ñ³É¹¦
	
//	printf("SC7I20_WR\r\n");
	
//	//¶ÁÈ¡Ò»´Î£¬É¾³ý¸Ã´ÎÊý¾Ý¡£	
//	i2cRead(IIC_ADDRESS_LIST[pos][0], IIC_ADDRESS_LIST[pos][3], 6,DATA);
//	Delay_ms(2);
//	i2cRead(IIC_ADDRESS_LIST[pos][0], IIC_ADDRESS_LIST[pos][4], 6,DATA);
//	Delay_ms(2);
	
//	printf("SC7I20_INIT_OK\r\n");
	
//	//4816G Ð£×¼
//	t=sizeof(SC7I20_READ_REG)/2;
//	for(i=0;i<t;i++)
//	{
//			i2cRead(IIC_ADDRESS_LIST[pos][0], SC7I20_READ_REG[2*i], 1,&SC7I20_READ_REG[2*i+1]);
//	}
//	Delay_ms(100);
//	
//	for(i=0;i<3;i++)
//	{
//		SC7I20_READ_REG[6*i+7] =(SC7I20_READ_REG[1]&0x80)|((SC7I20_READ_REG[1]&0x7f)>>(i+1));
//		SC7I20_READ_REG[6*i+9] =(SC7I20_READ_REG[3]&0x80)|((SC7I20_READ_REG[3]&0x7f)>>(i+1));
//		SC7I20_READ_REG[6*i+11]=(SC7I20_READ_REG[5]&0x80)|((SC7I20_READ_REG[5]&0x7f)>>(i+1));
//	}

//	t=sizeof(SC7I20_READ_REG)/2;
//	
//	for(i=3;i<t;i++)
//	{
//			i2cWrite(IIC_ADDRESS_LIST[pos][0], SC7I20_READ_REG[2*i], SC7I20_READ_REG[2*i+1]);
//			SC7A20_READ_REG[2*i+1]=0X00;
//	}
//	Delay_ms(100);
//	t=sizeof(SC7I20_READ_REG)/2;
//	for(i=0;i<t;i++)
//	{
//			i2cRead(IIC_ADDRESS_LIST[pos][0], SC7I20_READ_REG[2*i], 1,&SC7I20_READ_REG[2*i+1]);
//	}
//	Delay_ms(100);	//
//	
//	i2cWrite(IIC_ADDRESS_LIST[pos][0], 0x1e, 0x80);
//	Delay_ms(800);

	
	return 1;
	
}

//************************MPU6050************************//
uint8_t MPU6050_INIT_REG[]=
{
	0x6B,0x03,//ACCEL ODR   
	0x19,0x00,//GYRO  ODR   
	0x1A,0x03,//
	0x1B,0x18,//
	0x1C,0x00 //
};

char MPU6050_INIT(u8 pos)
{
	u8 t,i,DATA[6];
	
	t=sizeof(MPU6050_INIT_REG)/2;
	for(i=0;i<t;i++)
	{
			i2cWrite(IIC_ADDRESS_LIST[pos][0], MPU6050_INIT_REG[2*i], MPU6050_INIT_REG[2*i+1]);
			MPU6050_INIT_REG[2*i+1]=0X00;
	}
	Delay_ms(100);	
	for(i=0;i<t;i++)
	{
			i2cRead(IIC_ADDRESS_LIST[pos][0], MPU6050_INIT_REG[2*i], 1,&MPU6050_INIT_REG[2*i+1]);
	}
	Delay_ms(10);	//ÅÐ¶ÏÐ´ÈëÊÇ·ñ³É¹¦
	
	i2cRead(IIC_ADDRESS_LIST[pos][0], IIC_ADDRESS_LIST[pos][3], 6,DATA);
	Delay_ms(2);
	i2cRead(IIC_ADDRESS_LIST[pos][0], IIC_ADDRESS_LIST[pos][4], 6,DATA);
	Delay_ms(2);

}






//************************ST6DS3************************//
uint8_t ST6DS3_INIT_REG[]=
{
	0x10,0x80,//ACCEL ODR  1660Hz	 ¡À2G
	0x11,0x8C,//GYRO  ODR  1600Hz ¡À2000dps
	0x12,0x44,//BDU=1 IF_INC=1 
	0x13,0x00,//
	0x18,0x38,//ACCEL  OUTPUT ENABLE
	0x19,0x38 //GYRO   OUTPUT ENABLE
};

short ACCEL_GYRO_STATIONARY_DATA[3]={0,0,0};

void ST6DS3_INIT(u8 pos)
{ 
	short ACCEL_GYRO_DATA_BUF[20][3];
	signed long ST6DS3_STATIONARY_DATA[3]={0,0,0};
	u8 t,i,DATA[6];
	
	t=sizeof(ST6DS3_INIT_REG)/2;
	for(i=0;i<t;i++)
	{
			i2cWrite(IIC_ADDRESS_LIST[pos][0], ST6DS3_INIT_REG[2*i], ST6DS3_INIT_REG[2*i+1]);
			ST6DS3_INIT_REG[2*i+1]=0X00;
	}
	Delay_ms(100);	
	for(i=0;i<t;i++)
	{
			i2cRead(IIC_ADDRESS_LIST[pos][0], ST6DS3_INIT_REG[2*i], 1,&ST6DS3_INIT_REG[2*i+1]);
	}
	Delay_ms(10);	//ÅÐ¶ÏÐ´ÈëÊÇ·ñ³É¹¦
	
//	printf("BMI160_WR\r\n");
	t=0x11;
	i2cWrite(IIC_ADDRESS_LIST[pos][0],0x7e,t);
	Delay_ms(100);
	t=0x15;
	i2cWrite(IIC_ADDRESS_LIST[pos][0],0x7e,t);	
	Delay_ms(100);
	
	//¶ÁÈ¡Ò»´Î£¬É¾³ý¸Ã´ÎÊý¾Ý¡£	
	i2cRead(IIC_ADDRESS_LIST[pos][0], IIC_ADDRESS_LIST[pos][3], 6,DATA);
	Delay_ms(2);
	i2cRead(IIC_ADDRESS_LIST[pos][0], IIC_ADDRESS_LIST[pos][4], 6,DATA);
	Delay_ms(2);
	for(i=0;i<20;i++)
	{
		i2cRead(IIC_ADDRESS_LIST[pos][0], IIC_ADDRESS_LIST[pos][4], 6,DATA);					
		ACCEL_GYRO_DATA_BUF[i][0] =(int16_t)(((unsigned char)DATA[1] * 256 ) + (unsigned char)DATA[0]);//16Î»
		ACCEL_GYRO_DATA_BUF[i][1] =(int16_t)(((unsigned char)DATA[3] * 256 ) + (unsigned char)DATA[2]);//16Î»
		ACCEL_GYRO_DATA_BUF[i][2] =(int16_t)(((unsigned char)DATA[5] * 256 ) + (unsigned char)DATA[4]);//16Î»	
	}

	for(t=0;t<3;t++)
	{
			for(i=0;i<20;i++)
			{			
					ST6DS3_STATIONARY_DATA[t]=ST6DS3_STATIONARY_DATA[t]+ACCEL_GYRO_DATA_BUF[i][t];
			}
			ACCEL_GYRO_STATIONARY_DATA[t]=(signed short)(ST6DS3_STATIONARY_DATA[t]/20);	//¸Õ¿ªÊ¼ÏÈÇó¸öÆ½¾ù
	}
	
//	printf("BMI160_INIT_OK\r\n");
}


//************************BMI160************************//
uint8_t BMI160_INIT_REG[]=
{
	0x40,0x3A,//ACCEL ODR   1600Hz
	0x42,0x0C,//GYRO  ODR   1600Hz
	0x41,0x03,//ACCEL RANGE ¡À2G
	0x43,0x00 //GYRO  RANGE ¡À2000dps 16.4
};

void BMI160_INIT(u8 pos)
{
	u8 t,i,DATA[6];
	
	t=sizeof(BMI160_INIT_REG)/2;
	for(i=0;i<t;i++)
	{
			i2cWrite(IIC_ADDRESS_LIST[pos][0], BMI160_INIT_REG[2*i], BMI160_INIT_REG[2*i+1]);
			BMI160_INIT_REG[2*i+1]=0X00;
	}
	Delay_ms(100);	
	for(i=0;i<t;i++)
	{
			i2cRead(IIC_ADDRESS_LIST[pos][0], BMI160_INIT_REG[2*i], 1,&BMI160_INIT_REG[2*i+1]);
	}
	Delay_ms(10);	//ÅÐ¶ÏÐ´ÈëÊÇ·ñ³É¹¦
	
//	printf("BMI160_WR\r\n");
	t=0x11;
	i2cWrite(IIC_ADDRESS_LIST[pos][0],0x7e,t);
	Delay_ms(100);
	t=0x15;
	i2cWrite(IIC_ADDRESS_LIST[pos][0],0x7e,t);	
	Delay_ms(100);
	
	//¶ÁÈ¡Ò»´Î£¬É¾³ý¸Ã´ÎÊý¾Ý¡£	
	i2cRead(IIC_ADDRESS_LIST[pos][0], IIC_ADDRESS_LIST[pos][3], 6,DATA);
	Delay_ms(2);
	i2cRead(IIC_ADDRESS_LIST[pos][0], IIC_ADDRESS_LIST[pos][4], 6,DATA);
	Delay_ms(2);
	
//	printf("BMI160_INIT_OK\r\n");
	
}

uint8_t BMP280_CAL_PARA_REG[]=
{
	0x88,0x00,0x00,//BMP280_REGISTER_DIG_T1 1
	0x8A,0x00,0x00,//BMP280_REGISTER_DIG_T2 3
	0x8C,0x00,0x00,//BMP280_REGISTER_DIG_T3 5

	0x8E,0x00,0x00,//BMP280_REGISTER_DIG_P1 7
	0x90,0x00,0x00,//BMP280_REGISTER_DIG_P2 9
	0x92,0x00,0x00,//BMP280_REGISTER_DIG_P3 11
	0x94,0x00,0x00,//BMP280_REGISTER_DIG_P4 13
	0x96,0x00,0x00,//BMP280_REGISTER_DIG_P5 15
	0x98,0x00,0x00,//BMP280_REGISTER_DIG_P6 17
	0x9A,0x00,0x00,//BMP280_REGISTER_DIG_P7 19
	0x9C,0x00,0x00,//BMP280_REGISTER_DIG_P8 21
	0x9E,0x00,0x00 //BMP280_REGISTER_DIG_P9 23
};

uint16_t BMP280_CAL_PARA_S16[]=
{
0x0000,//BMP280_REGISTER_DIG_T1 0
0x0000,//BMP280_REGISTER_DIG_T2 1
0x0000,//BMP280_REGISTER_DIG_T3 2

0x0000,//BMP280_REGISTER_DIG_P1 3
0x0000,//BMP280_REGISTER_DIG_P2 4
0x0000,//BMP280_REGISTER_DIG_P3 5
0x0000,//BMP280_REGISTER_DIG_P4 6
0x0000,//BMP280_REGISTER_DIG_P5 5
0x0000,//BMP280_REGISTER_DIG_P6 8
0x0000,//BMP280_REGISTER_DIG_P7 9
0x0000,//BMP280_REGISTER_DIG_P8 10
0x0000 //BMP280_REGISTER_DIG_P9 11
};

void	BMP280_READ_CAL_PARA(u8 pos)
{
	u8 t,i;
	t=sizeof(BMP280_CAL_PARA_REG)/3;
	for(i=0;i<t;i++)
	{
			i2cRead(IIC_ADDRESS_LIST[pos][0], BMP280_CAL_PARA_REG[3*i], 2,&BMP280_CAL_PARA_REG[3*i+1]);
	}
	
	t=sizeof(BMP280_CAL_PARA_S16)/2;
	for(i=0;i<t;i++)
	{
			BMP280_CAL_PARA_S16[i]=(uint16_t)(BMP280_CAL_PARA_REG[3*i+2]<<8)+(uint16_t)(BMP280_CAL_PARA_REG[3*i+1]);
	}

}

void	BMP280_CAL_PRESS_TEMP(int32_t BMP280_PRESS,	int32_t BMP280_Temp)
{
	int32_t i,j;
  int32_t T_var1, T_var2,T;	
	int64_t P_var1, P_var2, p;
	int32_t t_fine;
	T_var1  = ((((BMP280_Temp>>3) - ((int32_t)BMP280_CAL_PARA_S16[0] <<1))) * 
						((int32_t)BMP280_CAL_PARA_S16[1])) >> 11;

	T_var2  = (((((BMP280_Temp>>4) - ((int32_t)BMP280_CAL_PARA_S16[0])) * 
						((BMP280_Temp>>4) - ((int32_t)BMP280_CAL_PARA_S16[0]))) >> 12) * 
						((int32_t)BMP280_CAL_PARA_S16[2])) >> 14;

	t_fine = T_var1 + T_var2;
	T  = (t_fine * 5 + 128) >> 8;
//	return T/100;	
	
  P_var1 = ((int64_t)t_fine) - 128000;
  P_var2 = P_var1 * P_var1 * (int64_t)BMP280_CAL_PARA_S16[8];
  P_var2 = P_var2 + ((P_var1*(int64_t)BMP280_CAL_PARA_S16[7])<<17);
  P_var2 = P_var2 + (((int64_t)BMP280_CAL_PARA_S16[6])<<35);
  P_var1 = ((P_var1 * P_var1 * (int64_t)BMP280_CAL_PARA_S16[5])>>8) +((P_var1 * (int64_t)BMP280_CAL_PARA_S16[4])<<12);
  P_var1 = (((((int64_t)1)<<47)+P_var1))*((int64_t)BMP280_CAL_PARA_S16[3])>>33;
  
  if (P_var1 == 0) {
//    return 0;  // avoid exception caused by division by zero
  }
	else
	{
		p = 1048576 - BMP280_PRESS;
		p = (((p<<31) - P_var2)*3125) / P_var1;
		P_var1 = (((int64_t)BMP280_CAL_PARA_S16[11]) * (p>>13) * (p>>13)) >> 25;
		P_var2 = (((int64_t)BMP280_CAL_PARA_S16[10]) * p) >> 19;

		p = ((p + P_var1 + P_var2) >> 8) + (((int64_t)BMP280_CAL_PARA_S16[9])<<4);
		
		p =(uint32_t)p;
		
		i=0;
//		return (float)p/256;
	}


}



uint8_t BMP280_INIT_REG1[]=
{
	0xf4,0x93	  // 8*sample pressure,8*sample temperyre// normal mode
};

void BMP280_INIT(u8 pos)//24,25
{
	u8 t,i,DATA[6];
	int32_t BMP280_PRESS_VALUE=0;
	int32_t BMP280_Temp_VALUE=0;	
	//ONE	
	t=sizeof(BMP280_INIT_REG1)/2;
	for(i=0;i<t;i++)
	{
			i2cWrite(IIC_ADDRESS_LIST[pos][0], BMP280_INIT_REG1[2*i], BMP280_INIT_REG1[2*i+1]);
			BMP280_INIT_REG1[2*i+1]=0X00;
	}
	Delay_ms(100);	
	for(i=0;i<t;i++)
	{
			i2cRead(IIC_ADDRESS_LIST[pos][0], BMP280_INIT_REG1[2*i], 1,&BMP280_INIT_REG1[2*i+1]);
	}
	Delay_ms(100);	//ÅÐ¶ÏÐ´ÈëÊÇ·ñ³É¹¦

	//¶ÁÈ¡Ò»´Î£¬É¾³ý¸Ã´ÎÊý¾Ý¡£	
	i2cRead(IIC_ADDRESS_LIST[pos][0], IIC_ADDRESS_LIST[pos][3], 6,DATA);
	Delay_ms(2);
	
	BMP280_PRESS_VALUE=(DATA[0]<<16)|(DATA[1]<<8)|(DATA[2]);
	BMP280_PRESS_VALUE=BMP280_PRESS_VALUE>>4;
	
	BMP280_Temp_VALUE=(DATA[3]<<16)|(DATA[4]<<8)|(DATA[5]);
	BMP280_Temp_VALUE=BMP280_Temp_VALUE>>4;	
	Delay_ms(2);
	BMP280_READ_CAL_PARA(pos);
	Delay_ms(2);
	BMP280_CAL_PRESS_TEMP(BMP280_PRESS_VALUE,	BMP280_Temp_VALUE);
	Delay_ms(2);

}



//************************SC7P20************************//

uint8_t SC7P20_INIT_REG1[]=
{
	0x1a,0x20,	  // µ÷½ÚÆ«ÖÃµçÁ÷Îª10uA£¬Test4¹Ü½Å
	0x1d,0xfb,	  // µ÷½ÚÆµÂÊ£¬HF&LF ¹Ü½Å²é¿´
	0x3a,0xc2,	  // µ÷½ÚLDO»ù×¼£¬¸ßÎ»¹Ì¶¨Îª1,0x80+Æ«ÒÆ   				    
	   				    // BG_TC µ÷½ÚÎÂ¶ÈÏµÊý£¬µ÷½Úµ½ÁãÎÂÆ¯
					      // BG_C£¬¾ø¶ÔÖµ £¬Test2¹Ü½Å
};

uint8_t SC7P20_INIT_REG2[]=
{
	0x01, 0x0, 
	//	0x02, 0x80,
	//	0x03, 0x80,
	// 0x04, set IA1_T_IEN, external Vin(ep.5522A)
	// 0x08, set TIN_VCMS, VCOM input 
	0x10, 0x7a,// if ODR=25Hz,reg 10H is suggested to set 6AH
	// avg  
	// bit[3:0] avgP, 0b1010, 512 max; 0b0000, 1
	// bit[6:4] avgT, 0b111, 128; 0b000,1
	0x13,0x90,	// open chop ,T_GC=center c0
	// close chop,T_GC=center 40
   /*pressure calibretion*/
	0x14,0x0,
	0x15,0x0,
	0x17,0x0,
	0x1e,0x27,
   /**********************/
	0x20, 0xc0,
	0x2f,0x0,
 /*******************************
 Õâ¼¸¸ö¼Ä´æÆ÷Öµ£¬ÓÐÊ±ºò»áÓÐÆ«²î£¬Õý³£Îª0£»
 Îª²âÊÔµçÂ·ÐÔÄÜ£¬Èí¼þÖÃÁã£¬ÒÔ±£³ÖÕý³£¹¤×÷×´Ì¬¡£
 *******************************/
	0x08,0x00,
	0x09,0x00,
	0x0a,0x00,
	0x25,0x00,
	0x26,0x00,
	0x3C,0x00,
	0x3D,0x00,
	0x3E,0x00,

	0x11,0x35,
	0x12,0x1e,
	0x16,0x00,
	0x18,0x00,
	0x19,0x00,
	0x1b,0x00,
	0x1c,0x00,

	0x1f,0x00,
	0x30,0x00,	
	0x31,0x00,	
	0x32,0x00,
	0x33,0x00,
	0x34,0x00,
	0x35,0x00,	
	0x36,0x00,	
	0x37,0x00,
	0x38,0x00,
	0x39,0x00,
	0x3B,0x00,	
	0x3C,0x00,
	0x3D,0x00,
	0x3E,0x00,
	0x3F,0x00,

/********************************/	 
};
void SC7P20_INIT(u8 pos)//26,27
{
	u8 t,i,DATA[6];
	int32_t SC7P20_PRESS_VALUE=0;
	int16_t SC7P20_Temp_VALUE=0;	
	//ONE	
	t=sizeof(SC7P20_INIT_REG1)/2;
	for(i=0;i<t;i++)
	{
			i2cWrite(IIC_ADDRESS_LIST[pos][0], SC7P20_INIT_REG1[2*i], SC7P20_INIT_REG1[2*i+1]);
			SC7P20_INIT_REG1[2*i+1]=0X00;
	}
	Delay_ms(100);	
	for(i=0;i<t;i++)
	{
			i2cRead(IIC_ADDRESS_LIST[pos][0], SC7P20_INIT_REG1[2*i], 1,&SC7P20_INIT_REG1[2*i+1]);
	}
	Delay_ms(100);	//ÅÐ¶ÏÐ´ÈëÊÇ·ñ³É¹¦
	//TWO
	t=sizeof(SC7P20_INIT_REG2)/2;
	for(i=0;i<t;i++)
	{
			i2cWrite(IIC_ADDRESS_LIST[pos][0], SC7P20_INIT_REG2[2*i], SC7P20_INIT_REG2[2*i+1]);
			SC7P20_INIT_REG2[2*i+1]=0X00;
	}
	Delay_ms(100);	
	for(i=0;i<t;i++)
	{
			i2cRead(IIC_ADDRESS_LIST[pos][0], SC7P20_INIT_REG2[2*i], 1,&SC7P20_INIT_REG2[2*i+1]);
	}
	Delay_ms(100);	//ÅÐ¶ÏÐ´ÈëÊÇ·ñ³É¹¦	
	
	//¶ÁÈ¡Ò»´Î£¬É¾³ý¸Ã´ÎÊý¾Ý¡£	
	i2cRead(IIC_ADDRESS_LIST[pos][0], IIC_ADDRESS_LIST[pos][3], 5,DATA);
	Delay_ms(2);
	SC7P20_PRESS_VALUE=(DATA[2]<<16)|(DATA[1]<<8)|(DATA[0]);
	SC7P20_PRESS_VALUE=SC7P20_PRESS_VALUE/4096;
	SC7P20_Temp_VALUE=(DATA[4]<<8)|(DATA[3]);
	SC7P20_Temp_VALUE=42.5+SC7P20_Temp_VALUE/480;	
	Delay_ms(2);
}




	uint8_t BMA421_INIT_REG[]=
{
	0x7d,	0x04,//ENABLE sensor acc
	0x40,	0x27,//continue filter nomal mode 400 ODR
	0x41,	0x00,//Acc_Range 00:2g  01:4g  02:8g  03:16g
	0x7c, 0x03,

	0x1d, 0x80,//enable data_ready


	
};


char BMA421_INIT(u8 pos)
{
	u8 t=0,i=0,DATA[6]={0xff,0xff,0xff,0xff,0xff,0xff},DATA2[1];
	u8 Int_Data[24],len[5]={8,16,12,20,24},
	Int1_Data[24]={0x00,0x00,0x78,0X00,0x00,0x18,0x32,0x01,
		             0xCC,0xf1,0x84,0x00,0x39,0xd9,0x07,0x00,
		             0xC9,0xEA,0X54,0XFD,0X05,0X00,0XAA,0X00};
	u8 rslt,rslt1;
	
	t=sizeof(BMA421_INIT_REG)/2;
	
	
	
	
		

		
			rslt=	BMA421_Write_File(16);//		enable feature in	
			

 
	i2cWrite(IIC_ADDRESS_LIST[pos][0], 0x7c, 0x00);
	Delay_ms(100);	
	i2cWrite(IIC_ADDRESS_LIST[pos][0], 0x59, 0x00);
	i2cRead(IIC_ADDRESS_LIST[pos][0],0x5e,24,Int_Data);
	i2cWriteBuffer(IIC_ADDRESS_LIST[pos][0],0x5e,24,Int1_Data);
	i2cWrite(IIC_ADDRESS_LIST[pos][0], 0x59, 0x01);
		Delay_ms(200);								 
  //i2cRead(IIC_ADDRESS_LIST[pos][0],0x2A,1,DATA2);
	i2cRead(IIC_ADDRESS_LIST[pos][0],0x5e,24,Int_Data);//ÅÐ¶ÏÊÇ·ñ¶ÁÐ´³É¹¦5e¼Ä´æÆ÷		
								 
	if(rslt==1)			
	{
			GPIO_ResetBits(GPIOB, GPIO_Pin_7);	
		for(i=0;i<t;i++)
	{
			i2cWrite(IIC_ADDRESS_LIST[pos][0], BMA421_INIT_REG[2*i], BMA421_INIT_REG[2*i+1]);
			BMA421_INIT_REG[2*i+1]=0X00;				 
	}
	
	}
	
	
	
	
	Delay_ms(100);	
	for(i=0;i<t;i++)
	{
			i2cRead(IIC_ADDRESS_LIST[pos][0], BMA421_INIT_REG[2*i], 1,&BMA421_INIT_REG[2*i+1]);
	}
	Delay_ms(100);	//ÅÐ¶ÏÐ´ÈëÊÇ·ñ³É¹¦
//	
//	//printf("BMA255_WR\r\n");
//	
//	//¶ÁÈ¡Ò»´Î£¬É¾³ý¸Ã´ÎÊý¾Ý¡£	
//	i2cRead(IIC_ADDRESS_LIST[pos][0], IIC_ADDRESS_LIST[pos][3], 6,DATA);
//	Delay_ms(200);

//	//printf("BMA255_INIT_OK\r\n");
	return 1;
	
}


uint8_t Kion_INIT_REG[]=
{
//	0x18,	0x40,//2g
//	0x1b,	0x01,//25HZ ODR
//  0x18,	0xD0,
	0x18,	0x40,
  0x1b,	0x07,
	//0x35, 0x08,

  0x18,	0xe0,
};
                                                  
char Kion_INIT(u8 pos)
{
	u8 t=0,i=0,DATA[6],DATA2[1];
	
	t=sizeof(Kion_INIT_REG)/2;
	
	for(i=0;i<t;i++)
	{
			//i2cWrite(0x08, Kion_INIT_REG[2*i], Kion_INIT_REG[2*i+1]); //kx022
				i2cWrite(0x1f, Kion_INIT_REG[2*i], Kion_INIT_REG[2*i+1]); //kx023
	}
	Delay_ms(100);
		//i2cRead(0x1f, 0x1b, 1,DATA2);
	 //i2cRead(0x1f, 0x15,1,DATA2);
			    //i2cRead(0x1f, 0x13,1,DATA2);

	return 1;

		

}

uint8_t MMA7660_INIT_REG[]=
{
//	0x18,	0x40,//2g
//	0x1b,	0x01,//25HZ ODR
//  0x18,	0xD0,
	0x07,	0x00,
	0X08, 0X00,
	0X09, 0X0d,
	0X07, 0X01
  

  
};

char MMA7660_INIT(u8 pos)
{
	u8 t=0,i=0,DATA[6],DATA2[1];
	
	t=sizeof(MMA7660_INIT_REG)/2;
	
	for(i=0;i<t;i++)
	{
			i2cWrite(0X4C, MMA7660_INIT_REG[2*i], MMA7660_INIT_REG[2*i+1]);
				
	}
	Delay_ms(100);
		i2cRead(0X4C, 0X07, 1,DATA2);

	return 1;

		

}
uint8_t QMA6981_INIT_REG[]=
{
	0x0F,	0x01,//PMU_RANGE:¡À2G
	0x10,	0x07,//ODR=60HZ
	0x11,	0x80,//normal mode  &¡¡sleep duration=2MS
	//0x12,	0x8f,
	//0x13,	0x0C,
	//0x14,	0x14,
	//0x15,	0x10,
	//0x16,	0x08,	
	//0x32,	0x00,
	 0x17, 0x10,//enable drdy
	 //0x21, 0x20,
};
char QMA6981_INIT(u8 pos)
{
	u8 t=0,i=0,DATA[6],DATA2[1];
	
	t=sizeof(QMA6981_INIT_REG)/2;
	
	for(i=0;i<t;i++)
	{
			i2cWrite(0X12, QMA6981_INIT_REG[2*i], QMA6981_INIT_REG[2*i+1]);
				
	}
	Delay_ms(100);
		i2cRead(0X12, 0X21, 1,DATA2);

	return 1;

}

uint8_t KX126_INIT_REG[]=
{
	0x1a, 0x40,//¡À2G
	0x1f, 0x07,// odr 25hz
	0x1a, 0xe0//
	
};

char KX126_INIT(u8 pos)
{
	u8 t=0,i=0,DATA[6],DATA2[1];
	
	t=sizeof(KX126_INIT_REG)/2;
	
	for(i=0;i<t;i++)
	{
			i2cWrite(0X1f, KX126_INIT_REG[2*i], KX126_INIT_REG[2*i+1]);
				
	}
	Delay_ms(100);
		i2cRead(0X1f, 0X17, 1,DATA2);

	return 1;

}
uint8_t LIS2DW_INIT_REG[]=
{
   0x20,	0x44,
	
};

char LIS2DW_INIT(u8 pos)
{
	u8 t=0,i=0,DATA[6],DATA2[1];
	
	t=sizeof(LIS2DW_INIT_REG)/2;
	
	for(i=0;i<t;i++)
	{
			i2cWrite(0X19, LIS2DW_INIT_REG[2*i], LIS2DW_INIT_REG[2*i+1]);
				
	}
	Delay_ms(100);
		i2cRead(0X19, 0X17, 1,DATA2);

	return 1;

}






u8 DEVICE_INIT(void)
{
		//¼ÓËÙ¶È¼Æ
		if(DEVICE_LIST_CONNECT[0]==1)//7A30E_L
		{
				SC7A30_INIT(0);
				MEMS_DEVICE_NUM++;
		}
		if(DEVICE_LIST_CONNECT[1]==1)//7A30E_H
		{
				SC7A30_INIT(1);
				MEMS_DEVICE_NUM++;
		}
		if(DEVICE_LIST_CONNECT[2]==1)//7A20_L
		{
				SC7A20_INIT(2);
				MEMS_DEVICE_NUM++;
		}
		if(DEVICE_LIST_CONNECT[3]==1)//7A20_H
		{
				SC7A20_INIT(3);
				MEMS_DEVICE_NUM++;
		}
		if(DEVICE_LIST_CONNECT[4]==1)//SC7660_L
		{
			
				MEMS_DEVICE_NUM++;
		}
		if(DEVICE_LIST_CONNECT[5]==1)//SC7660_H
		{
				MEMS_DEVICE_NUM++;
		}
		if(DEVICE_LIST_CONNECT[6]==1)//BMA255_L
		{
				BMA255_INIT(6);			
				MEMS_DEVICE_NUM++;
		}
		if(DEVICE_LIST_CONNECT[7]==1)//BMA255_H
		{
				BMA255_INIT(7);
				MEMS_DEVICE_NUM++;
		}
		if(DEVICE_LIST_CONNECT[8]==1)//ST3DH_L
		{
				MEMS_DEVICE_NUM++;
		}
		if(DEVICE_LIST_CONNECT[9]==1)//ST3DH_H
		{
				MEMS_DEVICE_NUM++;
		}
		if(DEVICE_LIST_CONNECT[10]==1)//ADXL345_L
		{
				MEMS_DEVICE_NUM++;
		}
		if(DEVICE_LIST_CONNECT[11]==1)//ADXL345_H
		{
				MEMS_DEVICE_NUM++;
		}
		//´ÅÁ¦¼Æ
		if(DEVICE_LIST_CONNECT[12]==1)//7M10_L
		{
				SC7M10_INIT(12);
				MEMS_DEVICE_NUM++;
		}
		if(DEVICE_LIST_CONNECT[13]==1)//7M10_H
		{
				SC7M10_INIT(13);
				MEMS_DEVICE_NUM++;
		}
		if(DEVICE_LIST_CONNECT[14]==1)//AK8975_L+8963
		{
				MEMS_DEVICE_NUM++;
		}
		if(DEVICE_LIST_CONNECT[15]==1)//AK8975_H+8963
		{
				MEMS_DEVICE_NUM++;
		}
		//6Öá´«¸ÐÆ÷
		if(DEVICE_LIST_CONNECT[16]==1)//SC7I20_L
		{
				SC7I20_INIT(16);
				MEMS_DEVICE_NUM++;
		}
		if(DEVICE_LIST_CONNECT[17]==1)//SC7I20_H
		{
				SC7I20_INIT(17);
				MEMS_DEVICE_NUM++;
		}
		if(DEVICE_LIST_CONNECT[18]==1)//MPU6050_L
		{
				MPU6050_INIT(18);	
				MEMS_DEVICE_NUM++;
		}
		if(DEVICE_LIST_CONNECT[19]==1)//MPU6050_H
		{
				MPU6050_INIT(19);
				MEMS_DEVICE_NUM++;
		}
		if(DEVICE_LIST_CONNECT[20]==1)//ST6DS3_L
		{
				ST6DS3_INIT(20);		
				MEMS_DEVICE_NUM++;
		}	
		if(DEVICE_LIST_CONNECT[21]==1)//ST6DS3_H
		{
//				ST6DS3_INIT(21);				
				MEMS_DEVICE_NUM++;
		}
		if(DEVICE_LIST_CONNECT[22]==1)//BMI160_L
		{
				BMI160_INIT(22);
				MEMS_DEVICE_NUM++;
		}
		if(DEVICE_LIST_CONNECT[23]==1)//BMI160_H
		{
				BMI160_INIT(23);
				MEMS_DEVICE_NUM++;
		}
		
		if(DEVICE_LIST_CONNECT[24]==1)
		{
				BMP280_INIT(24);
				MEMS_DEVICE_NUM++;
		}
		if(DEVICE_LIST_CONNECT[25]==1)
		{
				BMP280_INIT(25);
				MEMS_DEVICE_NUM++;
		}
		if(DEVICE_LIST_CONNECT[26]==1)
		{
				SC7P20_INIT(26);
				MEMS_DEVICE_NUM++;
		}
		if(DEVICE_LIST_CONNECT[27]==1)
		{
				SC7P20_INIT(27);
				MEMS_DEVICE_NUM++;
		}
		if(DEVICE_LIST_CONNECT[28]==1)
		{
       // MMA7660_INIT(28);
				MEMS_DEVICE_NUM++;
		}

		if(DEVICE_LIST_CONNECT[29]==1)
		{
	
				MEMS_DEVICE_NUM++;
		}
		if(DEVICE_LIST_CONNECT[30]==1)
		{
				BMA421_INIT(30);
				MEMS_DEVICE_NUM++;
		}

		if(DEVICE_LIST_CONNECT[31]==1)
		{
			//MMA7660_INIT(31);
						Kion_INIT(31);
				MEMS_DEVICE_NUM++;
		}
			if(DEVICE_LIST_CONNECT[32]==1)
		{
			
						QMA6981_INIT(32);
				MEMS_DEVICE_NUM++;
			
		}		
		if(DEVICE_LIST_CONNECT[33]==1)
		{
						KX126_INIT(33);
				MEMS_DEVICE_NUM++;
			
		}	
		if(DEVICE_LIST_CONNECT[34]==1)
		{
						LIS2DW_INIT(34);
				MEMS_DEVICE_NUM++;
			
		}	
			
		return 1;
}

u8	READ_ACCEL(void)
{
	u8 i=0,DATA[6],DATA3[1],ACC_NUM=0;
  u8  DATA1[4],DATA2[1];
	int16_t ACCEL_DATA[3];
	for(i=0;i<36;i++)
	{
		if(DEVICE_LIST_CONNECT[i]==1)
		{
       
			//	i2cRead(0X4C, 0x00,3,DATA);//mma7660 
//       i2cRead_dma(IIC_ADDRESS_LIST[i][0], 0x5e,0x04, 2, DATA1);
			    i2cRead(IIC_ADDRESS_LIST[i][0], IIC_ADDRESS_LIST[i][3],6,DATA);
			    i2cRead(IIC_ADDRESS_LIST[i][0], 0x1E,4,DATA1);
		     // i2cRead(0x1f, 0x06,6,DATA);kx023
				
					//ACCEL_DATA[0] =(int16_t)(((unsigned char)DATA[1] * 256) + (unsigned char)DATA[0]);//16Î»
					//ACCEL_DATA[1] =(int16_t)(((unsigned char)DATA[3] * 256) + (unsigned char)DATA[2]);//16Î»
					//ACCEL_DATA[2] =(int16_t)(((unsigned char)DATA[5] * 256) + (unsigned char)DATA[4]);//16Î»
				
			    ACCEL_DATA[0] =(int16_t)(((unsigned char)DATA[1] * 256) + (unsigned char)DATA[0]);//16Î»//qma6981
					ACCEL_DATA[1] =(int16_t)(((unsigned char)DATA[3] * 256) + (unsigned char)DATA[2]);//16Î»
					ACCEL_DATA[2] =(int16_t)(((unsigned char)DATA[5] * 256) + (unsigned char)DATA[4]);//16Î»

			
			
			
					STM32_DATA[0+ACC_NUM*10] =((unsigned int)ACCEL_DATA[0])/256;
					STM32_DATA[1+ACC_NUM*10] =((unsigned int)ACCEL_DATA[0])%256;
					STM32_DATA[2+ACC_NUM*10] =((unsigned int)ACCEL_DATA[1])/256;
					STM32_DATA[3+ACC_NUM*10] =((unsigned int)ACCEL_DATA[1])%256;
					STM32_DATA[4+ACC_NUM*10] =((unsigned int)ACCEL_DATA[2])/256;
					STM32_DATA[5+ACC_NUM*10] =((unsigned int)ACCEL_DATA[2])%256;
					
					
					
			//	STM32_DATA[0+ACC_NUM*10] =(((unsigned char)(DATA[0]&0X3F))<<2);//È¡ÓÐÐ§6Î»Êý¾Ý
			//	STM32_DATA[1+ACC_NUM*10] =(((unsigned char)(DATA[1]&0X3F))<<2);
			//	STM32_DATA[2+ACC_NUM*10] =(((unsigned char)(DATA[2]&0X3F))<<2);
				
				//STM32_DATA[3+ACC_NUM*10] =DATA3[0];
			
					//STM32_DATA[4+ACC_NUM*10] =DATA3[0]&0X80;
					//STM32_DATA[5+ACC_NUM*10] =DATA3[0]&0X20;
					
					Accel_Cal_Noise(ACC_NUM,ACCEL_DATA[0],ACCEL_DATA[1],ACCEL_DATA[2]);
					if((DATA1[0]!=0)||(DATA1[1])!=0||(DATA1[2]!=0)||(DATA1[3]!=0))
					{
						STM32_DATA[6+ACC_NUM*10] =1;//ÔëÉù
					}
					else
					STM32_DATA[6+ACC_NUM*10] =0;//ÔëÉù
					
					STM32_DATA[8+ACC_NUM*10] =DATA1[3];//ÔëÉù
				
					STM32_DATA[9+ACC_NUM*10] =IIC_ADDRESS_LIST[i][0];//IIC_ADDR
					ACC_NUM++;
					if(ACC_NUM==3) break;
		}
	}
	STM32_DATA[30] =ACC_NUM;//¶àÉÙ¸öACCÓÐÐ§
	STM32_DATA[31] =1;//Êý¾Ý°üÐòºÅ1
	
	if(ACC_NUM==0) return 0;
	else           return 1;
	
}





u8	READ_MAG(void)
{
	u8 i=0,DATA[6],MAG_NUM=0;
	int16_t MAG_DATA[3];
	for(i=12;i<16;i++)
	{
		if(DEVICE_LIST_CONNECT[i]==1)
		{

				i2cRead(IIC_ADDRESS_LIST[i][0], IIC_ADDRESS_LIST[i][3], 6,DATA);
			  if(IIC_ADDRESS_LIST[i][5]==1)//¸ßÎ»Êý¾ÝÔÚµÍµØÖ·
				{
					MAG_DATA[0] =(int16_t)(((unsigned char)DATA[1] * 256 ) + (unsigned char)DATA[0]);//16Î»
					MAG_DATA[1] =(int16_t)(((unsigned char)DATA[3] * 256 ) + (unsigned char)DATA[2]);//16Î»
					MAG_DATA[2] =(int16_t)(((unsigned char)DATA[5] * 256 ) + (unsigned char)DATA[4]);//16Î»
				}
				else
				{
					MAG_DATA[0] =(int16_t)(((unsigned char)DATA[1] * 256 ) + (unsigned char)DATA[0]);//16Î»
					MAG_DATA[1] =(int16_t)(((unsigned char)DATA[3] * 256 ) + (unsigned char)DATA[2]);//16Î»
					MAG_DATA[2] =(int16_t)(((unsigned char)DATA[5] * 256 ) + (unsigned char)DATA[4]);//16Î»
				}
				
					STM32_DATA[0+MAG_NUM*10] =((unsigned int)MAG_DATA[0])/256;
					STM32_DATA[1+MAG_NUM*10] =((unsigned int)MAG_DATA[0])%256;
					STM32_DATA[2+MAG_NUM*10] =((unsigned int)MAG_DATA[1])/256;
					STM32_DATA[3+MAG_NUM*10] =((unsigned int)MAG_DATA[1])%256;
					STM32_DATA[4+MAG_NUM*10] =((unsigned int)MAG_DATA[2])/256;
					STM32_DATA[5+MAG_NUM*10] =((unsigned int)MAG_DATA[2])%256;
					

					Mag_Cal_Noise(MAG_NUM,MAG_DATA[0],MAG_DATA[1],MAG_DATA[2]);
					STM32_DATA[6+MAG_NUM*10] =Mag_Noise_Result[MAG_NUM][0];//ÔëÉù
					STM32_DATA[7+MAG_NUM*10] =Mag_Noise_Result[MAG_NUM][1];//ÔëÉù
					STM32_DATA[8+MAG_NUM*10] =Mag_Noise_Result[MAG_NUM][2];//ÔëÉù
				
					STM32_DATA[9+MAG_NUM*10] =IIC_ADDRESS_LIST[i][0];//IIC_ADDR
					MAG_NUM++;
					if(MAG_NUM==3) break;
		}
	}
	STM32_DATA[30] =MAG_NUM;//¶àÉÙ¸öMAGÓÐÐ§
	STM32_DATA[31] =2;//Êý¾Ý°üÐòºÅ1
	
	if(MAG_NUM==0) return 0;
	else           return 1;

}


u8	READ_SIX_1(void)
{
	float P[5];
	static  float Filter_X[Filter_Num]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	//static  short Filter_X_FIX[Filter_Num]={1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};
	static  float Filter_Y[Filter_Num]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	//unsigned char keystatus_final=0;
	static unsigned char  Mouse_Dir=1;
	static unsigned char  Mouse_Dir_Temp=0;
	static unsigned char  Mouse_Dir_Counter[3]={0,0,0};	
	static unsigned int  Mouse_Time[4]={0,0,0,0};
	u8 t,i=0,j=0,m=0,DATA[6],Six_Num=0;
	u8 Screen_Width=0,Screen_Hight=0;
	int X,Y,X1,Y1;
	int16_t ACCEL_DATA[3],GYRO_DATA[3];
	float Accel_DATA[3],Gyro_DATA[3];
	short ACCEL_GYRO_DATA_BUF[20][6];
	signed long SC7I20_Avr_DATA[6]={0,0,0,0,0,0};
	short Six_Avr_AccelandGyro_result [6]={0,0,0,0,0,0};
	for(i=16;i<22;i++)
	{
		if(DEVICE_LIST_CONNECT[i]==1)
		{
				i2cRead(IIC_ADDRESS_LIST[i][0], IIC_ADDRESS_LIST[i][3], 6,DATA);
			  if(IIC_ADDRESS_LIST[i][5]==1)//¸ßÎ»Êý¾ÝÔÚµÍµØÖ·
				{
					ACCEL_DATA[0] =(int16_t)(((unsigned char)DATA[0] * 256 ) + (unsigned char)DATA[1]);//16Î»
					ACCEL_DATA[1] =(int16_t)(((unsigned char)DATA[2] * 256 ) + (unsigned char)DATA[3]);//16Î»
					ACCEL_DATA[2] =(int16_t)(((unsigned char)DATA[4] * 256 ) + (unsigned char)DATA[5]);//16Î»
				}
				else
				{
					ACCEL_DATA[0] =(int16_t)(((unsigned char)DATA[1] * 256 ) + (unsigned char)DATA[0]);//16Î»
					ACCEL_DATA[1] =(int16_t)(((unsigned char)DATA[3] * 256 ) + (unsigned char)DATA[2]);//16Î»
					ACCEL_DATA[2] =(int16_t)(((unsigned char)DATA[5] * 256 ) + (unsigned char)DATA[4]);//16Î»
				}
				
//				ACCEL_DATA[0] =	ACCEL_DATA[0]>>8;
//				ACCEL_DATA[1] =	ACCEL_DATA[1]>>8;
//				ACCEL_DATA[2] =	ACCEL_DATA[2]>>8; 
				
					STM32_DATA[0] =((unsigned int)ACCEL_DATA[0])/256;
					STM32_DATA[1] =((unsigned int)ACCEL_DATA[0])%256;
					STM32_DATA[2] =((unsigned int)ACCEL_DATA[1])/256;
					STM32_DATA[3] =((unsigned int)ACCEL_DATA[1])%256;
					STM32_DATA[4] =((unsigned int)ACCEL_DATA[2])/256;
					STM32_DATA[5] =((unsigned int)ACCEL_DATA[2])%256;
					
				
				i2cRead(IIC_ADDRESS_LIST[i][0], IIC_ADDRESS_LIST[i][4], 6,DATA);
			  if(IIC_ADDRESS_LIST[i][5]==1)//¸ßÎ»Êý¾ÝÔÚµÍµØÖ·
				{
					GYRO_DATA[0] =(int16_t)(((unsigned char)DATA[0] * 256 ) + (unsigned char)DATA[1]);//16Î»
					GYRO_DATA[1] =(int16_t)(((unsigned char)DATA[2] * 256 ) + (unsigned char)DATA[3]);//16Î»
					GYRO_DATA[2] =(int16_t)(((unsigned char)DATA[4] * 256 ) + (unsigned char)DATA[5]);//16Î»
				}
				else
				{
					GYRO_DATA[0] =(int16_t)(((unsigned char)DATA[1] * 256 ) + (unsigned char)DATA[0]);//16Î»
					GYRO_DATA[1] =(int16_t)(((unsigned char)DATA[3] * 256 ) + (unsigned char)DATA[2]);//16Î»
					GYRO_DATA[2] =(int16_t)(((unsigned char)DATA[5] * 256 ) + (unsigned char)DATA[4]);//16Î»
				}
				
				GYRO_DATA[0] = GYRO_DATA[0]-ACCEL_GYRO_STATIONARY_DATA[0];
				GYRO_DATA[1] =(GYRO_DATA[1]-ACCEL_GYRO_STATIONARY_DATA[1]);
				GYRO_DATA[2] =(GYRO_DATA[2]-ACCEL_GYRO_STATIONARY_DATA[2]);
				
					STM32_DATA[9]  =((unsigned int)GYRO_DATA[0])/256;
					STM32_DATA[10] =((unsigned int)GYRO_DATA[0])%256;
					STM32_DATA[11] =((unsigned int)GYRO_DATA[1])/256;
					STM32_DATA[12] =((unsigned int)GYRO_DATA[1])%256;
					STM32_DATA[13] =((unsigned int)GYRO_DATA[2])/256;
					STM32_DATA[14] =((unsigned int)GYRO_DATA[2])%256;
//					STM32_DATA[15] =0;//GYROÔëÉù
//					STM32_DATA[16] =0;//ÔëÉù
//					STM32_DATA[17] =0;//ÔëÉù

				  //¶ÔÊý¾Ý½øÐÐ´¦Àí²¿·Ö
					//3¸öÖá¶¼½øÐÐ½Ç¶È»»Ëã
			   	Accel_DATA[0]=(32768-(ACCEL_DATA[0]+32768)*1.0)/16384;//¶àÉÙG?==>½Ç¶È¶àÉÙ£¿
					Accel_DATA[1]=(32768-(ACCEL_DATA[1]+32768)*1.0)/16384;//¶àÉÙG?==>½Ç¶È¶àÉÙ£¿		
					Accel_DATA[2]=(32768-(ACCEL_DATA[2]+32768)*1.0)/16384;//¶àÉÙG?==>½Ç¶È¶àÉÙ£¿
					for(j=0;j<3;j++)
					{
							if(Accel_DATA[j]>1) 			Accel_DATA[j] =  1; 
							else if(Accel_DATA[j]<-1)	Accel_DATA[j] = -1;
							else	Accel_DATA[j]=Accel_DATA[j];
							Accel_DATA[j]=(asinf(Accel_DATA[j]))*180/3.1415926;//½Ç¶ÈÖµÕæÊµ½Ç¶ÈÖµ		asinf³öÀ´µÄÊÇ»¡¶ÈÖµ ×¢Òâ	
					}
					//full scale :2000dps-->14.3;	
					Gyro_DATA[0]=(32768-(GYRO_DATA[0]+32768)*1.0)/14.3;//×ª¹ýÁË¶àÉÙ¶È14.3 28000×óÓÒÂúÁ¿³Ì
					Gyro_DATA[1]=(32768-(GYRO_DATA[1]+32768)*1.0)/14.3;//×ª¹ýÁË¶àÉÙ¶È14.3
					Gyro_DATA[2]=(32768-(GYRO_DATA[2]+32768)*1.0)/14.3;//×ª¹ýÁË¶àÉÙ¶È14.3
					//ÈýÖákalmanÂË²¨
          //GPIOB->ODR^=GPIO_Pin_11;//È¡·´¹Ü½Å	
					Kalman_Filter_X(Accel_DATA[0],Gyro_DATA[0]);//pitch
					
				    // GPIO_ResetBits(GPIOB, GPIO_Pin_11);
//					Kalman_Filter_Y(Accel_DATA[1],Gyro_DATA[1]);//Yaw 
//					Kalman_Filter_Z(Accel_DATA[2],Gyro_DATA[2]);//Roll
					
//								//ÇÐÖá 0 LEFT 1 MID  2 RIGHT
////					Mouse_Time[3]=0;
//				if((keystatus==0x01)&&(keystatus_flag&0x01==0x01)&&(keystatus_flag&0x80)!=0x80)
//				{                   //µ¥»÷flagµÍµÚÒ»Î»Îª1£¬statusÎª1£¬ÇÒflag¸ßÎ»²»Îª1£¬µ¥»÷½øÈëºóflag¸ßÎ»Îª1.
//						if((keystatus_flag&0x02)!=0x02) //µ¥»÷ºó µÍµÚ¶þÎ»Îª1
//						{
//							keystatus_flag=keystatus_flag|0x02;
//								  keystatus_final=0x01;
//						}
//					
//					  if(keystatus_flag&0x03==0x03)
//						{
//									Mouse_Time[0]++;
//									if(Mouse_Time[0]>10)      //  10*3*8ms
//									{
//										 keystatus_flag=keystatus_flag|0x80;	//µ¥»÷³¤°´flg¸ßÎ»Îª1£¬µÍ¶þÎ»ÇåÁã					
//										 Mouse_Time[0]=0;
//                     keystatus_flag=keystatus_flag&0xFD;
//									}
//						}		
//				}
//				else if(keystatus==0x00&&(keystatus_flag&0x02)==0x02&&(keystatus_flag&0x80)!=0x80)
//				{                             //µ¥»÷µ¯Æð ´Î¸ßÎ»ÖÃ1

//							if((keystatus_flag&0x04)!=0x04) 
//							{
//									keystatus_flag=keystatus_flag|0x04;
//							
//							}
//							
//							Mouse_Time[1]++;
//							if(Mouse_Time[1]>30)
//							{
//									keystatus_flag=keystatus_flag|0x80; //µ¯ÆðºÜ¾ÃºóÄ¬ÈÏÎª³õÊ¼×´Ì¬
//								  Mouse_Time[1]=0;
//							}	
//				}
//				else if((keystatus==0x01&&(keystatus_flag&0x04==0x04))||((keystatus_flag&0x80==0x80)&&(keystatus_flag&0x40!=0x40)))
//				{
//					 Mouse_Time[0]=0;
//					 Mouse_Time[1]=0;
//					 Mouse_Time[2]++;
//					 keystatus_final=0x01;
//					 if(Mouse_Time[2]>1)
//					 {
//						  Mouse_Time[2]=0;
//					    keystatus_flag=(keystatus_flag&0X81)|0xC0;//µ¥»÷³¤°´		
//              keystatus_final=0x00;						 
//					 }	 
//				}
//				else if(keystatus==0x00&&(keystatus_flag&0x40==0x40))
//				{
////			     Mouse_Time[3]++;
////					 if(Mouse_Time[3]>10)
////					 {
//					   keystatus_flag=0X00;
//					   
////					 }
//				}

//		
//		switch(keystatus_flag)
//		{
//			case 0X81: 
//			case 0XC1: 			
//			case 0X00: 	
			    if((angle_X<=100)&&(angle_X>80))//ÓÒÆ«×ª90
						{
							Mouse_Dir_Counter[2]++;
							Mouse_Dir_Counter[1]=0;
							Mouse_Dir_Counter[0]=0;
							if(Mouse_Dir_Counter[2]>10)   Mouse_Dir = 2 ;
									
					 }
					 else if((angle_X<=-80)&&(angle_X>-100))
						{
							Mouse_Dir_Counter[0]++;
							Mouse_Dir_Counter[1]=0;
							Mouse_Dir_Counter[2]=0;
							if(Mouse_Dir_Counter[0]>10)   Mouse_Dir = 0 ;
						}
					 else if((angle_X<=10)&&(angle_X>-10))
						{
							Mouse_Dir_Counter[1]++;
							Mouse_Dir_Counter[0]=0;
							Mouse_Dir_Counter[2]=0;
							if(Mouse_Dir_Counter[1]>10)   Mouse_Dir = 1 ;
						}
					 else
						{
					//±£³Ö²»±ä
						}	
//					 break;				
////	   case 1:
////						Mouse_Dir=3;	
////			    break;
////		 case 2:
////				    Mouse_Dir=3;
////					break;
//		 default: 
//			   Mouse_Dir=3;
//				 break;
//	}			
		
			//¸ù¾ÝÆÁÄ»·Ö±æÂÊµ÷½ÚÁéÃô¶È»òÕßÍ¨¹ýÉÏÏÂ°´¼üÀ´µ÷½ÚÁéÃô¶È
//Screen_Width=GetSystemMetrics(SM_CXSCREEN);			
						
			switch(Mouse_Dir)
			{
				case 0:
					if(GYRO_DATA[0]*GYRO_DATA[0]>1500)//       ½ÇËÙ¶È×îÐ¡¸Ä±äÁ¿50
							X=(GYRO_DATA[0])/Sensitivity;          //      ¿Éµ÷ÁéÃô¶È
					else
						X=0;
					
					if(GYRO_DATA[2]*GYRO_DATA[2]>1500)
						Y=(-GYRO_DATA[2])/Sensitivity;
					else
						Y=0;
				  break;
					
				case 1:
					if(GYRO_DATA[0]*GYRO_DATA[0]>1500)//       ½ÇËÙ¶È×îÐ¡¸Ä±äÁ¿50
							Y=(GYRO_DATA[0])/Sensitivity;//      ¿Éµ÷ÁéÃô¶È
					else
							Y=0;
					if(GYRO_DATA[2]*GYRO_DATA[2]>1500)
							X=(GYRO_DATA[2])/Sensitivity;
					else
							X=0;
				  break;			
	
				case 2:
					if(GYRO_DATA[0]*GYRO_DATA[0]>1500)//       ½ÇËÙ¶È×îÐ¡¸Ä±äÁ¿50
								X=(-GYRO_DATA[0])/Sensitivity;       //      ¿Éµ÷ÁéÃô¶È
					else
					      X=0;
					if(GYRO_DATA[2]*GYRO_DATA[2]>1500)
					     Y=(GYRO_DATA[2])/Sensitivity;
					else
					     Y=0;		
				  break;
				case 3:	
               X=0;
               Y=0;				
					break;
				
				default:
					break;
			
			}

							for(m=Filter_Num;m>0;m--)//×îÐÂµÄÊý¾Ý½øÈë ·ÅÈëÊý×é×îºó½øÐÐ×îÐ¡¶þ³Ë
								{
									Filter_X[m-2]=Filter_X[m-1];	//×óÒÆ¶¯±£´æÊý¾Ý
									Filter_Y[m-2]=Filter_Y[m-1];	
								}
//										Filter_X[Filter_Num-1]=X+Filter_X[Filter_Num-2];//×îÐÂÊý¾Ý·ÅÔÚÊý×é×îºóÒ»Î»
//									Filter_Y[Filter_Num-1]=Y+Filter_Y[Filter_Num-2];
						
						      Filter_X[Filter_Num-1]=X;//×îÐÂÊý¾Ý·ÅÔÚÊý×é×îºóÒ»Î»
									Filter_Y[Filter_Num-1]=Y;
					
								  //LineFitting(Filter_X,Filter_Y,Filter_Num);//ÏßÐÔÄâºÏ
//						GPIOB->ODR^=GPIO_Pin_11;//È¡·´¹Ü½Å	
//						polyfit(Filter_Num, Filter_X, Filter_Y, Filter_Jnum, P); //×îÐ¡¶þ³Ë·¨½øÄâºÏ
////						GPIOB->ODR^=GPIO_Pin_11;//È¡·´¹Ü½Å			
//								  X1=X;//Ô­Ê¼Êý¾Ý±£´æ
//								  Y1=Y;
//								
								
								
////						     			 	
////					}
	//½øÈ¥Ò»¸öÊý×öÒ»´Î×îÐ¡¶þ³Ë£¬À´ÐÞÕýµ±Ç°×ø±ê				
					
//				if(X==0||Y==0)
//				{
//					
//				}
//        else
//				{
//					Y=P[0]+P[1]*X+P[2]*X*X+P[3]*X*X*X+P[4]*X*X*X*X;
//				}			
//									X=X+Filter_X[Filter_Num-2];
//									Y=Y+Filter_Y[Filter_Num-2];				
//				if(B_D>200)
//				{
//						while(1)
//						{
//						}
//				}
			
				  STM32_DATA[15] =((unsigned int)X)/256;
					STM32_DATA[16] =((unsigned int)X)%256;
					STM32_DATA[17] =((unsigned int)Y)/256;
				  STM32_DATA[18] =((unsigned int)Y)%256;
	        STM32_DATA[19] =keystatus;
	        STM32_DATA[20] =Roller_KEY;
			
					STM32_DATA[21] =((unsigned int)((signed int)(angle_X*100)))/256;
					STM32_DATA[22] =((unsigned int)((signed int)(angle_X*100)))%256;
					STM32_DATA[23] =((unsigned int)((signed int)(angle_Y*100)))/256;
					STM32_DATA[24] =((unsigned int)((signed int)(angle_Y*100)))%256;	
					STM32_DATA[25] =((unsigned int)((signed int)(angle_Z*100)))/256;
					STM32_DATA[26] =((unsigned int)((signed int)(angle_Z*100)))%256;//ÈÚºÏ½Ç¶ÈÖµ×Ô¼º		
         			
					
					STM32_DATA[27] =IIC_ADDRESS_LIST[i][0];//IIC_ADDR
				  STM32_DATA[28] =((unsigned int)X1)/256;
					STM32_DATA[29] =((unsigned int)X1)%256;
					STM32_DATA[30] =((unsigned int)Y1)/256;
				  STM32_DATA[6] =((unsigned int)Y1)%256;
//			STM32_DATA[30] =(unsigned int)(Filter_X[19])/256;
//			STM32_DATA[6]  =(unsigned int)(Filter_X[19])%256;
//      STM32_DATA[28] =(unsigned int)(Filter_Y[19])/256;
//			STM32_DATA[29] =(unsigned int)(Filter_Y[19])%256;
			
					
					STM32_DATA[31] =3;//Êý¾Ý°üÐòºÅ1
					ACCEL_GYRO_NUM=i;
					break;
		}
	}
	
	if(ACCEL_GYRO_NUM==0) return 0;
	else                  return 1;

}



u8	READ_SIX_2(void)
{
	u8 i=0,j=0,DATA[6];
	int16_t ACCEL_DATA[3],GYRO_DATA[3];
	u8 LAST_ACCEL_GYRO_NUM=0;
	if(ACCEL_GYRO_NUM==0)//Î´²å6Öá´«¸ÐÆ÷
	{
		ACCEL_GYRO_NUM=0;
	  return 0;	
	}
	else
	{
		LAST_ACCEL_GYRO_NUM=ACCEL_GYRO_NUM;//±£´æµ±Ç°×´Ì¬Öµ	
	}

	if(ACCEL_GYRO_NUM>15&&ACCEL_GYRO_NUM<23)
	for(i=ACCEL_GYRO_NUM+1;i<23;i++)
	{
		if(DEVICE_LIST_CONNECT[i]==1)
		{
				i2cRead(IIC_ADDRESS_LIST[i][0], IIC_ADDRESS_LIST[i][3], 6,DATA);
			  if(IIC_ADDRESS_LIST[i][5]==1)//¸ßÎ»Êý¾ÝÔÚµÍµØÖ·
				{
					ACCEL_DATA[0] =(int16_t)(((unsigned char)DATA[0] * 256 ) + (unsigned char)DATA[1]);//16Î»
					ACCEL_DATA[1] =(int16_t)(((unsigned char)DATA[2] * 256 ) + (unsigned char)DATA[3]);//16Î»
					ACCEL_DATA[2] =(int16_t)(((unsigned char)DATA[4] * 256 ) + (unsigned char)DATA[5]);//16Î»
				}
				else
				{
					ACCEL_DATA[0] =(int16_t)(((unsigned char)DATA[1] * 256 ) + (unsigned char)DATA[0]);//16Î»
					ACCEL_DATA[1] =(int16_t)(((unsigned char)DATA[3] * 256 ) + (unsigned char)DATA[2]);//16Î»
					ACCEL_DATA[2] =(int16_t)(((unsigned char)DATA[5] * 256 ) + (unsigned char)DATA[4]);//16Î»
				}
				
					STM32_DATA[0] =((unsigned int)ACCEL_DATA[0])/256;
					STM32_DATA[1] =((unsigned int)ACCEL_DATA[0])%256;
					STM32_DATA[2] =((unsigned int)ACCEL_DATA[1])/256;
					STM32_DATA[3] =((unsigned int)ACCEL_DATA[1])%256;
					STM32_DATA[4] =((unsigned int)ACCEL_DATA[2])/256;
					STM32_DATA[5] =((unsigned int)ACCEL_DATA[2])%256;
					STM32_DATA[6] =0;//ACCELÔëÉù
					STM32_DATA[7] =0;//ÔëÉù
					STM32_DATA[8] =0;//ÔëÉù
				
				i2cRead(IIC_ADDRESS_LIST[i][0], IIC_ADDRESS_LIST[i][4], 6,DATA);
			  if(IIC_ADDRESS_LIST[i][5]==1)//¸ßÎ»Êý¾ÝÔÚµÍµØÖ·
				{
					GYRO_DATA[0] =(int16_t)(((unsigned char)DATA[0] * 256 ) + (unsigned char)DATA[1]);//16Î»
					GYRO_DATA[1] =(int16_t)(((unsigned char)DATA[2] * 256 ) + (unsigned char)DATA[3]);//16Î»
					GYRO_DATA[2] =(int16_t)(((unsigned char)DATA[4] * 256 ) + (unsigned char)DATA[5]);//16Î»
				}
				else
				{
					GYRO_DATA[0] =(int16_t)(((unsigned char)DATA[1] * 256 ) + (unsigned char)DATA[0]);//16Î»
					GYRO_DATA[1] =(int16_t)(((unsigned char)DATA[3] * 256 ) + (unsigned char)DATA[2]);//16Î»
					GYRO_DATA[2] =(int16_t)(((unsigned char)DATA[5] * 256 ) + (unsigned char)DATA[4]);//16Î»
				}
				
					STM32_DATA[9]  =((unsigned int)GYRO_DATA[0])/256;
					STM32_DATA[10] =((unsigned int)GYRO_DATA[0])%256;
					STM32_DATA[11] =((unsigned int)GYRO_DATA[1])/256;
					STM32_DATA[12] =((unsigned int)GYRO_DATA[1])%256;
					STM32_DATA[13] =((unsigned int)GYRO_DATA[2])/256;
					STM32_DATA[14] =((unsigned int)GYRO_DATA[2])%256;
					STM32_DATA[15] =0;//GYROÔëÉù
					STM32_DATA[16] =0;//ÔëÉù
					STM32_DATA[17] =0;//ÔëÉù
					//KALMAN X
					//KALMAN Y
					//KALMAN Z
					//KALMAN X NOSE
					//KALMAN Y NOSE
					//KALMAN Z NOSE				
					STM32_DATA[18] =0;//GYROX
					STM32_DATA[19] =0;//X
					STM32_DATA[20] =0;//y
					STM32_DATA[21] =0;//y
					STM32_DATA[22] =0;//z
					STM32_DATA[23] =0;//z
					STM32_DATA[24] =0;//GYROÔëÉù
					STM32_DATA[25] =0;//ÔëÉù
					STM32_DATA[26] =0;//ÔëÉù
					
					STM32_DATA[27] =IIC_ADDRESS_LIST[i][0];//IIC_ADDR
					STM32_DATA[31] =4;//Êý¾Ý°üÐòºÅ1
					ACCEL_GYRO_NUM=i;
					break;
		}
	}

	if(LAST_ACCEL_GYRO_NUM==ACCEL_GYRO_NUM)//µÚÒ»¸ö¹ýºó£¬Ã»ÓÐ6ÖáÁË
  {
		 ACCEL_GYRO_NUM=0;
		 return 0;
  }
	else   
	{
		return 1;
	}		

}
u8	READ_SIX_3(void)
{
	u8 i=0,DATA[6];
	int16_t ACCEL_DATA[3],GYRO_DATA[3];
	u8 LAST_ACCEL_GYRO_NUM=0;
	if(ACCEL_GYRO_NUM==0)//Î´²å6Öá´«¸ÐÆ÷
	{
		ACCEL_GYRO_NUM=0;
	  return 0;	
	}
	else
	{
		LAST_ACCEL_GYRO_NUM=ACCEL_GYRO_NUM;//±£´æµ±Ç°×´Ì¬Öµ	
	}
	
	if(ACCEL_GYRO_NUM>16&&ACCEL_GYRO_NUM<23)	
	for(i=ACCEL_GYRO_NUM+1;i<23;i++)
	{
		if(DEVICE_LIST_CONNECT[i]==1)
		{
				i2cRead(IIC_ADDRESS_LIST[i][0], IIC_ADDRESS_LIST[i][3], 6,DATA);
			  if(IIC_ADDRESS_LIST[i][5]==1)//¸ßÎ»Êý¾ÝÔÚµÍµØÖ·
				{
					ACCEL_DATA[0] =(int16_t)(((unsigned char)DATA[1] * 256 ) + (unsigned char)DATA[0]);//16Î»
					ACCEL_DATA[1] =(int16_t)(((unsigned char)DATA[3] * 256 ) + (unsigned char)DATA[2]);//16Î»
					ACCEL_DATA[2] =(int16_t)(((unsigned char)DATA[5] * 256 ) + (unsigned char)DATA[4]);//16Î»
				}
				else
				{
					ACCEL_DATA[0] =(int16_t)(((unsigned char)DATA[1] * 256 ) + (unsigned char)DATA[0]);//16Î»
					ACCEL_DATA[1] =(int16_t)(((unsigned char)DATA[3] * 256 ) + (unsigned char)DATA[2]);//16Î»
					ACCEL_DATA[2] =(int16_t)(((unsigned char)DATA[5] * 256 ) + (unsigned char)DATA[4]);//16Î»
				}
				
					STM32_DATA[0] =((unsigned int)ACCEL_DATA[0])/256;
					STM32_DATA[1] =((unsigned int)ACCEL_DATA[0])%256;
					STM32_DATA[2] =((unsigned int)ACCEL_DATA[1])/256;
					STM32_DATA[3] =((unsigned int)ACCEL_DATA[1])%256;
					STM32_DATA[4] =((unsigned int)ACCEL_DATA[2])/256;
					STM32_DATA[5] =((unsigned int)ACCEL_DATA[2])%256;
					STM32_DATA[6] =0;//ACCELÔëÉù
					STM32_DATA[7] =0;//ÔëÉù
					STM32_DATA[8] =0;//ÔëÉù
				
				i2cRead(IIC_ADDRESS_LIST[i][0], IIC_ADDRESS_LIST[i][4], 6,DATA);
			  if(IIC_ADDRESS_LIST[i][5]==1)//¸ßÎ»Êý¾ÝÔÚµÍµØÖ·
				{
					GYRO_DATA[0] =(int16_t)(((unsigned char)DATA[1] * 256 ) + (unsigned char)DATA[0]);//16Î»
					GYRO_DATA[1] =(int16_t)(((unsigned char)DATA[3] * 256 ) + (unsigned char)DATA[2]);//16Î»
					GYRO_DATA[2] =(int16_t)(((unsigned char)DATA[5] * 256 ) + (unsigned char)DATA[4]);//16Î»
				}
				else
				{
					GYRO_DATA[0] =(int16_t)(((unsigned char)DATA[1] * 256 ) + (unsigned char)DATA[0]);//16Î»
					GYRO_DATA[1] =(int16_t)(((unsigned char)DATA[3] * 256 ) + (unsigned char)DATA[2]);//16Î»
					GYRO_DATA[2] =(int16_t)(((unsigned char)DATA[5] * 256 ) + (unsigned char)DATA[4]);//16Î»
				}
				
					STM32_DATA[9]  =((unsigned int)GYRO_DATA[0])/256;
					STM32_DATA[10] =((unsigned int)GYRO_DATA[0])%256;
					STM32_DATA[11] =((unsigned int)GYRO_DATA[1])/256;
					STM32_DATA[12] =((unsigned int)GYRO_DATA[1])%256;
					STM32_DATA[13] =((unsigned int)GYRO_DATA[2])/256;
					STM32_DATA[14] =((unsigned int)GYRO_DATA[2])%256;
					STM32_DATA[15] =0;//GYROÔëÉù
					STM32_DATA[16] =0;//ÔëÉù
					STM32_DATA[17] =0;//ÔëÉù
					//KALMAN X
					//KALMAN Y
					//KALMAN Z
					//KALMAN X NOSE
					//KALMAN Y NOSE
					//KALMAN Z NOSE				
					STM32_DATA[18] =0;//GYROX
					STM32_DATA[19] =0;//X
					STM32_DATA[20] =0;//y
					STM32_DATA[21] =0;//y
					STM32_DATA[22] =0;//z
					STM32_DATA[23] =0;//z
					STM32_DATA[24] =0;//GYROÔëÉù
					STM32_DATA[25] =0;//ÔëÉù
					STM32_DATA[26] =0;//ÔëÉù
					
					STM32_DATA[27] =IIC_ADDRESS_LIST[i][0];//IIC_ADDR
					
					STM32_DATA[31] =5;//Êý¾Ý°üÐòºÅ1
					ACCEL_GYRO_NUM=0;//ÖØÐÂ¿ªÊ¼¼ÆÊý
					break;
		}
	}
	
	if(LAST_ACCEL_GYRO_NUM==ACCEL_GYRO_NUM)//µÚÒ»¸ö¹ýºó£¬Ã»ÓÐ6ÖáÁË
  {
		 ACCEL_GYRO_NUM=0;
		 return 0;
  }
	else   
	{
		return 1;
	}	
	
}


u8	READ_SIX_FIFO_TEST(void)
{
	u8 i=0,j=0,DATA[6];
	u8 FIFO_FULL_FLAG[2];
	int16_t ACCEL_DATA[3][10],GYRO_DATA[3][10];
	int16_t ACCEL_DATA_AVG[3]={0,0,0},GYRO_DATA_AVG[3]={0,0,0};
	for(i=16;i<18;i++)
	{
		if(DEVICE_LIST_CONNECT[i]==1)
		{	
			i2cRead(IIC_ADDRESS_LIST[i][0], 0x1D,1,&FIFO_FULL_FLAG[0]);	
			FIFO_FULL_FLAG[0]=FIFO_FULL_FLAG[0]&0x80;//ÉèÖÃ10¸öÊý¾Ý		
			i2cRead(IIC_ADDRESS_LIST[i][0], 0x2F,1,&FIFO_FULL_FLAG[1]);	
			FIFO_FULL_FLAG[1]=FIFO_FULL_FLAG[1]&0x80;	//ÉèÖÃ10¸öÊý¾Ý		
			if(FIFO_FULL_FLAG[0]==0x80&&FIFO_FULL_FLAG[1]==0x80)
			{
					for(j=0;j<10;j++)
					{
						i2cRead(IIC_ADDRESS_LIST[i][0], IIC_ADDRESS_LIST[i][3], 6,DATA);					
						if(IIC_ADDRESS_LIST[i][5]==1)//¸ßÎ»Êý¾ÝÔÚµÍµØÖ·
						{
							ACCEL_DATA[0][j] =(int16_t)(((unsigned char)DATA[0] * 256 ) + (unsigned char)DATA[1]);//16Î»
							ACCEL_DATA[1][j] =(int16_t)(((unsigned char)DATA[2] * 256 ) + (unsigned char)DATA[3]);//16Î»
							ACCEL_DATA[2][j] =(int16_t)(((unsigned char)DATA[4] * 256 ) + (unsigned char)DATA[5]);//16Î»
						}
						else
						{
							ACCEL_DATA[0][j] =(int16_t)(((unsigned char)DATA[1] * 256 ) + (unsigned char)DATA[0]);//16Î»
							ACCEL_DATA[1][j] =(int16_t)(((unsigned char)DATA[3] * 256 ) + (unsigned char)DATA[2]);//16Î»
							ACCEL_DATA[2][j] =(int16_t)(((unsigned char)DATA[5] * 256 ) + (unsigned char)DATA[4]);//16Î»
						}
						i2cRead(IIC_ADDRESS_LIST[i][0], IIC_ADDRESS_LIST[i][4], 6,DATA);
						if(IIC_ADDRESS_LIST[i][5]==1)//¸ßÎ»Êý¾ÝÔÚµÍµØÖ·
						{
							GYRO_DATA[0][j] =(int16_t)(((unsigned char)DATA[0] * 256 ) + (unsigned char)DATA[1]);//16Î»
							GYRO_DATA[1][j] =(int16_t)(((unsigned char)DATA[2] * 256 ) + (unsigned char)DATA[3]);//16Î»
							GYRO_DATA[2][j] =(int16_t)(((unsigned char)DATA[4] * 256 ) + (unsigned char)DATA[5]);//16Î»
						}
						else
						{
							GYRO_DATA[0][j] =(int16_t)(((unsigned char)DATA[1] * 256 ) + (unsigned char)DATA[0]);//16Î»
							GYRO_DATA[1][j] =(int16_t)(((unsigned char)DATA[3] * 256 ) + (unsigned char)DATA[2]);//16Î»
							GYRO_DATA[2][j] =(int16_t)(((unsigned char)DATA[5] * 256 ) + (unsigned char)DATA[4]);//16Î»
						}
				}
				i2cWrite(IIC_ADDRESS_LIST[i][0], 0x1C, 0x00);
				i2cWrite(IIC_ADDRESS_LIST[i][0], 0X2E, 0x00);
				for(j=0;j<10;j++)
				{
						ACCEL_DATA_AVG[0]=ACCEL_DATA_AVG[0]+ACCEL_DATA[0][j]/10;
						ACCEL_DATA_AVG[1]=ACCEL_DATA_AVG[1]+ACCEL_DATA[1][j]/10;
						ACCEL_DATA_AVG[2]=ACCEL_DATA_AVG[2]+ACCEL_DATA[2][j]/10;
						GYRO_DATA_AVG[0]=GYRO_DATA_AVG[0]+GYRO_DATA[0][j]/10;
						GYRO_DATA_AVG[1]=GYRO_DATA_AVG[1]+GYRO_DATA[1][j]/10;
						GYRO_DATA_AVG[2]=GYRO_DATA_AVG[2]+GYRO_DATA[2][j]/10;
				}
				
				STM32_DATA[0] =((unsigned int)ACCEL_DATA_AVG[0])/256;
				STM32_DATA[1] =((unsigned int)ACCEL_DATA_AVG[0])%256;
				STM32_DATA[2] =((unsigned int)ACCEL_DATA_AVG[1])/256;
				STM32_DATA[3] =((unsigned int)ACCEL_DATA_AVG[1])%256;
				STM32_DATA[4] =((unsigned int)ACCEL_DATA_AVG[2])/256;
				STM32_DATA[5] =((unsigned int)ACCEL_DATA_AVG[2])%256;
	
				STM32_DATA[9]  =((unsigned int)GYRO_DATA_AVG[0])/256;
				STM32_DATA[10] =((unsigned int)GYRO_DATA_AVG[0])%256;
				STM32_DATA[11] =((unsigned int)GYRO_DATA_AVG[1])/256;
				STM32_DATA[12] =((unsigned int)GYRO_DATA_AVG[1])%256;
				STM32_DATA[13] =((unsigned int)GYRO_DATA_AVG[2])/256;
				STM32_DATA[14] =((unsigned int)GYRO_DATA_AVG[2])%256;		
					
				STM32_DATA[6] =0;//ACCELÔëÉù
				STM32_DATA[7] =0;//ÔëÉù
				STM32_DATA[8] =0;//ÔëÉù					
				STM32_DATA[15] =0;//GYROÔëÉù
				STM32_DATA[16] =0;//ÔëÉù
				STM32_DATA[17] =0;//ÔëÉù

				//3¸öÖá¶¼½øÐÐ½Ç¶È»»Ëã
				ACCEL_DATA_AVG[0]=(32768-(ACCEL_DATA_AVG[0]+32768)*1.0)/16384;//¶àÉÙG?==>½Ç¶È¶àÉÙ£¿
				ACCEL_DATA_AVG[1]=(32768-(ACCEL_DATA_AVG[1]+32768)*1.0)/16384;//¶àÉÙG?==>½Ç¶È¶àÉÙ£¿		
				ACCEL_DATA_AVG[2]=(32768-(ACCEL_DATA_AVG[2]+32768)*1.0)/16384;//¶àÉÙG?==>½Ç¶È¶àÉÙ£¿
				for(j=0;j<3;j++)
				{
						if(ACCEL_DATA_AVG[j]>1) 			ACCEL_DATA_AVG[j] =  1; 
						else if(ACCEL_DATA_AVG[j]<-1)	ACCEL_DATA_AVG[j] = -1;
						else	ACCEL_DATA_AVG[j]=ACCEL_DATA_AVG[j];
						ACCEL_DATA_AVG[j]=ACCEL_DATA_AVG[j]*57.3;//½Ç¶ÈÖµÕæÊµ½Ç¶ÈÖµ			
				}
				//full scale :2000dps-->14.3;	
				GYRO_DATA_AVG[0]=(32768-(GYRO_DATA_AVG[0]+32768)*1.0)/14.3;//×ª¹ýÁË¶àÉÙ¶È14.3
				GYRO_DATA_AVG[1]=(32768-(GYRO_DATA_AVG[1]+32768)*1.0)/14.3;//×ª¹ýÁË¶àÉÙ¶È14.3
				GYRO_DATA_AVG[2]=(32768-(GYRO_DATA_AVG[2]+32768)*1.0)/14.3;//×ª¹ýÁË¶àÉÙ¶È14.3
			
				STM32_DATA[27] =IIC_ADDRESS_LIST[i][0];//IIC_ADDR
				STM32_DATA[31] =3;//Êý¾Ý°üÐòºÅ1		


				
				i2cWrite(IIC_ADDRESS_LIST[i][0], 0x1C, 0x4A);
				i2cWrite(IIC_ADDRESS_LIST[i][0], 0X2E, 0x4A);				
				return  1;

			}
			else
			{
					return 0;
			}
		}
	}
	
	return 0;
}


u8	READ_FIFO_ACCEL(void)
{
	u8 i=0,j=0,DATA[6],ACC_NUM=0;
	u8 FIFO_FULL_FLAG[2];
	int16_t ACCEL_DATA[3][31];
	int16_t ACCEL_DATA_AVG[3]={0,0,0};
	for(i=0;i<12;i++)
	{
		if(DEVICE_LIST_CONNECT[i]==1)
		{
//			i2cRead(IIC_ADDRESS_LIST[i][0], 0x2F,1,&FIFO_FULL_FLAG[0]);			
//			FIFO_FULL_FLAG[0]=FIFO_FULL_FLAG[0]&0x40;//ÉèÖÃ10¸öÊý¾Ý	
//			if(FIFO_FULL_FLAG[0]==0x40)			
//			if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15)==0)
//			{
						for(j=0;j<32;j++)
						{
								i2cRead(IIC_ADDRESS_LIST[i][0], IIC_ADDRESS_LIST[i][3], 6,DATA);					
								if(IIC_ADDRESS_LIST[i][5]==1)//¸ßÎ»Êý¾ÝÔÚµÍµØÖ·
								{
										ACCEL_DATA[0][j] =(int16_t)(((unsigned char)DATA[0] * 256 ) + (unsigned char)DATA[1]);//16Î»
										ACCEL_DATA[1][j] =(int16_t)(((unsigned char)DATA[2] * 256 ) + (unsigned char)DATA[3]);//16Î»
										ACCEL_DATA[2][j] =(int16_t)(((unsigned char)DATA[4] * 256 ) + (unsigned char)DATA[5]);//16Î»
								}
								else
								{
										ACCEL_DATA[0][j] =(int16_t)(((unsigned char)DATA[1] * 256 ) + (unsigned char)DATA[0]);//16Î»
										ACCEL_DATA[1][j] =(int16_t)(((unsigned char)DATA[3] * 256 ) + (unsigned char)DATA[2]);//16Î»
										ACCEL_DATA[2][j] =(int16_t)(((unsigned char)DATA[5] * 256 ) + (unsigned char)DATA[4]);//16Î»
								}		
						}
						i2cWrite(IIC_ADDRESS_LIST[i][0], 0X2E, 0x00);//BY PASS FIFO			
						for(j=0;j<32;j++)
						{
								ACCEL_DATA_AVG[0]=ACCEL_DATA_AVG[0]+ACCEL_DATA[0][j]/32;
								ACCEL_DATA_AVG[1]=ACCEL_DATA_AVG[1]+ACCEL_DATA[1][j]/32;
								ACCEL_DATA_AVG[2]=ACCEL_DATA_AVG[2]+ACCEL_DATA[2][j]/32;
						}
						STM32_DATA[0] =((unsigned int)ACCEL_DATA_AVG[0])/256;
						STM32_DATA[1] =((unsigned int)ACCEL_DATA_AVG[0])%256;
						STM32_DATA[2] =((unsigned int)ACCEL_DATA_AVG[1])/256;
						STM32_DATA[3] =((unsigned int)ACCEL_DATA_AVG[1])%256;
						STM32_DATA[4] =((unsigned int)ACCEL_DATA_AVG[2])/256;
						STM32_DATA[5] =((unsigned int)ACCEL_DATA_AVG[2])%256;
						Accel_Cal_Noise(ACC_NUM,ACCEL_DATA_AVG[0],ACCEL_DATA_AVG[1],ACCEL_DATA_AVG[2]);
						STM32_DATA[6+ACC_NUM*10] =Accel_Noise_Result[ACC_NUM][0];//ÔëÉù
						STM32_DATA[7+ACC_NUM*10] =Accel_Noise_Result[ACC_NUM][1];//ÔëÉù
						STM32_DATA[8+ACC_NUM*10] =Accel_Noise_Result[ACC_NUM][2];//ÔëÉù
				
						STM32_DATA[9+ACC_NUM*10] =IIC_ADDRESS_LIST[i][0];//IIC_ADDR
						ACC_NUM++;
						i2cWrite(IIC_ADDRESS_LIST[i][0], 0X2E, 0x40);	
		}
	}
	STM32_DATA[30] =ACC_NUM;//¶àÉÙ¸öACCÓÐÐ§
	STM32_DATA[31] =1;//Êý¾Ý°üÐòºÅ1
	
	if(ACC_NUM==0) return 0;
	else           return 1;
	
}



u8	READ_Heart_Rate(void)
{
	u8 i=0,DATA[6],ACC_NUM=0;
	int16_t HR_DATA[3];
	u8 HR_INT_STATUS=0;
	for(i=30;i<31;i++)
	{
		if(DEVICE_LIST_CONNECT[i]==1)
		{
			
					i2cRead(IIC_ADDRESS_LIST[i][0], 0x21, 1,&HR_INT_STATUS);
					i2cRead(IIC_ADDRESS_LIST[i][0], 0X22, 6,DATA);
					HR_DATA[0] =(int16_t)(((unsigned char)DATA[1] * 256 ) + (unsigned char)DATA[0]);//16Î» VIS
					HR_DATA[1] =(int16_t)(((unsigned char)DATA[3] * 256 ) + (unsigned char)DATA[2]);//16Î» IR
					HR_DATA[2] =(int16_t)(((unsigned char)DATA[5] * 256 ) + (unsigned char)DATA[4]);//16Î» PS1
					
					STM32_DATA[0+ACC_NUM*10] =((unsigned int)HR_DATA[0])/256;
					STM32_DATA[1+ACC_NUM*10] =((unsigned int)HR_DATA[0])%256;
					STM32_DATA[2+ACC_NUM*10] =((unsigned int)HR_DATA[1])/256;
					STM32_DATA[3+ACC_NUM*10] =((unsigned int)HR_DATA[1])%256;
					STM32_DATA[4+ACC_NUM*10] =((unsigned int)HR_DATA[2])/256;
					STM32_DATA[5+ACC_NUM*10] =((unsigned int)HR_DATA[2])%256;
				
					STM32_DATA[9+ACC_NUM*10] =IIC_ADDRESS_LIST[i][0];//IIC_ADDR
					ACC_NUM++;
					if(ACC_NUM==3) break;
					STM32_DATA[30] =ACC_NUM;//¶àÉÙ¸öACCÓÐÐ§
					STM32_DATA[31] =1;//Êý¾Ý°üÐòºÅ1
			
					i2cWrite(IIC_ADDRESS_LIST[i][0], 0x21, HR_INT_STATUS);
		}
	}
	
	if(ACC_NUM==0) return 0;
	else           return 1;

	
}








