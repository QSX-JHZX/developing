
/*
========================================================
						基于NEC红外协议的红外接收程序
========================================================
											By:祁书轩
========================================================
							TIME : 2024-12-26-00:27
========================================================
	  OPEN BUG：
								1、USART1串口通讯不能与I2C总线同时运行
								
								2、未实现重复码接收逻辑
========================================================

*/

#include "stm32f10x.h"
//#include "usart.h"
#include "delay.h"
#include "si2c.h"
#include "oled.h"

#define   hw_pin      GPIO_Pin_8 //红外信号引脚
//#define   usart_pin   GPIO_Pin_9 //串口TX引脚
#define   MODE        GPIO_Mode_IN_FLOATING//红外信号引脚模式

#define   SCL_GPIOX       GPIOB   //SCL引脚接口
#define		SDA_GPIOX				GPIOB		//SDA引脚接口
#define   SCL_PINX       	GPIO_Pin_6     //SCL PIN接口
#define		SDA_PINX		    GPIO_Pin_7	    //SDA PIN接口


SI2C_TypeDef si2c;  //si2c结构体
OLED_TypeDef oled;  //oled名字

static uint32_t  nct = 0;   // 存储当前红外接收的数据
static uint32_t  last = 0;  //储存为上一次接收的编码

void siic_init(void); //配置软I2C
void my_oled_Init(void); //配置OLED
int i2c_write_bytes(uint8_t addr,const uint8_t *pdata,uint16_t size);

void HWIO(void);		//配置红外接收引脚
void US_Init(void); //配置USART1串口
void Time(void); 		//配置输入捕获
void NV(void);			//配置中断优先级
void TIM1_CC_IRQHandler(void); //TIM1中断响应函数

int main(void)
{
	siic_init();
	my_oled_Init();
	
//	US_Init();	//配置USART1串口
	HWIO();			//配置红外接收引脚
	Time();			//配置输入捕获
  NV();			  //配置中断优先级
	
	
	OLED_SetCursor(&oled, 60, 32); 
	OLED_Printf(&oled, "wait");
	OLED_SendBuffer(&oled);
  
	while(1)
	{

		
		if(last != nct)  	 //判断是否重复接收相同数据
			{
					Delay(200);  //消除显示抖动
					OLED_Clear(&oled);
					OLED_SetCursor(&oled, 30, 32); 
					OLED_Printf(&oled, "NEC:%x", nct);
					OLED_SendBuffer(&oled);
					nct = 0;  
					last = nct; 
			}
	}
}

void siic_init(void)
{
	si2c.SCL_GPIOx = SCL_GPIOX;
	si2c.SCL_GPIO_Pin = SCL_PINX;
	si2c.SDA_GPIOx = SDA_GPIOX;
	si2c.SDA_GPIO_Pin = SDA_PINX;
	My_SI2C_Init(&si2c);
}

//oled 初始化
int i2c_write_bytes(uint8_t addr,const uint8_t *pdata,uint16_t size) 
{
	return My_SI2C_SendBytes(&si2c,addr,pdata,size);
}
void my_oled_Init(void)
{
	OLED_InitTypeDef oled1;
	oled1.i2c_write_cb = i2c_write_bytes;
	OLED_Init(&oled,&oled1);
	
	OLED_SetPen(&oled, PEN_COLOR_WHITE, 1); // 设置白画笔
	OLED_SetBrush(&oled, PEN_COLOR_TRANSPARENT); // 透明画刷
}



void TIM1_CC_IRQHandler(void)
{
	 static uint16_t lastCapture = 0;  // 上一次捕获的计数值
	 static uint8_t bitIndex = 0;      // 红外数据位索引
	 static uint32_t irData = 0;       // 存储红外接收的数据
	 uint16_t currentCapture;          // 当前捕获的计数值
 	 uint16_t pulseWidth;              // 脉冲宽度
	
	//判断输入捕获的中断标志位是否触发中断
	 if (TIM_GetITStatus(TIM1, TIM_IT_CC1) != RESET)
	 {

				currentCapture = TIM_GetCapture1(TIM1); //捕获的计数值

		    // 计算脉冲宽度
        if (currentCapture >= lastCapture)
        {
            pulseWidth = currentCapture - lastCapture;
        }
        else
        {
            pulseWidth = (0xFFFF - lastCapture) + currentCapture + 1;
        }
        lastCapture = currentCapture; // 更新上一次捕获值
				
				
				  if (pulseWidth > 13000 && pulseWidth < 14000) // 引导码：13.5ms
        {
            bitIndex = 0;    // 数据位索引清零
            irData = 0;      // 清空数据
        }
        else if (pulseWidth > 2000 && pulseWidth < 2500) // 逻辑 1：2250μs
        {
            irData |= (1 << (31 - bitIndex)); // 存储 1
            bitIndex++;
        }
        else if (pulseWidth > 900 && pulseWidth < 1300) // 逻辑 0：1120μs
        {
            irData &= ~(1 << (31 - bitIndex)); // 存储 0
            bitIndex++;
        }
				
				if (bitIndex >= 32) //接收满32位
				{
            bitIndex = 0;    // 数据位索引清零
				}
				
				nct = irData;
			 // 清除中断标志位
        TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);
				
	 
				
	 }

}


void HWIO(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitTypeDef A8;
	A8.GPIO_Mode = MODE ;
	A8.GPIO_Pin = hw_pin;
	GPIO_Init(GPIOA,&A8);
}

void Time(void)
{
	//使能定时器时钟单元
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	//配置时基单元
	TIM_TimeBaseInitTypeDef T;
	T.TIM_CounterMode = TIM_CounterMode_Up;
	T.TIM_Period = 65535;
	T.TIM_Prescaler = 71;
	T.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit (TIM1,&T);
	
	//输入捕获红外信号
	//CH1捕获下降沿
	TIM_ICInitTypeDef IC;
	IC.TIM_Channel = TIM_Channel_1;
	IC.TIM_ICFilter = 0;
	IC.TIM_ICPolarity = TIM_ICPolarity_Falling;
	IC.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	IC.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInit(TIM1,&IC);
	
	//清除中断标志位
	TIM_ClearFlag(TIM1, TIM_FLAG_CC1);

	//中断配置
	// 使能 CC1 通道捕获中断
	TIM_ITConfig(TIM1, TIM_IT_CC1, ENABLE); 
   
	//启动定时器
	TIM_Cmd(TIM1, ENABLE);

}

//void US_Init(void)
//{
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
//	GPIO_InitTypeDef A9;
//	A9.GPIO_Mode = GPIO_Mode_AF_PP;
//	A9.GPIO_Pin = usart_pin;
//	A9.GPIO_Speed = GPIO_Speed_2MHz;
//	GPIO_Init(GPIOA,&A9);
//	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
//	USART_InitTypeDef US;
//	US.USART_BaudRate = 115200;
//	US.USART_Mode = USART_Mode_Tx;
//	US.USART_Parity = USART_Parity_No;
//	US.USART_StopBits = USART_StopBits_1;
//	US.USART_WordLength = USART_WordLength_8b;
//	USART_Init(USART1,&US);
//	
//	USART_Cmd(USART1,ENABLE);
//}

void NV(void)
{
	NVIC_InitTypeDef NV;
	NV.NVIC_IRQChannel =  TIM1_CC_IRQn; 
	NV.NVIC_IRQChannelPreemptionPriority = 0;
	NV.NVIC_IRQChannelSubPriority = 1;
	NV.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NV);
}
