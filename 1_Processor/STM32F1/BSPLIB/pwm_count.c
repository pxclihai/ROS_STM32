/***********************************************************************************************************************
* Copyright (c) Hands Free Team. All rights reserved.
* FileName: pwm_in.c
* Contact:  QQ Exchange Group -- 521037187
* Version:  V2.0
*
* LICENSING TERMS:  
* The Hands Free is licensed generally under a permissive 3-clause BSD license. 
* Contributions are requiredto be made under the same license.
*
* History: 
* <author>    <time>      <version >       <desc>
* mawenke     2015.10.1   V1.0             creat
* LiuDong     2016.1.8    V1.57            update the name of function
*
* Description:  capturing PWM can be realized by timer or external interrupt
*               this file take external interrupt to realize capturing, and define 5 channels
*               STM32F1--------------
*               PC0 PC1 PC2 PC3 PC4
***********************************************************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif 

#include "pwm_count.h"
#include "nvic.h"
#include "delay.h"

static float PWM_Input_CH[5];  //current input value of PWM (Unit:us)
void ETRInputConfig(TIM_TypeDef *timx);

/***********************************************************************************************************************
* Function:     void HF_PwmIn_Init(void)
*
* Scope:        public
*
* Description:  configure input of PWM,and capture the time of high level.
*                PC0 PC1 PC2 PC3 PC4  external interrupt line of Pin.
*
* Arguments:
*
* Return:
*
* Cpu_Time:
*
* History:
* by   mawenke    2015.12.1                          creat
* by   LiuDong    2016.1.8       V1.57       update the name of function
***********************************************************************************************************************/
void HF_PwmCount_Init(uint8_t pwmin_channel)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING; //
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //50M时钟速度
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE);
    ETRInputConfig(TIM4);
    TIM4->CNT = 0;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //50M时钟速度
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE);
    ETRInputConfig(TIM3);


}
void ETRInputConfig(TIM_TypeDef *timx)
{
  if(timx==TIM1) RCC->APB2ENR|=1<<11;    //最大 72M
  else if(timx==TIM2) RCC->APB1ENR|=1<<0;//最大 36M * 2
  else if(timx==TIM3) RCC->APB1ENR|=1<<1;
  else if(timx==TIM4) RCC->APB1ENR|=1<<2;
  else if(timx==TIM5) RCC->APB1ENR|=1<<3;
  //else if(timx==TIM6) RCC->APB1ENR|=1<<4; //这几个定时器不能 使用ETR
  //else if(timx==TIM7) RCC->APB1ENR|=1<<5;
  else if(timx==TIM8) RCC->APB2ENR|=1<<13;

  //设定 CR1
  timx->CR1&=~(u16)(0x3<<8);            //清空分频因子
  timx->CR1|=(u16)(0x0<<8);             //设定分频因子为0
  //timx->CR1|=(u16)(0x1<<7);             //自动重装请允许，TIMx_ARR寄存器被装入缓冲器
  timx->CR1&=~(u16)(0x3<<5);            //边沿对齐方式
  timx->CR1&=~(u16)(0x1<<4);            //向上计数

  //设定 SMCR 从模式寄存器
  timx->SMCR=(0x1<<15);//ETR 输入不反相，高电平/上升沿有效
  timx->SMCR|= (u16)(0x1<<14);//使能外部时钟2
  timx->SMCR&=~(u16)(0x3<<12);//ETPS,不分频
  timx->SMCR&=~(u16)(0xf<<8); //ETF,采样不分频
  //预分频值，不分频
  timx->PSC=0;
  //重装值，0xffff
  timx->ARR=~(u16)(0x0);
  //设定 DIER -- DMA/中断 制寄存器,关更新中断
  timx->DIER&=~(u16)(0x1<<0);
  //计数前先清空一下计数器
  TIM_ETRClockMode2Config(timx, TIM_ExtTRGPSC_OFF,TIM_ExtTRGPolarity_NonInverted, 5);//5次采样滤波  外部时钟模式2
  timx->CNT=0;

  timx->CR1|=(u16)(0x1<<0);             //开计数器
}
float HF_Get_PWMCount_Value(uint8_t channel_x)
{
    return PWM_Input_CH[channel_x];
}


#ifdef __cplusplus
}
#endif

