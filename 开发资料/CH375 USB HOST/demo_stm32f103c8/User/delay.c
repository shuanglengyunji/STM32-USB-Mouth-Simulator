#include "delay.h"

static void Delay_Timer_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    
    TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Down;
    TIM_TimeBaseInitStruct.TIM_Period = 100-1;
    TIM_TimeBaseInitStruct.TIM_Prescaler = (84-1);
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);
    
    while((TIM3->SR & TIM_FLAG_Update)!=SET);
    TIM3->SR = (uint16_t)~TIM_FLAG_Update;
}

static void Delay_us(uint32_t us_cnt)
{
    TIM3->CNT = us_cnt-1;
    TIM3->CR1 |= TIM_CR1_CEN;    
    while((TIM3->SR & TIM_FLAG_Update)!=SET);
    TIM3->SR = (uint16_t)~TIM_FLAG_Update;
    TIM3->CR1 &= ~TIM_CR1_CEN;
}

/////////////////////////////////////////////////////

void delay_init(void)
{
	Delay_Timer_Init();
}

void delay_us(uint32_t nus)
{
	Delay_us(nus);
}
	
void delay_ms(uint32_t nms)
{
	u16 i=0;  
	while(nms--)
	{
		i=12000;  //自己定义
		while(i--) ;    
	}
}
