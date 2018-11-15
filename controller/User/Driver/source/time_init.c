/**
******************************************************************************
* @file    time_init.c
* @author
* @version V2.1.0
* @date    2-September-2014
* @brief   此文件主要定义定时器函数及PWM初始化
******************************************************************************
* @device  Master---STM32F407VET6
******************************************************************************
*/

#include "time_init.h"

void Time_init( void )
{
    NVIC_InitTypeDef NVIC_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    /* TIM3 clock enable */
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM3, ENABLE ); //84MHz
    NVIC_PriorityGroupConfig( NVIC_PriorityGroup_2 );
    /* Enable the TIM3 gloabal Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init( &NVIC_InitStructure );
    TIM_TimeBaseStructure.TIM_Period = 2499;
    TIM_TimeBaseStructure.TIM_Prescaler = 83;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit( TIM3, &TIM_TimeBaseStructure );
    TIM_ClearFlag( TIM3, TIM_FLAG_Update );
    TIM_ITConfig( TIM3, TIM_IT_Update, ENABLE );
    TIM_Cmd( TIM3, ENABLE );
}


void Tim2_init( void )
{
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM2, ENABLE );
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure ;
    TIM_TimeBaseStructure.TIM_Period = 99;//重装载
    TIM_TimeBaseStructure.TIM_Prescaler = 84;//84分频
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit( TIM2, &TIM_TimeBaseStructure ); //初始化定时器2
    TIM_ITConfig( TIM2, TIM_IT_Update, ENABLE ); //开定时器中断
    TIM_Cmd( TIM2, ENABLE ); //使能定时器2,10000hz
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init( &NVIC_InitStructure );
}
