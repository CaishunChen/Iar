/**
  ******************************************************************************
  * @file    ringbuffer.h
  * @author  UAV Application Team
  * @version V2.1.0
  * @date    2-September-2014
  * @brief   此文件主要包含ringbuffer头文件。
  ****************************************************************************** 
  * @device  Master---STM32F405RGT6
  ******************************************************************************  
*/ 
#ifndef __RINGBUFFER_H
#define __RINGBUFFER_H

#ifdef __cplusplus
 extern "C" {
#endif 


#include "stm32f4xx.h"
#include <stdio.h>

typedef struct {
	uint8_t* pBuff;
	uint8_t* pEnd;  // pBuff + legnth
	uint8_t* wp;    // Write Point
	uint8_t* rp;    // Read Point
	uint16_t length;
	uint8_t  flagOverflow; // set when buffer overflowed
} RingBuffer;

void rbInitialize(RingBuffer* pRingBuff, uint8_t* buff, uint16_t length);
void rbClear(RingBuffer* pRingBuff);
void rbPush(RingBuffer* pRingBuff, uint8_t value);
uint8_t rbPop(RingBuffer* pRingBuff);
uint16_t rbGetCount(const RingBuffer* pRingBuff);
int8_t rbIsEmpty(const RingBuffer* pRingBuff);
int8_t rbIsFull(const RingBuffer* pRingBuff);

#ifdef __cplusplus
}
#endif

#endif 

/************************ (C) COPYRIGHT AOYIWEIYENG *****END OF FILE***********/
