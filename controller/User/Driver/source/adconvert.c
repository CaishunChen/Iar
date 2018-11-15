#include "adconvert.h"
#define ADC1_DR_Address    ((uint32_t)0x4001204C)  //ADC数据保存
__IO uint16_t ADCConvertedValue1[1];


d_Senser_Data Senser_Data;
d_Senser_Data *pSenser_Data = &Senser_Data;

//void ADC_DMA_init(void)
//{
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);
//
//  GPIO_InitTypeDef  GPIO_InitStructure;
//  
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 ;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
//  GPIO_Init(GPIOB, &GPIO_InitStructure);
//  
//  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4| GPIO_Pin_5;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
//  GPIO_Init(GPIOC, &GPIO_InitStructure);
//    
//  /*------------------------------加入DMA-------------------------------------*/
//   DMA_InitTypeDef DMA_InitStructure;
//  DMA_DeInit(DMA2_Stream0);
//  //DMA_DeInit(DMA2_Stream4);
//  DMA_InitStructure.DMA_Channel = DMA_Channel_0;
//  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
//  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADCConvertedValue1;
//  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
//  DMA_InitStructure.DMA_BufferSize = 1;
//  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
//  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
//  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
//  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
//  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
//  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
//  DMA_Init(DMA2_Stream0,&DMA_InitStructure);
//  DMA_Cmd(DMA2_Stream0,ENABLE);
//  
//  ADC_InitTypeDef ADC_InitStructure;
//  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
//  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
//  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
//  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
//  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
//  ADC_InitStructure.ADC_NbrOfConversion = 1;
//  ADC_Init(ADC1, &ADC_InitStructure);
//  
//  ADC_CommonInitTypeDef ADC_CommonInitStructure;
//  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
//  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
//  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
//  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
//  ADC_CommonInit(&ADC_CommonInitStructure);
//  
//  ADC_RegularChannelConfig(ADC1, ADC_Channel_8 ,1, ADC_SampleTime_144Cycles);//ax1
////  ADC_RegularChannelConfig(ADC1, ADC_Channel_14 ,2, ADC_SampleTime_144Cycles);//ay1
////  ADC_RegularChannelConfig(ADC1, ADC_Channel_15 ,3, ADC_SampleTime_144Cycles);//ay2  
//
//  ADC_ContinuousModeCmd(ADC1,ENABLE);
//  ADC_DMACmd(ADC1, ENABLE);
//  ADC_DMARequestAfterLastTransferCmd(ADC1,ENABLE);
//  ADC_Cmd(ADC1,ENABLE);
//  ADC_SoftwareStartConv(ADC1);    
//}

void AD_To_RawAhrs()
{
  pSenser_Data->ADIax= ADCConvertedValue1[0];
//  pSenser_Data->ADIay= ADCConvertedValue1[1];
//  pSenser_Data->ADIaz= ADCConvertedValue1[2];
 }
