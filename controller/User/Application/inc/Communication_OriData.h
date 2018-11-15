#ifndef __COMMUNICATION_ORIDATA_H
#define __COMMUNICATION_ORIDATA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"
#include "main.h"
#include "stdio.h"
#include "structural.h"
#include "stm32f4xx_it.h"
#include "uart_init.h"


void Communication_OriData_Configure();
void OriData__Communication_Send();
uint8_t Choose_Package_OriData(u8* txbuf);

#ifdef __cplusplus
}
#endif
#endif