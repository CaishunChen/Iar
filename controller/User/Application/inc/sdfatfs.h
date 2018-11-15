
#ifndef __ISO_SDCARD_FATFS_H
#define __ISO_SDCARD_FATFS_H

#include "stm32f4xx.h"
#include "ff.h"
#include "diskio.h"		/* FatFs lower layer API */
#include <string.h>
#include "filter.h" 

#define MIN_FREE_SPACE_KB (51200)//50M

extern FATFS sdfs;
extern FIL file_rcd;
extern FIL file_loc;
extern FIL file_reccnt;

FRESULT Add_DateFile();
FRESULT Add_LocalFile();
FRESULT Read_CfgFile();
void Get_Config_Data();

void Add_Ending_Message();

extern char rcdfname[10];
//void ISO_SDCardFatfsReadWriteFile(void);
FRESULT ISO_GetSDCardfree(uint8_t *drv,uint32_t *total,uint32_t *free);
FRESULT set_timestamp (char *obj,int year,int month,int mday,int hour,int min,int sec);/* Pointer to the file name */
FRESULT AddNewFile(FATFS *fsp,FIL *fp);
unsigned int Asc_To_Num(char *str);
void Record_Data(FIL* f);

void Record_Log_Data(FIL* f);
FRESULT Creat_Log_File();

FRESULT Creat_New_RcdFile();
FRESULT ISO_GetSDCardfree(uint8_t *drv,uint32_t *total,uint32_t *free);

uint8_t strstr_Length(char *str, char *string);
uint8_t strchr_Length(char *str, char chr);
void Parmeters_Decode(char *buf,uint8_t parmeters_i);
void Parmeters_DecodeID(char *buf,uint8_t parmeters_j,uint8_t parmeters_i);

#endif


