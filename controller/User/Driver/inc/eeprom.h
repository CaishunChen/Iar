#ifndef __EEPROM_H
#define __EEPROM_H

#include "stm32f4xx.h"
#include "stm32f4xx_flash.h"
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbytes */
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) /* Base @ of Sector 8, 128 Kbytes */
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) /* Base @ of Sector 9, 128 Kbytes */
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) /* Base @ of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) /* Base @ of Sector 11, 128 Kbytes */
struct data_map{
  
  int16_t Mag_is_good;   //数据是否有效
  int16_t Acc_is_good;
  float dAx_offset;
  float dAy_offset;
  float dAz_offset;
  
  float dMx_offset;
  float dMy_offset;
  float dMz_offset;
  
  float dMx_min;
  float dMy_min;
  float dMz_min;
  
  float dMx_max;
  float dMy_max;
  float dMz_max;
  float MAGXY_factor;
  float MAGXZ_factor;	
};

struct Device_Info{
  uint8_t Hardware_Version[5];
  uint8_t Software_Version[5];
  uint8_t Device_ID[5];
  uint8_t User_ID[5];
};

extern struct data_map Config;
extern struct Device_Info Device_Information;
extern struct Device_Info Device_MAG_Information;
void EEPROM_Loading_Config(void);
void EEPROM_Write_Config(void);
void EEPROM_Write_DeviceInformation(void);
void EEPROM_Loading_DeviceInformation(void);
void EEPROM_Write_DeviceInformation_Init(void);
void EEPROM_Write_MAG_DeviceInformation_Init(void);
uint16_t GetBytes_DeviceInformation(uint8_t *ptr);
uint16_t GetBytes_MAG_DeviceInformation(uint8_t *ptr);

#endif /* __EEPROM_H */

//------------------End of File----------------------------
