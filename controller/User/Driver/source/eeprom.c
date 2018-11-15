/*
功能：
将Flash用作EEPROM 用于保存偏置和标定数据
------------------------------------
*/

#include "eeprom.h"
#define  DEVICE_INFORMATION_ADRESS    (0x080C0000)      //将配置数据存放在第10个扇区
#define  PAGE_Config                  (0x080E0000)      //将配置数据存放在第11个扇区

struct data_map Config;
struct Device_Info Device_Information;


void EEPROM_Loading_Config( void )
{
    int16_t i;
    int16_t* ptr = &Config.Mag_is_good;
    int16_t* temp_addr = ( int16_t* )PAGE_Config;
    //  FLASH_Unlock();
    for( i = 0 ; i < sizeof( Config ) / 2; i++ )
    {
        *ptr = *temp_addr;
        temp_addr++;
        ptr++;
    }
    //  FLASH_Lock();
    if( Config.Mag_is_good != ( int16_t )0xA66A )
    {
        Config.Mag_is_good = 0x00;
        Config.dMx_offset = 0.0;
        Config.dMy_offset = 0.0;
        Config.dMz_offset = 0.0;
        Config.dMx_min = 0.0;
        Config.dMy_min = 0.0;
        Config.dMz_min = 0.0;
        Config.dMx_max = 0.0;
        Config.dMy_max = 0.0;
        Config.dMz_max = 0.0;
        Config.MAGXY_factor = 1.0;
        Config.MAGXZ_factor = 1.0;
        EEPROM_Write_Config();
    }
    //      if(Config.Acc_is_good != (int16_t)0xA66A)
    //       {
    //        Config.Acc_is_good = 0x00;
    //
    //        Config.dAx_offset = 0.0;
    //  Config.dAy_offset = 0.0;
    //  Config.dAz_offset = 0.0;
    //        EEPROM_Write_Config();
    //        }
}


void EEPROM_Write_Config( void )
{
    FLASH_Status temp_status;
    int16_t i;
    int16_t* ptr = &Config.Mag_is_good;
    uint32_t ptemp_addr = PAGE_Config;
    FLASH_Unlock();
    FLASH_DataCacheCmd( DISABLE );
    FLASH_ClearFlag( FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR );
    if( FLASH_EraseSector( FLASH_Sector_11, VoltageRange_3 ) != FLASH_COMPLETE )
    {
        FLASH_Lock();
    }
    for( i = 0; i < sizeof( Config ) / 2; i++ )
    {
        temp_status = FLASH_ProgramHalfWord( ptemp_addr + 2 * i, ptr[i] );
        if( temp_status != FLASH_COMPLETE )
        {
            FLASH_Lock();
        }
    }
    FLASH_DataCacheCmd( ENABLE );
    FLASH_Lock();
}


/*------------------------ 版本号/设备ID -------------------------------------*/
extern uint8_t  m_Log_Data_Buff[35];

void EEPROM_Write_DeviceInformation( void )
{
    FLASH_Status temp_status;
    int16_t i;
    uint8_t* ptr = Device_Information.Hardware_Version;
    uint32_t ptemp_addr = DEVICE_INFORMATION_ADRESS;
    EEPROM_Write_DeviceInformation_Init();
    FLASH_Unlock();
    FLASH_ClearFlag( FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR );
    if( FLASH_EraseSector( FLASH_Sector_10, VoltageRange_3 ) != FLASH_COMPLETE )
    {
        FLASH_Lock();
    }
    for( i = 0; i < sizeof( Device_Information ); i++ )
    {
        temp_status = FLASH_ProgramByte( ptemp_addr + i, ptr[i] );
        if( temp_status != FLASH_COMPLETE )
        {
            FLASH_Lock();
        }
    }
    FLASH_Lock();
}

void EEPROM_Loading_DeviceInformation( void )
{
    int16_t i;
    uint8_t* ptr = Device_Information.Hardware_Version;
    uint8_t* temp_addr = ( uint8_t* )DEVICE_INFORMATION_ADRESS;
    //  FLASH_Unlock();
    for( i = 0 ; i < sizeof( Device_Information ); i++ )
    {
        *ptr = *temp_addr;
        temp_addr++;
        ptr++;
    }
    //  FLASH_Lock();
    if( Device_Information.Hardware_Version[0] != 'T'
            || Device_Information.Hardware_Version[1] != '3'
            || Device_Information.Software_Version[0] != 'W'
            || Device_Information.Software_Version[1] != '3'
            || Device_Information.Device_ID[0]        != 'S'
            || Device_Information.Device_ID[1]        != '0'
            || Device_Information.Device_ID[3]        != '0'
            || Device_Information.Device_ID[4]        != '1' )
    {
        EEPROM_Write_DeviceInformation();
    }
}

extern  char ID_buffer[2];
void EEPROM_Write_DeviceInformation_Init( void )
{
    Device_Information.Hardware_Version[0]        = 'T';
    Device_Information.Hardware_Version[1]        = '3';
    Device_Information.Hardware_Version[2]        = '.';
    Device_Information.Hardware_Version[3]        = '0';
    Device_Information.Hardware_Version[4]        = '1';
    Device_Information.Software_Version[0]        = 'W';
    Device_Information.Software_Version[1]        = '3';
    Device_Information.Software_Version[2]        = '.';
    Device_Information.Software_Version[3]        = '2';
    Device_Information.Software_Version[4]        = '0';
    Device_Information.Device_ID[0]               = 'S';
    Device_Information.Device_ID[1]               = '0';
    Device_Information.Device_ID[2]               = '0';
    Device_Information.Device_ID[3]               = '0';
    Device_Information.Device_ID[4]               = '0';
    /*----------------------------------用户自定义 -------------------------------*/
    Device_Information.User_ID[0]                 = 'T';
    Device_Information.User_ID[1]                 = 'W';
    Device_Information.User_ID[2]                 = 'S';
    Device_Information.User_ID[3]                 = ID_buffer[0];
    Device_Information.User_ID[4]                 = ID_buffer[1];
}

uint16_t GetBytes_DeviceInformation( uint8_t* ptr )
{
    static uint8_t Offset;
    uint16_t Bytes_DeviceInformation;
    Bytes_DeviceInformation = ( Offset << 8 ) | ( *( ptr + Offset ) );
    Offset++;
    if( Offset >= 20 )
    {
        Offset = 0;
    }
    return Bytes_DeviceInformation;
}



/*----------------------------------End of File-------------------------------*/
