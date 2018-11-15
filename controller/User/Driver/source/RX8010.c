#include "RX8010.h"
#include "SysTick.h"
#include "Communication.h"

d_RTC_Time RTC_Time;
d_RTC_Time* pRTC_Time = &RTC_Time;

extern d_Gps_Data*  pGps_Data;


void RS8010sj_Init()
{
    RS8010sj_Init_IO();
    delay_ms( 100 );
    RX8010_Init();
    //
    Read_RTC_Time();//读取时间，文件写入时间
}

void RS8010sj_Init_IO()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOE, ENABLE );
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;//scl
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//无上下拉
    GPIO_Init( GPIOE, &GPIO_InitStructure );
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//sda
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_Init( GPIOE, &GPIO_InitStructure );
    GPIO_SetBits( GPIOE, GPIO_Pin_3 );
    GPIO_SetBits( GPIOE, GPIO_Pin_4 );
}

u8 Time_Display_Buff[7];/*按照秒。分钟，小时，天，月，年，周存储*/
u8 Read_Years, Read_Mon, Read_Day, Read_Hour, Read_Min, Read_Sec, Read_Week;
u8 Set_Years, Set_Mon, Set_Day, Set_Hour, Set_Min, Set_Sec, Set_Week;
float Hdc_Tim, Hdc_Hum;
float Tim_Tim_Dat, Tim_Hum_Dat;
u16  Hdc_Tim_Display, Hdc_Hum_Display;
extern u8 Diaplay[4];


char BCDToHex( char dat )
{
    char cReturn = 0;
    if( ( dat & 0x40 ) >> 6 == 1 )
    {
        cReturn += 40;
    }
    if( ( dat & 0x20 ) >> 5 == 1 )
    {
        cReturn += 20;
    }
    if( ( dat & 0x10 ) >> 4 == 1 )
    {
        cReturn += 10;
    }
    cReturn += dat % 16;
    return cReturn;
}



unsigned int HEX2BCD( unsigned char hex_data )
{
    unsigned int bcd_data;
    unsigned char temp;
    temp = hex_data % 100;
    bcd_data = ( ( unsigned int )hex_data ) / 100 << 8;
    bcd_data = bcd_data | temp / 10 << 4;
    bcd_data = bcd_data | temp % 10;
    return bcd_data;
}



void IIC_Start( void )
{
    TIME_SDA_OUT();// SDA_OUT();     //sda???
    TIME_SDA_ON;//RTC_SDA=1;
    TIME_SCL_ON;//RTC_SCL=1;
    delay_us( IIC_Dlay );
    TIME_SDA_OFF;//   RTC_SDA=0;//START:when CLK is high,DATA change form high to low
    delay_us( IIC_Dlay );
    TIME_SCL_OFF;//RTC_SCL=0;//??I2C??,?????????
}


void IIC_Stop( void )
{
    TIME_SDA_OUT();// SDA_OUT();//sda???
    TIME_SCL_OFF;
    delay_us( IIC_Dlay );
    TIME_SDA_OFF;//  RTC_SDA=0;//STOP:when CLK is high DATA change form low to high
    delay_us( IIC_Dlay );
    TIME_SCL_ON;
    delay_us( IIC_Dlay );
    TIME_SDA_ON;//RTC_SDA=1;//??I2C??????
    delay_us( IIC_Dlay );
}

u8 IIC_Wait_Ack( void )
{
    u8 ucErrTime = 0;
    TIME_SDA_IN();//SDA_IN();      //SDA?????
    TIME_SDA_ON;//    RTC_SCL=1;
    delay_us( IIC_Dlay );
    TIME_SCL_ON;//    RTC_SCL=1;
    delay_us( IIC_Dlay );
    if( I2C_SDA_READ() )
    {
        ucErrTime = 1;
    }
    else
    {
        ucErrTime = 0;
    }
    TIME_SCL_OFF;//   RTC_SCL=0;//????0
    delay_us( IIC_Dlay );
    return ucErrTime;
}

void IIC_Ack( void )
{
    TIME_SCL_OFF;
    TIME_SDA_OUT();//SDA_OUT();
    TIME_SDA_OFF;//RTC_SDA=0;
    delay_us( IIC_Dlay );
    TIME_SCL_ON;//RTC_SCL=1;
    delay_us( IIC_Dlay );
    TIME_SCL_OFF;//RTC_SCL=0;
    TIME_SDA_ON;
}
void IIC_NAck( void )
{
    //  TIME_SCL_OFF;
    TIME_SDA_OUT();// SDA_OUT();
    TIME_SDA_ON;//RTC_SDA=1;
    delay_us( IIC_Dlay );
    TIME_SCL_ON;//RTC_SCL=1;
    delay_us( IIC_Dlay );
    TIME_SCL_OFF;//RTC_SCL=0;
}


void IIC_SendByte( u8 txd )
{
    u8 t;
    TIME_SDA_OUT();// SDA_OUT();
    // TIME_SCL_OFF;
    for( t = 0; t < 8; t++ )
    {
        if( txd & 0x80 ) //>>7==1)
        {
            TIME_SDA_ON;
        }
        else
        {
            TIME_SDA_OFF;
        }
        txd <<= 1;
        delay_us( IIC_Dlay );
        TIME_SCL_ON;//RTC_SCL=1;
        delay_us( IIC_Dlay );
//        if( t == 7 )
//        {
//            TIME_SDA_ON;
//        }
    }
    TIME_SCL_OFF;//RTC_SCL=1;

}

u8 IIC_ReadByte( void )
{
    unsigned char i, receive = 0;
    TIME_SDA_IN();//SDA_IN();//SDA?????
    //TIME_SDA_ON;
    //TIME_SCL_OFF;
    for( i = 0; i < 8; i++ )
    {
        TIME_SCL_OFF;// RTC_SCL=1;
        delay_us(IIC_Dlay);
        TIME_SCL_ON;//  RTC_SCL=1;
        delay_us( IIC_Dlay );
        receive <<= 1;
        if( I2C_SDA_READ() )
        {
            receive++;
        }

    }
    TIME_SCL_OFF;
    delay_us( IIC_Dlay );
    return receive;
}


u8 Read_Constant_Time( u8 Idx )
{
    u8 Time;
    IIC_Start();
    IIC_SendByte( 0x64 );
    if( IIC_Wait_Ack() )
    {
        IIC_Stop();
        return 0;
    }
    IIC_SendByte( Idx );
    if( IIC_Wait_Ack() )
    {
        IIC_Stop();
        return 0;
    }
    IIC_Start();
    IIC_SendByte( 0x65 );
    if( IIC_Wait_Ack() )
    {
        IIC_Stop();
        return 0;
    }
    Time = BCDToHex( IIC_ReadByte() );
    //IIC_Ack();
    //IIC_NAck();
    IIC_Stop();
    return Time;
}


u8 Write_Constant_Time( u8 Idx, u8 Dat )
{
    IIC_Start();
    IIC_SendByte( 0x64 );
    if( IIC_Wait_Ack() )
    {
        IIC_Stop();
        return 0;
    }
    IIC_SendByte( Idx );
    if( IIC_Wait_Ack() )
    {
        IIC_Stop();
        return 0;
    }
    IIC_SendByte( HEX2BCD( Dat ) );
    if( IIC_Wait_Ack() )
    {
        IIC_Stop();
        return 0;
    }
    IIC_Stop();
    return 1;
}



//uint8_t temp_buf[7] = {0};
void RX8010_Init( void )
{
    pRTC_Time->Vel_flag = Read_Constant_Time( 0x1E );
    if( !( pRTC_Time->Vel_flag & 0x02 ) )
    {
        return;    //只有VLF位(bit1)为1时才需要初始化，否则直接退出
    }
    delay_ms( 100 );
    Write_Constant_Time( 0x17, 0xD8 );
    delay_ms( 10 ) ;
    // temp_buf[1] = Read_Constant_Time(0x17);//RX8010_ADDR_RSV17必须初始化为0xD8
    // delay_ms(10) ;
    Write_Constant_Time( 0x30, 0x00 );
    delay_ms( 10 ) ;
    //  temp_buf[2] = Read_Constant_Time(0x30);//RX8010_ADDR_RSV30必须初始化为0x00
    // delay_ms(10) ;
    Write_Constant_Time( 0x31, 0x08 );
    delay_ms( 10 ) ;
    //  temp_buf[3] = Read_Constant_Time(0x31);//RX8010_ADDR_RSV31必须初始化为0x18
    // delay_ms(10) ;
    Write_Constant_Time( 0x32, 0x00 );
    delay_ms( 10 ) ;
    //  temp_buf[4] = Read_Constant_Time(0x32);//RX8010_ADDR_IRQ_CTRL中关闭IRQ输出以降低功耗
    // delay_ms(10) ;
    Write_Constant_Time( 0x1D, 0x04 );
    delay_ms( 10 ) ;
    //  temp_buf[4] = Read_Constant_Time(0x1D);//RX8010_ADDR_EXT_REG中，关闭FOUT/固定周期定时等功能
    // delay_ms(10) ;
    Write_Constant_Time( 0x1F, 0x00 );
    delay_ms( 10 ) ;
    // temp_buf[6] = Read_Constant_Time(0x1F);//RX8010_ADDR_CTRL_REG中，禁止TIME UPDATE中断/闹钟中断/固定周期定时中断
    // delay_ms(10) ;
    /*----------------------------  设置时间  -----------------------------------*/
    Write_Constant_Time( 0x10, 0x00 ); //sec
    delay_ms( 10 ) ;
    Write_Constant_Time( 0x11, 0x00 ); //min
    delay_ms( 10 ) ;
    Write_Constant_Time( 0x12, 0x17 ); //hour
    delay_ms( 10 ) ;
    Write_Constant_Time( 0x13, 0x02 ); //week
    delay_ms( 10 ) ;
    Write_Constant_Time( 0x14, 0x1F ); //day
    delay_ms( 10 ) ;
    Write_Constant_Time( 0x15, 0x08 ); //month
    delay_ms( 10 ) ;
    Write_Constant_Time( 0x16, 0x12 ); //year
    delay_ms( 10 ) ;
    Write_Constant_Time( 0x1E, 0x00 );
    delay_ms( 10 ) ;
    //  temp_buf[5] = Read_Constant_Time(0x1E);//只有VLF位(bit1)为1时才需要初始化，否则直接退出
    // delay_ms(10) ;
}


void Read_RTC_Time( void )
{
    pRTC_Time->sec  = Read_Constant_Time( 0x10 ); //sec
    pRTC_Time->min  = Read_Constant_Time( 0x11 ); //min
    pRTC_Time->hour = Read_Constant_Time( 0x12 ); //hour
    pRTC_Time->week = Read_Constant_Time( 0x13 ); //week
    pRTC_Time->day  = Read_Constant_Time( 0x14 ); //day
    pRTC_Time->mon  = Read_Constant_Time( 0x15 ); //mon 8
    pRTC_Time->year = Read_Constant_Time( 0x16 ); //year 18
}

void Time_Read_Run()
{
    pRTC_Time->sec  = Read_Constant_Time( 0x10 ); //sec
    pRTC_Time->min  = Read_Constant_Time( 0x11 ); //min
    if( pRTC_Time->min == 0 )
    {
        pRTC_Time->hour = Read_Constant_Time( 0x12 ); //hour
        if( pRTC_Time->hour == 0 )
        {
            pRTC_Time->week = Read_Constant_Time( 0x13 ); //week
            pRTC_Time->day  = Read_Constant_Time( 0x14 ); //day
            pRTC_Time->mon  = Read_Constant_Time( 0x15 ); //mon
            pRTC_Time->year = Read_Constant_Time( 0x16 ); //year
        }
    }
}

void RTC_Time_Adj()//时间校准  次步操作后 需对系统进行重启
{
    if( pHost_Cmd->Time_Adj_Mode == 2 ) //GPS时间
    {
        Write_Constant_Time( 0x10, ( pGps_Data->Beijing_Second ) ); //sec
        delay_ms( 10 ) ;
        Write_Constant_Time( 0x11, pGps_Data->Beijing_Minute ); //min
        delay_ms( 10 ) ;
        Write_Constant_Time( 0x12, pGps_Data->Beijing_Hour ); //hour
        delay_ms( 10 ) ;
        Write_Constant_Time( 0x14, pGps_Data->Beijing_Day ); //day
        delay_ms( 10 ) ;
        Write_Constant_Time( 0x15, pGps_Data->Beijing_Month ); //month
        delay_ms( 10 ) ;
        Write_Constant_Time( 0x16, pGps_Data->Beijing_Year - 2000 ); //year
        delay_ms( 10 ) ;
    }
    else if( pHost_Cmd->Time_Adj_Mode == 1 ) //电脑时间
    {
        Write_Constant_Time( 0x10, ( pRTC_Time->Local_Second ) ); //sec
        delay_ms( 10 ) ;
        Write_Constant_Time( 0x11, pRTC_Time->Local_Minute ); //min
        delay_ms( 10 ) ;
        Write_Constant_Time( 0x12, pRTC_Time->Local_Hour ); //hour
        delay_ms( 10 ) ;
        Write_Constant_Time( 0x14, pRTC_Time->Local_Day ); //day
        delay_ms( 10 ) ;
        Write_Constant_Time( 0x15, pRTC_Time->Local_Month ); //month
        delay_ms( 10 ) ;
        Write_Constant_Time( 0x16, pRTC_Time->Local_Year ); //year
        delay_ms( 10 ) ;
    }
    pHost_Cmd->Time_Adj_Mode = 0;
}