#include "ICM2060x.h"
#include "filter.h"


#if ICM20602
const struct sensor_reg_s reg =
{
    .who_am_i       = 0x75,
    .rate_div       = 0x19,
    .lpf            = 0x1A,
    //    .prod_id        = 0x0C,
    .xg_offs_usr    = 0x13,
    .yg_offs_usr    = 0x15,
    .zg_offs_usr    = 0x17,
    .user_ctrl      = 0x6A,
    .fifo_en        = 0x23,
    .gyro_cfg       = 0x1B,
    .accel_cfg      = 0x1C,
    .accel_cfg2     = 0x1D,
    .lp_mode_cfg    = 0x1E,
    .motion_thr     = 0x1F,
    //    .motion_dur     = 0x20,
    .fifo_count_h   = 0x72,
    .fifo_r_w       = 0x74,
    .raw_gyro       = 0x43,
    .raw_accel      = 0x3B,
    .temp           = 0x41,
    .int_enable     = 0x38,
    //    .dmp_int_status = 0x39,
    .int_status     = 0x3A,
    .accel_intel    = 0x69,
    .pwr_mgmt_1     = 0x6B,
    .pwr_mgmt_2     = 0x6C,
    .int_pin_cfg    = 0x37,
    //    .mem_r_w        = 0x6F,
    .xa_offset      = 0x77,
    .ya_offset      = 0x7A,
    .za_offset      = 0x7D,
    //    .i2c_mst        = 0x24,
    //    .bank_sel       = 0x6D,
    //    .mem_start_addr = 0x6E,
    //    .prgm_start_h   = 0x70,
    //    .fifo_wm_th     = 0x61,
    .signal_reset   = 0x68,
    //    .st_gyro        = 0x00,
    .st_accel       = 0x0D,
};
#endif
//#define pi 3.1415926

extern d_Senser_Data*  pSenser_Data;

uint8_t S0_write_reg_succeed = 1;

static void ICM2060x_CHIP_Init( void );
static void ICM2060x_ACC_Config( void );
static void ICM2060x_GYRO_Config( void );

void ICM2060x_Init( void )
{
    I2C_Init_IO();
    write_1B( reg.user_ctrl, 0x01 ); //传感器寄存器初始化Reset
    if( S0_write_reg_succeed == 0 )
    {
        return;
    }
#if ICM20602
    DelayMS( 10 );
    ICM2060x_CHIP_Init();//Device Reset
    ICM2060x_ACC_Config();
    ICM2060x_GYRO_Config();
    pSenser_Data->ICM_flag = 1;
    pSenser_Data->Senser_Mode = 1;
#endif
}

static void ICM2060x_CHIP_Init( void )
{
    write_1B( reg.pwr_mgmt_1, 0x80 ); //pwr_mgmt_1-0x6B Device Reset
    DelayMS( 100 );
    write_1B( reg.pwr_mgmt_1, 0x01 ); //auto Clk
    DelayMS( 100 );
}


static void ICM2060x_ACC_Config( void )
{
    uint8_t ch;
    /* Enable Acc Bit5-3 */
    read_1B( reg.pwr_mgmt_2, &ch );
    //ch &= 0xC7;
    ch &= 0x07;
    write_1B( reg.pwr_mgmt_2, ch );
    /* Config Acc*/
    //    //量程，16g
    //    write_1B(reg.accel_cfg,0x18);
    //量程，2g
    write_1B( reg.accel_cfg, 0x00 );
    /* 内部滤波带宽 */
    //    write_1B(reg.accel_cfg2,0x02);//99Hz
    write_1B( reg.accel_cfg2, 0x04 ); //21.2Hz
    //ODR，500Hz
    write_1B( reg.rate_div, 0x01 );
}

static void ICM2060x_GYRO_Config( void )
{
    uint8_t ch;
    /* Enable Gyro */
    read_1B( reg.pwr_mgmt_2, &ch );
    ch &= 0x38;
    write_1B( reg.pwr_mgmt_2, ch );
    DelayMS( 80 );
    /* Config Gyro */
    //量程，500dps
    write_1B( reg.gyro_cfg, 0x08 );
    //滤波器截止频率和噪声带宽
    //    write_1B(reg.lpf,0x02);//lpf-0x1A 92Hz
    write_1B( reg.lpf, 0x04 ); //lpf-0x1A 20Hz
    /*ODR,500Hz*/
    write_1B( reg.rate_div, 0x01 );
}


extern struct data_map Config;


float acc1[3];
uint16_t errcount = 0 ;
void ICM2060x_GetACC( void )
{
    uint8_t buf[6];
    io_haldware_reg_read( reg.raw_accel, buf, 6 );
    /*------------------------- AD 转化原始值 ----------------------------------*/
    pSenser_Data->ICMAX = ( ( ( ( int16_t )( ( ( int16_t )buf[0] ) << 8 | buf[1] ) ) * 9.7949 / 8192.0 / 2.0 - ACC_ICMX_OFFSET ) * K_ACC_ICMX );
    pSenser_Data->ICMAY = ( ( ( ( int16_t )( ( ( int16_t )buf[2] ) << 8 | buf[3] ) ) * 9.7949 / 8192.0 / 2.0 - ACC_ICMY_OFFSET ) * K_ACC_ICMY );
    pSenser_Data->ICMAZ = -( ( ( ( int16_t )( ( ( int16_t )buf[4] ) << 8 | buf[5] ) ) * 9.7949 / 8192.0 / 2.0 - ACC_ICMZ_OFFSET ) * K_ACC_ICMZ );
    ICMdata_filter_Acc();
}


void ICM2060x_GetGYRO( void )
{
    uint8_t buf[6];
    io_haldware_reg_read( reg.raw_gyro, buf, 6 );
    pSenser_Data->ICMGY = -( ( int16_t )( ( ( int16_t )buf[0] ) << 8 | buf[1] ) ) * pi / 180.0 / 65.5;
    pSenser_Data->ICMGX = -( ( int16_t )( ( ( int16_t )buf[2] ) << 8 | buf[3] ) ) * pi / 180.0 / 65.5;
    pSenser_Data->ICMGZ = ( ( int16_t )( ( ( int16_t )buf[4] ) << 8 | buf[5] ) ) * pi / 180.0 / 65.5;
    ICMdata_filter_Gyro();
}

