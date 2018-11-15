#ifndef __ICM2060X_H__
#define __ICM2060X_H__

#include "Master_i2c.h"
//#include "ICM2060x_RegDef.h"
#include "SysTick.h"
#include "structural.h" 
//#include "BNO055.h"

#define DelayMS delay_ms 
#define MPUREG_WHO_AM_I 0x75

#define ICM20602 1

/*------------------------------   1  ----------------------------------------*/
#define ACC_ICMX_OFFSET  (0.02)
#define ACC_ICMY_OFFSET  (-0.04)
#define ACC_ICMZ_OFFSET  (0.25)

#define K_ACC_ICMX     (1.0)
#define K_ACC_ICMY     (1.0)
#define K_ACC_ICMZ     (1.0)

/*------------------------------   2  ----------------------------------------*/
//#define ACC_ICMX_OFFSET  (0.02)
//#define ACC_ICMY_OFFSET  (-0.2)
//#define ACC_ICMZ_OFFSET  (-0.02)
//
//#define K_ACC_ICMX     (1.0)
//#define K_ACC_ICMY     (1.0)
//#define K_ACC_ICMZ     (1.0)


#if ICM20602
/* Hardware registers needed by driver. */
struct sensor_reg_s {
    unsigned char who_am_i;
    unsigned char rate_div;
    unsigned char lpf;
//    unsigned char prod_id; 
    unsigned char xg_offs_usr;
    unsigned char yg_offs_usr;
    unsigned char zg_offs_usr;
    unsigned char user_ctrl;
    unsigned char fifo_en;
    unsigned char gyro_cfg;
    unsigned char accel_cfg;
    unsigned char accel_cfg2;
    unsigned char lp_mode_cfg;
    unsigned char motion_thr;
//    unsigned char motion_dur;
    unsigned char fifo_count_h;
    unsigned char fifo_r_w;
    unsigned char raw_gyro;
    unsigned char raw_accel;
    unsigned char temp;
    unsigned char int_enable;
//    unsigned char dmp_int_status;
    unsigned char int_status;
    unsigned char accel_intel;
    unsigned char pwr_mgmt_1;
    unsigned char pwr_mgmt_2;
    unsigned char int_pin_cfg;
//    unsigned char mem_r_w;
    unsigned char xa_offset;
    unsigned char ya_offset;
    unsigned char za_offset;
//    unsigned char i2c_mst;
//    unsigned char bank_sel;
//    unsigned char mem_start_addr;
//    unsigned char prgm_start_h;
//    unsigned char fifo_wm_th;
    unsigned char signal_reset;
//    unsigned char st_gyro;
    unsigned char st_accel;
};
#endif


void ICM2060x_Init(void);
void ICM2060x_CHIP_INIT(void);
void ICM2060x_CHIP_POWERON(void);
void ICM2060x_CHIP_POWEROFF(void);
void ICM2060x_ACC_POWERON(void);
void ICM2060x_ACC_POWEROFF(void);
void ICM2060x_GYRO_POWERON(void);
void ICM2060x_GYRO_POWEROFF(void);
void ICM2060x_ACC_ENABLE(void);
void ICM2060x_GYRO_ENABLE(void);
void ICM2060x_SET_ACC_GYRO_ODR(uint32_t HZ);
void ICM2060x_GetACC(void);
void ICM2060x_GetGYRO(void);


void Gyroad_Initial_Offset(void);
////////////////////////////////////////////////////////////////////////
//                 FIFO Setting                                     ////
////////////////////////////////////////////////////////////////////////
void ICM2060x_RESET_FIFO(void);
void ICM2060x_ACC_FIFOMODE_ENABLE(void);
void ICM2060x_ACC_FIFOMODE_DISABLE(void);
void ICM2060x_GYRO_FIFOMODE_ENABLE(void);
void ICM2060x_GYRO_FIFOMODE_DISABLE(void);

////////////////////////////////////////////////////////////////////////
//                Lowpower Motion Detect                            ////
////////////////////////////////////////////////////////////////////////

#endif