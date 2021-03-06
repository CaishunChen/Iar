#include "sdio_sdcard.h"
#include "string.h"
#include "sys.h"
#include "SysTick.h"
//////////////////////////////////////////////////////////////////////////////////


/*܇?܇�~sdio3?��??����??��11��?*/
SDIO_InitTypeDef SDIO_InitStructure;
SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
SDIO_DataInitTypeDef SDIO_DataInitStructure;

SD_Error CmdError( void );
SD_Error CmdResp7Error( void );
SD_Error CmdResp1Error( u8 cmd );
SD_Error CmdResp3Error( void );
SD_Error CmdResp2Error( void );
SD_Error CmdResp6Error( u8 cmd, u16* prca );
SD_Error SDEnWideBus( u8 enx );
SD_Error IsCardProgramming( u8* pstatus );
SD_Error FindSCR( u16 rca, u32* pscr );
u8 convert_from_bytes_to_power_of_two( u16 NumberOfBytes );


static u8 CardType = SDIO_STD_CAPACITY_SD_CARD_V1_1;    //SD?���ǽ�D��S��??��??a1.x?���S?
static u32 CSD_Tab[4], CID_Tab[4], RCA = 0;             //SD?��CSD,CID��??��?��??��??�{(RCA)��y?Y
static u8 DeviceMode = SD_DMA_MODE;                     //1�N�G��?�S��?,�G�S��a,1�N�G��?�S��?�I?D?�ࡰ1ySD_SetDeviceMode,o܇2???��y.?a��???��??����?��?????��?��??�S��?(SD_DMA_MODE)
static u8 StopCondition = 0;                            //��?�{?�{�S?���S?1��?��?�I��????,DMA?��?��?��D����?�ǩIo��܇?��?
volatile SD_Error TransferError = SD_OK;                //��y?Y��?��?�ɳ�?܇�I��??,DMA?��D���ǩI��1܇?
volatile u8 TransferEnd = 0;                            //��?��??����?�I��??,DMA?��D���ǩI��1܇?
SD_CardInfo SDCardInfo;                                 //SD?��D??�S

//SD_ReadDisk/SD_WriteDisko����y�G��܇?buf,�שI?a��???o����y��?��y?Y?o��???��??�{2?��?4�G??�~????��?�ǩIo��,
//D����a܇?��???��y�G��,���{�I�S��y?Y?o��???��??�{��?4�G??�~????��?.
#pragma pack(push,4)
__no_init u8 SDIO_DATA_BUFFER[512];


void SDIO_Register_Deinit()
{
    SDIO->POWER = 0x00000000;
    SDIO->CLKCR = 0x00000000;
    SDIO->ARG = 0x00000000;
    SDIO->CMD = 0x00000000;
    SDIO->DTIMER = 0x00000000;
    SDIO->DLEN = 0x00000000;
    SDIO->DCTRL = 0x00000000;
    SDIO->ICR = 0x00C007FF;
    SDIO->MASK = 0x00000000;
}

//3?��??��SD?��
//�{��???��:�ɳ�?܇���~??;(0,?T�ɳ�?܇)
SD_Error SD_Init( void )
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    SD_Error errorstatus = SD_OK;
    u8 clkdiv = 0;
    RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_DMA2, ENABLE ); //��1?��GPIOC,GPIOD DMA2�ǩI?܇
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_SDIO, ENABLE ); //SDIO�ǩI?܇��1?��
    RCC_APB2PeriphResetCmd( RCC_APB2Periph_SDIO, ENABLE ); //SDIO?��??
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12; //PC8,9,10,11,12?��܇?1|?����?3?
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//?��܇?1|?��
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//100M
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//��?��-
    GPIO_Init( GPIOC, &GPIO_InitStructure ); // PC8,9,10,11,12?��܇?1|?����?3?
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init( GPIOD, &GPIO_InitStructure ); //PD2?��܇?1|?����?3?
    //��y???��܇?܇3��?����??
    GPIO_PinAFConfig( GPIOC, GPIO_PinSource8, GPIO_AF_SDIO ); //PC8,AF12
    GPIO_PinAFConfig( GPIOC, GPIO_PinSource9, GPIO_AF_SDIO );
    GPIO_PinAFConfig( GPIOC, GPIO_PinSource10, GPIO_AF_SDIO );
    GPIO_PinAFConfig( GPIOC, GPIO_PinSource11, GPIO_AF_SDIO );
    GPIO_PinAFConfig( GPIOC, GPIO_PinSource12, GPIO_AF_SDIO );
    GPIO_PinAFConfig( GPIOD, GPIO_PinSource2, GPIO_AF_SDIO );
    RCC_APB2PeriphResetCmd( RCC_APB2Periph_SDIO, DISABLE ); //SDIO?����??��??
    //SDIO��a����??��??������???a??��??��
    SDIO_Register_Deinit();
    NVIC_InitStructure.NVIC_IRQChannel = SDIO_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //?��??܇??��??3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;    //�G܇܇??��??3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           //IRQ�ࡰ�׽���1?��
    NVIC_Init( &NVIC_InitStructure ); //?��?Y???����?2?��y3?��??��VIC??��??��?�S
    errorstatus = SD_PowerON();       //SD?����?��?
    if( errorstatus == SD_OK )
    {
        errorstatus = SD_InitializeCards();    //3?��??��SD?��
    }
    if( errorstatus == SD_OK )
    {
        errorstatus = SD_GetCardInfo( &SDCardInfo );    //??��??��D??�S
    }
    if( errorstatus == SD_OK )
    {
        errorstatus = SD_SelectDeselect( ( u32 )( SDCardInfo.RCA << 16 ) );    //???DSD?��
    }
    if( errorstatus == SD_OK )
    {
        errorstatus = SD_EnableWideBusOperation( SDIO_BusWide_4b );    //4???��?��,��?1?��?MMC?��,?��2??��܇?4???�S��?
    }
    if( ( errorstatus == SD_OK ) || ( SDIO_MULTIMEDIA_CARD == CardType ) )
    {
        if( SDCardInfo.CardType == SDIO_STD_CAPACITY_SD_CARD_V1_1 || SDCardInfo.CardType == SDIO_STD_CAPACITY_SD_CARD_V2_0 )
        {
            clkdiv = SDIO_TRANSFER_CLK_DIV + 2; //V1.1/V2.0?���S?����??�G???48/4=12Mhz
        }
        else
        {
            clkdiv = SDIO_TRANSFER_CLK_DIV;    //SDHC����?????���S?����??�G???48/2=24Mhz
        }
        SDIO_Clock_Set( clkdiv ); //����??�ǩI?܇?��?��,SDIO�ǩI?܇????1?��?:SDIO_CK�ǩI?܇=SDIOCLK/[clkdiv+2];???D,SDIOCLK1��?��?a48Mhz
        //errorstatus=SD_SetDeviceMode(SD_DMA_MODE);    //����???aDMA?�S��?
        errorstatus = SD_SetDeviceMode( SD_POLLING_MODE ); //����???a2��?��?�S��?
    }
    return errorstatus;
}
//SDIO�ǩI?܇3?��??������??
//clkdiv:�ǩI?܇�{??��?����y
//CK�ǩI?܇=SDIOCLK/[clkdiv+2];(SDIOCLK�ǩI?܇1��?��?a48Mhz)
void SDIO_Clock_Set( u8 clkdiv )
{
    u32 tmpreg = SDIO->CLKCR;
    tmpreg &= 0XFFFFFF00;
    tmpreg |= clkdiv;
    SDIO->CLKCR = tmpreg;
}


//?����?��?
//2��?��?��܇DSDIO?܇?�~��?��??�������I?,2�S2��?��??��??1o��????�ǩI?܇
//�{��???��:�ɳ�?܇���~??;(0,?T�ɳ�?܇)
SD_Error SD_PowerON( void )
{
    u8 i = 0;
    SD_Error errorstatus = SD_OK;
    u32 response = 0, count = 0, validvoltage = 0;
    u32 SDType = SD_STD_CAPACITY;
    /*3?��??���ǩI��?�ǩI?܇2??���܇܇�~400KHz*/
    SDIO_InitStructure.SDIO_ClockDiv = SDIO_INIT_CLK_DIV; /* HCLK = 72MHz, SDIOCLK = 72MHz, SDIO_CK = HCLK/(178 + 2) = 400 KHz */
    SDIO_InitStructure.SDIO_ClockEdge = SDIO_ClockEdge_Rising;
    SDIO_InitStructure.SDIO_ClockBypass = SDIO_ClockBypass_Disable;  //2?��1܇?bypass?�S��?�S??�I?܇܇?HCLK??DD�{??����?��?SDIO_CK
    SDIO_InitStructure.SDIO_ClockPowerSave = SDIO_ClockPowerSave_Disable; // ???D�ǩI2?1?�I?�ǩI?܇��??��
    SDIO_InitStructure.SDIO_BusWide = SDIO_BusWide_1b;                    //1??��y?Y??
    SDIO_InitStructure.SDIO_HardwareFlowControl = SDIO_HardwareFlowControl_Disable;//܇2?t�֡�
    SDIO_Init( &SDIO_InitStructure );
    SDIO_SetPowerState( SDIO_PowerState_ON ); //��?��?�G����?,?a???���ǩI?܇
    SDIO->CLKCR |= 1 << 8;        //SDIOCK��1?��
    for( i = 0; i < 74; i++ )
    {
        SDIO_CmdInitStructure.SDIO_Argument = 0x0;//�{�S?��CMD0??��?IDLE STAGE?�S��??����?.
        SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_GO_IDLE_STATE; //cmd0
        SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_No;  //?T?��܇|
        SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
        SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;  //?��CPSM?�~?a��?�{�S?��?����????��������y��y?Y��?��??����??�S
        SDIO_SendCommand( &SDIO_CmdInitStructure );         //D��?����????����???��??��
        errorstatus = CmdError();
        if( errorstatus == SD_OK )
        {
            break;
        }
    }
    if( errorstatus )
    {
        return errorstatus;    //�{��??�ɳ�?܇�G����?
    }
    SDIO_CmdInitStructure.SDIO_Argument = SD_CHECK_PATTERN;   //�{�S?��CMD8,?��?��܇|,?��2��SD?��?܇?�~��?D?
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_SEND_IF_COND;  //cmd8
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;     //r7
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;            //1?�I?������y?D??
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand( &SDIO_CmdInitStructure );
    errorstatus = CmdResp7Error();                    //������yR7?��܇|
    if( errorstatus == SD_OK )                            //R7?��܇|?y3�S
    {
        CardType = SDIO_STD_CAPACITY_SD_CARD_V2_0;      //SD 2.0?��
        SDType = SD_HIGH_CAPACITY;                      //??��Y��??��
    }
    SDIO_CmdInitStructure.SDIO_Argument = 0x00;//�{�S?��CMD55,?��?��܇|
    SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_CMD;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand( &SDIO_CmdInitStructure );   //�{�S?��CMD55,?��?��܇|
    errorstatus = CmdResp1Error( SD_CMD_APP_CMD );        //������yR1?��܇|
    if( errorstatus == SD_OK ) //SD2.0/SD 1.1,�{??��?aMMC?��
    {
        //SD?��,�{�S?��ACMD41 SD_APP_OP_COND,2?��y?a:0x80100000
        while( ( !validvoltage ) && ( count < SD_MAX_VOLT_TRIAL ) )
        {
            SDIO_CmdInitStructure.SDIO_Argument = 0x00;//�{�S?��CMD55,?��?��܇|
            SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_CMD;   //CMD55
            SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
            SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
            SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
            SDIO_SendCommand( &SDIO_CmdInitStructure );       //�{�S?��CMD55,?��?��܇|
            errorstatus = CmdResp1Error( SD_CMD_APP_CMD );    //������yR1?��܇|
            if( errorstatus != SD_OK )
            {
                return errorstatus;    //?��܇|�ɳ�?܇
            }
            //acmd41�S??����?2?��y܇��?��3?��?��??1�{??��?��HCS??�G��3�ȩS?HCS????��?�ǡ�??�{??����?SDSc?1��?sdhc
            SDIO_CmdInitStructure.SDIO_Argument = SD_VOLTAGE_WINDOW_SD | SDType;  //�{�S?��ACMD41,?��?��܇|
            SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SD_APP_OP_COND;
            SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;  //r3
            SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
            SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
            SDIO_SendCommand( &SDIO_CmdInitStructure );
            errorstatus = CmdResp3Error();                //������yR3?��܇|
            if( errorstatus != SD_OK )
            {
                return errorstatus;    //?��܇|�ɳ�?܇
            }
            response = SDIO->RESP1;;                          //��?��??��܇|
            validvoltage = ( ( ( response >> 31 ) == 1 ) ? 1 : 0 ); //?D??SD?����?��?��?�{?����3��
            count++;
        }
        if( count >= SD_MAX_VOLT_TRIAL )
        {
            errorstatus = SD_INVALID_VOLTRANGE;
            return errorstatus;
        }
        if( response &= SD_HIGH_CAPACITY )
        {
            CardType = SDIO_HIGH_CAPACITY_SD_CARD;
        }
    }
    return( errorstatus );
}
//SD?�� Power OFF
//�{��???��:�ɳ�?܇���~??;(0,?T�ɳ�?܇)
SD_Error SD_PowerOFF( void )
{
    SDIO_SetPowerState( SDIO_PowerState_OFF ); //SDIO��??��1?�I?,�ǩI?܇��S?1
    return SD_OK;
}
//3?��??��?��܇D��??��,2�S��??��??��??��D���G����?
//�{��???��:�ɳ�?܇���~??
SD_Error SD_InitializeCards( void )
{
    SD_Error errorstatus = SD_OK;
    u16 rca = 0x01;
    if( SDIO_GetPowerState() == SDIO_PowerState_OFF ) //?��2����??�ɨG����?,���{�I�S?a��?��?�G����?
    {
        errorstatus = SD_REQUEST_NOT_APPLICABLE;
        return( errorstatus );
    }
    if( SDIO_SECURE_DIGITAL_IO_CARD != CardType )     //�{?SECURE_DIGITAL_IO_CARD
    {
        SDIO_CmdInitStructure.SDIO_Argument = 0x0;//�{�S?��CMD2,��?��?CID,3�N?��܇|
        SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_ALL_SEND_CID;
        SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Long;
        SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
        SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
        SDIO_SendCommand( &SDIO_CmdInitStructure ); //�{�S?��CMD2,��?��?CID,3�N?��܇|
        errorstatus = CmdResp2Error();                  //������yR2?��܇|
        if( errorstatus != SD_OK )
        {
            return errorstatus;    //?��܇|�ɳ�?܇
        }
        CID_Tab[0] = SDIO->RESP1;
        CID_Tab[1] = SDIO->RESP2;
        CID_Tab[2] = SDIO->RESP3;
        CID_Tab[3] = SDIO->RESP4;
    }
    if( ( SDIO_STD_CAPACITY_SD_CARD_V1_1 == CardType ) || ( SDIO_STD_CAPACITY_SD_CARD_V2_0 == CardType ) || ( SDIO_SECURE_DIGITAL_IO_COMBO_CARD == CardType ) || ( SDIO_HIGH_CAPACITY_SD_CARD == CardType ) ) //?D???���ǽ�D��
    {
        SDIO_CmdInitStructure.SDIO_Argument = 0x00;//�{�S?��CMD3,?��?��܇|
        SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_REL_ADDR;  //cmd3
        SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short; //r6
        SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
        SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
        SDIO_SendCommand( &SDIO_CmdInitStructure ); //�{�S?��CMD3,?��?��܇|
        errorstatus = CmdResp6Error( SD_CMD_SET_REL_ADDR, &rca ); //������yR6?��܇|
        if( errorstatus != SD_OK )
        {
            return errorstatus;    //?��܇|�ɳ�?܇
        }
    }
    if( SDIO_MULTIMEDIA_CARD == CardType )
    {
        SDIO_CmdInitStructure.SDIO_Argument = ( u32 )( rca << 16 ); //�{�S?��CMD3,?��?��܇|
        SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_REL_ADDR;  //cmd3
        SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short; //r6
        SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
        SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
        SDIO_SendCommand( &SDIO_CmdInitStructure ); //�{�S?��CMD3,?��?��܇|
        errorstatus = CmdResp2Error();                  //������yR2?��܇|
        if( errorstatus != SD_OK )
        {
            return errorstatus;    //?��܇|�ɳ�?܇
        }
    }
    if( SDIO_SECURE_DIGITAL_IO_CARD != CardType )         //�{?SECURE_DIGITAL_IO_CARD
    {
        RCA = rca;
        SDIO_CmdInitStructure.SDIO_Argument = ( uint32_t )( rca << 16 ); //�{�S?��CMD9+?��RCA,��?��?CSD,3�N?��܇|
        SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SEND_CSD;
        SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Long;
        SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
        SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
        SDIO_SendCommand( &SDIO_CmdInitStructure );
        errorstatus = CmdResp2Error();                  //������yR2?��܇|
        if( errorstatus != SD_OK )
        {
            return errorstatus;    //?��܇|�ɳ�?܇
        }
        CSD_Tab[0] = SDIO->RESP1;
        CSD_Tab[1] = SDIO->RESP2;
        CSD_Tab[2] = SDIO->RESP3;
        CSD_Tab[3] = SDIO->RESP4;
    }
    return SD_OK;//?��3?��??��3��1|
}
//��?��??��D??�S
//cardinfo:?��D??�S��?�ɨS??
//�{��???��:�ɳ�?܇�G����?
SD_Error SD_GetCardInfo( SD_CardInfo* cardinfo )
{
    SD_Error errorstatus = SD_OK;
    u8 tmp = 0;
    cardinfo->CardType = ( u8 )CardType;          //?���ǽ�D��
    cardinfo->RCA = ( u16 )RCA;                       //?��RCA?��
    tmp = ( u8 )( ( CSD_Tab[0] & 0xFF000000 ) >> 24 );
    cardinfo->SD_csd.CSDStruct = ( tmp & 0xC0 ) >> 6; //CSD?��11
    cardinfo->SD_csd.SysSpecVersion = ( tmp & 0x3C ) >> 2; //2.0D-����?1???����??a2?�{?(?a�I�S��?),܇|??��?o܇D?D-����?����?��?
    cardinfo->SD_csd.Reserved1 = tmp & 0x03;      //2??�I�S��???
    tmp = ( u8 )( ( CSD_Tab[0] & 0x00FF0000 ) >> 16 ); //���~1??�G??�~
    cardinfo->SD_csd.TAAC = tmp;                      //��y?Y?���ǩI??1
    tmp = ( u8 )( ( CSD_Tab[0] & 0x0000FF00 ) >> 8 ); //���~2??�G??�~
    cardinfo->SD_csd.NSAC = tmp;                      //��y?Y?���ǩI??2
    tmp = ( u8 )( CSD_Tab[0] & 0x000000FF );      //���~3??�G??�~
    cardinfo->SD_csd.MaxBusClkFrec = tmp;             //��?��??��?��
    tmp = ( u8 )( ( CSD_Tab[1] & 0xFF000000 ) >> 24 ); //���~4??�G??�~
    cardinfo->SD_csd.CardComdClasses = tmp << 4;  //?��??��?�ǽ�??????
    tmp = ( u8 )( ( CSD_Tab[1] & 0x00FF0000 ) >> 16 ); //���~5??�G??�~
    cardinfo->SD_csd.CardComdClasses |= ( tmp & 0xF0 ) >> 4; //?��??��?�ǽ��׳�????
    cardinfo->SD_csd.RdBlockLen = tmp & 0x0F;     //�G?��܇?����?��y?Y3�N?��
    tmp = ( u8 )( ( CSD_Tab[1] & 0x0000FF00 ) >> 8 ); //���~6??�G??�~
    cardinfo->SD_csd.PartBlockRead = ( tmp & 0x80 ) >> 7; //?��D��{??��?��
    cardinfo->SD_csd.WrBlockMisalign = ( tmp & 0x40 ) >> 6; //D��?�ȡɳ�??
    cardinfo->SD_csd.RdBlockMisalign = ( tmp & 0x20 ) >> 5; //?��?�ȡɳ�??
    cardinfo->SD_csd.DSRImpl = ( tmp & 0x10 ) >> 4;
    cardinfo->SD_csd.Reserved2 = 0;                   //�I�S��?
    if( ( CardType == SDIO_STD_CAPACITY_SD_CARD_V1_1 ) || ( CardType == SDIO_STD_CAPACITY_SD_CARD_V2_0 ) || ( SDIO_MULTIMEDIA_CARD == CardType ) ) //�I�ǨG?1.1/2.0?��/MMC?��
    {
        cardinfo->SD_csd.DeviceSize = ( tmp & 0x03 ) << 10; //C_SIZE(12??)
        tmp = ( u8 )( CSD_Tab[1] & 0x000000FF );    //���~7??�G??�~
        cardinfo->SD_csd.DeviceSize |= ( tmp ) << 2;
        tmp = ( u8 )( ( CSD_Tab[2] & 0xFF000000 ) >> 24 ); //���~8??�G??�~
        cardinfo->SD_csd.DeviceSize |= ( tmp & 0xC0 ) >> 6;
        cardinfo->SD_csd.MaxRdCurrentVDDMin = ( tmp & 0x38 ) >> 3;
        cardinfo->SD_csd.MaxRdCurrentVDDMax = ( tmp & 0x07 );
        tmp = ( u8 )( ( CSD_Tab[2] & 0x00FF0000 ) >> 16 ); //���~9??�G??�~
        cardinfo->SD_csd.MaxWrCurrentVDDMin = ( tmp & 0xE0 ) >> 5;
        cardinfo->SD_csd.MaxWrCurrentVDDMax = ( tmp & 0x1C ) >> 2;
        cardinfo->SD_csd.DeviceSizeMul = ( tmp & 0x03 ) << 1; //C_SIZE_MULT
        tmp = ( u8 )( ( CSD_Tab[2] & 0x0000FF00 ) >> 8 ); //���~10??�G??�~
        cardinfo->SD_csd.DeviceSizeMul |= ( tmp & 0x80 ) >> 7;
        cardinfo->CardCapacity = ( cardinfo->SD_csd.DeviceSize + 1 ); //?????����Y��?
        cardinfo->CardCapacity *= ( 1 << ( cardinfo->SD_csd.DeviceSizeMul + 2 ) );
        cardinfo->CardBlockSize = 1 << ( cardinfo->SD_csd.RdBlockLen ); //?�ȡ�܇D?
        cardinfo->CardCapacity *= cardinfo->CardBlockSize;
    }
    else if( CardType == SDIO_HIGH_CAPACITY_SD_CARD ) //??��Y��??��
    {
        tmp = ( u8 )( CSD_Tab[1] & 0x000000FF ); //���~7??�G??�~
        cardinfo->SD_csd.DeviceSize = ( tmp & 0x3F ) << 16; //C_SIZE
        tmp = ( u8 )( ( CSD_Tab[2] & 0xFF000000 ) >> 24 ); //���~8??�G??�~
        cardinfo->SD_csd.DeviceSize |= ( tmp << 8 );
        tmp = ( u8 )( ( CSD_Tab[2] & 0x00FF0000 ) >> 16 ); //���~9??�G??�~
        cardinfo->SD_csd.DeviceSize |= ( tmp );
        tmp = ( u8 )( ( CSD_Tab[2] & 0x0000FF00 ) >> 8 ); //���~10??�G??�~
        cardinfo->CardCapacity = ( long long )( cardinfo->SD_csd.DeviceSize + 1 ) * 512 * 1024; //?????����Y��?
        cardinfo->CardBlockSize = 512;          //?�ȡ�܇D?1��?��?a512�G??�~
    }
    cardinfo->SD_csd.EraseGrSize = ( tmp & 0x40 ) >> 6;
    cardinfo->SD_csd.EraseGrMul = ( tmp & 0x3F ) << 1;
    tmp = ( u8 )( CSD_Tab[2] & 0x000000FF );  //���~11??�G??�~
    cardinfo->SD_csd.EraseGrMul |= ( tmp & 0x80 ) >> 7;
    cardinfo->SD_csd.WrProtectGrSize = ( tmp & 0x7F );
    tmp = ( u8 )( ( CSD_Tab[3] & 0xFF000000 ) >> 24 ); //���~12??�G??�~
    cardinfo->SD_csd.WrProtectGrEnable = ( tmp & 0x80 ) >> 7;
    cardinfo->SD_csd.ManDeflECC = ( tmp & 0x60 ) >> 5;
    cardinfo->SD_csd.WrSpeedFact = ( tmp & 0x1C ) >> 2;
    cardinfo->SD_csd.MaxWrBlockLen = ( tmp & 0x03 ) << 2;
    tmp = ( u8 )( ( CSD_Tab[3] & 0x00FF0000 ) >> 16 ); //���~13??�G??�~
    cardinfo->SD_csd.MaxWrBlockLen |= ( tmp & 0xC0 ) >> 6;
    cardinfo->SD_csd.WriteBlockPaPartial = ( tmp & 0x20 ) >> 5;
    cardinfo->SD_csd.Reserved3 = 0;
    cardinfo->SD_csd.ContentProtectAppli = ( tmp & 0x01 );
    tmp = ( u8 )( ( CSD_Tab[3] & 0x0000FF00 ) >> 8 ); //���~14??�G??�~
    cardinfo->SD_csd.FileFormatGrouop = ( tmp & 0x80 ) >> 7;
    cardinfo->SD_csd.CopyFlag = ( tmp & 0x40 ) >> 6;
    cardinfo->SD_csd.PermWrProtect = ( tmp & 0x20 ) >> 5;
    cardinfo->SD_csd.TempWrProtect = ( tmp & 0x10 ) >> 4;
    cardinfo->SD_csd.FileFormat = ( tmp & 0x0C ) >> 2;
    cardinfo->SD_csd.ECC = ( tmp & 0x03 );
    tmp = ( u8 )( CSD_Tab[3] & 0x000000FF );  //���~15??�G??�~
    cardinfo->SD_csd.CSD_CRC = ( tmp & 0xFE ) >> 1;
    cardinfo->SD_csd.Reserved4 = 1;
    tmp = ( u8 )( ( CID_Tab[0] & 0xFF000000 ) >> 24 ); //���~0??�G??�~
    cardinfo->SD_cid.ManufacturerID = tmp;
    tmp = ( u8 )( ( CID_Tab[0] & 0x00FF0000 ) >> 16 ); //���~1??�G??�~
    cardinfo->SD_cid.OEM_AppliID = tmp << 8;
    tmp = ( u8 )( ( CID_Tab[0] & 0x000000FF00 ) >> 8 ); //���~2??�G??�~
    cardinfo->SD_cid.OEM_AppliID |= tmp;
    tmp = ( u8 )( CID_Tab[0] & 0x000000FF );  //���~3??�G??�~
    cardinfo->SD_cid.ProdName1 = tmp << 24;
    tmp = ( u8 )( ( CID_Tab[1] & 0xFF000000 ) >> 24 ); //���~4??�G??�~
    cardinfo->SD_cid.ProdName1 |= tmp << 16;
    tmp = ( u8 )( ( CID_Tab[1] & 0x00FF0000 ) >> 16 ); //���~5??�G??�~
    cardinfo->SD_cid.ProdName1 |= tmp << 8;
    tmp = ( u8 )( ( CID_Tab[1] & 0x0000FF00 ) >> 8 ); //���~6??�G??�~
    cardinfo->SD_cid.ProdName1 |= tmp;
    tmp = ( u8 )( CID_Tab[1] & 0x000000FF );  //���~7??�G??�~
    cardinfo->SD_cid.ProdName2 = tmp;
    tmp = ( u8 )( ( CID_Tab[2] & 0xFF000000 ) >> 24 ); //���~8??�G??�~
    cardinfo->SD_cid.ProdRev = tmp;
    tmp = ( u8 )( ( CID_Tab[2] & 0x00FF0000 ) >> 16 ); //���~9??�G??�~
    cardinfo->SD_cid.ProdSN = tmp << 24;
    tmp = ( u8 )( ( CID_Tab[2] & 0x0000FF00 ) >> 8 ); //���~10??�G??�~
    cardinfo->SD_cid.ProdSN |= tmp << 16;
    tmp = ( u8 )( CID_Tab[2] & 0x000000FF );      //���~11??�G??�~
    cardinfo->SD_cid.ProdSN |= tmp << 8;
    tmp = ( u8 )( ( CID_Tab[3] & 0xFF000000 ) >> 24 ); //���~12??�G??�~
    cardinfo->SD_cid.ProdSN |= tmp;
    tmp = ( u8 )( ( CID_Tab[3] & 0x00FF0000 ) >> 16 ); //���~13??�G??�~
    cardinfo->SD_cid.Reserved1 |= ( tmp & 0xF0 ) >> 4;
    cardinfo->SD_cid.ManufactDate = ( tmp & 0x0F ) << 8;
    tmp = ( u8 )( ( CID_Tab[3] & 0x0000FF00 ) >> 8 ); //���~14??�G??�~
    cardinfo->SD_cid.ManufactDate |= tmp;
    tmp = ( u8 )( CID_Tab[3] & 0x000000FF );  //���~15??�G??�~
    cardinfo->SD_cid.CID_CRC = ( tmp & 0xFE ) >> 1;
    cardinfo->SD_cid.Reserved2 = 1;
    return errorstatus;
}
//����??SDIO�G��???��?��(MMC?��2??��3?4bit?�S��?)
//wmode:???��?�S��?.0,1??��y?Y?��?��;1,4??��y?Y?��?��;2,8??��y?Y?��?��
//�{��???��:SD?���ɳ�?܇�G����?

//����??SDIO�G��???��?��(MMC?��2??��3?4bit?�S��?)
//   @arg SDIO_BusWide_8b: 8-bit data transfer (Only for MMC)
//   @arg SDIO_BusWide_4b: 4-bit data transfer
//   @arg SDIO_BusWide_1b: 1-bit data transfer (??��?)
//�{��???��:SD?���ɳ�?܇�G����?


SD_Error SD_EnableWideBusOperation( u32 WideMode )
{
    SD_Error errorstatus = SD_OK;
    if( SDIO_MULTIMEDIA_CARD == CardType )
    {
        errorstatus = SD_UNSUPPORTED_FEATURE;
        return( errorstatus );
    }
    else if( ( SDIO_STD_CAPACITY_SD_CARD_V1_1 == CardType ) || ( SDIO_STD_CAPACITY_SD_CARD_V2_0 == CardType ) || ( SDIO_HIGH_CAPACITY_SD_CARD == CardType ) )
    {
        if( SDIO_BusWide_8b == WideMode )  //2.0 sd2??��3?8bits
        {
            errorstatus = SD_UNSUPPORTED_FEATURE;
            return( errorstatus );
        }
        else
        {
            errorstatus = SDEnWideBus( WideMode );
            if( SD_OK == errorstatus )
            {
                SDIO->CLKCR &= ~( 3 << 11 ); //??3y???����????�����??
                SDIO->CLKCR |= WideMode; //1??/4??�G��???��?��
                SDIO->CLKCR |= 0 << 14;     //2??a??܇2?t�֡�????
            }
        }
    }
    return errorstatus;
}
//����??SD?��1�N�G��?�S��?
//Mode:
//�{��???��:�ɳ�?܇�G����?
SD_Error SD_SetDeviceMode( u32 Mode )
{
    SD_Error errorstatus = SD_OK;
    if( ( Mode == SD_DMA_MODE ) || ( Mode == SD_POLLING_MODE ) )
    {
        DeviceMode = Mode;
    }
    else
    {
        errorstatus = SD_INVALID_PARAMETER;
    }
    return errorstatus;
}
//???��
//�{�S?��CMD7,?????��??��??�{(rca)?aaddr��??��,��????????��.��?1??a0,?��??2?????.
//addr:?����?RCA��??�{
SD_Error SD_SelectDeselect( u32 addr )
{
    SDIO_CmdInitStructure.SDIO_Argument =  addr;//�{�S?��CMD7,?????��,?��?��܇|
    SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SEL_DESEL_CARD;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand( &SDIO_CmdInitStructure ); //�{�S?��CMD7,?????��,?��?��܇|
    return CmdResp1Error( SD_CMD_SEL_DESEL_CARD );
}
//SD?��?����?��????��
//buf:?����y?Y?o��???(�I?D?4�G??�~????!!)
//addr:?����?��??�{
//blksize:?�ȡ�܇D?
SD_Error SD_ReadBlock( u8* buf, long long addr, u16 blksize )
{
    SD_Error errorstatus = SD_OK;
    u8 power;
    u32 count = 0, *tempbuff = ( u32* )buf; //�Ga???au32????
    u32 timeout = SDIO_DATATIMEOUT;
    if( NULL == buf )
    {
        return SD_INVALID_PARAMETER;
    }
    SDIO->DCTRL = 0x0; //��y?Y??????��??��??��?(1?DMA)
    if( CardType == SDIO_HIGH_CAPACITY_SD_CARD ) //��܇��Y��??��
    {
        blksize = 512;
        addr >>= 9;
    }
    SDIO_DataInitStructure.SDIO_DataBlockSize = SDIO_DataBlockSize_1b ; //??3yDPSM�G����??�~????
    SDIO_DataInitStructure.SDIO_DataLength = 0 ;
    SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT ;
    SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
    SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToCard;
    SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
    SDIO_DataConfig( &SDIO_DataInitStructure );
    if( SDIO->RESP1 & SD_CARD_LOCKED )
    {
        return SD_LOCK_UNLOCK_FAILED;    //?��??��?
    }
    if( ( blksize > 0 ) && ( blksize <= 2048 ) && ( ( blksize & ( blksize - 1 ) ) == 0 ) )
    {
        power = convert_from_bytes_to_power_of_two( blksize );
        SDIO_CmdInitStructure.SDIO_Argument =  blksize;
        SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_BLOCKLEN;
        SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
        SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
        SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
        SDIO_SendCommand( &SDIO_CmdInitStructure ); //�{�S?��CMD16+����??��y?Y3�N?��?ablksize,?��?��܇|
        errorstatus = CmdResp1Error( SD_CMD_SET_BLOCKLEN ); //������yR1?��܇|
        if( errorstatus != SD_OK )
        {
            return errorstatus;    //?��܇|�ɳ�?܇
        }
    }
    else
    {
        return SD_INVALID_PARAMETER;
    }
    SDIO_DataInitStructure.SDIO_DataBlockSize = power << 4 ; //??3yDPSM�G����??�~????
    SDIO_DataInitStructure.SDIO_DataLength = blksize ;
    SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT ;
    SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
    SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToSDIO;
    SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
    SDIO_DataConfig( &SDIO_DataInitStructure );
    SDIO_CmdInitStructure.SDIO_Argument =  addr;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_READ_SINGLE_BLOCK;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand( &SDIO_CmdInitStructure ); //�{�S?��CMD17+��܇addr��??�{3??����?��y?Y,?��?��܇|
    errorstatus = CmdResp1Error( SD_CMD_READ_SINGLE_BLOCK ); //������yR1?��܇|
    if( errorstatus != SD_OK )
    {
        return errorstatus;    //?��܇|�ɳ�?܇
    }
    if( DeviceMode == SD_POLLING_MODE )                   //2��?��?�S��?,???����y?Y
    {
        INTX_DISABLE();//1?�I?�G��?D??(POLLING?�S��?,?????D??�ɳ�??SDIO?��D��2���G��!!!)
        while( !( SDIO->STA & ( ( 1 << 5 ) | ( 1 << 1 ) | ( 1 << 3 ) | ( 1 << 10 ) | ( 1 << 9 ) ) ) ) //?T��?��?/CRC/3?�ǩI/����3��(�I��??)/?e��???�ɳ�?܇
        {
            if( SDIO_GetFlagStatus( SDIO_FLAG_RXFIFOHF ) != RESET )                   //?܇��???��??�~,�I����??�ֹ�����?��?8??�G?
            {
                for( count = 0; count < 8; count++ )    //?-?�{?����?��y?Y
                {
                    *( tempbuff + count ) = SDIO->FIFO;
                }
                tempbuff += 8;
                timeout = 0X7FFFFF; //?����y?Y��?3?�ǩI??
            }
            else      //��|�ǳ�3?�ǩI
            {
                if( timeout == 0 )
                {
                    return SD_DATA_TIMEOUT;
                }
                timeout--;
            }
        }
        if( SDIO_GetFlagStatus( SDIO_FLAG_DTIMEOUT ) != RESET ) //��y?Y3?�ǩI�ɳ�?܇
        {
            SDIO_ClearFlag( SDIO_FLAG_DTIMEOUT ); //??�ɳ�?܇�I��??
            return SD_DATA_TIMEOUT;
        }
        else if( SDIO_GetFlagStatus( SDIO_FLAG_DCRCFAIL ) != RESET ) //��y?Y?��CRC�ɳ�?܇
        {
            SDIO_ClearFlag( SDIO_FLAG_DCRCFAIL );     //??�ɳ�?܇�I��??
            return SD_DATA_CRC_FAIL;
        }
        else if( SDIO_GetFlagStatus( SDIO_FLAG_RXOVERR ) != RESET ) //?܇��?fifo��?��?�ɳ�?܇
        {
            SDIO_ClearFlag( SDIO_FLAG_RXOVERR );      //??�ɳ�?܇�I��??
            return SD_RX_OVERRUN;
        }
        else if( SDIO_GetFlagStatus( SDIO_FLAG_STBITERR ) != RESET ) //?܇��??e��???�ɳ�?܇
        {
            SDIO_ClearFlag( SDIO_FLAG_STBITERR ); //??�ɳ�?܇�I��??
            return SD_START_BIT_ERR;
        }
        while( SDIO_GetFlagStatus( SDIO_FLAG_RXDAVL ) != RESET ) //FIFO��???,?1��??�~?��܇?��y?Y
        {
            *tempbuff = SDIO->FIFO; //?-?�{?����?��y?Y
            tempbuff++;
        }
        INTX_ENABLE();//?a??�G��?D??
        SDIO_ClearFlag( SDIO_STATIC_FLAGS ); //??3y?��܇D�I��??
    }
    else if( DeviceMode == SD_DMA_MODE )
    {
        TransferError = SD_OK;
        StopCondition = 0;          //�ר�?��?��,2?D����a�{�S?���S?1��?��???��?
        TransferEnd = 0;            //��?��??����?�I��????�S??�~?D??�{t????1
        SDIO->MASK |= ( 1 << 1 ) | ( 1 << 3 ) | ( 1 << 8 ) | ( 1 << 5 ) | ( 1 << 9 ); //????D����a��??D??
        SDIO->DCTRL |= 1 << 3;      //SDIO DMA��1?��
        SD_DMA_Config( ( u32* )buf, blksize, DMA_DIR_PeripheralToMemory );
        while( ( ( DMA2->LISR & ( 1 << 27 ) ) == RESET ) && ( TransferEnd == 0 ) && ( TransferError == SD_OK ) && timeout )
        {
            timeout--;    //������y��?��?����3��
        }
        if( timeout == 0 )
        {
            return SD_DATA_TIMEOUT;    //3?�ǩI
        }
        if( TransferError != SD_OK )
        {
            errorstatus = TransferError;
        }
    }
    return errorstatus;
}
//SD?��?����??��???��
//buf:?����y?Y?o��???
//addr:?����?��??�{
//blksize:?�ȡ�܇D?
//nblks:��a?����?��??����y
//�{��???��:�ɳ�?܇�G����?
#pragma pack(push,4)
__no_init u32* tempbuff;
SD_Error SD_ReadMultiBlocks( u8* buf, long long addr, u16 blksize, u32 nblks )
{
    SD_Error errorstatus = SD_OK;
    u8 power;
    u32 count = 0;
    u32 timeout = SDIO_DATATIMEOUT;
    tempbuff = ( u32* )buf; //�Ga???au32????
    SDIO->DCTRL = 0x0;    //��y?Y??????��??��??��?(1?DMA)
    if( CardType == SDIO_HIGH_CAPACITY_SD_CARD ) //��܇��Y��??��
    {
        blksize = 512;
        addr >>= 9;
    }
    SDIO_DataInitStructure.SDIO_DataBlockSize = 0; ; //??3yDPSM�G����??�~????
    SDIO_DataInitStructure.SDIO_DataLength = 0 ;
    SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT ;
    SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
    SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToCard;
    SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
    SDIO_DataConfig( &SDIO_DataInitStructure );
    if( SDIO->RESP1 & SD_CARD_LOCKED )
    {
        return SD_LOCK_UNLOCK_FAILED;    //?��??��?
    }
    if( ( blksize > 0 ) && ( blksize <= 2048 ) && ( ( blksize & ( blksize - 1 ) ) == 0 ) )
    {
        power = convert_from_bytes_to_power_of_two( blksize );
        SDIO_CmdInitStructure.SDIO_Argument =  blksize;//�{�S?��CMD16+����??��y?Y3�N?��?ablksize,?��?��܇|
        SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_BLOCKLEN;
        SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
        SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
        SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
        SDIO_SendCommand( &SDIO_CmdInitStructure );
        errorstatus = CmdResp1Error( SD_CMD_SET_BLOCKLEN ); //������yR1?��܇|
        if( errorstatus != SD_OK )
        {
            return errorstatus;    //?��܇|�ɳ�?܇
        }
    }
    else
    {
        return SD_INVALID_PARAMETER;
    }
    if( nblks > 1 )                                       //?��?��?��
    {
        if( nblks * blksize > SD_MAX_DATA_LENGTH )
        {
            return SD_INVALID_PARAMETER;    //?D??��?�{?3?1y�G?��܇?܇��?3�N?��
        }
        SDIO_DataInitStructure.SDIO_DataBlockSize = power << 4; ; //nblks*blksize,512?�ȡ�܇D?,?����??????��
        SDIO_DataInitStructure.SDIO_DataLength = nblks * blksize ;
        SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT ;
        SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
        SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToSDIO;
        SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
        SDIO_DataConfig( &SDIO_DataInitStructure );
        SDIO_CmdInitStructure.SDIO_Argument =  addr;//�{�S?��CMD18+��܇addr��??�{3??����?��y?Y,?��?��܇|
        SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_READ_MULT_BLOCK;
        SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
        SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
        SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
        SDIO_SendCommand( &SDIO_CmdInitStructure );
        errorstatus = CmdResp1Error( SD_CMD_READ_MULT_BLOCK ); //������yR1?��܇|
        if( errorstatus != SD_OK )
        {
            return errorstatus;    //?��܇|�ɳ�?܇
        }
        if( DeviceMode == SD_POLLING_MODE )
        {
            INTX_DISABLE();//1?�I?�G��?D??(POLLING?�S��?,?????D??�ɳ�??SDIO?��D��2���G��!!!)
            while( !( SDIO->STA & ( ( 1 << 5 ) | ( 1 << 1 ) | ( 1 << 3 ) | ( 1 << 8 ) | ( 1 << 9 ) ) ) ) //?T��?��?/CRC/3?�ǩI/����3��(�I��??)/?e��???�ɳ�?܇
            {
                if( SDIO_GetFlagStatus( SDIO_FLAG_RXFIFOHF ) != RESET )                 //?܇��???��??�~,�I����??�ֹ�����?��?8??�G?
                {
                    for( count = 0; count < 8; count++ )  //?-?�{?����?��y?Y
                    {
                        *( tempbuff + count ) = SDIO->FIFO;
                    }
                    tempbuff += 8;
                    timeout = 0X7FFFFF;   //?����y?Y��?3?�ǩI??
                }
                else    //��|�ǳ�3?�ǩI
                {
                    if( timeout == 0 )
                    {
                        return SD_DATA_TIMEOUT;
                    }
                    timeout--;
                }
            }
            if( SDIO_GetFlagStatus( SDIO_FLAG_DTIMEOUT ) != RESET )   //��y?Y3?�ǩI�ɳ�?܇
            {
                SDIO_ClearFlag( SDIO_FLAG_DTIMEOUT );   //??�ɳ�?܇�I��??
                return SD_DATA_TIMEOUT;
            }
            else if( SDIO_GetFlagStatus( SDIO_FLAG_DCRCFAIL ) != RESET ) //��y?Y?��CRC�ɳ�?܇
            {
                SDIO_ClearFlag( SDIO_FLAG_DCRCFAIL );       //??�ɳ�?܇�I��??
                return SD_DATA_CRC_FAIL;
            }
            else if( SDIO_GetFlagStatus( SDIO_FLAG_RXOVERR ) != RESET ) //?܇��?fifo��?��?�ɳ�?܇
            {
                SDIO_ClearFlag( SDIO_FLAG_RXOVERR );    //??�ɳ�?܇�I��??
                return SD_RX_OVERRUN;
            }
            else if( SDIO_GetFlagStatus( SDIO_FLAG_STBITERR ) != RESET )  //?܇��??e��???�ɳ�?܇
            {
                SDIO_ClearFlag( SDIO_FLAG_STBITERR ); //??�ɳ�?܇�I��??
                return SD_START_BIT_ERR;
            }
            while( SDIO_GetFlagStatus( SDIO_FLAG_RXDAVL ) != RESET ) //FIFO��???,?1��??�~?��܇?��y?Y
            {
                *tempbuff = SDIO->FIFO; //?-?�{?����?��y?Y
                tempbuff++;
            }
            if( SDIO_GetFlagStatus( SDIO_FLAG_DATAEND ) != RESET )    //?܇��??����?
            {
                if( ( SDIO_STD_CAPACITY_SD_CARD_V1_1 == CardType ) || ( SDIO_STD_CAPACITY_SD_CARD_V2_0 == CardType ) || ( SDIO_HIGH_CAPACITY_SD_CARD == CardType ) )
                {
                    SDIO_CmdInitStructure.SDIO_Argument =  0;//�{�S?��CMD12+?����?��?��?
                    SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_STOP_TRANSMISSION;
                    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
                    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
                    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
                    SDIO_SendCommand( &SDIO_CmdInitStructure );
                    errorstatus = CmdResp1Error( SD_CMD_STOP_TRANSMISSION ); //������yR1?��܇|
                    if( errorstatus != SD_OK )
                    {
                        return errorstatus;
                    }
                }
            }
            INTX_ENABLE();//?a??�G��?D??
            SDIO_ClearFlag( SDIO_STATIC_FLAGS ); //??3y?��܇D�I��??
        }
        else if( DeviceMode == SD_DMA_MODE )
        {
            TransferError = SD_OK;
            StopCondition = 1;        //?��?��?��,D����a�{�S?���S?1��?��???��?
            TransferEnd = 0;              //��?��??����?�I��????�S??�~?D??�{t????1
            SDIO->MASK |= ( 1 << 1 ) | ( 1 << 3 ) | ( 1 << 8 ) | ( 1 << 5 ) | ( 1 << 9 ); //????D����a��??D??
            SDIO->DCTRL |= 1 << 3;                            //SDIO DMA��1?��
            SD_DMA_Config( ( u32* )buf, nblks * blksize, DMA_DIR_PeripheralToMemory );
            while( ( ( DMA2->LISR & ( 1 << 27 ) ) == RESET ) && timeout )
            {
                timeout--;    //������y��?��?����3��
            }
            if( timeout == 0 )
            {
                return SD_DATA_TIMEOUT;    //3?�ǩI
            }
            while( ( TransferEnd == 0 ) && ( TransferError == SD_OK ) );
            if( TransferError != SD_OK )
            {
                errorstatus = TransferError;
            }
        }
    }
    return errorstatus;
}
//SD?��D��1???��
//buf:��y?Y?o��???
//addr:D����??�{
//blksize:?�ȡ�܇D?
//�{��???��:�ɳ�?܇�G����?
SD_Error SD_WriteBlock( u8* buf, long long addr,  u16 blksize )
{
    SD_Error errorstatus = SD_OK;
    u8  power = 0, cardstate = 0;
    u32 timeout = 0, bytestransferred = 0;
    u32 cardstatus = 0, count = 0, restwords = 0;
    u32   tlen = blksize;                     //�G��3�N?��(�G??�~)
    u32* tempbuff = ( u32* )buf;
    if( buf == NULL )
    {
        return SD_INVALID_PARAMETER;    //2?��y�ɳ�?܇
    }
    SDIO->DCTRL = 0x0;                        //��y?Y??????��??��??��?(1?DMA)
    SDIO_DataInitStructure.SDIO_DataBlockSize = 0; ; //??3yDPSM�G����??�~????
    SDIO_DataInitStructure.SDIO_DataLength = 0 ;
    SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT ;
    SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
    SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToCard;
    SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
    SDIO_DataConfig( &SDIO_DataInitStructure );
    if( SDIO->RESP1 & SD_CARD_LOCKED )
    {
        return SD_LOCK_UNLOCK_FAILED;    //?��??��?
    }
    if( CardType == SDIO_HIGH_CAPACITY_SD_CARD ) //��܇��Y��??��
    {
        blksize = 512;
        addr >>= 9;
    }
    if( ( blksize > 0 ) && ( blksize <= 2048 ) && ( ( blksize & ( blksize - 1 ) ) == 0 ) )
    {
        power = convert_from_bytes_to_power_of_two( blksize );
        SDIO_CmdInitStructure.SDIO_Argument = blksize;//�{�S?��CMD16+����??��y?Y3�N?��?ablksize,?��?��܇|
        SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_BLOCKLEN;
        SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
        SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
        SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
        SDIO_SendCommand( &SDIO_CmdInitStructure );
        errorstatus = CmdResp1Error( SD_CMD_SET_BLOCKLEN ); //������yR1?��܇|
        if( errorstatus != SD_OK )
        {
            return errorstatus;    //?��܇|�ɳ�?܇
        }
    }
    else
    {
        return SD_INVALID_PARAMETER;
    }
    SDIO_CmdInitStructure.SDIO_Argument = ( u32 )RCA << 16; //�{�S?��CMD13,2��?��?����?�G����?,?��?��܇|
    SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SEND_STATUS;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand( &SDIO_CmdInitStructure );
    errorstatus = CmdResp1Error( SD_CMD_SEND_STATUS );    //������yR1?��܇|
    if( errorstatus != SD_OK )
    {
        return errorstatus;
    }
    cardstatus = SDIO->RESP1;
    timeout = SD_DATATIMEOUT;
    while( ( ( cardstatus & 0x00000100 ) == 0 ) && ( timeout > 0 ) ) //?��2��READY_FOR_DATA??��?�{?????
    {
        timeout--;
        SDIO_CmdInitStructure.SDIO_Argument = ( u32 )RCA << 16; //�{�S?��CMD13,2��?��?����?�G����?,?��?��܇|
        SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SEND_STATUS;
        SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
        SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
        SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
        SDIO_SendCommand( &SDIO_CmdInitStructure );
        errorstatus = CmdResp1Error( SD_CMD_SEND_STATUS ); //������yR1?��܇|
        if( errorstatus != SD_OK )
        {
            return errorstatus;
        }
        cardstatus = SDIO->RESP1;
    }
    if( timeout == 0 )
    {
        return SD_ERROR;
    }
    SDIO_CmdInitStructure.SDIO_Argument = addr;//�{�S?��CMD24,D���ר�?��??��?,?��?��܇|
    SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_WRITE_SINGLE_BLOCK;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand( &SDIO_CmdInitStructure );
    errorstatus = CmdResp1Error( SD_CMD_WRITE_SINGLE_BLOCK ); //������yR1?��܇|
    if( errorstatus != SD_OK )
    {
        return errorstatus;
    }
    StopCondition = 0;                                //�ר�?��D��,2?D����a�{�S?���S?1��?��???��?
    SDIO_DataInitStructure.SDIO_DataBlockSize = power << 4; ; //blksize, ?????����??��
    SDIO_DataInitStructure.SDIO_DataLength = blksize ;
    SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT ;
    SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
    SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToCard;
    SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
    SDIO_DataConfig( &SDIO_DataInitStructure );
    timeout = SDIO_DATATIMEOUT;
    if( DeviceMode == SD_POLLING_MODE )
    {
        INTX_DISABLE();//1?�I?�G��?D??(POLLING?�S��?,?????D??�ɳ�??SDIO?��D��2���G��!!!)
        while( !( SDIO->STA & ( ( 1 << 10 ) | ( 1 << 4 ) | ( 1 << 1 ) | ( 1 << 3 ) | ( 1 << 9 ) ) ) ) //��y?Y?�ȩ{�S?��3��1|/??��?/CRC/3?�ǩI/?e��???�ɳ�?܇
        {
            if( SDIO_GetFlagStatus( SDIO_FLAG_TXFIFOHE ) != RESET )                       //�{�S?��??��???,�I����??�ֹ�����?��?8??�G?
            {
                if( ( tlen - bytestransferred ) < SD_HALFFIFOBYTES ) //2?1?32�G??�~��?
                {
                    restwords = ( ( tlen - bytestransferred ) % 4 == 0 ) ? ( ( tlen - bytestransferred ) / 4 ) : ( ( tlen - bytestransferred ) / 4 + 1 );
                    for( count = 0; count < restwords; count++, tempbuff++, bytestransferred += 4 )
                    {
                        SDIO->FIFO = *tempbuff;
                    }
                }
                else
                {
                    for( count = 0; count < 8; count++ )
                    {
                        SDIO->FIFO = *( tempbuff + count );
                    }
                    tempbuff += 8;
                    bytestransferred += 32;
                }
                timeout = 0X3FFFFFFF; //D����y?Y��?3?�ǩI??
            }
            else
            {
                if( timeout == 0 )
                {
                    return SD_DATA_TIMEOUT;
                }
                timeout--;
            }
        }
        if( SDIO_GetFlagStatus( SDIO_FLAG_DTIMEOUT ) != RESET ) //��y?Y3?�ǩI�ɳ�?܇
        {
            SDIO_ClearFlag( SDIO_FLAG_DTIMEOUT ); //??�ɳ�?܇�I��??
            return SD_DATA_TIMEOUT;
        }
        else if( SDIO_GetFlagStatus( SDIO_FLAG_DCRCFAIL ) != RESET ) //��y?Y?��CRC�ɳ�?܇
        {
            SDIO_ClearFlag( SDIO_FLAG_DCRCFAIL );     //??�ɳ�?܇�I��??
            return SD_DATA_CRC_FAIL;
        }
        else if( SDIO_GetFlagStatus( SDIO_FLAG_TXUNDERR ) != RESET ) //?܇��?fifo??��?�ɳ�?܇
        {
            SDIO_ClearFlag( SDIO_FLAG_TXUNDERR );     //??�ɳ�?܇�I��??
            return SD_TX_UNDERRUN;
        }
        else if( SDIO_GetFlagStatus( SDIO_FLAG_STBITERR ) != RESET ) //?܇��??e��???�ɳ�?܇
        {
            SDIO_ClearFlag( SDIO_FLAG_STBITERR ); //??�ɳ�?܇�I��??
            return SD_START_BIT_ERR;
        }
        INTX_ENABLE();//?a??�G��?D??
        SDIO_ClearFlag( SDIO_STATIC_FLAGS ); //??3y?��܇D�I��??
    }
    else if( DeviceMode == SD_DMA_MODE )
    {
        TransferError = SD_OK;
        StopCondition = 0;          //�ר�?��D��,2?D����a�{�S?���S?1��?��???��?
        TransferEnd = 0;            //��?��??����?�I��????�S??�~?D??�{t????1
        SDIO->MASK |= ( 1 << 1 ) | ( 1 << 3 ) | ( 1 << 8 ) | ( 1 << 4 ) | ( 1 << 9 ); //????2�~���~��y?Y?܇��?����3��?D??
        SD_DMA_Config( ( u32* )buf, blksize, DMA_DIR_MemoryToPeripheral );          //SDIO DMA????
        SDIO->DCTRL |= 1 << 3;                          //SDIO DMA��1?��.
        while( ( ( DMA2->LISR & ( 1 << 27 ) ) == RESET ) && timeout )
        {
            timeout--;    //������y��?��?����3��
        }
        if( timeout == 0 )
        {
            SD_Init();                        //??D?3?��??��SD?��,?�ȳ�??a??D����??��?�~��??����a
            return SD_DATA_TIMEOUT;           //3?�ǩI
        }
        timeout = SDIO_DATATIMEOUT;
        while( ( TransferEnd == 0 ) && ( TransferError == SD_OK ) && timeout )
        {
            timeout--;
        }
        if( timeout == 0 )
        {
            return SD_DATA_TIMEOUT;    //3?�ǩI
        }
        if( TransferError != SD_OK )
        {
            return TransferError;
        }
    }
    SDIO_ClearFlag( SDIO_STATIC_FLAGS ); //??3y?��܇D�I��??
    errorstatus = IsCardProgramming( &cardstate );
    while( ( errorstatus == SD_OK ) && ( ( cardstate == SD_CARD_PROGRAMMING ) || ( cardstate == SD_CARD_RECEIVING ) ) )
    {
        errorstatus = IsCardProgramming( &cardstate );
    }
    return errorstatus;
}
//SD?��D��?��???��
//buf:��y?Y?o��???
//addr:D����??�{
//blksize:?�ȡ�܇D?
//nblks:��aD����?��??����y
//�{��???��:�ɳ�?܇�G����?
SD_Error SD_WriteMultiBlocks( u8* buf, long long addr, u16 blksize, u32 nblks )
{
    SD_Error errorstatus = SD_OK;
    u8  power = 0, cardstate = 0;
    u32 timeout = 0, bytestransferred = 0;
    u32 count = 0, restwords = 0;
    u32 tlen = nblks * blksize;           //�G��3�N?��(�G??�~)
    u32* tempbuff = ( u32* )buf;
    if( buf == NULL )
    {
        return SD_INVALID_PARAMETER;    //2?��y�ɳ�?܇
    }
    SDIO->DCTRL = 0x0;                        //��y?Y??????��??��??��?(1?DMA)
    SDIO_DataInitStructure.SDIO_DataBlockSize = 0; ;  //??3yDPSM�G����??�~????
    SDIO_DataInitStructure.SDIO_DataLength = 0 ;
    SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT ;
    SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
    SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToCard;
    SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
    SDIO_DataConfig( &SDIO_DataInitStructure );
    if( SDIO->RESP1 & SD_CARD_LOCKED )
    {
        return SD_LOCK_UNLOCK_FAILED;    //?��??��?
    }
    if( CardType == SDIO_HIGH_CAPACITY_SD_CARD ) //��܇��Y��??��
    {
        blksize = 512;
        addr >>= 9;
    }
    if( ( blksize > 0 ) && ( blksize <= 2048 ) && ( ( blksize & ( blksize - 1 ) ) == 0 ) )
    {
        power = convert_from_bytes_to_power_of_two( blksize );
        SDIO_CmdInitStructure.SDIO_Argument = blksize;  //�{�S?��CMD16+����??��y?Y3�N?��?ablksize,?��?��܇|
        SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_BLOCKLEN;
        SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
        SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
        SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
        SDIO_SendCommand( &SDIO_CmdInitStructure );
        errorstatus = CmdResp1Error( SD_CMD_SET_BLOCKLEN ); //������yR1?��܇|
        if( errorstatus != SD_OK )
        {
            return errorstatus;    //?��܇|�ɳ�?܇
        }
    }
    else
    {
        return SD_INVALID_PARAMETER;
    }
    if( nblks > 1 )
    {
        if( nblks * blksize > SD_MAX_DATA_LENGTH )
        {
            return SD_INVALID_PARAMETER;
        }
        if( ( SDIO_STD_CAPACITY_SD_CARD_V1_1 == CardType ) || ( SDIO_STD_CAPACITY_SD_CARD_V2_0 == CardType ) || ( SDIO_HIGH_CAPACITY_SD_CARD == CardType ) )
        {
            //����??D??��
            SDIO_CmdInitStructure.SDIO_Argument = ( u32 )RCA << 16;   //�{�S?��ACMD55,?��?��܇|
            SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_CMD;
            SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
            SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
            SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
            SDIO_SendCommand( &SDIO_CmdInitStructure );
            errorstatus = CmdResp1Error( SD_CMD_APP_CMD );    //������yR1?��܇|
            if( errorstatus != SD_OK )
            {
                return errorstatus;
            }
            SDIO_CmdInitStructure.SDIO_Argument = nblks;      //�{�S?��CMD23,����???����y��?,?��?��܇|
            SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_BLOCK_COUNT;
            SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
            SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
            SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
            SDIO_SendCommand( &SDIO_CmdInitStructure );
            errorstatus = CmdResp1Error( SD_CMD_SET_BLOCK_COUNT ); //������yR1?��܇|
            if( errorstatus != SD_OK )
            {
                return errorstatus;
            }
        }
        SDIO_CmdInitStructure.SDIO_Argument = addr; //�{�S?��CMD25,?��?��D��??��?,?��?��܇|
        SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_WRITE_MULT_BLOCK;
        SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
        SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
        SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
        SDIO_SendCommand( &SDIO_CmdInitStructure );
        errorstatus = CmdResp1Error( SD_CMD_WRITE_MULT_BLOCK ); //������yR1?��܇|
        if( errorstatus != SD_OK )
        {
            return errorstatus;
        }
        SDIO_DataInitStructure.SDIO_DataBlockSize = power << 4; ; //blksize, ?????����??��
        SDIO_DataInitStructure.SDIO_DataLength = nblks * blksize ;
        SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT ;
        SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
        SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToCard;
        SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
        SDIO_DataConfig( &SDIO_DataInitStructure );
        if( DeviceMode == SD_POLLING_MODE )
        {
            timeout = SDIO_DATATIMEOUT;
            INTX_DISABLE();//1?�I?�G��?D??(POLLING?�S��?,?????D??�ɳ�??SDIO?��D��2���G��!!!)
            while( !( SDIO->STA & ( ( 1 << 4 ) | ( 1 << 1 ) | ( 1 << 8 ) | ( 1 << 3 ) | ( 1 << 9 ) ) ) ) //??��?/CRC/��y?Y?����?/3?�ǩI/?e��???�ɳ�?܇
            {
                if( SDIO_GetFlagStatus( SDIO_FLAG_TXFIFOHE ) != RESET )                     //�{�S?��??��???,�I����??�ֹ�����?��?8�G?(32�G??�~)
                {
                    if( ( tlen - bytestransferred ) < SD_HALFFIFOBYTES ) //2?1?32�G??�~��?
                    {
                        restwords = ( ( tlen - bytestransferred ) % 4 == 0 ) ? ( ( tlen - bytestransferred ) / 4 ) : ( ( tlen - bytestransferred ) / 4 + 1 );
                        for( count = 0; count < restwords; count++, tempbuff++, bytestransferred += 4 )
                        {
                            SDIO->FIFO = *tempbuff;
                        }
                    }
                    else                                          //�{�S?��??��???,?�ȳ�?�{�S?��?�ֹ���8�G?(32�G??�~)��y?Y
                    {
                        for( count = 0; count < SD_HALFFIFO; count++ )
                        {
                            SDIO->FIFO = *( tempbuff + count );
                        }
                        tempbuff += SD_HALFFIFO;
                        bytestransferred += SD_HALFFIFOBYTES;
                    }
                    timeout = 0X3FFFFFFF; //D����y?Y��?3?�ǩI??
                }
                else
                {
                    if( timeout == 0 )
                    {
                        return SD_DATA_TIMEOUT;
                    }
                    timeout--;
                }
            }
            if( SDIO_GetFlagStatus( SDIO_FLAG_DTIMEOUT ) != RESET )   //��y?Y3?�ǩI�ɳ�?܇
            {
                SDIO_ClearFlag( SDIO_FLAG_DTIMEOUT );   //??�ɳ�?܇�I��??
                return SD_DATA_TIMEOUT;
            }
            else if( SDIO_GetFlagStatus( SDIO_FLAG_DCRCFAIL ) != RESET ) //��y?Y?��CRC�ɳ�?܇
            {
                SDIO_ClearFlag( SDIO_FLAG_DCRCFAIL );       //??�ɳ�?܇�I��??
                return SD_DATA_CRC_FAIL;
            }
            else if( SDIO_GetFlagStatus( SDIO_FLAG_TXUNDERR ) != RESET )  //?܇��?fifo??��?�ɳ�?܇
            {
                SDIO_ClearFlag( SDIO_FLAG_TXUNDERR );   //??�ɳ�?܇�I��??
                return SD_TX_UNDERRUN;
            }
            else if( SDIO_GetFlagStatus( SDIO_FLAG_STBITERR ) != RESET )  //?܇��??e��???�ɳ�?܇
            {
                SDIO_ClearFlag( SDIO_FLAG_STBITERR ); //??�ɳ�?܇�I��??
                return SD_START_BIT_ERR;
            }
            if( SDIO_GetFlagStatus( SDIO_FLAG_DATAEND ) != RESET )    //�{�S?��?����?
            {
                if( ( SDIO_STD_CAPACITY_SD_CARD_V1_1 == CardType ) || ( SDIO_STD_CAPACITY_SD_CARD_V2_0 == CardType ) || ( SDIO_HIGH_CAPACITY_SD_CARD == CardType ) )
                {
                    SDIO_CmdInitStructure.SDIO_Argument = 0; //�{�S?��CMD12+?����?��?��?
                    SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_STOP_TRANSMISSION;
                    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
                    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
                    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
                    SDIO_SendCommand( &SDIO_CmdInitStructure );
                    errorstatus = CmdResp1Error( SD_CMD_STOP_TRANSMISSION ); //������yR1?��܇|
                    if( errorstatus != SD_OK )
                    {
                        return errorstatus;
                    }
                }
            }
            INTX_ENABLE();//?a??�G��?D??
            SDIO_ClearFlag( SDIO_STATIC_FLAGS ); //??3y?��܇D�I��??
        }
        else if( DeviceMode == SD_DMA_MODE )
        {
            TransferError = SD_OK;
            StopCondition = 1;        //?��?��D��,D����a�{�S?���S?1��?��???��?
            TransferEnd = 0;              //��?��??����?�I��????�S??�~?D??�{t????1
            SDIO->MASK |= ( 1 << 1 ) | ( 1 << 3 ) | ( 1 << 8 ) | ( 1 << 4 ) | ( 1 << 9 ); //????2�~���~��y?Y?܇��?����3��?D??
            SD_DMA_Config( ( u32* )buf, nblks * blksize, DMA_DIR_MemoryToPeripheral ); //SDIO DMA????
            SDIO->DCTRL |= 1 << 3;                            //SDIO DMA��1?��.
            timeout = SDIO_DATATIMEOUT;
            while( ( ( DMA2->LISR & ( 1 << 27 ) ) == RESET ) && timeout )
            {
                timeout--;    //������y��?��?����3��
            }
            if( timeout == 0 )                                //3?�ǩI
            {
                SD_Init();                      //??D?3?��??��SD?��,?�ȳ�??a??D����??��?�~��??����a
                return SD_DATA_TIMEOUT;         //3?�ǩI
            }
            timeout = SDIO_DATATIMEOUT;
            while( ( TransferEnd == 0 ) && ( TransferError == SD_OK ) && timeout )
            {
                timeout--;
            }
            if( timeout == 0 )
            {
                return SD_DATA_TIMEOUT;    //3?�ǩI
            }
            if( TransferError != SD_OK )
            {
                return TransferError;
            }
        }
    }
    SDIO_ClearFlag( SDIO_STATIC_FLAGS ); //??3y?��܇D�I��??
    errorstatus = IsCardProgramming( &cardstate );
    while( ( errorstatus == SD_OK ) && ( ( cardstate == SD_CARD_PROGRAMMING ) || ( cardstate == SD_CARD_RECEIVING ) ) )
    {
        errorstatus = IsCardProgramming( &cardstate );
    }
    return errorstatus;
}
//SDIO?D??�{t??o����y
void SDIO_IRQHandler( void )
{
    SD_ProcessIRQSrc();//��|�ǳ�?��܇DSDIO?��1??D??
}
//SDIO?D??��|�ǳ�o����y
//��|�ǳ�SDIO��?��?1y3��?D��??��???D??��???
//�{��???��:�ɳ�?܇���~??
SD_Error SD_ProcessIRQSrc( void )
{
    if( SDIO_GetFlagStatus( SDIO_FLAG_DATAEND ) != RESET ) //?܇��?����3��?D??
    {
        if( StopCondition == 1 )
        {
            SDIO_CmdInitStructure.SDIO_Argument = 0; //�{�S?��CMD12+?����?��?��?
            SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_STOP_TRANSMISSION;
            SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
            SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
            SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
            SDIO_SendCommand( &SDIO_CmdInitStructure );
            TransferError = CmdResp1Error( SD_CMD_STOP_TRANSMISSION );
        }
        else
        {
            TransferError = SD_OK;
        }
        SDIO->ICR |= 1 << 8; //??3y����3��?D??�I��??
        SDIO->MASK &= ~( ( 1 << 1 ) | ( 1 << 3 ) | ( 1 << 8 ) | ( 1 << 14 ) | ( 1 << 15 ) | ( 1 << 4 ) | ( 1 << 5 ) | ( 1 << 9 ) ); //1?�I??��1??D??
        TransferEnd = 1;
        return( TransferError );
    }
    if( SDIO_GetFlagStatus( SDIO_FLAG_DCRCFAIL ) != RESET ) //��y?YCRC�ɳ�?܇
    {
        SDIO_ClearFlag( SDIO_FLAG_DCRCFAIL );       //??�ɳ�?܇�I��??
        SDIO->MASK &= ~( ( 1 << 1 ) | ( 1 << 3 ) | ( 1 << 8 ) | ( 1 << 14 ) | ( 1 << 15 ) | ( 1 << 4 ) | ( 1 << 5 ) | ( 1 << 9 ) ); //1?�I??��1??D??
        TransferError = SD_DATA_CRC_FAIL;
        return( SD_DATA_CRC_FAIL );
    }
    if( SDIO_GetFlagStatus( SDIO_FLAG_DTIMEOUT ) != RESET ) //��y?Y3?�ǩI�ɳ�?܇
    {
        SDIO_ClearFlag( SDIO_FLAG_DTIMEOUT );           //???D??�I��??
        SDIO->MASK &= ~( ( 1 << 1 ) | ( 1 << 3 ) | ( 1 << 8 ) | ( 1 << 14 ) | ( 1 << 15 ) | ( 1 << 4 ) | ( 1 << 5 ) | ( 1 << 9 ) ); //1?�I??��1??D??
        TransferError = SD_DATA_TIMEOUT;
        return( SD_DATA_TIMEOUT );
    }
    if( SDIO_GetFlagStatus( SDIO_FLAG_RXOVERR ) != RESET ) //FIFO��?��?�ɳ�?܇
    {
        SDIO_ClearFlag( SDIO_FLAG_RXOVERR );            //???D??�I��??
        SDIO->MASK &= ~( ( 1 << 1 ) | ( 1 << 3 ) | ( 1 << 8 ) | ( 1 << 14 ) | ( 1 << 15 ) | ( 1 << 4 ) | ( 1 << 5 ) | ( 1 << 9 ) ); //1?�I??��1??D??
        TransferError = SD_RX_OVERRUN;
        return( SD_RX_OVERRUN );
    }
    if( SDIO_GetFlagStatus( SDIO_FLAG_TXUNDERR ) != RESET ) //FIFO??��?�ɳ�?܇
    {
        SDIO_ClearFlag( SDIO_FLAG_TXUNDERR );           //???D??�I��??
        SDIO->MASK &= ~( ( 1 << 1 ) | ( 1 << 3 ) | ( 1 << 8 ) | ( 1 << 14 ) | ( 1 << 15 ) | ( 1 << 4 ) | ( 1 << 5 ) | ( 1 << 9 ) ); //1?�I??��1??D??
        TransferError = SD_TX_UNDERRUN;
        return( SD_TX_UNDERRUN );
    }
    if( SDIO_GetFlagStatus( SDIO_FLAG_STBITERR ) != RESET ) //?e��???�ɳ�?܇
    {
        SDIO_ClearFlag( SDIO_FLAG_STBITERR );       //???D??�I��??
        SDIO->MASK &= ~( ( 1 << 1 ) | ( 1 << 3 ) | ( 1 << 8 ) | ( 1 << 14 ) | ( 1 << 15 ) | ( 1 << 4 ) | ( 1 << 5 ) | ( 1 << 9 ) ); //1?�I??��1??D??
        TransferError = SD_START_BIT_ERR;
        return( SD_START_BIT_ERR );
    }
    return( SD_OK );
}

//?��2��CMD0��??��DD�G����?
//�{��???��:sd?���ɳ�?܇??
SD_Error CmdError( void )
{
    SD_Error errorstatus = SD_OK;
    u32 timeout = SDIO_CMD0TIMEOUT;
    while( timeout-- )
    {
        if( SDIO_GetFlagStatus( SDIO_FLAG_CMDSENT ) != RESET )
        {
            break;    //?����?��?�{�S?��(?TD��?��܇|)
        }
    }
    if( timeout == 0 )
    {
        return SD_CMD_RSP_TIMEOUT;
    }
    SDIO_ClearFlag( SDIO_STATIC_FLAGS ); //??3y?��܇D�I��??
    return errorstatus;
}
//?��2��R7?��܇|��?�ɳ�?܇�G����?
//�{��???��:sd?���ɳ�?܇??
SD_Error CmdResp7Error( void )
{
    SD_Error errorstatus = SD_OK;
    u32 status;
    u32 timeout = SDIO_CMD0TIMEOUT;
    while( timeout-- )
    {
        status = SDIO->STA;
        if( status & ( ( 1 << 0 ) | ( 1 << 2 ) | ( 1 << 6 ) ) )
        {
            break;    //CRC�ɳ�?܇/?����??��܇|3?�ǩI/��??-��?��??��܇|(CRCD�S?��3��1|)
        }
    }
    if( ( timeout == 0 ) || ( status & ( 1 << 2 ) ) ) //?��܇|3?�ǩI
    {
        errorstatus = SD_CMD_RSP_TIMEOUT; //�שI?��?��2?��?2.0??��Y?��,?��??2??��3?����?����?��??1�{??��
        SDIO_ClearFlag( SDIO_FLAG_CTIMEOUT );           //??3y?����??��܇|3?�ǩI�I��??
        return errorstatus;
    }
    if( status & 1 << 6 )                 //3��1|?܇��?��??��܇|
    {
        errorstatus = SD_OK;
        SDIO_ClearFlag( SDIO_FLAG_CMDREND );            //??3y?��܇|�I��??
    }
    return errorstatus;
}
//?��2��R1?��܇|��?�ɳ�?܇�G����?
//cmd:�שI?��?����?
//�{��???��:sd?���ɳ�?܇??
SD_Error CmdResp1Error( u8 cmd )
{
    u32 status;
    while( 1 )
    {
        status = SDIO->STA;
        if( status & ( ( 1 << 0 ) | ( 1 << 2 ) | ( 1 << 6 ) ) )
        {
            break;    //CRC�ɳ�?܇/?����??��܇|3?�ǩI/��??-��?��??��܇|(CRCD�S?��3��1|)
        }
    }
    if( SDIO_GetFlagStatus( SDIO_FLAG_CTIMEOUT ) != RESET )               //?��܇|3?�ǩI
    {
        SDIO_ClearFlag( SDIO_FLAG_CTIMEOUT );               //??3y?����??��܇|3?�ǩI�I��??
        return SD_CMD_RSP_TIMEOUT;
    }
    if( SDIO_GetFlagStatus( SDIO_FLAG_CCRCFAIL ) != RESET )               //CRC�ɳ�?܇
    {
        SDIO_ClearFlag( SDIO_FLAG_CCRCFAIL );               //??3y�I��??
        return SD_CMD_CRC_FAIL;
    }
    if( SDIO->RESPCMD != cmd )
    {
        return SD_ILLEGAL_CMD;    //?����?2??��??
    }
    SDIO_ClearFlag( SDIO_STATIC_FLAGS ); //??3y?��܇D�I��??
    return ( SD_Error )( SDIO->RESP1 & SD_OCR_ERRORBITS ); //�{��???��?��܇|
}
//?��2��R3?��܇|��?�ɳ�?܇�G����?
//�{��???��:�ɳ�?܇�G����?
SD_Error CmdResp3Error( void )
{
    u32 status;
    while( 1 )
    {
        status = SDIO->STA;
        if( status & ( ( 1 << 0 ) | ( 1 << 2 ) | ( 1 << 6 ) ) )
        {
            break;    //CRC�ɳ�?܇/?����??��܇|3?�ǩI/��??-��?��??��܇|(CRCD�S?��3��1|)
        }
    }
    if( SDIO_GetFlagStatus( SDIO_FLAG_CTIMEOUT ) != RESET )               //?��܇|3?�ǩI
    {
        SDIO_ClearFlag( SDIO_FLAG_CTIMEOUT );       //??3y?����??��܇|3?�ǩI�I��??
        return SD_CMD_RSP_TIMEOUT;
    }
    SDIO_ClearFlag( SDIO_STATIC_FLAGS ); //??3y?��܇D�I��??
    return SD_OK;
}
//?��2��R2?��܇|��?�ɳ�?܇�G����?
//�{��???��:�ɳ�?܇�G����?
SD_Error CmdResp2Error( void )
{
    SD_Error errorstatus = SD_OK;
    u32 status;
    u32 timeout = SDIO_CMD0TIMEOUT;
    while( timeout-- )
    {
        status = SDIO->STA;
        if( status & ( ( 1 << 0 ) | ( 1 << 2 ) | ( 1 << 6 ) ) )
        {
            break;    //CRC�ɳ�?܇/?����??��܇|3?�ǩI/��??-��?��??��܇|(CRCD�S?��3��1|)
        }
    }
    if( ( timeout == 0 ) || ( status & ( 1 << 2 ) ) ) //?��܇|3?�ǩI
    {
        errorstatus = SD_CMD_RSP_TIMEOUT;
        SDIO_ClearFlag( SDIO_FLAG_CTIMEOUT );       //??3y?����??��܇|3?�ǩI�I��??
        return errorstatus;
    }
    if( SDIO_GetFlagStatus( SDIO_FLAG_CCRCFAIL ) != RESET )                   //CRC�ɳ�?܇
    {
        errorstatus = SD_CMD_CRC_FAIL;
        SDIO_ClearFlag( SDIO_FLAG_CCRCFAIL );   //??3y?��܇|�I��??
    }
    SDIO_ClearFlag( SDIO_STATIC_FLAGS ); //??3y?��܇D�I��??
    return errorstatus;
}
//?��2��R6?��܇|��?�ɳ�?܇�G����?
//cmd:???���{�S?����??����?
//prca:?���{��??��?RCA��??�{
//�{��???��:�ɳ�?܇�G����?
SD_Error CmdResp6Error( u8 cmd, u16* prca )
{
    SD_Error errorstatus = SD_OK;
    u32 status;
    u32 rspr1;
    while( 1 )
    {
        status = SDIO->STA;
        if( status & ( ( 1 << 0 ) | ( 1 << 2 ) | ( 1 << 6 ) ) )
        {
            break;    //CRC�ɳ�?܇/?����??��܇|3?�ǩI/��??-��?��??��܇|(CRCD�S?��3��1|)
        }
    }
    if( SDIO_GetFlagStatus( SDIO_FLAG_CTIMEOUT ) != RESET )               //?��܇|3?�ǩI
    {
        SDIO_ClearFlag( SDIO_FLAG_CTIMEOUT );       //??3y?����??��܇|3?�ǩI�I��??
        return SD_CMD_RSP_TIMEOUT;
    }
    if( SDIO_GetFlagStatus( SDIO_FLAG_CCRCFAIL ) != RESET )                   //CRC�ɳ�?܇
    {
        SDIO_ClearFlag( SDIO_FLAG_CCRCFAIL );               //??3y?��܇|�I��??
        return SD_CMD_CRC_FAIL;
    }
    if( SDIO->RESPCMD != cmd )            //?D??��?�{??��܇|cmd?����?
    {
        return SD_ILLEGAL_CMD;
    }
    SDIO_ClearFlag( SDIO_STATIC_FLAGS ); //??3y?��܇D�I��??
    rspr1 = SDIO->RESP1;                  //��?��??��܇|
    if( SD_ALLZERO == ( rspr1 & ( SD_R6_GENERAL_UNKNOWN_ERROR | SD_R6_ILLEGAL_CMD | SD_R6_COM_CRC_FAILED ) ) )
    {
        *prca = ( u16 )( rspr1 >> 16 ); //܇����?16??��?��?,rca
        return errorstatus;
    }
    if( rspr1 & SD_R6_GENERAL_UNKNOWN_ERROR )
    {
        return SD_GENERAL_UNKNOWN_ERROR;
    }
    if( rspr1 & SD_R6_ILLEGAL_CMD )
    {
        return SD_ILLEGAL_CMD;
    }
    if( rspr1 & SD_R6_COM_CRC_FAILED )
    {
        return SD_COM_CRC_FAILED;
    }
    return errorstatus;
}

//SDIO��1?��?��G��???�S��?
//enx:0,2?��1?��;1,��1?��;
//�{��???��:�ɳ�?܇�G����?
SD_Error SDEnWideBus( u8 enx )
{
    SD_Error errorstatus = SD_OK;
    u32 scr[2] = {0, 0};
    u8 arg = 0X00;
    if( enx )
    {
        arg = 0X02;
    }
    else
    {
        arg = 0X00;
    }
    if( SDIO->RESP1 & SD_CARD_LOCKED )
    {
        return SD_LOCK_UNLOCK_FAILED;    //SD?����|܇�~LOCKED�G����?
    }
    errorstatus = FindSCR( RCA, scr );                //��?��?SCR??��??����y?Y
    if( errorstatus != SD_OK )
    {
        return errorstatus;
    }
    if( ( scr[1]&SD_WIDE_BUS_SUPPORT ) != SD_ALLZERO ) //?��3??��G��??
    {
        SDIO_CmdInitStructure.SDIO_Argument = ( uint32_t ) RCA << 16; //�{�S?��CMD55+RCA,?��?��܇|
        SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_CMD;
        SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
        SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
        SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
        SDIO_SendCommand( &SDIO_CmdInitStructure );
        errorstatus = CmdResp1Error( SD_CMD_APP_CMD );
        if( errorstatus != SD_OK )
        {
            return errorstatus;
        }
        SDIO_CmdInitStructure.SDIO_Argument = arg;//�{�S?��ACMD6,?��?��܇|,2?��y:10,4??;00,1??.
        SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_SD_SET_BUSWIDTH;
        SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
        SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
        SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
        SDIO_SendCommand( &SDIO_CmdInitStructure );
        errorstatus = CmdResp1Error( SD_CMD_APP_SD_SET_BUSWIDTH );
        return errorstatus;
    }
    else
    {
        return SD_REQUEST_NOT_APPLICABLE;    //2??��3??��G��??����??
    }
}
//?��2��?����?�{??y?�~?��DDD��2���G��
//pstatus:�שI?���G����?.
//�{��???��:�ɳ�?܇���~??
SD_Error IsCardProgramming( u8* pstatus )
{
    vu32 respR1 = 0, status = 0;
    SDIO_CmdInitStructure.SDIO_Argument = ( uint32_t ) RCA << 16; //?��?��??��??�{2?��y
    SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SEND_STATUS;//�{�S?��CMD13
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand( &SDIO_CmdInitStructure );
    status = SDIO->STA;
    while( !( status & ( ( 1 << 0 ) | ( 1 << 6 ) | ( 1 << 2 ) ) ) )
    {
        status = SDIO->STA;    //������y2���G������3��
    }
    if( SDIO_GetFlagStatus( SDIO_FLAG_CCRCFAIL ) != RESET )       //CRC?��2a�ǡҡ���
    {
        SDIO_ClearFlag( SDIO_FLAG_CCRCFAIL ); //??3y�ɳ�?܇�I��??
        return SD_CMD_CRC_FAIL;
    }
    if( SDIO_GetFlagStatus( SDIO_FLAG_CTIMEOUT ) != RESET )       //?����?3?�ǩI
    {
        SDIO_ClearFlag( SDIO_FLAG_CTIMEOUT );       //??3y�ɳ�?܇�I��??
        return SD_CMD_RSP_TIMEOUT;
    }
    if( SDIO->RESPCMD != SD_CMD_SEND_STATUS )
    {
        return SD_ILLEGAL_CMD;
    }
    SDIO_ClearFlag( SDIO_STATIC_FLAGS ); //??3y?��܇D�I��??
    respR1 = SDIO->RESP1;
    *pstatus = ( u8 )( ( respR1 >> 9 ) & 0x0000000F );
    return SD_OK;
}
//?����?�שI?��?���G����?
//pcardstatus:?���G����?
//�{��???��:�ɳ�?܇���~??
SD_Error SD_SendStatus( uint32_t* pcardstatus )
{
    SD_Error errorstatus = SD_OK;
    if( pcardstatus == NULL )
    {
        errorstatus = SD_INVALID_PARAMETER;
        return errorstatus;
    }
    SDIO_CmdInitStructure.SDIO_Argument = ( uint32_t ) RCA << 16; //�{�S?��CMD13,?��?��܇|
    SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SEND_STATUS;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand( &SDIO_CmdInitStructure );
    errorstatus = CmdResp1Error( SD_CMD_SEND_STATUS ); //2��?��?��܇|�G����?
    if( errorstatus != SD_OK )
    {
        return errorstatus;
    }
    *pcardstatus = SDIO->RESP1; //?����??��܇|?��
    return errorstatus;
}
//�{��??SD?����?�G����?
//�{��???��:SD?���G����?
SDCardState SD_GetState( void )
{
    u32 resp1 = 0;
    if( SD_SendStatus( &resp1 ) != SD_OK )
    {
        return SD_CARD_ERROR;
    }
    else
    {
        return ( SDCardState )( ( resp1 >> 9 ) & 0x0F );
    }
}
//2��?��SD?����?SCR??��??��?��
//rca:?��?��??��??�{
//pscr:��y?Y?o��???(��?�ɨSSCR?�~��Y)
//�{��???��:�ɳ�?܇�G����?
SD_Error FindSCR( u16 rca, u32* pscr )
{
    u32 index = 0;
    SD_Error errorstatus = SD_OK;
    u32 tempscr[2] = {0, 0};
    SDIO_CmdInitStructure.SDIO_Argument = ( uint32_t )8;   //�{�S?��CMD16,?��?��܇|,����??Block Size?a8�G??�~
    SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_BLOCKLEN; //  cmd16
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;  //r1
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand( &SDIO_CmdInitStructure );
    errorstatus = CmdResp1Error( SD_CMD_SET_BLOCKLEN );
    if( errorstatus != SD_OK )
    {
        return errorstatus;
    }
    SDIO_CmdInitStructure.SDIO_Argument = ( uint32_t ) RCA << 16;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_CMD;//�{�S?��CMD55,?��?��܇|
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand( &SDIO_CmdInitStructure );
    errorstatus = CmdResp1Error( SD_CMD_APP_CMD );
    if( errorstatus != SD_OK )
    {
        return errorstatus;
    }
    SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
    SDIO_DataInitStructure.SDIO_DataLength = 8;  //8??�G??�~3�N?��,block?a8�G??�~,SD?����?SDIO.
    SDIO_DataInitStructure.SDIO_DataBlockSize = SDIO_DataBlockSize_8b  ;  //?�ȡ�܇D?8byte
    SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToSDIO;
    SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
    SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
    SDIO_DataConfig( &SDIO_DataInitStructure );
    SDIO_CmdInitStructure.SDIO_Argument = 0x0;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SD_APP_SEND_SCR; //�{�S?��ACMD51,?��?��܇|,2?��y?a0
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;  //r1
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand( &SDIO_CmdInitStructure );
    errorstatus = CmdResp1Error( SD_CMD_SD_APP_SEND_SCR );
    if( errorstatus != SD_OK )
    {
        return errorstatus;
    }
    while( !( SDIO->STA & ( SDIO_FLAG_RXOVERR | SDIO_FLAG_DCRCFAIL | SDIO_FLAG_DTIMEOUT | SDIO_FLAG_DBCKEND | SDIO_FLAG_STBITERR ) ) )
    {
        if( SDIO_GetFlagStatus( SDIO_FLAG_RXDAVL ) != RESET ) //?܇��?FIFO��y?Y?��܇?
        {
            *( tempscr + index ) = SDIO->FIFO; //?����?FIFO?�~��Y
            index++;
            if( index >= 2 )
            {
                break;
            }
        }
    }
    if( SDIO_GetFlagStatus( SDIO_FLAG_DTIMEOUT ) != RESET )   //��y?Y3?�ǩI�ɳ�?܇
    {
        SDIO_ClearFlag( SDIO_FLAG_DTIMEOUT );   //??�ɳ�?܇�I��??
        return SD_DATA_TIMEOUT;
    }
    else if( SDIO_GetFlagStatus( SDIO_FLAG_DCRCFAIL ) != RESET ) //��y?Y?��CRC�ɳ�?܇
    {
        SDIO_ClearFlag( SDIO_FLAG_DCRCFAIL );       //??�ɳ�?܇�I��??
        return SD_DATA_CRC_FAIL;
    }
    else if( SDIO_GetFlagStatus( SDIO_FLAG_RXOVERR ) != RESET ) //?܇��?fifo��?��?�ɳ�?܇
    {
        SDIO_ClearFlag( SDIO_FLAG_RXOVERR );    //??�ɳ�?܇�I��??
        return SD_RX_OVERRUN;
    }
    else if( SDIO_GetFlagStatus( SDIO_FLAG_STBITERR ) != RESET )  //?܇��??e��???�ɳ�?܇
    {
        SDIO_ClearFlag( SDIO_FLAG_STBITERR ); //??�ɳ�?܇�I��??
        return SD_START_BIT_ERR;
    }
    SDIO_ClearFlag( SDIO_STATIC_FLAGS ); //??3y?��܇D�I��??
    //��?��y?Y?3D������8???a�ר�??��11y�ǡ�.
    *( pscr + 1 ) = ( ( tempscr[0] & SD_0TO7BITS ) << 24 ) | ( ( tempscr[0] & SD_8TO15BITS ) << 8 ) | ( ( tempscr[0] & SD_16TO23BITS ) >> 8 ) | ( ( tempscr[0] & SD_24TO31BITS ) >> 24 );
    *( pscr ) = ( ( tempscr[1] & SD_0TO7BITS ) << 24 ) | ( ( tempscr[1] & SD_8TO15BITS ) << 8 ) | ( ( tempscr[1] & SD_16TO23BITS ) >> 8 ) | ( ( tempscr[1] & SD_24TO31BITS ) >> 24 );
    return errorstatus;
}
//��?��?NumberOfBytes��?2?a�רG��???��y.
//NumberOfBytes:�G??�~��y.
//�{��???��:��?2?a�רG��???��y?��
u8 convert_from_bytes_to_power_of_two( u16 NumberOfBytes )
{
    u8 count = 0;
    while( NumberOfBytes != 1 )
    {
        NumberOfBytes >>= 1;
        count++;
    }
    return count;
}

//????SDIO DMA
//mbuf:��?�ɨS?����??�{
//bufsize:��?��?��y?Y��?
//dir:�{??��;DMA_DIR_MemoryToPeripheral  ��?�ɨS?��-->SDIO(D����y?Y);DMA_DIR_PeripheralToMemory SDIO-->��?�ɨS?��(?����y?Y);
void SD_DMA_Config( u32* mbuf, u32 bufsize, u32 dir )
{
    DMA_InitTypeDef  DMA_InitStructure;
    while( DMA_GetCmdStatus( DMA2_Stream3 ) != DISABLE ) {} //������yDMA?��????
    DMA_DeInit( DMA2_Stream3 ); //???????��??stream3��?��??��܇D?D??�I��??
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;  //�ࡰ�׽�????
    DMA_InitStructure.DMA_PeripheralBaseAddr = ( u32 )&SDIO->FIFO; //DMA��a������??�{
    DMA_InitStructure.DMA_Memory0BaseAddr = ( u32 )mbuf; //DMA ��?�ɨS?��0��??�{
    DMA_InitStructure.DMA_DIR = dir;//��?�ɨS?����?��a����?�S��?
    DMA_InitStructure.DMA_BufferSize = 0;//��y?Y��?��?��?
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//��a�����{???��??�S��?
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//��?�ɨS?��??��??�S��?
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;//��a������y?Y3�N?��:32??
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;//��?�ɨS?����y?Y3�N?��:32??
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// ��1܇???�ࡰ?�S��?
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;//�G???܇??��??
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;   //FIFO��1?��
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;//��?FIFO
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_INC4;//��a������?�{�S4��?��?��?
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_INC4;//��?�ɨS?����?�{�S4��?��?��?
    DMA_Init( DMA2_Stream3, &DMA_InitStructure ); //3?��??��DMA Stream
    DMA_FlowControllerConfig( DMA2_Stream3, DMA_FlowCtrl_Peripheral ); //��a�����֡�????
    DMA_Cmd( DMA2_Stream3, ENABLE ); //?a??DMA��?��?
}


//?��SD?��
//buf:?����y?Y?o��???
//sector:����??��??�{
//cnt:����????��y
//�{��???��:�ɳ�?܇�G����?;0,?y3�S;????,�ɳ�?܇���~??;
u8 SD_ReadDisk( u8* buf, u32 sector, u8 cnt )
{
    u8 sta = SD_OK;
    long long lsector = sector;
    u8 n;
    lsector <<= 9;
    if( ( u32 )buf % 4 != 0 )
    {
        for( n = 0; n < cnt; n++ )
        {
            sta = SD_ReadBlock( SDIO_DATA_BUFFER, lsector + 512 * n, 512 ); //�ר�??sector��??��2���G��
            memcpy( buf, SDIO_DATA_BUFFER, 512 );
            buf += 512;
        }
    }
    else
    {
        if( cnt == 1 )
        {
            sta = SD_ReadBlock( buf, lsector, 512 );    //�ר�??sector��??��2���G��
        }
        else
        {
            sta = SD_ReadMultiBlocks( buf, lsector, 512, cnt );    //?��??sector
        }
    }
    return sta;
}
//D��SD?��
//buf:D����y?Y?o��???
//sector:����??��??�{
//cnt:����????��y
//�{��???��:�ɳ�?܇�G����?;0,?y3�S;????,�ɳ�?܇���~??;
u8 SD_WriteDisk( u8* buf, u32 sector, u8 cnt )
{
    u8 sta = SD_OK;
    u8 n;
    long long lsector = sector;
    lsector <<= 9;
    if( ( u32 )buf % 4 != 0 )
    {
        for( n = 0; n < cnt; n++ )
        {
            memcpy( SDIO_DATA_BUFFER, buf, 512 );
            sta = SD_WriteBlock( SDIO_DATA_BUFFER, lsector + 512 * n, 512 ); //�ר�??sector��?D��2���G��
            buf += 512;
        }
    }
    else
    {
        if( cnt == 1 )
        {
            sta = SD_WriteBlock( buf, lsector, 512 );    //�ר�??sector��?D��2���G��
        }
        else
        {
            sta = SD_WriteMultiBlocks( buf, lsector, 512, cnt );    //?��??sector
        }
    }
    return sta;
}








