/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2014        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "diskio.h"		/* FatFs lower layer API */
#include "sdio_sdcard.h"
#include "malloc.h"

#define SECTOR_SIZE		512	/* SD?¡§¨¦¨¨??¡ä¨®D?¡À?D??a512 */


/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/
DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber (0..) */
)
{
	u8 res=0;	    
	switch(pdrv)
	{
		case SD_CARD://SD?¡§
			res=SD_Init();//SD?¡§3?¨º??¡¥ 
  			break;
		case MMC:
                  
			
 			break;
		default:
			res=1; 
	}		 
	if(res)return  STA_NOINIT;
	else return 0; //3?¨º??¡¥3¨¦1|
}  

//??¦Ì?¡ä??¨¬¡Á¡ä¨¬?
DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber (0..) */
)
{ 
	return 0;
} 

//?¨¢¨¦¨¨??
//drv:¡ä??¨¬¡À¨¤o?0~9
//*buff:¨ºy?Y?¨®¨º??o3?¨º¡Á¦Ì??¡¤
//sector:¨¦¨¨??¦Ì??¡¤
//count:D¨¨¨°a?¨¢¨¨?¦Ì?¨¦¨¨??¨ºy
DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Sector address (LBA) */
	UINT count		/* Number of sectors to read (1..128) */
)
{
	u8 res=0; 
    if (!count)return RES_PARERR;//count2??¨¹¦Ì¨¨¨®¨²0¡ê?¡¤??¨°¡¤¦Ì??2?¨ºy¡ä¨ª?¨®		 	 
	switch(pdrv)
	{
		case SD_CARD://SD?¡§
			res=SD_ReadDisk(buff,sector,count);	 
			while(res)//?¨¢3?¡ä¨ª
			{
				SD_Init();	//??D?3?¨º??¡¥SD?¡§
				res=SD_ReadDisk(buff,sector,count);	
				//printf("sd rd error:%d\r\n",res);
			}
			break;
		case USB:
                  
			return RES_OK;
			break;
		default:
			res=1; 
	}
   //¡ä|¨¤¨ª¡¤¦Ì???¦Ì¡ê???SPI_SD_driver.c¦Ì?¡¤¦Ì???¦Ì¡Áa3¨¦ff.c¦Ì?¡¤¦Ì???¦Ì
    if(res==0x00)return RES_OK;	 
    else return RES_ERROR;	   
}

//D¡ä¨¦¨¨??
//drv:¡ä??¨¬¡À¨¤o?0~9
//*buff:¡¤¡é?¨ª¨ºy?Y¨º¡Á¦Ì??¡¤
//sector:¨¦¨¨??¦Ì??¡¤
//count:D¨¨¨°aD¡ä¨¨?¦Ì?¨¦¨¨??¨ºy
#if _USE_WRITE
DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber (0..) */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Sector address (LBA) */
	UINT count			/* Number of sectors to write (1..128) */
)
{
	u8 res=0;  
    if (!count)return RES_PARERR;//count2??¨¹¦Ì¨¨¨®¨²0¡ê?¡¤??¨°¡¤¦Ì??2?¨ºy¡ä¨ª?¨®		 	 
	switch(pdrv)
	{
		case SD_CARD://SD?¡§
			res=SD_WriteDisk((u8*)buff,sector,count);
			while(res)//D¡ä3?¡ä¨ª
			{
				SD_Init();	//??D?3?¨º??¡¥SD?¡§
				res=SD_WriteDisk((u8*)buff,sector,count);	
				//printf("sd wr error:%d\r\n",res);
			}
			break;
		case USB:
			return RES_OK;
			break;
		default:
			res=1; 
	}
    //¡ä|¨¤¨ª¡¤¦Ì???¦Ì¡ê???SPI_SD_driver.c¦Ì?¡¤¦Ì???¦Ì¡Áa3¨¦ff.c¦Ì?¡¤¦Ì???¦Ì
    if(res == 0x00)return RES_OK;	 
    else return RES_ERROR;	
}
#endif


//????¡À¨ª2?¨ºy¦Ì???¦Ì?
 //drv:¡ä??¨¬¡À¨¤o?0~9
 //ctrl:????¡ä¨²??
 //*buff:¡¤¡é?¨ª/?¨®¨º??o3???????
#if _USE_IOCTL
DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	DRESULT res;						  			     
	if(pdrv==SD_CARD)//SD?¡§
	{
	    switch(cmd)
	    {
		    case CTRL_SYNC:
				res = RES_OK; 
		        break;	 
		    case GET_SECTOR_SIZE:
				*(DWORD*)buff = 512; 
		        res = RES_OK;
		        break;	 
		    case GET_BLOCK_SIZE:
				*(WORD*)buff = SDCardInfo.CardBlockSize;
		        res = RES_OK;
		        break;	 
		    case GET_SECTOR_COUNT:
		        *(DWORD*)buff = SDCardInfo.CardCapacity/512;
		        res = RES_OK;
		        break;
		    default:
		        res = RES_PARERR;
		        break;
	    }
	}else res=RES_ERROR;//????¦Ì?2??¡ì3?
    return res;
}
#endif
//??¦Ì?¨º¡À??
//User defined function to give a current time to fatfs module      */
//31-25: Year(0-127 org.1980), 24-21: Month(1-12), 20-16: Day(1-31) */                                                                                                                                                                                                                                          
//15-11: Hour(0-23), 10-5: Minute(0-59), 4-0: Second(0-29 *2) */                                                                                                                                                                                                                                                
DWORD get_fattime (void)
{				 
	return 0;
}			 
//?¡¥¨¬?¡¤????¨²¡ä?
void *ff_memalloc (UINT size)			
{
	return (void*)mymalloc(size);
}
//¨º¨ª¡¤??¨²¡ä?
void ff_memfree (void* mf)		 
{
	myfree(mf);
}