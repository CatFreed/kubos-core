#include "kubos-core/modules/fatfs/stm32f4/diskio.h"
#include "stm32cubef4/stm32f4xx_hal.h"
#include "stm32cubef4/stm32f4xx_hal_sd.h"
#include "kubos-hal/gpio.h"

static SD_HandleTypeDef sd_handle;
/* Status of SDCARD */
static volatile DSTATUS Stat = STA_NOINIT;

#define SD_BLOCK_SIZE 512

static void sd_msp_init(void)
{
    GPIO_InitTypeDef GPIO_Init_Structure;

	//sd_handle.Instance = NULL;

	SET_BIT(RCC->AHB1ENR,
		STM32F4_PIN_AHB1ENR_BIT(PC6) | STM32F4_PIN_AHB1ENR_BIT(PC7));

    /* Enable SDIO clock */
	__HAL_RCC_SDIO_CLK_ENABLE();
	__SDIO_CLK_ENABLE();

    __GPIOC_CLK_ENABLE();
    __GPIOD_CLK_ENABLE();
    __GPIOA_CLK_ENABLE();
    __GPIOB_CLK_ENABLE();

    // 4 pins for 4-bit
    // PC8  -> D0
    // PC9  -> D1
    // PC10 -> D2
    // PC11 -> D3
	// PC12 -> CLK
    GPIO_Init_Structure.Mode = GPIO_MODE_AF_PP;
    GPIO_Init_Structure.Pull = GPIO_PULLUP;
    GPIO_Init_Structure.Speed = GPIO_SPEED_HIGH;
    GPIO_Init_Structure.Alternate = GPIO_AF12_SDIO;
    GPIO_Init_Structure.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_12;
    HAL_GPIO_Init(GPIOC, &GPIO_Init_Structure);

    GPIO_Init_Structure.Pin = GPIO_PIN_11;
    GPIO_Init_Structure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_Init_Structure);


    // PD2 -> CMD
    GPIO_Init_Structure.Pin = GPIO_PIN_2;
    GPIO_Init_Structure.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOD, &GPIO_Init_Structure);

    // PB8 -> D4
    // PB9 -> D5
    GPIO_Init_Structure.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    HAL_GPIO_Init(GPIOB, &GPIO_Init_Structure);


	// Detect pin
    // PA8 -> DETECT
    GPIO_Init_Structure.Pin = GPIO_PIN_8;
    GPIO_Init_Structure.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(GPIOA, &GPIO_Init_Structure);



	/* NVIC configuration for SDIO interrupts */
	HAL_NVIC_SetPriority(SDIO_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(SDIO_IRQn);
}

void HAL_SD_MspInit(SD_HandleTypeDef * hsd)
{

}

void HAL_SD_MspDeInit(SD_HandleTypeDef * hsd)
{
    HAL_NVIC_DisableIRQ(SDIO_IRQn);
    __SDIO_CLK_DISABLE();
}

DSTATUS sd_is_present()
{
    return (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == 0);
}

DSTATUS disk_initialize (BYTE pdrv)
{
    // check if sd card is present
    if (!sd_is_present())
    {
        return STA_NOINIT;
    }

    // have we already done this?
    if (sd_handle.Instance != NULL)
    {
        return RES_OK;
    }
    HAL_SD_CardInfoTypedef card_info;

    sd_msp_init();

    sd_handle.Instance = SDIO;
    sd_handle.Init.ClockEdge           = SDIO_CLOCK_EDGE_RISING;
    sd_handle.Init.ClockBypass         = SDIO_CLOCK_BYPASS_DISABLE;
    sd_handle.Init.ClockPowerSave      = SDIO_CLOCK_POWER_SAVE_DISABLE;
    sd_handle.Init.BusWide             = SDIO_BUS_WIDE_1B;
    sd_handle.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
    sd_handle.Init.ClockDiv            = SDIO_TRANSFER_CLK_DIV;

    DSTATUS ret = SD_OK;

    for (int tries = 50; tries > 0; tries--)
    {
        if ((ret = HAL_SD_Init(&sd_handle, &card_info)) == SD_OK)
        {
            break;
        }
		vTaskDelay(5);
    }
    if (ret != SD_OK)
    {
      return STA_NOINIT;
    }

    // configure the SD bus width for wide operation
    if (HAL_SD_WideBusOperation_Config(&sd_handle, SDIO_BUS_WIDE_4B) != SD_OK) {
        HAL_SD_DeInit(&sd_handle);
        return STA_NOINIT;
    }

    return RES_OK;
}



DSTATUS disk_status (BYTE pdrv)
{
	if (sd_handle.Instance == NULL)
	{
		return RES_ERROR;
	}
    Stat = STA_NOINIT;
    HAL_SD_CardStatusTypedef card_status;
    if (HAL_SD_GetCardStatus(&sd_handle, &card_status) == SD_OK)
    {
        Stat &= ~STA_NOINIT;
    }
    else
    {
        Stat |= STA_NOINIT;
    }

    return Stat;
}

DRESULT disk_read (BYTE pdrv, BYTE*buff, DWORD sector, UINT count)
{
	if (sd_handle.Instance == NULL)
	{
		return RES_ERROR;
	}
    uint64_t block_addr = sector * SD_BLOCK_SIZE;
    if (HAL_SD_ReadBlocks(&sd_handle, (uint32_t*)buff, block_addr, SD_BLOCK_SIZE, count) != SD_OK)
    {
        return RES_ERROR;
    }
    return RES_OK;
}

DRESULT disk_write (BYTE pdrv, const BYTE* buff, DWORD sector, UINT count)
{
	if (sd_handle.Instance == NULL)
	{
		return RES_ERROR;
	}
    uint64_t block_addr = sector * SD_BLOCK_SIZE;
    if (HAL_SD_WriteBlocks(&sd_handle, (uint32_t*)buff, block_addr, SD_BLOCK_SIZE, count) != SD_OK)
    {
        return RES_ERROR;
    }
    return RES_OK;
}

DRESULT disk_ioctl (BYTE pdrv, BYTE cmd, void* buff)
{
    DRESULT res = RES_ERROR;
	HAL_SD_CardInfoTypedef card_info;

	/* Check if init OK */
	if (sd_handle.Instance != NULL)
    {
    	switch (cmd) {
    		/* Make sure that no pending write process */
    		case CTRL_SYNC :
    			res = RES_OK;
    			break;

    		/* Size in bytes for single sector */
    		case GET_SECTOR_SIZE:
    			*(WORD *)buff = SD_BLOCK_SIZE;
    			res = RES_OK;
    			break;

    		/* Get number of sectors on the disk (DWORD) */
    		case GET_SECTOR_COUNT :
                HAL_SD_Get_CardInfo(&sd_handle, &card_info);
    			*(DWORD *)buff = card_info.CardCapacity / SD_BLOCK_SIZE;
    			res = RES_OK;
    			break;

    		/* Get erase block size in unit of sector (DWORD) */
    		case GET_BLOCK_SIZE :
    			*(DWORD*)buff = SD_BLOCK_SIZE;
    			break;

    		default:
    			res = RES_PARERR;
    	}
    }

	return res;
}

DWORD get_fattime (void)
{
    return	  ((DWORD)(2016 - 1980) << 25)	/* Year 2013 */
        | ((DWORD)7 << 21)				/* Month 7 */
        | ((DWORD)28 << 16)				/* Mday 28 */
        | ((DWORD)0 << 11)				/* Hour 0 */
        | ((DWORD)0 << 5)				/* Min 0 */
        | ((DWORD)0 >> 1);				/* Sec 0 */
}


// void SDIO_IRQHandler(void)
// {
// 	HAL_SD_IRQHandler(&sd_handle);
// }
