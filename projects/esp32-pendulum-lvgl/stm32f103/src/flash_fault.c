#include "main.h"
#include "flash_fault.h"

#ifdef FLASH_FAULT_ENABLE

/* 计算校验和 */
static uint32_t Flash_CalcChecksum(Flash_FaultRecord_t *record)
{
    uint32_t sum = 0;
    uint8_t *p = (uint8_t *)record;
    /* 计算除checksum字段外的所有字节和 */
    for (uint32_t i = 0; i < sizeof(Flash_FaultRecord_t) - sizeof(uint32_t); i++)
    {
        sum += p[i];
    }
    return sum;
}

/**
  * @brief  读取Flash中的故障记录
  * @param  record: 故障记录结构体指针
  * @retval 1=读取成功, 0=无有效记录
  */
uint8_t Flash_ReadFault(Flash_FaultRecord_t *record)
{
    Flash_FaultRecord_t *flash = (Flash_FaultRecord_t *)FLASH_FAULT_PAGE_ADDR;

    /* 检查魔数标记 */
    if (flash->magic != FAULT_MAGIC_VALID)
    {
        return 0;  /* 无有效记录 */
    }

    /* 检查校验和 */
    if (flash->checksum != Flash_CalcChecksum((Flash_FaultRecord_t *)FLASH_FAULT_PAGE_ADDR))
    {
        return 0;  /* 数据损坏 */
    }

    /* 复制数据 */
    *record = *flash;
    return 1;
}

/**
  * @brief  保存故障记录到Flash
  * @param  record: 故障记录结构体指针
  * @retval 1=保存成功, 0=保存失败
  */
uint8_t Flash_SaveFault(Flash_FaultRecord_t *record)
{
    HAL_StatusTypeDef status;
    uint32_t page_error = 0;

    /* 设置魔数和校验和 */
    record->magic = FAULT_MAGIC_VALID;
    record->checksum = Flash_CalcChecksum(record);

    /* 解锁Flash */
    HAL_FLASH_Unlock();

    /* 擦除最后一页 */
    FLASH_EraseInitTypeDef erase_init;
    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.PageAddress = FLASH_FAULT_PAGE_ADDR;
    erase_init.NbPages = 1;

    status = HAL_FLASHEx_Erase(&erase_init, &page_error);
    if (status != HAL_OK)
    {
        HAL_FLASH_Lock();
        return 0;
    }

    /* 按字写入数据 */
    uint32_t *src = (uint32_t *)record;
    uint32_t addr = FLASH_FAULT_PAGE_ADDR;

    for (uint32_t i = 0; i < sizeof(Flash_FaultRecord_t) / 4; i++)
    {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, src[i]);
        if (status != HAL_OK)
        {
            HAL_FLASH_Lock();
            return 0;
        }
        addr += 4;
    }

    /* 锁定Flash */
    HAL_FLASH_Lock();
    return 1;
}

/**
  * @brief  清除Flash中的故障记录
  * @retval 1=清除成功, 0=清除失败
  */
uint8_t Flash_ClearFault(void)
{
    HAL_StatusTypeDef status;
    uint32_t page_error = 0;

    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase_init;
    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.PageAddress = FLASH_FAULT_PAGE_ADDR;
    erase_init.NbPages = 1;

    status = HAL_FLASHEx_Erase(&erase_init, &page_error);

    HAL_FLASH_Lock();

    return (status == HAL_OK) ? 1 : 0;
}

#endif /* FLASH_FAULT_ENABLE */
