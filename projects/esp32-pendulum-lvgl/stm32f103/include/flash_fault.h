#ifndef __FLASH_FAULT_H
#define __FLASH_FAULT_H

#include <stdint.h>

/* Flash故障记录功能开关：取消注释下面这行即可启用 */
//#define FLASH_FAULT_ENABLE

/* STM32F103C8T6 最后一页地址（64KB Flash，页大小1KB） */
#define FLASH_FAULT_PAGE_ADDR   0x0800FC00
#define FLASH_FAULT_PAGE_SIZE   1024

/* 故障记录结构体 */
typedef struct {
    uint32_t magic;           /* 魔数标记，用于判断是否有效数据 */
    uint32_t fall_count;      /* 倒下总次数 */
    uint32_t last_angle;      /* 最后一次倒下时的角度 */
    uint32_t last_reason;     /* 最后一次倒下的原因 */
    uint32_t total_power_on;  /* 总上电次数 */
    uint32_t checksum;        /* 校验和 */
} Flash_FaultRecord_t;

/* 魔数标记 */
#define FAULT_MAGIC_VALID   0x4641554C  /* "FAUL" */
#define FAULT_MAGIC_EMPTY   0xFFFFFFFF  /* Flash擦除状态 */

#ifdef FLASH_FAULT_ENABLE
/**
  * @brief  读取Flash中的故障记录
  * @param  record: 故障记录结构体指针
  * @retval 1=读取成功, 0=无有效记录
  */
uint8_t Flash_ReadFault(Flash_FaultRecord_t *record);

/**
  * @brief  保存故障记录到Flash
  * @param  record: 故障记录结构体指针
  * @retval 1=保存成功, 0=保存失败
  */
uint8_t Flash_SaveFault(Flash_FaultRecord_t *record);

/**
  * @brief  清除Flash中的故障记录
  * @retval 1=清除成功, 0=清除失败
  */
uint8_t Flash_ClearFault(void);
#endif

#endif /* __FLASH_FAULT_H */
