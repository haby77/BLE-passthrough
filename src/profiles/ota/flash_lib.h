
#ifndef _FLASH_LIB_
#define _FLASH_LIB_

#include <stdint.h>

struct boot_info_t
{
    uint32_t app_in_flash_addr;
    uint32_t app_reset_addr;
    uint32_t app_size;
    uint16_t app_crc;
};

struct s_boot
{
    uint32_t code_in_flash_addr;
    uint32_t code_reset_addr;
    uint32_t code_size;
    uint16_t code_crc;
};

enum LOCK_FLASH
{
    FLASH_LOCK,
    FLASH_UNLOCK
};

extern uint32_t flash_bl_app_addr(void);
/**
 ****************************************************************************************
 * @brief calling this function when begin to write ota data to flash
 ****************************************************************************************
 */
extern void flash_security_enable(void); 

/**
 ****************************************************************************************
 * @brief calling this function when finish to write ota data to flash
 *
 * @param[in]    pboot_info    OTA data infomation, include crc, size, in flash address, code reset address
 * @param[in]    lock          chip lock flag
 ****************************************************************************************
 */
extern void flash_security_lock(const struct s_boot *pboot_info, enum LOCK_FLASH lock); 

/**
 ****************************************************************************************
 * @brief soft reset chip
 ****************************************************************************************
 */
void reset_chip(void);

#endif
