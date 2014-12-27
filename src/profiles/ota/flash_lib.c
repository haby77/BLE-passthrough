

#include "app_config.h"
#include "app_env.h"

#include "flash_lib.h"
#include "serialflash.h"
#include "qn9020.h"

/*definition save special data  address*/
//definition BL size, HW BL SIZE + SW BL SIZE
#define HW_BL_INFOR_SIZE    16

#define NVDS_SIZE                   (4*1024)    //reserved 4K for NVDS
//definition default app in flash address
#define DEF_APP_IN_FLASH_ADDR       (NVDS_SIZE+256)

//saving app in flash address
#define SAVE_APP_IN_FLASH_ADDR      (NVDS_SIZE+HW_BL_INFOR_SIZE+0x00)
//save the address of 4 bytes application size
#define SAVE_APP_SIZE_ADDR          (NVDS_SIZE+HW_BL_INFOR_SIZE+0x04)
//save  the address of 4 bytes application's crc address
#define SAVE_APP_CRC_ADDR           (NVDS_SIZE+HW_BL_INFOR_SIZE+0x08)
//save  the address of 4 bytes flash clock address
#define SAVE_FLASH_CLK_ADDR         (NVDS_SIZE+HW_BL_INFOR_SIZE+0x0C)
//save  the address of 4 bytes application's in ram address
#define SAVE_APP_IN_RAM_ADDR        (NVDS_SIZE+HW_BL_INFOR_SIZE+0x10)
//save  the address of 4 bytes application's reset address
#define SAVE_APP_RESET_ADDR         (NVDS_SIZE+HW_BL_INFOR_SIZE+0x14)
//save  the address of 8 bytes flash's commands
#define SAVE_FLASH_CMD_ADDR         (NVDS_SIZE+HW_BL_INFOR_SIZE+0x20)

uint32_t flash_bl_app_addr(void)
{
    uint32_t addr;
    read_flash(SAVE_APP_IN_FLASH_ADDR, &addr, 4);
    QPRINTF("Current code address: 0x%X\r\n", addr);
    return addr;
}

/**
 ****************************************************************************************
 * @brief calling this function when begin to write ota data to flash
 ****************************************************************************************
 */
void flash_security_enable(void)
{
    *(volatile uint32_t *)0x3FFFFFD0 = QN_TIMER0->CNT;          //write random 1
    *(volatile uint32_t *)0x3FFFFFD4 = *(uint32_t *)(__TIME__); //write random 2
    *(volatile uint32_t *)0x3FFFFFD8 = *(uint32_t *)(__DATE__); //write random 3
}

/**
 ****************************************************************************************
 * @brief calling this function when finish to write ota data to flash
 *
 * @param[in]    pboot_info    OTA data infomation, include crc, size, in flash address, code reset address
 * @param[in]    lock          chip lock flag
 ****************************************************************************************
 */
void flash_security_lock(const struct s_boot *pboot_info, enum LOCK_FLASH lock)
{
    const uint8_t flash_cmd[8]= {0x05, 0x06, 0x20, 0x52, 0x60, 0xB9, 0xAB, 0x01};
    const uint32_t app_in_ram_addr = 0x10000000;
    const uint32_t flash_clk = 8000000;
    uint32_t code_in_flash_addr = pboot_info->code_in_flash_addr;
    uint32_t code_reset_addr = pboot_info->code_reset_addr;
    uint32_t code_size = pboot_info->code_size;
    uint32_t code_crc = pboot_info->code_crc;

    QPRINTF("code_in_flash_addr: 0x%x,code_reset_addr: 0x%x, code_size: 0x%x,code_crc:0x%x\n",code_in_flash_addr,code_reset_addr,code_size,code_crc);
    write_flash(SAVE_APP_IN_FLASH_ADDR, &code_in_flash_addr, 4);
    write_flash(SAVE_APP_RESET_ADDR,    &code_reset_addr   , 4);
    write_flash(SAVE_APP_SIZE_ADDR,     &code_size         , 4);
    write_flash(SAVE_APP_CRC_ADDR,      &code_crc          , 4);

    write_flash(SAVE_FLASH_CLK_ADDR,  (uint32_t *)&flash_clk, 4);
    write_flash(SAVE_FLASH_CMD_ADDR,  (uint32_t *)flash_cmd, 8);
    write_flash(SAVE_APP_IN_RAM_ADDR, (uint32_t *)&app_in_ram_addr, 4);

    write_flash(0x00001000, (uint32_t *)"1234", 4);
    write_flash(0x00001004, (uint32_t *)"1234", 4);
    write_flash(0x00001008, (uint32_t *)"1234", 4);

    if(lock == FLASH_LOCK)
    {
        write_flash(0x3000100C, (uint32_t *)"\x00\x00\x00\x00", 4); //if 0 is lock
    }
}


