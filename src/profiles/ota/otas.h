/**
 ****************************************************************************************
 *
 * @file otas.h
 *
 * @brief Header file - Quintic Private Profile Server.
 *
 * Copyright (C) Quintic 2013-2013
 *
 * $Rev: $
 *
 ****************************************************************************************
 */

#ifndef __OTAS_H__
#define __OTAS_H__


/**
 ****************************************************************************************
 * @addtogroup Quintic private profile Server
 * @ingroup OTAS
 * @brief Quintic private profile Server
 *
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

#if (BLE_OTA_SERVER)
#include "prf_types.h"
#include "attm.h"
#include "atts.h"
#include "atts_db.h"
#include "flash_lib.h"

/*
 * DEFINES
 ****************************************************************************************
 */

// Derek, used as max data length

#define OTAS_MANDATORY_INCREASE_NUM     2
#define OTAS_MANDATORY_INCREASE_MASK    0x03
#define OTAS_MANDATORY_SUM_NUM          7
#define OTAS_MANDATORY_SUM_MASK         0x7F

#define OTAS_RX_CHAR_NUM                15
#define OTAS_RX_CHAR_PER_VOLUME         18
#define OTAS_PROP_RX_CHAR               (ATT_CHAR_PROP_WR | ATT_CHAR_PROP_WR_NO_RESP)

#define OTAS_TX_CHAR_NUM                1
#define OTAS_TX_CHAR_PER_VOLUME         20
#define OTAS_TX_CHAR_USR_PER_VOLUME     (OTAS_TX_CHAR_PER_VOLUME - 1 - 2 - 2)

#define OTA_RX_DATA_MAX                 (OTAS_RX_CHAR_PER_VOLUME * OTAS_RX_CHAR_NUM-2)

#define OTA_UPGRADE_CODE_VERSION        0x0101


#define BRICK_DATA_SIZE                 256
#define FLASH_SECTOR_SIZE               (4<<10)  // 4KB
#define OTA_WATCH_TIMER_MS              10  // 20ms

#define FLASH_SIZE                      (128<<10) // 64KB 
#define TIMER_1MS                       (111.111111*__TIMER_CLK/1000000)
#define LED_DISPLAY_TIME                (100 * TIMER_1MS)
#define BUZZER_ON_TIME                  (400 * TIMER_1MS)
#define BUZZER_OFF_TIME                 (200 * TIMER_1MS)
#define BUZZER_REPEAT_TIME              5

#define FLASH_BANK1_CODE_START_ADDRESS  0x2100
#define FLASH_BANK2_CODE_START_ADDRESS  ((FLASH_SIZE >> 1) + FLASH_BANK1_CODE_START_ADDRESS - 0x1000)
#define FLASH_CODE_ERASE_START_ADDRESS  (FLASH_BANK2_CODE_START_ADDRESS - 0x100)  // should 4K start
#define BANK1_FLASH_CODE_SECTOR_NUM     ((FLASH_BANK2_CODE_START_ADDRESS-FLASH_BANK1_CODE_START_ADDRESS) >> 12)
#define BANK2_FLASH_CODE_SECTOR_NUM     ((FLASH_SIZE-(FLASH_BANK2_CODE_START_ADDRESS-0x100)) >> 12)
#define FLASH_CODE_SECTOR_NUM           (BANK1_FLASH_CODE_SECTOR_NUM <= BANK2_FLASH_CODE_SECTOR_NUM ? BANK1_FLASH_CODE_SECTOR_NUM : BANK2_FLASH_CODE_SECTOR_NUM)
#define FLASH_META_DATA_ADDRESS         FLASH_CODE_ERASE_START_ADDRESS 
#define FLASH_CODE_SIZE_MAX             ((FLASH_SIZE-FLASH_BANK1_CODE_START_ADDRESS)>>1)
#define OTA_REPEAT_COUNT_MAX            10

#define FLASH_BL_BASE_POS                               0x1000
#define FLASH_BL_APP_CODE_ADDRESS_OFFSET 		        0x80    // 0x00009100 => 00 91 00 00
#define FLASH_BL_APP_CODE_SIZE_OFFSET       	        0x84
#define FLASH_BL_APP_CODE_CRC16_OFFSET      	        0x88
#define FLASH_BL_SPI_FLASH_CLOCK_OFFSET     	        0x8c
#define FLASH_BL_APP_IN_RAM_ADDRESS_OFFSET  	        0x90
#define FLASH_BL_APP_RESET_ADDRESS_OFFSET   	        0x94


#define OTA_COMMAND                                     (data[0]&0x0f)
#define OTA_REPEAT                                      (data[0]&0x10)

#define FLASH_ROUND(x)                                  (((x+3)>>2)<<2)

enum
{
 	OTAS_SVC_PRIVATE_UUID          = 0xCC03,
	OTAS_CHAR_TX_DATA_UUID         = 0xCD01,
  OTAS_CHAR_RX_DATA_START_UUID   = 0xCE01,
};

enum
{
    OTA_CMD_UPDATED_APP_INFORMATION,
    OTA_CMD_UPDATED_APP_DATA,
    
    
    OTA_CMD_CONFIRM,
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */
enum 
{
    OTA_FLASH_ERASED,
    OTA_FLASH_NORMAL,
    OTA_FLASH_READY,
    OTA_FLASH_NOT_READY
};

enum 
{
    OTA_CMD_META_DATA          = 0x01,
    OTA_CMD_BRICK_DATA         = 0x02,
    OTA_CMD_DATA_VERIFY        = 0x03,
    OTA_CMD_EXECUTION_NEW_CODE = 0x04,
};

enum 
{
    OTA_IDLE,
    OTA_INITED,
    OTA_META_CARRYING,
    OTA_DATA_CARRYING,
    OTA_DATA_ARRIVED,
    OTA_STATUS,
    OTA_RESUME,
    OTA_CHECK,
    OTA_FINISHED
};

enum OTA_Result
{
    OTA_RESULT_SUCCESS,
    OTA_RESULT_CHECKSUM_ERROR,
    OTA_RESULT_UNKNOWN_ERROR,
    OTA_RESULT_FAIL,
};

enum 
{
    OTA_ERROR_NO_ERROR,
    OTA_ERROR_SANITY_CHECK_FAILED,
    OTA_ERROR_FLASH_CORRUPTED,
    OTA_ERROR_BRICK_DATA_INVALID,
    OTA_ERROR_OVERSIZE,
    OTA_ERROR_META_DATA_NOT_VALID,
};

///Attributes State Machine
enum
{
    OTAS_IDX_SVC,

    OTAS_IDX_TX_DATA_CHAR,
    OTAS_IDX_TX_DATA_VAL,
    OTAS_IDX_TX_DATA_NTF_CFG,
    OTAS_IDX_TX_DATA_USER_DESP,

    OTAS_IDX_RX_DATA_CHAR,
    OTAS_IDX_RX_DATA_VAL,

    OTAS_IDX_NB,
};

enum
{
    OTAS_VALUE_NTF_CFG  = 0x01,
    OTAS_VALUE_READ_CFG,
};

struct s_flash_meta_data
{
    uint16_t id; 
    uint16_t version;
    uint16_t checksum; 
    uint32_t erase_start_addr; 
    uint32_t meta_data_start_addr; 
    uint32_t code_start_addr; 
    uint32_t size; 
    uint32_t sectorSize; 
    uint32_t sectorNum; 
};

struct s_flash
{
    uint8_t status; 
    uint16_t brickSize;
    uint16_t brickChecksum; 
    uint16_t byteCount;
    uint16_t codeSize; 
    uint32_t addr;
    struct s_flash_meta_data meta; 
    struct s_boot boot;
};

struct s_ota
{
    /// control handler
    uint8_t conhdl; 
    /// Profile role state: enabled/disabled
    uint8_t state;
    /// code size 
    uint16_t size; 
    /// previous read size
    uint16_t readSize; 
    /// previous command
    uint8_t cmd; 
    /// brick data
    uint8_t data[BRICK_DATA_SIZE];
    struct s_flash flash; 
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Server environment variable
struct otas_env_tag
{
    ///Application Task Id
    ke_task_id_t appid;
    ///Connection handle
    uint16_t conhdl;

    ///Service Start Handle
    uint16_t shdl;
    ///Database configuration
    uint32_t features;

    struct
    {
        uint8_t  cmd;
        uint16_t index;
        uint16_t length;
        uint16_t capacity;
        uint8_t *pdata;
    }rx;
};


/*
 * MACROS
 ****************************************************************************************
 */


/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */

extern const struct atts_desc otas_att_db[OTAS_IDX_NB];

///  Service - only one instance for now
extern const atts_svc_desc_t otas_svc;

extern const struct atts_char_desc otas_value_char;
extern const struct atts_char_desc otas_char_rx_data;

extern struct otas_env_tag otas_env;

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

extern struct s_ota *const pOTA;
extern struct s_ota otas;
extern void ota_buzzer(int repeat);
extern void ota_chip_reset(void);
extern void ota_disconnect_proc(struct s_ota *ota);
extern void ota_flash_on(void);
extern void ota_flash_lock(void);
extern void ota_reset_chip(void);
extern void app_timer_set(ke_msg_id_t const timer_id, uint16_t const delay); 
extern void ota_init(void); 
extern void ota_wr_data_req(uint16_t conhdl, uint8_t *val, uint8_t len, int type);
extern void ota_cmd_meta_data_write(uint16_t conhdl, struct s_flash *flash);
extern void ota_cmd_brick_data_write(uint16_t conhdl, struct s_ota *ota);
extern void ota_cmd_flash_check(uint16_t conhdl, struct s_flash *flash);
extern uint8_t ota_flash_brick_write(struct s_ota *ota); 
extern uint8_t ota_processing_status(struct s_ota *ota);
extern void ota_brick_data_read(struct s_ota *ota); 
extern void ota_resume_brick_data_read(struct s_ota *ota);
extern void ota_cmd_flash_burn(uint16_t conhdl, struct s_flash *flash);

extern uint8_t proc_cmd_meta_data_write(const uint8_t *data, size_t len);
extern void proc_cmd_status_update(const uint8_t *data);
extern uint8_t proc_cmd_brick_data_write(const uint8_t *data, size_t len);
extern uint8_t proc_cmd_flash_check(const uint8_t *data, size_t len);
extern void transmit_rsp_to_client(uint8_t cmd, const uint8_t *pdata, size_t len);
extern void transmit_error_rsp_to_client(uint8_t cmd, uint8_t error_code);


/*
 ****************************************************************************************
 * @brief Create Quintic Private Profile service database - at initiation
 *
 ****************************************************************************************
 */
void app_otas_create_db(uint8_t features);

/*
 ****************************************************************************************
 * @brief Start the profile - at connection    
 * 
 ****************************************************************************************
 */
void app_otas_enable_req(uint16_t conhdl, uint8_t sec_lvl, uint8_t con_type);

/*
 ****************************************************************************************
 * @brief Disable profile role - at disconnection    
 * 
 ****************************************************************************************
 */
void app_otas_disable_req(uint16_t conhdl);

/**
 ****************************************************************************************
 * @brief Initialization of the Quintic private profile module.
 * This function performs all the initializations of the OTASS module.
 ****************************************************************************************
 */
void otas_init(void);

/**
 ****************************************************************************************
 * @brief Indicate error to higher layers.
 * @param status Error status to send to requester task
 ****************************************************************************************
 */
void otas_error_ind_send(uint8_t status);

/**
 ****************************************************************************************
 * @brief Disable actions grouped in getting back to IDLE and sending configuration to requester task
 ****************************************************************************************
 */
void otas_disable(void);

#endif /* #if (BLE_OTA_SERVER) */

/// @} OTASS

#endif /* _OTASS_H_ */
