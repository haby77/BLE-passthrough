/**
 ****************************************************************************************
 *
 * @file qpps.c
 *
 * @brief Quintic Private Profile Server implementation.
 *
 * Copyright (C) Quintic 2013-2013
 *
 * $Rev: $
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup OTAS
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_config.h"

#if (BLE_OTA_SERVER && (defined TASK_OTAS))
#include "gap.h"
#include "gatt_task.h"
#include "atts_util.h"
#include "smpc_task.h"
#include "usr_design.h"
#include "otas.h"
#include "otas_task.h"

#include "serialflash.h"


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */


/*
 * PROFILE ATTRIBUTES
 ****************************************************************************************
 */
/// Full OTAS Database Description - Used to add attributes into the database
const struct atts_desc otas_att_db[OTAS_IDX_NB] =
{
    // Service Declaration
    [OTAS_IDX_SVC]                  =   {ATT_DECL_PRIMARY_SERVICE, PERM(RD, ENABLE), sizeof(otas_svc),
                                         sizeof(otas_svc), (uint8_t *)&otas_svc},

    /*****************************************TX data notify*******************************************/
    // Characteristic Declaration
    [OTAS_IDX_TX_DATA_CHAR]         =   {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), sizeof(otas_value_char),
                                         sizeof(otas_value_char), (uint8_t *)&otas_value_char},
    // Characteristic Value
    [OTAS_IDX_TX_DATA_VAL]          =   {OTAS_CHAR_TX_DATA_UUID, PERM(NTF, ENABLE), OTAS_TX_CHAR_PER_VOLUME,
                                         6, (uint8_t *)"OTA TX"},
    // Client Characteristic Configuration Descriptor
    [OTAS_IDX_TX_DATA_NTF_CFG]      =   {ATT_DESC_CLIENT_CHAR_CFG, PERM(RD, ENABLE)|PERM(WR, ENABLE), sizeof(uint16_t),
                                         0, NULL},
    // User Descriptor
    [OTAS_IDX_TX_DATA_USER_DESP]    =   {ATT_DESC_CHAR_USER_DESCRIPTION, PERM(RD, ENABLE), 15,
                                         15, (uint8_t *)"Quintic OTAS OTA"},

    /**********************************************RX data**************************************************/
    // Received data Characteristic Declaration
    [OTAS_IDX_RX_DATA_CHAR]         =   {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), sizeof(otas_char_rx_data),
                                         sizeof(otas_char_rx_data), (uint8_t *)&otas_char_rx_data},
    // Received data Characteristic Value
    [OTAS_IDX_RX_DATA_VAL]          =   {OTAS_CHAR_RX_DATA_START_UUID, PERM(WR, ENABLE), OTAS_RX_CHAR_PER_VOLUME,
                                         0, NULL},
};

/*
 * PROFILE ATTRIBUTES
 ****************************************************************************************
 */
/// Server Service
const atts_svc_desc_t otas_svc = OTAS_SVC_PRIVATE_UUID;

/// Server Service - Server value Characteristic
const struct atts_char_desc otas_value_char = ATTS_CHAR(ATT_CHAR_PROP_NTF,
                                                        0,
                                                        OTAS_CHAR_TX_DATA_UUID);

/// RX data characteristic
const struct atts_char_desc otas_char_rx_data = ATTS_CHAR(OTAS_PROP_RX_CHAR,
                                                          0,
                                                          OTAS_CHAR_RX_DATA_START_UUID);

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
struct otas_env_tag otas_env;

uint8_t testData[BRICK_DATA_SIZE];

struct s_ota otas;
struct s_ota * const pOTA = &otas;

const unsigned short Table_CRC[256] =
{
    0X0000L,  0X1021L,  0X2042L,  0X3063L,
    0X4084L,  0X50a5L,  0X60c6L,  0X70e7L,
    0X8108L,  0X9129L,  0Xa14aL,  0Xb16bL,
    0Xc18cL,  0Xd1adL,  0Xe1ceL,  0Xf1efL,
    0X1231L,  0X0210L,  0X3273L,  0X2252L,
    0X52b5L,  0X4294L,  0X72f7L,  0X62d6L,
    0X9339L,  0X8318L,  0Xb37bL,  0Xa35aL,
    0Xd3bdL,  0Xc39cL,  0Xf3ffL,  0Xe3deL,
    0X2462L,  0X3443L,  0X0420L,  0X1401L,
    0X64e6L,  0X74c7L,  0X44a4L,  0X5485L,
    0Xa56aL,  0Xb54bL,  0X8528L,  0X9509L,
    0Xe5eeL,  0Xf5cfL,  0Xc5acL,  0Xd58dL,
    0X3653L,  0X2672L,  0X1611L,  0X0630L,
    0X76d7L,  0X66f6L,  0X5695L,  0X46b4L,
    0Xb75bL,  0Xa77aL,  0X9719L,  0X8738L,
    0Xf7dfL,  0Xe7feL,  0Xd79dL,  0Xc7bcL,
    0X48c4L,  0X58e5L,  0X6886L,  0X78a7L,
    0X0840L,  0X1861L,  0X2802L,  0X3823L,
    0Xc9ccL,  0Xd9edL,  0Xe98eL,  0Xf9afL,
    0X8948L,  0X9969L,  0Xa90aL,  0Xb92bL,
    0X5af5L,  0X4ad4L,  0X7ab7L,  0X6a96L,
    0X1a71L,  0X0a50L,  0X3a33L,  0X2a12L,
    0XdbfdL,  0XcbdcL,  0XfbbfL,  0Xeb9eL,
    0X9b79L,  0X8b58L,  0Xbb3bL,  0Xab1aL,
    0X6ca6L,  0X7c87L,  0X4ce4L,  0X5cc5L,
    0X2c22L,  0X3c03L,  0X0c60L,  0X1c41L,
    0XedaeL,  0Xfd8fL,  0XcdecL,  0XddcdL,
    0Xad2aL,  0Xbd0bL,  0X8d68L,  0X9d49L,
    0X7e97L,  0X6eb6L,  0X5ed5L,  0X4ef4L,
    0X3e13L,  0X2e32L,  0X1e51L,  0X0e70L,
    0Xff9fL,  0XefbeL,  0XdfddL,  0XcffcL,
    0Xbf1bL,  0Xaf3aL,  0X9f59L,  0X8f78L,
    0X9188L,  0X81a9L,  0Xb1caL,  0Xa1ebL,
    0Xd10cL,  0Xc12dL,  0Xf14eL,  0Xe16fL,
    0X1080L,  0X00a1L,  0X30c2L,  0X20e3L,
    0X5004L,  0X4025L,  0X7046L,  0X6067L,
    0X83b9L,  0X9398L,  0Xa3fbL,  0Xb3daL,
    0Xc33dL,  0Xd31cL,  0Xe37fL,  0Xf35eL,
    0X02b1L,  0X1290L,  0X22f3L,  0X32d2L,
    0X4235L,  0X5214L,  0X6277L,  0X7256L,
    0Xb5eaL,  0Xa5cbL,  0X95a8L,  0X8589L,
    0Xf56eL,  0Xe54fL,  0Xd52cL,  0Xc50dL,
    0X34e2L,  0X24c3L,  0X14a0L,  0X0481L,
    0X7466L,  0X6447L,  0X5424L,  0X4405L,
    0Xa7dbL,  0Xb7faL,  0X8799L,  0X97b8L,
    0Xe75fL,  0Xf77eL,  0Xc71dL,  0Xd73cL,
    0X26d3L,  0X36f2L,  0X0691L,  0X16b0L,
    0X6657L,  0X7676L,  0X4615L,  0X5634L,
    0Xd94cL,  0Xc96dL,  0Xf90eL,  0Xe92fL,
    0X99c8L,  0X89e9L,  0Xb98aL,  0Xa9abL,
    0X5844L,  0X4865L,  0X7806L,  0X6827L,
    0X18c0L,  0X08e1L,  0X3882L,  0X28a3L,
    0Xcb7dL,  0Xdb5cL,  0Xeb3fL,  0Xfb1eL,
    0X8bf9L,  0X9bd8L,  0XabbbL,  0Xbb9aL,
    0X4a75L,  0X5a54L,  0X6a37L,  0X7a16L,
    0X0af1L,  0X1ad0L,  0X2ab3L,  0X3a92L,
    0Xfd2eL,  0Xed0fL,  0Xdd6cL,  0Xcd4dL,
    0XbdaaL,  0Xad8bL,  0X9de8L,  0X8dc9L,
    0X7c26L,  0X6c07L,  0X5c64L,  0X4c45L,
    0X3ca2L,  0X2c83L,  0X1ce0L,  0X0cc1L,
    0Xef1fL,  0Xff3eL,  0Xcf5dL,  0Xdf7cL,
    0Xaf9bL,  0XbfbaL,  0X8fd9L,  0X9ff8L,
    0X6e17L,  0X7e36L,  0X4e55L,  0X5e74L,
    0X2e93L,  0X3eb2L,  0X0ed1L,  0X1ef0L
};

/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

uint32_t ota_flash_bl_addr()
{
    return flash_bl_app_addr();
}

void otas_init(void)
{
    static bool is_ota_flash_init = false;
    
    // Reset environment
    memset(&otas_env, 0, sizeof(otas_env));

    // Register OTASS task into kernel
    task_otas_desc_register();

    // Go to IDLE state
    ke_state_set(TASK_OTAS, OTAS_DISABLED);

    if(!is_ota_flash_init)
    {
        is_ota_flash_init = true;
        ota_init();
    }
}

void otas_error_ind_send(uint8_t status)
{
    struct otas_error_ind *ind = KE_MSG_ALLOC(OTAS_ERROR_IND,
                                              otas_env.appid,
                                              TASK_OTAS,
                                              otas_error_ind);
    ind->conhdl    = otas_env.conhdl;
    //it will be an OTASC status code
    ind->status    = status;
    // send the message
    ke_msg_send(ind);
}

void otas_disable(void)
{
    //Disable HRS in database
    attsdb_svc_set_permission(otas_env.shdl, PERM_RIGHT_DISABLE);

    //Send current configuration to APP
    struct otas_disable_cfm * cfg = KE_MSG_ALLOC(OTAS_DISABLE_CFM,
                                                 otas_env.appid, TASK_OTAS,
                                                 otas_disable_cfm);

    memcpy(&cfg->conhdl, &otas_env.conhdl, sizeof(uint16_t));

    //Notifications Configuration
    cfg->ntf_en = otas_env.features;

    ke_msg_send(cfg);

    //Go to idle state
    ke_state_set(TASK_OTAS, OTAS_IDLE);
}

/*
 ****************************************************************************************
 * @brief Create Quintic Private Profile service database - at initiation        *//**
 *
 * @param[in] features
 *
 * @response
 * @description
 *
 ****************************************************************************************
 */
void app_otas_create_db(uint8_t rx_char_num)
{
    struct otas_create_db_req * msg = KE_MSG_ALLOC(OTAS_CREATE_DB_REQ, TASK_OTAS, TASK_APP, otas_create_db_req);

    ASSERT_ERR(rx_char_num > 0);

    msg->rx_char_num = rx_char_num;

    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Start the Quintic Private profile - at connection      *//**
 *
 * @param[in] conhdl Connection handle.
 * @param[in] sec_lvl Security level required for protection of HRS attributes:
 * Service Hide and Disable are not permitted. Possible values are:
 * - PERM_RIGHT_ENABLE
 * - PERM_RIGHT_UNAUTH
 * - PERM_RIGHT_AUTH
 * - PERM_RIGHT_AUTHZ
 * @param[in] con_type Connection type: configuration(0) or discovery(1)
 * @param[in] ntf_en Notification configuration
 *
 * @response None
 * @description
 *
 ****************************************************************************************
 */
void app_otas_enable_req(uint16_t conhdl, uint8_t sec_lvl, uint8_t con_type)
{
    struct otas_enable_req * msg = KE_MSG_ALLOC(OTAS_ENABLE_REQ, TASK_OTAS, TASK_APP,
                                                otas_enable_req);

    msg->conhdl = conhdl;
    msg->sec_lvl = sec_lvl;
    msg->con_type = con_type;

    ota_flash_on();

    pOTA->state = OTA_INITED;
    
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Disable profile role - at disconnection       *//**
 *
 * @param[in] conhdl Connection handle for which the profile role is enabled
 *
 * @response OTAS_DISABLE_CFM
 * @description
 *
 ****************************************************************************************
 */
void app_otas_disable_req(uint16_t conhdl)
{
    struct otas_disable_req * msg = KE_MSG_ALLOC(OTAS_DISABLE_REQ, TASK_OTAS, TASK_APP,
                                                 otas_disable_req);

    msg->conhdl = conhdl;

    ota_disconnect_proc(pOTA);

    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Send data - at connection.        *//**
 *
 * @param[in] conhdl Connection handle for which the profile Heart Rate sensor role is
 * enabled.
 * @param[in] length data length
 * @param[in] data Pointer to data
 *
 * @response
 * @description
 *
 ****************************************************************************************
 */
void app_otas_data_send(uint16_t conhdl, uint8_t index, uint8_t length, uint8_t *data)
{
    //QPRINTF("app_otas_data_send, cur_time: %d\n", ke_time()*10);
    struct otas_data_send_req * msg = KE_MSG_ALLOC_DYN(OTAS_DATA_SEND_REQ, TASK_OTAS, TASK_APP,
                                                       otas_data_send_req, length);

    msg->conhdl = conhdl;
    msg->index = index;
    msg->length = length;
    memcpy(msg->data, data, length);

    ke_msg_send(msg);

}



uint32_t ota_get_new_flash_code_address(struct s_flash *flash)
{
    uint32_t addr;
    if(flash->meta.code_start_addr == FLASH_BANK2_CODE_START_ADDRESS)
    {
        addr = FLASH_BANK1_CODE_START_ADDRESS;
    }
    else
    {
        addr = FLASH_BANK2_CODE_START_ADDRESS;
    }
    QPRINTF("ota_get_new_flash_code_address: 0x%x\n", addr);
    return addr;
}

/*
    used for data checksum calculation

*/
uint16_t ota_checksum(const uint8_t *data, uint16_t count)
{
    uint16_t i;
    uint16_t checksum = 0;
    for(i=0; i<count; i++)
        checksum += data[i];

    return checksum;
}

/*
   do flash code checksum calculation
*/
uint16_t ota_flash_crc16(uint32_t startAddr, uint16_t codeSize)
{
    uint32_t size,i;
    uint16_t crc16 = 0;
    uint32_t addr;
    uint32_t d;
    uint32_t reset_address = ~0;

    const uint32_t endAddr = startAddr + codeSize;

    for(addr = startAddr; addr < endAddr; addr += size)
    {
        d = endAddr - addr;
        size = d >= BRICK_DATA_SIZE ? BRICK_DATA_SIZE : d;
        read_flash(addr, (uint32_t *)testData, BRICK_DATA_SIZE);

        if(addr == startAddr)
            reset_address = *(uint32_t *)&testData[4];

        for (i = 0; i < size; i++)
            crc16 = ( crc16 << 8 ) ^ (uint16_t)Table_CRC[( crc16 >> 8 ) ^ testData[i]];
    }

    otas.flash.boot.code_reset_addr = reset_address & (~3);

    return crc16;
}

void ota_flash_on(void)
{
    QPRINTF("\nota_flash_on\r\n");
    power_on_flash();
}

void ota_flash_lock(void)
{
    QPRINTF("\nota_flash_lock\n");
    //erase flash
    sector_erase_flash(0x1000, 1);
    // ota finish
    flash_security_lock(&otas.flash.boot, FLASH_UNLOCK);
}

void ota_flash_off(void)
{
    power_off_flash();
}

void ota_reset_chip(void)
{
    QPRINTF("\nota_reset_chip\n");

    *(volatile uint32_t *)0x40000000 = 1UL << 16;
}

void app_timer_set(ke_msg_id_t const timer_id, uint16_t const delay)
{
    ke_timer_set(timer_id, TASK_APP, delay);
}

void ota_flash_erase(struct s_flash *flash)
{
    block_erase_flash(flash->meta.erase_start_addr, flash->meta.sectorSize, flash->meta.sectorNum);
    flash->status = OTA_FLASH_ERASED;
}



void ota_flash_meta_data_init(struct s_flash *flash)
{
    flash->meta.size = FLASH_CODE_SIZE_MAX;
    flash->meta.id = 0xbabe;
    flash->meta.version = OTA_UPGRADE_CODE_VERSION;
    flash->meta.erase_start_addr = FLASH_CODE_ERASE_START_ADDRESS;
    flash->meta.meta_data_start_addr = FLASH_META_DATA_ADDRESS;
    flash->meta.code_start_addr = ota_flash_bl_addr();
    flash->meta.checksum = 0;
    flash->meta.sectorSize = FLASH_SECTOR_SIZE;
    flash->meta.sectorNum = FLASH_CODE_SECTOR_NUM;
}

void ota_copy_ram_to_flash(int dst, uint8_t *src, int size)
{
    uint32_t *psrc = (uint32_t *)src;
    write_flash(dst, psrc, size);
}

void ota_copy_flash_to_ram(uint8_t *dst, int src, int size)
{
    uint32_t *pdst = (uint32_t *)dst;
    read_flash(src, pdst, size);
}

void ota_flash_init(struct s_ota *ota)
{
    struct s_flash *flash = &ota->flash;

    ota_flash_meta_data_init(flash);

    flash->brickSize = BRICK_DATA_SIZE;

    flash->addr = ota_get_new_flash_code_address(flash);
    flash->boot.code_in_flash_addr = flash->addr;
//    sector_erase_flash(flash->addr-0x100, FLASH_CODE_SECTOR_NUM);
//    QPRINTF("sector_erase_flash: addr = 0x%x, sector num: %d\n", flash->addr-0x100, FLASH_CODE_SECTOR_NUM);

    flash->brickChecksum = 0;
}

/*
 flashOn:
     1. enable flash;
     2. init flash, read out boot loader info (not set security, it will be set after connection built)
     3. erase flash to get empty space for OTA on server side
 ledOn:
     1. take quite a while (few of seconds) to get buzzer and led display to show what's the code is running
*/
void ota_init(void)
{   
    QPRINTF("ota_init\n");

    memset(&otas, 0, sizeof(otas));
    
    ota_flash_on();
    ota_flash_init(pOTA);
    
    flash_security_enable();
    
    pOTA->state = OTA_INITED;
    pOTA->flash.status = OTA_FLASH_READY;
}

/*
  after disconnection message received, doing:
  1. set suspending flag (suspend) = 1;
  2. for master, set state = RESUME (so switch there to do corresponding processing (like read brick data)),  and do reconnection.
  3. for slave, simply advertising again
*/
void ota_disconnect_proc(struct s_ota *ota)
{
    QPRINTF("ota_disconnect_proc:\r\n");
    
    app_gap_adv_start_req(GAP_GEN_DISCOVERABLE|GAP_UND_CONNECTABLE,
        app_env.adv_data, app_set_adv_data(GAP_GEN_DISCOVERABLE),
        app_env.scanrsp_data, app_set_scan_rsp_data(app_get_local_service_flag()),
        GAP_ADV_FAST_INTV1, GAP_ADV_FAST_INTV2);
}

void transmit_error_rsp_to_client(uint8_t cmd, uint8_t error_code)
{
    transmit_rsp_to_client(cmd, &error_code, 1);
}

void transmit_rsp_to_client(uint8_t cmd, const uint8_t *pdata, size_t len)
{
    uint8_t notify_data[OTAS_TX_CHAR_PER_VOLUME];
    int notify_data_index, i;
    uint16_t checksum = 0;
    
    notify_data[0] = (len+1) >> 8;
    notify_data[1] = (len+1) >> 0;
    notify_data[2] = cmd;
    
    memcpy(&notify_data[3], pdata, len);
    
    for(i=2; i<3+len; ++i) //from cmd to data
        checksum += notify_data[i];
    
    notify_data_index = 3 + len;
    
    notify_data[notify_data_index++] = checksum >> 8;
    notify_data[notify_data_index++] = checksum >> 0;
    
    ASSERT_ERR(notify_data_index<=OTAS_TX_CHAR_PER_VOLUME);
    
#if OTAS_DEBUG
    QPRINTF("==>Notify start, len=%d", notify_data_index);
    for(i=0; i<notify_data_index; ++i)
    {
        if(i%16 == 0)
            QPRINTF("\n");
        QPRINTF("%02X ", notify_data[i]);
    }
    QPRINTF("\n<==Notify data end\n");
#endif
    
    attsdb_att_set_value(otas_env.shdl + OTAS_IDX_TX_DATA_VAL, notify_data_index, notify_data);

    //send notification through GATT
    struct gatt_notify_req * ntf = KE_MSG_ALLOC(GATT_NOTIFY_REQ, TASK_GATT,
                                                TASK_OTAS, gatt_notify_req);
    ntf->conhdl  = otas_env.conhdl;
    ntf->charhdl = otas_env.shdl + OTAS_IDX_TX_DATA_VAL;

    ke_msg_send(ntf);
}


/*
  when entire brick data arrived, burn all of them to flash, not verified at this time to save OTA time


*/
uint8_t ota_flash_brick_write(struct s_ota *ota)
{
    uint32_t addr = ota->flash.addr + ota->flash.byteCount;
    
    if((addr == ota->flash.addr) || ((addr & 0xFFFU) == 0))
    {
        sector_erase_flash(addr & ~0xFFFU, 1);
    }
    
    memset(testData,0,ota->flash.brickSize);
    memcpy(testData, &ota->data[0], ota->flash.brickSize);
    write_flash(addr, (uint32_t *)testData, ota->flash.brickSize);
    ota->flash.byteCount += ota->flash.brickSize;

    return OTA_ERROR_NO_ERROR;
}

void rsp_cmd_meta_data_write(enum OTA_Result result, uint16_t writted_byte_count)
{
    //Update value in DB
    uint8_t data[OTAS_TX_CHAR_USR_PER_VOLUME];
    uint8_t i=0;
    
    data[i++] = (uint8_t)result;
    data[i++] = writted_byte_count >> 8;
    data[i++] = writted_byte_count >> 0;

    ASSERT_ERR(i < OTAS_TX_CHAR_USR_PER_VOLUME);

    transmit_rsp_to_client(OTA_CMD_META_DATA, data, i);
}

void rsp_cmd_brick_data_write(enum OTA_Result result, uint16_t writted_byte_count)
{
    //Update value in DB
    uint8_t data[OTAS_TX_CHAR_USR_PER_VOLUME];
    uint8_t i=0;
    
    data[i++] = (uint8_t)result;
    data[i++] = writted_byte_count >> 8;
    data[i++] = writted_byte_count >> 0;

    ASSERT_ERR(i < OTAS_TX_CHAR_USR_PER_VOLUME);

    transmit_rsp_to_client(OTA_CMD_BRICK_DATA, data, i);
}

void rsp_cmd_flash_check(enum OTA_Result result)
{
    //Update value in DB
    uint8_t data[OTAS_TX_CHAR_USR_PER_VOLUME];
    uint8_t i=0;
    
    data[i++] = (uint8_t)result;

    ASSERT_ERR(i < OTAS_TX_CHAR_USR_PER_VOLUME);

    transmit_rsp_to_client(OTA_CMD_DATA_VERIFY, data, i);
}

/*
  Server side get meta info from master, record in otam.flash
  1. version:  code version, in case need to check with old version to decide if OTA upgrading is necessary
  2. checksum: the new code checksum (in word: two bytes)
  3. code_in_flash_addr: actually it's decided by otas.flash.addr for boot loader info upgrading usage, so this field of two bytes
     could be used for other information OTA meta data
  4. code_size:  the new code size
  5. code_reset_addr: after reset, boot loader will copy flash code from this address in RAM and then jump to here to execute
  6. code_crc: boot loader will do crc check
*/
uint8_t proc_cmd_meta_data_write(const uint8_t *data, size_t len)
{
    struct s_flash *flash = &otas.flash;
    
    int i=0;
    uint16_t version =   (data[i] << 8) | data[i+1]; i+=2;
    uint16_t code_size = (data[i] << 8) | data[i+1]; i+=2;
    uint16_t code_crc =  (data[i] << 8) | data[i+1]; i+=2;
    
    if( flash->meta.version   == version &&
        flash->boot.code_size == code_size &&
        flash->boot.code_crc  == code_crc)
    {
        QPRINTF("Meta data same, resume transmit\n");
    }
    else
    {
        //ota_init();
        flash->meta.version = version;
        flash->boot.code_size = code_size;
        flash->boot.code_crc = code_crc;
    }

    pOTA->flash.codeSize = flash->codeSize = flash->boot.code_size;

    QPRINTF("got meta data: code_size: 0x%x, code_crc: 0x%x\n", flash->boot.code_size,flash->boot.code_crc);
    
    rsp_cmd_meta_data_write(OTA_RESULT_SUCCESS, flash->byteCount);
    
    return OTA_DATA_CARRYING;
}

/*
  Process brick data write command
  1. OTA_CMD_BRICK_FULL_DATA_WRITE
     just do data copy from OTA brick PDU data to pOTA->data, wait for entire brick data arriving
  2. OTA_CMD_BRICK_DATA_DONE
     the last OTA brick data PDU, at this time, doing:
     a. if it's not boundary data (could not carry on checksum, normally it possible happen when last brick data
        size is the number to match this case), then will have checksum comparison
     b. if checksum is ok, then burn entire brick data to flash
     c. if it's boundary data (for example, last brick size is 16 bytes, in the case of OTA data PDU is 17 bytes,
        WRITE_DATA_PAYLOAD_SIZE - size < 2
        there is only one byte left, so will not carry on checksum and checksum comparison, then rely on
        flash checksum check
*/
uint8_t proc_cmd_brick_data_write(const uint8_t *data, size_t len)
{
    struct s_flash *flash = &pOTA->flash;
        
    if(flash->byteCount >= otas.flash.codeSize)
    {
        // no more data to be processed
        return OTA_DATA_ARRIVED;
    }

    memcpy(pOTA->data, data, len);
    
    ota_flash_brick_write(pOTA);
    
    QPRINTF("Write: byteCount=0x%02X\n", flash->byteCount);

    rsp_cmd_brick_data_write(OTA_RESULT_SUCCESS, flash->byteCount);

    if(pOTA->flash.byteCount >= otas.flash.codeSize)
    {
        QPRINTF("switch to OTA_DATA_ARRIVED\n");
        return OTA_DATA_ARRIVED;
    }

    return OTA_DATA_CARRYING;
}

uint16_t print_flash_data(uint32_t startAddr, uint16_t codeSize)
{
    uint32_t size,i;
    uint16_t crc16 = 0;
    uint32_t addr;
    uint32_t d;

    const uint32_t endAddr = startAddr + codeSize;

    QPRINTF("Dump Flash data:\r\n");
    for(addr = startAddr; addr < endAddr; addr += size)
    {
        d = endAddr - addr;
        size = d >= BRICK_DATA_SIZE ? BRICK_DATA_SIZE : d;
        read_flash(addr, (uint32_t *)testData, BRICK_DATA_SIZE);
        
        for (i = 0; i < size; i++)
        {
            QPRINTF("%02X ", testData[i]);
            crc16 = ( crc16 << 8 ) ^ (uint16_t)Table_CRC[( crc16 >> 8 ) ^ testData[i]];
        }
        QPRINTF("\r\n");
    }
    
    QPRINTF("crc=%04X\r\n", crc16);
    
    return crc16;
}
/*
  updated flash code read out check
  1. read out new code from flash and do checksum
  2. if comparison success, then switch to OTA_FINISHED state
     otherwise do ota_init
*/
uint8_t proc_cmd_flash_check(const uint8_t *data, size_t len)
{
    // at this time, brickChecksum is entire flash code checksum
    struct s_flash *flash = &pOTA->flash;

    const uint16_t crc16 = ota_flash_crc16(ota_get_new_flash_code_address(flash), flash->codeSize);

    if(otas.flash.boot.code_crc == crc16)
    {
        ota_flash_lock();
        rsp_cmd_flash_check(OTA_RESULT_SUCCESS);
        QPRINTF("flash_check successfully\n");
        return OTA_FINISHED;
    }
    else
    {
	    flash->byteCount = 0;
        QPRINTF("flash_check fail, Air=%04X calc=%04X\n", otas.flash.boot.code_crc, crc16);
        rsp_cmd_flash_check(OTA_RESULT_FAIL);
        return OTA_INITED;
    }
}

#endif /* BLE_OTA_SERVER */

/// @} OTASS
