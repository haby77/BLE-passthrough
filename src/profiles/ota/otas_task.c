/**
 ****************************************************************************************
 *
 * @file otas_task.c
 *
 * @brief Quintic Private Profile Server Task Implementation.
 *
 * Copyright (C) Quintic 2013-2013
 *
 * $Rev: $
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup OTASSTASK
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
#include "otas.h"
#include "otas_task.h"
#include "ke_mem.h"
#include "lib.h"
#include "prf_utils.h"


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */



/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * MACROS
 ****************************************************************************************
 */


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @brief Handles reception of the @ref OTAS_CREATE_DB_REQ message.
 * The handler adds Quintic private service into the database using the database
 * configuration value given in param.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int otas_create_db_req_handler(ke_msg_id_t const msgid,
                                      struct otas_create_db_req const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    //Service Configuration Flag
    uint64_t cfg_flag = OTAS_MANDATORY_SUM_MASK;
    uint8_t idx_nb = OTAS_MANDATORY_SUM_NUM;
    //Database Creation Status
    uint8_t status;

    //rx char number
    const int rx_char_num = param->rx_char_num;

    //Save Application ID
    otas_env.appid = src_id;

    /*---------------------------------------------------*
     * Quintic private Service Creation
     *---------------------------------------------------*/

    if(rx_char_num > 1)
    {
        int i = 0;
        struct atts_desc *otas_db = NULL;
        struct atts_char_desc *char_desc_def = NULL;

        otas_db = (struct atts_desc *)ke_malloc(sizeof(otas_att_db) +
                            OTAS_MANDATORY_INCREASE_NUM * (rx_char_num - 1) * sizeof(struct atts_desc));

        char_desc_def = (struct atts_char_desc *)ke_malloc((rx_char_num - 1) * sizeof(struct atts_char_desc));

        for (i = 0; i < rx_char_num - 1; i++)
        {
            const struct atts_char_desc value_char = ATTS_CHAR(OTAS_PROP_RX_CHAR, 0,
                                                     OTAS_CHAR_RX_DATA_START_UUID + i + 1);
            char_desc_def[i] = value_char;
        }

        memcpy(otas_db, otas_att_db, sizeof(otas_att_db));

        for (i = 0; i < rx_char_num - 1; i++)
        {
            cfg_flag = (cfg_flag << OTAS_MANDATORY_INCREASE_NUM) | OTAS_MANDATORY_INCREASE_MASK;
            idx_nb += OTAS_MANDATORY_INCREASE_NUM;

            otas_db[OTAS_IDX_NB + i * OTAS_MANDATORY_INCREASE_NUM] = otas_att_db[OTAS_IDX_RX_DATA_CHAR];
            otas_db[OTAS_IDX_NB + i * OTAS_MANDATORY_INCREASE_NUM].value = (uint8_t*)&char_desc_def[i];

            otas_db[OTAS_IDX_NB + i * OTAS_MANDATORY_INCREASE_NUM + 1] = otas_att_db[OTAS_IDX_RX_DATA_VAL];
            otas_db[OTAS_IDX_NB + i * OTAS_MANDATORY_INCREASE_NUM + 1].uuid = OTAS_CHAR_RX_DATA_START_UUID + i + 1;
        }

        //Add Service Into Database
        status = atts_svc_create_db(&otas_env.shdl, (uint8_t *)&cfg_flag, idx_nb, NULL,
                                   dest_id, &otas_db[0]);
        ke_free(otas_db);
        ke_free(char_desc_def);
    }
    else
    {
        //Add Service Into Database
        status = atts_svc_create_db(&otas_env.shdl, (uint8_t *)&cfg_flag, idx_nb, NULL,
                                   dest_id, &otas_att_db[0]);
    }

    if(otas_env.rx.pdata == NULL)
    {
        otas_env.rx.capacity = rx_char_num * OTAS_RX_CHAR_PER_VOLUME;
        otas_env.rx.pdata = ke_malloc(otas_env.rx.capacity);
    }
    //Disable OTASS
    attsdb_svc_set_permission(otas_env.shdl, PERM(SVC, DISABLE));

    //Go to Idle State
    if (status == ATT_ERR_NO_ERROR)
    {
        //If we are here, database has been fulfilled with success, go to idle test
        ke_state_set(TASK_OTAS, OTAS_IDLE);
    }

    //Send response to application
    struct otas_create_db_cfm * cfm = KE_MSG_ALLOC(OTAS_CREATE_DB_CFM, otas_env.appid,
                                                   TASK_OTAS, otas_create_db_cfm);
    cfm->status = status;
    ke_msg_send(cfm);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref OTAS_ENABLE_REQ message.
 * The handler enables the Quintic private Profile.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int otas_enable_req_handler(ke_msg_id_t const msgid,
                                   struct otas_enable_req const *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    // Save the application task id
    otas_env.appid = src_id;
    // Save the connection handle associated to the profile
    otas_env.conhdl = param->conhdl;

    // If this connection is a not configuration one, apply config saved by app
    if(param->con_type == PRF_CON_NORMAL)
    {
        otas_env.features = param->ntf_en;
    }

    // Enable Service + Set Security Level
    attsdb_svc_set_permission(otas_env.shdl, param->sec_lvl);

    // Go to connected state
    ke_state_set(TASK_OTAS, OTAS_CONNECTED);
    
    
    
    

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref OTAS_DISABLE_REQ message.
 * The handler disables the Heart Rate Server Role.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int otas_disable_req_handler(ke_msg_id_t const msgid,
                                    struct otas_disable_req const *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    //Check Connection Handle
    if (param->conhdl == otas_env.conhdl)
    {
        otas_disable();
    }
    else
    {
        otas_error_ind_send(PRF_ERR_INVALID_PARAM);
    }

    return (KE_MSG_CONSUMED);
}

static void proc_data_ind(uint8_t cmd, const uint8_t *pdata, size_t len)
{
#if QN_DBG_PRINT
    static int elapse_time = 0;
#endif
    
    switch(pOTA->state)
    {
        case OTA_INITED:
            
            //QPRINTF("Start to transmit updated applicaion! time: %dms\n", (elapse_time = ke_time()) * 10);
            if(cmd == OTA_CMD_META_DATA)
                pOTA->state = proc_cmd_meta_data_write(pdata, len);
            
            break;
            
        case OTA_DATA_CARRYING:
            if(cmd == OTA_CMD_BRICK_DATA)
                pOTA->state = proc_cmd_brick_data_write(pdata, len);
            break;

        case OTA_DATA_ARRIVED:
            QPRINTF("All data recieved! elapse time: %dms\n", (ke_time() - elapse_time) * 10);
            if(cmd == OTA_CMD_DATA_VERIFY)
                pOTA->state = proc_cmd_flash_check(pdata, len);   
            break;

        case OTA_FINISHED:
            if(cmd == OTA_CMD_EXECUTION_NEW_CODE)
            {            
                ota_reset_chip();
            }
            break;

        default:
            QPRINTF("status: Unknown\n");
            break;
        }
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATT_WRITE_CMD_IND message.
 * The handler compares the new values with current ones and notifies them if they changed.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gatt_write_cmd_ind_handler(ke_msg_id_t const msgid,
                                      struct gatt_write_cmd_ind const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    uint8_t status = PRF_ERR_OK;

    if(param->conhdl == otas_env.conhdl)
    {
        ASSERT_ERR(param->length <= OTAS_RX_CHAR_PER_VOLUME);
        
        const uint8_t char_index = param->handle - (otas_env.shdl + OTAS_IDX_RX_DATA_VAL);

        if(char_index % OTAS_MANDATORY_INCREASE_NUM == 0)
        {
            const uint8_t channel = char_index / OTAS_MANDATORY_INCREASE_NUM;
#if OTAS_DEBUG
            static int time_base = -1, total_len = 0, counter = 0;

            if(time_base == -1)
                time_base = ke_time();
            total_len += param->length;

            if(++counter > 100)
            {
                const int time = ke_time() - time_base;
                QPRINTF("speed: %dB/s\n", time ? total_len*100/time : 0);
                counter = 0; time_base = -1; total_len = 0;
            }
#endif
            if(channel == 0)
            { // This channel include command
                otas_env.rx.length = ((param->value[0] << 8) | param->value[1]) + 4;
                otas_env.rx.cmd = param->value[2];
                
                if(otas_env.rx.length > otas_env.rx.capacity)
                {
                    QPRINTF("Packet length overflow rx capacity!\n");
                    transmit_error_rsp_to_client(otas_env.rx.cmd, OTA_RESULT_UNKNOWN_ERROR);
                    goto END;
                }
                
                otas_env.rx.index  = 0;

#if OTAS_DEBUG
                QPRINTF("\n==> Get packet start, rx.length=%d\n", otas_env.rx.length);
#endif
            }
            
            if(otas_env.rx.length == 0)
            {
                transmit_error_rsp_to_client(otas_env.rx.cmd, OTA_RESULT_UNKNOWN_ERROR);
                goto END;
            }
            
            if(param->length + otas_env.rx.index > otas_env.rx.capacity)
            {
                otas_env.rx.length = 0;
                QPRINTF("Packet data overflow!\n");
                transmit_error_rsp_to_client(otas_env.rx.cmd, OTA_RESULT_UNKNOWN_ERROR);
                goto END;
            }
            
            memcpy(&otas_env.rx.pdata[otas_env.rx.index], param->value, param->length);
            otas_env.rx.index += param->length;

#if OTAS_DEBUG
            QPRINTF("channel=%d, len=%d, rx.index=%d\n", channel, param->length, otas_env.rx.index);
#endif

            if(otas_env.rx.index >= otas_env.rx.length)
            {
#if OTAS_DEBUG
                for(int i=0; i<otas_env.rx.length; ++i)
                {
                    if(i%16 == 0)
                        QPRINTF("\n");
                    QPRINTF("%02X ", otas_env.rx.pdata[i]);
                }

                QPRINTF("\n<== Get packet end!\n");
#endif
                uint16_t calc_checksum = 0;
                int i;
                const uint16_t data_checksum = (otas_env.rx.pdata[otas_env.rx.length - 2] << 8) | otas_env.rx.pdata[otas_env.rx.length - 1];
                
                for(i=2; i<otas_env.rx.length-2; ++i)
                    calc_checksum += otas_env.rx.pdata[i];
                
                if(calc_checksum != data_checksum)
                {
                    QPRINTF("packet check sum fail!\n");
                    transmit_error_rsp_to_client(otas_env.rx.cmd, OTA_RESULT_CHECKSUM_ERROR);
                    goto END;
                }

                proc_data_ind(otas_env.rx.cmd, &otas_env.rx.pdata[3], otas_env.rx.length - 5);
                
                otas_env.rx.length = 0;
            }
        }
    }

END:
    if (param->response)
    {
        //Send write response
        atts_write_rsp_send(otas_env.conhdl, param->handle, status);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATT_WRITE_CMD_IND message.
 * The handler compares the new values with current ones and notifies them if they changed.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gatt_write_cmd_rsp_handler(ke_msg_id_t const msgid,
                                      struct gatt_write_cmd_ind const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    QPRINTF("gatt_write_cmd_rsp_handler: \n");
    uint8_t status = PRF_ERR_OK;

    if (param->conhdl == otas_env.conhdl)
    {
        if (param->response)
        {
            //Send write response
            atts_write_rsp_send(otas_env.conhdl, param->handle, status);
        }
    }
    return (KE_MSG_CONSUMED);
}
/**
 ****************************************************************************************
 * @brief Handles @ref GATT_NOTIFY_CMP_EVT message meaning that Measurement notification
 * has been correctly sent to peer device (but not confirmed by peer device).
 *
 * Convey this information to appli task using @ref OTAS_MEAS_SEND_CFM
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gatt_notify_cmp_evt_handler(ke_msg_id_t const msgid,
                                       struct gatt_notify_cmp_evt const *param,
                                       ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id)
{
    // Send CFM to APP that value was not sent not
    struct otas_data_send_cfm * cfm = KE_MSG_ALLOC(OTAS_DATA_SEND_CFM, otas_env.appid,
                                                   TASK_OTAS, otas_data_send_cfm);

    cfm->conhdl = otas_env.conhdl;
    cfm->char_index = (param->handle - (otas_env.shdl + OTAS_IDX_TX_DATA_VAL)) / 3;
    cfm->status = param->status;

    ke_msg_send(cfm);

   
    return (KE_MSG_CONSUMED);
}


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Disabled State handler definition.
const struct ke_msg_handler otas_disabled[] =
{
    {OTAS_CREATE_DB_REQ,        (ke_msg_func_t) otas_create_db_req_handler}
};

/// Idle State handler definition.
const struct ke_msg_handler otas_idle[] =
{
    {OTAS_ENABLE_REQ,           (ke_msg_func_t) otas_enable_req_handler}
};

/// Connected State handler definition.
const struct ke_msg_handler otas_connected[] =
{
    {GATT_WRITE_CMD_IND,    (ke_msg_func_t) gatt_write_cmd_ind_handler},
    {GATT_WRITE_CHAR_RESP,  (ke_msg_func_t) gatt_write_cmd_rsp_handler},
    {GATT_NOTIFY_CMP_EVT,   (ke_msg_func_t) gatt_notify_cmp_evt_handler},
};

/* Default State handlers definition. */
const struct ke_msg_handler otas_default_state[] =
{
    {OTAS_DISABLE_REQ,               (ke_msg_func_t) otas_disable_req_handler},
};

/// Specifies the message handler structure for every input state.
const struct ke_state_handler otas_state_handler[OTAS_STATE_MAX] =
{
    [OTAS_DISABLED]       = KE_STATE_HANDLER(otas_disabled),
    [OTAS_IDLE]           = KE_STATE_HANDLER(otas_idle),
    [OTAS_CONNECTED]      = KE_STATE_HANDLER(otas_connected),
};

/// Specifies the message handlers that are common to all states.
const struct ke_state_handler otas_default_handler = KE_STATE_HANDLER(otas_default_state);

/// Defines the place holder for the states of all the task instances.
ke_state_t otas_state[OTAS_IDX_MAX];

// Register OTASS task into kernel
void task_otas_desc_register(void)
{
    struct ke_task_desc  task_otas_desc;

    task_otas_desc.state_handler = otas_state_handler;
    task_otas_desc.default_handler=&otas_default_handler;
    task_otas_desc.state = otas_state;
    task_otas_desc.state_max = OTAS_STATE_MAX;
    task_otas_desc.idx_max = OTAS_IDX_MAX;

    task_desc_register(TASK_OTAS, task_otas_desc);
}

#endif /* #if (BLE_OTA_SERVER) */

/// @} OTASSTASK
