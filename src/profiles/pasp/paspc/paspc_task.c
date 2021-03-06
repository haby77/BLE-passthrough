/**
 ****************************************************************************************
 *
 * @file paspc_task.c
 *
 * @brief Phone Alert Status Profile Client Task implementation.
 *
 * Copyright (C) RivieraWaves 2009-2013
 *
 * $ Rev $
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup PASPCTASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

 #include "app_config.h"

#if (BLE_PAS_CLIENT)

#include "gap.h"
#include "attm.h"
#include "paspc.h"
#include "paspc_task.h"
#include "gatt_task.h"

/*
 * STRUCTURES
 ****************************************************************************************
 */

/// State machine used to retrieve Phone Alert Status service characteristics information
const struct prf_char_def paspc_pass_char[PASPC_CHAR_MAX] =
{
    /// Alert Status
    [PASPC_CHAR_ALERT_STATUS]      = {ATT_CHAR_ALERT_STATUS,
                                      ATT_MANDATORY,
                                      ATT_CHAR_PROP_RD | ATT_CHAR_PROP_NTF},
    /// Ringer Setting
    [PASPC_CHAR_RINGER_SETTING]    = {ATT_CHAR_RINGER_SETTING,
                                      ATT_MANDATORY,
                                      ATT_CHAR_PROP_RD | ATT_CHAR_PROP_NTF},
    /// Ringer Control Point
    [PASPC_CHAR_RINGER_CTNL_PT]    = {ATT_CHAR_RINGER_CNTL_POINT,
                                      ATT_OPTIONAL,
                                      ATT_CHAR_PROP_WR_NO_RESP},
};

/// State machine used to retrieve Phone Alert Status service characteristic descriptor information
const struct prf_char_desc_def paspc_pass_char_desc[PASPC_DESC_MAX] =
{
    /// Alert Status Client Characteristic Configuration
    [PASPC_DESC_ALERT_STATUS_CL_CFG]   = {ATT_DESC_CLIENT_CHAR_CFG,
                                          ATT_MANDATORY,
                                          PASPC_CHAR_ALERT_STATUS},
    /// Ringer Setting Client Characteristic Configuration
    [PASPC_DESC_RINGER_SETTING_CL_CFG] = {ATT_DESC_CLIENT_CHAR_CFG,
                                          ATT_MANDATORY,
                                          PASPC_CHAR_RINGER_SETTING},
};

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref PASPC_ENABLE_CMD message.
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int paspc_enable_cmd_handler(ke_msg_id_t const msgid,
                                    struct paspc_enable_cmd const *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    // Status
    uint8_t status = PRF_ERR_OK;
    // Phone Alert Status Profile Client Role Task Environment
    struct paspc_env_tag *paspc_env;
    // Connection Information
    struct prf_con_info con_info;

    // Check if the provided connection handle is valid
    if (gap_get_rec_idx(param->conhdl) != GAP_INVALID_CONIDX)
    {
        // Fill the Connection Information structure
        con_info.conhdl = param->conhdl;
        con_info.prf_id = dest_id;
        con_info.appid  = src_id;

        // Add an environment for the provided connection
        status = PRF_CLIENT_ENABLE(con_info, param, paspc);
    }
    else
    {
        status = PRF_ERR_REQ_DISALLOWED;
    }

    if (status == PRF_ERR_OK)
    {
        paspc_env = PRF_CLIENT_GET_ENV(dest_id, paspc);

        // Keep the connection info
        memcpy(&paspc_env->con_info, &con_info, sizeof(struct prf_con_info));

        // Start discovering
        if (param->con_type == PRF_CON_DISCOVERY)
        {
            prf_disc_svc_send(&(paspc_env->con_info), ATT_SVC_PHONE_ALERT_STATUS);

            // Configure the environment for a discovery procedure
            paspc_env->operation     = PASPC_ENABLE_OP_CODE;
            paspc_env->last_uuid_req = ATT_SVC_PHONE_ALERT_STATUS;
        }
        // Bond information are provided
        else
        {
            // Keep the provided database content
            memcpy(&paspc_env->pass, &param->pass, sizeof(struct paspc_pass_content));

            // Register in GATT for notifications/indications
            prf_register_atthdl2gatt(&paspc_env->con_info, &paspc_env->pass.svc);

            // Inform the application that the profile can be used
            paspc_send_cmp_evt(dest_id, src_id, param->conhdl, PASPC_ENABLE_OP_CODE, PRF_ERR_OK);

            /* --------------------------------------------------------------------------------
             * After connection establishment, once the discovery procedure is successful,
             * the client shall read the Alert Status
             *  -------------------------------------------------------------------------------- */
            // Configure the environment for a read procedure
            paspc_env->operation      = PASPC_READ_OP_CODE;
            paspc_env->last_char_code = PASPC_RD_ALERT_STATUS;

            // Send the read request
            prf_read_char_send(&(paspc_env->con_info), paspc_env->pass.svc.shdl,
                               paspc_env->pass.svc.ehdl, paspc_env->pass.chars[PASPC_CHAR_ALERT_STATUS].val_hdl);
        }

        // Go to BUSY state
        ke_state_set(dest_id, PASPC_BUSY);
    }
    else if (status == PRF_ERR_FEATURE_NOT_SUPPORTED)
    {
        // The message has been forwarded to another task id.
        return (KE_MSG_NO_FREE);
    }
    else
    {
        // The request is disallowed (profile already enabled for this connection, or not enough memory, ...)
        paspc_send_cmp_evt(dest_id, src_id, param->conhdl, PASPC_ENABLE_OP_CODE, PRF_ERR_REQ_DISALLOWED);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref PASPC_READ_CMD message.
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int paspc_read_cmd_handler(ke_msg_id_t const msgid,
                                  struct paspc_read_cmd const *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id)
{
    // Message status
    uint8_t msg_status = KE_MSG_CONSUMED;

    // Get the address of the environment
    struct paspc_env_tag *paspc_idx_env = PRF_CLIENT_GET_ENV(dest_id, paspc);

    if (paspc_idx_env != NULL)
    {
        // Attribute Handle
        uint16_t handle    = ATT_INVALID_SEARCH_HANDLE;
        // Status
        uint8_t status     = PRF_ERR_OK;

        ASSERT_ERR(ke_state_get(dest_id) != PASPC_IDLE);

        // Check the provided connection handle
        if (param->conhdl == paspc_idx_env->con_info.conhdl)
        {
            // Check the current state
            if (ke_state_get(dest_id) == PASPC_BUSY)
            {
                // Keep the request for later, state is PRF_ERR_OK
                msg_status = KE_MSG_NO_FREE;
            }
            else    // State is PASPC_CONNECTED
            {
                switch (param->read_code)
                {
                    // Read Alert Status Characteristic Value
                    case (PASPC_RD_ALERT_STATUS):
                    {
                        handle = paspc_idx_env->pass.chars[PASPC_CHAR_ALERT_STATUS].val_hdl;
                    } break;

                    // Read Ringer Setting Characteristic Value
                    case (PASPC_RD_RINGER_SETTING):
                    {
                        handle = paspc_idx_env->pass.chars[PASPC_CHAR_RINGER_SETTING].val_hdl;
                    } break;

                    // Read Alert Status Characteristic Client Char. Cfg. Descriptor Value
                    case (PASPC_RD_WR_ALERT_STATUS_CFG):
                    {
                        handle = paspc_idx_env->pass.descs[PASPC_DESC_ALERT_STATUS_CL_CFG].desc_hdl;
                    } break;

                    // Read Ringer Setting Characteristic Client Char. Cfg. Descriptor Value
                    case (PASPC_RD_WR_RINGER_SETTING_CFG):
                    {
                        handle = paspc_idx_env->pass.descs[PASPC_DESC_RINGER_SETTING_CL_CFG].desc_hdl;
                    } break;

                    default:
                    {
                        // Handle is ATT_INVALID_SEARCH_HANDLE
                        status = PRF_ERR_INVALID_PARAM;
                    } break;
                }

                if (status == PRF_ERR_OK)
                {
                    // Check if handle is viable
                    if (handle != ATT_INVALID_SEARCH_HANDLE)
                    {
                        // Configure the environment for a read operation
                        paspc_idx_env->operation      = PASPC_READ_OP_CODE;
                        paspc_idx_env->last_char_code = param->read_code;

                        // Send the read request
                        prf_read_char_send(&(paspc_idx_env->con_info), paspc_idx_env->pass.svc.shdl,
                                           paspc_idx_env->pass.svc.ehdl, handle);

                        // Go to the Busy state
                        ke_state_set(dest_id, PASPC_BUSY);
                    }
                    else
                    {
                        status = PRF_ERR_INEXISTENT_HDL;
                    }
                }
            }
        }
        else
        {
            status = PRF_ERR_INVALID_PARAM;
        }

        if (status != PRF_ERR_OK)
        {
            paspc_send_cmp_evt(paspc_idx_env->con_info.prf_id, paspc_idx_env->con_info.appid, paspc_idx_env->con_info.conhdl,
                               PASPC_READ_OP_CODE, status);
        }
    }
    else
    {
        // No connection exists
        paspc_send_cmp_evt(dest_id, src_id, param->conhdl, PASPC_READ_OP_CODE, PRF_ERR_REQ_DISALLOWED);
    }

    return (int)msg_status;
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref PASPC_WRITE_CMD message.
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int paspc_write_cmd_handler(ke_msg_id_t const msgid,
                                   struct paspc_write_cmd const *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    // Status
    uint8_t status     = PRF_ERR_OK;
    // Message status
    uint8_t msg_status = KE_MSG_CONSUMED;

    // Get the address of the environment
    struct paspc_env_tag *paspc_idx_env = PRF_CLIENT_GET_ENV(dest_id, paspc);

    if (paspc_idx_env != NULL)
    {
        ASSERT_ERR(ke_state_get(dest_id) != PASPC_IDLE);

        // Check the provided connection handle
        if (param->conhdl == paspc_idx_env->con_info.conhdl)
        {
            // Check the current state
            if (ke_state_get(dest_id) == PASPC_BUSY)
            {
                msg_status = KE_MSG_NO_FREE;
            }
            else    // state = PASPC_CONNECTED
            {
                // Attribute handle
                uint16_t handle    = ATT_INVALID_SEARCH_HANDLE;
                // Write type
                uint8_t wr_type = 0;
                // Length
                uint8_t length = 0;

                switch (param->write_code)
                {
                    // Write Ringer Control Point Characteristic Value
                    case (PASPC_WR_RINGER_CTNL_PT):
                    {
                        if ((param->value.ringer_ctnl_pt >= PASP_SILENT_MODE) &&
                            (param->value.ringer_ctnl_pt <= PASP_CANCEL_SILENT_MODE))
                        {
                            handle  = paspc_idx_env->pass.chars[PASPC_CHAR_RINGER_CTNL_PT].val_hdl;
                            length  = sizeof(uint8_t);
                            wr_type = GATT_WRITE_NO_RESPONSE;
                        }
                        else
                        {
                            status = PRF_ERR_INVALID_PARAM;
                        }
                    } break;

                    // Write Alert Status Characteristic Client Char. Cfg. Descriptor Value
                    case (PASPC_RD_WR_ALERT_STATUS_CFG):
                    {
                        if (param->value.alert_status_ntf_cfg <= PRF_CLI_START_NTF)
                        {
                            handle  = paspc_idx_env->pass.descs[PASPC_DESC_ALERT_STATUS_CL_CFG].desc_hdl;
                            length  = sizeof(uint16_t);
                            wr_type = GATT_WRITE_CHAR;
                        }
                        else
                        {
                            status = PRF_ERR_INVALID_PARAM;
                        }
                    } break;

                    // Write Ringer Setting Characteristic Client Char. Cfg. Descriptor Value
                    case (PASPC_RD_WR_RINGER_SETTING_CFG):
                    {
                        if (param->value.ringer_setting_ntf_cfg <= PRF_CLI_START_NTF)
                        {
                            handle  = paspc_idx_env->pass.descs[PASPC_DESC_RINGER_SETTING_CL_CFG].desc_hdl;
                            length  = sizeof(uint16_t);
                            wr_type = GATT_WRITE_CHAR;
                        }
                        else
                        {
                            status = PRF_ERR_INVALID_PARAM;
                        }
                    } break;

                    default:
                    {
                        status = PRF_ERR_INVALID_PARAM;
                    } break;
                }

                if (status == PRF_ERR_OK)
                {
                    // Check if handle is viable
                    if (handle != ATT_INVALID_SEARCH_HANDLE)
                    {
                        // Send the write request
                        prf_gatt_write(&(paspc_idx_env->con_info), handle, (uint8_t *)&param->value, length, wr_type);

                        if (wr_type == GATT_WRITE_CHAR)
                        {
                            // Configure the environment for a read operation
                            paspc_idx_env->operation      = PASPC_WRITE_OP_CODE;
                            paspc_idx_env->last_char_code = param->write_code;

                            // Go to the Busy state
                            ke_state_set(dest_id, PASPC_BUSY);
                        }
                        else    // wr_type = GATT_WRITE_NO_RESPONSE
                        {
                            paspc_send_cmp_evt(dest_id, src_id, param->conhdl, PASPC_WRITE_OP_CODE, status);
                        }
                    }
                    else
                    {
                        status = PRF_ERR_INEXISTENT_HDL;
                    }
                }
            }
        }
        else
        {
            status = PRF_ERR_INVALID_PARAM;
        }
    }
    else
    {
        status = PRF_ERR_REQ_DISALLOWED;
    }

    if (status != PRF_ERR_OK)
    {
        paspc_send_cmp_evt(dest_id, src_id, param->conhdl, PASPC_WRITE_OP_CODE, status);
    }

    return (int)msg_status;
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATT_DISC_SVC_BY_UUID_CMP_EVT message.
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gatt_disc_svc_by_uuid_evt_handler(ke_msg_id_t const msgid,
                                             struct gatt_disc_svc_by_uuid_cmp_evt const *param,
                                             ke_task_id_t const dest_id,
                                             ke_task_id_t const src_id)
{
    // Get the address of the environment
    struct paspc_env_tag *paspc_env = PRF_CLIENT_GET_ENV(dest_id, paspc);

    if (paspc_env != NULL)
    {
        // Status is always ATT_ERR_NO_ERROR, keep only one instance of the service
        if (paspc_env->nb_svc == 0)
        {
            // Keep the start handle and the end handle of the service
            paspc_env->pass.svc.shdl = param->list[0].start_hdl;
            paspc_env->pass.svc.ehdl = param->list[0].end_hdl;
            paspc_env->nb_svc++;
        }
    }
    // else drop the message

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATT_DISC_CHAR_ALL_CMP_EVT message.
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gatt_disc_char_all_evt_handler(ke_msg_id_t const msgid,
                                          struct gatt_disc_char_all_cmp_evt const *param,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{
    // Get the address of the environment
    struct paspc_env_tag *paspc_env = PRF_CLIENT_GET_ENV(dest_id, paspc);

    if (paspc_env != NULL)
    {
        // Status is always ATT_ERR_NO_ERROR, Retrieve PASS characteristics
        prf_search_chars(paspc_env->pass.svc.ehdl, PASPC_CHAR_MAX,
                         &paspc_env->pass.chars[0], &paspc_pass_char[0],
                         param, (uint8_t *)&paspc_env->last_char_code);
    }
    // else drop the message

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATT_DISC_CHAR_DESC_CMP_EVT message.
 * @param[in] msgid Id of the message received
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gatt_disc_char_desc_evt_handler(ke_msg_id_t const msgid,
                                           struct gatt_disc_char_desc_cmp_evt const *param,
                                           ke_task_id_t const dest_id,
                                           ke_task_id_t const src_id)
{
    // Get the address of the environment
    struct paspc_env_tag *paspc_env = PRF_CLIENT_GET_ENV(dest_id, paspc);

    if (paspc_env != NULL)
    {
        // Status is always ATT_ERR_NO_ERROR, Retrieve PASS descriptors
        prf_search_descs(PASPC_DESC_MAX, &paspc_env->pass.descs[0], &paspc_pass_char_desc[0],
                         param, paspc_env->last_char_code);
    }
    // else drop the message

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATT_CMP_EVT message.
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gatt_cmp_evt_handler(ke_msg_id_t const msgid,
                                struct gatt_cmp_evt const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    // Status
    uint8_t status = PRF_ERR_STOP_DISC_CHAR_MISSING;
    // Get the address of the environment
    struct paspc_env_tag *paspc_env = PRF_CLIENT_GET_ENV(dest_id, paspc);

    if (paspc_env != NULL)
    {
        if ((param->status == ATT_ERR_ATTRIBUTE_NOT_FOUND) ||
            (param->status == ATT_ERR_NO_ERROR))
        {
            /* -------------------------------------------------
             * SERVICE DISCOVERY -------------------------------
             * ------------------------------------------------- */
            if (paspc_env->last_uuid_req == ATT_SVC_PHONE_ALERT_STATUS)
            {
                if (paspc_env->nb_svc > 0)
                {
                    // Check if service handles are OK
                    if ((paspc_env->pass.svc.shdl != ATT_INVALID_HANDLE) &&
                        (paspc_env->pass.svc.ehdl != ATT_INVALID_HANDLE) &&
                        (paspc_env->pass.svc.shdl < paspc_env->pass.svc.ehdl))
                    {
                        status = PRF_ERR_OK;

                        // Discover all PASS characteristics
                        prf_disc_char_all_send(&(paspc_env->con_info), &(paspc_env->pass.svc));
                        paspc_env->last_uuid_req = ATT_DECL_CHARACTERISTIC;
                    }
                    // Handles are not corrects, the Phone Alert Status Service has not been found
                }
                // The Phone Alert Status Service has not been found
            }

            /* -------------------------------------------------
             * CHARACTERISTICS DISCOVERY -----------------------
             * ------------------------------------------------- */
            else if (paspc_env->last_uuid_req == ATT_DECL_CHARACTERISTIC)
            {
                // Check if mandatory properties have been found and if properties are correct
                status = prf_check_svc_char_validity(PASPC_CHAR_MAX, paspc_env->pass.chars, paspc_pass_char);

                // Check for characteristic properties.
                if (status == PRF_ERR_OK)
                {
                    paspc_env->last_uuid_req  = ATT_INVALID_HANDLE;
                    paspc_env->last_char_code = PASPC_CHAR_ALERT_STATUS;

                    // Find the Client Characteristic Configuration Descriptor for the Alert Status characteristic
                    prf_disc_char_desc_send(&(paspc_env->con_info), &(paspc_env->pass.chars[PASPC_CHAR_ALERT_STATUS]));
                }
            }

            /* -------------------------------------------------
             * DESCRIPTORS DISCOVERY ---------------------------
             * ------------------------------------------------- */
            else
            {
                ASSERT_ERR(paspc_env->last_uuid_req == ATT_INVALID_HANDLE);

                if (paspc_env->last_char_code == PASPC_CHAR_ALERT_STATUS)
                {
                    status = PRF_ERR_OK;

                    paspc_env->last_uuid_req  = ATT_INVALID_HANDLE;
                    paspc_env->last_char_code = PASPC_CHAR_RINGER_SETTING;

                    // Find the Client Characteristic Configuration Descriptor for the Ringer Setting characteristic
                    prf_disc_char_desc_send(&(paspc_env->con_info), &(paspc_env->pass.chars[PASPC_CHAR_RINGER_SETTING]));
                }
                else
                {
                    ASSERT_ERR(paspc_env->last_char_code == PASPC_CHAR_RINGER_SETTING);

                    status = prf_check_svc_char_desc_validity(PASPC_DESC_MAX,
                                                              paspc_env->pass.descs,
                                                              paspc_pass_char_desc,
                                                              paspc_env->pass.chars);

                    if (status == PRF_ERR_OK)
                    {
                        // Reset number of services
                        paspc_env->nb_svc = 0;

                        // Register in GATT for notifications/indications
                        prf_register_atthdl2gatt(&paspc_env->con_info, &paspc_env->pass.svc);

                        // Send the content of the service to the HL
                        struct paspc_pass_content_ind *ind = KE_MSG_ALLOC(PASPC_PASS_CONTENT_IND,
                                                                          paspc_env->con_info.appid,
                                                                          paspc_env->con_info.prf_id,
                                                                          paspc_pass_content_ind);

                        ind->conhdl = paspc_env->con_info.conhdl;
                        memcpy(&ind->pass, &paspc_env->pass, sizeof(struct paspc_pass_content));

                        ke_msg_send(ind);

                        /* --------------------------------------------------------------------------------
                         * After connection establishment, once the discovery procedure is successful,
                         * the client shall read the Alert Status
                         *  -------------------------------------------------------------------------------- */
                        // Configure the environment for a read procedure
                        paspc_env->last_char_code = PASPC_RD_ALERT_STATUS;

                        // Send the read request
                        prf_read_char_send(&(paspc_env->con_info), paspc_env->pass.svc.shdl,
                                           paspc_env->pass.svc.ehdl, paspc_env->pass.chars[PASPC_CHAR_ALERT_STATUS].val_hdl);
                    }
                }
            }
        }
        else
        {
            status = param->status;
        }

        if (status != PRF_ERR_OK)
        {
            // Inform HL about the error
            paspc_send_cmp_evt(paspc_env->con_info.prf_id, paspc_env->con_info.appid,
                               paspc_env->con_info.conhdl, PASPC_ENABLE_OP_CODE, status);
        }
    }
    // else ignore the message

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATT_READ_CHAR_RESP message.
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gatt_rd_char_rsp_handler(ke_msg_id_t const msgid,
                                    struct gatt_read_char_resp const *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    // Get the address of the environment
    struct paspc_env_tag *paspc_env = PRF_CLIENT_GET_ENV(dest_id, paspc);

    if (paspc_env != NULL)
    {
        // Status
        uint8_t status = PRF_ERR_OK;

        ASSERT_ERR(ke_state_get(dest_id) == PASPC_BUSY);

        if (param->status == ATT_ERR_NO_ERROR)
        {
            // Prepare the indication message for the HL
            struct paspc_value_ind *ind = KE_MSG_ALLOC(PASPC_VALUE_IND,
                                                       paspc_env->con_info.appid,
                                                       paspc_env->con_info.prf_id,
                                                       paspc_value_ind);

            ind->conhdl = paspc_env->con_info.conhdl;

            switch (paspc_env->last_char_code)
            {
                // Read Alert Status Characteristic Value
                case (PASPC_RD_ALERT_STATUS):
                {
                    ind->value.alert_status = param->data.data[0];
                } break;

                // Read Ringer Setting Characteristic Value
                case (PASPC_RD_RINGER_SETTING):
                {
                    ind->value.ringer_setting = param->data.data[0];
                } break;

                // Read Alert Status Characteristic Client Char. Cfg. Descriptor Value
                case (PASPC_RD_WR_ALERT_STATUS_CFG):
                {
                    ind->value.alert_status_ntf_cfg = co_read16p(&param->data.data[0]);
                } break;

                // Read Ringer Setting Characteristic Client Char. Cfg. Descriptor Value
                case (PASPC_RD_WR_RINGER_SETTING_CFG):
                {
                    ind->value.ringer_setting_ntf_cfg = co_read16p(&param->data.data[0]);
                } break;

                default:
                {
                    ASSERT_ERR(0);
                } break;
            }

            ind->att_code = paspc_env->last_char_code;

            // Send the message
            ke_msg_send(ind);
        }
        else
        {
            status = param->status;
        }

        // Send a complete event status to the HL
        paspc_send_cmp_evt(paspc_env->con_info.prf_id, paspc_env->con_info.appid,
                           paspc_env->con_info.conhdl, paspc_env->operation, status);
    }
    // else ignore the message

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATT_WRITE_CHAR_RESP message.
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gatt_write_char_rsp_handler(ke_msg_id_t const msgid,
                                       struct gatt_write_char_resp const *param,
                                       ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id)
{
    // Get the address of the environment
    struct paspc_env_tag *paspc_env = PRF_CLIENT_GET_ENV(dest_id, paspc);

    if (paspc_env != NULL)
    {
        ASSERT_ERR(ke_state_get(dest_id) == PASPC_BUSY);
        ASSERT_ERR(paspc_env->operation == PASPC_WRITE_OP_CODE);

        // Send a complete event status to the HL
        paspc_send_cmp_evt(paspc_env->con_info.prf_id, paspc_env->con_info.appid,
                           paspc_env->con_info.conhdl, PASPC_WRITE_OP_CODE, param->status);
    }
    // else ignore the message

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATT_HANDLE_VALUE_NTF message.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gatt_handle_value_ntf_handler(ke_msg_id_t const msgid,
                                        struct gatt_handle_value_notif const *param,
                                        ke_task_id_t const dest_id,
                                        ke_task_id_t const src_id)
{
    // Get the address of the environment
    struct paspc_env_tag *paspc_env = PRF_CLIENT_GET_ENV(dest_id, paspc);

    if (paspc_env != NULL)
    {
        ASSERT_ERR((param->charhdl == paspc_env->pass.chars[PASPC_CHAR_ALERT_STATUS].val_hdl) ||
                   (param->charhdl == paspc_env->pass.chars[PASPC_CHAR_RINGER_SETTING].val_hdl));

        // Prepare the indication message for the HL
        struct paspc_value_ind *ind = KE_MSG_ALLOC(PASPC_VALUE_IND,
                                                   paspc_env->con_info.appid,
                                                   paspc_env->con_info.prf_id,
                                                   paspc_value_ind);

        ind->conhdl = paspc_env->con_info.conhdl;

        // Alert Status Characteristic Value
        if (param->charhdl == paspc_env->pass.chars[PASPC_CHAR_ALERT_STATUS].val_hdl)
        {
            ind->value.alert_status = param->value[0];
            ind->att_code = PASPC_RD_ALERT_STATUS;
        }
        // Ringer Setting Characteristic Value
        else
        {
            ind->value.ringer_setting = param->value[0];
            ind->att_code = PASPC_RD_RINGER_SETTING;
        }

        // Send the message
        ke_msg_send(ind);
    }
    // else ignore the message

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Disconnection indication to PASPC.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gap_discon_cmp_evt_handler(ke_msg_id_t const msgid,
                                      struct gap_discon_cmp_evt const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    PRF_CLIENT_DISABLE_IND_SEND(paspc_envs, dest_id, PASPC);

    return (KE_MSG_CONSUMED);
}

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Specifies the default message handlers
const struct ke_msg_handler paspc_default_state[] =
{
    {PASPC_ENABLE_CMD,              (ke_msg_func_t)paspc_enable_cmd_handler},
    {PASPC_READ_CMD,                (ke_msg_func_t)paspc_read_cmd_handler},
    {PASPC_WRITE_CMD,               (ke_msg_func_t)paspc_write_cmd_handler},

    {GATT_DISC_SVC_BY_UUID_CMP_EVT, (ke_msg_func_t)gatt_disc_svc_by_uuid_evt_handler},
    {GATT_DISC_CHAR_ALL_CMP_EVT,    (ke_msg_func_t)gatt_disc_char_all_evt_handler},
    {GATT_DISC_CHAR_DESC_CMP_EVT,   (ke_msg_func_t)gatt_disc_char_desc_evt_handler},
    {GATT_CMP_EVT,                  (ke_msg_func_t)gatt_cmp_evt_handler},

    {GATT_READ_CHAR_RESP,           (ke_msg_func_t)gatt_rd_char_rsp_handler},
    {GATT_WRITE_CHAR_RESP,          (ke_msg_func_t)gatt_write_char_rsp_handler},
    {GATT_HANDLE_VALUE_NOTIF,       (ke_msg_func_t)gatt_handle_value_ntf_handler},

    {GAP_DISCON_CMP_EVT,            (ke_msg_func_t)gap_discon_cmp_evt_handler},
};

// Specifies the message handler structure for every input state.
const struct ke_state_handler paspc_state_handler[PASPC_STATE_MAX] =
{
    [PASPC_IDLE]        = KE_STATE_HANDLER_NONE,
    [PASPC_CONNECTED]   = KE_STATE_HANDLER_NONE,
    [PASPC_BUSY]        = KE_STATE_HANDLER_NONE,
};

// Specifies the message handlers that are common to all states.
const struct ke_state_handler paspc_default_handler = KE_STATE_HANDLER(paspc_default_state);

// Defines the place holder for the states of all the task instances.
ke_state_t paspc_state[PASPC_IDX_MAX];

// Registet PASPC task into kernel
void task_paspc_desc_register(void)
{
    struct ke_task_desc task_paspc_desc;
    
    task_paspc_desc.state_handler = paspc_state_handler;
    task_paspc_desc.default_handler= &paspc_default_handler;
    task_paspc_desc.state = paspc_state;
    task_paspc_desc.state_max = PASPC_STATE_MAX;
    task_paspc_desc.idx_max = PASPC_IDX_MAX;
    
    task_desc_register(TASK_PASPC, task_paspc_desc);
}
#endif //(BLE_PAS_CLIENT)

/// @} PASPCTASK
