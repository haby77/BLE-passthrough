/**
 ****************************************************************************************
 *
 * @file app_qpps.c
 *
 * @brief Application QPPS API
 *
 * Copyright (C) Quintic 2009-2012
 *
 * $Rev: $
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup APP_QPPS_API
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
 
#include "app_env.h"

#if BLE_QPP_SERVER
#include "app_qpps.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

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
void app_qpps_create_db(uint8_t num)
{
    struct qpps_create_db_req * msg = KE_MSG_ALLOC(QPPS_CREATE_DB_REQ, TASK_QPPS, TASK_APP, qpps_create_db_req);

    msg->tx_char_num = num;
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
void app_qpps_enable_req(uint16_t conhdl, uint8_t sec_lvl, uint8_t con_type, uint16_t ntf_en)
{
    struct qpps_enable_req * msg = KE_MSG_ALLOC(QPPS_ENABLE_REQ, TASK_QPPS, TASK_APP,
                                                qpps_enable_req);

    msg->conhdl = conhdl;
    msg->sec_lvl = sec_lvl;
    msg->con_type = con_type;
    msg->ntf_en = ntf_en;

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
void app_qpps_data_send(uint16_t conhdl, uint8_t index, uint8_t length, uint8_t const *data)
{
    struct qpps_data_send_req * msg = KE_MSG_ALLOC_DYN(QPPS_DATA_SEND_REQ, TASK_QPPS, TASK_APP,
                                                       qpps_data_send_req, length);

    if (msg != NULL)
    {
        //QPRINTF("----\r\n");
        msg->conhdl = conhdl;
				msg->index = index;
        msg->length = length;
        memcpy(msg->data, data, length);

        ke_msg_send(msg);
    }
}

#endif // BLE_QPP_SERVER

/// @} APP_QPPS_API
