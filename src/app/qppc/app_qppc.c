/**
 ****************************************************************************************
 *
 * @file app_qppc.c
 *
 * @brief Application QPPC API
 *
 * Copyright (C) Quintic 2009-2012
 *
 * $Rev: $
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup APP_QPPC_API
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"

#if BLE_QPP_CLIENT
#include "app_qppc.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Start the profile - at connection. *//**
 * @param[in] qpps          
 * @param[in] conhdl        Connection handle for which the profile client role is enabled.
 * @response  QPPC_ENABLE_CFM
 * @description
 *
 ****************************************************************************************
 */
void app_qppc_enable_req(struct qpps_content *qpps, uint16_t conhdl)
{
    struct qppc_enable_req * msg = KE_MSG_ALLOC(QPPC_ENABLE_REQ, TASK_QPPC, TASK_APP,
                                                 qppc_enable_req);

    ///Connection handle
    msg->conhdl = conhdl;
    ///Connection type
    if (qpps == NULL)
    {
        msg->con_type = PRF_CON_DISCOVERY;
    }
    else
    {
        msg->con_type = PRF_CON_NORMAL;
        memcpy(&msg->qpps, qpps, sizeof(struct qpps_content));
    }

    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Generic message to read a QPPS characteristic value. *//**
 * @param[in] char_code     Code for which characteristic to read:
 *  - 
 * @param[in] conhdl        Connection handle for which the profile Heart Rate Collector role is enabled.
 * @response  QPPC_RD_CHAR_RSP or QPPC_ERROR_IND
 * @description
 *
 ****************************************************************************************
 */
void app_qppc_rd_char_req(uint8_t char_code, uint16_t conhdl)
{
    struct qppc_rd_char_req *msg = KE_MSG_ALLOC(QPPC_RD_CHAR_REQ, TASK_QPPC, TASK_APP,
                                                qppc_rd_char_req);
    ///Connection handle
    msg->conhdl = conhdl;
    ///Characteristic value code
    msg->char_code = char_code;

    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Generic message for configuring the 2 characteristics that can be handled. *//**
 * @param[in] cfg_val       Stop/notify/indicate value to configure into the peer characteristic.
 *  - PRF_CLI_STOP_NTFIND
 *  - PRF_CLI_START_NTF
 *  - PRF_CLI_START_IND
 * @param[in] conhdl        Connection handle
 * @response  QPPC_WR_CHAR_RSP or QPPC_ERROR_IND
 * @description
 *

 ****************************************************************************************
 */
void app_qppc_cfg_indntf_req(uint16_t cfg_val, uint16_t conhdl)
{
    struct qppc_cfg_indntf_req *msg = KE_MSG_ALLOC(QPPC_CFG_INDNTF_REQ, TASK_QPPC, TASK_APP,
                                                   qppc_cfg_indntf_req);

    ///Connection handle
    msg->conhdl = conhdl;
    ///Stop/notify/indicate value to configure into the peer characteristic
    msg->cfg_val = cfg_val;
    
    // Send the message
    ke_msg_send(msg);
}

/*
 ****************************************************************************************
 * @brief Send data request. *//**
 * @param[in] val           data.
 * @param[in] conhdl        Connection handle
 * @response  QPPC_WR_CHAR_RSP or QPPC_ERROR_IND
 * @description
 *
 ****************************************************************************************
 */
void app_qppc_wr_data_req(uint8_t len, uint8_t *val, uint16_t conhdl)
{
    struct qppc_wr_data_req *msg = KE_MSG_ALLOC_DYN(QPPC_WR_DATA_REQ, TASK_QPPC, TASK_APP,
                                                qppc_wr_data_req, len);

    ///Connection handle
    msg->conhdl = conhdl;
    msg->length = len;
    memcpy(msg->data, val, msg->length);

    // Send the message
    ke_msg_send(msg);
}

#endif 

/// @} APP_QPPC_API
