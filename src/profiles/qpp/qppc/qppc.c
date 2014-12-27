/**
 ****************************************************************************************
 *
 * @file qppc.c
 *
 * @brief Header file - Quintic Private Profile Client implementation.
 *
 * Copyright (C) Quintic 2013-2013
 *
 * $Rev: $
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup QPPC
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_config.h"

#if (BLE_QPP_CLIENT)
#include "qppc.h"
#include "qppc_task.h"

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */
struct qppc_env_tag qppc_env;

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


/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

void qppc_init(void)
{
    // Reset the Quintic Private Profile Client environment
    memset(&qppc_env, 0, sizeof(qppc_env));

    qppc_env.con_info.prf_id = TASK_QPPC;
    
    // Register QPPC task into kernel
    task_qppc_desc_register();
    
    // Go to IDLE state
    ke_state_set(TASK_QPPC, QPPC_IDLE);
}


void qppc_enable_cfm_send(uint8_t status)
{
    //send APP the details of the discovered attributes on QPPC
    struct qppc_enable_cfm * rsp = KE_MSG_ALLOC(QPPC_ENABLE_CFM,
                                                qppc_env.con_info.appid, TASK_QPPC,
                                                qppc_enable_cfm);

    rsp->conhdl = qppc_env.con_info.conhdl;
    rsp->status = status;
    rsp->qpps   = qppc_env.qpps;

    ke_msg_send(rsp);

    // Go to connected state
    ke_state_set(TASK_QPPC, QPPC_CONNECTED);

    if (status == PRF_ERR_OK)
    {
        //register QPPC task in gatt for indication/notifications
        prf_register_atthdl2gatt(&qppc_env.con_info, &qppc_env.qpps.svc);
    }
}

void qppc_error_ind_send(uint8_t status)
{
    struct qppc_error_ind *ind = KE_MSG_ALLOC(QPPC_ERROR_IND,
                                              qppc_env.con_info.appid, TASK_QPPC,
                                              qppc_error_ind);

    ind->conhdl    = qppc_env.con_info.conhdl;
    //it will be an QPPC status code
    ind->status    = status;
    // send the message
    ke_msg_send(ind);
}

void qppc_disable(void)
{
    struct prf_client_disable_ind *ind = KE_MSG_ALLOC(QPPC_DISABLE_IND,
                                                      qppc_env.con_info.appid, TASK_QPPC,
                                                      prf_client_disable_ind);

    ind->conhdl    = qppc_env.con_info.conhdl;
    ind->status    = PRF_ERR_OK;

    // Send the message
    ke_msg_send(ind);

    // Go to idle state
    ke_state_set(TASK_QPPC, QPPC_IDLE);
}

#endif /* (BLE_QPP_CLIENT) */

/// @} QPPC
