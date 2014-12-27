/**
 ****************************************************************************************
 *
 * @file app_hrpc_task.h
 *
 * @brief Application HRPC task implementation
 *
 * Copyright (C) Quintic 2009-2012
 *
 * $Rev: $
 *
 ****************************************************************************************
 */
#ifndef APP_HRPC_TASK_H_
#define APP_HRPC_TASK_H_


/**
 ****************************************************************************************
 * @addtogroup APP_HRPC_TASK Heart Rate Profile Collector Task API
 * @ingroup APP_HRPC
 * @brief Heart Rate Profile Collector Task API
 *
 * Heart Rate Profile Collector Task APIs are used to handle the message from HRPC or APP.
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#if BLE_QPP_CLIENT
#include "app_hrpc.h"
/// @cond

/// environment variable
struct app_qppc_env_tag
{
    /// Profile role state: enabled/disabled
    uint8_t enabled;
    /// Connection handle
    uint16_t conhdl;
#if QN_SVC_CONTENT_USED	
    /// HRS handle values and characteristic properties
    struct qpps_content hrs;
#endif
    uint8_t cur_code;    
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
extern struct app_qppc_env_tag *app_qppc_env;
/// @endcond

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Handles the enable confirmation from the HRPC.
 *
 ****************************************************************************************
 */
int app_hrpc_enable_cfm_handler(ke_msg_id_t const msgid,
                      struct qppc_enable_cfm *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the generic error message from the HRPC.
 *
 ****************************************************************************************
 */
int app_hrpc_error_ind_handler(ke_msg_id_t const msgid,
                      struct qppc_error_ind *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the generic message for read responses for APP.
 *
 ****************************************************************************************
 */
int app_hrpc_rd_char_rsp_handler(ke_msg_id_t const msgid,
                      struct qppc_rd_char_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the generic message for write characteristic response status to APP
 *
 ****************************************************************************************
 */
int app_hrpc_wr_char_rsp_handler(ke_msg_id_t const msgid,
                      struct qppc_wr_char_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);

/*
 ****************************************************************************************
 * @brief Handles the Heart Rate value send to APP
 *
 ****************************************************************************************
 */
int app_hrpc_meas_ind_handler(ke_msg_id_t const msgid,
                      struct qppc_data_ind *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id);
                      
/*
 ****************************************************************************************
 * @brief Handles the QPPC disable indication. *//**
 *
 * @param[in] msgid     QPPC_DISABLE_IND
 * @param[in] param     Pointer to struct prf_client_disable_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_QPPC
 * @return If the message was consumed or not.
 * @description
 *
 *
 ****************************************************************************************
 */
int app_qppc_disable_ind_handler(ke_msg_id_t const msgid,
                                 struct prf_client_disable_ind *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id);
                      
//Derek, Test
int app_hrpc_timer_handler(ke_msg_id_t const msgid,
                           void *param,
                           ke_task_id_t const dest_id,
                           ke_task_id_t const src_id);                      

#endif // BLE_QPP_CLIENT

/// @} APP_HRPC_TASK

#endif // APP_HRPC_TASK_H_
