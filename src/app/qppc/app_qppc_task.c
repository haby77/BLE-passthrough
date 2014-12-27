/**
 ****************************************************************************************
 *
 * @file app_hrpc_task.c
 *
 * @brief Application HRPC task implementation
 *
 * Copyright (C) Quintic 2009-2012
 *
 * $Rev: $
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup APP_HRPC_TASK
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

/// @cond
/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
struct app_qppc_env_tag *app_qppc_env = &app_env.qppc_ev;
/// @endcond

#define APP_SEND_DATA_TO   20 //200ms
static void app_test_send_data(void);

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Handles the enable confirmation from the HRPC. *//**
 *
 * @param[in] msgid     QPPC_ENABLE_CFM
 * @param[in] param     Pointer to struct hrpc_enable_cfm
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HRPC
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used by the Collector to either send the discovery results of HRS on the Heart Rate 
 *  and confirm enabling of the Collector role, or to simply confirm enabling of Collector role if it is a normal connection 
 *  and the attribute details are already known. 
 *
 ****************************************************************************************
 */
int app_hrpc_enable_cfm_handler(ke_msg_id_t const msgid,
                      struct qppc_enable_cfm *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("QPPC enable confirmation status: 0x%x.\r\n", param->status);

    if (param->status == CO_ERROR_NO_ERROR) 
    {
        app_qppc_env->conhdl = param->conhdl;
        app_qppc_env->enabled = true;
        app_qppc_rd_char_req(QPPC_RD_QPPS_INTPUT_VALUE_USER_DESP, param->conhdl);
        app_qppc_env->cur_code = 1;
    }
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the generic error message from the HRPC. *//**
 *
 * @param[in] msgid     QPPC_ERROR_IND
 * @param[in] param     Pointer to struct hrpc_error_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HRPC
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used by the Collector role to inform the Application of an error occurred in different 
 *  situations. The error codes are proprietary and defined in prf_types.h. An error may occur during attribute 
 *  discovery or due to application request parameters. Following reception of this message, the application will decide 
 *  the necessary action
 *
 ****************************************************************************************
 */
int app_hrpc_error_ind_handler(ke_msg_id_t const msgid,
                      struct qppc_error_ind *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("QPPC error indication status: 0x%x, cur_code is: %d\r\n", param->status, app_qppc_env->cur_code);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the generic message for read responses for APP. *//**
 *
 * @param[in] msgid     QPPC_RD_CHAR_RSP
 * @param[in] param     Pointer to struct hrpc_rd_char_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HRPC
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used by the Collector role to inform the Application of a received read response. The 
 *  status and the data from the read response are passed directly to Application, which must interpret them based on 
 *  the request it made. 
 * @note 
 * Response for read Body Sensor Location and Heart Rate Measurement Client Cfg.Desc
 *
 ****************************************************************************************
 */
int app_hrpc_rd_char_rsp_handler(ke_msg_id_t const msgid,
                      struct qppc_rd_char_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("QPPC read char response status: 0x%x.\r\n", param->status);

    switch (app_qppc_env->cur_code)
    {
    case 1:
        //app_qppc_cfg_indntf_req(PRF_CLI_START_NTF, param->conhdl);
        //app_qppc_env->cur_code = 2;
        break;
    case 2:
        break;
    default:
        break;
    }
    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the generic message for write characteristic response status to APP.  *//**
 *
 * @param[in] msgid     QPPC_WR_CHAR_RSP
 * @param[in] param     Pointer to struct hrpc_wr_char_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HRPC
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used by the Collector role to inform the Application of a received write response. The 
 *  status and the data from the write response are passed directly to Application, which must interpret them based on 
 *  the request it made.
 *
 ****************************************************************************************
 */
int app_hrpc_wr_char_rsp_handler(ke_msg_id_t const msgid,
                      struct qppc_wr_char_rsp *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    QPRINTF("QPPC write char response status: 0x%x.\r\n", param->status);

    switch (app_qppc_env->cur_code)
    {
    case 3:
        app_qppc_env->cur_code = 4;

        // Derek, start test here
 //       ke_timer_set(APP_QPPC_TIMER, TASK_APP, APP_SEND_DATA_TO);
//        app_test_send_data();
        break;
    case 4:  
        // Derek, start test here
        //ke_timer_set(APP_HRPC_TIMER, TASK_APP, APP_SEND_DATA_TO);
        //app_test_send_data();
        break;
    default:
        break;
    }
    
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the Heart Rate value send to APP. *//**
 *
 * @param[in] msgid     HRPC_MEAS_IND
 * @param[in] param     Pointer to struct hrpc_meas_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HRPC
 * @return If the message was consumed or not.
 * @description
 *
 *  This API is used by the Collector role to inform the Application of a received Heart Rate value by 
 *  notification. The application will do what it needs to do with the received measurement. No confirmation of reception 
 *  is needed because the GATT sends it directly to the peer. 
 * @note  
 *
 *  Heart Rate measurement structure refer to struct hrs_hr_meas
 *
 ****************************************************************************************
 */
int app_hrpc_meas_ind_handler(ke_msg_id_t const msgid,
                      struct qppc_data_ind *param,
                      ke_task_id_t const dest_id,
                      ke_task_id_t const src_id)
{
    // Derek, data received
    if (param->length > 0)
    {
        //QPRINTF("DI:%02X,%02X\r\n", param->meas_val.length, param->meas_val.data[0]);
        QPRINTF("len %d, I%02X", param->length, param->data[0]);
        //QPRINTF("\r\n");
    }

    app_task_msg_hdl(msgid, param);

    return (KE_MSG_CONSUMED);
}

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
                                 ke_task_id_t const src_id)
{
    QPRINTF("QPPC disable ind\r\n");
    return (KE_MSG_CONSUMED);
}


static uint8_t index = 0;
static uint8_t val[] = {0,0,1,2,3,4,5,6,7,8,9,8,7,6,5,4,3,2,1,0};
int app_hrpc_timer_handler(ke_msg_id_t const msgid,
                           void *param,
                           ke_task_id_t const dest_id,
                           ke_task_id_t const src_id)
{
    if (1)
    {
        val[0] = index++;
        app_qppc_wr_data_req(QPP_DATA_MAX_LEN, val, app_qppc_env->conhdl);
        
        //QPRINTF("DO:%02X,%02X\r\n", QPP_DATA_MAX_LEN, val[0]);
        ke_timer_set(APP_QPPC_TIMER, TASK_APP, APP_SEND_DATA_TO);
    }
    
    return (KE_MSG_CONSUMED);
}

static void app_test_send_data(void)
{
    val[0] = index++;
    app_qppc_wr_data_req(QPP_DATA_MAX_LEN, val, app_qppc_env->conhdl);
    
    //QPRINTF("DO:%02X,%02X\r\n", QPP_DATA_MAX_LEN, val[0]);
    //QPRINTF("O%02X", val[0]);
}

#endif

/// @} APP_HRPC_TASK
