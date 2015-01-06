/**
 ****************************************************************************************
 *
 * @file app_qpps_task.c
 *
 * @brief Application QPPS implementation
 *
 * Copyright (C) Quintic 2009-2012
 *
 * $Rev: $
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup APP_QPPS_TASK
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
#include "app_pt.h"
#include "app_pt_gpio.h"

/// @cond
/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
struct app_qpps_env_tag *app_qpps_env = &app_env.qpps_ev;
/// @endcond

#define APP_QPP_SEND_DATA_TO  500


//static uint8_t index = 1;
//static uint32_t send_cnt = 0;

uint32_t get_bit_num(uint32_t val)
{
	uint32_t bit_cnt = 0;

	while (val != 0)
	{
		if (val & 0x1)
			bit_cnt++;
		val >>= 1;
	}
	return bit_cnt;
}

/*
 ****************************************************************************************
 * @brief Handles the create database confirmation from the QPPS.       *//**
 *
 * @param[in] msgid     QPPS_CREATE_DB_CFM
 * @param[in] param     struct qpps_create_db_cfm
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_QPPS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler will be triggered after a database creation. It contains status of database creation.
 ****************************************************************************************
 */
int app_qpps_create_db_cfm_handler(ke_msg_id_t const msgid,
                                   struct qpps_create_db_cfm *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    if (param->status == ATT_ERR_NO_ERROR)
    {
        app_clear_local_service_flag(BLE_QPP_SERVER_BIT);
    }
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the disable database indication from the QPPS.      *//**
 *
 * @param[in] msgid     QPPS_DISABLE_IND
 * @param[in] param     Pointer to the struct qpps_disable_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_QPPS
 *
 * @return If the message was consumed or not.
 * @description
 * 
 ****************************************************************************************
 */
int app_qpps_disable_ind_handler(ke_msg_id_t const msgid,
                                 struct qpps_disable_ind *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
    app_qpps_env->conhdl = 0xFFFF;
    app_qpps_env->enabled = false;
    app_qpps_env->features = 0;
    app_qpps_env->char_status = 0;
//	send_cnt = 0;
//	index = 1;

    QPRINTF("QPPS disable ind.\r\n");
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the error indication nessage from the QPPS.        *//**
 *
 * @param[in] msgid     QPPS_ERROR_IND
 * @param[in] param     Pointer to the struct qpps_error_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_QPPS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to inform the Application of an occurred error.
 ****************************************************************************************
 */
int app_qpps_error_ind_handler(ke_msg_id_t const msgid,
                               struct qpps_error_ind *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id)
{
    QPRINTF("QPPS Error indication.\r\n");
    
    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the send data confirm message from the QPPS.     *//**
 *
 * @param[in] msgid     QPPS_DATA_SEND_CFM
 * @param[in] param     Pointer to the struct qpps_data_send_cfm
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_QPPS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to report to the application a confirmation, or error status of a notification
 * request being sent by application.
 ****************************************************************************************
 */
int app_qpps_data_send_cfm_handler(ke_msg_id_t const msgid,
                                   struct qpps_data_send_cfm *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
//    static uint32_t pkt_number = 0;
	if (app_qpps_env->conhdl == param->conhdl && param->status == PRF_ERR_OK)
	{
		//QPRINTF("Cfm %d.\r\n", param->status);
		app_qpps_env->char_status |= (1 << param->char_index);
		
		///leo add
		app_pt_task_msg_hdl(msgid, param);
		///leo end
	}
	else
	{
		QPRINTF("QPPS send error %d.\r\n", param->status);
	}

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the ind/ntf indication message from the QPPS.     *//**
 *
 * @param[in] msgid     HRPS_CFG_INDNTF_IND
 * @param[in] param     Pointer to the struct qpps_cfg_indntf_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_QPPS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is usedto inform application that peer device has changed notification
 * configuration.
 ****************************************************************************************
 */
int app_qpps_cfg_indntf_ind_handler(ke_msg_id_t const msgid,
                                    struct qpps_cfg_indntf_ind *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    QPRINTF("Configure.\r\n");
    if (app_qpps_env->conhdl == param->conhdl)
    {
        if (param->cfg_val == 0x0001)//PRF_CLI_START_NTF)
        {
            app_qpps_env->features |= (QPPS_VALUE_NTF_CFG << param->char_index);
					
            if (get_bit_num(app_qpps_env->features) == QPPS_VAL_CHAR_NUM)
            {
                app_qpps_env->char_status = app_qpps_env->features;
            }
        }
        else
        {
            app_qpps_env->features &= ~(QPPS_VALUE_NTF_CFG << param->char_index);
            app_qpps_env->char_status &= ~(QPPS_VALUE_NTF_CFG << param->char_index);
        }
		
        ///leo add
        app_pt_task_msg_hdl(msgid, param);
        ///leo end
    }

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * @brief Handles the data ind message from the QPPS.       *//**
 *
 * @param[in] msgid     QPPS_DAVA_VAL_IND
 * @param[in] param     Pointer to the struct qpps_data_val_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_QPPS
 *
 * @return If the message was consumed or not.
 * @description
 *
 ****************************************************************************************
 */
int app_qpps_data_ind_handler(ke_msg_id_t const msgid,
                              struct qpps_data_val_ind const *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id)
{
		uint8_t i;
    if (param->length > 0)
    {
        QPRINTF("len=%d, I%02X", param->length, param->data[0]);
				QPRINTF("\r\n");
				for(i=0;i<param->length;i++)
					QPRINTF("  0x%02X",param->data[i]);
				app_task_msg_hdl(msgid, param);
	
        ///leo add
        struct qpps_data_val_ind* par = (struct qpps_data_val_ind*)param;
        pt_pdu_send(par->length, &(par->data[0]));
        ///leo end
			
    }
    QPRINTF("\r\n");

    return (KE_MSG_CONSUMED);
}


#endif // BLE_QPP_SERVER

/// @} APP_QPPS_TASK
