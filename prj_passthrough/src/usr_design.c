/**
 ****************************************************************************************
 *
 * @file usr_design.c
 *
 * @brief Product related design.
 *
 * Copyright (C) Quintic 2012-2013
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup  USR
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <stdint.h>
#include "app_env.h"
#include "uart.h"
#include "lib.h"
#include "usr_design.h"
#include "gpio.h"
#include "button.h"
#if (defined(QN_ADV_WDT))
#include "wdt.h"
#endif
#include "sleep.h"

#if(defined(QN_PT))
#include "app_pt.h"
#include "app_pt_gpio.h"
#endif

#include "usr_task.h"
#include "led.h"
/*
 * MACRO DEFINITIONS
 ****************************************************************************************
 */

#define LED_ON_DUR_ADV_FAST        2
#define LED_OFF_DUR_ADV_FAST       (uint16_t)((GAP_ADV_FAST_INTV2*0.625)/10)
#define LED_ON_DUR_ADV_SLOW        2
#define LED_OFF_DUR_ADV_SLOW       (uint16_t)((GAP_ADV_SLOW_INTV*0.625)/10)
#define LED_ON_DUR_CON             0xffff
#define LED_OFF_DUR_CON            0
#define LED_ON_DUR_IDLE            0
#define LED_OFF_DUR_IDLE           0xffff



/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
#if (defined(QN_ADV_WDT))
static void adv_wdt_to_handler(void)
{
    ke_state_set(TASK_APP, APP_IDLE);

    // start adv
    app_gap_adv_start_req(GAP_GEN_DISCOVERABLE|GAP_UND_CONNECTABLE,
                          app_env.adv_data, app_set_adv_data(GAP_GEN_DISCOVERABLE),
                          app_env.scanrsp_data, app_set_scan_rsp_data(app_get_local_service_flag()),
                          GAP_ADV_FAST_INTV1, GAP_ADV_FAST_INTV2);
}
struct usr_env_tag usr_env = {LED_ON_DUR_IDLE, LED_OFF_DUR_IDLE, false, adv_wdt_to_handler};
#else
struct usr_env_tag usr_env = {LED_ON_DUR_IDLE, LED_OFF_DUR_IDLE};
#endif

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief   Application task message handler
 ****************************************************************************************
 */
void app_task_msg_hdl(ke_msg_id_t const msgid, void const *param)
{
    switch(msgid)
    {
        case GAP_SET_MODE_REQ_CMP_EVT:
            if(APP_IDLE == ke_state_get(TASK_APP))
            {
///leo add							
#if (defined(QN_PT))
				app_pt_task_msg_hdl(msgid, param);
#endif
///leo end
#if (defined(QN_ADV_WDT))
                usr_env.adv_wdt_enable = true;
#endif
            }
            else if(APP_ADV == ke_state_get(TASK_APP))
            {
#if (defined(QN_ADV_WDT))
                usr_env.adv_wdt_enable = true;
#endif
            }
            break;

        case GAP_ADV_REQ_CMP_EVT:
            ke_timer_clear(APP_ADV_INTV_UPDATE_TIMER, TASK_APP);
///leo add
#if (defined(QN_PT))
								app_pt_task_msg_hdl(msgid, param);
#endif
///leo end
            break;

        case GAP_DISCON_CMP_EVT:
///leo add									
#if (defined(QN_PT))
                    app_pt_task_msg_hdl(msgid, param);
#endif
///leo end
            break;

        case GAP_LE_CREATE_CONN_REQ_CMP_EVT:
            if(((struct gap_le_create_conn_req_cmp_evt *)param)->conn_info.status == CO_ERROR_NO_ERROR)
            {
                if(GAP_PERIPHERAL_SLV == app_get_role())
                {
                    ke_timer_clear(APP_ADV_INTV_UPDATE_TIMER, TASK_APP);
#if (defined(QN_ADV_WDT))
                    usr_env.adv_wdt_enable = false;
#endif

///leo add									
#if (defined(QN_PT))
                    app_pt_task_msg_hdl(msgid, param);
#endif
///leo end
                    // Update cnx parameters
//                    if (((struct gap_le_create_conn_req_cmp_evt *)param)->conn_info.con_interval < GAP_PPCP_CONN_INTV_MIN)
                    {
                        // Update connection parameters here
                        struct gap_conn_param_update conn_par;
                        /// Connection interval minimum
                        conn_par.intv_min = GAP_PPCP_CONN_INTV_MIN;
                        /// Connection interval maximum
                        conn_par.intv_max = GAP_PPCP_CONN_INTV_MAX;
                        /// Latency
                        conn_par.latency = GAP_PPCP_SLAVE_LATENCY;
                        /// Supervision timeout, Time = N * 10 msec
                        conn_par.time_out = GAP_CONN_SUPERV_TIMEOUT;
                        app_gap_param_update_req(((struct gap_le_create_conn_req_cmp_evt *)param)->conn_info.conhdl, &conn_par);
                    }
                }
            }
            break;

        default:
            break;
    }
}



///leo add
int app_adv_stop_timer_handler(ke_msg_id_t const msgid, void const *param,
                                          ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    if(APP_ADV == ke_state_get(TASK_APP))
    {
        // stop adv
        app_gap_adv_stop_req();

#if (QN_DEEP_SLEEP_EN)
        // allow entering into deep sleep mode
        sleep_set_pm(CPU_DEEP_SLEEP_ALLOW);
#endif
    }
    return (KE_MSG_CONSUMED);
}
///leo end
/**
 ****************************************************************************************
 * @brief Handles advertising mode timer event.
 *
 * @param[in] msgid     APP_ADV_INTV_UPDATE_TIMER
 * @param[in] param     None
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_APP
 *
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application that first phase of adversting mode is timeout.
 ****************************************************************************************
 */
int app_gap_adv_intv_update_timer_handler(ke_msg_id_t const msgid, void const *param,
                                          ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    if(APP_ADV == ke_state_get(TASK_APP))
    {
        // Update Advertising Parameters
        app_gap_adv_start_req(GAP_GEN_DISCOVERABLE|GAP_UND_CONNECTABLE,
                                app_env.adv_data, app_set_adv_data(GAP_GEN_DISCOVERABLE),
                                app_env.scanrsp_data, app_set_scan_rsp_data(app_get_local_service_flag()),
                                GAP_ADV_SLOW_INTV, GAP_ADV_SLOW_INTV);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief   Restore peripheral setting after wakeup
 ****************************************************************************************
 */
void usr_sleep_restore(void)
{
#if QN_DBG_PRINT
    uart_init(QN_DEBUG_UART, USARTx_CLK(0), UART_9600);
    uart_tx_enable(QN_DEBUG_UART, MASK_ENABLE);
    uart_rx_enable(QN_DEBUG_UART, MASK_ENABLE);
#endif
///leo add
#if (defined(QN_PT))	
    uart_init(QN_HCI_UART, USARTx_CLK(0), UART_9600);
    uart_tx_enable(QN_HCI_UART, MASK_ENABLE);
    uart_rx_enable(QN_HCI_UART, MASK_ENABLE);
#endif
///leo end

#if (defined(QN_ADV_WDT))
    if(usr_env.adv_wdt_enable)
    {
        wdt_init(1007616, WDT_INT_MOD); // 30.75s
    }
#endif
}

/**
 ****************************************************************************************
 * @brief   All GPIO interrupt callback
 ****************************************************************************************
 */
void gpio_interrupt_callback(enum gpio_pin pin)
{
    switch(pin)
    {
//        case PT_STATE_CHANGE:
//				{
//            //Button 1 is used to enter adv mode.
//            gpio_stchange_cb();
//					
//				}break;
        case PT_WAKEUP:
				{
            gpio_write_pin(LED1_PIN,GPIO_LOW);
						usr_button1_cb();
						//gpio_txwakeup_cb();
					
				}break;

#if (defined(QN_TEST_CTRL_PIN))
        case QN_TEST_CTRL_PIN:
				{
            //When test controll pin is changed to low level, this function will reboot system.
            gpio_disable_interrupt(QN_TEST_CTRL_PIN);
            syscon_SetCRSS(QN_SYSCON, SYSCON_MASK_REBOOT_SYS);
				}
            break;
#endif

        default:
            break;
    }
}

/**
 ****************************************************************************************
 * @brief   User initialize
 ****************************************************************************************
 */
void usr_init(void)
{
	
	 task_usr_desc_register();  
    ke_state_set(TASK_USR, USR_DISABLE);
	
	if(KE_EVENT_OK != ke_evt_callback_set(EVENT_BUTTON1_PRESS_ID,
                                            app_event_button1_press_handler))
    {
        ASSERT_ERR(0);
    }
	
	  if(KE_EVENT_OK != ke_evt_callback_set(EVENT_ADC_KEY_SAMPLE_CMP_ID,
                                            app_event_adc_key_sample_cmp_handler))
    {
        ASSERT_ERR(0);
    }
}

/// @} USR
