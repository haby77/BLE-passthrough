/**
 ****************************************************************************************
 *
 * @file app_pt_gpio.c
 *
 * @brief all the gpio of pass through driver
 *
 * Copyright (C) Quintic 2012-2013
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup  BUTTON
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "gpio.h"
#include "app_pt_gpio.h"
#include "lib.h"
#include "app_pt.h"
#include "sleep.h"
#if 	defined(CFG_JOYSTICK)
#include "usr_button.h"
#endif


/**
 ****************************************************************************************
 * @brief   pt_gpio_init initilization
 ****************************************************************************************
 */
void pt_gpio_init(void)
{
	//PT_STATE1 , PT_STATE0
	gpio_set_direction_field(PT_STATE0,(uint32_t)GPIO_OUTPUT);
	gpio_write_pin(PT_STATE0,GPIO_HIGH);

//	// PT_STATE_CHANGE
//    gpio_wakeup_config(PT_STATE_CHANGE, GPIO_WKUP_BY_LOW);
//    gpio_enable_interrupt(PT_STATE_CHANGE);

	// PT_TX_WAKEUP
	gpio_wakeup_config(PT_WAKEUP, GPIO_WKUP_BY_LOW);
	gpio_enable_interrupt(PT_WAKEUP);
    
    
    if(KE_EVENT_OK != ke_evt_callback_set(EVENT_GPIO_STCHANGE_ID, 
                                            app_event_gpio_stchange_handler))
    {
        ASSERT_ERR(0);
    }
    
    if(KE_EVENT_OK != ke_evt_callback_set(EVENT_GPIO_TXWAKEUP_ID, 
                                            app_event_gpio_txwakeup_handler))
    {
        ASSERT_ERR(0);
    }
}

/**
 ****************************************************************************************
 * @brief   pt_state_set
 * @param[in]    
 * @param[in]    
 * @description
 *  This function set the pt state
 ****************************************************************************************
 */
void pt_state_set(enum pt_st state)
{
	switch(state)
	{
//		case PT_DEEPSLEEP:
//		{
//			//gpio_write_pin(PT_STATE1, GPIO_LOW);
//			gpio_write_pin(PT_STATE0, GPIO_LOW);
//		}break;
//		case PT_ADV:
//		{
//			//gpio_write_pin(PT_STATE1, GPIO_LOW);
//			gpio_write_pin(PT_STATE0, GPIO_HIGH);
//		}break;
		case PT_CONN_EMPTY:
		{
			//gpio_write_pin(PT_STATE1, GPIO_HIGH);
			gpio_write_pin(PT_STATE0, GPIO_LOW);
		}break;
		case PT_CONN_FULL:
		{
			//gpio_write_pin(PT_STATE1, GPIO_HIGH);
			gpio_write_pin(PT_STATE0, GPIO_HIGH);
		}break;
		
		default :break;
	}
}

/**
 ****************************************************************************************
 * @brief Handles button press after cancel the jitter.
 *
 * @param[in] msgid     Id of the message received
 * @param[in] param     None
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_APP
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
#if (defined (CFG_PT_BOTTON))
int app_pt_gpio_timer_handler(ke_msg_id_t const msgid, void const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    switch(msgid)
    {
        case APP_SYS_GPIO_STCHANGE_TIMER:
        {
			app_pt_gpio_stchange_process();
			
        }break;

        case APP_SYS_GPIO_TXWAKEUP_TIMER:
        {
			app_pt_gpio_txwakeup_process();
			
        }break;

        default:
		ASSERT_ERR(0);
		break;
    }

    return (KE_MSG_CONSUMED);
}
#endif

void app_pt_gpio_stchange_process(void)
{
   if(usr_button_env.joystick_dir == key_up)
	{
		switch(pt_env.pt_state)
		{
			case PT_DEEPSLEEP:
			{
				if(APP_IDLE == ke_state_get(TASK_APP))
				{
					struct app_qpps_env_tag *app_qpps_env = &app_env.qpps_ev;
					if(!app_qpps_env->enabled)
					{
						// start adv
						app_gap_adv_start_req(GAP_GEN_DISCOVERABLE|GAP_UND_CONNECTABLE,
								app_env.adv_data, app_set_adv_data(GAP_GEN_DISCOVERABLE),
								app_env.scanrsp_data, app_set_scan_rsp_data(app_get_local_service_flag()),
								GAP_ADV_FAST_INTV1, GAP_ADV_FAST_INTV2);
#if (QN_DEEP_SLEEP_EN)
                        // prevent entering into deep sleep mode
                        sleep_set_pm(PM_SLEEP);
#endif
					}
				}
			}break;
			case PT_ADV:
			{
				if(APP_ADV == ke_state_get(TASK_APP))
				{
					// stop adv
					app_gap_adv_stop_req();

#if (QN_DEEP_SLEEP_EN)
                    // allow entering into deep sleep mode
                    sleep_set_pm(PM_DEEP_SLEEP);
#endif
				}
			}break;
			case PT_CONN_EMPTY:
			{
				app_gap_discon_req(0);
			}break;
			case PT_CONN_FULL:
			{
			   app_gap_discon_req(0);
			}break;
			
			default :
			{
				ASSERT_ERR(0);
			}break;
        }
    }
}

void app_pt_gpio_txwakeup_process(void)
{
	if(usr_button_env.joystick_dir == key_left)
	{		
		switch(pt_env.pt_state)
		{
			case PT_DEEPSLEEP:
			{
				
			}break;
			case PT_ADV:
			{
				
			}break;
			case PT_CONN_EMPTY:
			{
				pt_uart_rx_start();
				
			}break;
			case PT_CONN_FULL:
			{

			}break;
							
			default :break;
		}
	}
}
/**
 ****************************************************************************************
 * @brief Handles button press before key debounce.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
void app_event_gpio_stchange_handler(void)
{
    
#if ((QN_DEEP_SLEEP_EN) && (!QN_32K_RCO))
    // start 32k xtal wakeup timer
    wakeup_32k_xtal_start_timer();
#endif

    // delay 20ms to debounce
    ke_evt_clear(1UL << EVENT_GPIO_STCHANGE_ID);
	
#if (defined (CFG_PT_BOTTON))
	ke_timer_set(APP_SYS_GPIO_STCHANGE_TIMER, TASK_APP, 2);
#else
	app_pt_gpio_stchange_process();
#endif
}


/**
 ****************************************************************************************
 * @brief   Button 1 click callback
 * @description
 *  Button 1 is used to enter adv mode.
 ****************************************************************************************
 */
void gpio_stchange_cb(void)
{
    // If BLE is in the sleep mode, wakeup it.
    if(ble_ext_wakeup_allow())
    {
#if ((QN_DEEP_SLEEP_EN) && (!QN_32K_RCO))
        wakeup_32k_xtal_switch_clk();
#endif

        sw_wakeup_ble_hw();

#if (QN_DEEP_SLEEP_EN)
        // prevent deep sleep
        if(sleep_get_pm() == CPU_DEEP_SLEEP_ALLOW)
        {
            sleep_set_pm(CPU_POWER_DOWN_ALLOW);
        }
#endif
    }
    // key debounce:
    // We can set a soft timer to debounce.
    // After wakeup BLE, the timer is not calibrated immediately and it is not precise.
    // So We set a event, in the event handle, set the soft timer.
    ke_evt_set(1UL << EVENT_GPIO_STCHANGE_ID);
}

/**
 ****************************************************************************************
 * @brief Handles button press before key debounce.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
void app_event_gpio_txwakeup_handler(void)
{
    // delay 20ms to debounce
    ke_evt_clear(1UL << EVENT_GPIO_TXWAKEUP_ID);
	
#if (defined (CFG_PT_BOTTON))
	 ke_timer_set(APP_SYS_GPIO_TXWAKEUP_TIMER, TASK_APP, 2);
#else
	 app_pt_gpio_txwakeup_process();
#endif
}

/**
 ****************************************************************************************
 * @brief   Button 1 click callback
 * @description
 *  Button 1 is used to enter adv mode.
 ****************************************************************************************
 */
void gpio_txwakeup_cb(void)
{
    // If BLE is in the sleep mode, wakeup it.
    if(ble_ext_wakeup_allow())
    {
        sw_wakeup_ble_hw();

#if (QN_DEEP_SLEEP_EN)
        // prevent deep sleep
        if(sleep_get_pm() == CPU_DEEP_SLEEP_ALLOW)
        {
            sleep_set_pm(CPU_POWER_DOWN_ALLOW);
        }
#endif
    }
    // key debounce:
    // We can set a soft timer to debounce.
    // After wakeup BLE, the timer is not calibrated immediately and it is not precise.
    // So We set a event, in the event handle, set the soft timer.
    ke_evt_set(1UL << EVENT_GPIO_TXWAKEUP_ID);
}


/**
 ****************************************************************************************
 * @brief   uint8_t moduleGPIO_test(void)
 * @description
 *  MODULE GPIO TEST 
 ****************************************************************************************
 */
uint8_t module_gpio_test(void)
{
			
	uint32_t rd_value=0;
	uint8_t result=0;

	//(1) GPIO jointing test
	//p0.6-p0.7, p1.0-p1.1, p1.2-p1.3, p2.3-p2.4, p0.3-p3.1,and p3.0 for DTM select, p0.0-p0.7 for DTM communication
	
	syscon_SetPMCR0WithMask(QN_SYSCON, 
										P06_MASK_PIN_CTRL    | P07_MASK_PIN_CTRL   |
										P10_MASK_PIN_CTRL    | P11_MASK_PIN_CTRL   |
										P12_MASK_PIN_CTRL    | P13_MASK_PIN_CTRL   |
										P23_MASK_PIN_CTRL    | P24_MASK_PIN_CTRL   |
										P03_MASK_PIN_CTRL    | P31_MASK_PIN_CTRL,
										P06_GPIO_6_PIN_CTRL  | P07_GPIO_7_PIN_CTRL |
										P10_GPIO_8_PIN_CTRL  | P11_GPIO_9_PIN_CTRL |
										P12_GPIO_10_PIN_CTRL | P13_GPIO_11_PIN_CTRL|
										P23_GPIO_19_PIN_CTRL | P24_GPIO_20_PIN_CTRL|
										P03_GPIO_3_PIN_CTRL  | P31_GPIO_25_PIN_CTRL);
	//a)step1:
	gpio_set_direction_field(MODULE_GPIO_FIELD1,(uint32_t) GPIO_OUTPUT);
	gpio_set_direction_field(MODULE_GPIO_FIELD2,(uint32_t) GPIO_INPUT);
	
	gpio_write_pin_field(MODULE_GPIO_FIELD1,(uint32_t)GPIO_HIGH);
	rd_value=gpio_read_pin_field(MODULE_GPIO_FIELD2);
	if(  (rd_value & (MODULE_GPIO_FIELD2)) != (MODULE_GPIO_FIELD2) )
	result++;
	
	gpio_write_pin_field(MODULE_GPIO_FIELD1,(uint32_t)GPIO_LOW);
	rd_value=gpio_read_pin_field(MODULE_GPIO_FIELD2);
	if( (rd_value | (~(MODULE_GPIO_FIELD2))) != (~(MODULE_GPIO_FIELD2)) )
	result++;
	
	//b)step2:
	gpio_set_direction_field(MODULE_GPIO_FIELD2,(uint32_t) GPIO_OUTPUT);
	gpio_set_direction_field(MODULE_GPIO_FIELD1,(uint32_t) GPIO_INPUT);
	
	gpio_write_pin_field(MODULE_GPIO_FIELD2,(uint32_t)GPIO_HIGH);
	rd_value=gpio_read_pin_field(MODULE_GPIO_FIELD1);
	if( (rd_value & (MODULE_GPIO_FIELD1)) != (MODULE_GPIO_FIELD1))
	result++;
	
	gpio_write_pin_field(MODULE_GPIO_FIELD2,(uint32_t)GPIO_LOW);
	rd_value=gpio_read_pin_field(MODULE_GPIO_FIELD1);
	if( (rd_value | (~(MODULE_GPIO_FIELD1))) != (~(MODULE_GPIO_FIELD1)) ) 
	result++;
	
	return result;
}

/**
 ****************************************************************************************
 * @brief   uint8_t moduleGPIO_test(void)
 * @description
 *  MODULE GPIO TEST 
 ****************************************************************************************
 */
uint8_t module_32k_test(void)
{
	uint32_t rd_value;
	uint8_t result=0;


	rd_value = QN_SYSCON->BLESR;
	//sucessful 
	if( (rd_value & SYSCON_MASK_CLK_XTAL32_RDY) == SYSCON_MASK_CLK_XTAL32_RDY )
	result=0;
	//fail
	else
	result++;

	return result;
}



























