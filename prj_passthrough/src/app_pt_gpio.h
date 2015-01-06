/**
 ****************************************************************************************
 *
 * @file app_pt_gpio.h
 *
 * @brief Header file of all the gpio define for pass through
 *
 * Copyright (C) Quintic 2012-2013
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */
#ifndef _APP_PT_GPIO_H_
#define _APP_PT_GPIO_H_

#include "ke_task.h"
#include "ke_msg.h"
#include "app_pt.h"
/*
 * MACRO DEFINITIONS
 ****************************************************************************************
 */
//#define PT_STATE1				(GPIO_P23)   
#define PT_STATE0				(GPIO_P26)
//#define PT_STATE1				(GPIO_P05)   
//#define PT_STATE0				(GPIO_P04)

/*        PT_STATE1           PT_STATE0             STATE 

							0										0		            DEEP SLEEP
							0										1                   ADV
							1										0             CONNECT--BUF FULL
							1										1             CONNECT--BUF EMPERTY	 
*/

//#define PT_STATE_CHANGE (GPIO_P03)		//This pin is used to change the ble state.
//#define PT_TX_WAKEUP	(GPIO_P12)		  	//This pin is used to wakeup MCU to sent data.
//#define PT_STATE_CHANGE (GPIO_P14)			//This pin is used to change the ble state.
#define PT_WAKEUP	(GPIO_P03)		  	//This pin is used to wakeup MCU to sent data.
  

#define EVENT_GPIO_STCHANGE_ID     1
#define EVENT_GPIO_TXWAKEUP_ID     2






//#define MODULE_GPIO_FIELD1			GPIO_P10|GPIO_P12|GPIO_P23|GPIO_P03
//#define MODULE_GPIO_FIELD2			GPIO_P11|GPIO_P13|GPIO_P24|GPIO_P31
#define MODULE_GPIO_FIELD1			GPIO_P06|GPIO_P10|GPIO_P12|GPIO_P23|GPIO_P03
#define MODULE_GPIO_FIELD2			GPIO_P07|GPIO_P11|GPIO_P13|GPIO_P24|GPIO_P31

#define FUNCTION_TEST_RESULT_REG_ADDRESS      0x400000B8
#define FUNCTION_TEST_RESULT_MASK             0xF
#define FUNCTION_TEST_RESULT_RUNNING          0x8
#define FUNCTION_TEST_RESULT_SUCCESS          0x9
#define FUNCTION_TEST_RESULT_FAIL             0xA



/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */
extern void pt_gpio_init(void);
extern void pt_state_set(enum pt_st state);
extern void gpio_txwakeup_cb(void);
extern void gpio_stchange_cb(void);
extern int app_pt_gpio_timer_handler(ke_msg_id_t const msgid, void const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern void app_event_gpio_stchange_handler(void);
extern void gpio_stchange_cb(void);
extern void app_event_gpio_txwakeup_handler(void);
extern void gpio_txwakeup_cb(void);
extern uint8_t module_gpio_test(void);
extern uint8_t module_32k_test(void);
extern void app_pt_gpio_stchange_process(void);
extern void app_pt_gpio_txwakeup_process(void);


extern void button_init(void);
extern void button_cb_reg(void (*button1_cb)(void), void (*button2_cb)(void));
extern void click_button1_handle(void);
extern void click_button2_handle(void);
extern int check_button_state(int btn);

#endif

