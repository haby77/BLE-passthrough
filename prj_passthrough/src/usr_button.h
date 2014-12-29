#ifndef __USR_BUTTON_H_
#define __USR_BUTTON_H_

#include "gpio.h"
#include "lib.h"
#include "adc.h"

/*
 * MACRO DEFINITIONS
 ****************************************************************************************
 */
 #define		KEY_SAMPLE_NUMBER		10
 
 #define		key_up				0x01
 #define		key_right			0x02
 #define		key_down			0x04
 #define		key_left			0x08
 #define		key_center		0x10

#define EVENT_BUTTON1_PRESS_ID						0
#define EVENT_ADC_KEY_SAMPLE_CMP_ID				4
#define APP_KEY_CHEAK_PRIOD                         10   //100ms对按键轮询一次

enum button_state
{
			button_press = 0x00,
			button_release,
			button_idle,
};

typedef struct button_env_tag
{
		volatile uint8_t joystick_dir;
		enum  button_state button_st;
} button_env;

extern button_env usr_button_env;

extern void app_event_button1_press_handler(void);
extern void usr_button1_cb(void);
extern int usr_key_process_timer_handler(ke_msg_id_t const msgid, void const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int usr_key_scan_timer_handler(ke_msg_id_t const msgid, void const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern void app_event_adc_key_sample_cmp_handler(void);
#endif

