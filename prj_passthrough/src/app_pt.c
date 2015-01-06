/**
 ****************************************************************************************
 *
 * @file app_pt.c
 *
 * @brief Pass through project process
 *
 * Copyright (C) Quintic 2012-2013
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"
#include "app_pt.h"
#include "app_pt_gpio.h"
#include "uart.h"
#include "lib.h"

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */
 
 struct pt_env_tag  pt_env;
 extern uint32_t get_bit_num(uint32_t val);

 
 void pt_init(void)
 {
    pt_env.pt_state = PT_DEEPSLEEP;
    pt_state_set(PT_DEEPSLEEP);

    //for pt uart tx  
    pt_env.tx_state = PT_UART_TX_IDLE;	//initialize tx state
    co_list_init(&pt_env.queue_tx);			//init TX queue
     
    if(KE_EVENT_OK != ke_evt_callback_set(EVENT_UART_TX_ID, pt_tx_done))
    ASSERT_ERR(0);
    if(KE_EVENT_OK != ke_evt_callback_set(EVENT_UART_RX_FRAME_ID, pt_event_uart_rx_frame_handler))
    ASSERT_ERR(0);
    if(KE_EVENT_OK != ke_evt_callback_set(EVENT_UART_RX_TIMEOUT_ID, pt_event_uart_rx_timeout_handler))
    ASSERT_ERR(0);

 }
 

void app_pt_task_msg_hdl(ke_msg_id_t const msgid, void const *param)
{
    switch(msgid)
    {
			
		case GAP_SET_MODE_REQ_CMP_EVT:
		{
			ke_timer_set(APP_ADV_STOP_TIMER, TASK_APP, 50 * 100);
			pt_env.pt_state = PT_ADV;
			pt_state_set(PT_ADV);

		}break;
		case GAP_ADV_REQ_CMP_EVT:
		{
			pt_env.pt_state = PT_DEEPSLEEP;
			pt_state_set(PT_DEEPSLEEP);
			
		}break;
					
        case GAP_LE_CREATE_CONN_REQ_CMP_EVT:// The module has been connected with the extern machine,but the notifications aren't configed.
        {
            pt_env.pt_state = PT_CONN_FULL;
            pt_state_set(PT_CONN_FULL);
			
        }break;
        case QPPS_CFG_INDNTF_IND://all of the notifications have been confige to 1,indicate the state to connected and empty 
        {
            uint8_t bit_num = get_bit_num(app_qpps_env->char_status);
            if (bit_num >= QPPS_VAL_CHAR_NUM)  
            {                  
                pt_env.pt_state = PT_CONN_EMPTY;
                pt_state_set(PT_CONN_EMPTY);
								pt_uart_rx_start();
            }
			else
			{
				pt_env.pt_state = PT_CONN_FULL;
				pt_state_set(PT_CONN_FULL);
			}
            
        }break;
		case QPPS_DATA_SEND_CFM://all the data have send through ble 
		{
			uint8_t bit_num = get_bit_num(app_qpps_env->char_status);
			if (bit_num >= QPPS_VAL_CHAR_NUM)
			{
				pt_uart_rx_start();
				pt_env.pt_state = PT_CONN_EMPTY;
				pt_state_set(PT_CONN_EMPTY);
			}
			//								ke_evt_set(1UL << EVENT_GPIO_TXWAKEUP_ID);

		}break;
		case GAP_DISCON_CMP_EVT://after disconnect with the equipment,the module will advertising auto.
		{
			///clear all event or timer
			ke_evt_clear(1UL << EVENT_UART_RX_FRAME_ID);
			ke_evt_clear(1UL << EVENT_UART_RX_TIMEOUT_ID);
			ke_timer_clear(APP_PT_RX_TIMEOUT_TIMER, TASK_APP);
			uart_rx_int_enable(QN_HCI_UART, MASK_DISABLE);  //disable uart rx interrupt 
			
//			pt_env.pt_state = PT_DEEPSLEEP;
//			pt_state_set(PT_DEEPSLEEP);
			
			// start adv
			app_gap_adv_start_req(GAP_GEN_DISCOVERABLE|GAP_UND_CONNECTABLE,
							app_env.adv_data, app_set_adv_data(GAP_GEN_DISCOVERABLE),
							app_env.scanrsp_data, app_set_scan_rsp_data(app_get_local_service_flag()),
							GAP_ADV_FAST_INTV1, GAP_ADV_FAST_INTV2);
		}break;

        default :break;
    }
}

void pt_uart_rx_start(void)
{
		//QPRINTF("pt_uart_rx_start\r\n");
    pt_env.pt_rx_len = 0;
    uart_read(QN_COM_UART, &pt_env.pt_rx_buf[pt_env.pt_rx_len], 1, pt_uart_rx);
}

 
void pt_uart_rx(void)
{
    pt_env.pt_rx_len++;
    //QPRINTF("pt_env.pt_rx_len %d\r\n",pt_env.pt_rx_len);
    //set pt gpio state 
		if (pt_env.pt_rx_buf[0] == 0x02 )
    pt_env.pt_state = PT_CONN_FULL;
		pt_state_set(PT_CONN_FULL);
		
    if(pt_env.pt_rx_len==QPPS_VAL_CHAR_NUM*QPP_DATA_MAX_LEN)  //receive data buf is full, should sent them to ble
    {
		ke_evt_set(1UL << EVENT_UART_RX_FRAME_ID);
			
			///leo test
//			pt_uart_rx_start();
//			pt_env.pt_state = PT_CONN_EMPTY;
//			pt_state_set(PT_CONN_EMPTY);
			///leo test end
    }
    else
    {
					ke_evt_set(1UL << EVENT_UART_RX_TIMEOUT_ID);
        	uart_read(QN_COM_UART, &pt_env.pt_rx_buf[pt_env.pt_rx_len], 1, pt_uart_rx);
    }
}

void pt_event_uart_rx_frame_handler(void)
{
	struct app_uart_data_ind *pt_data = ke_msg_alloc(APP_PT_UART_RX_DONE_IND,
															 TASK_APP,
															 TASK_APP,
															 pt_env.pt_rx_len+1);
	pt_data->len=pt_env.pt_rx_len;
	memcpy(pt_data->data,pt_env.pt_rx_buf,pt_env.pt_rx_len);
	ke_msg_send(pt_data);

	ke_timer_clear(APP_PT_RX_TIMEOUT_TIMER, TASK_APP);

	ke_evt_clear(1UL << EVENT_UART_RX_FRAME_ID);
}

void pt_event_uart_rx_timeout_handler(void)
{
	ke_timer_set(APP_PT_RX_TIMEOUT_TIMER, TASK_APP, PT_FRAME_TIMEOUT);

	ke_evt_clear(1UL << EVENT_UART_RX_TIMEOUT_ID);
}

int app_pt_rx_timeout_handler(ke_msg_id_t const msgid, void const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uart_rx_int_enable(QN_HCI_UART, MASK_DISABLE);  //disable uart rx interrupt 
    struct app_uart_data_ind *pt_data = ke_msg_alloc(APP_PT_UART_RX_DONE_IND,
                                 TASK_APP,
                                 TASK_APP,
                                 pt_env.pt_rx_len+1);
    pt_data->len=pt_env.pt_rx_len;
    memcpy(pt_data->data,pt_env.pt_rx_buf,pt_env.pt_rx_len);
    ke_msg_send(pt_data);
    
    return (KE_MSG_CONSUMED);
}

int app_pt_uart_rx_done_ind_handler(ke_msg_id_t const msgid, void const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    switch(msgid)
    {
        case APP_PT_UART_RX_DONE_IND:
        {
            struct app_uart_data_ind* frame = (struct app_uart_data_ind*)param;
            
            //for (uint8_t i=0;i<frame->len;i++)
            //QPRINTF("Cfm %d.\r\n", *(frame->data+i));
            
            ///leo test
//              pt_pdu_send(frame->len, &(frame->data[0]));
						///leo test end
            
            if(frame->len) //have data
            {    
                //calculate page num;
                 uint8_t pagket_res = frame->len%QPP_DATA_MAX_LEN;
                 uint8_t pagket_num;
                 if(pagket_res)
                 pagket_num = frame->len/QPP_DATA_MAX_LEN + 1;
                 else
                 pagket_num = frame->len/QPP_DATA_MAX_LEN;
                 
                 uint8_t cnt=0,sent_pagket=0; 

                for (cnt = 0; (sent_pagket<pagket_num) && cnt < QPPS_VAL_CHAR_NUM; cnt++)
                {
                     if ((app_qpps_env->char_status >> cnt) & QPPS_VALUE_NTF_CFG)
                     {
												app_qpps_env->char_status &= ~(QPPS_VALUE_NTF_CFG << cnt);
											 
                         if((pagket_res)&&(pagket_num-sent_pagket==1))
                         app_qpps_data_send(app_qpps_env->conhdl, cnt, pagket_res, (frame->data+sent_pagket*20));
                         else
                         app_qpps_data_send(app_qpps_env->conhdl, cnt, QPP_DATA_MAX_LEN, (frame->data+sent_pagket*20)); 
                         
                         sent_pagket++;
                     }
                }
            }
        }break;
        default :break;
    }
       
    return (KE_MSG_CONSUMED);
}




void app_event_pt_tx_handler(void)
{
	ke_evt_set(1UL<<EVENT_UART_TX_ID);
}


void pt_uart_write(struct ke_msg *msg)
{
    //go to start tx state
    pt_env.tx_state = PT_UART_TX_ONGOING;

    uart_write(QN_COM_UART, ((uint8_t *)&msg->param), msg->param_len, app_event_pt_tx_handler);
		delay(0x2fff);
}

// Push msg into eaci tx queue
static void pt_push(struct ke_msg *msg)
{
    // Push the message into the list of messages pending for transmission
    co_list_push_back(&pt_env.queue_tx, &msg->hdr);

    // Check if there is no transmission ongoing
    if (pt_env.tx_state == PT_UART_TX_IDLE)
        // Forward the message to the HCI UART for immediate transmission
        pt_uart_write(msg);
}

/**
 ****************************************************************************************
 * @brief EACI send PDU
 *
 ****************************************************************************************
 */
void pt_pdu_send(uint8_t len, uint8_t *par)
{
    // Allocate one msg for EACI tx
    uint8_t *msg_param = (uint8_t*)ke_msg_alloc(0, 0, 0, len);

    // Save the PDU in the MSG
    memcpy(msg_param, par, len);

     //extract the ke_msg pointer from the param passed and push it in HCI queue
    pt_push(ke_param2msg(msg_param));
}

/**
 ****************************************************************************************
 * @brief After-process when one PDU has been sent.
 *
 ****************************************************************************************
 */
void pt_tx_done(void)
{
    struct ke_msg * msg;
    // Clear the event
    ke_evt_clear(1<<EVENT_UART_TX_ID);
    // Go back to IDLE state
    pt_env.tx_state = PT_UART_TX_IDLE;
    //release current message (which was just sent)
    msg = (struct ke_msg *)co_list_pop_front(&pt_env.queue_tx);
    // Free the kernel message space
    ke_msg_free(msg);
    // Check if there is a new message pending for transmission
    if ((msg = (struct ke_msg *)co_list_pick(&pt_env.queue_tx)) != NULL)
    {
        // Forward the message to the HCI UART for immediate transmission
        pt_uart_write(msg);
    }
}




 

 


/// @} app_pt
