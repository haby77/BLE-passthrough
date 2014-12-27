/**
 ****************************************************************************************
 *
 * @file app_eaci_uart.h
 *
 * @brief UART transport module functions for Easy Application Controller Interface.
 *
 * Copyright (C) Quintic 2012-2013
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef APP_PT_H_
#define APP_PT_H_

#include "app_env.h"


enum pt_st
{
    PT_DEEPSLEEP	= 0,
    PT_ADV			= 1,
    PT_CONN_EMPTY   = 2,
    PT_CONN_FULL	= 3,
    
    PT_UART_TX_IDLE = 4,
    PT_UART_TX_ONGOING=5
};


#define PT_FRAME_TIMEOUT 2 //2*10ms

#define EVENT_UART_TX_ID 3
#define EVENT_UART_RX_FRAME_ID 4
#define EVENT_UART_RX_TIMEOUT_ID 5
struct pt_env_tag
{
    uint8_t pt_state ;
    uint8_t pt_state_chage;
    uint8_t pt_tx_en;
    
    ///Message id
    uint8_t msg_id;
    
    ///UART TX parameter 
    uint8_t tx_state;       //either transmitting or done.
    struct co_list queue_tx;///Queue of kernel messages corresponding to packets sent through HCI
    
    ///UART RX parameter 
    uint8_t pt_rx_len;
    uint8_t pt_rx_buf[QPPS_VAL_CHAR_NUM*QPP_DATA_MAX_LEN];

};

extern  struct pt_env_tag  pt_env;


extern void pt_uart_write(struct ke_msg *msg);
extern void pt_pdu_send(uint8_t len, uint8_t *par);
extern void pt_tx_done(void);
extern void pt_event_uart_rx_frame_handler(void);
extern void pt_event_uart_rx_timeout_handler(void);





















// Field length of Message Type and Parameter Length
#define EACI_MSG_HDR_LEN    2

typedef void (*p_ke_evt_set)(uint32_t const event);
#define ke_evt_set ((p_ke_evt_set)(_ke_evt_set))

typedef void (*p_ke_evt_clear)(uint32_t const event);
#define ke_evt_clear ((p_ke_evt_clear)(_ke_evt_clear))

/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */

extern struct eaci_env_tag eaci_env;

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */
 
/*
 ****************************************************************************************
 * @brief EACI UART initialization function.
 *
 ****************************************************************************************
 */
void eaci_uart_init(void);

/*
 ****************************************************************************************
 * @brief EACI UART write function.
 *
 ****************************************************************************************
 */
void eaci_uart_write(struct ke_msg *msg);

/*
 ****************************************************************************************
 * @brief EACI Function called at each RX interrupt.
 *
 *****************************************************************************************
 */
void eaci_uart_rx_done(void);

/*
 ****************************************************************************************
 * @brief 
 *
 *****************************************************************************************
 */
 int app_pt_uart_ind_handler(ke_msg_id_t const msgid, void const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id);
                               
                               
 /*
 ****************************************************************************************
 * @brief 
 *
 *****************************************************************************************
 */                              
 void app_pt_task_msg_hdl(ke_msg_id_t const msgid, void const *param);
 
 int app_pt_rx_timeout_handler(ke_msg_id_t const msgid, void const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id);
 int app_pt_uart_rx_done_ind_handler(ke_msg_id_t const msgid, void const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id);
 
 
 void pt_uart_rx(void);
 
 void pt_uart_rx_start(void);
 extern  void pt_init(void);
															 



















															 
															 
															 
															 
															 

/// @} EACI_UART
#endif // APP_EACI_UART_H_
