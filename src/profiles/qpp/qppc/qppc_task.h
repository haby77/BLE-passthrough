/**
 ****************************************************************************************
 *
 * @file qppc_task.h
 *
 * @brief Header file - Quintic Private Profile Client Task.
 *
 * Copyright (C) Quintic 2013-2013
 *
 * $Rev: $
 *
 ****************************************************************************************
 */

#ifndef _QPPC_TASK_H_
#define _QPPC_TASK_H_

/// @cond

/**
 ****************************************************************************************
 * @addtogroup QPPCTASK Quintic Private Profile Client Task
 * @ingroup QPPC
 * @brief Quintic Private Profile Client Task
 *
 * The QPPCTASK is responsible for handling the messages coming in and out of the
 * @ref QPPC monitor block of the BLE Host.
 *
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_config.h"

#if (BLE_QPP_CLIENT)
#include "ke_task.h"
#include "gatt_task.h"
#include "co_error.h"
#include "qppc.h"
#include "prf_types.h"
#include "prf_utils.h"

/*
 * DEFINES
 ****************************************************************************************
 */

///Maximum number of Quintic Private Profile Client task instances
#define QPPC_IDX_MAX    0x01


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Possible states of the QPPC task
enum
{
    /// IDLE state
    QPPC_IDLE,
    /// connected state
    QPPC_CONNECTED,
    /// Discovering Quintic Private Profile SVC and CHars
    QPPC_DISCOVERING,
    // Number of defined states.
    QPPC_STATE_MAX
};

enum
{
    /// Start the Quintic Private Profile - at connection
    QPPC_ENABLE_REQ = KE_FIRST_MSG(TASK_QPPC),
    ///Confirm that cfg connection has finished with discovery results, or that normal cnx started
    QPPC_ENABLE_CFM,
    ///Disable profile role - at disconnection
    QPPC_DISABLE_IND,
    ///Generic error message
    QPPC_ERROR_IND,

    /// Generic message to read a QPPS characteristic value
    QPPC_RD_CHAR_REQ,
    ///Generic message for read responses for APP
    QPPC_RD_CHAR_RSP,
    ///Generic message for configuring the 2 characteristics that can be handled
    QPPC_CFG_INDNTF_REQ,
    ///APP send data request to server
    QPPC_WR_DATA_REQ,
    ///Generic message for write characteristic response status to APP
    QPPC_WR_CHAR_RSP,

    /// value send to APP
    QPPC_DATA_IND,
};


/// Parameters of the @ref QPPC_ENABLE_REQ message
struct qppc_enable_req
{
    /// Connection handle
    uint16_t conhdl;
    ///Connection type
    uint8_t con_type;
    ///Existing handle values hrs
    struct qpps_content qpps;
};

///Parameters of the @ref QPPC_RD_CHAR_REQ message
struct qppc_rd_char_req
{
    ///Connection handle
    uint16_t conhdl;
    ///Characteristic value code
    uint8_t  char_code;
};

///Parameters of the @ref QPPC_CFG_INDNTF_REQ message
struct qppc_cfg_indntf_req
{
    ///Connection handle
    uint16_t conhdl;
    ///Stop/notify/indicate value to configure into the peer characteristic
    uint16_t cfg_val;
};

///Parameters of the @ref QPPC_WR_DATA_REQ message
struct qppc_wr_data_req
{
    ///Connection handle
    uint16_t conhdl;
    /// Length
    uint8_t length;
    /// Data
    uint8_t data[1];
};

/// @endcond

/**
 ****************************************************************************************
 * @addtogroup APP_QPPC_TASK
 * @{
 ****************************************************************************************
 */

/// Parameters of the @ref QPPC_ENABLE_CFM message
struct qppc_enable_cfm
{
    /// Connection handle
    uint16_t conhdl;
    ///status
    uint8_t status;
    ///Existing handle values hrs
    struct qpps_content qpps;
};

///Parameters of the @ref QPPC_ERROR_IND message
struct qppc_error_ind
{
    ///Connection handle
    uint16_t conhdl;
    ///Status
    uint8_t  status;
};

///Parameters of the @ref QPPC_RD_CHAR_RSP message
struct qppc_rd_char_rsp
{
    ///Connection handle
    uint16_t conhdl;
    ///Status
    uint8_t  status;
    ///Holder of retrieved data
    struct att_info_data data;
};

///Parameters of the @ref QPPC_WR_CHAR_RSP message
struct qppc_wr_char_rsp
{
    ///Connection handle
    uint16_t conhdl;
    ///Status
    uint8_t  status;
};

///Parameters of the @ref QPPC_DATA_IND message
struct qppc_data_ind
{
    ///Connection handle
    uint16_t conhdl;
    /// Length
    uint8_t length;
    /// Data
    uint8_t data[1];
};

/// @} APP_QPPC_TASK
/// @cond

/*
 * MACROS
 ****************************************************************************************
 */


/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 * TASK DESCRIPTOR DECLARATIONS
 ****************************************************************************************
 */
extern const struct ke_state_handler qppc_state_handler[QPPC_STATE_MAX];
extern const struct ke_state_handler qppc_default_handler;
extern ke_state_t qppc_state[QPPC_IDX_MAX];

extern void task_qppc_desc_register(void);

#endif //(BLE_QPP_CLIENT)

/// @} QPPCTASK
/// @endcond
#endif /* _QPPC_TASK_H_ */
