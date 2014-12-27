/**
 ****************************************************************************************
 *
 * @file otas_task.h
 *
 * @brief Header file - Quintic Private Profile Server Task.
 *
 * Copyright (C) Quintic 2013-2013
 *
 * $Rev: $
 *
 ****************************************************************************************
 */

#ifndef _OTAS_TASK_H_
#define _OTAS_TASK_H_


/// @cond

#define OTAS_DEBUG 0

/**
 ****************************************************************************************
 * @addtogroup OTASSTASK Task
 * @ingroup OTASS
 * @brief Quintic private profile Task.
 *
 * The OTASSTASK is responsible for handling the messages coming in and out of the
 * @ref OTASS collector block of the BLE Host.
 *
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#if BLE_OTA_SERVER
#include <stdint.h>
#include "ke_task.h"
#include "otas.h"

/*
 * DEFINES
 ****************************************************************************************
 */

///Maximum number of Quintic private profile task instances
#define OTAS_IDX_MAX     0x01

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Possible states of the OTASS task
enum
{
    /// Disabled state
    OTAS_DISABLED,
    /// Idle state
    OTAS_IDLE,
    /// Connected state
    OTAS_CONNECTED,

    /// Number of defined states.
    OTAS_STATE_MAX,
};

/// Messages for Quintic private Profile Server
enum
{
    ///Start the Quintic private Profile Server - at connection
    OTAS_ENABLE_REQ = KE_FIRST_MSG(TASK_OTAS),
    ///Disable profile role - at disconnection
    OTAS_DISABLE_REQ,
    ///Disable confirmation with configuration to save after profile disabled
    OTAS_DISABLE_CFM,
    ///Error indication to Host
    OTAS_ERROR_IND,
    ///Send value from APP
    OTAS_DATA_SEND_REQ,
    ///Send data confirm to APP if correctly sent.
    OTAS_DATA_SEND_CFM,
    ///Inform APP of new configuration value
    OTAS_CFG_INDNTF_IND,
    ///Client value send to APP
    OTAS_DAVA_VAL_IND,

    ///Add OTASS into the database
    OTAS_CREATE_DB_REQ,
    ///Inform APP about DB creation status
    OTAS_CREATE_DB_CFM,
};

///Parameters of the @ref OTAS_CREATE_DB_REQ message
struct otas_create_db_req
{
    /// Database configuration
    uint8_t features;
    
    /// rx char number
    uint8_t rx_char_num;
};

/// Parameters of the @ref OTAS_ENABLE_REQ message
struct otas_enable_req
{
    ///Connection handle
    uint16_t conhdl;
    /// security level: b0= nothing, b1=unauthenticated, b2=authenticated, b3=authorized;
    /// b1 or b2 and b3 can go together
    uint8_t sec_lvl;
    ///Type of connection - will someday depend on button press length; can be CFG or DISCOVERY
    uint8_t con_type;

    /// Notification configuration
    uint32_t ntf_en;
};

/// Parameters of the @ref OTAS_DISABLE_REQ message
struct otas_disable_req
{
    ///Connection handle
    uint16_t conhdl;
};

/////Parameters of the @ref OTAS_DATA_SEND_REQ message
struct otas_data_send_req
{
    uint8_t status;
    ///Connection handle
    uint16_t conhdl;
    /// Char index
    uint8_t index;
    /// Length
    uint8_t length;
    /// Data
    uint8_t data[1];
};

/// @endcond

/**
 ****************************************************************************************
 * @addtogroup APP_OTASS_TASK
 * @{
 ****************************************************************************************
 */

///Parameters of the @ref OTAS_CREATE_DB_CFM message
struct otas_create_db_cfm
{
    ///Status
    uint8_t status;
};

///Parameters of the @ref OTAS_DISABLE_CFM message
struct otas_disable_cfm
{
    uint16_t conhdl;
    /// Notification configuration
    uint16_t ntf_en;
};

///Parameters of the @ref OTAS_ERROR_IND message
struct otas_error_ind
{
    ///Connection handle
    uint16_t conhdl;
    ///Status
    uint8_t  status;
};

///Parameters of the @ref OTAS_CFG_INDNTF_IND message
struct otas_cfg_indntf_ind
{
    ///Connection handle
    uint16_t conhdl;
    ///Char index
    uint8_t char_index;
    ///Stop/notify/indicate value to configure into the peer characteristic
    uint16_t cfg_val;
};

///Parameters of the @ref OTAS_DATA_SEND_CFM message
struct otas_data_send_cfm
{
    ///Connection handle
    uint16_t conhdl;
    ///Char index
    uint8_t char_index;
    ///Status
    uint8_t status;
};

///Parameters of the @ref OTAS_DAVA_VAL_IND message
struct otas_data_val_ind
{
    ///Connection handle
    uint16_t conhdl;

    uint16_t length;
    uint8_t data[1];
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// @} APP_OTASS_TASK

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
extern const struct ke_state_handler otas_state_handler[OTAS_STATE_MAX];
extern const struct ke_state_handler otas_default_handler;
extern ke_state_t otas_state[OTAS_IDX_MAX];

extern void task_otas_desc_register(void);

#endif /* BLE_OTA_SERVER */

/// @} OTASSTASK
/// @endcond
#endif /* _OTASS_TASK_H_ */
