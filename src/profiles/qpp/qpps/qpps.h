/**
 ****************************************************************************************
 *
 * @file qpps.h
 *
 * @brief Header file - Quintic Private Profile Server.
 *
 * Copyright (C) Quintic 2013-2013
 *
 * $Rev: $
 *
 ****************************************************************************************
 */

#ifndef _QPPS_H_
#define _QPPS_H_

/**
 ****************************************************************************************
 * @addtogroup Quintic private profile Server
 * @ingroup QPP
 * @brief Quintic private profile Server
 *
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#if (BLE_QPP_SERVER)
#include "qpp_common.h"
#include "prf_types.h"
#include "attm.h"
#include "atts.h"
#include "atts_db.h"

/*
 * DEFINES
 ****************************************************************************************
 */
#define QPPS_MANDATORY_MASK             (0x000f)

/*
 * MACROS
 ****************************************************************************************
 */

#define QPPS_IS_SUPPORTED(idx, mask) \
    ((((qpps_env.features >> idx) & mask) == mask))

///Attributes State Machine
enum
{
    QPPS_IDX_SVC,

    QPPS_IDX_RX_DATA_CHAR,
    QPPS_IDX_RX_DATA_VAL,
    QPPS_IDX_RX_DATA_USER_DESP,
	
    QPPS_IDX_VAL_CHAR,
    QPPS_IDX_VAL,
    QPPS_IDX_VAL_NTF_CFG,

    QPPS_IDX_NB,
};

enum
{
    QPPS_VALUE_NTF_CFG	= 0x01,
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Server environment variable
struct qpps_env_tag
{
    ///Application Task Id
    ke_task_id_t appid;
    ///Connection handle
    uint16_t conhdl;

    ///Service Start Handle
    uint16_t shdl;
    ///Database configuration
    uint32_t features;
    ///Notify char number
    uint8_t ntf_char_num;
};


/*
 * MACROS
 ****************************************************************************************
 */


/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */

extern const struct atts_desc qpps_att_db[QPPS_IDX_NB];

///  Service - only one instance for now
extern const atts_svc_desc_t qpps_svc;

extern const struct atts_char_desc qpps_value_char;
extern const struct atts_char_desc qpps_char_rx_data;

extern struct qpps_env_tag qpps_env;

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialization of the Quintic private profile module.
 * This function performs all the initializations of the QPPS module.
 ****************************************************************************************
 */
void qpps_init(void);

/**
 ****************************************************************************************
 * @brief Indicate error to higher layers.
 * @param status Error status to send to requester task
 ****************************************************************************************
 */
void qpps_error_ind_send(uint8_t status);

/**
 ****************************************************************************************
 * @brief Disable actions grouped in getting back to IDLE and sending configuration to requester task
 ****************************************************************************************
 */
void qpps_disable(void);

#endif /* #if (BLE_QPP_SERVER) */

/// @} QPPS

#endif /* _QPPS_H_ */
