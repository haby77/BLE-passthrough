/**
 ****************************************************************************************
 *
 * @file qppc.h
 *
 * @brief Header file - Quintic Private Profile Client Role.
 *
 * Copyright (C) Quintic 2013-2013
 *
 * $Rev: $
 *
 ****************************************************************************************
 */

#ifndef _QPPC_H_
#define _QPPC_H_

/**
 ****************************************************************************************
 * @addtogroup QPPC Quintic Private Profile Client
 * @ingroup QPP
 * @brief Quintic Private Profile Client
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#if (BLE_QPP_CLIENT)
#include "qpp_common.h"
#include "ke_task.h"
#include "prf_types.h"
#include "prf_utils.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/// Characteristics
enum
{
    /// Quintic Private Profile Output Value
    QPPC_CHAR_QPP_OUTPUT_VALUE,
    /// Quintic Private Profile Input Value
    QPPC_CHAR_QPP_INTPUT_VALUE,

    QPPC_CHAR_MAX,
};

/// Characteristic descriptors
enum
{
    /// Output Value Client config
    QPPC_DESC_OUTPUT_VALUE_CLI_CFG,
    /// Output Value User Desp
    QPPC_DESC_OUTPUT_VALUE_USER_DESP,
    /// Input Value User Desp
    QPPC_DESC_INTPUT_VALUE_USER_DESP,    

    QPPC_DESC_MAX,
    QPPC_DESC_MASK = 0x10,
};

/// Internal codes for reading a QPPS characteristic with one single request
enum
{
    ///Read QPPS Value
    QPPC_RD_QPPS_OUTPUT_VALUE               = QPPC_CHAR_QPP_OUTPUT_VALUE,
    ///Input Value
    QPPC_RD_QPPS_INTPUT_VALUE               = QPPC_CHAR_QPP_INTPUT_VALUE,

    ///Read QPPS Client Cfg. Desc
    QPPC_RD_QPPS_OUTPUT_VALUE_CFG           = (QPPC_DESC_MASK | QPPC_DESC_OUTPUT_VALUE_CLI_CFG),
    QPPC_RD_QPPS_OUTPUT_VALUE_USER_DESP     = (QPPC_DESC_MASK | QPPC_DESC_OUTPUT_VALUE_USER_DESP),
    QPPC_RD_QPPS_INTPUT_VALUE_USER_DESP     = (QPPC_DESC_MASK | QPPC_DESC_INTPUT_VALUE_USER_DESP),
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

///Structure containing the characteristics handles, value handles and descriptors
struct qpps_content
{
    /// service info
    struct prf_svc svc;

    /// characteristic info:
    ///  - Output Value
    ///  - Input Value
    struct prf_char_inf chars[QPPC_CHAR_MAX];

    /// Descriptor handles:
    struct prf_char_desc_inf descs[QPPC_DESC_MAX];
};

/// Quintic Private Profile Client environment variable
struct qppc_env_tag
{
    /// Profile Connection Info
    struct prf_con_info con_info;
    ///Last requested UUID(to keep track of the two services and char)
    uint16_t last_uuid_req;

    ///QPPS characteristics
    struct qpps_content qpps;

    /// Last char. code requested to read.
    uint8_t last_char_code;

    /// counter used to check service uniqueness
    uint8_t nb_svc;
};


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
extern struct qppc_env_tag qppc_env;
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


/**
 ****************************************************************************************
 * @brief Initialization of the QPPC module.
 * This function performs all the initializations of the QPPC module.
 ****************************************************************************************
 */
void qppc_init(void);

/**
 ****************************************************************************************
 * @brief Send ATT DB discovery results to QPPC host.
 ****************************************************************************************
 */
void qppc_enable_cfm_send(uint8_t status);

/**
 ****************************************************************************************
 * @brief Send error indication from profile to Host, with proprietary status codes.
 * @param status Status code of error.
 ****************************************************************************************
 */
void qppc_error_ind_send(uint8_t status);

/**
 ****************************************************************************************
 * @brief Disable Quintic Private Profile Client Task
 ****************************************************************************************
 */
void qppc_disable(void);

#endif /* (BLE_QPP_CLIENT) */

/// @} QPPC

#endif /* _QPPC_H_ */
