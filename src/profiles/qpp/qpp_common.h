/**
 ****************************************************************************************
 *
 * @file qpp_common.h
 *
 * @brief Header File - Quintic private profile common types.
 *
 * Copyright (C) Quintic 2013-2013
 *
 * $Rev: $
 *
 ****************************************************************************************
 */

#ifndef _QPP_COMMON_H_
#define _QPP_COMMON_H_

/**
 ****************************************************************************************
 * @addtogroup Quintic private Profile
 * @ingroup PROFILE
 * @brief Quintic private Profile
 *
 *****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#if (BLE_QPP_CLIENT || BLE_QPP_SERVER)

#include "prf_types.h"

/*
 * DEFINES
 ****************************************************************************************
 */

#define QPP_SVC_PRIVATE_UUID            0xCC07

enum
{
	QPP_CHAR_VAL_UUID 		  = 0xCD01,
	QPP_CHAR_INPUT_DATA_UUID  = 0xCD20,
};

// Derek, used as max data length
#define QPP_DATA_MAX_LEN         (20)

// error code
#define QPPS_ERR_RX_DATA_NOT_SUPPORTED      (0x80)
#define QPPS_ERR_RX_DATA_EXCEED_MAX_LENGTH  (0x81)
#define QPPS_ERR_INVALID_PARAM				(0x82)

///QPPS codes for the 2 possible client configuration characteristic descriptors determination
enum
{
    /// Output Value
    QPPS_OUTPUT_VALUE_CODE = 0x01,
    /// Input Value
    QPPS_INPUT_VALUE_CODE,
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

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

#endif /* #if (BLE_QPP_CLIENT || BLE_QPP_SERVER) */

/// @} qpp_common

#endif /* _QPP_COMMON_H_ */
