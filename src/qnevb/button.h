/**
 ****************************************************************************************
 *
 * @file button.h
 *
 * @brief Header file of button driver for qn evb.
 *
 * Copyright (C) Quintic 2012-2013
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */
#ifndef _BUTTON_H_
#define _BUTTON_H_

/*
 * MACRO DEFINITIONS
 ****************************************************************************************
 */
#define BUTTON1_PIN    (GPIO_P14)
#define BUTTON2_PIN    (GPIO_P15)


/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

extern void button_init(void);
extern int check_button_state(int btn);

#endif

