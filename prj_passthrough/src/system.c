/**
 ****************************************************************************************
 *
 * @file system.c
 *
 * @brief User system setup and initial configuration source file.
 *
 * Copyright (C) Quintic 2012-2013
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#include "system.h"
#include "uart.h"
#include "gpio.h"
#include "spi.h"
#include "timer.h"
#include "pwm.h"
#include "dma.h"
#include "serialflash.h"
#include "adc.h"
#include "analog.h"
#include "led.h"
#include "button.h"
#include "lib.h"
#include "usr_design.h"
#include "app_pt_gpio.h"

/**
 ****************************************************************************************
 * @brief  GPIO functionn configuration.
 *
 * Set Pins as gpio pin or function pin
 *****************************************************************************************
 */
static void SystemIOCfg(void)
{
    // pin mux
    syscon_SetPMCR0(QN_SYSCON, P00_UART0_TXD_PIN_CTRL
                             | P01_GPIO_1_PIN_CTRL
                             | P02_GPIO_2_PIN_CTRL
                             | P03_GPIO_3_PIN_CTRL
                             | P04_GPIO_4_PIN_CTRL
                             | P05_GPIO_5_PIN_CTRL
                             | P06_SW_DAT_PIN_CTRL
                             | P07_SW_CLK_PIN_CTRL

                             | P10_GPIO_8_PIN_CTRL
                             | P11_GPIO_9_PIN_CTRL
                             | P12_GPIO_10_PIN_CTRL
                             | P13_GPIO_11_PIN_CTRL
                             | P14_GPIO_12_PIN_CTRL
                             | P15_GPIO_13_PIN_CTRL
                             | P16_GPIO_14_PIN_CTRL
                             | P17_UART0_RXD_PIN_CTRL
                             );
    syscon_SetPMCR1(QN_SYSCON, P20_GPIO_16_PIN_CTRL
                             | P21_GPIO_17_PIN_CTRL
                             | P22_GPIO_18_PIN_CTRL
                             | P23_GPIO_19_PIN_CTRL
                             | P24_GPIO_20_PIN_CTRL
                             | P25_GPIO_21_PIN_CTRL
                             | P26_GPIO_22_PIN_CTRL
                             | P27_GPIO_23_PIN_CTRL

                             | P30_GPIO_24_PIN_CTRL
                             | P31_GPIO_25_PIN_CTRL
                             | P32_GPIO_26_PIN_CTRL
                             | P33_GPIO_27_PIN_CTRL
                             | P34_GPIO_28_PIN_CTRL
                             | P35_GPIO_29_PIN_CTRL
                             | P36_GPIO_30_PIN_CTRL
                             );

    // pin select
    syscon_SetPMCR2(QN_SYSCON, SYSCON_MASK_UART1_PIN_SEL | SYSCON_MASK_SPI0_PIN_SEL);

    // driver ability
    syscon_SetPDCR(QN_SYSCON, 0x0); // 0 : low driver, 1 : high driver

    // pin pull ( 00 : High-Z,  01 : Pull-down,  10 : Pull-up,  11 : Reserved )
    syscon_SetPPCR0(QN_SYSCON, 0xAAAA5AAA);
    syscon_SetPPCR1(QN_SYSCON, 0x2AAAAAAA);
}

/**
 ****************************************************************************************
 * @brief  Setup the microcontroller system.
 *
 *  Initialize the system clock and pins.
 *****************************************************************************************
 */
void SystemInit(void)
{
    /*
     **************************
     * Sub module clock setting
     **************************
     */

    // Disable all peripheral clock, will be enabled in the driver initilization.
    timer_clock_off(QN_TIMER0);
    timer_clock_off(QN_TIMER1);
    timer_clock_off(QN_TIMER2);
    timer_clock_off(QN_TIMER3);
    uart_clock_off(QN_UART0);
    uart_clock_off(QN_UART1);
    spi_clock_off(QN_SPI0);
    usart_reset((uint32_t) QN_SPI1);
    spi_clock_off(QN_SPI1);
    flash_clock_off();
    gpio_clock_off();
    adc_clock_off();
    dma_clock_off();
    pwm_clock_off();
    
    // Configure sytem clock. 
    syscon_set_sysclk_src(CLK_XTAL, __XTAL);
    syscon_set_ahb_clk(__AHB_CLK);
    syscon_set_ble_clk(__BLE_CLK);
    syscon_set_apb_clk(__APB_CLK);
    syscon_set_timer_clk(__TIMER_CLK);
    syscon_set_usart_clk((uint32_t)QN_UART0, __USART_CLK);
    syscon_set_usart_clk((uint32_t)QN_UART1, __USART_CLK);  
    clk32k_enable(__32K_TYPE);
    
    /*
     **************************
     * IO configuration
     **************************
     */

    SystemIOCfg();

    /*
     **************************
     * Peripheral setting
     **************************
     */

    // GPIO initialization for led, button & test control pin.
    gpio_init(gpio_interrupt_callback);

    // Test controll pin is input to check work mode
#if (defined(QN_TEST_CTRL_PIN))

    gpio_pull_set(QN_TEST_CTRL_PIN, GPIO_PULL_UP);
    gpio_set_direction_field(QN_TEST_CTRL_PIN, (uint32_t)GPIO_INPUT);
		
	//leo add		
	if(gpio_read_pin(QN_TEST_CTRL_PIN) == GPIO_HIGH)					//
	{
#endif

#if (defined(QN_PT))
		uart_init(QN_HCI_UART, USARTx_CLK(0), UART_115200);
		uart_tx_enable(QN_HCI_UART, MASK_ENABLE);
		uart_rx_enable(QN_HCI_UART, MASK_ENABLE);
		pt_gpio_init();
		pt_init();
		
//		///leo test
//		gpio_set_direction_field(GPIO_P31,(uint32_t)GPIO_OUTPUT);
//		gpio_write_pin(GPIO_P31, GPIO_HIGH);
//		gpio_set_direction_field(GPIO_P30,(uint32_t)GPIO_OUTPUT);
//		gpio_write_pin(GPIO_P30, GPIO_HIGH);
//		///leo test end
#endif
			
#if (QN_DBG_PRINT)
    uart_init(QN_DEBUG_UART, USARTx_CLK(0), UART_115200);
    uart_tx_enable(QN_DEBUG_UART, MASK_ENABLE);
    uart_rx_enable(QN_DEBUG_UART, MASK_ENABLE);
#endif
			
#if (defined(QN_TEST_CTRL_PIN))
		}
		else	//enter test mode
		{
			uint8_t func_result1,func_result2;
			uint32_t temp_val;
			
			//(1)user test
			
			//(a)set test flag = running
			temp_val = *((uint32_t*)FUNCTION_TEST_RESULT_REG_ADDRESS);
			temp_val &= (~FUNCTION_TEST_RESULT_MASK);
			temp_val |= (FUNCTION_TEST_RESULT_MASK & FUNCTION_TEST_RESULT_RUNNING);
			*((uint32_t*)FUNCTION_TEST_RESULT_REG_ADDRESS) = temp_val;
			
			//(b)for module gpio test
			func_result1 = module_gpio_test();
			//(c)32K crystal test
			func_result2 = module_32k_test();

			
			//(d)set test flag = fail
			if((func_result1!=0) | (func_result2!=0))
			{
				temp_val = *((uint32_t*)FUNCTION_TEST_RESULT_REG_ADDRESS);
				temp_val &= (~FUNCTION_TEST_RESULT_MASK);
				temp_val |= (FUNCTION_TEST_RESULT_MASK & FUNCTION_TEST_RESULT_FAIL);
				*((uint32_t*)FUNCTION_TEST_RESULT_REG_ADDRESS) = temp_val;
			}
			//(e)set test flag = sucessful
			else
			{
				temp_val = *((uint32_t*)FUNCTION_TEST_RESULT_REG_ADDRESS);
				temp_val &= (~FUNCTION_TEST_RESULT_MASK);
				temp_val |= (FUNCTION_TEST_RESULT_MASK & FUNCTION_TEST_RESULT_SUCCESS);
				*((uint32_t*)FUNCTION_TEST_RESULT_REG_ADDRESS) = temp_val;
			}
			//for DTM test
			uart_init(QN_HCI_UART, USARTx_CLK(0), UART_9600);
			uart_tx_enable(QN_HCI_UART, MASK_ENABLE);
			uart_rx_enable(QN_HCI_UART, MASK_ENABLE);
		}
		
#endif

///leo end
}


/// @} SYSTEM_CONTROLLER

