/**
 ****************************************************************************************
 *
 * @file adc.h
 *
 * @brief Header file of ADC for QN9020.
 *
 * Copyright (C) Quintic 2012-2013
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */
#ifndef _ADC_H_
#define _ADC_H_
#include "driver_config.h"
#if CONFIG_ENABLE_DRIVER_ADC==TRUE
#include "syscon.h"
#include "sleep.h"

/**
 ****************************************************************************************
 * @defgroup ADC ADC Driver
 * @ingroup DRIVERS
 * @brief ADC driver
 *
 *  QN9020 contains an up to 12 bits resolution successive approximation analog-to-digital converter
 *  (SAR A/D converter) with 12 input channels. It takes about 20 ADC clock cycles to convert one sample,
 *  and the maximum input clock to ADC is 1MHz. The A/D converter supports multi operation modes
 *  and can be started by 4 types of trigger sources.
 *
 *  The main features of ADC are listed as follow:
 *    - Maximum sample rate is 1MSPS
 *    - Support 8/10/12 bits resolution for one sample data
 *    - ADC input can be selected from 12 sources which includes 10 single-end input and 2 differential input
 *    - ADC conversion can be triggered by 4 sources: Software Start, Timer0/1 overflow and GPIO
 *    - Support selectable decimation rates, thereby corresponding improved effective resolutions
 *    - Support window compare, and generate corresponding interrupt
 *    - Support single conversion mode, continuous conversion mode
 *    - Support single burst mode, burst length is same to decimation rates
 *    - Support single scan conversion mode, continuous scan conversion mode
 *    - Support up to 1MHz/20 sampling rate
 *    - Support DMA
 *    - Support selectable reference voltage
 *
 * @{
 *
 ****************************************************************************************
 */

/* \example adc_example.c
 * This is an example of how to use the ADC driver.
 */


#ifdef __cplusplus
#if __cplusplus
extern "C"{
#endif
#endif /* __cplusplus */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */


/*
 * MACRO DEFINITIONS
 ****************************************************************************************
 */

/// ADC differential input with buffer driver, input singal should 0.2 =< VIN(V) <= VDD-0.2.
#define ADC_DIFF_WITH_BUF_DRV                       0
/// ADC differential input without buffer driver, input singal should 0 =< VIN(V) <= VDD, and have enough driving capability.
#define ADC_DIFF_WITHOUT_BUF_DRV                    1
/// ADC single-ended input with buffer driver, input singal should 0.2 =< VIN(V) <= 1.5*VREF <= VDD-0.2.
#define ADC_SINGLE_WITH_BUF_DRV                     2
/// ADC single-ended input without buffer driver, input singal should 0 =< VIN(V) <= 1.5*VREF <= VDD, and have enough driving capability.
#define ADC_SINGLE_WITHOUT_BUF_DRV                  3

/// Internal reference voltage: mv
#define ADC_INT_REF_VOL                             ADC_SCALE
/// External reference voltage: mv
#define ADC_EXT_REF_VOL                             (1000)


/// configure ADC input mode: differential or single-ended, with or without buffer driver
#define CFG_ADC_IN_MOD                              ADC_SINGLE_WITHOUT_BUF_DRV
/// configure ADC reference voltage: internal or external(mv)
#define CFG_ADC_REF_VOL                             ADC_INT_REF_VOL
/// configure ADC resolution
#define CFG_ADC_RESOLUTION                          ADC_12BIT


#if CFG_ADC_IN_MOD==ADC_DIFF_WITH_BUF_DRV
#define ADC_BUFF_IN_P_CFG                           ADC_BUFIN_CHANNEL
#define ADC_BUFF_IN_N_CFG                           ADC_BUFIN_CHANNEL
#define ADC_BUFF_CTRL_CFG                           ADC_MASK_BUF_GAIN_BP
#elif CFG_ADC_IN_MOD==ADC_DIFF_WITHOUT_BUF_DRV
#define ADC_BUFF_IN_P_CFG                           ADC_BUFIN_CHANNEL
#define ADC_BUFF_IN_N_CFG                           ADC_BUFIN_CHANNEL
#define ADC_BUFF_CTRL_CFG                           (ADC_MASK_BUF_PD|ADC_MASK_INBUF_BP)
#elif CFG_ADC_IN_MOD==ADC_SINGLE_WITH_BUF_DRV
#define ADC_BUFF_IN_P_CFG                           ADC_BUFIN_CHANNEL
#define ADC_BUFF_IN_N_CFG                           ADC_BUFIN_VCM
#define ADC_BUFF_CTRL_CFG                           ADC_MASK_BUF_GAIN_BP
#elif CFG_ADC_IN_MOD==ADC_SINGLE_WITHOUT_BUF_DRV
#define ADC_BUFF_IN_P_CFG                           ADC_BUFIN_CHANNEL
#define ADC_BUFF_IN_N_CFG                           ADC_BUFIN_VCM
#define ADC_BUFF_CTRL_CFG                           ADC_MASK_INBUF_BP
#endif


/*
 * ENUMERATION DEFINITIONS
 *****************************************************************************************
 */

/// ADC channel index
enum ADC_CH
{
    AIN0 = 0,           /*!< Analog single channel 0, P3.0 */
    AIN1,               /*!< Analog single channel 1, P3.1 */
    AIN2,               /*!< Analog single channel 2, P0.6 */
    AIN3,               /*!< Analog single channel 3, P0.7 */
    AIN01,              /*!< Analog differential channel 0/1, P3.0/P3.1 */
    AIN23,              /*!< Analog differential channel 2/3, P0.6/P0.7 */
    TEMP,               /*!< temperture sensor channel */
    BATT,               /*!< Battery detector channel */
    ADC_CH_NUM
};

/// ADC work mode
enum ADC_MODE
{
    SINGLE_MOD = 0,     /*!< Single mode,  */
    CONTINUE_MOD,       /*!< Continue mode, only need trigger once */
    SINGLE_SCAN_MOD,    /*!< Single Scan mode */
    CONTINUE_SCAN_MOD   /*!< Continue Scan mode */
};

/// ADC reference voltage
enum ADC_REF
{
    ADC_INT_REF = 0,    /*!< Internal reference voltage, 1V */
    ADC_EXT_REF,        /*!< External reference voltage: (VREF <= VDD-1.0V) */
    ADC_EXT_VDD         /*!< External VDD: (VDD-1.0V<VREF<=VDD) */
};

/// ADC Trigger source
enum ADC_TRIG_SRC
{
    ADC_TRIG_SOFT = 0,  /*!< Triggered by software */
    ADC_TRIG_TOVF0,     /*!< Triggered by timer0 overflow */
    ADC_TRIG_TOVF1,     /*!< Triggered by timer1 overflow */
    ADC_TRIG_GPIO,      /*!< Triggered by GPIO */
    ADC_TRIG_CALIB      /*!< Triggered by Calibration */
};

/// ADC GPIO trigger PIN
enum ADC_GPIO_TRIG
{
    ADC_GPIO06_TRIG = 0,            /*!< Triggered by GPIO06 */
    ADC_GPIO15_TRIG = 0x00000008    /*!< Triggered by GPIO15 */
};

/// ADC resolution
enum ADC_RESOLUTION
{
    ADC_12BIT = 0,      /*!< 12 bits resolution */
    ADC_10BIT,          /*!< 10 bits resolution */
    ADC_8BIT            /*!< 8 bits resolution */

};

/// ADC clock source
enum ADC_CLK_SRC
{
    CLK_HIGH = 0,       /*!< 32MHz or 16MHz, depends on system clock */
    CLK_LOW = 1         /*!< 32KHz */
};

/// ADC working clock(ADC_SOURCE_CLK / (2<<ADC_DIV), ADC_SOURCE_CLK not from AHB)
enum ADC_WORK_CLK
{
#if __SYSTEM_CLOCK == 32000000UL
    ADC_CLK_1000000     = 0x4,      /*!< ADC work at 1MHz, when clock source is 32MHz */
    ADC_CLK_500000      = 0x5,      /*!< ADC work at 500KHz, when clock source is 32MHz */
    ADC_CLK_250000      = 0x6,      /*!< ADC work at 250KHz, when clock source is 32MHz */
    ADC_CLK_125000      = 0x7,      /*!< ADC work at 125KHz, when clock source is 32MHz */
    ADC_CLK_62500       = 0x8,      /*!< ADC work at 62.5KHz, when clock source is 32MHz */
    ADC_CLK_31250       = 0x9,      /*!< ADC work at 31.25KHz, when clock source is 32MHz */
    ADC_CLK_15625       = 0xA,      /*!< ADC work at 15.625KHz, when clock source is 32MHz */
#else // default:  __SYSTEM_CLOCK == 16000000UL
    ADC_CLK_1000000     = 0x3,      /*!< ADC work at 1MHz, when clock source is 16MHz */
    ADC_CLK_500000      = 0x4,      /*!< ADC work at 500KHz, when clock source is 16MHz */
    ADC_CLK_250000      = 0x5,      /*!< ADC work at 250KHz, when clock source is 16MHz */
    ADC_CLK_125000      = 0x6,      /*!< ADC work at 125KHz, when clock source is 16MHz */
    ADC_CLK_62500       = 0x7,      /*!< ADC work at 62.5KHz, when clock source is 16MHz */
    ADC_CLK_31250       = 0x8,      /*!< ADC work at 31.25KHz, when clock source is 16MHz */
    ADC_CLK_15625       = 0x9,      /*!< ADC work at 15.625KHz, when clock source is 16MHz */
#endif
    ADC_CLK32K_16000    = (0x0|SYSCON_MASK_ADC_CLK_SEL),      /*!< ADC work at 16KHz, when clock source is 32KHz */
    ADC_CLK32K_8000     = (0x1|SYSCON_MASK_ADC_CLK_SEL),      /*!< ADC work at 8KHz, when clock source is 32KHz */
    ADC_CLK32K_4000     = (0x2|SYSCON_MASK_ADC_CLK_SEL),      /*!< ADC work at 4KHz, when clock source is 32KHz */
    ADC_CLK32K_2000     = (0x3|SYSCON_MASK_ADC_CLK_SEL),      /*!< ADC work at 2KHz, when clock source is 32KHz */
    ADC_CLK32K_1000     = (0x4|SYSCON_MASK_ADC_CLK_SEL),      /*!< ADC work at 1KHz, when clock source is 32KHz */
    ADC_CLK32K_500      = (0x5|SYSCON_MASK_ADC_CLK_SEL),      /*!< ADC work at 500Hz, when clock source is 32KHz */
    ADC_CLK32K_250      = (0x6|SYSCON_MASK_ADC_CLK_SEL),      /*!< ADC work at 250Hz, when clock source is 32KHz */
    ADC_CLK32K_125      = (0x7|SYSCON_MASK_ADC_CLK_SEL)       /*!< ADC work at 125Hz, when clock source is 32KHz */
};

/// Window comparator data source
enum WCMP_DATA
{
    ADC_DATA = 0,                       /*!< ADC raw data */
    DECI_DATA = ADC_MASK_WCMP_SEL       /*!< Decimation data */
};

/// Decimation rate
enum DECIMATION_RATE
{
    DECI_RATE_64 = 0,                   /*!< Decimation rate: 64 */
    DECI_RATE_256 = 1,                  /*!< Decimation rate: 256 */
    DECI_RATE_1024 = 2                  /*!< Decimation rate: 1024 */
};

/// ADC buffer input type
enum BUFF_IN_TYPE
{
    ADC_BUFIN_VCM = 1,                  /*!< VCM */
    ADC_BUFIN_CHANNEL = 2,              /*!< ADC channel */
    ADC_BUFIN_GND = 3                   /*!< GND */
};

/// ADC input buffer gain control
enum ADC_BUFF_GAIN
{
    ADC_BUF_NEG_6DB = 0,                /*!< -6dB */
    ADC_BUF_0DB = 1,                    /*!<  0dB */
    ADC_BUF_POS_6DB = 2,                /*!<  6dB */
    ADC_BUF_POS_12DB = 3,               /*!< 12dB */
    ADC_BUF_GAIN_BYPASS = 4,            /*!< Bypass ADC input buffer gain stage */
    ADC_BUF_BYPASS = 8,                 /*!< Bypass ADC input buffer */
};

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Enable or disable adc.
 * @param[in]   able        MASK_ENABLE or MASK_DISABLE
 * @description
 *  This function is used to enable or disable ADC module.
 *****************************************************************************************
 */
__STATIC_INLINE void adc_enable(uint32_t able)
{
    adc_adc_SetADC0WithMask(QN_ADC, ADC_MASK_ADC_EN, able);

    if (able == MASK_DISABLE) {
        dev_allow_sleep(PM_MASK_ADC_ACTIVE_BIT);
    }
}

/**
 ****************************************************************************************
 * @brief   Enable ADC module clock
 * @description
 *  This function is used to enable ADC module clock
 *
 *****************************************************************************************
 */
__STATIC_INLINE void adc_clock_on(void)
{
    // enable ADC module clock
    syscon_SetCRSC(QN_SYSCON, SYSCON_MASK_GATING_ADC);
}

/**
 ****************************************************************************************
 * @brief   Disable ADC module clock
 * @description
 *  This function is used to disable ADC module clock
 *
 *****************************************************************************************
 */
__STATIC_INLINE void adc_clock_off(void)
{
    // disable ADC module clock
    syscon_SetCRSS(QN_SYSCON, SYSCON_MASK_GATING_ADC);
}

/**
 ****************************************************************************************
 * @brief   Power on ADC
 * @description
 *  This function is used to power on ADC module
 *
 *****************************************************************************************
 */
__STATIC_INLINE void adc_power_on(void)
{
    // power on ADC module
    syscon_SetPGCR1WithMask(QN_SYSCON, SYSCON_MASK_DIS_SAR_ADC|SYSCON_MASK_DIS_SAR_BUF, MASK_DISABLE);
}

/**
 ****************************************************************************************
 * @brief   Power off ADC
 * @description
 *  This function is used to power off ADC module
 *
 *****************************************************************************************
 */
__STATIC_INLINE void adc_power_off(void)
{
    // power off ADC module
    syscon_SetPGCR1WithMask(QN_SYSCON, SYSCON_MASK_DIS_SAR_ADC|SYSCON_MASK_DIS_SAR_BUF, MASK_ENABLE);
}

/**
 ****************************************************************************************
 * @brief   Reset ADC module
 * @description
 *  This function is used to reset ADC module
 *
 *****************************************************************************************
 */
__STATIC_INLINE void adc_reset(void)
{
    // Reset ADC module
    syscon_SetADCCRWithMask(QN_SYSCON, SYSCON_MASK_ADC_DIG_RST, MASK_DISABLE);
    syscon_SetADCCRWithMask(QN_SYSCON, SYSCON_MASK_ADC_DIG_RST, MASK_ENABLE);
}


/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */
#if CONFIG_ENABLE_ROM_DRIVER_ADC==TRUE

typedef void (*p_adc_void)(void);
typedef void (*p_adc_init)(enum ADC_WORK_CLK work_clk, enum ADC_TRIG_SRC trig_src, enum ADC_REF ref_vol);
typedef void (*p_adc_buf_in_set)(enum BUFF_IN_TYPE buf_in_p, enum BUFF_IN_TYPE buf_in_n);
typedef void (*p_adc_buf_gain_set)(enum ADC_BUFF_GAIN gain);
typedef void (*p_adc_compare_init)(enum WCMP_DATA data, int16_t high, int16_t low, void (*callback)(void));
typedef void (*p_adc_decimation_enable)(enum DECIMATION_RATE rate, uint32_t able);
typedef void (*p_adc_read)(enum ADC_MODE mode, enum ADC_CH start_ch, enum ADC_CH end_ch, int16_t *buf, uint32_t samples, void (*callback)(void));

#define adc_clean_fifo        ((p_adc_void)              _adc_clean_fifo)
#define adc_init              ((p_adc_init)              _adc_init)
#define adc_buf_in_set      ((p_adc_buf_in_set)      _adc_buf_in_set)
#define adc_buf_gain_set      ((p_adc_buf_gain_set)      _adc_buf_gain_set)
#define adc_compare_init      ((p_adc_compare_init)      _adc_compare_init)
#define adc_decimation_enable ((p_adc_decimation_enable) _adc_decimation_enable)
#define adc_read              ((p_adc_read)              _adc_read)

#else

#if CONFIG_ADC_DEFAULT_IRQHANDLER==TRUE
void ADC_IRQHandler(void);
#endif

extern void adc_clean_fifo(void);
extern void adc_init(enum ADC_WORK_CLK work_clk, enum ADC_TRIG_SRC trig_src, enum ADC_REF ref_vol);
extern void adc_buf_in_set(enum BUFF_IN_TYPE buf_in_p, enum BUFF_IN_TYPE buf_in_n);
extern void adc_buf_gain_set(enum ADC_BUFF_GAIN gain);
extern void adc_compare_init(enum WCMP_DATA data, int16_t high, int16_t low, void (*callback)(void));
extern void adc_decimation_enable(enum DECIMATION_RATE rate, uint32_t able);
extern void adc_read(enum ADC_MODE mode, enum ADC_CH start_ch, enum ADC_CH end_ch, int16_t *buf, uint32_t samples, void (*callback)(void));
extern void adc_offset_get(void);
extern int16_t ADC_SING_RESULT_mV(int16_t adc_data);
extern int16_t ADC_DIFF_RESULT_mV(int16_t adc_data);
#endif

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */

/// @} ADC
#endif /* CONFIG_ENABLE_DRIVER_ADC==TRUE */
#endif /* _ADC_H_ */
