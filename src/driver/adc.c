/**
 ****************************************************************************************
 *
 * @file adc.c
 *
 * @brief ADC driver for QN9020.
 *
 * Copyright (C) Quintic 2012-2013
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup  ADC
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "adc.h"
#if CONFIG_ENABLE_DRIVER_ADC==TRUE && CONFIG_ENABLE_ROM_DRIVER_ADC==FALSE
#include "nvds.h"
#if ADC_DMA_EN==TRUE
#include "dma.h"
#endif

/*
 * STRUCTURE DEFINITIONS
 ****************************************************************************************
 */

///ADC environment parameters
struct adc_env_tag
{
    enum ADC_MODE       mode;
    enum ADC_CH         start_ch;  // current channel, in scan mode, as start channel
    enum ADC_CH         end_ch;
    int32_t             samples;
    int16_t             *bufptr;
#if ADC_CALLBACK_EN==TRUE
    void                (*callback)(void);
#endif
#if ADC_WCMP_CALLBACK_EN==TRUE
    void                (*wcmp_callback)(void);
#endif
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
#define ADC_INT_REF_VOL_DEFAULT                      (1000)

#if CONFIG_ADC_DEFAULT_IRQHANDLER==TRUE
///ADC environment variable
static struct adc_env_tag adc_env;
#endif
///ADC trigger source
static enum ADC_TRIG_SRC adc_trigger;
///ADC SCAN channel number
static uint8_t scan_ch_num;

static uint16_t ADC_SCALE = ADC_INT_REF_VOL_DEFAULT;
static uint16_t ADC_VCM = (ADC_INT_REF_VOL_DEFAULT>>1);
static int16_t ADC_OFFSET = 0;


/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
//{ 64bit function use the standard compiler helper function library in Run-time ABI for the ARM 
#ifdef __ICCARM__ // for IAR compiler
#define __QAEABI __nounwind __interwork __softfp __aapcs_core
#else // for KEIL compiler
#define __QAEABI
#endif

//64bit signed multiplication
extern __QAEABI int64_t __aeabi_lmul(int64_t x, int64_t y);
//} end


/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

#if CONFIG_ADC_DEFAULT_IRQHANDLER==TRUE
 /**
 ****************************************************************************************
 * @brief ADC interrupt handler
 ****************************************************************************************
 */
void ADC_IRQHandler(void)
{
    uint32_t status, data;

    status = adc_adc_GetSR(QN_ADC);
    if (status & ADC_MASK_DAT_RDY_IF)
    {
        while (adc_adc_GetSR(QN_ADC) & ADC_MASK_DAT_RDY_IF)
        {
            /* clear interrupt flag by read data */
            data = adc_adc_GetDATA(QN_ADC);
            if (adc_env.samples > 0) {
                *adc_env.bufptr++ = data;
                adc_env.samples--;

                if (adc_env.samples == 0) {
                    adc_enable(MASK_DISABLE); // disable ADC
                    adc_clean_fifo();
                    NVIC_ClearPendingIRQ(ADC_IRQn);

#if ADC_CALLBACK_EN==TRUE
                    if (adc_env.callback != NULL)
                    {
                        adc_env.callback();
                    }
#endif
                }
                else {

                    // single mode enable, software trigger
                    if ((adc_trigger == ADC_TRIG_SOFT) && ((adc_env.mode == SINGLE_SCAN_MOD)||(adc_env.mode == SINGLE_MOD))) {

                        scan_ch_num--;
                        if (scan_ch_num == 0) {

                            // SFT_START 0->1 trigger ADC conversion
                            adc_adc_SetADC0WithMask(QN_ADC, ADC_MASK_SFT_START, MASK_DISABLE);
                            adc_adc_SetADC0WithMask(QN_ADC, ADC_MASK_SFT_START, MASK_ENABLE);

                            // restore scan_ch_num
                            if (adc_env.mode == SINGLE_SCAN_MOD) {
                                scan_ch_num = adc_env.end_ch - adc_env.start_ch + 1;
                            }
                            else {
                                scan_ch_num = 1;
                            }
                        }
                    }
                }
            }
        }
    }

    if (status & ADC_MASK_FIFO_OF_IF)
    {
        adc_clean_fifo();

        /* clear interrupt flag */
        adc_adc_ClrSR(QN_ADC, ADC_MASK_FIFO_OF_IF);
    }


    if (status & ADC_MASK_WCMP_IF)
    {
        /* clear interrupt flag */
        adc_adc_ClrSR(QN_ADC, ADC_MASK_WCMP_IF);

#if ADC_WCMP_CALLBACK_EN==TRUE
        if (adc_env.wcmp_callback != NULL)
        {
            adc_env.wcmp_callback();
        }
#endif
    }
}
#endif /* CONFIG_ADC_DEFAULT_IRQHANDLER==TRUE */

/**
 ****************************************************************************************
 * @brief  Clean ADC FIFO
 * @description
 *  This function is used to clean ADC FIFO.
 *****************************************************************************************
 */
void adc_clean_fifo(void)
{
    // clean fifo
    while(adc_adc_GetSR(QN_ADC) & ADC_MASK_DAT_RDY_IF)
    {
        adc_adc_GetDATA(QN_ADC);
    }
}

/**
 ****************************************************************************************
 * @brief   Initialize ADC
 * @param[in]    work_clk       work_clk = (ADC_SOURCE_CLK / (2<<ADC_DIV)), ADC_DIV = 0 - 15,
 *  ADC_SOURCE_CLK is 32k or system clock(4 types, decided by CLK_MUX), the max work_clk = 1MHz
 * @param[in]    trig_src       ADC trigger source
 * @param[in]    ref_vol        ADC reference voltage
 * @description
 *  This function is used to set ADC module work clock, resolution, trigger source, reference
 *  voltage and interrupt, ADC buffer gain default set to bypass.
 *
 *****************************************************************************************
 */
void adc_init(enum ADC_WORK_CLK work_clk, enum ADC_TRIG_SRC trig_src, enum ADC_REF ref_vol)
{
    uint32_t reg;
    uint32_t mask;

#if CONFIG_ADC_DEFAULT_IRQHANDLER==TRUE
    adc_env.mode = SINGLE_MOD;
    adc_env.samples = 0;
    adc_env.bufptr = NULL;
#if ADC_CALLBACK_EN==TRUE
    adc_env.callback = NULL;
#endif
#endif
    adc_trigger = trig_src;

    // enable ADC module clock
    adc_clock_on();
    // power on ADC module
    adc_power_on();
    // delay for power on
    delay(300);
    // reset adc register
    adc_reset();

    // set work clock
    mask = SYSCON_MASK_ADC_CLK_SEL
         | SYSCON_MASK_ADC_DIV_BYPASS
         | SYSCON_MASK_ADC_DIV;
    reg =  work_clk;
    syscon_SetADCCRWithMask(QN_SYSCON, mask, reg);


    // adc int
    mask = ADC_MASK_INT_MASK        // mask ADC int
         | ADC_MASK_FIFO_OF_IE      // mask ADC fifo overflow int
         | ADC_MASK_DAT_RDY_IE      // mask ADC data ready int
         | ADC_MASK_BUF_PD          // SAR ADC buffer is power down
         | ADC_MASK_VREF_SEL        // Select referecnce voltage
         | ADC_MASK_INBUF_BP        // Bypass SAR ADC input buffer
         | ADC_MASK_BUF_GAIN_BP     // Bypass SAR ADC input buffer gain stage
         | ADC_MASK_BUF_IN_P        // SAR ADC buffer input selection  AN0/2
         | ADC_MASK_BUF_IN_N        // SAR ADC buffer input selection  AN1/3
         | ADC_MASK_RES_SEL         // SAR ADC resolution
         ;
    reg = (ref_vol << ADC_POS_VREF_SEL)
        | ADC_BUFF_CTRL_CFG
        | (ADC_BUFF_IN_P_CFG << ADC_POS_BUF_IN_P)    // ADC channel
        | (ADC_BUFF_IN_N_CFG << ADC_POS_BUF_IN_N)    // VCM
        | (CFG_ADC_RESOLUTION << ADC_POS_RES_SEL)    // 12bit
        ;

#if CONFIG_ADC_ENABLE_INTERRUPT==TRUE
    reg |= (ADC_MASK_INT_MASK | ADC_MASK_FIFO_OF_IE | ADC_MASK_DAT_RDY_IE);

    /* Enable the adc Interrupt */
    NVIC_EnableIRQ(ADC_IRQn);
#endif

    adc_adc_SetADC1WithMask(QN_ADC, mask, reg);

    // read from NVDS
    uint16_t len = 4;
    uint32_t data = 0;
    ADC_SCALE = ADC_INT_REF_VOL_DEFAULT;
    if (NVDS_OK == nvds_get(NVDS_TAG_ADC_INT_REF_SCALE, &len, (uint8_t *)&data)) {
        if ((data>900) && (data<1100)) {
            ADC_SCALE = data;
        }
    }

    if (ref_vol == ADC_INT_REF) {
        ADC_VCM = ADC_SCALE>>1;
        if (NVDS_OK == nvds_get(NVDS_TAG_ADC_INT_REF_VCM, &len, (uint8_t *)&data)) {
            if ((data>450) && (data<550)) {
                ADC_VCM = data;
            }
        }
    }
    else {
        ADC_VCM = ADC_EXT_REF_VOL>>1;
    }
}

/**
 ****************************************************************************************
 * @brief   Set ADC buffer input source
 * @param[in]   buf_in_p        ADC Buffer input+
 * @param[in]   buf_in_n        ADC Buffer input-
 * @description
 *  This function is used to set ADC buffer input source
 *
 *****************************************************************************************
 */
void adc_buf_in_set(enum BUFF_IN_TYPE buf_in_p, enum BUFF_IN_TYPE buf_in_n)
{
    uint32_t mask, reg;
    // adc buffer input
    mask = ADC_MASK_BUF_IN_P        // SAR ADC buffer input selection  AN0/2
         | ADC_MASK_BUF_IN_N        // SAR ADC buffer input selection  AN1/3
         ;
    reg = (buf_in_p << ADC_POS_BUF_IN_P)
        | (buf_in_n << ADC_POS_BUF_IN_N)
        ;

    adc_adc_SetADC1WithMask(QN_ADC, mask, reg);
}

/**
 ****************************************************************************************
 * @brief  ADC buffer gain set
 * @param[in]    gain         ADC buffer gain stage
 * @description
 *  This function is used to set ADC buffer gain stage, and only available at the input mode 
 *  with buffer driver.
 *****************************************************************************************
 */
void adc_buf_gain_set(enum ADC_BUFF_GAIN gain)
{
    uint32_t reg, mask;

    mask = ADC_MASK_INBUF_BP        // Bypass SAR ADC input buffer
         | ADC_MASK_BUF_GAIN_BP     // Bypass SAR ADC input buffer gain stage
         | ADC_MASK_BUF_GAIN        // SAR ADC buffer gain control
         ;
    reg = (gain << ADC_POS_BUF_GAIN);

    adc_adc_SetADC1WithMask(QN_ADC, mask, reg);
}

/**
 ****************************************************************************************
 * @brief  Initialize ADC comparator
 * @param[in]    data         data source of comparator
 * @param[in]    high         high level of compare window: higher than this level will generate interrupt
 * @param[in]    low          lwo level of compare window: lower than this level will generate interrupt
 * @param[in]    callback     callback after interrupt
 * @description
 *  This function is used to initialize ADC window comparator.
 *****************************************************************************************
 */
void adc_compare_init(enum WCMP_DATA data, int16_t high, int16_t low, void (*callback)(void))
{
    uint32_t reg, mask;

#if CONFIG_ADC_DEFAULT_IRQHANDLER==TRUE && ADC_WCMP_CALLBACK_EN==TRUE
    adc_env.wcmp_callback = callback;
#endif

    // set window
    reg = (high << ADC_POS_WCMP_TH_HI)   // window compare high threshold
        | (low & 0xFFFF);               // window compare low threshold
    adc_adc_SetADC2(QN_ADC, reg);

    // enable compare
    mask = ADC_MASK_INT_MASK |      // mask ADC int
           ADC_MASK_WCMP_IE |       // mask window compare interrupt
           ADC_MASK_WCMP_SEL |      // select compare input data
           ADC_MASK_WCMP_EN;        // enable window compare
    reg = ADC_MASK_INT_MASK |       // mask ADC int
          ADC_MASK_WCMP_IE |        // mask window compare interrupt
          data |                    // select compare input data as adc data
          ADC_MASK_WCMP_EN;         // enable window compare
    adc_adc_SetADC1WithMask(QN_ADC, mask, reg);
}

/**
 ****************************************************************************************
 * @brief  Enable/Disable ADC decimation
 * @param[in]    rate      decimation rate
 * @param[in]    able      mask of enable or disable
 * @description
 *  This function is used to enable or disable ADC decimation
 *****************************************************************************************
 */
void adc_decimation_enable(enum DECIMATION_RATE rate, uint32_t able)
{
    uint32_t reg, mask;

    // enable decimation
    mask = ADC_MASK_DECI_DIV        // decimation rate
         | ADC_MASK_DECI_EN;        // enable decimation
    reg = (rate << ADC_POS_DECI_DIV)
        | (ADC_MASK_DECI_EN&able);
    adc_adc_SetADC1WithMask(QN_ADC, mask, reg);
}

/**
 ****************************************************************************************
 * @brief  Read ADC channel
 * @param[in]    mode         ADC mode: single, continue, single scan, continue scan
 * @param[in]    start_ch     ADC current channel, or start channel in scan mode
 * @param[in]    end_ch       ADC end channel in scan mode
 * @param[in]    buf          ADC result
 * @param[in]    samples      sample number
 * @param[in]    callback     callback after convert
 * @description
 *  This function is used to read specified ADC channel.
 * @note
 *  When use scaning mode, only can select first 6 channel (AIN0,AIN1,AIN2,AIN3,AIN01,AIN23)
 *****************************************************************************************
 */
void adc_read(enum ADC_MODE mode, enum ADC_CH start_ch, enum ADC_CH end_ch, int16_t *buf, uint32_t samples, void (*callback)(void))
{
    uint32_t reg;
    uint32_t mask;
#if (CONFIG_ADC_ENABLE_INTERRUPT==FALSE) && (ADC_DMA_EN==FALSE)
    uint32_t i;
#endif

#if CONFIG_ADC_DEFAULT_IRQHANDLER==TRUE

    adc_env.mode = mode;
    adc_env.start_ch = start_ch;
    adc_env.end_ch = end_ch;
    adc_env.bufptr = buf;
    adc_env.samples = samples;
    adc_env.callback = callback;

#endif

    // single scan mode, need read all of the channel after once trigger
    if (mode == SINGLE_SCAN_MOD) {
        scan_ch_num = end_ch - start_ch + 1;
    }
    else {
        scan_ch_num = 1;
    }

#if (CONFIG_ADC_ENABLE_INTERRUPT==FALSE) && (ADC_DMA_EN==TRUE)
    dma_init();
    // samples*2 <= 0x7FF
    dma_rx(DMA_TRANS_HALF_WORD, DMA_ADC, (uint32_t)buf, samples*2, callback);
#endif

    mask = ADC_MASK_SCAN_CH_START
         | ADC_MASK_SCAN_CH_END
         | ADC_MASK_SCAN_INTV
         | ADC_MASK_SCAN_EN
         | ADC_MASK_SINGLE_EN
         | ADC_MASK_START_SEL
         | ADC_MASK_SFT_START
         | ADC_MASK_POW_UP_DLY
         | ADC_MASK_POW_DN_CTRL
         | ADC_MASK_ADC_EN;                    // enable ADC

    reg = (start_ch << ADC_POS_SCAN_CH_START)  // set adc channel, or set scan start channel
        | (end_ch << ADC_POS_SCAN_CH_END)      // set scan end channel
        | (0x03 << ADC_POS_SCAN_INTV)          // should not be set to 0 at single mode
        | (adc_trigger << ADC_POS_START_SEL)   // select ADC trigger source
        | (0x9 << ADC_POS_POW_UP_DLY)
        | ADC_MASK_POW_DN_CTRL                 // enable power down control by hardware, only work in single mode
        | ADC_MASK_ADC_EN;                     // enable ADC

    if ((mode == SINGLE_SCAN_MOD) || (mode == SINGLE_MOD)) { // default is continue
        reg |= ADC_MASK_SINGLE_EN;
    }
    if ((mode == SINGLE_SCAN_MOD) || (mode == CONTINUE_SCAN_MOD)) { // default is not scan
        reg |= ADC_MASK_SCAN_EN;
    }

    adc_adc_SetADC0WithMask(QN_ADC, mask, reg);

    if (adc_trigger == ADC_TRIG_SOFT) {
        // SFT_START 0->1 trigger ADC conversion
        adc_adc_SetADC0WithMask(QN_ADC, ADC_MASK_SFT_START, MASK_ENABLE);
    }

#if CONFIG_ADC_ENABLE_INTERRUPT==TRUE
    dev_prevent_sleep(PM_MASK_ADC_ACTIVE_BIT);
#elif (CONFIG_ADC_ENABLE_INTERRUPT==FALSE) && (ADC_DMA_EN==FALSE)

    // polling
    while(samples > 0)
    {
        for (i = 0; i < scan_ch_num; i++) {
            while(!(adc_adc_GetSR(QN_ADC) & ADC_MASK_DAT_RDY_IF));
            *buf++ = adc_adc_GetDATA(QN_ADC);
            samples--;
        }

        // single mode enable, software trigger
        if ((adc_trigger == ADC_TRIG_SOFT) && ((mode == SINGLE_SCAN_MOD)||(mode == SINGLE_MOD))) {
            // SFT_START 0->1 trigger ADC conversion
            adc_adc_SetADC0WithMask(QN_ADC, ADC_MASK_SFT_START, MASK_DISABLE);
            adc_adc_SetADC0WithMask(QN_ADC, ADC_MASK_SFT_START, MASK_ENABLE);
        }
    }

    // disable ADC
    adc_enable(MASK_DISABLE);

#if ADC_CALLBACK_EN==TRUE
    if (callback != NULL)
    {
        callback();
    }
#endif

#endif
}

static volatile uint8_t adc_offset_get_done = 0;
static void adc_offset_get_cb(void)
{
    adc_offset_get_done = 1;
}

/**
 ****************************************************************************************
 * @brief   Get ADC offset for conversion result correction
 * @description
 *  This function is used to get ADC offset for conversion result correction, and should be called after 
 *  ADC initialization and buffer gain settings.
 *
 *****************************************************************************************
 */
void adc_offset_get(void)
{
    enum ADC_TRIG_SRC adc_trigger_restore = adc_trigger;
    uint32_t buff_in_restore = adc_adc_GetADC1(QN_ADC);

    adc_trigger = ADC_TRIG_SOFT;
    adc_buf_in_set(ADC_BUFIN_VCM, ADC_BUFIN_VCM);

    adc_offset_get_done = 0;
    adc_read(SINGLE_MOD, AIN01, AIN01, &ADC_OFFSET, 1, adc_offset_get_cb);
    while (!adc_offset_get_done);
    
    // restore trigger source
    adc_trigger = adc_trigger_restore;
    // restore buffer input source
    adc_adc_SetADC1WithMask(QN_ADC, ADC_MASK_BUF_IN_P|ADC_MASK_BUF_IN_N, buff_in_restore);
}

/**
 ****************************************************************************************
 * @brief   ADC single mode result(mv)
 * @param[in]   adc_data        ADC data
 * @return voltage value(mv)
 * @description
 *  This function is used to calculate ADC single mode voltage value
 *
 *****************************************************************************************
 */
int16_t ADC_SING_RESULT_mV(int16_t adc_data)
{
    int16_t result;
    result = (__aeabi_lmul(adc_data - ADC_OFFSET, CFG_ADC_REF_VOL) >> 11) + ADC_VCM;
    return result;
}

/**
 ****************************************************************************************
 * @brief   ADC differential mode result(mv)
 * @param[in]   adc_data        ADC data
 * @return voltage value(mv)
 * @description
 *  This function is used to calculate ADC differential mode voltage value
 *
 *****************************************************************************************
 */
int16_t ADC_DIFF_RESULT_mV(int16_t adc_data)
{
    int16_t result;
    result = __aeabi_lmul(adc_data - ADC_OFFSET, CFG_ADC_REF_VOL) >> 11;
    return result;
}

#endif /* CONFIG_ENABLE_DRIVER_ADC==TRUE */
/// @} ADC
