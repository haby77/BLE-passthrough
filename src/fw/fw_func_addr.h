/**
 ****************************************************************************************
 *
 * @file fw_func_addr.h
 *
 * @brief The address of code and data in the firmware.
 *
 * Copyright (C) Quintic 2012-2013
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */
#ifndef _FW_FUNC_ADDR_H_
#define _FW_FUNC_ADDR_H_


/** @addtogroup QN_ROM_DRIVER_ADDR QN9020 Rom Driver Address
  @{
 */


#ifdef __cplusplus
#if __cplusplus
extern "C"{
#endif
#endif /* __cplusplus */

#include "app_config.h"

#if (defined(QN_9020_B2) || defined(QN_9020_B1))

#define _calibration_init                               0x01000339
#define _seq_calibration                                0x010003ad
#define _freq_hop_calibration                           0x01000419
#define _clock_32k_calibration                          0x0100048d
#define _ref_pll_calibration                            0x01000563
#define _rco_calibration                                0x0100058d
#define _DMA_IRQHandler                                 0x01000745
#define _dma_init                                       0x01000765
#define _dma_abort                                      0x0100078b
#define _dma_memory_copy                                0x01000793
#define _dma_tx                                         0x010007ad
#define _dma_rx                                         0x010007d9
#define _dma_transfer                                   0x01000807
#define _GPIO_IRQHandler                                0x0100085d
#define _gpio_init                                      0x01000881
#define _gpio_read_pin                                  0x01000891
#define _gpio_write_pin                                 0x010008a5
#define _gpio_set_direction                             0x010008bb
#define _gpio_read_pin_field                            0x010008cb
#define _gpio_write_pin_field                           0x010008d5
#define _gpio_set_direction_field                       0x010008e7
#define _gpio_toggle_pin                                0x010008f5
#define _gpio_set_interrupt                             0x0100090d
#define _gpio_enable_interrupt                          0x01000939
#define _gpio_disable_interrupt                         0x01000943
#define _gpio_pull_set                                  0x0100094d
#define _gpio_wakeup_config                             0x010009b3
#define _I2C_IRQHandler                                 0x01000a11
#define _i2c_init                                       0x01000ad5
#define _i2c_bus_check                                  0x01000afd
#define _i2c_read                                       0x01000b17
#define _i2c_write                                      0x01000b79
#define _I2C_BYTE_READ                                  0x01000bb9
#define _I2C_BYTE_READ2                                 0x01000bdb
#define _I2C_nBYTE_READ2                                0x01000c03
#define _I2C_BYTE_WRITE                                 0x01000c47
#define _I2C_BYTE_WRITE2                                0x01000c63
#define _I2C_nBYTE_WRITE2                               0x01000c83
#define _PWM0_IRQHandler                                0x01000ccd
#define _pwm_init                                       0x01000cdb
#define _rf_enable_sw_set_freq                          0x01000d7d
#define _rf_set_freq                                    0x01000d97
#define _rf_enable                                      0x01000df5
#define _rf_tx_power_level_set                          0x01000e55
#define _rf_tx_power_level_get                          0x01000e69
#define _rf_init                                        0x01000ed1
#define _dec2bcd                                        0x01000f15
#define _bcd2dec                                        0x01000f25
#define _rtc_time_get                                   0x01000f61
#define _RTC_IRQHandler                                 0x01000f77
#define _RTC_CAP_IRQHandler                             0x01000f91
#define _rtc_init                                       0x01000fb1
#define _rtc_calibration                                0x01000fd1
#define _rtc_correction                                 0x01000ff7
#define _rtc_capture_enable                             0x01001023
#define _rtc_capture_disable                            0x0100105b
#define _rtc_time_set                                   0x01001075
#define _is_flash_busy                                  0x0100110f
#define _set_flash_clock                                0x010010fd
#define _read_flash_id                                  0x01001151
#define _sector_erase_flash                             0x0100116f
#define _block_erase_flash                              0x01001197
#define _chip_erase_flash                               0x010011bd
#define _read_flash                                     0x010011cf
#define _write_flash                                    0x010011f5
#define _power_on_or_off_flash                          0x01001217
#define _SPI0_TX_IRQHandler                             0x01001353
#define _SPI0_RX_IRQHandler                             0x01001399
#define _spi_init                                       0x01001403
#define _spi_read                                       0x01001475
#define _spi_write                                      0x010014c1
#define _timer_delay                                    0x01001549
#define _TIMER0_IRQHandler                              0x01001575
#define _TIMER1_IRQHandler                              0x010015b3
#define _TIMER2_IRQHandler                              0x010015f1
#define _TIMER3_IRQHandler                              0x0100162f
#define _timer_init                                     0x0100166d
#define _timer_config                                   0x010016ed
#define _timer_pwm_config                               0x01001729
#define _timer_capture_config                           0x0100176d
#define _UART0_TX_IRQHandler                            0x01001879
#define _UART0_RX_IRQHandler                            0x010018b3
#define _uart_init                                      0x01001915
#define _uart_read                                      0x01001985
#define _uart_write                                     0x010019b9
#define _uart_printf                                    0x010019ed
#define _uart_finish_transfers                          0x01001a01
#define _uart_check_tx_free                             0x01001a09
#define _uart_flow_on                                   0x01001a3d
#define _uart_flow_off                                  0x01001a67
#define _nvds_read                                      0x01001f59
#define _nvds_write                                     0x01001fd1
#define _nvds_erase                                     0x010020ad
#define _nvds_init_memory                               0x010020c3
#define _nvds_init                                      0x0100214f
#define _nvds_get                                       0x01002187
#define _nvds_del                                       0x010021c5
#define _nvds_lock                                      0x010021ff
#define _nvds_put                                       0x0100222b
#define _co_bt_bdaddr_compare                           0x01002ae5
#define _co_list_init                                   0x01002b8d
#define _co_list_push_back                              0x01002b95
#define _co_list_push_front                             0x01002c1b
#define _co_list_pop_front                              0x01002c33
#define _co_list_extract                                0x01002c41
#define _co_list_find                                   0x01002c63
#define _co_list_merge                                  0x01002c7d
#define _task_desc_register                             0x01002cfd
#define _ke_state_set                                   0x01002d61
#define _ke_state_get                                   0x01002d89
#define _ke_msg_discard                                 0x01002eaf
#define _ke_msg_save                                    0x01002eb3
#define _ke_timer_set                                   0x01002f3b
#define _ke_timer_clear                                 0x01002faf
#define _ke_timer_sleep_check                           0x01003079
#define _ke_accurate_timer_set                          0x010030d1
#define _ke_evt_set                                     0x01003159
#define _ke_evt_clear                                   0x01003173
#define _ke_evt_callback_set                            0x0100318d 
#define _ke_malloc                                      0x01003289
#define _ke_free                                        0x010032db
#define _ke_msg_alloc                                   0x0100335d
#define _ke_msg_send                                    0x0100338b
#define _ke_msg_send_front                              0x010033b1
#define _ke_msg_send_basic                              0x010033d7
#define _ke_msg_forward                                 0x010033e5
#define _ke_msg_free                                    0x010033ef
#define _aci_init                                       0x01003423
#define _aci_tx_done                                    0x01003441
#define _aci_enter_sleep                                0x0100348b
#define _aci_exit_sleep                                 0x010034d1
#define _hci_init                                       0x01003ec7
#define _hci_tx_done                                    0x01004875
#define _hci_enter_sleep                                0x010048b9
#define _hci_exit_sleep                                 0x010048d9
#define _lld_crypt_isr                                  0x010072e7
#define _lld_evt_rx_isr                                 0x010085b1
#define _lld_evt_start_isr                              0x0100858d
#define _lld_evt_end_isr                                0x0100859f
#define _lld_evt_timer_isr                              0x010085c3
#define _deep_sleep_init                                0x0100876f
#define _set_32k_ppm                                    0x010087df
#define _set_32k_freq                                   0x010087e5
#define _revise_wakeup_delay                            0x010087ff
#define _attm_get_mtu                                   0x0100ab1b
#define _attsdb_add_service                             0x0100ae61
#define _attsdb_get_service                             0x0100af2d
#define _attsdb_add_attribute                           0x0100af75
#define _attsdb_destroy                                 0x0100b09b
#define _attsdb_get_attribute                           0x0100b0b5
#define _attsdb_get_next_att                            0x0100b0d3
#define _attsdb_att_update_value                        0x0100b17b
#define _attsdb_att_set_value                           0x0100b1b5
#define _attsdb_att_partial_value_update                0x0100b1bb
#define _attsdb_att_get_value                           0x0100b1ed
#define _attsdb_att_get_uuid                            0x0100b20d
#define _attsdb_att_set_permission                      0x0100b281
#define _attsdb_att_get_permission                      0x0100b299
#define _attsdb_att_update_perm                         0x0100b2b3
#define _attsdb_svc_set_permission                      0x0100b2d3
#define _attsdb_svc_get_permission                      0x0100b2eb
#define _atts_write_no_resp                             0x0100c78d
#define _atts_write_rsp_send                            0x0100cb4f
#define _atts_svc_create_db                             0x0100cb7d
#define _gap_get_rec_idx                                0x0100d07d
#define _gap_get_lk_sec_status                          0x0100d0cd
#define _gap_get_enc_keysize                            0x0100d0d7
#define _gap_get_conhdl                                 0x0100d0e1
#define _gap_get_security                               0x0100d0fd
#define _gap_send_discon_cmp_evt                        0x0100d103
#define _gap_send_connect_req                           0x0100d123
#define _ble_prevent_sleep_set                          0x01015207
#define _ble_wakeup                                     0x01015221
#define _ble_wakeup_end                                 0x01015253
#define _ble_disable                                    0x01015275
#define _ble_enable                                     0x01015285
#define _ble_reset                                      0x01015331
#define _ble_version                                    0x01015375
#define _ble_schedule                                   0x010153a7
#define _ble_sleep                                      0x010153e3
#define _ble_isr                                        0x01015455
#define _reg_sleep_cb                                   0x010154fd
#define _qn_config_init                                 0x01015545
#define _config_work_mode                               0x01015611
#define _save_cal_setting                               0x0101564d
#define _save_ble_reg                                   0x0101568d
#define _save_sys_regs                                  0x010156d7
#define _save_ble_setting                               0x01015743
#define _restore_ble_setting                            0x0101574f
#define _enable_ble_sleep                               0x0101578f
#define _set_max_sleep_duration                         0x01015797
#define _sw_wakeup_ble_hw                               0x010157b1
#define _qn_plf_init                                    0x01015981
#define _qn_ble_init                                    0x010159a5
#define _prf_init_reg                                   0x01015a39
#define _prf_disp_disconnect_reg                        0x01015a3f
#define _rtc_capture_env                                0x1000cdb4
#define _timer0_env                                     0x1000cdc4
#define _timer1_env                                     0x1000cdcc
#define _timer2_env                                     0x1000cdd4
#define _timer3_env                                     0x1000cddc
#define _ke_evt_hdlr                                    0x1000cdf4
#define _rtc_env                                        0x1000d03c
#define _nvds_env                                       0x1000d0b0
#define _ke_env                                         0x1000d1fc
#define _sleep_api                                      0x1000dbf0
#define _gap_env                                        0x1000dd5c
#define _ble_env                                        0x1000f2e8
#define _ble_rf                                         0x1000f2f4
#define _config_api                                     0x1000f32c

#elif defined(QN_9020_B0)

#define _adc_clean_fifo                                 0x01000351
#define _ADC_IRQHandler                                 0x0100035f
#define _adc_init                                       0x01000419
#define _adc_compare_init                               0x01000489
#define _adc_decimation_enable                          0x010004ad
#define _adc_read                                       0x010004c3
#define _COMPARATOR0_IRQHandler                         0x010005e5
#define _COMPARATOR1_IRQHandler                         0x010005f1
#define _comparator_init                                0x010005ff
#define _comparator_enable                              0x01000651
#define _battery_monitor_enable                         0x01000683
#define _brown_out_enable                               0x01000693
#define _temp_sensor_enable                             0x010006a3
#define _get_reset_source                               
#define _CALIB_IRQHandler                               0x010006d5
#define _calibration_init                               0x0100071b
#define _seq_calibration                                0x01000799
#define _freq_hop_calibration                           0x01000805
#define _clock_32k_calibration                          0x01000879
#define _ref_pll_calibration                            0x01000949
#define _rc_calibration                                 0x01000973
#define _lo_calibration                                 0x01000999
#define _lo_kcal_calibration                            0x010009cb
#define _pa_calibration                                 0x01000a01
#define _r_calibration                                  0x01000a8d
#define _ros_calibration                                0x01000afd
#define _rco_calibration                                0x01000b27
#define _DMA_IRQHandler                                 0x01000b91
#define _dma_init                                       0x01000bb1
#define _dma_abort                                      0x01000bd1
#define _dma_memory_copy                                0x01000bd9
#define _dma_tx                                         0x01000bef
#define _dma_rx                                         0x01000c17
#define _dma_transfer                                   0x01000c41
#define _GPIO_IRQHandler                                0x01000c85
#define _gpio_init                                      0x01000ca9
#define _gpio_read_pin                                  0x01000cb9
#define _gpio_write_pin                                 0x01000ccd
#define _gpio_set_direction                             0x01000ce3
#define _gpio_read_pin_field                            0x01000cf3
#define _gpio_write_pin_field                           0x01000cfd
#define _gpio_set_direction_field                       0x01000d0f
#define _gpio_toggle_pin                                0x01000d1d
#define _gpio_set_interrupt                             0x01000d35
#define _gpio_enable_interrupt                          0x01000d61
#define _gpio_disable_interrupt                         0x01000d6b
#define _gpio_pull_set                                  0x01000d75
#define _gpio_wakeup_config                             0x01000db1
#define _I2C_IRQHandler                                 0x01000ded
#define _i2c_init                                       0x01000eb1
#define _i2c_bus_check                                  0x01000ed9
#define _i2c_read                                       0x01000ef3
#define _i2c_write                                      0x01000f47
#define _I2C_BYTE_READ                                  0x01000f87
#define _I2C_BYTE_READ2                                 0x01000fa9
#define _I2C_nBYTE_READ2                                0x01000fd1
#define _I2C_BYTE_WRITE                                 0x01001015
#define _I2C_BYTE_WRITE2                                0x01001031
#define _I2C_nBYTE_WRITE2                               0x01001051
#define _PWM0_IRQHandler                                0x01001099
#define _pwm_init                                       0x010010a7
#define _rf_enable_sw_set_freq                          0x01001145
#define _rf_set_freq                                    0x0100115f
#define _rf_enable                                      0x010011bd
#define _rf_tx_power_level_set                          0x0100121d
#define _rf_tx_power_level_get                          0x01001231
#define _rf_init                                        0x01001299
#define _dec2bcd                                        0x010012dd
#define _bcd2dec                                        0x010012ed
#define _rtc_time_get                                   0x01001329
#define _RTC_IRQHandler                                 0x0100133f
#define _RTC_CAP_IRQHandler                             0x01001359
#define _rtc_init                                       0x01001379
#define _rtc_calibration                                0x01001399
#define _rtc_correction                                 0x010013bf
#define _rtc_capture_enable                             0x010013eb
#define _rtc_capture_disable                            0x01001421
#define _rtc_time_set                                   0x0100143b
#define _set_flash_clock                                0x010014c1
#define _is_flash_busy                                  0x010014d3
#define _read_flash_id                                  0x01001515
#define _sector_erase_flash                             0x01001533
#define _block_erase_flash                              0x0100155b
#define _chip_erase_flash                               0x01001581
#define _read_flash                                     0x01001593
#define _write_flash                                    0x010015b9
#define _power_on_or_off_flash                          0x010015db
#define _SPI0_TX_IRQHandler                             0x010016c3
#define _SPI0_RX_IRQHandler                             0x01001709
#define _spi_init                                       0x01001773
#define _spi_read                                       0x010017e5
#define _spi_write                                      0x01001831
#define _timer_delay                                    0x010018b9
#define _TIMER0_IRQHandler                              0x010018e5
#define _TIMER1_IRQHandler                              0x01001923
#define _TIMER2_IRQHandler                              0x01001961
#define _TIMER3_IRQHandler                              0x0100199f
#define _timer_init                                     0x010019dd
#define _timer_config                                   0x01001a5d
#define _timer_pwm_config                               0x01001a99
#define _timer_capture_config                           0x01001add
#define _UART0_TX_IRQHandler                            0x01001be9
#define _UART0_RX_IRQHandler                            0x01001c23
#define _uart_init                                      0x01001c85
#define _uart_read                                      0x01001cf5
#define _uart_write                                     0x01001d29
#define _uart_printf                                    0x01001d5d
#define _uart_finish_transfers                          0x01001d71
#define _uart_check_tx_free                             0x01001d79
#define _uart_flow_on                                   0x01001dad
#define _uart_flow_off                                  0x01001dd7
#define _wdt_unlock                                     0x01001ec5
#define _wdt_lock                                       0x01001ecd
#define _wdt_irq_clear                                  0x01001ed5
#define _WDT_IRQHandler                                 0x01001ee7
#define _wdt_init                                       0x01001ef3
#define _wdt_set                                        0x01001f35
#define _nvds_read                                      0x01002289
#define _nvds_write                                     0x010022f9
#define _nvds_erase                                     0x010023cd
#define _nvds_init_memory                               0x010023d7
#define _nvds_init                                      0x0100245f
#define _nvds_get                                       0x01002497
#define _nvds_del                                       0x010024d5
#define _nvds_lock                                      0x0100250f
#define _nvds_put                                       0x0100253b
#define _co_bt_bdaddr_compare                           0x01002df5
#define _co_list_init                                   0x01002e9d
#define _co_list_push_back                              0x01002ea5
#define _co_list_push_front                             0x01002f2b
#define _co_list_pop_front                              0x01002f43
#define _co_list_extract                                0x01002f51
#define _co_list_find                                   0x01002f73
#define _co_list_merge                                  0x01002f8d
#define _task_desc_register                             0x0100300d
#define _ke_state_set                                   0x01003071
#define _ke_state_get                                   0x01003099
#define _ke_msg_discard                                 0x010031bf
#define _ke_msg_save                                    0x010031c3
#define _ke_timer_set                                   0x0100324b
#define _ke_timer_clear                                 0x010032cb
#define _ke_timer_sleep_check                           0x01003399
#define _ke_evt_set                                     0x01003415
#define _ke_evt_clear                                   0x0100342f
#define _ke_malloc                                      0x01003525
#define _ke_free                                        0x01003577
#define _ke_msg_alloc                                   0x010035f9
#define _ke_msg_send                                    0x01003627
#define _ke_msg_send_front                              0x0100364d
#define _ke_msg_send_basic                              0x01003673
#define _ke_msg_forward                                 0x01003681
#define _ke_msg_free                                    0x0100368b
#define _aci_init                                       0x010036bf
#define _aci_tx_done                                    0x010036dd
#define _aci_enter_sleep                                0x01003727
#define _aci_exit_sleep                                 0x0100376d
#define _hci_init                                       0x01004163
#define _hci_tx_done                                    0x01004b11
#define _hci_enter_sleep                                0x01004b55
#define _hci_exit_sleep                                 0x01004b75
#define _lld_crypt_isr                                  0x01007519
#define _lld_evt_rx_isr                                 0x010087d5
#define _lld_evt_start_isr                              0x010087b1
#define _lld_evt_end_isr                                0x010087c3
#define _lld_evt_timer_isr                              0x010087e7
#define _deep_sleep_init                                0x01008979
#define _set_32k_ppm                                    0x010089e1
#define _revise_wakeup_delay                            0x010089f1
#define _attm_get_mtu                                   0x0100acbf
#define _attsdb_add_service                             0x0100afe1
#define _attsdb_get_service                             0x0100b0ad
#define _attsdb_add_attribute                           0x0100b0f5
#define _attsdb_destroy                                 0x0100b21b
#define _attsdb_get_attribute                           0x0100b235
#define _attsdb_get_next_att                            0x0100b253
#define _attsdb_att_update_value                        0x0100b2fb
#define _attsdb_att_set_value                           0x0100b335
#define _attsdb_att_partial_value_update                0x0100b33b
#define _attsdb_att_get_value                           0x0100b36d
#define _attsdb_att_get_uuid                            0x0100b38d
#define _attsdb_att_set_permission                      0x0100b401
#define _attsdb_att_get_permission                      0x0100b419
#define _attsdb_att_update_perm                         0x0100b433
#define _attsdb_svc_set_permission                      0x0100b453
#define _attsdb_svc_get_permission                      0x0100b46b
#define _atts_write_no_resp                             0x0100c927
#define _atts_write_rsp_send                            0x0100cccb
#define _atts_svc_create_db                             0x0100ccf9
#define _gap_get_rec_idx                                0x0100d1f5
#define _gap_get_lk_sec_status                          0x0100d245
#define _gap_get_enc_keysize                            0x0100d24f
#define _gap_get_conhdl                                 0x0100d259
#define _gap_get_security                               0x0100d263
#define _gap_send_discon_cmp_evt                        0x0100d269
#define _gap_send_connect_req                           0x0100d289
#define _ble_prevent_sleep_set                          0x010151d3
#define _ble_wakeup                                     0x010151ed
#define _ble_wakeup_end                                 0x0101521f
#define _ble_disable                                    0x01015241
#define _ble_enable                                     0x01015251
#define _ble_reset                                      0x010152fd
#define _ble_version                                    0x01015341
#define _ble_schedule                                   0x01015383
#define _ble_sleep                                      0x010153bf
#define _ble_isr                                        0x01015431
#define _reg_sleep_cb                                   0x010154b7
#define _qn_config_init                                 0x010154c1
#define _config_work_mode                               0x01015513
#define _save_cal_setting                               0x010155ad
#define _save_ble_reg                                   0x010155e9
#define _save_sys_regs                                  0x01015633
#define _save_ble_setting                               0x01015695
#define _restore_ble_setting                            0x010156b3
#define _enable_ble_sleep                               0x010156d9
#define _qn_plf_init                                    0x0101575d
#define _qn_ble_init                                    0x01015781
#define _prf_init_reg                                   0x01015815
#define _prf_disp_disconnect_reg                        0x0101581b
#define _rtc_capture_env                                0x1000cdb8
#define _timer0_env                                     0x1000cdc8
#define _timer1_env                                     0x1000cdd0
#define _timer2_env                                     0x1000cdd8
#define _timer3_env                                     0x1000cde0
#define _ke_evt_hdlr                                    0x1000cdfc
#define _rtc_env                                        0x1000d054
#define _nvds_env                                       0x1000d0c8
#define _ke_env                                         0x1000d214
#define _sleep_api                                      0x1000dc08
#define _gap_env                                        0x1000dd70
#define _ble_env                                        0x1000f2fc
#define _ble_rf                                         0x1000f308
#define _config_api                                     0x1000f340

#endif

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif

/// @} QN_ROM_DRIVER_ADDR
#endif