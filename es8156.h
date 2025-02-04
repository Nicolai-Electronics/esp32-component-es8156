// ES8156 stereo audio DAC component
// SPDX-FileCopyrightText: 2025 Nicolai Electronics
// SPDX-License-Identifier: MIT

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

// Defines

// Types
typedef struct {
    i2c_master_bus_handle_t i2c_bus;          /// Handle of the I2C bus of the coprocessor
    uint16_t i2c_address;                     /// I2C address of the coprocessor (7-bit)
    SemaphoreHandle_t concurrency_semaphore;  /// Semaphore for making I2C bus operation thread safe
} es8156_config_t;

typedef struct es8156 es8156_t;
typedef es8156_t* es8156_handle_t;

typedef enum es8156_csm_state {
    ES8156_CSM_STATE_NOT_FORCED = 0,
    ES8156_CSM_STATE_S0,
    ES8156_CSM_STATE_S1,
    ES8156_CSM_STATE_S2,
    ES8156_CSM_STATE_S3,
    ES8156_CSM_STATE_S6,
} es8156_csm_state_t;

typedef enum es8156_sel_state {
    ES8156_SEL_NA = 0,
    ES8156_SEL_DEFAULT = 1,
    ES8156_SEL_S2 = 2,
    ES8156_SEL_S3 = 3,
} es8156_sel_state_t;

// Functions

esp_err_t es8156_initialize(const es8156_config_t* configuration, es8156_handle_t* out_handle);

esp_err_t es8156_write_reset_control(es8156_handle_t handle, bool csm_on, bool seq_dis, bool rst_dig, bool rst_dac_dig,
                                     bool rst_mstgen, bool rst_regs);
esp_err_t es8156_read_reset_control(es8156_handle_t handle, bool* out_csm_on, bool* out_seq_dis, bool* out_rst_dig,
                                    bool* out_rst_dac_dig, bool* out_rst_mstgen, bool* out_rst_regs);

esp_err_t es8156_write_main_clock_control(es8156_handle_t handle, uint8_t clk_dac_div, bool osr128_sel,
                                          uint8_t multp_factor);

esp_err_t es8156_write_mode_config(es8156_handle_t handle, bool ms_mode, bool speed_mode, bool soft_mode_sel,
                                   bool eq_high_mode, bool sclk_inv_mode, bool sclklrck_tri, bool isclklrck_sel,
                                   bool mclk_sel);

esp_err_t es8156_write_lrck_divider(es8156_handle_t handle, uint16_t m_lrck_div);

esp_err_t es8156_write_master_clock_control(es8156_handle_t handle, uint8_t m_sclk_div, bool m_sclk_mode);

esp_err_t es8156_write_nfs_config(es8156_handle_t handle, uint8_t lrck_rate_mode);

esp_err_t es8156_write_misc_control_1(es8156_handle_t handle, bool lrck_extend, uint8_t clock_doubler_pw_sel,
                                      uint8_t clk_dac_div0, bool mclk_inv);

esp_err_t es8156_write_clock_off(es8156_handle_t handle, bool mclk_on, bool dac_mclk_on, bool ana_clk_on,
                                 bool ext_sclklrck_on, bool master_clk_on, bool p2s_clk_on);

esp_err_t es8156_write_misc_control_2(es8156_handle_t handle, bool pull_up, bool dll_on, bool csm_cnt_use_master,
                                      bool internal_master_clk_src);

esp_err_t es8156_write_time_control_1(es8156_handle_t handle, uint8_t v_t1);

esp_err_t es8156_write_time_control_2(es8156_handle_t handle, uint8_t v_t2);

esp_err_t es8156_read_chip_status(es8156_handle_t handle, es8156_csm_state_t* out_force_csm,
                                  es8156_csm_state_t* out_csm_state);

esp_err_t es8156_write_chip_status(es8156_handle_t handle, es8156_csm_state_t force_csm_state);

// Page 0 register 0x0D: p2s control
esp_err_t es8156_write_p2s_control(es8156_handle_t handle, bool p2s_sdout_tri, bool p2s_sdout_sel, bool p2s_sdout_muteb,
                                   bool p2s_nfs_flagoff, uint8_t lrck_1stcnt);

// Page 0 register 0x10: DAC counter parameter
esp_err_t es8156_write_dac_counter_parameter(es8156_handle_t handle, uint8_t dac_ns);

// Page 0 register 0x11: SDP interface config 1
esp_err_t es8156_write_sdp_interface_config_1(es8156_handle_t handle, uint8_t sp_protocal, bool sp_lrp, bool sp_mute,
                                              uint8_t sp_wl);

// Page 0 register 0x12: automute control
esp_err_t es8156_write_automute_control(es8156_handle_t handle, uint8_t automute_size, uint8_t automute_ng);

// Page 0 register 0x13: mute control
esp_err_t es8156_write_mute_control(es8156_handle_t handle, bool am_ena, bool lch_dsm_smute, bool rch_dsm_smute,
                                    bool am_dsmmute_ena, bool am_aclkoff_ena, bool am_attenu6_ena, bool intout_clipen);

// Page 0 register 0x14: volume control
esp_err_t es8156_write_volume_control(es8156_handle_t handle, uint8_t dac_volume_db);
esp_err_t es8156_read_volume_control(es8156_handle_t handle, uint8_t* out_dac_volume_db);

// Page 0 register 0x15: ALC config 1
esp_err_t es8156_write_alc_config_1(es8156_handle_t handle, bool dac_alc_en, uint8_t alc_mute_gain);

// Page 0 register 0x16: ALC config 2
esp_err_t es8156_write_alc_config_2(es8156_handle_t handle, uint8_t alc_win_size, uint8_t alc_ramp_rate);

// Page 0 register 0x17: ALC level
esp_err_t es8156_write_alc_level(es8156_handle_t handle, uint8_t alc_minlevel, uint8_t alc_maxlevel);

// Page 0 register 0x18: misc control 3
esp_err_t es8156_write_misc_control_3(es8156_handle_t handle, bool dac_ram_clr, bool dsm_ditheron, bool rch_inv,
                                      bool lch_inv, uint8_t chn_cross, bool p2s_dpath_sel, bool p2s_data_bitnum);

// Page 0 register 0x19: EQ control 1
esp_err_t es8156_write_eq_control_1(es8156_handle_t handle, bool eq_on, bool eq_cfg_wr, bool eq_cfg_rd, bool eq_rst,
                                    uint8_t eq_band_num);

// Page 0 register 0x1A: EQ config 2
esp_err_t es8156_write_eq_config_2(es8156_handle_t handle, uint8_t eq_1stcnt, bool eq_1stcnt_vld);

// Page 0 register 0x20: analog system
esp_err_t es8156_write_analog_system_1(es8156_handle_t handle, uint8_t s6_sel, uint8_t s2_sel, uint8_t s3_sel);

// Page 0 register 0x21: analog system: note vsel is bits 2-4, not 0-4
esp_err_t es8156_write_analog_system_2(es8156_handle_t handle, uint8_t vsel, bool vref_rmpdn1, bool vref_rmpdn2);

// Page 0 register 0x22: analog system
esp_err_t es8156_write_analog_system_3(es8156_handle_t handle, bool out_mute, bool swrmpsel, bool hpsw);

// Page 0 register 0x23: analog system
esp_err_t es8156_write_analog_system_4(es8156_handle_t handle, bool hpcom_ref1, bool hpcom_ref2, bool vroi,
                                       bool dac_ibias_sw, uint8_t vmidlvl, uint8_t ibias_sw);

// Page 0 register 0x24: analog system
esp_err_t es8156_write_analog_system_5(es8156_handle_t handle, bool lpvrefbuf, bool lphpcom, bool lpdacvrp, bool lpdac);

// Page 0 register 0x25: analog system
esp_err_t es8156_write_analog_system_6(es8156_handle_t handle, bool pdn_dac, bool pdn_vrefbuf, bool pdn_dacvrefgen,
                                       bool enhpcom, uint8_t vmidsel, bool enrefr, bool pdn_ana);

// Page 0 register 0x40-0x5E: EQ data ram clear
esp_err_t es8156_eq_data_ram_clear(es8156_handle_t handle);

// Page 0 register 0xFC: page select
esp_err_t es8156_write_page_select(es8156_handle_t handle, bool select_second_page);

// Page 0 register 0xFD & 0xFE: chip identifier
esp_err_t es8156_read_chip_id(es8156_handle_t handle, uint16_t* out_id);

// Page 0 register 0xFF: chip version
esp_err_t es8156_read_chip_version(es8156_handle_t handle, uint8_t* out_version);

// Page 1 register 0x00-0xD1: EQ coefficient
esp_err_t es8156_write_eq_coefficient(es8156_handle_t handle, uint8_t address, uint8_t value);

esp_err_t es8156_configure(es8156_handle_t handle);