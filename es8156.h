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
    ES8156_CSM_STATE_INVALID = 0,
    ES8156_CSM_STATE_NOT_FORCED,
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
esp_err_t es8156_read_main_clock_control(es8156_handle_t handle, uint8_t* out_clk_dac_div, bool* out_osr128_sel,
                                         uint8_t* out_multp_factor);

esp_err_t es8156_write_mode_config(es8156_handle_t handle, bool ms_mode, bool speed_mode, bool soft_mode_sel,
                                   bool eq_high_mode, bool sclk_inv_mode, bool sclklrck_tri, bool isclklrck_sel,
                                   bool mclk_sel);
esp_err_t es8156_read_mode_config(es8156_handle_t handle, bool* out_ms_mode, bool* out_speed_mode,
                                  bool* out_soft_mode_sel, bool* out_eq_high_mode, bool* out_sclk_inv_mode,
                                  bool* out_sclklrck_tri, bool* out_isclklrck_sel, bool* out_mclk_sel);

esp_err_t es8156_write_lrck_divider(es8156_handle_t handle, uint16_t m_lrck_div);
esp_err_t es8156_read_lrck_divider(es8156_handle_t handle, uint16_t* out_m_lrck_div);

esp_err_t es8156_write_master_clock_control(es8156_handle_t handle, uint8_t m_sclk_div, bool m_sclk_mode);
esp_err_t es8156_read_master_clock_control(es8156_handle_t handle, uint8_t* out_m_sclk_div, bool* out_m_sclk_mode);

esp_err_t es8156_write_nfs_config(es8156_handle_t handle, uint8_t lrck_rate_mode);
esp_err_t es8156_read_nfs_config(es8156_handle_t handle, uint8_t* out_lrck_rate_mode);

esp_err_t es8156_write_misc_control_1(es8156_handle_t handle, bool lrck_extend, uint8_t clock_doubler_pw_sel,
                                      uint8_t clk_dac_div0, bool mclk_inv);
esp_err_t es8156_read_misc_control_1(es8156_handle_t handle, bool* out_lrck_extend, uint8_t* out_clock_doubler_pw_sel,
                                     uint8_t* out_clk_dac_div0, bool* out_mclk_inv);

esp_err_t es8156_write_clock_off(es8156_handle_t handle, bool mclk_on, bool dac_mclk_on, bool ana_clk_on,
                                 bool ext_sclklrck_on, bool master_clk_on, bool p2s_clk_on);
esp_err_t es8156_read_clock_off(es8156_handle_t handle, bool* out_mclk_on, bool* out_dac_mclk_on, bool* out_ana_clk_on,
                                bool* out_ext_sclklrck_on, bool* out_master_clk_on, bool* out_p2s_clk_on);

esp_err_t es8156_write_misc_control_2(es8156_handle_t handle, bool pull_up, bool dll_on, bool csm_cnt_use_master,
                                      bool internal_master_clk_src);
esp_err_t es8156_read_misc_control_2(es8156_handle_t handle, bool* out_pull_up, bool* out_dll_on,
                                     bool* out_csm_cnt_use_master, bool* out_internal_master_clk_src);

esp_err_t es8156_write_time_control_1(es8156_handle_t handle, uint8_t v_t1);
esp_err_t es8156_read_time_control_1(es8156_handle_t handle, uint8_t* out_v_t1);

esp_err_t es8156_write_time_control_2(es8156_handle_t handle, uint8_t v_t2);
esp_err_t es8156_read_time_control_2(es8156_handle_t handle, uint8_t* out_v_t2);

esp_err_t es8156_write_chip_status(es8156_handle_t handle, es8156_csm_state_t force_csm_state);

esp_err_t es8156_read_chip_status(es8156_handle_t handle, es8156_csm_state_t* out_force_csm,
                                  es8156_csm_state_t* out_csm_state);

esp_err_t es8156_write_p2s_control(es8156_handle_t handle, bool p2s_sdout_tri, bool p2s_sdout_sel, bool p2s_sdout_muteb,
                                   bool p2s_nfs_flagoff, uint8_t lrck_1stcnt);
esp_err_t es8156_read_p2s_control(es8156_handle_t handle, bool* out_p2s_sdout_tri, bool* out_p2s_sdout_sel,
                                  bool* out_p2s_sdout_muteb, bool* out_p2s_nfs_flagoff, uint8_t* out_lrck_1stcnt);

esp_err_t es8156_write_dac_counter_parameter(es8156_handle_t handle, uint8_t dac_ns);
esp_err_t es8156_read_dac_counter_parameter(es8156_handle_t handle, uint8_t* out_dac_ns);

esp_err_t es8156_write_sdp_interface_config_1(es8156_handle_t handle, uint8_t sp_protocal, bool sp_lrp, bool sp_mute,
                                              uint8_t sp_wl);
esp_err_t es8156_read_sdp_interface_config_1(es8156_handle_t handle, uint8_t* out_sp_protocal, bool* out_sp_lrp,
                                             bool* out_sp_mute, uint8_t* out_sp_wl);

esp_err_t es8156_write_automute_control(es8156_handle_t handle, uint8_t automute_size, uint8_t automute_ng);
esp_err_t es8156_read_automute_control(es8156_handle_t handle, uint8_t* out_automute_size, uint8_t* out_automute_ng);

esp_err_t es8156_write_mute_control(es8156_handle_t handle, bool am_ena, bool lch_dsm_smute, bool rch_dsm_smute,
                                    bool am_dsmmute_ena, bool am_aclkoff_ena, bool am_attenu6_ena, bool intout_clipen);
esp_err_t es8156_read_mute_control(es8156_handle_t handle, bool* out_am_ena, bool* out_lch_dsm_smute,
                                   bool* out_rch_dsm_smute, bool* out_am_dsmmute_ena, bool* out_am_aclkoff_ena,
                                   bool* out_am_attenu6_ena, bool* out_intout_clipen);

esp_err_t es8156_write_volume_control(es8156_handle_t handle, uint8_t dac_volume_db);
esp_err_t es8156_read_volume_control(es8156_handle_t handle, uint8_t* out_dac_volume_db);

esp_err_t es8156_write_alc_config_1(es8156_handle_t handle, bool dac_alc_en, uint8_t alc_mute_gain);
esp_err_t es8156_read_alc_config_1(es8156_handle_t handle, bool* out_dac_alc_en, uint8_t* out_alc_mute_gain);

esp_err_t es8156_write_alc_config_2(es8156_handle_t handle, uint8_t alc_win_size, uint8_t alc_ramp_rate);
esp_err_t es8156_read_alc_config_2(es8156_handle_t handle, uint8_t* out_alc_win_size, uint8_t* out_alc_ramp_rate);

esp_err_t es8156_write_alc_level(es8156_handle_t handle, uint8_t alc_minlevel, uint8_t alc_maxlevel);
esp_err_t es8156_read_alc_level(es8156_handle_t handle, uint8_t* out_alc_minlevel, uint8_t* out_alc_maxlevel);

esp_err_t es8156_write_misc_control_3(es8156_handle_t handle, bool dac_ram_clr, bool dsm_ditheron, bool rch_inv,
                                      bool lch_inv, uint8_t chn_cross, bool p2s_dpath_sel, bool p2s_data_bitnum);
esp_err_t es8156_read_misc_control_3(es8156_handle_t handle, bool* out_dac_ram_clr, bool* out_dsm_ditheron,
                                     bool* out_rch_inv, bool* out_lch_inv, uint8_t* out_chn_cross,
                                     bool* out_p2s_dpath_sel, bool* out_p2s_data_bitnum);

esp_err_t es8156_write_eq_control_1(es8156_handle_t handle, bool eq_on, bool eq_cfg_wr, bool eq_cfg_rd, bool eq_rst,
                                    uint8_t eq_band_num);
esp_err_t es8156_read_eq_control_1(es8156_handle_t handle, bool* out_eq_on, bool* out_eq_cfg_wr, bool* out_eq_cfg_rd,
                                   bool* out_eq_rst, uint8_t* out_eq_band_num);

esp_err_t es8156_write_eq_config_2(es8156_handle_t handle, uint8_t eq_1stcnt, bool eq_1stcnt_vld);
esp_err_t es8156_read_eq_config_2(es8156_handle_t handle, uint8_t* out_eq_1stcnt, bool* out_eq_1stcnt_vld);

esp_err_t es8156_write_analog_system_1(es8156_handle_t handle, uint8_t s6_sel, uint8_t s2_sel, uint8_t s3_sel);
esp_err_t es8156_read_analog_system_1(es8156_handle_t handle, uint8_t* out_s6_sel, uint8_t* out_s2_sel,
                                      uint8_t* out_s3_sel);

esp_err_t es8156_write_analog_system_2(es8156_handle_t handle, uint8_t vsel, bool vref_rmpdn1, bool vref_rmpdn2);
esp_err_t es8156_read_analog_system_2(es8156_handle_t handle, uint8_t* out_vsel, bool* out_vref_rmpdn1,
                                      bool* out_vref_rmpdn2);

esp_err_t es8156_write_analog_system_3(es8156_handle_t handle, bool out_mute, bool swrmpsel, bool hpsw);
esp_err_t es8156_read_analog_system_3(es8156_handle_t handle, bool* out_out_mute, bool* out_swrmpsel, bool* out_hpsw);

esp_err_t es8156_write_analog_system_4(es8156_handle_t handle, bool hpcom_ref1, bool hpcom_ref2, bool vroi,
                                       bool dac_ibias_sw, uint8_t vmidlvl, uint8_t ibias_sw);
esp_err_t es8156_read_analog_system_4(es8156_handle_t handle, bool* out_hpcom_ref1, bool* out_hpcom_ref2,
                                      bool* out_vroi, bool* out_dac_ibias_sw, uint8_t* out_vmidlvl,
                                      uint8_t* out_ibias_sw);

esp_err_t es8156_write_analog_system_5(es8156_handle_t handle, bool lpvrefbuf, bool lphpcom, bool lpdacvrp, bool lpdac);
esp_err_t es8156_read_analog_system_5(es8156_handle_t handle, bool* out_lpvrefbuf, bool* out_lphpcom,
                                      bool* out_lpdacvrp, bool* out_lpdac);

esp_err_t es8156_write_analog_system_6(es8156_handle_t handle, bool pdn_dac, bool pdn_vrefbuf, bool pdn_dacvrefgen,
                                       bool enhpcom, uint8_t vmidsel, bool enrefr, bool pdn_ana);
esp_err_t es8156_read_analog_system_6(es8156_handle_t handle, bool* out_pdn_dac, bool* out_pdn_vrefbuf,
                                      bool* out_pdn_dacvrefgen, bool* out_enhpcom, uint8_t* out_vmidsel,
                                      bool* out_enrefr, bool* out_pdn_ana);

esp_err_t es8156_eq_data_ram_clear(es8156_handle_t handle);

esp_err_t es8156_read_chip_id(es8156_handle_t handle, uint16_t* out_id);

esp_err_t es8156_read_chip_version(es8156_handle_t handle, uint8_t* out_version);

esp_err_t es8156_write_eq_coefficient(es8156_handle_t handle, uint8_t address, uint8_t value);
esp_err_t es8156_read_eq_coefficient(es8156_handle_t handle, uint8_t address, uint8_t* out_value);

esp_err_t es8156_configure(es8156_handle_t handle);
esp_err_t es8156_powerdown(es8156_handle_t handle);
esp_err_t es8156_standby_nopop(es8156_handle_t handle);
esp_err_t es8156_reset(es8156_handle_t handle);
