// SPDX-FileCopyrightText: 2025 Nicolai Electronics
// SPDX-License-Identifier: MIT

#include "es8156.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "es8156_regs.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/queue.h"

// Defines
#define ES8156_CHIPID  0x81551
#define ES8156_TIMEOUT 100  // ms

// Registers
#define TANMATSU_COPROCESSOR_I2C_REG_FW_VERSION_0 0  // LSB

typedef enum {
    ES8156_PAGE_UNKNOWN = 0,
    ES8156_PAGE_0 = 1,
    ES8156_PAGE_1 = 2,
} es8156_page_t;

typedef struct es8156 {
    i2c_master_dev_handle_t dev_handle;  /// I2C device handle
    es8156_config_t configuration;       /// Copy of the configuration struct provided during initialization
    es8156_page_t current_page;          /// Currently selected register page
} es8156_t;

static char const TAG[] = "ES8156";

// Wrapping functions for making ESP-IDF I2C driver thread-safe

static void claim_i2c_bus(es8156_handle_t handle) {
    // Claim I2C bus
    if (handle->configuration.concurrency_semaphore != NULL) {
        xSemaphoreTake(handle->configuration.concurrency_semaphore, portMAX_DELAY);
    } else {
        ESP_LOGW(TAG, "No concurrency semaphore");
    }
}

static void release_i2c_bus(es8156_handle_t handle) {
    // Release I2C bus
    if (handle->configuration.concurrency_semaphore != NULL) {
        xSemaphoreGive(handle->configuration.concurrency_semaphore);
    }
}

static esp_err_t ts_i2c_master_transmit_receive(es8156_handle_t handle, const uint8_t* write_buffer, size_t write_size,
                                                uint8_t* read_buffer, size_t read_size, int xfer_timeout_ms) {
    claim_i2c_bus(handle);
    esp_err_t res = i2c_master_transmit_receive(handle->dev_handle, write_buffer, write_size, read_buffer, read_size,
                                                xfer_timeout_ms);
    release_i2c_bus(handle);
    return res;
}

static esp_err_t ts_i2c_master_transmit(es8156_handle_t handle, const uint8_t* write_buffer, size_t write_size,
                                        int xfer_timeout_ms) {
    claim_i2c_bus(handle);
    printf("Transmit: ");
    for (uint8_t pos = 0; pos < write_size; pos++) {
        printf("%02X ", write_buffer[pos]);
    }
    printf("\r\n");
    esp_err_t res = i2c_master_transmit(handle->dev_handle, write_buffer, write_size, xfer_timeout_ms);
    release_i2c_bus(handle);
    return res;
}

esp_err_t es8156_initialize(const es8156_config_t* configuration, es8156_handle_t* out_handle) {
    ESP_RETURN_ON_FALSE(configuration, ESP_ERR_INVALID_ARG, TAG, "invalid argument: configuration");
    ESP_RETURN_ON_FALSE(out_handle, ESP_ERR_INVALID_ARG, TAG, "invalid argument: handle");

    ESP_RETURN_ON_FALSE(configuration->i2c_bus, ESP_ERR_INVALID_ARG, TAG, "invalid argument: i2c bus");
    ESP_RETURN_ON_FALSE(configuration->i2c_address, ESP_ERR_INVALID_ARG, TAG, "invalid argument: i2c address");

    ESP_RETURN_ON_ERROR(i2c_master_probe(configuration->i2c_bus, configuration->i2c_address, 100), TAG,
                        "ES8156 not detected on I2C bus");

    es8156_t* handle = heap_caps_calloc(1, sizeof(es8156_t), MALLOC_CAP_DEFAULT);
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_NO_MEM, TAG, "no memory for ES8156 struct");

    memcpy(&handle->configuration, configuration, sizeof(es8156_config_t));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = configuration->i2c_address,
        .scl_speed_hz = 400000,
    };

    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(configuration->i2c_bus, &dev_cfg, &handle->dev_handle), TAG,
                        "Failed to add ES8156 device to I2C bus");

    *out_handle = handle;
    return ESP_OK;
}

// Register read and write functions

esp_err_t es8156_write_register(es8156_handle_t handle, uint8_t reg, uint8_t value) {
    uint8_t buffer[2];
    buffer[0] = reg;
    buffer[1] = value;
    return ts_i2c_master_transmit(handle, buffer, sizeof(buffer), ES8156_TIMEOUT);
};

esp_err_t es8156_read_register(es8156_handle_t handle, uint8_t reg, uint8_t* out_value) {
    return ts_i2c_master_transmit_receive(handle, &reg, 1, out_value, 1, ES8156_TIMEOUT);
};

esp_err_t es8156_read_registers(es8156_handle_t handle, uint8_t reg, uint8_t* out_value, uint8_t length) {
    return ts_i2c_master_transmit_receive(handle, &reg, 1, out_value, length, ES8156_TIMEOUT);
};

// Page select

esp_err_t es8156_write_page_select(es8156_handle_t handle, bool select_second_page) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    value |= (select_second_page << ES8156_REGFC_PAGE_SEL_OFFSET) & ES8156_REGFC_PAGE_SEL_MASK;
    return es8156_write_register(handle, ES8156_REG_PAGE_SELECT, value);
}

esp_err_t es8156_read_page_select(es8156_handle_t handle, bool* out_select_second_page) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    esp_err_t res = es8156_read_register(handle, ES8156_REG_PAGE_SELECT, &value);
    if (res != ESP_OK) return res;
    if (out_select_second_page != NULL) {
        *out_select_second_page = (value & ES8156_REGFC_PAGE_SEL_MASK) >> ES8156_REGFC_PAGE_SEL_OFFSET;
    }
    return ESP_OK;
}

esp_err_t es8156_select_page(es8156_handle_t handle, es8156_page_t page) {
    if (handle->current_page == ES8156_PAGE_UNKNOWN) {
        bool page1 = false;
        esp_err_t res = es8156_read_page_select(handle, &page1);
        if (res != ESP_OK) return res;
        if (page1) {
            handle->current_page = ES8156_PAGE_1;
        } else {
            handle->current_page = ES8156_PAGE_0;
        }
    }
    if (handle->current_page != page) {
        bool page1 = (page == ES8156_PAGE_1);
        esp_err_t res = es8156_write_page_select(handle, page1);
        if (res != ESP_OK) return res;
        handle->current_page = page;
    }
    return ESP_OK;
}

esp_err_t es8156_write_register_page(es8156_handle_t handle, es8156_page_t page, uint8_t reg, uint8_t value) {
    esp_err_t res = es8156_select_page(handle, page);
    if (res != ESP_OK) return res;
    return es8156_write_register(handle, reg, value);
};

esp_err_t es8156_read_register_page(es8156_handle_t handle, es8156_page_t page, uint8_t reg, uint8_t* out_value) {
    esp_err_t res = es8156_select_page(handle, page);
    if (res != ESP_OK) return res;
    return es8156_read_register(handle, reg, out_value);
};

esp_err_t es8156_read_registers_page(es8156_handle_t handle, es8156_page_t page, uint8_t reg, uint8_t* out_value,
                                     uint8_t length) {
    esp_err_t res = es8156_select_page(handle, page);
    if (res != ESP_OK) return res;
    return es8156_read_registers(handle, reg, out_value, length);
};

// Direct register operations

esp_err_t es8156_write_reset_control(es8156_handle_t handle, bool csm_on, bool seq_dis, bool rst_dig, bool rst_dac_dig,
                                     bool rst_mstgen, bool rst_regs) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    value |= (csm_on << ES8156_REG00_CSM_ON_OFFSET) & ES8156_REG00_CSM_ON_MASK;
    value |= (seq_dis << ES8156_REG00_SEQ_DIS_OFFSET) & ES8156_REG00_SEQ_DIS_MASK;
    value |= (rst_dig << ES8156_REG00_RST_DIG_OFFSET) & ES8156_REG00_RST_DIG_MASK;
    value |= (rst_dac_dig << ES8156_REG00_RST_DAC_DIG_OFFSET) & ES8156_REG00_RST_DAC_DIG_MASK;
    value |= (rst_mstgen << ES8156_REG00_RST_MSTGEN_OFFSET) & ES8156_REG00_RST_MSTGEN_MASK;
    value |= (rst_regs << ES8156_REG00_RST_REGS_OFFSET) & ES8156_REG00_RST_REGS_MASK;
    return es8156_write_register_page(handle, ES8156_PAGE_0, ES8156_REG_RESET_CONTROL, value);
}

esp_err_t es8156_read_reset_control(es8156_handle_t handle, bool* out_csm_on, bool* out_seq_dis, bool* out_rst_dig,
                                    bool* out_rst_dac_dig, bool* out_rst_mstgen, bool* out_rst_regs) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    esp_err_t res = es8156_read_register_page(handle, ES8156_PAGE_0, ES8156_REG_RESET_CONTROL, &value);
    if (res != ESP_OK) return res;
    if (out_csm_on != NULL) {
        *out_csm_on = (value & ES8156_REG00_CSM_ON_MASK) >> ES8156_REG00_CSM_ON_OFFSET;
    }
    if (out_seq_dis != NULL) {
        *out_seq_dis = (value & ES8156_REG00_SEQ_DIS_MASK) >> ES8156_REG00_SEQ_DIS_OFFSET;
    }
    if (out_rst_dig != NULL) {
        *out_rst_dig = (value & ES8156_REG00_RST_DIG_MASK) >> ES8156_REG00_RST_DIG_OFFSET;
    }
    if (out_rst_dac_dig != NULL) {
        *out_rst_dac_dig = (value & ES8156_REG00_RST_DAC_DIG_MASK) >> ES8156_REG00_RST_DAC_DIG_OFFSET;
    }
    if (out_rst_mstgen != NULL) {
        *out_rst_mstgen = (value & ES8156_REG00_RST_MSTGEN_MASK) >> ES8156_REG00_RST_MSTGEN_OFFSET;
    }
    if (out_rst_regs != NULL) {
        *out_rst_regs = (value & ES8156_REG00_RST_REGS_MASK) >> ES8156_REG00_RST_REGS_OFFSET;
    }
    return ESP_OK;
}

esp_err_t es8156_write_main_clock_control(es8156_handle_t handle, uint8_t clk_dac_div, bool osr128_sel,
                                          uint8_t multp_factor) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    value |= (clk_dac_div << ES8156_REG01_CLK_DAC_DIV_OFFSET) & ES8156_REG01_CLK_DAC_DIV_MASK;
    value |= (osr128_sel << ES8156_REG01_OSR128_SEL_OFFSET) & ES8156_REG01_OSR128_SEL_MASK;
    value |= (multp_factor << ES8156_REG01_MULTP_FACTOR_OFFSET) & ES8156_REG01_MULTP_FACTOR_MASK;
    return es8156_write_register_page(handle, ES8156_PAGE_0, ES8156_REG_MAIN_CLOCK_CONTROL, value);
}

esp_err_t es8156_read_main_clock_control(es8156_handle_t handle, uint8_t* out_clk_dac_div, bool* out_osr128_sel,
                                         uint8_t* out_multp_factor) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    esp_err_t res = es8156_read_register_page(handle, ES8156_PAGE_0, ES8156_REG_MAIN_CLOCK_CONTROL, &value);
    if (res != ESP_OK) return res;
    if (out_clk_dac_div != NULL) {
        *out_clk_dac_div = (value & ES8156_REG01_CLK_DAC_DIV_MASK) >> ES8156_REG01_CLK_DAC_DIV_OFFSET;
    }
    if (out_osr128_sel != NULL) {
        *out_osr128_sel = (value & ES8156_REG01_OSR128_SEL_MASK) >> ES8156_REG01_OSR128_SEL_OFFSET;
    }
    if (out_multp_factor != NULL) {
        *out_multp_factor = (value & ES8156_REG01_MULTP_FACTOR_MASK) >> ES8156_REG01_MULTP_FACTOR_OFFSET;
    }
    return ESP_OK;
}

esp_err_t es8156_write_mode_config(es8156_handle_t handle, bool ms_mode, bool speed_mode, bool soft_mode_sel,
                                   bool eq_high_mode, bool sclk_inv_mode, bool sclklrck_tri, bool isclklrck_sel,
                                   bool mclk_sel) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    value |= (ms_mode << ES8156_REG02_MS_MODE_OFFSET) & ES8156_REG02_MS_MODE_MASK;
    value |= (speed_mode << ES8156_REG02_SPEED_MODE_OFFSET) & ES8156_REG02_SPEED_MODE_MASK;
    value |= (soft_mode_sel << ES8156_REG02_SOFT_MODE_SEL_OFFSET) & ES8156_REG02_SOFT_MODE_SEL_MASK;
    value |= (eq_high_mode << ES8156_REG02_EQ_HIGH_MODE_OFFSET) & ES8156_REG02_EQ_HIGH_MODE_MASK;
    value |= (sclk_inv_mode << ES8156_REG02_SCLK_INV_MODE_OFFSET) & ES8156_REG02_SCLK_INV_MODE_MASK;
    value |= (sclklrck_tri << ES8156_REG02_SCLKLRCK_TRI_OFFSET) & ES8156_REG02_SCLKLRCK_TRI_MASK;
    value |= (isclklrck_sel << ES8156_REG02_ISCLKLRCK_SEL_OFFSET) & ES8156_REG02_ISCLKLRCK_SEL_MASK;
    value |= (mclk_sel << ES8156_REG02_MCLK_SEL_OFFSET) & ES8156_REG02_MCLK_SEL_MASK;
    return es8156_write_register_page(handle, ES8156_PAGE_0, ES8156_REG_MODE_CONFIG_1, value);
}

esp_err_t es8156_read_mode_config(es8156_handle_t handle, bool* out_ms_mode, bool* out_speed_mode,
                                  bool* out_soft_mode_sel, bool* out_eq_high_mode, bool* out_sclk_inv_mode,
                                  bool* out_sclklrck_tri, bool* out_isclklrck_sel, bool* out_mclk_sel) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    esp_err_t res = es8156_read_register_page(handle, ES8156_PAGE_0, ES8156_REG_MODE_CONFIG_1, &value);
    if (res != ESP_OK) return res;
    if (out_ms_mode != NULL) {
        *out_ms_mode = (value & ES8156_REG02_MS_MODE_MASK) >> ES8156_REG02_MS_MODE_OFFSET;
    }
    if (out_speed_mode != NULL) {
        *out_speed_mode = (value & ES8156_REG02_SPEED_MODE_MASK) >> ES8156_REG02_SPEED_MODE_OFFSET;
    }
    if (out_soft_mode_sel != NULL) {
        *out_soft_mode_sel = (value & ES8156_REG02_SOFT_MODE_SEL_MASK) >> ES8156_REG02_SOFT_MODE_SEL_OFFSET;
    }
    if (out_eq_high_mode != NULL) {
        *out_eq_high_mode = (value & ES8156_REG02_EQ_HIGH_MODE_MASK) >> ES8156_REG02_EQ_HIGH_MODE_OFFSET;
    }
    if (out_sclk_inv_mode != NULL) {
        *out_sclk_inv_mode = (value & ES8156_REG02_SCLK_INV_MODE_MASK) >> ES8156_REG02_SCLK_INV_MODE_OFFSET;
    }
    if (out_sclklrck_tri != NULL) {
        *out_sclklrck_tri = (value & ES8156_REG02_SCLKLRCK_TRI_MASK) >> ES8156_REG02_SCLKLRCK_TRI_OFFSET;
    }
    if (out_isclklrck_sel != NULL) {
        *out_isclklrck_sel = (value & ES8156_REG02_ISCLKLRCK_SEL_MASK) >> ES8156_REG02_ISCLKLRCK_SEL_OFFSET;
    }
    if (out_mclk_sel != NULL) {
        *out_mclk_sel = (value & ES8156_REG02_MCLK_SEL_MASK) >> ES8156_REG02_MCLK_SEL_OFFSET;
    }
    return ESP_OK;
}

esp_err_t es8156_write_lrck_divider(es8156_handle_t handle, uint16_t m_lrck_div) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t hi = (m_lrck_div >> 8);
    uint8_t lo = (m_lrck_div & 0xFF);
    uint8_t buffer[3];
    buffer[0] = ES8156_REG_MASTER_LRCK_DIVIDER_1;
    buffer[1] = (hi << ES8156_REG03_M_LRCK_DIV_HIGH4_OFFSET) & ES8156_REG03_M_LRCK_DIV_HIGH4_MASK;
    buffer[2] = (lo << ES8156_REG04_M_LRCK_DIV_LOW8_OFFSET) & ES8156_REG04_M_LRCK_DIV_LOW8_MASK;
    return ts_i2c_master_transmit(handle, buffer, sizeof(buffer), ES8156_TIMEOUT);
}

esp_err_t es8156_read_lrck_divider(es8156_handle_t handle, uint16_t* out_m_lrck_div) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value[2] = {0};
    esp_err_t res =
        es8156_read_registers_page(handle, ES8156_PAGE_0, ES8156_REG_MASTER_LRCK_DIVIDER_1, value, sizeof(value));
    if (res != ESP_OK) return res;
    if (out_m_lrck_div != NULL) {
        uint8_t hi = (value[0] & ES8156_REG03_M_LRCK_DIV_HIGH4_MASK) >> ES8156_REG03_M_LRCK_DIV_HIGH4_OFFSET;
        uint8_t lo = (value[1] & ES8156_REG04_M_LRCK_DIV_LOW8_MASK) >> ES8156_REG04_M_LRCK_DIV_LOW8_OFFSET;
        *out_m_lrck_div = (hi << 8) | lo;
    }
    return ESP_OK;
}

esp_err_t es8156_write_master_clock_control(es8156_handle_t handle, uint8_t m_sclk_div, bool m_sclk_mode) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    value |= (m_sclk_div << ES8156_REG05_M_SCLK_DIV_OFFSET) & ES8156_REG05_M_SCLK_DIV_MASK;
    value |= (m_sclk_mode << ES8156_REG05_M_SCLK_MODE_OFFSET) & ES8156_REG05_M_SCLK_MODE_MASK;
    return es8156_write_register_page(handle, ES8156_PAGE_0, ES8156_REG_MASTER_CLOCK_CONTROL, value);
}

esp_err_t es8156_read_master_clock_control(es8156_handle_t handle, uint8_t* out_m_sclk_div, bool* out_m_sclk_mode) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    esp_err_t res = es8156_read_register_page(handle, ES8156_PAGE_0, ES8156_REG_MASTER_CLOCK_CONTROL, &value);
    if (res != ESP_OK) return res;
    if (out_m_sclk_div != NULL) {
        *out_m_sclk_div = (value & ES8156_REG05_M_SCLK_DIV_MASK) >> ES8156_REG05_M_SCLK_DIV_OFFSET;
    }
    if (out_m_sclk_mode != NULL) {
        *out_m_sclk_mode = (value & ES8156_REG05_M_SCLK_MODE_MASK) >> ES8156_REG05_M_SCLK_MODE_OFFSET;
    }
    return ESP_OK;
}

esp_err_t es8156_write_nfs_config(es8156_handle_t handle, uint8_t lrck_rate_mode) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    value |= (lrck_rate_mode << ES8156_REG06_LRCK_RATE_MODE_OFFSET) & ES8156_REG06_LRCK_RATE_MODE_MASK;
    return es8156_write_register_page(handle, ES8156_PAGE_0, ES8156_REG_NFS_CONFIG, value);
}

esp_err_t es8156_read_nfs_config(es8156_handle_t handle, uint8_t* out_lrck_rate_mode) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    esp_err_t res = es8156_read_register_page(handle, ES8156_PAGE_0, ES8156_REG_NFS_CONFIG, &value);
    if (res != ESP_OK) return res;
    if (out_lrck_rate_mode != NULL) {
        *out_lrck_rate_mode = (value & ES8156_REG06_LRCK_RATE_MODE_MASK) >> ES8156_REG06_LRCK_RATE_MODE_OFFSET;
    }
    return ESP_OK;
}

esp_err_t es8156_write_misc_control_1(es8156_handle_t handle, bool lrck_extend, uint8_t clock_doubler_pw_sel,
                                      uint8_t clk_dac_div0, bool mclk_inv) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    value |= (lrck_extend << ES8156_REG07_LRCK_EXTEND_OFFSET) & ES8156_REG07_LRCK_EXTEND_MASK;
    value |= (clock_doubler_pw_sel << ES8156_REG07_CLKDBL_PW_SEL_OFFSET) & ES8156_REG07_CLKDBL_PW_SEL_MASK;
    value |= (clk_dac_div0 << ES8156_REG07_CLK_DAC_DIV0_OFFSET) & ES8156_REG07_CLK_DAC_DIV0_MASK;
    value |= (mclk_inv << ES8156_REG07_MCLK_INV_OFFSET) & ES8156_REG07_MCLK_INV_MASK;
    return es8156_write_register_page(handle, ES8156_PAGE_0, ES8156_REG_MISC_CONTROL_1, value);
}

esp_err_t es8156_read_misc_control_1(es8156_handle_t handle, bool* out_lrck_extend, uint8_t* out_clock_doubler_pw_sel,
                                     uint8_t* out_clk_dac_div0, bool* out_mclk_inv) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    esp_err_t res = es8156_read_register_page(handle, ES8156_PAGE_0, ES8156_REG_MISC_CONTROL_1, &value);
    if (res != ESP_OK) return res;
    if (out_lrck_extend != NULL) {
        *out_lrck_extend = (value & ES8156_REG07_LRCK_EXTEND_MASK) >> ES8156_REG07_LRCK_EXTEND_OFFSET;
    }
    if (out_clock_doubler_pw_sel != NULL) {
        *out_clock_doubler_pw_sel = (value & ES8156_REG07_CLKDBL_PW_SEL_MASK) >> ES8156_REG07_CLKDBL_PW_SEL_OFFSET;
    }
    if (out_clk_dac_div0 != NULL) {
        *out_clk_dac_div0 = (value & ES8156_REG07_CLK_DAC_DIV0_MASK) >> ES8156_REG07_CLK_DAC_DIV0_OFFSET;
    }
    if (out_mclk_inv != NULL) {
        *out_mclk_inv = (value & ES8156_REG07_MCLK_INV_MASK) >> ES8156_REG07_MCLK_INV_OFFSET;
    }
    return ESP_OK;
}

esp_err_t es8156_write_clock_off(es8156_handle_t handle, bool mclk_on, bool dac_mclk_on, bool ana_clk_on,
                                 bool ext_sclklrck_on, bool master_clk_on, bool p2s_clk_on) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    value |= (mclk_on << ES8156_REG08_MCLK_ON_OFFSET) & ES8156_REG08_MCLK_ON_MASK;
    value |= (dac_mclk_on << ES8156_REG08_DAC_MCLK_ON_OFFSET) & ES8156_REG08_DAC_MCLK_ON_MASK;
    value |= (ana_clk_on << ES8156_REG08_ANA_CLK_ON_OFFSET) & ES8156_REG08_ANA_CLK_ON_MASK;
    value |= (ext_sclklrck_on << ES8156_REG08_EXT_SCLKLRCK_ON_OFFSET) & ES8156_REG08_EXT_SCLKLRCK_ON_MASK;
    value |= (master_clk_on << ES8156_REG08_MASTER_CLK_ON_OFFSET) & ES8156_REG08_MASTER_CLK_ON_MASK;
    value |= (p2s_clk_on << ES8156_REG08_P2S_CLK_ON_OFFSET) & ES8156_REG08_P2S_CLK_ON_MASK;
    return es8156_write_register_page(handle, ES8156_PAGE_0, ES8156_REG_CLOCK_OFF, value);
}

esp_err_t es8156_read_clock_off(es8156_handle_t handle, bool* out_mclk_on, bool* out_dac_mclk_on, bool* out_ana_clk_on,
                                bool* out_ext_sclklrck_on, bool* out_master_clk_on, bool* out_p2s_clk_on) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    esp_err_t res = es8156_read_register_page(handle, ES8156_PAGE_0, ES8156_REG_CLOCK_OFF, &value);
    if (res != ESP_OK) return res;
    if (out_mclk_on != NULL) {
        *out_mclk_on = (value & ES8156_REG08_MCLK_ON_MASK) >> ES8156_REG08_MCLK_ON_OFFSET;
    }
    if (out_dac_mclk_on != NULL) {
        *out_dac_mclk_on = (value & ES8156_REG08_DAC_MCLK_ON_MASK) >> ES8156_REG08_DAC_MCLK_ON_OFFSET;
    }
    if (out_ana_clk_on != NULL) {
        *out_ana_clk_on = (value & ES8156_REG08_ANA_CLK_ON_MASK) >> ES8156_REG08_ANA_CLK_ON_OFFSET;
    }
    if (out_ext_sclklrck_on != NULL) {
        *out_ext_sclklrck_on = (value & ES8156_REG08_EXT_SCLKLRCK_ON_MASK) >> ES8156_REG08_EXT_SCLKLRCK_ON_OFFSET;
    }
    if (out_master_clk_on != NULL) {
        *out_master_clk_on = (value & ES8156_REG08_MASTER_CLK_ON_MASK) >> ES8156_REG08_MASTER_CLK_ON_OFFSET;
    }
    if (out_p2s_clk_on != NULL) {
        *out_p2s_clk_on = (value & ES8156_REG08_P2S_CLK_ON_MASK) >> ES8156_REG08_P2S_CLK_ON_OFFSET;
    }
    return ESP_OK;
}

esp_err_t es8156_write_misc_control_2(es8156_handle_t handle, bool pull_up, bool dll_on, bool csm_cnt_use_master,
                                      bool internal_master_clk_src) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    value |= (pull_up << ES8156_REG09_PUPDN_OFF_OFFSET) & ES8156_REG09_PUPDN_OFF_MASK;
    value |= (dll_on << ES8156_REG09_DLL_ON_OFFSET) & ES8156_REG09_DLL_ON_MASK;
    value |= (csm_cnt_use_master << ES8156_REG09_CSM_CNTSEL_OFFSET) & ES8156_REG09_CSM_CNTSEL_MASK;
    value |= (internal_master_clk_src << ES8156_REG09_MSTCLK_SRCSEL_OFFSET) & ES8156_REG09_MSTCLK_SRCSEL_MASK;
    return es8156_write_register_page(handle, ES8156_PAGE_0, ES8156_REG_MISC_CONTROL_2, value);
}

esp_err_t es8156_read_misc_control_2(es8156_handle_t handle, bool* out_pull_up, bool* out_dll_on,
                                     bool* out_csm_cnt_use_master, bool* out_internal_master_clk_src) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    esp_err_t res = es8156_read_register_page(handle, ES8156_PAGE_0, ES8156_REG_MISC_CONTROL_2, &value);
    if (res != ESP_OK) return res;
    if (out_pull_up != NULL) {
        *out_pull_up = (value & ES8156_REG09_PUPDN_OFF_MASK) >> ES8156_REG09_PUPDN_OFF_OFFSET;
    }
    if (out_dll_on != NULL) {
        *out_dll_on = (value & ES8156_REG09_DLL_ON_MASK) >> ES8156_REG09_DLL_ON_OFFSET;
    }
    if (out_csm_cnt_use_master != NULL) {
        *out_csm_cnt_use_master = (value & ES8156_REG09_CSM_CNTSEL_MASK) >> ES8156_REG09_CSM_CNTSEL_OFFSET;
    }
    if (out_internal_master_clk_src != NULL) {
        *out_internal_master_clk_src = (value & ES8156_REG09_MSTCLK_SRCSEL_MASK) >> ES8156_REG09_MSTCLK_SRCSEL_OFFSET;
    }
    return ESP_OK;
}

esp_err_t es8156_write_time_control_1(es8156_handle_t handle, uint8_t v_t1) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    value |= (v_t1 << ES8156_REG0A_V_T1_OFFSET) & ES8156_REG0A_V_T1_MASK;
    return es8156_write_register_page(handle, ES8156_PAGE_0, ES8156_REG_TIME_CONTROL_1, value);
}

esp_err_t es8156_read_time_control_1(es8156_handle_t handle, uint8_t* out_v_t1) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    esp_err_t res = es8156_read_register_page(handle, ES8156_PAGE_0, ES8156_REG_TIME_CONTROL_1, &value);
    if (res != ESP_OK) return res;
    if (out_v_t1 != NULL) {
        *out_v_t1 = (value & ES8156_REG0A_V_T1_MASK) >> ES8156_REG0A_V_T1_OFFSET;
    }
    return ESP_OK;
}

esp_err_t es8156_write_time_control_2(es8156_handle_t handle, uint8_t v_t2) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    value |= (v_t2 << ES8156_REG0B_V_T2_OFFSET) & ES8156_REG0B_V_T2_MASK;
    return es8156_write_register_page(handle, ES8156_PAGE_0, ES8156_REG_TIME_CONTROL_2, value);
}

esp_err_t es8156_read_time_control_2(es8156_handle_t handle, uint8_t* out_v_t2) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    esp_err_t res = es8156_read_register_page(handle, ES8156_PAGE_0, ES8156_REG_TIME_CONTROL_2, &value);
    if (res != ESP_OK) return res;
    if (out_v_t2 != NULL) {
        *out_v_t2 = (value & ES8156_REG0B_V_T2_MASK) >> ES8156_REG0B_V_T2_OFFSET;
    }
    return ESP_OK;
}

esp_err_t es8156_write_chip_status(es8156_handle_t handle, es8156_csm_state_t force_csm_state) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t force_csm_value = 0;
    switch (force_csm_state) {
        case ES8156_CSM_STATE_S0:
            force_csm_value = 4;
            break;
        case ES8156_CSM_STATE_S1:
            force_csm_value = 5;
            break;
        case ES8156_CSM_STATE_S2:
            force_csm_value = 6;
            break;
        case ES8156_CSM_STATE_S3:
            force_csm_value = 7;
            break;
        case ES8156_CSM_STATE_S6:
            force_csm_value = 6;  // TODO: mistake in datasheet?
            break;
        case ES8156_CSM_STATE_NOT_FORCED:
        default:
            force_csm_value = 0;
            break;
    }
    uint8_t value = 0;
    value |= (force_csm_value << ES8156_REG0C_FORCE_CSM_OFFSET) & ES8156_REG0C_FORCE_CSM_MASK;
    return es8156_write_register_page(handle, ES8156_PAGE_0, ES8156_REG_CHIP_STATUS, value);
}

esp_err_t es8156_read_chip_status(es8156_handle_t handle, es8156_csm_state_t* out_force_csm,
                                  es8156_csm_state_t* out_csm_state) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    esp_err_t res = es8156_read_register_page(handle, ES8156_PAGE_0, ES8156_REG_CHIP_STATUS, &value);
    if (res != ESP_OK) return res;
    if (out_force_csm != NULL) {
        uint8_t force_csm = (value & ES8156_REG0C_FORCE_CSM_MASK) >> ES8156_REG0C_FORCE_CSM_OFFSET;
        switch (force_csm) {
                // TODO: mistake in datasheet?
            case 4:
                *out_force_csm = ES8156_CSM_STATE_S0;
                break;
            case 5:
                *out_force_csm = ES8156_CSM_STATE_S1;
                break;
            case 6:
                *out_force_csm = ES8156_CSM_STATE_S2;
                break;
            case 7:
                *out_force_csm = ES8156_CSM_STATE_S3;
                break;
            case 0:
                *out_force_csm = ES8156_CSM_STATE_NOT_FORCED;
                break;
            default:
                *out_force_csm = ES8156_CSM_STATE_INVALID;
                break;
        }
    }
    if (out_csm_state != NULL) {
        uint8_t csm_state = (value & ES8156_REG0C_CSM_STATE_MASK) >> ES8156_REG0C_CSM_STATE_OFFSET;
        switch (csm_state) {
            case 0:
                *out_csm_state = ES8156_CSM_STATE_S0;
                break;
            case 1:
                *out_csm_state = ES8156_CSM_STATE_S1;
                break;
            case 2:
                *out_csm_state = ES8156_CSM_STATE_S2;
                break;
            case 3:
                *out_csm_state = ES8156_CSM_STATE_S3;
                break;
            case 4:
                *out_csm_state = ES8156_CSM_STATE_S6;
                break;
            default:
                *out_csm_state = ES8156_CSM_STATE_INVALID;
                break;
        }
    }
    return ESP_OK;
}

esp_err_t es8156_write_p2s_control(es8156_handle_t handle, bool p2s_sdout_tri, bool p2s_sdout_sel, bool p2s_sdout_muteb,
                                   bool p2s_nfs_flagoff, uint8_t lrck_1stcnt) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    value |= (p2s_sdout_tri << ES8156_REG0D_P2S_SDOUT_TRI_OFFSET) & ES8156_REG0D_P2S_SDOUT_TRI_MASK;
    value |= (p2s_sdout_sel << ES8156_REG0D_P2S_SDOUT_SEL_OFFSET) & ES8156_REG0D_P2S_SDOUT_SEL_MASK;
    value |= (p2s_sdout_muteb << ES8156_REG0D_P2S_SDOUT_MUTEB_OFFSET) & ES8156_REG0D_P2S_SDOUT_MUTEB_MASK;
    value |= (p2s_nfs_flagoff << ES8156_REG0D_P2S_NFS_FLAGOFF_OFFSET) & ES8156_REG0D_P2S_NFS_FLAGOFF_MASK;
    value |= (lrck_1stcnt << ES8156_REG0D_LRCK_1STCNT_OFFSET) & ES8156_REG0D_LRCK_1STCNT_MASK;
    return es8156_write_register_page(handle, ES8156_PAGE_0, ES8156_REG_P2S_CONTROL, value);
}

esp_err_t es8156_read_p2s_control(es8156_handle_t handle, bool* out_p2s_sdout_tri, bool* out_p2s_sdout_sel,
                                  bool* out_p2s_sdout_muteb, bool* out_p2s_nfs_flagoff, uint8_t* out_lrck_1stcnt) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    esp_err_t res = es8156_read_register_page(handle, ES8156_PAGE_0, ES8156_REG_P2S_CONTROL, &value);
    if (res != ESP_OK) return res;
    if (out_p2s_sdout_tri != NULL) {
        *out_p2s_sdout_tri = (value & ES8156_REG0D_P2S_SDOUT_TRI_MASK) >> ES8156_REG0D_P2S_SDOUT_TRI_OFFSET;
    }
    if (out_p2s_sdout_sel != NULL) {
        *out_p2s_sdout_sel = (value & ES8156_REG0D_P2S_SDOUT_SEL_MASK) >> ES8156_REG0D_P2S_SDOUT_SEL_OFFSET;
    }
    if (out_p2s_sdout_muteb != NULL) {
        *out_p2s_sdout_muteb = (value & ES8156_REG0D_P2S_SDOUT_MUTEB_MASK) >> ES8156_REG0D_P2S_SDOUT_MUTEB_OFFSET;
    }
    if (out_p2s_nfs_flagoff != NULL) {
        *out_p2s_nfs_flagoff = (value & ES8156_REG0D_P2S_NFS_FLAGOFF_MASK) >> ES8156_REG0D_P2S_NFS_FLAGOFF_OFFSET;
    }
    if (out_lrck_1stcnt != NULL) {
        *out_lrck_1stcnt = (value & ES8156_REG0D_LRCK_1STCNT_MASK) >> ES8156_REG0D_LRCK_1STCNT_OFFSET;
    }
    return ESP_OK;
}

esp_err_t es8156_write_dac_counter_parameter(es8156_handle_t handle, uint8_t dac_ns) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    value |= (dac_ns << ES8156_REG10_DAC_NS_OFFSET) & ES8156_REG10_DAC_NS_MASK;
    return es8156_write_register_page(handle, ES8156_PAGE_0, ES8156_REG_DAC_COUNTER_PARAMETER, value);
}

esp_err_t es8156_read_dac_counter_parameter(es8156_handle_t handle, uint8_t* out_dac_ns) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    esp_err_t res = es8156_read_register_page(handle, ES8156_PAGE_0, ES8156_REG_DAC_COUNTER_PARAMETER, &value);
    if (res != ESP_OK) return res;
    if (out_dac_ns != NULL) {
        *out_dac_ns = (value & ES8156_REG10_DAC_NS_MASK) >> ES8156_REG10_DAC_NS_OFFSET;
    }
    return ESP_OK;
}

esp_err_t es8156_write_sdp_interface_config_1(es8156_handle_t handle, uint8_t sp_protocal, bool sp_lrp, bool sp_mute,
                                              uint8_t sp_wl) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    value |= (sp_protocal << ES8156_REG11_SP_PROTOCAL_OFFSET) & ES8156_REG11_SP_PROTOCAL_MASK;
    value |= (sp_lrp << ES8156_REG11_SP_LRP_OFFSET) & ES8156_REG11_SP_LRP_MASK;
    value |= (sp_mute << ES8156_REG11_SP_MUTE_OFFSET) & ES8156_REG11_SP_MUTE_MASK;
    value |= (sp_wl << ES8156_REG11_SP_WL_OFFSET) & ES8156_REG11_SP_WL_MASK;
    return es8156_write_register_page(handle, ES8156_PAGE_0, ES8156_REG_SDP_INTERFACE_CONFIG_1, value);
}

esp_err_t es8156_read_sdp_interface_config_1(es8156_handle_t handle, uint8_t* out_sp_protocal, bool* out_sp_lrp,
                                             bool* out_sp_mute, uint8_t* out_sp_wl) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    esp_err_t res = es8156_read_register_page(handle, ES8156_PAGE_0, ES8156_REG_SDP_INTERFACE_CONFIG_1, &value);
    if (res != ESP_OK) return res;
    if (out_sp_protocal != NULL) {
        *out_sp_protocal = (value & ES8156_REG11_SP_PROTOCAL_MASK) >> ES8156_REG11_SP_PROTOCAL_OFFSET;
    }
    if (out_sp_lrp != NULL) {
        *out_sp_lrp = (value & ES8156_REG11_SP_LRP_MASK) >> ES8156_REG11_SP_LRP_OFFSET;
    }
    if (out_sp_mute != NULL) {
        *out_sp_mute = (value & ES8156_REG11_SP_MUTE_MASK) >> ES8156_REG11_SP_MUTE_OFFSET;
    }
    if (out_sp_wl != NULL) {
        *out_sp_wl = (value & ES8156_REG11_SP_WL_MASK) >> ES8156_REG11_SP_WL_OFFSET;
    }
    return ESP_OK;
}

esp_err_t es8156_write_automute_control(es8156_handle_t handle, uint8_t automute_size, uint8_t automute_ng) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    value |= (automute_size << ES8156_REG12_AUTOMUTE_SIZE_OFFSET) & ES8156_REG12_AUTOMUTE_SIZE_MASK;
    value |= (automute_ng << ES8156_REG12_AUTOMUTE_NG_OFFSET) & ES8156_REG12_AUTOMUTE_NG_MASK;
    return es8156_write_register_page(handle, ES8156_PAGE_0, ES8156_REG_AUTOMUTE_CONTROL, value);
}

esp_err_t es8156_read_automute_control(es8156_handle_t handle, uint8_t* out_automute_size, uint8_t* out_automute_ng) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    esp_err_t res = es8156_read_register_page(handle, ES8156_PAGE_0, ES8156_REG_AUTOMUTE_CONTROL, &value);
    if (res != ESP_OK) return res;
    if (out_automute_size != NULL) {
        *out_automute_size = (value & ES8156_REG12_AUTOMUTE_SIZE_MASK) >> ES8156_REG12_AUTOMUTE_SIZE_OFFSET;
    }
    if (out_automute_ng != NULL) {
        *out_automute_ng = (value & ES8156_REG12_AUTOMUTE_NG_MASK) >> ES8156_REG12_AUTOMUTE_NG_OFFSET;
    }
    return ESP_OK;
}

esp_err_t es8156_write_mute_control(es8156_handle_t handle, bool am_ena, bool lch_dsm_smute, bool rch_dsm_smute,
                                    bool am_dsmmute_ena, bool am_aclkoff_ena, bool am_attenu6_ena, bool intout_clipen) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    value |= (am_ena << ES8156_REG13_AM_ENA_OFFSET) & ES8156_REG13_AM_ENA_MASK_MASK;
    value |= (lch_dsm_smute << ES8156_REG13_LCH_DSM_SMUTE_OFFSET) & ES8156_REG13_LCH_DSM_SMUTE_MASK;
    value |= (rch_dsm_smute << ES8156_REG13_RCH_DSM_SMUTE_OFFSET) & ES8156_REG13_RCH_DSM_SMUTE_MASK;
    value |= (am_dsmmute_ena << ES8156_REG13_AM_DSMMUTE_ENA_OFFSET) & ES8156_REG13_AM_DSMMUTE_ENA_MASK;
    value |= (am_aclkoff_ena << ES8156_REG13_AM_ACLKOFF_ENA_OFFSET) & ES8156_REG13_AM_ACLKOFF_ENA_MASK;
    value |= (am_attenu6_ena << ES8156_REG13_AM_ATTENU6_ENA_OFFSET) & ES8156_REG13_AM_ATTENU6_ENA_MASK;
    value |= (intout_clipen << ES8156_REG13_INTOUT_CLIPEN_OFFSET) & ES8156_REG13_INTOUT_CLIPEN_MASK;
    return es8156_write_register_page(handle, ES8156_PAGE_0, ES8156_REG_MUTE_CONTROL, value);
}

esp_err_t es8156_read_mute_control(es8156_handle_t handle, bool* out_am_ena, bool* out_lch_dsm_smute,
                                   bool* out_rch_dsm_smute, bool* out_am_dsmmute_ena, bool* out_am_aclkoff_ena,
                                   bool* out_am_attenu6_ena, bool* out_intout_clipen) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    esp_err_t res = es8156_read_register_page(handle, ES8156_PAGE_0, ES8156_REG_MUTE_CONTROL, &value);
    if (res != ESP_OK) return res;
    if (out_am_ena != NULL) {
        *out_am_ena = (value & ES8156_REG13_AM_ENA_MASK_MASK) >> ES8156_REG13_AM_ENA_OFFSET;
    }
    if (out_lch_dsm_smute != NULL) {
        *out_lch_dsm_smute = (value & ES8156_REG13_LCH_DSM_SMUTE_MASK) >> ES8156_REG13_LCH_DSM_SMUTE_OFFSET;
    }
    if (out_rch_dsm_smute != NULL) {
        *out_rch_dsm_smute = (value & ES8156_REG13_RCH_DSM_SMUTE_MASK) >> ES8156_REG13_RCH_DSM_SMUTE_OFFSET;
    }
    if (out_am_dsmmute_ena != NULL) {
        *out_am_dsmmute_ena = (value & ES8156_REG13_AM_DSMMUTE_ENA_MASK) >> ES8156_REG13_AM_DSMMUTE_ENA_OFFSET;
    }
    if (out_am_aclkoff_ena != NULL) {
        *out_am_aclkoff_ena = (value & ES8156_REG13_AM_ACLKOFF_ENA_MASK) >> ES8156_REG13_AM_ACLKOFF_ENA_OFFSET;
    }
    if (out_am_attenu6_ena != NULL) {
        *out_am_attenu6_ena = (value & ES8156_REG13_AM_ATTENU6_ENA_MASK) >> ES8156_REG13_AM_ATTENU6_ENA_OFFSET;
    }
    if (out_intout_clipen != NULL) {
        *out_intout_clipen = (value & ES8156_REG13_INTOUT_CLIPEN_MASK) >> ES8156_REG13_INTOUT_CLIPEN_OFFSET;
    }
    return ESP_OK;
}

esp_err_t es8156_write_volume_control(es8156_handle_t handle, uint8_t dac_volume_db) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    value |= (dac_volume_db << ES8156_REG14_DAC_VOLUME_DB_OFFSET) & ES8156_REG14_DAC_VOLUME_DB_MASK;
    return es8156_write_register_page(handle, ES8156_PAGE_0, ES8156_REG_VOLUME_CONTROL, value);
}

esp_err_t es8156_read_volume_control(es8156_handle_t handle, uint8_t* out_dac_volume_db) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    esp_err_t res = es8156_read_register_page(handle, ES8156_PAGE_0, ES8156_REG_VOLUME_CONTROL, &value);
    if (res != ESP_OK) return res;
    *out_dac_volume_db = (value & ES8156_REG14_DAC_VOLUME_DB_MASK) >> ES8156_REG14_DAC_VOLUME_DB_OFFSET;
    return ESP_OK;
}

esp_err_t es8156_write_alc_config_1(es8156_handle_t handle, bool dac_alc_en, uint8_t alc_mute_gain) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    value |= (dac_alc_en << ES8156_REG15_DAC_ALC_EN_OFFSET) & ES8156_REG15_DAC_ALC_EN_MASK;
    value |= (alc_mute_gain << ES8156_REG15_ALC_MUTE_GAIN_OFFSET) & ES8156_REG15_ALC_MUTE_GAIN_MASK;
    return es8156_write_register_page(handle, ES8156_PAGE_0, ES8156_REG_ALC_CONFIG_1, value);
}

esp_err_t es8156_read_alc_config_1(es8156_handle_t handle, bool* out_dac_alc_en, uint8_t* out_alc_mute_gain) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    esp_err_t res = es8156_read_register_page(handle, ES8156_PAGE_0, ES8156_REG_ALC_CONFIG_1, &value);
    if (res != ESP_OK) return res;
    if (out_dac_alc_en != NULL) {
        *out_dac_alc_en = (value & ES8156_REG15_DAC_ALC_EN_MASK) >> ES8156_REG15_DAC_ALC_EN_OFFSET;
    }
    if (out_alc_mute_gain != NULL) {
        *out_alc_mute_gain = (value & ES8156_REG15_ALC_MUTE_GAIN_MASK) >> ES8156_REG15_ALC_MUTE_GAIN_OFFSET;
    }
    return ESP_OK;
}

esp_err_t es8156_write_alc_config_2(es8156_handle_t handle, uint8_t alc_win_size, uint8_t alc_ramp_rate) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    value |= (alc_win_size << ES8156_REG16_ALC_WIN_SIZE_OFFSET) & ES8156_REG16_ALC_WIN_SIZE_MASK;
    value |= (alc_ramp_rate << ES8156_REG16_ALC_RAMP_RATE_OFFSET) & ES8156_REG16_ALC_RAMP_RATE_MASK;
    return es8156_write_register_page(handle, ES8156_PAGE_0, ES8156_REG_ALC_CONFIG_2, value);
}

esp_err_t es8156_read_alc_config_2(es8156_handle_t handle, uint8_t* out_alc_win_size, uint8_t* out_alc_ramp_rate) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    esp_err_t res = es8156_read_register_page(handle, ES8156_PAGE_0, ES8156_REG_ALC_CONFIG_2, &value);
    if (res != ESP_OK) return res;
    if (out_alc_win_size != NULL) {
        *out_alc_win_size = (value & ES8156_REG16_ALC_WIN_SIZE_MASK) >> ES8156_REG16_ALC_WIN_SIZE_OFFSET;
    }
    if (out_alc_ramp_rate != NULL) {
        *out_alc_ramp_rate = (value & ES8156_REG16_ALC_RAMP_RATE_MASK) >> ES8156_REG16_ALC_RAMP_RATE_OFFSET;
    }
    return ESP_OK;
}

esp_err_t es8156_write_alc_level(es8156_handle_t handle, uint8_t alc_minlevel, uint8_t alc_maxlevel) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    value |= (alc_minlevel << ES8156_REG17_ALC_MINLEVEL_OFFSET) & ES8156_REG17_ALC_MINLEVEL_MASK;
    value |= (alc_maxlevel << ES8156_REG17_ALC_MAXLEVEL_OFFSET) & ES8156_REG17_ALC_MAXLEVEL_MASK;
    return es8156_write_register_page(handle, ES8156_PAGE_0, ES8156_REG_ALC_LEVEL, value);
}

esp_err_t es8156_read_alc_level(es8156_handle_t handle, uint8_t* out_alc_minlevel, uint8_t* out_alc_maxlevel) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    esp_err_t res = es8156_read_register_page(handle, ES8156_PAGE_0, ES8156_REG_ALC_LEVEL, &value);
    if (res != ESP_OK) return res;
    if (out_alc_minlevel != NULL) {
        *out_alc_minlevel = (value & ES8156_REG17_ALC_MINLEVEL_MASK) >> ES8156_REG17_ALC_MINLEVEL_OFFSET;
    }
    if (out_alc_maxlevel != NULL) {
        *out_alc_maxlevel = (value & ES8156_REG17_ALC_MAXLEVEL_MASK) >> ES8156_REG17_ALC_MAXLEVEL_OFFSET;
    }
    return ESP_OK;
}

esp_err_t es8156_write_misc_control_3(es8156_handle_t handle, bool dac_ram_clr, bool dsm_ditheron, bool rch_inv,
                                      bool lch_inv, uint8_t chn_cross, bool p2s_dpath_sel, bool p2s_data_bitnum) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    value |= (dac_ram_clr << ES8156_REG18_DAC_RAM_CLR_OFFSET) & ES8156_REG18_DAC_RAM_CLR_MASK;
    value |= (dsm_ditheron << ES8156_REG18_DSM_DITHERON_OFFSET) & ES8156_REG18_DSM_DITHERON_MASK;
    value |= (rch_inv << ES8156_REG18_RCH_INV_OFFSET) & ES8156_REG18_RCH_INV_MASK;
    value |= (lch_inv << ES8156_REG18_LCH_INV_OFFSET) & ES8156_REG18_LCH_INV_MASK;
    value |= (chn_cross << ES8156_REG18_CHN_CROSS_OFFSET) & ES8156_REG18_CHN_CROSS_MASK;
    value |= (p2s_dpath_sel << ES8156_REG18_P2S_DPATH_SEL_OFFSET) & ES8156_REG18_P2S_DPATH_SEL_MASK;
    value |= (p2s_data_bitnum << ES8156_REG18_P2S_DATA_BITNUM_OFFSET) & ES8156_REG18_P2S_DATA_BITNUM_MASK;
    return es8156_write_register_page(handle, ES8156_PAGE_0, ES8156_REG_MISC_CONTROL_3, value);
}

esp_err_t es8156_read_misc_control_3(es8156_handle_t handle, bool* out_dac_ram_clr, bool* out_dsm_ditheron,
                                     bool* out_rch_inv, bool* out_lch_inv, uint8_t* out_chn_cross,
                                     bool* out_p2s_dpath_sel, bool* out_p2s_data_bitnum) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    esp_err_t res = es8156_read_register_page(handle, ES8156_PAGE_0, ES8156_REG_MISC_CONTROL_3, &value);
    if (res != ESP_OK) return res;
    if (out_dac_ram_clr != NULL) {
        *out_dac_ram_clr = (value & ES8156_REG18_DAC_RAM_CLR_MASK) >> ES8156_REG18_DAC_RAM_CLR_OFFSET;
    }
    if (out_dsm_ditheron != NULL) {
        *out_dsm_ditheron = (value & ES8156_REG18_DSM_DITHERON_MASK) >> ES8156_REG18_DSM_DITHERON_OFFSET;
    }
    if (out_rch_inv != NULL) {
        *out_rch_inv = (value & ES8156_REG18_RCH_INV_MASK) >> ES8156_REG18_RCH_INV_OFFSET;
    }
    if (out_lch_inv != NULL) {
        *out_lch_inv = (value & ES8156_REG18_LCH_INV_MASK) >> ES8156_REG18_LCH_INV_OFFSET;
    }
    if (out_chn_cross != NULL) {
        *out_chn_cross = (value & ES8156_REG18_CHN_CROSS_MASK) >> ES8156_REG18_CHN_CROSS_OFFSET;
    }
    if (out_p2s_dpath_sel != NULL) {
        *out_p2s_dpath_sel = (value & ES8156_REG18_P2S_DPATH_SEL_MASK) >> ES8156_REG18_P2S_DPATH_SEL_OFFSET;
    }
    if (out_p2s_data_bitnum != NULL) {
        *out_p2s_data_bitnum = (value & ES8156_REG18_P2S_DATA_BITNUM_MASK) >> ES8156_REG18_P2S_DATA_BITNUM_OFFSET;
    }
    return ESP_OK;
}

esp_err_t es8156_write_eq_control_1(es8156_handle_t handle, bool eq_on, bool eq_cfg_wr, bool eq_cfg_rd, bool eq_rst,
                                    uint8_t eq_band_num) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    value |= (eq_on << ES8156_REG19_EQ_ON_OFFSET) & ES8156_REG19_EQ_ON_MASK;
    value |= (eq_cfg_wr << ES8156_REG19_EQ_CFG_WR_OFFSET) & ES8156_REG19_EQ_CFG_WR_MASK;
    value |= (eq_cfg_rd << ES8156_REG19_EQ_CFG_RD_OFFSET) & ES8156_REG19_EQ_CFG_RD_MASK;
    value |= (eq_rst << ES8156_REG19_EQ_RST_OFFSET) & ES8156_REG19_EQ_RST_MASK;
    value |= (eq_band_num << ES8156_REG19_EQ_BAND_NUM_OFFSET) & ES8156_REG19_EQ_BAND_NUM_MASK;
    return es8156_write_register_page(handle, ES8156_PAGE_0, ES8156_REG_EQ_CONTROL_1, value);
}

esp_err_t es8156_read_eq_control_1(es8156_handle_t handle, bool* out_eq_on, bool* out_eq_cfg_wr, bool* out_eq_cfg_rd,
                                   bool* out_eq_rst, uint8_t* out_eq_band_num) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    esp_err_t res = es8156_read_register_page(handle, ES8156_PAGE_0, ES8156_REG_EQ_CONTROL_1, &value);
    if (res != ESP_OK) return res;
    if (out_eq_on != NULL) {
        *out_eq_on = (value & ES8156_REG19_EQ_ON_MASK) >> ES8156_REG19_EQ_ON_OFFSET;
    }
    if (out_eq_cfg_wr != NULL) {
        *out_eq_cfg_wr = (value & ES8156_REG19_EQ_CFG_WR_MASK) >> ES8156_REG19_EQ_CFG_WR_OFFSET;
    }
    if (out_eq_cfg_rd != NULL) {
        *out_eq_cfg_rd = (value & ES8156_REG19_EQ_CFG_RD_MASK) >> ES8156_REG19_EQ_CFG_RD_OFFSET;
    }
    if (out_eq_rst != NULL) {
        *out_eq_rst = (value & ES8156_REG19_EQ_RST_MASK) >> ES8156_REG19_EQ_RST_OFFSET;
    }
    if (out_eq_band_num != NULL) {
        *out_eq_band_num = (value & ES8156_REG19_EQ_BAND_NUM_MASK) >> ES8156_REG19_EQ_BAND_NUM_OFFSET;
    }
    return ESP_OK;
}

esp_err_t es8156_write_eq_config_2(es8156_handle_t handle, uint8_t eq_1stcnt, bool eq_1stcnt_vld) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    value |= (eq_1stcnt << ES8156_REG1A_EQ_RUN_1STCNT_OFFSET) & ES8156_REG1A_EQ_RUN_1STCNT_MASK;
    value |= (eq_1stcnt_vld << ES8156_REG1A_EQ_1STCNT_VLD_OFFSET) & ES8156_REG1A_EQ_1STCNT_VLD_MASK;
    return es8156_write_register_page(handle, ES8156_PAGE_0, ES8156_REG_EQ_CONFIG_2, value);
}

esp_err_t es8156_read_eq_config_2(es8156_handle_t handle, uint8_t* out_eq_1stcnt, bool* out_eq_1stcnt_vld) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    esp_err_t res = es8156_read_register_page(handle, ES8156_PAGE_0, ES8156_REG_EQ_CONFIG_2, &value);
    if (res != ESP_OK) return res;
    if (out_eq_1stcnt != NULL) {
        *out_eq_1stcnt = (value & ES8156_REG1A_EQ_RUN_1STCNT_MASK) >> ES8156_REG1A_EQ_RUN_1STCNT_OFFSET;
    }
    if (out_eq_1stcnt_vld != NULL) {
        *out_eq_1stcnt_vld = (value & ES8156_REG1A_EQ_1STCNT_VLD_MASK) >> ES8156_REG1A_EQ_1STCNT_VLD_OFFSET;
    }
    return ESP_OK;
}

esp_err_t es8156_write_analog_system_1(es8156_handle_t handle, uint8_t s6_sel, uint8_t s2_sel, uint8_t s3_sel) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    value |= (s6_sel << ES8156_REG20_S6_SEL_OFFSET) & ES8156_REG20_S6_SEL_MASK;
    value |= (s2_sel << ES8156_REG20_S2_SEL_OFFSET) & ES8156_REG20_S2_SEL_MASK;
    value |= (s3_sel << ES8156_REG20_S3_SEL_OFFSET) & ES8156_REG20_S3_SEL_MASK;
    return es8156_write_register_page(handle, ES8156_PAGE_0, ES8156_REG_ANALOG_SYSTEM_1, value);
}

esp_err_t es8156_read_analog_system_1(es8156_handle_t handle, uint8_t* out_s6_sel, uint8_t* out_s2_sel,
                                      uint8_t* out_s3_sel) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    esp_err_t res = es8156_read_register_page(handle, ES8156_PAGE_0, ES8156_REG_ANALOG_SYSTEM_1, &value);
    if (res != ESP_OK) return res;
    if (out_s6_sel != NULL) {
        *out_s6_sel = (value & ES8156_REG20_S6_SEL_MASK) >> ES8156_REG20_S6_SEL_OFFSET;
    }
    if (out_s2_sel != NULL) {
        *out_s2_sel = (value & ES8156_REG20_S2_SEL_MASK) >> ES8156_REG20_S2_SEL_OFFSET;
    }
    if (out_s3_sel != NULL) {
        *out_s3_sel = (value & ES8156_REG20_S3_SEL_MASK) >> ES8156_REG20_S3_SEL_OFFSET;
    }
    return ESP_OK;
}

esp_err_t es8156_write_analog_system_2(es8156_handle_t handle, uint8_t vsel, bool vref_rmpdn1, bool vref_rmpdn2) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    value |= (vsel << ES8156_REG21_VSEL_OFFSET) & ES8156_REG21_VSEL_MASK;
    value |= (vref_rmpdn1 << ES8156_REG21_VREF_RMPDN1_OFFSET) & ES8156_REG21_VREF_RMPDN1_MASK;
    value |= (vref_rmpdn2 << ES8156_REG21_VREF_RMPDN2_OFFSET) & ES8156_REG21_VREF_RMPDN2_MASK;
    return es8156_write_register_page(handle, ES8156_PAGE_0, ES8156_REG_ANALOG_SYSTEM_2, value);
}

esp_err_t es8156_read_analog_system_2(es8156_handle_t handle, uint8_t* out_vsel, bool* out_vref_rmpdn1,
                                      bool* out_vref_rmpdn2) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    esp_err_t res = es8156_read_register_page(handle, ES8156_PAGE_0, ES8156_REG_ANALOG_SYSTEM_2, &value);
    if (res != ESP_OK) return res;
    if (out_vsel != NULL) {
        *out_vsel = (value & ES8156_REG21_VSEL_MASK) >> ES8156_REG21_VSEL_OFFSET;
    }
    if (out_vref_rmpdn1 != NULL) {
        *out_vref_rmpdn1 = (value & ES8156_REG21_VREF_RMPDN1_MASK) >> ES8156_REG21_VREF_RMPDN1_OFFSET;
    }
    if (out_vref_rmpdn2 != NULL) {
        *out_vref_rmpdn2 = (value & ES8156_REG21_VREF_RMPDN2_MASK) >> ES8156_REG21_VREF_RMPDN2_OFFSET;
    }
    return ESP_OK;
}

esp_err_t es8156_write_analog_system_3(es8156_handle_t handle, bool out_mute, bool swrmpsel, bool hpsw) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    value |= (out_mute << ES8156_REG22_OUT_MUTE_OFFSET) & ES8156_REG22_OUT_MUTE_MASK;
    value |= (swrmpsel << ES8156_REG22_SWRMPSEL_OFFSET) & ES8156_REG22_SWRMPSEL_MASK;
    value |= (hpsw << ES8156_REG22_HPSW_OFFSET) & ES8156_REG22_HPSW_MASK;
    return es8156_write_register_page(handle, ES8156_PAGE_0, ES8156_REG_ANALOG_SYSTEM_3, value);
}

esp_err_t es8156_read_analog_system_3(es8156_handle_t handle, bool* out_out_mute, bool* out_swrmpsel, bool* out_hpsw) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    esp_err_t res = es8156_read_register_page(handle, ES8156_PAGE_0, ES8156_REG_ANALOG_SYSTEM_3, &value);
    if (res != ESP_OK) return res;
    if (out_out_mute != NULL) {
        *out_out_mute = (value & ES8156_REG22_OUT_MUTE_MASK) >> ES8156_REG22_OUT_MUTE_OFFSET;
    }
    if (out_swrmpsel != NULL) {
        *out_swrmpsel = (value & ES8156_REG22_SWRMPSEL_MASK) >> ES8156_REG22_SWRMPSEL_OFFSET;
    }
    if (out_hpsw != NULL) {
        *out_hpsw = (value & ES8156_REG22_HPSW_MASK) >> ES8156_REG22_HPSW_OFFSET;
    }
    return ESP_OK;
}

esp_err_t es8156_write_analog_system_4(es8156_handle_t handle, bool hpcom_ref1, bool hpcom_ref2, bool vroi,
                                       bool dac_ibias_sw, uint8_t vmidlvl, uint8_t ibias_sw) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    value |= (hpcom_ref1 << ES8156_REG23_HPCOM_REF1_OFFSET) & ES8156_REG23_HPCOM_REF1_MASK;
    value |= (hpcom_ref2 << ES8156_REG23_HPCOM_REF2_OFFSET) & ES8156_REG23_HPCOM_REF2_MASK;
    value |= (vroi << ES8156_REG23_VROI_OFFSET) & ES8156_REG23_VROI_MASK;
    value |= (dac_ibias_sw << ES8156_REG23_DAC_IBIAS_SW_OFFSET) & ES8156_REG23_DAC_IBIAS_SW_MASK;
    value |= (vmidlvl << ES8156_REG23_VMIDLVL_OFFSET) & ES8156_REG23_VMIDLVL_MASK;
    value |= (ibias_sw << ES8156_REG23_IBIAS_SW_OFFSET) & ES8156_REG23_IBIAS_SW_MASK;
    return es8156_write_register_page(handle, ES8156_PAGE_0, ES8156_REG_ANALOG_SYSTEM_4, value);
}

esp_err_t es8156_read_analog_system_4(es8156_handle_t handle, bool* out_hpcom_ref1, bool* out_hpcom_ref2,
                                      bool* out_vroi, bool* out_dac_ibias_sw, uint8_t* out_vmidlvl,
                                      uint8_t* out_ibias_sw) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    esp_err_t res = es8156_read_register_page(handle, ES8156_PAGE_0, ES8156_REG_ANALOG_SYSTEM_4, &value);
    if (res != ESP_OK) return res;
    if (out_hpcom_ref1 != NULL) {
        *out_hpcom_ref1 = (value & ES8156_REG23_HPCOM_REF1_MASK) >> ES8156_REG23_HPCOM_REF1_OFFSET;
    }
    if (out_hpcom_ref2 != NULL) {
        *out_hpcom_ref2 = (value & ES8156_REG23_HPCOM_REF2_MASK) >> ES8156_REG23_HPCOM_REF2_OFFSET;
    }
    if (out_vroi != NULL) {
        *out_vroi = (value & ES8156_REG23_VROI_MASK) >> ES8156_REG23_VROI_OFFSET;
    }
    if (out_dac_ibias_sw != NULL) {
        *out_dac_ibias_sw = (value & ES8156_REG23_DAC_IBIAS_SW_MASK) >> ES8156_REG23_DAC_IBIAS_SW_OFFSET;
    }
    if (out_vmidlvl != NULL) {
        *out_vmidlvl = (value & ES8156_REG23_VMIDLVL_MASK) >> ES8156_REG23_VMIDLVL_OFFSET;
    }
    if (out_ibias_sw != NULL) {
        *out_ibias_sw = (value & ES8156_REG23_IBIAS_SW_MASK) >> ES8156_REG23_IBIAS_SW_OFFSET;
    }
    return ESP_OK;
}

esp_err_t es8156_write_analog_system_5(es8156_handle_t handle, bool lpvrefbuf, bool lphpcom, bool lpdacvrp,
                                       bool lpdac) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    value |= (lpvrefbuf << ES8156_REG24_LPVREFBUF_OFFSET) & ES8156_REG24_LPVREFBUF_MASK;
    value |= (lphpcom << ES8156_REG24_LPHPCOM_OFFSET) & ES8156_REG24_LPHPCOM_MASK;
    value |= (lpdacvrp << ES8156_REG24_LPDACVRP_OFFSET) & ES8156_REG24_LPDACVRP_MASK;
    value |= (lpdac << ES8156_REG24_LPDAC_OFFSET) & ES8156_REG24_LPDAC_MASK;
    return es8156_write_register_page(handle, ES8156_PAGE_0, ES8156_REG_ANALOG_SYSTEM_5, value);
}

esp_err_t es8156_read_analog_system_5(es8156_handle_t handle, bool* out_lpvrefbuf, bool* out_lphpcom,
                                      bool* out_lpdacvrp, bool* out_lpdac) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    esp_err_t res = es8156_read_register_page(handle, ES8156_PAGE_0, ES8156_REG_ANALOG_SYSTEM_5, &value);
    if (res != ESP_OK) return res;
    if (out_lpvrefbuf != NULL) {
        *out_lpvrefbuf = (value & ES8156_REG24_LPVREFBUF_MASK) >> ES8156_REG24_LPVREFBUF_OFFSET;
    }
    if (out_lphpcom != NULL) {
        *out_lphpcom = (value & ES8156_REG24_LPHPCOM_MASK) >> ES8156_REG24_LPHPCOM_OFFSET;
    }
    if (out_lpdacvrp != NULL) {
        *out_lpdacvrp = (value & ES8156_REG24_LPDACVRP_MASK) >> ES8156_REG24_LPDACVRP_OFFSET;
    }
    if (out_lpdac != NULL) {
        *out_lpdac = (value & ES8156_REG24_LPDAC_MASK) >> ES8156_REG24_LPDAC_OFFSET;
    }
    return ESP_OK;
}

esp_err_t es8156_write_analog_system_6(es8156_handle_t handle, bool pdn_dac, bool pdn_vrefbuf, bool pdn_dacvrefgen,
                                       bool enhpcom, uint8_t vmidsel, bool enrefr, bool pdn_ana) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    value |= (pdn_dac << ES8156_REG25_PDN_DAC_OFFSET) & ES8156_REG25_PDN_DAC_MASK;
    value |= (pdn_vrefbuf << ES8156_REG25_PDN_VREFBUF_OFFSET) & ES8156_REG25_PDN_VREFBUF_MASK;
    value |= (pdn_dacvrefgen << ES8156_REG25_PDN_DACVREFGEN_OFFSET) & ES8156_REG25_PDN_DACVREFGEN_MASK;
    value |= (enhpcom << ES8156_REG25_ENHPCOM_OFFSET) & ES8156_REG25_ENHPCOM_MASK;
    value |= (vmidsel << ES8156_REG25_VMIDSEL_OFFSET) & ES8156_REG25_VMIDSEL_MASK;
    value |= (enrefr << ES8156_REG25_ENREFR_OFFSET) & ES8156_REG25_ENREFR_MASK;
    value |= (pdn_ana << ES8156_REG25_PDN_ANA_OFFSET) & ES8156_REG25_PDN_ANA_MASK;
    return es8156_write_register_page(handle, ES8156_PAGE_0, ES8156_REG_ANALOG_SYSTEM_6, value);
}

esp_err_t es8156_read_analog_system_6(es8156_handle_t handle, bool* out_pdn_dac, bool* out_pdn_vrefbuf,
                                      bool* out_pdn_dacvrefgen, bool* out_enhpcom, uint8_t* out_vmidsel,
                                      bool* out_enrefr, bool* out_pdn_ana) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = 0;
    esp_err_t res = es8156_read_register_page(handle, ES8156_PAGE_0, ES8156_REG_ANALOG_SYSTEM_6, &value);
    if (res != ESP_OK) return res;
    if (out_pdn_dac != NULL) {
        *out_pdn_dac = (value & ES8156_REG25_PDN_DAC_MASK) >> ES8156_REG25_PDN_DAC_OFFSET;
    }
    if (out_pdn_vrefbuf != NULL) {
        *out_pdn_vrefbuf = (value & ES8156_REG25_PDN_VREFBUF_MASK) >> ES8156_REG25_PDN_VREFBUF_OFFSET;
    }
    if (out_pdn_dacvrefgen != NULL) {
        *out_pdn_dacvrefgen = (value & ES8156_REG25_PDN_DACVREFGEN_MASK) >> ES8156_REG25_PDN_DACVREFGEN_OFFSET;
    }
    if (out_enhpcom != NULL) {
        *out_enhpcom = (value & ES8156_REG25_ENHPCOM_MASK) >> ES8156_REG25_ENHPCOM_OFFSET;
    }
    if (out_vmidsel != NULL) {
        *out_vmidsel = (value & ES8156_REG25_VMIDSEL_MASK) >> ES8156_REG25_VMIDSEL_OFFSET;
    }
    if (out_enrefr != NULL) {
        *out_enrefr = (value & ES8156_REG25_ENREFR_MASK) >> ES8156_REG25_ENREFR_OFFSET;
    }
    if (out_pdn_ana != NULL) {
        *out_pdn_ana = (value & ES8156_REG25_PDN_ANA_MASK) >> ES8156_REG25_PDN_ANA_OFFSET;
    }
    return ESP_OK;
}

esp_err_t es8156_eq_data_ram_clear(es8156_handle_t handle) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t buffer[ES8156_EQ_DATA_RAM_LENGTH + 1] = {0};
    buffer[0] = ES8156_REG_EQ_DATA_RAM_CLEAR;
    return ts_i2c_master_transmit(handle, buffer, sizeof(buffer), ES8156_TIMEOUT);
}

esp_err_t es8156_read_chip_id(es8156_handle_t handle, uint16_t* out_id) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value[2];
    esp_err_t res = es8156_read_registers_page(handle, ES8156_PAGE_0, ES8156_REG_CHIP_ID1, value, sizeof(value));
    if (res != ESP_OK) return res;
    *out_id = ((value[0] & ES8156_REGFD_CHIP_ID1_MASK) >> ES8156_REGFD_CHIP_ID1_OFFSET) << 8;
    *out_id |= ((value[1] & ES8156_REGFE_CHIP_ID0_MASK) >> ES8156_REGFE_CHIP_ID0_OFFSET);
    return ESP_OK;
}

esp_err_t es8156_read_chip_version(es8156_handle_t handle, uint8_t* out_version) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value;
    esp_err_t res = es8156_read_register_page(handle, ES8156_PAGE_0, ES8156_REG_CHIP_VERSION, &value);
    if (res != ESP_OK) return res;
    if (out_version != NULL) {
        *out_version = (value & ES8156_REGFF_CHIP_ID_VERSION_ID1_MASK) >> ES8156_REGFF_CHIP_ID_VERSION_ID1_OFFSET;
    }
    return ESP_OK;
}

esp_err_t es8156_write_eq_coefficient(es8156_handle_t handle, uint8_t address, uint8_t value) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    value |= (value << ES8156_PAGE1_REG00_EQ_COEF_OFFSET) & ES8156_PAGE1_REG00_EQ_COEF_MASK;
    return es8156_write_register_page(handle, ES8156_PAGE_1, ES8156_REG_EQ_COEFFICIENT_FIRST + address, value);
}

esp_err_t es8156_read_eq_coefficient(es8156_handle_t handle, uint8_t address, uint8_t* out_value) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    address += ES8156_REG_EQ_COEFFICIENT_FIRST;
    if (address > ES8156_REG_EQ_COEFFICIENT_LAST) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value;
    esp_err_t res = es8156_read_register_page(handle, ES8156_PAGE_1, address, &value);
    if (res != ESP_OK) return res;
    if (out_value != NULL) {
        *out_value = (value & ES8156_PAGE1_REG00_EQ_COEF_MASK) >> ES8156_PAGE1_REG00_EQ_COEF_OFFSET;
    }
    return ESP_OK;
}

// Control routines

esp_err_t es8156_configure(es8156_handle_t handle) {
    esp_err_t res;
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    res = es8156_write_mode_config(handle, false, false, true, false, false, false, false, false);
    if (res != ESP_OK) return res;
    res = es8156_write_analog_system_1(handle, 2, 2, 2);
    if (res != ESP_OK) return res;
    res = es8156_write_analog_system_2(handle, 0x1C, true, false);
    if (res != ESP_OK) return res;
    res = es8156_write_analog_system_3(handle, false, false, false);
    if (res != ESP_OK) return res;
    res = es8156_write_analog_system_5(handle, true, true, true, false);
    if (res != ESP_OK) return res;
    res = es8156_write_analog_system_4(handle, false, false, true, false, 0, 0);
    if (res != ESP_OK) return res;
    res = es8156_write_time_control_1(handle, 1);
    if (res != ESP_OK) return res;
    res = es8156_write_time_control_2(handle, 1);
    if (res != ESP_OK) return res;
    res = es8156_write_sdp_interface_config_1(handle, 0, false, false, 0);
    if (res != ESP_OK) return res;
    res = es8156_write_p2s_control(handle, false, false, true, false, 1);
    if (res != ESP_OK) return res;
    res = es8156_write_misc_control_3(handle, false, false, false, false, 0, false, false);
    if (res != ESP_OK) return res;
    res = es8156_write_clock_off(handle, true, true, true, true, true, true);
    if (res != ESP_OK) return res;
    res = es8156_write_reset_control(handle, false, true, false, false, false, false);
    if (res != ESP_OK) return res;
    res = es8156_write_reset_control(handle, true, true, false, false, false, false);
    if (res != ESP_OK) return res;
    res = es8156_write_analog_system_6(handle, false, false, false, false, 2, false, false);
    if (res != ESP_OK) return res;
    res = es8156_write_analog_system_6(handle, false, false, false, false, 2, false, false);
    if (res != ESP_OK) return res;

    res = es8156_write_volume_control(handle, 100);
    if (res != ESP_OK) return res;

    ESP_LOGI(TAG, "Audio codec configured");

    return ESP_OK;
}

esp_err_t es8156_powerdown(es8156_handle_t handle) {
    esp_err_t res = es8156_write_volume_control(handle, 0);
    if (res != ESP_OK) {
        return res;
    }
    res = es8156_write_eq_control_1(handle, false, true, false, false, 0);
    if (res != ESP_OK) {
        return res;
    }
    res = es8156_write_analog_system_3(handle, false, true, false);
    if (res != ESP_OK) {
        return res;
    }
    res = es8156_write_analog_system_6(handle, true, false, false, false, 0, false, true);
    if (res != ESP_OK) {
        return res;
    }
    res = es8156_write_misc_control_3(handle, true, false, false, false, false, false, false);
    if (res != ESP_OK) {
        return res;
    }
    res = es8156_write_misc_control_2(handle, false, true, false, false);
    if (res != ESP_OK) {
        return res;
    }
    res = es8156_write_misc_control_2(handle, true, false, false, false);
    if (res != ESP_OK) {
        return res;
    }
    res = es8156_write_clock_off(handle, false, false, false, false, false, false);
    if (res != ESP_OK) {
        return res;
    }
    vTaskDelay(pdMS_TO_TICKS(500));
    return es8156_write_analog_system_6(handle, true, true, true, false, 0, false, true);
}

esp_err_t es8156_standby_nopop(es8156_handle_t handle) {
    esp_err_t res = es8156_write_volume_control(handle, 0);
    if (res != ESP_OK) {
        return res;
    }
    res = es8156_write_eq_control_1(handle, false, true, false, false, 0);
    if (res != ESP_OK) {
        return res;
    }
    res = es8156_write_analog_system_6(handle, true, false, false, false, 2, false, true);
    if (res != ESP_OK) {
        return res;
    }
    res = es8156_write_misc_control_3(handle, true, false, false, false, false, false, false);
    if (res != ESP_OK) {
        return res;
    }
    res = es8156_write_misc_control_2(handle, false, true, false, false);
    if (res != ESP_OK) {
        return res;
    }
    res = es8156_write_misc_control_2(handle, true, false, false, false);
    if (res != ESP_OK) {
        return res;
    }
    return es8156_write_clock_off(handle, false, false, false, false, false, false);
}

esp_err_t es8156_reset(es8156_handle_t handle) {
    esp_err_t res = es8156_write_reset_control(handle, false, false, true, true, true, false);
    if (res != ESP_OK) {
        return res;
    }
    return es8156_write_reset_control(handle, true, false, false, false, false, false);
}
