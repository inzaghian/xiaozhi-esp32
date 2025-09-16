
/**
 * @file qmi8658c.c
 * @brief QMI8658C sensor driver
 * @author xyzroe
 * ESP-IDF driver for QMI8658C sensor
 *
 * Datasheet: https://qstcorp.com/upload/pdf/202202/QMI8658C%20datasheet%20rev%200.9.pdf
 *
 * Copyright (c) 2024 xyzroe <i@xyzroe.cc>
 */
#if 1
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_idf_lib_helpers.h>
#include "scd4x.h"

static const char *TAG = "scd4x";

#define CMD_START_PERIODIC_MEASUREMENT             (0x21B1)
#define CMD_READ_MEASUREMENT                       (0xEC05)
#define CMD_STOP_PERIODIC_MEASUREMENT              (0x3F86)
#define CMD_SET_TEMPERATURE_OFFSET                 (0x241D)
#define CMD_GET_TEMPERATURE_OFFSET                 (0x2318)
#define CMD_SET_SENSOR_ALTITUDE                    (0x2427)
#define CMD_GET_SENSOR_ALTITUDE                    (0x2322)
#define CMD_SET_AMBIENT_PRESSURE                   (0xE000)
#define CMD_PERFORM_FORCED_RECALIBRATION           (0x362F)
#define CMD_SET_AUTOMATIC_SELF_CALIBRATION_ENABLED (0x2416)
#define CMD_GET_AUTOMATIC_SELF_CALIBRATION_ENABLED (0x2313)
#define CMD_START_LOW_POWER_PERIODIC_MEASUREMENT   (0x21AC)
#define CMD_GET_DATA_READY_STATUS                  (0xE4B8)
#define CMD_PERSIST_SETTINGS                       (0x3615)
#define CMD_GET_SERIAL_NUMBER                      (0x3682)
#define CMD_PERFORM_SELF_TEST                      (0x3639)
#define CMD_PERFORM_FACTORY_RESET                  (0x3632)
#define CMD_REINIT                                 (0x3646)
#define CMD_MEASURE_SINGLE_SHOT                    (0x219D)
#define CMD_MEASURE_SINGLE_SHOT_RHT_ONLY           (0x2196)
#define CMD_POWER_DOWN                             (0x36E0)
#define CMD_WAKE_UP                                (0x36F6)

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

static uint8_t crc8(const uint8_t *data, size_t count)
{
    uint8_t res = 0xff;

    for (size_t i = 0; i < count; ++i)
    {
        res ^= data[i];
        for (uint8_t bit = 8; bit > 0; --bit)
        {
            if (res & 0x80)
                res = (res << 1) ^ 0x31;
            else
                res = (res << 1);
        }
    }
    return res;
}

static inline uint16_t swap(uint16_t v)
{
    return (v << 8) | (v >> 8);
}


Scd4x::Scd4x(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr) 
{
}

esp_err_t Scd4x::send_cmd(uint16_t cmd, uint16_t *data, size_t words)
{
    uint8_t buf[2 + words * 3] = {0};
    // add command
    *(uint16_t *)buf = swap(cmd);
    if (data && words)
        // add arguments
        for (size_t i = 0; i < words; i++)
        {
            uint8_t *p = buf + 2 + i * 3;
            *(uint16_t *)p = swap(data[i]);
            *(p + 2) = crc8(p, 2);
        }

    Write(buf, sizeof(buf));
    return ESP_OK;
}

esp_err_t Scd4x::read_resp(uint16_t *data, size_t words)
{
    uint8_t buf[words * 3] = {0};
    Read(buf, sizeof(buf));

    for (size_t i = 0; i < words; i++)
    {
        uint8_t *p = buf + i * 3;
        uint8_t crc = crc8(p, 2);
        if (crc != *(p + 2))
        {
            ESP_LOGE(TAG, "Invalid CRC 0x%02x, expected 0x%02x", crc, *(p + 2));
            return ESP_ERR_INVALID_CRC;
        }
        data[i] = swap(*(uint16_t *)p);
    }
    return ESP_OK;
}

esp_err_t Scd4x::execute_cmd(uint16_t cmd, uint32_t timeout_ms,
                             uint16_t *out_data, size_t out_words, uint16_t *in_data, size_t in_words)
{
    // CHECK_ARG();

    // I2C_DEV_TAKE_MUTEX();
    send_cmd(cmd, out_data, out_words);
    if (timeout_ms)
    {
        if (timeout_ms > 10)
            vTaskDelay(pdMS_TO_TICKS(timeout_ms));
        else
            esp_rom_delay_us(timeout_ms * 1000);
    }
    if (in_data && in_words)
        read_resp(in_data, in_words);
    // I2C_DEV_GIVE_MUTEX();

    return ESP_OK;
}


///////////////////////////////////////////////////////////////////////////////

esp_err_t Scd4x::start_periodic_measurement(void)
{
    return execute_cmd(CMD_START_PERIODIC_MEASUREMENT, 1, NULL, 0, NULL, 0);
}

esp_err_t Scd4x::read_measurement_ticks(uint16_t *co2, uint16_t *temperature, uint16_t *humidity)
{
    CHECK_ARG(co2 || temperature || humidity);

    uint16_t buf[3]={0};
    CHECK(execute_cmd(CMD_READ_MEASUREMENT, 1, NULL, 0, buf, 3));
    if (co2)
        *co2 = buf[0];
    if (temperature)
        *temperature = buf[1];
    if (humidity)
        *humidity = buf[2];

    return ESP_OK;
}

esp_err_t Scd4x::read_measurement(uint16_t *co2, float *temperature, float *humidity)
{
    CHECK_ARG(co2 || temperature || humidity);
    uint16_t t_raw, h_raw;

    CHECK(read_measurement_ticks(co2, &t_raw, &h_raw));
    if (temperature)
        *temperature = (float)t_raw * 175.0f / 65536.0f - 45.0f;
    if (humidity)
        *humidity = (float)h_raw * 100.0f / 65536.0f;

    return ESP_OK;
}

esp_err_t Scd4x::stop_periodic_measurement(void)
{
    return execute_cmd(CMD_STOP_PERIODIC_MEASUREMENT, 500, NULL, 0, NULL, 0);
}

esp_err_t Scd4x::get_temperature_offset_ticks(uint16_t *t_offset)
{
    CHECK_ARG(t_offset);

    return execute_cmd(CMD_GET_TEMPERATURE_OFFSET, 1, NULL, 0, t_offset, 1);
}

esp_err_t Scd4x::get_temperature_offset(float *t_offset)
{
    CHECK_ARG(t_offset);
    uint16_t raw = 0;

    CHECK(get_temperature_offset_ticks(&raw));

    *t_offset = (float)raw * 175.0f / 65536.0f;

    return ESP_OK;
}

esp_err_t Scd4x::set_temperature_offset_ticks(uint16_t t_offset)
{
    return execute_cmd(CMD_SET_TEMPERATURE_OFFSET, 1, &t_offset, 1, NULL, 0);
}

esp_err_t Scd4x::set_temperature_offset(float t_offset)
{
    uint16_t raw = (uint16_t)(t_offset * 65536.0f / 175.0f + 0.5f);
    return set_temperature_offset_ticks(raw);
}

esp_err_t Scd4x::get_sensor_altitude(uint16_t *altitude)
{
    CHECK_ARG(altitude);

    return execute_cmd(CMD_GET_SENSOR_ALTITUDE, 1, NULL, 0, altitude, 1);
}

esp_err_t Scd4x::set_sensor_altitude(uint16_t altitude)
{
    return execute_cmd(CMD_SET_SENSOR_ALTITUDE, 1, &altitude, 1, NULL, 0);
}

esp_err_t Scd4x::set_ambient_pressure(uint16_t pressure)
{
    return execute_cmd(CMD_SET_AMBIENT_PRESSURE, 1, &pressure, 1, NULL, 0);
}

esp_err_t Scd4x::perform_forced_recalibration(uint16_t target_co2_concentration,
                                             uint16_t *frc_correction)
{
    CHECK_ARG(frc_correction);

    return execute_cmd(CMD_PERFORM_FORCED_RECALIBRATION, 400,
                       &target_co2_concentration, 1, frc_correction, 1);
}

esp_err_t Scd4x::get_automatic_self_calibration(bool *enabled)
{
    CHECK_ARG(enabled);
    uint16_t e = *enabled;
    esp_err_t err = execute_cmd(CMD_GET_AUTOMATIC_SELF_CALIBRATION_ENABLED, 1, NULL, 0, &e, 1);
    if (err == ESP_OK)
    {
        *enabled = !!e;
    }
    return err;
}

esp_err_t Scd4x::set_automatic_self_calibration(bool enabled)
{
    uint16_t e = enabled;
    return execute_cmd(CMD_SET_AUTOMATIC_SELF_CALIBRATION_ENABLED, 1, &e, 1, NULL, 0);
}

esp_err_t Scd4x::start_low_power_periodic_measurement(void)
{
    return execute_cmd(CMD_START_LOW_POWER_PERIODIC_MEASUREMENT, 0, NULL, 0, NULL, 0);
}

esp_err_t Scd4x::get_data_ready_status(bool *data_ready)
{
    CHECK_ARG(data_ready);

    uint16_t status = 0;
    CHECK(execute_cmd(CMD_GET_DATA_READY_STATUS, 1, NULL, 0, &status, 1));
    *data_ready = (status & 0x7ff) != 0;

    return ESP_OK;
}

esp_err_t Scd4x::persist_settings(void)
{
    return execute_cmd(CMD_PERSIST_SETTINGS, 800, NULL, 0, NULL, 0);
}

esp_err_t Scd4x::get_serial_number(uint16_t *serial0, uint16_t *serial1, uint16_t *serial2)
{
    CHECK_ARG(serial0 && serial1 && serial2);

    uint16_t buf[3] = {0};
    CHECK(execute_cmd(CMD_GET_SERIAL_NUMBER, 1, NULL, 0, buf, 3));
    *serial0 = buf[0];
    *serial1 = buf[1];
    *serial2 = buf[2];

    return ESP_OK;
}

esp_err_t Scd4x::perform_self_test(bool *malfunction)
{
    CHECK_ARG(malfunction);

    return execute_cmd(CMD_PERFORM_SELF_TEST, 10000, NULL, 0, (uint16_t *)malfunction, 1);
}

esp_err_t Scd4x::perform_factory_reset(void)
{
    return execute_cmd(CMD_PERFORM_FACTORY_RESET, 800, NULL, 0, NULL, 0);
}

esp_err_t Scd4x::reinit(void)
{
    return execute_cmd(CMD_REINIT, 20, NULL, 0, NULL, 0);
}

esp_err_t Scd4x::measure_single_shot(void)
{
    return execute_cmd(CMD_MEASURE_SINGLE_SHOT, 5000, NULL, 0, NULL, 0);
}

esp_err_t Scd4x::measure_single_shot_rht_only(void)
{
    return execute_cmd(CMD_MEASURE_SINGLE_SHOT_RHT_ONLY, 50, NULL, 0, NULL, 0);
}

esp_err_t Scd4x::power_down(void)
{
    return execute_cmd(CMD_POWER_DOWN, 1, NULL, 0, NULL, 0);
}

esp_err_t Scd4x::wake_up(void)
{
    return execute_cmd(CMD_WAKE_UP, 20, NULL, 0, NULL, 0);
}

#endif