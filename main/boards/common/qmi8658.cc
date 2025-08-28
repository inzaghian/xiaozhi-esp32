
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

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_idf_lib_helpers.h>
#include "qmi8658.h"

static const char *TAG = "qmi8658c";

Qmi8658::Qmi8658(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr) 
{
    // WriteReg(QMI8658_RESET, 0xB0);
}

void Qmi8658::reset(void)
{
    WriteReg(QMI8658_RESET, 0xB0);
}

esp_err_t Qmi8658::setup(qmi8658c_config_t *config)
{
    // CHECK_ARG(config);

    // reset device
    reset();
    vTaskDelay(pdMS_TO_TICKS(10));
    // set mode
    uint8_t ctrl7 = ReadReg(QMI8658_CTRL7);
    // ESP_LOGI(TAG,"1 ctrl7=0x%02X",ctrl7);
    // CHECK(ReadReg(QMI8658_CTRL7, &ctrl7));
    ctrl7 = (ctrl7 & 0xFC) | config->mode;
    WriteReg(QMI8658_CTRL7, ctrl7);
    // ESP_LOGI(TAG,"2 ctrl7=0x%02X",ctrl7);
    // ctrl7 = ReadReg(QMI8658_CTRL7);
    // ESP_LOGI(TAG,"3 ctrl7=0x%02X",ctrl7);
    // CHECK(send_command(dev, QMI8658_CTRL7, ctrl7));

    // set accelerometr scale and ODR
    uint8_t ctrl2 = ReadReg(QMI8658_CTRL2);
    // CHECK(ReadReg(QMI8658_CTRL2, &ctrl2));
    ctrl2 = (ctrl2 & 0xF0) | config->acc_odr;
    ctrl2 = (ctrl2 & 0x8F) | (config->acc_scale << 4);
    WriteReg(QMI8658_CTRL2, ctrl2);
    // CHECK(send_command(dev, QMI8658_CTRL2, ctrl2));

    // set accelerometer scale and sensitivity
    qmi_ctx.acc_scale = config->acc_scale;
    qmi_ctx.acc_sensitivity = acc_scale_sensitivity_table[config->acc_scale];

    // set gyroscope scale and ODR
    uint8_t ctrl3 = ReadReg(QMI8658_CTRL3);
    // CHECK(read_register(dev, QMI8658_CTRL3, &ctrl3));
    ctrl3 = (ctrl3 & 0xF0) | config->gyro_odr;
    ctrl3 = (ctrl3 & 0x8F) | (config->gyro_scale << 4);
    WriteReg(QMI8658_CTRL3, ctrl3);
    // CHECK(send_command(dev, QMI8658_CTRL3, ctrl3));

    // set gyroscope scale and sensitivity
    qmi_ctx.gyro_scale = config->gyro_scale;
    qmi_ctx.gyro_sensitivity = gyro_scale_sensitivity_table[config->gyro_scale];

    // read device ID and revision ID
    // CHECK(read_register(dev, QMI8658_WHO_AM_I, &qmi_ctx.who_am_i));
    qmi_ctx.who_am_i = ReadReg(QMI8658_WHO_AM_I);
    // CHECK(read_register(dev, QMI8658_REVISION, &qmi_ctx.revision));
    qmi_ctx.revision = ReadReg(QMI8658_REVISION);

    ESP_LOGI(TAG, "device ID: 0x%02X, revision: 0x%02X", qmi_ctx.who_am_i, qmi_ctx.revision);

    // Verify mode setting
    uint8_t qmi8658_ctrl7 = ReadReg(QMI8658_CTRL7);
    // CHECK(read_register(dev, QMI8658_CTRL7, &qmi8658_ctrl7));
    if ((qmi8658_ctrl7 & 0x03) != config->mode)
    {
        ESP_LOGE(TAG, "Mode setting verification failed");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t Qmi8658::read_data(qmi8658c_data_t *data)
{
    // CHECK_ARG(dev && data);

    // Read accelerometer data
    // ESP_LOGI(TAG,"ctrl1:%02x", ReadReg(QMI8658_CTRL1));
    // ESP_LOGI(TAG,"ctrl2:%02x", ReadReg(QMI8658_CTRL2));
    // ESP_LOGI(TAG,"ctrl3:%02x", ReadReg(QMI8658_CTRL3));
    // ESP_LOGI(TAG,"ctrl4:%02x", ReadReg(QMI8658_CTRL4));
    // ESP_LOGI(TAG,"ctrl5:%02x", ReadReg(QMI8658_CTRL5));
    // ESP_LOGI(TAG,"ctrl6:%02x", ReadReg(QMI8658_CTRL6));
    // ESP_LOGI(TAG,"ctrl7:%02x", ReadReg(QMI8658_CTRL7));
    // ESP_LOGI(TAG,"ctrl9:%02x", ReadReg(QMI8658_CTRL9));
    
    // ESP_LOGI(TAG,"status0:%02x", ReadReg(QMI8658_STATUS0));
    // ESP_LOGI(TAG,"status1:%02x", ReadReg(0x2F));
    // ESP_LOGI(TAG,"statusint:%02x", ReadReg(0x2D));
    // ESP_LOGI(TAG,"0x46:%02x", ReadReg(0x46));
    if((ReadReg(QMI8658_STATUS0) & 0x03) == 0)
    {
        ESP_LOGE(TAG,"data is not ready");
        return -1;
    }
    
    
    uint8_t acc_x_l, acc_x_h, acc_y_l, acc_y_h, acc_z_l, acc_z_h;
    // CHECK(read_register(dev, QMI8658_ACC_X_L, &acc_x_l));
    // CHECK(read_register(dev, QMI8658_ACC_X_H, &acc_x_h));
    // CHECK(read_register(dev, QMI8658_ACC_Y_L, &acc_y_l));
    // CHECK(read_register(dev, QMI8658_ACC_Y_H, &acc_y_h));
    // CHECK(read_register(dev, QMI8658_ACC_Z_L, &acc_z_l));
    // CHECK(read_register(dev, QMI8658_ACC_Z_H, &acc_z_h));
    acc_x_l = ReadReg(QMI8658_ACC_X_L);
    acc_x_h = ReadReg(QMI8658_ACC_X_H);
    acc_y_l = ReadReg(QMI8658_ACC_Y_L);
    acc_y_h = ReadReg(QMI8658_ACC_Y_H);
    acc_z_l = ReadReg(QMI8658_ACC_Z_L);
    acc_z_h = ReadReg(QMI8658_ACC_Z_H);

    // ESP_LOGI(TAG, "acc_x_l: %d, acc_x_h: %d, acc_y_l: %d, acc_y_h: %d, acc_z_l: %d, acc_z_h: %d", acc_x_l, acc_x_h, acc_y_l, acc_y_h, acc_z_l, acc_z_h);

    int16_t acc_x = (int16_t)((acc_x_h << 8) | acc_x_l);
    int16_t acc_y = (int16_t)((acc_y_h << 8) | acc_y_l);
    int16_t acc_z = (int16_t)((acc_z_h << 8) | acc_z_l);

    // ESP_LOGI(TAG, "acc_x: %d, acc_y: %d, acc_z: %d", acc_x, acc_y, acc_z);

    data->acc.x = (float)acc_x / qmi_ctx.acc_sensitivity;
    data->acc.y = (float)acc_y / qmi_ctx.acc_sensitivity;
    data->acc.z = (float)acc_z / qmi_ctx.acc_sensitivity;

    // Read gyroscope data
    uint8_t gyr_x_l, gyr_x_h, gyr_y_l, gyr_y_h, gyr_z_l, gyr_z_h;
    // CHECK(read_register(dev, QMI8658_GYR_X_L, &gyr_x_l));
    // CHECK(read_register(dev, QMI8658_GYR_X_H, &gyr_x_h));
    // CHECK(read_register(dev, QMI8658_GYR_Y_L, &gyr_y_l));
    // CHECK(read_register(dev, QMI8658_GYR_Y_H, &gyr_y_h));
    // CHECK(read_register(dev, QMI8658_GYR_Z_L, &gyr_z_l));
    // CHECK(read_register(dev, QMI8658_GYR_Z_H, &gyr_z_h));
    gyr_x_l = ReadReg(QMI8658_GYR_X_L);
    gyr_x_h = ReadReg(QMI8658_GYR_X_H);
    gyr_y_l = ReadReg(QMI8658_GYR_Y_L);
    gyr_y_h = ReadReg(QMI8658_GYR_Y_H);
    gyr_z_l = ReadReg(QMI8658_GYR_Z_L);
    gyr_z_h = ReadReg(QMI8658_GYR_Z_H);

    // ESP_LOGE(TAG, "gyr_x_l: %d, gyr_x_h: %d, gyr_y_l: %d, gyr_y_h: %d, gyr_z_l: %d, gyr_z_h: %d", gyr_x_l, gyr_x_h, gyr_y_l, gyr_y_h, gyr_z_l, gyr_z_h);

    int16_t gyr_x = (int16_t)((gyr_x_h << 8) | gyr_x_l);
    int16_t gyr_y = (int16_t)((gyr_y_h << 8) | gyr_y_l);
    int16_t gyr_z = (int16_t)((gyr_z_h << 8) | gyr_z_l);

    // ESP_LOGW(TAG, "gyr_x: %d, gyr_y: %d, gyr_z: %d", gyr_x, gyr_y, gyr_z);

    data->gyro.x = (float)gyr_x / qmi_ctx.gyro_sensitivity;
    data->gyro.y = (float)gyr_y / qmi_ctx.gyro_sensitivity;
    data->gyro.z = (float)gyr_z / qmi_ctx.gyro_sensitivity;

    // Read temperature data
    uint8_t temp_l, temp_h;
    // CHECK(read_register(dev, QMI8658_TEMP_L, &temp_l));
    // CHECK(read_register(dev, QMI8658_TEMP_H, &temp_h));
    temp_l = ReadReg(QMI8658_TEMP_L);
    temp_h = ReadReg(QMI8658_TEMP_H);

    // ESP_LOGI(TAG, "temp_l: %d, temp_h: %d", temp_l, temp_h);

    int16_t temp = (int16_t)((temp_h << 8) | temp_l);
    // ESP_LOGW(TAG, "temp: %d", temp);

    data->temperature = (float)temp / TEMPERATURE_SENSOR_RESOLUTION;

    // ESP_LOGI(TAG, "Acc: x=%f, y=%f, z=%f; Gyro: x=%f, y=%f, z=%f; Temp: %f",
    //          data->acc.x, data->acc.y, data->acc.z, data->gyro.x, data->gyro.y, data->gyro.z, data->temperature);
    // WriteReg(QMI8658_CTRL7,0x03);
    return ESP_OK;
}