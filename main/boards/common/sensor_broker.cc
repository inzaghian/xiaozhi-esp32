
/**
 * @file sensor_broker.c
 * @brief sensor_broker
 * @author anzhu
 * 
 *
 * 
 *
 * Copyright (c) 2025
 */

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_idf_lib_helpers.h>
#include "application.h"
#include "sensor_broker.h"

static const char *TAG = "sensor_broker";
SensorBroker::SensorBroker()
{
    cnt = 0;
    co2_cnt_good = 0;
    co2_cnt_bad = 0;
    
    state = ENV_GOOD;
    xTaskCreate([](void* arg) {
        auto this_ = (SensorBroker*)arg;
        this_->SensorBrokerTask();
        vTaskDelete(NULL);
    }, "sensor_broker", 4096, this, 3, nullptr);
}

void SensorBroker::SensorBrokerTask()
{
    auto& app = Application::GetInstance();
    auto& board = Board::GetInstance();
    auto imu = (Qmi8658 *)board.GetImu();
    auto scd4x_ = (Scd4x *)board.GetEnvSensor();

    while (true)
    {
        cnt++;
        if(cnt % 10 == 0) {
            // imu->data_update();
            // ESP_LOGI(TAG,"%s",imu->data_json_get().c_str());
        }
        if(cnt % 100 == 0) {
            scd4x_->data_update();
            ESP_LOGI(TAG,"%s",scd4x_->data_json_get().c_str());
            if(scd4x_->data_.co2 > 700) {
                if (state == ENV_GOOD && co2_cnt_bad >= 3) {
                    state = ENV_BAD;
                    co2_cnt_bad = 0;
                    app.WakeWordInvoke(std::string ("这是一条警告，二氧化碳浓度过高, 请先播报在回答"));
                }
                else {
                    co2_cnt_bad++;
                    co2_cnt_good = 0;
                }
            }
            else {
                if (state == ENV_BAD && co2_cnt_good >= 3) {
                    state = ENV_GOOD;
                    co2_cnt_good = 0;
                    app.WakeWordInvoke(std::string ("这是一条通知，二氧化碳浓度已经恢复正常, 请先播报在回答"));
                }
                else {
                    co2_cnt_good++;
                    co2_cnt_bad = 0;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
}
