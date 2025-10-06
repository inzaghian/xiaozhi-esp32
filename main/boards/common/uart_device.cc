#include "uart_device.h"
#include <esp_log.h>
#include <esp_err.h>
#include <algorithm>
#include <cstring>
#include <cstdlib>
#include <sstream>

#define TAG "UartDevice"

// UartDevice 构造函数实现
UartDevice::UartDevice(gpio_num_t tx_pin, gpio_num_t rx_pin, gpio_num_t dtr_pin, int baud_rate)
    : tx_pin_(tx_pin), rx_pin_(rx_pin), dtr_pin_(dtr_pin), uart_num_(UART_NUM),
      baud_rate_(baud_rate), initialized_(false),
      event_task_handle_(nullptr), event_queue_handle_(nullptr) {
}

UartDevice::~UartDevice() {
    if (event_task_handle_) {
        vTaskDelete(event_task_handle_);
    }

    if (initialized_) {
        uart_driver_delete(uart_num_);
    }
}

void UartDevice::Initialize() {
    if (initialized_) {
        return;
    }

    uart_config_t uart_config = {};
    uart_config.baud_rate = baud_rate_;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.source_clk = UART_SCLK_DEFAULT;
    
    ESP_ERROR_CHECK(uart_driver_install(uart_num_, 8192, 0, 100, &event_queue_handle_, ESP_INTR_FLAG_IRAM));
    ESP_ERROR_CHECK(uart_param_config(uart_num_, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num_, tx_pin_, rx_pin_, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    if (dtr_pin_ != GPIO_NUM_NC) {
        gpio_config_t config = {};
        config.pin_bit_mask = (1ULL << dtr_pin_);
        config.mode = GPIO_MODE_OUTPUT;
        config.pull_up_en = GPIO_PULLUP_DISABLE;
        config.pull_down_en = GPIO_PULLDOWN_DISABLE;
        config.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&config);
        gpio_set_level(dtr_pin_, 0);
    }

    xTaskCreate([](void* arg) {
        auto _self = (UartDevice*)arg;
        _self->EventTask();
        vTaskDelete(NULL);
    }, "uart device task", 2048, this, configMAX_PRIORITIES - 1, &event_task_handle_);

    initialized_ = true;
}

void UartDevice::EventTask() {
    uart_event_t event;
    while (true) {
        if (xQueueReceive(event_queue_handle_, &event, portMAX_DELAY) == pdTRUE) {
            switch (event.type)
            {
            case UART_DATA:
                size_t available;
                uart_get_buffered_data_len(uart_num_, &available);
                if (available > 0) {
                    // Extend rx_buffer_ and read into buffer
                    rx_buffer_.resize(rx_buffer_.size() + available);
                    char* rx_buffer_ptr = &rx_buffer_[rx_buffer_.size() - available];
                    uart_read_bytes(uart_num_, rx_buffer_ptr, available, portMAX_DELAY);
                    // while (ParseResponse()) {}
                    ESP_LOGI(TAG, "Received %d bytes: %s", available, rx_buffer_ptr);
                }
                break;
            case UART_BREAK:
                ESP_LOGE(TAG, "Break");
                break;
            case UART_BUFFER_FULL:
                ESP_LOGE(TAG, "Buffer full");
                break;
            case UART_FIFO_OVF:
                ESP_LOGE(TAG, "FIFO overflow");
                break;
            default:
                ESP_LOGE(TAG, "unknown event type: %d", event.type);
                break;
            }
        }
    }
}

bool UartDevice::SendData(const char* data, size_t length) {
    if (!initialized_) {
        ESP_LOGE(TAG, "uart not initialized");
        return false;
    }
    
    int ret = uart_write_bytes(uart_num_, data, length);
    if (ret < 0) {
        ESP_LOGE(TAG, "uart_write_bytes failed: %d", ret);
        return false;
    }
    return true;
}

bool UartDevice::ParseResponse() {

    auto end_pos = rx_buffer_.find("\r\n");
    if (end_pos == std::string::npos) {
        // FIXME: for +MHTTPURC: "ind", missing newline
            return false;
    }
    // Ignore empty lines
    if (end_pos == 0) {
        rx_buffer_.erase(0, 2);
        return true;
    }

    ESP_LOGI(TAG, "<< %.64s (%u bytes)", rx_buffer_.substr(0, end_pos).c_str(), end_pos);
    //从第一行开始解析NMEA协议
    std::string line = rx_buffer_.substr(0, end_pos);
    if (line.find("$GPGGA") != std::string::npos) {
        // 解析GPGGA数据
        std::stringstream ss(line.c_str());
        std::string token;
        // TODO: 实现GPGGA数据解析逻辑
        // 例如：解析时间、纬度、经度、高度等信息
        // 解析GPGGA数据
        if (std::getline(ss, token, ',')) { // 读取时间
            // TODO: 处理时间信息
            std::string time_str = token.substr(0, 2) + ":" + token.substr(2, 2) + ":" + token.substr(4, 2);
            ESP_LOGI(TAG, "time: %s", time_str.c_str());
        }
        if (std::getline(ss, token, ',')) { // 读取纬度
            // TODO: 处理纬度信息
            std::string lat_str = token.substr(0, 2) + "." + token.substr(2, 2) + "." + token.substr(4, 2);
            ESP_LOGI(TAG, "lat: %s", lat_str.c_str());
        }
        if (std::getline(ss, token, ',')) { // 读取纬度方向
            // TODO: 处理纬度方向信息
            std::string lat_dir = token;
            ESP_LOGI(TAG, "lat dir: %s", lat_dir.c_str());
        }
        if (std::getline(ss, token, ',')) { // 读取经度
            // TODO: 处理经度信息
            std::string lon_str = token.substr(0, 3) + "." + token.substr(3, 2) + "." + token.substr(5, 2);

            ESP_LOGI(TAG, "lon: %s", lon_str.c_str());
        }
        if (std::getline(ss, token, ',')) { // 读取经度方向
            // TODO: 处理经度方向信息
            std::string lon_dir = token;
            ESP_LOGI(TAG, "lon dir: %s", lon_dir.c_str());
        }
        if (std::getline(ss, token, ',')) { // 读取高度
            // TODO: 处理高度信息
            std::string height_str = token.substr(0, 2) + "." + token.substr(2, 2);
            ESP_LOGI(TAG, "height: %s", height_str.c_str());
        }
        if (std::getline(ss, token, ',')) { // 读取高度单位
            // TODO: 处理高度单位信息
            std::string height_unit = token;
            ESP_LOGI(TAG, "height unit: %s", height_unit.c_str());
        }
        return true;
    }
    else if (line.find("$GPRMC") != std::string::npos)

    {
        // 解析GPRMC数据
        std::stringstream ss(line.c_str());
        std::string token;
        // TODO: 实现GPRMC数据解析逻辑
        if (std::getline(ss, token, ',')) { // 读取时间
            // TODO: 处理时间信息
            std::string time_str = token.substr(0, 2) + ":" + token.substr(2, 2) + ":" + token.substr(4, 2);
            ESP_LOGI(TAG, "time: %s", time_str.c_str());
        }
        if (std::getline(ss, token, ',')) { // 读取状态
            // TODO: 处理状态信息
            std::string status = token;
            ESP_LOGI(TAG, "status: %s", status.c_str());
        }
        if (std::getline(ss, token, ',')) { // 读取纬度
            // TODO: 处理纬度信息
            std::string lat_str = token.substr(0, 2) + "." + token.substr(2, 2) + "." + token.substr(4, 2);
            ESP_LOGI(TAG, "lat: %s", lat_str.c_str());

        }
        if (std::getline(ss, token, ',')) { // 读取纬度方向

            // TODO: 处理纬度方向信息

            std::string lat_dir = token;

            ESP_LOGI(TAG, "lat dir: %s", lat_dir.c_str());
        }
        
        if (std::getline(ss, token, ',')) { // 读取经度
            // TODO: 处理经度信息
            std::string lon_str = token.substr(0, 3) + "." + token.substr(3, 2) + "." + token.substr(5, 2);
            ESP_LOGI(TAG, "lon: %s", lon_str.c_str());
        }
        
        if (std::getline(ss, token, ',')) { // 读取速度
            // TODO: 处理速度信息
            std::string speed_str = token;
            ESP_LOGI(TAG, "speed: %s", speed_str.c_str());
        }
        
        if (std::getline(ss, token, ',')) { // 读取航向
            // TODO: 处理航向信息
            std::string heading_str = token;
            ESP_LOGI(TAG, "heading: %s", heading_str.c_str());
        }

        return true;
    }
    
    rx_buffer_.erase(0, end_pos + 2);
    return false;
    
}