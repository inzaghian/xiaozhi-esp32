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
      event_task_handle_(nullptr), event_queue_handle_(nullptr), event_group_handle_(nullptr) {
}

UartDevice::~UartDevice() {
    if (event_task_handle_) {
        vTaskDelete(event_task_handle_);
    }
    if (event_group_handle_) {
        vEventGroupDelete(event_group_handle_);
    }
    if (initialized_) {
        uart_driver_delete(uart_num_);
    }
}

void UartDevice::Initialize() {
    if (initialized_) {
        return;
    }
    event_group_handle_ = xEventGroupCreate();
    if (!event_group_handle_) {
        ESP_LOGE(TAG, "xEventGroupCreate failed");
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

    xTaskCreatePinnedToCore([](void* arg) {
        auto ml307_at_modem = (UartDevice*)arg;
        ml307_at_modem->EventTask();
        vTaskDelete(NULL);
    }, "modem_event", 2048, this, configMAX_PRIORITIES - 1, &event_task_handle_, 0);

    xTaskCreatePinnedToCore([](void* arg) {
        auto ml307_at_modem = (UartDevice*)arg;
        ml307_at_modem->ReceiveTask();
        vTaskDelete(NULL);
    }, "modem_receive", 2048 * 3, this, configMAX_PRIORITIES - 2, &receive_task_handle_, 0);
    initialized_ = true;
}

void UartDevice::EventTask() {
    uart_event_t event;
    while (true) {
        if (xQueueReceive(event_queue_handle_, &event, portMAX_DELAY) == pdTRUE) {
            switch (event.type)
            {
            case UART_DATA:
                xEventGroupSetBits(event_group_handle_, AT_EVENT_DATA_AVAILABLE);
                break;
            case UART_BREAK:
                xEventGroupSetBits(event_group_handle_, AT_EVENT_BREAK);
                break;
            case UART_BUFFER_FULL:
                xEventGroupSetBits(event_group_handle_, AT_EVENT_BUFFER_FULL);
                break;
            case UART_FIFO_OVF:
                xEventGroupSetBits(event_group_handle_, AT_EVENT_FIFO_OVF);
                break;
            default:
                ESP_LOGE(TAG, "unknown event type: %d", event.type);
                break;
            }
        }
    }
}

void UartDevice::ReceiveTask() {
    while (true) {
        auto bits = xEventGroupWaitBits(event_group_handle_, AT_EVENT_DATA_AVAILABLE | AT_EVENT_FIFO_OVF | AT_EVENT_BUFFER_FULL | AT_EVENT_BREAK, pdTRUE, pdFALSE, portMAX_DELAY);
        if (bits & AT_EVENT_DATA_AVAILABLE) {
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
        }
        if (bits & AT_EVENT_FIFO_OVF) {
            ESP_LOGE(TAG, "FIFO overflow");
            // HandleUrc("FIFO_OVERFLOW", {});
        }
        if (bits & AT_EVENT_BREAK) {
            ESP_LOGE(TAG, "Break");
        }
        if (bits & AT_EVENT_BUFFER_FULL) {
            ESP_LOGE(TAG, "Buffer full");
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