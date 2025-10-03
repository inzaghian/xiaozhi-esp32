#ifndef UART_DEVICE_H
#define UART_DEVICE_H

#include <string>
#include <vector>
#include <functional>
#include <mutex>
#include <list>
#include <cstdlib>
#include <memory>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/event_groups.h>
#include <driver/gpio.h>
#include <driver/uart.h>

// UART事件定义
#define AT_EVENT_DATA_AVAILABLE BIT1
#define AT_EVENT_COMMAND_DONE   BIT2
#define AT_EVENT_COMMAND_ERROR  BIT3
#define AT_EVENT_BUFFER_FULL    BIT4
#define AT_EVENT_FIFO_OVF       BIT5
#define AT_EVENT_BREAK          BIT6
#define AT_EVENT_UNKNOWN        BIT7

// 默认配置
#define UART_NUM                UART_NUM_1

class UartDevice {
public:
    // 构造函数
    UartDevice(gpio_num_t tx_pin, gpio_num_t rx_pin, gpio_num_t dtr_pin = GPIO_NUM_NC, int baud_rate = 115200);
    ~UartDevice();

    // 初始化和配置
    void Initialize();
    
    // 数据发送
    bool SendData(const char* data, size_t length);
private:
        // 配置参数
    gpio_num_t tx_pin_;
    gpio_num_t rx_pin_;
    gpio_num_t dtr_pin_;
    uart_port_t uart_num_;
    int baud_rate_;
    bool initialized_;
    // int cme_error_code_ = 0;
    // std::string response_;
    // bool wait_for_response_ = false;
    // std::mutex command_mutex_;
    // std::mutex mutex_;
    
    // FreeRTOS 对象
    TaskHandle_t event_task_handle_ = nullptr;
    TaskHandle_t receive_task_handle_ = nullptr;
    QueueHandle_t event_queue_handle_;
    EventGroupHandle_t event_group_handle_;
    
    std::string rx_buffer_;
    
    // 回调函数
    // std::list<UrcCallback> urc_callbacks_;
    
    // 内部方法
    void EventTask();
    void ReceiveTask();

};

#endif // UART_DEVICE_H
