#ifndef MQTT_PRIVATE_H
#define MQTT_PRIVATE_H

#include <mqtt.h>
#include <cJSON.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <esp_timer.h>

#include <functional>
#include <string>
#include <map>
#include <mutex>
#include <chrono>

#define MQTT_RECONNECT_INTERVAL_MS 60000
#define MQTT_UP_STREAM_TOPIC    "feishu/upstream"
#define MQTT_DOWN_STREAM_TOPIC    "feishu/downstream"


class MqttPrivate {
public:
    MqttPrivate();
    ~MqttPrivate();

    bool Start() ;
    void publish(std::string data);
    bool is_remote_wakeup;
    std::string chat_id_str;
private:
    std::string publish_topic_;
    std::unique_ptr<Mqtt> mqtt_;
    esp_timer_handle_t reconnect_timer_;
};


#endif // MQTT_PROTOCOL_H
