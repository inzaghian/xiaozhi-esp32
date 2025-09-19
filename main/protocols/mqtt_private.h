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


class MqttPrivate {
public:
    MqttPrivate();
    ~MqttPrivate();

    bool Start() ;
private:
    std::string publish_topic_;
    std::unique_ptr<Mqtt> mqtt_;
    esp_timer_handle_t reconnect_timer_;
};


#endif // MQTT_PROTOCOL_H
