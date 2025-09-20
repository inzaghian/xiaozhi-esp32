#include "mqtt_private.h"
#include "board.h"
#include "application.h"
#include "settings.h"

#include <esp_log.h>
#include <cstring>
#include <arpa/inet.h>
#include "assets/lang_config.h"
#include <esp_console.h>
#include "argtable3/argtable3.h"
#define TAG "MQTT_PRIVATE"

typedef struct {
    struct arg_str *str;
    struct arg_end *end;
} publish_args_t;

static publish_args_t publish_args;

MqttPrivate::MqttPrivate() {
    // Initialize reconnect timer
    esp_timer_create_args_t reconnect_timer_args = {
        .callback = [](void* arg) {
            MqttPrivate* mqtt = (MqttPrivate*)arg;
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateIdle) {
                ESP_LOGI(TAG, "Reconnecting to private MQTT server");
                app.Schedule([mqtt]() {
                    mqtt->Start();
                });
            }
        },
        .arg = this,
    };
    esp_timer_create(&reconnect_timer_args, &reconnect_timer_);

    publish_args.str = arg_str0(NULL, NULL, "<string>", "a string value");  // 可选字符串参数
    publish_args.end = arg_end(2);                                          // 参数列表结束, 最多允许有 2 个额外的未解
    const esp_console_cmd_t cmd1 = {
        .command = "publish",
        .help = "publish message",
        .hint = nullptr,
        .argtable = &publish_args,
        .func_w_context = [](void *context,int argc, char** argv) -> int {
            int nerrors = arg_parse(argc, argv, (void **) &publish_args);
            if (nerrors != 0) {
                arg_print_errors(stderr, publish_args.end, argv[0]);
                return 1;
            }
            MqttPrivate *pctx = (MqttPrivate *)context;
            if(pctx->mqtt_) {
                pctx->mqtt_->Publish(pctx->publish_topic_, std::string(publish_args.str->sval[0]));
            }
            return 0;
        },
        .context = this
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd1));
}

MqttPrivate::~MqttPrivate() {
    ESP_LOGI(TAG, "MqttProtocol deinit");
    if (reconnect_timer_ != nullptr) {
        esp_timer_stop(reconnect_timer_);
        esp_timer_delete(reconnect_timer_);
    }
    mqtt_.reset();
}

bool MqttPrivate::Start() {
    if (mqtt_ != nullptr) {
        ESP_LOGW(TAG, "Mqtt client already started");
        mqtt_.reset();
    }

    auto endpoint = std::string("101.43.187.112:1883");
    auto client_id = std::string("esp32s3");
    auto username = std::string("xiaozhi");
    auto password = std::string("123456");
    int keepalive_interval = 240;
    publish_topic_ = std::string("test/mqtt_topic");

    auto network = Board::GetInstance().GetNetwork();
    mqtt_ = network->CreateMqtt(0);
    mqtt_->SetKeepAlive(keepalive_interval);

    mqtt_->OnDisconnected([this]() {
        ESP_LOGI(TAG, "MQTT disconnected, schedule reconnect in %d seconds", MQTT_RECONNECT_INTERVAL_MS / 1000);
        esp_timer_start_once(reconnect_timer_, MQTT_RECONNECT_INTERVAL_MS * 1000);
    });

    mqtt_->OnConnected([this]() {
        esp_timer_stop(reconnect_timer_);
        mqtt_->Subscribe(publish_topic_);
        ESP_LOGI(TAG,"mqtt 2 connected");
    });

    mqtt_->OnMessage([this](const std::string& topic, const std::string& payload) {
        ESP_LOGI(TAG,"%s:%s",topic.c_str(), payload.c_str());
    });

    ESP_LOGI(TAG, "Connecting to endpoint %s", endpoint.c_str());
    std::string broker_address;
    int broker_port = 8883;
    size_t pos = endpoint.find(':');
    if (pos != std::string::npos) {
        broker_address = endpoint.substr(0, pos);
        broker_port = std::stoi(endpoint.substr(pos + 1));
    } else {
        broker_address = endpoint;
    }
    if (!mqtt_->Connect(broker_address, broker_port, client_id, username, password)) {
        ESP_LOGE(TAG, "Failed to connect to endpoint");
        return false;
    }

    ESP_LOGI(TAG, "Connected to endpoint 2");
    return true;
}

