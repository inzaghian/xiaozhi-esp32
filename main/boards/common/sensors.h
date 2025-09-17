#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>
#include <esp_err.h>
#include <string>

#ifdef __cplusplus
extern "C" {
#endif

class Sensor{
public:
    Sensor(){}
    ~Sensor(){}
    virtual esp_err_t init(void *data) { return ESP_OK; }
    virtual esp_err_t data_update(void){ return ESP_OK; }
    virtual std::string data_json_get(){ return ""; }
};


#ifdef __cplusplus
}
#endif
#endif // SENSOR_BROKER_H
