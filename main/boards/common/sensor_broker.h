#ifndef SENSOR_BROKER_H
#define SENSOR_BROKER_H

#include <stdint.h>
#include <esp_err.h>
#include "qmi8658.h"
#include "scd4x.h"
#include "mcp_server.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CO2_THRESHOLD_IN_PPM    1500U

enum EnvState{
    ENV_GOOD = 0,
    ENV_BAD
};

class SensorBroker {
public:
    SensorBroker();
    void SensorBrokerTask();
private:
    EnvState state;
    uint32_t cnt;
    uint8_t co2_cnt_good;
    uint8_t co2_cnt_bad;
    
};

#ifdef __cplusplus
}
#endif
#endif // SENSOR_BROKER_H
