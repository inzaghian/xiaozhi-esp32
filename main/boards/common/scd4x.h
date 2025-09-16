#if 1
#ifndef SCD4X_H
#define SCD4X_H

#include <stdint.h>
#include <esp_err.h>
#include "i2c_device.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DEFAUL_SCD4X_ADDR 0x62 //!< I2C address for QMI8658C
#define SCD4X_I2C_FREQ_HZ 400000



class Scd4x : public I2cDevice {
public:
    Scd4x(i2c_master_bus_handle_t i2c_bus, uint8_t addr);
    esp_err_t start_periodic_measurement(void);
    esp_err_t read_measurement_ticks(uint16_t *co2, uint16_t *temperature, uint16_t *humidity);
    esp_err_t read_measurement(uint16_t *co2, float *temperature, float *humidity);

    esp_err_t stop_periodic_measurement(void);
    esp_err_t get_temperature_offset_ticks(uint16_t *t_offset);
    esp_err_t get_temperature_offset(float *t_offset);
    esp_err_t set_temperature_offset_ticks(uint16_t t_offset);
    esp_err_t set_temperature_offset(float t_offset);
    esp_err_t get_sensor_altitude(uint16_t *altitude);
    esp_err_t set_sensor_altitude(uint16_t altitude);
    esp_err_t set_ambient_pressure(uint16_t pressure);
    esp_err_t perform_forced_recalibration(uint16_t target_co2_concentration,
                                         uint16_t *frc_correction);
    esp_err_t get_automatic_self_calibration(bool *enabled);
    esp_err_t set_automatic_self_calibration(bool enabled);
    esp_err_t start_low_power_periodic_measurement(void);
    esp_err_t get_data_ready_status(bool *data_ready);
    esp_err_t persist_settings(void);
    esp_err_t get_serial_number(uint16_t *serial0, uint16_t *serial1, uint16_t *serial2);
    esp_err_t perform_self_test(bool *malfunction);
    esp_err_t perform_factory_reset(void);
    esp_err_t reinit(void);
    esp_err_t measure_single_shot(void);
    esp_err_t measure_single_shot_rht_only(void);
    esp_err_t power_down(void);
    esp_err_t wake_up(void);
private:
    esp_err_t send_cmd(uint16_t cmd, uint16_t *data, size_t words);
    esp_err_t read_resp(uint16_t *data, size_t words);
    esp_err_t execute_cmd(uint16_t cmd, uint32_t timeout_ms,
                            uint16_t *out_data, size_t out_words, uint16_t *in_data, size_t in_words);
};

#ifdef __cplusplus
}
#endif
#endif // SCD4X_H
#endif
