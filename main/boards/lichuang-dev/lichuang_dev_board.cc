#include "wifi_board.h"
#include "codecs/box_audio_codec.h"
#include "display/lcd_display.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "i2c_device.h"
#include "esp32_camera.h"

#include <esp_log.h>
#include <esp_lcd_panel_vendor.h>
#include <driver/i2c_master.h>
#include <driver/spi_common.h>
#include <wifi_station.h>
#include <esp_lcd_touch_ft5x06.h>
#include <esp_lvgl_port.h>
#include <lvgl.h>
#include "qmi8658.h"
#include "scd4x.h"
#include "sensor_broker.h"
#include <esp_console.h>
#include "uart_device.h"

#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "led/circular_strip.h"

#define TAG "LichuangDevBoard"

#define MOUNT_POINT              "/sdcard"

class Pca9557 : public I2cDevice {
public:
    Pca9557(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr) {
        WriteReg(0x01, 0x03);
        WriteReg(0x03, 0xf8);
    }

    void SetOutputState(uint8_t bit, uint8_t level) {
        uint8_t data = ReadReg(0x01);
        data = (data & ~(1 << bit)) | (level << bit);
        WriteReg(0x01, data);
    }
};

class CustomAudioCodec : public BoxAudioCodec {
private:
    Pca9557* pca9557_;

public:
    CustomAudioCodec(i2c_master_bus_handle_t i2c_bus, Pca9557* pca9557) 
        : BoxAudioCodec(i2c_bus, 
                       AUDIO_INPUT_SAMPLE_RATE, 
                       AUDIO_OUTPUT_SAMPLE_RATE,
                       AUDIO_I2S_GPIO_MCLK, 
                       AUDIO_I2S_GPIO_BCLK, 
                       AUDIO_I2S_GPIO_WS, 
                       AUDIO_I2S_GPIO_DOUT, 
                       AUDIO_I2S_GPIO_DIN,
                       GPIO_NUM_NC, 
                       AUDIO_CODEC_ES8311_ADDR, 
                       AUDIO_CODEC_ES7210_ADDR, 
                       AUDIO_INPUT_REFERENCE),
          pca9557_(pca9557) {
    }

    virtual void EnableOutput(bool enable) override {
        BoxAudioCodec::EnableOutput(enable);
        if (enable) {
            pca9557_->SetOutputState(1, 1);
        } else {
            pca9557_->SetOutputState(1, 0);
        }
    }
};
static StripColor RGBToColor(int red, int green, int blue) {
    return {static_cast<uint8_t>(red), static_cast<uint8_t>(green), static_cast<uint8_t>(blue)};
}
class LichuangDevBoard : public WifiBoard {
private:
    i2c_master_bus_handle_t i2c_bus_;
    i2c_master_dev_handle_t pca9557_handle_;
    Button boot_button_;
    LcdDisplay* display_;
    Pca9557* pca9557_;
    Esp32Camera* camera_;
    Qmi8658* qmi8658_;
    Scd4x* scd4x_;
    SensorBroker* sb_;
    UartDevice* uart_;
    CircularStrip* led_strip_;

    void InitializeI2c() {
        // Initialize I2C peripheral
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = (i2c_port_t)1,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));

        // Initialize PCA9557
        pca9557_ = new Pca9557(i2c_bus_, 0x19);

        qmi8658_ = new Qmi8658(i2c_bus_, 0x6A);
        qmi8658c_config_t config =
        {
            .mode = QMI8658C_MODE_DUAL,
            .acc_scale = QMI8658C_ACC_SCALE_16G,
            .acc_odr = QMI8658C_ACC_ODR_125,
            .gyro_scale = QMI8658C_GYRO_SCALE_2048DPS,
            .gyro_odr = QMI8658C_GYRO_ODR_125,
        };
        qmi8658_->setup(&config);
        
        scd4x_ = new Scd4x(i2c_bus_, 0x62);
        uint16_t s0,s1,s2=0;
        scd4x_->wake_up();
        scd4x_->stop_periodic_measurement();
        scd4x_->reinit();
        scd4x_->get_serial_number(&s0, &s1, &s2);
        ESP_LOGI(TAG,"s0=0x%04x,s1=0x%04x,s2=0x%04x",s0,s1,s2);
        // scd4x_->start_periodic_measurement();
        scd4x_->measure_single_shot();
        sb_ = new SensorBroker();
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = GPIO_NUM_40;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = GPIO_NUM_41;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
                ResetWifiConfiguration();
            }
            app.ToggleChatState();
        });

#if CONFIG_USE_DEVICE_AEC
        boot_button_.OnDoubleClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateIdle) {
                app.SetAecMode(app.GetAecMode() == kAecOff ? kAecOnDeviceSide : kAecOff);
            }
        });
#endif
    }

    void InitializeSt7789Display() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;
        // 液晶屏控制IO初始化
        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = GPIO_NUM_NC;
        io_config.dc_gpio_num = GPIO_NUM_39;
        io_config.spi_mode = 2;
        io_config.pclk_hz = 80 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        // 初始化液晶屏驱动芯片ST7789
        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GPIO_NUM_NC;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
        panel_config.bits_per_pixel = 16;
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));
        
        esp_lcd_panel_reset(panel);
        pca9557_->SetOutputState(0, 0);

        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, true);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
        display_ = new SpiLcdDisplay(panel_io, panel,
                                    DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }

    void InitializeTouch()
    {
        esp_lcd_touch_handle_t tp;
        esp_lcd_touch_config_t tp_cfg = {
            .x_max = DISPLAY_WIDTH,
            .y_max = DISPLAY_HEIGHT,
            .rst_gpio_num = GPIO_NUM_NC, // Shared with LCD reset
            .int_gpio_num = GPIO_NUM_NC, 
            .levels = {
                .reset = 0,
                .interrupt = 0,
            },
            .flags = {
                .swap_xy = 1,
                .mirror_x = 1,
                .mirror_y = 0,
            },
        };
        esp_lcd_panel_io_handle_t tp_io_handle = NULL;
        esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_FT5x06_CONFIG();
        tp_io_config.scl_speed_hz = 400000;

        esp_lcd_new_panel_io_i2c(i2c_bus_, &tp_io_config, &tp_io_handle);
        esp_lcd_touch_new_i2c_ft5x06(tp_io_handle, &tp_cfg, &tp);
        assert(tp);

        /* Add touch input (for selected screen) */
        const lvgl_port_touch_cfg_t touch_cfg = {
            .disp = lv_display_get_default(), 
            .handle = tp,
        };

        lvgl_port_add_touch(&touch_cfg);
    }

    void InitializeCamera() {
        // Open camera power
        pca9557_->SetOutputState(2, 0);

        camera_config_t config = {};
        config.ledc_channel = LEDC_CHANNEL_2;  // LEDC通道选择  用于生成XCLK时钟 但是S3不用
        config.ledc_timer = LEDC_TIMER_2; // LEDC timer选择  用于生成XCLK时钟 但是S3不用
        config.pin_d0 = CAMERA_PIN_D0;
        config.pin_d1 = CAMERA_PIN_D1;
        config.pin_d2 = CAMERA_PIN_D2;
        config.pin_d3 = CAMERA_PIN_D3;
        config.pin_d4 = CAMERA_PIN_D4;
        config.pin_d5 = CAMERA_PIN_D5;
        config.pin_d6 = CAMERA_PIN_D6;
        config.pin_d7 = CAMERA_PIN_D7;
        config.pin_xclk = CAMERA_PIN_XCLK;
        config.pin_pclk = CAMERA_PIN_PCLK;
        config.pin_vsync = CAMERA_PIN_VSYNC;
        config.pin_href = CAMERA_PIN_HREF;
        config.pin_sccb_sda = -1;   // 这里写-1 表示使用已经初始化的I2C接口
        config.pin_sccb_scl = CAMERA_PIN_SIOC;
        config.sccb_i2c_port = 1;
        config.pin_pwdn = CAMERA_PIN_PWDN;
        config.pin_reset = CAMERA_PIN_RESET;
        config.xclk_freq_hz = XCLK_FREQ_HZ;
        config.pixel_format = PIXFORMAT_RGB565;
        config.frame_size = FRAMESIZE_VGA;
        config.jpeg_quality = 12;
        config.fb_count = 1;
        config.fb_location = CAMERA_FB_IN_PSRAM;
        config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

        camera_ = new Esp32Camera(config);
    }    
    void InitializeUart() {
        uart_ = new UartDevice(UART_TX_PIN, UART_RX_PIN, UART_DTR_PIN, 9600);
        uart_->Initialize();
    }

    void InitializeSdcard() {
        esp_err_t ret;
        esp_vfs_fat_sdmmc_mount_config_t mount_config = {
            .format_if_mount_failed = true,   // 如果挂载不成功是否需要格式化SD卡
            .max_files = 5, // 允许打开的最大文件数
            .allocation_unit_size = 16 * 1024  // 分配单元大小
        };
        
        sdmmc_card_t *card;
        const char mount_point[] = MOUNT_POINT;
        ESP_LOGI(TAG, "Initializing SD card");
        ESP_LOGI(TAG, "Using SDMMC peripheral");
  
        sdmmc_host_t host = SDMMC_HOST_DEFAULT(); // SDMMC主机接口配置
        sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT(); // SDMMC插槽配置
        slot_config.width = 1;  // 设置为1线SD模式
        slot_config.clk = SD_CLK_PIN; 
        slot_config.cmd = SD_CMD_PIN;
        slot_config.d0 = SD_D0_PIN;
        slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP; // 打开内部上拉电阻

        ESP_LOGI(TAG, "Mounting filesystem");
        ret = esp_vfs_fat_sdmmc_mount(mount_point, &host, &slot_config, &mount_config, &card); // 挂载SD卡

        if (ret != ESP_OK) {  // 如果没有挂载成功
            if (ret == ESP_FAIL) { // 如果挂载失败
                ESP_LOGE(TAG, "Failed to mount filesystem. ");
            } else { // 如果是其它错误 打印错误名称
                ESP_LOGE(TAG, "Failed to initialize the card (%s). ", esp_err_to_name(ret));
            }
            return;
        }
        ESP_LOGI(TAG, "Filesystem mounted"); // 提示挂载成功
        sdmmc_card_print_info(stdout, card); // 终端打印SD卡的一些信息
        // /*test file system*/
        // // 测试创建文件
        // FILE *f = fopen("/sdcard/test.txt", "w");
        // if (f == NULL) {
        //     ESP_LOGE(TAG, "Failed to create file");
        //     return;
        // }
        // fprintf(f, "Hello, World!");
        // fclose(f);
        // ESP_LOGI(TAG, "File created successfully");
        
        // // 测试读取文件
        // f = fopen("/sdcard/test.txt", "r");
        // if (f == NULL) {
        //     ESP_LOGE(TAG, "Failed to open file for reading");
        //     return;
        // }
        // char buffer[100];
        // fgets(buffer, sizeof(buffer), f);
        // fclose(f);
        // ESP_LOGI(TAG, "File content: %s", buffer);
    }
    void InitializeCmd() {
        esp_console_repl_t *repl = NULL;
        esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
        repl_config.max_cmdline_length = 1024;
        repl_config.prompt = "xiaozhi>";
        
        const esp_console_cmd_t cmd1 = {
            .command = "reboot",
            .help = "reboot the device",
            .hint = nullptr,
            .func = [](int argc, char** argv) -> int {
                esp_restart();
                return 0;
            },
            .argtable = nullptr
        };
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmd1));

        const esp_console_cmd_t cmd2 = {
            .command = "tasklist",
            .help = "show task list",
            .hint = nullptr,
            .func = [](int argc, char** argv) -> int {
                char *buf = (char *)malloc(2048);
                printf("--------------- heap free:%ldKB ---------------------\r\n", esp_get_free_heap_size()/1024);
                if(buf) {
                    vTaskList(buf);
                    printf("----------------------------------------------------\r\n");
                    printf("task_name   status   priority   stack   num   core\r\n");
                    printf("%s", buf);
                    printf("----------------------------------------------------\r\n");
                    free(buf);
                }
                else {
                    printf("no memory\r\n");
                }

                return 0;
            },
            .argtable = nullptr
        };
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmd2));

        const esp_console_cmd_t cmd3 = {
            .command = "cpu",
            .help = "show cpu usage",
            .hint = nullptr,
            .func = [](int argc, char** argv) -> int {
                char *buf = (char *)malloc(2048);
                printf("--------------- heap free:%ldKB ---------------------\r\n", esp_get_free_heap_size()/1024);
                if(buf) {
                    vTaskGetRunTimeStats(buf);
                    printf("task_name      run_cnt                 usage_rate   \r\n");
                    printf("%s", buf);
                    printf("----------------------------------------------------\r\n");
                    free(buf);
                }
                else {
                    printf("no memory\r\n");
                }
                return 0;
            },
            .argtable = nullptr
        };
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmd3));

        esp_console_dev_uart_config_t hw_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_console_new_repl_uart(&hw_config, &repl_config, &repl));
        ESP_ERROR_CHECK(esp_console_start_repl(repl));
    }
    const StripColor heart_8x8[8*8] = {
        // 第1行（顶部深红）
        {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0},

        {0,0,0}, {255,0,0}, {255,0,0}, {0,0,0}, {0,0,0}, {255,0,0}, {255,0,0}, {0,0,0},

        {255,36,36}, {255,36,36}, {255,36,36}, {255,36,36}, {255,36,36}, {255,36,36}, {255,36,36}, {255,36,36},

        {255,73,73}, {255,73,73}, {255,73,73}, {255,73,73}, {255,73,73}, {255,73,73}, {255,73,73}, {255,73,73},

        {0,0,0}, {255,109,109}, {255,109,109}, {255,109,109}, {255,109,109}, {255,109,109}, {255,109,109}, {0,0,0},

        {0,0,0}, {0,0,0}, {255,146,146}, {255,146,146}, {255,146,146}, {255,146,146}, {0,0,0}, {0,0,0},

        {0,0,0}, {0,0,0}, {0,0,0}, {255,182,182}, {255,182,182}, {0,0,0}, {0,0,0}, {0,0,0},

        {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}
    };
    const StripColor mario_8x8[8 * 8] = {
        // 第1行 (y=0)
        {0,0,0}, {0,0,0}, {255,0,0}, {255,0,0}, {255,0,0}, {255,0,0}, {0,0,0}, {0,0,0},
        
        // 第2行 (y=1)
        {0,0,0}, {255,0,0}, {255,0,0}, {255,0,0}, {255,0,0}, {255,0,0}, {255,0,0}, {0,0,0},
        
        // 第3行 (y=2)
        {0,0,0}, {139,69,19}, {139,69,19}, {255,192,203}, {139,69,19}, {255,192,203}, {0,0,0}, {0,0,0},
        
        // 第4行 (y=3)
        {0,0,0}, {139,69,19}, {255,192,203}, {255,192,203}, {255,192,203}, {139,69,19}, {255,192,203}, {0,0,0},
        
        // 第5行 (y=4)
        {0,0,0}, {139,69,19}, {255,192,203}, {255,192,203}, {255,192,203}, {139,69,19}, {255,192,203}, {0,0,0},
        
        // 第6行 (y=5)
        {0,0,0}, {0,0,0}, {255,192,203}, {255,192,203}, {255,192,203}, {255,192,203}, {0,0,0}, {0,0,0},
        
        // 第7行 (y=6)
        {0,0,0}, {0,0,128}, {0,0,128}, {0,0,0}, {0,0,128}, {0,0,128}, {0,0,0}, {0,0,0},
        
        // 第8行 (y=7)
        {0,0,128}, {0,0,128}, {0,0,128}, {0,0,128}, {0,0,128}, {0,0,128}, {0,0,128}, {0,0,128}
    };
    const StripColor smiley_8x8[8 * 8] = {
    // 第1行 (y=0)
    {173,216,230}, {255,255,0}, {255,255,0}, {255,255,0}, {255,255,0}, {255,255,0}, {255,255,0}, {173,216,230},
    
    // 第2行 (y=1)
    {255,255,0}, {173,216,230}, {173,216,230}, {173,216,230}, {173,216,230}, {173,216,230}, {173,216,230}, {255,255,0},
    
    // 第3行 (y=2)
    {255,255,0}, {173,216,230}, {255,0,0}, {173,216,230}, {173,216,230}, {255,0,0}, {173,216,230}, {255,255,0},
    
    // 第4行 (y=3)
    {255,255,0}, {173,216,230}, {173,216,230}, {173,216,230}, {173,216,230}, {173,216,230}, {173,216,230}, {255,255,0},
    
    // 第5行 (y=4)
    {255,255,0}, {173,216,230}, {255,0,0}, {173,216,230}, {173,216,230}, {255,0,0}, {173,216,230}, {255,255,0},
    
    // 第6行 (y=5)
    {255,255,0}, {173,216,230}, {173,216,230}, {255,0,0}, {255,0,0}, {173,216,230}, {173,216,230}, {255,255,0},
    
    // 第7行 (y=6)
    {255,255,0}, {173,216,230}, {173,216,230}, {173,216,230}, {173,216,230}, {173,216,230}, {173,216,230}, {255,255,0},
    
    // 第8行 (y=7)
    {173,216,230}, {255,255,0}, {255,255,0}, {255,255,0}, {255,255,0}, {255,255,0}, {255,255,0}, {173,216,230}
};
const StripColor number8_8x8[8 * 8] = {
    // 第1行 (y=0)
    {255,255,153}, {255,255,255}, {255,255,255}, {255,255,255}, {255,255,255}, {255,255,255}, {255,255,255}, {255,255,153},
    
    // 第2行 (y=1)
    {255,255,255}, {255,255,153}, {255,255,153}, {255,255,153}, {255,255,153}, {255,255,153}, {255,255,153}, {255,255,255},
    
    // 第3行 (y=2)
    {255,255,255}, {255,255,153}, {255,255,153}, {255,255,153}, {255,255,153}, {255,255,153}, {255,255,153}, {255,255,255},
    
    // 第4行 (y=3)
    {255,255,153}, {255,255,255}, {255,255,255}, {255,255,255}, {255,255,255}, {255,255,255}, {255,255,255}, {255,255,153},
    
    // 第5行 (y=4)
    {255,255,255}, {255,255,153}, {255,255,153}, {255,255,153}, {255,255,153}, {255,255,153}, {255,255,153}, {255,255,255},
    
    // 第6行 (y=5)
    {255,255,255}, {255,255,153}, {255,255,153}, {255,255,153}, {255,255,153}, {255,255,153}, {255,255,153}, {255,255,255},
    
    // 第7行 (y=6)
    {255,255,153}, {255,255,255}, {255,255,255}, {255,255,255}, {255,255,255}, {255,255,255}, {255,255,255}, {255,255,153},
    
    // 第8行 (y=7)
    {255,255,153}, {255,255,153}, {255,255,153}, {255,255,153}, {255,255,153}, {255,255,153}, {255,255,153}, {255,255,153}
};
    void IntializeLedStrip() {
        // Initialize LED
        led_strip_ = new CircularStrip(STRIP_LED_PIN, STRIP_LED_NUM);
        led_strip_->SetBrightness(20,20);
        led_strip_->SetAllColor(RGBToColor(5, 5, 5));
        // led_strip_->Blink(RGBToColor(123, 4, 10), 1000);
        // led_strip_->Breathe(RGBToColor(0, 0, 0),RGBToColor(0, 0, 50), 10);
        led_strip_->Scroll(RGBToColor(59, 29, 54), RGBToColor(204, 141, 23), 2, 333);   // rgba(59, 29, 54, 1)   rrgba(204, 141, 23, 1)
        // led_strip_->ShowImage(heart_8x8);
        // led_strip_->ShowImage(number8_8x8);
    }
public:
    LichuangDevBoard() : boot_button_(BOOT_BUTTON_GPIO) {
        InitializeI2c();
        InitializeSpi();
        InitializeSt7789Display();
        InitializeTouch();
        InitializeButtons();
        InitializeCamera();
        InitializeCmd();
        // InitializeUart();
        InitializeSdcard();
        IntializeLedStrip();
        GetBacklight()->RestoreBrightness();
    }

    virtual AudioCodec* GetAudioCodec() override {
        static CustomAudioCodec audio_codec(
            i2c_bus_, 
            pca9557_);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }
    
    virtual Backlight* GetBacklight() override {
        static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        return &backlight;
    }

    virtual Camera* GetCamera() override {
        return camera_;
    }
    virtual Sensor* GetImu() override {
        return qmi8658_;
    }
    virtual Sensor* GetEnvSensor() override {
        return scd4x_;
    }
    
};

DECLARE_BOARD(LichuangDevBoard);
