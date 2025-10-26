#include "M5UnitSynth.h"
#include <esp_log.h>
#include "melody.h"
#include <esp_console.h>
#include "argtable3/argtable3.h"
#define TAG "M5UnitSynth"



typedef struct {
    struct arg_str *str;
    struct arg_end *end;
} cmd_args_t;

static cmd_args_t cmd_args;


M5UnitSynth::M5UnitSynth(gpio_num_t tx_pin): UartDevice(tx_pin, GPIO_NUM_NC, GPIO_NUM_NC, UNIT_SYNTH_BAUD)
{
    Initialize();
    cmd_args.str = arg_str0(NULL, NULL, "<string>", "a string value");  // 可选字符串参数
    cmd_args.end = arg_end(2);                                          // 参数列表结束, 最多允许有 2 个额外的未解
    const esp_console_cmd_t cmd1 = {
        .command = "set_instrument",
        .help = "set_instrument",
        .hint = nullptr,
        .argtable = &cmd_args,
        .func_w_context = [](void *context,int argc, char** argv) -> int {
            int nerrors = arg_parse(argc, argv, (void **) &cmd_args);
            if (nerrors != 0) {
                arg_print_errors(stderr, cmd_args.end, argv[0]);
                return 1;
            }
            auto _self = (M5UnitSynth *)context;
            int num = 0;
            sscanf(cmd_args.str->sval[0], "%d", &num);
            ESP_LOGI(TAG, "set_instrument: %s,%d", cmd_args.str->sval[0], num);
            if (num < 0 || num > 127) {
                ESP_LOGE(TAG, "set_instrument: %s,%d", cmd_args.str->sval[0], num);
                return 1;
            }
            _self->setAllNotesOff(0);
            _self->setInstrument(0, 0, (uint8_t)num);
            return 0;
        },
        .context = this
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd1));
    const esp_console_cmd_t cmd2 = {
        .command = "set_volume",
        .help = "set_volume",
        .hint = nullptr,
        .argtable = &cmd_args,
        .func_w_context = [](void *context,int argc, char** argv) -> int {
            int nerrors = arg_parse(argc, argv, (void **) &cmd_args);
            if (nerrors != 0) {
                arg_print_errors(stderr, cmd_args.end, argv[0]);
                return 1;
            }
            auto _self = (M5UnitSynth *)context;
            
            int num = 0;
            sscanf(cmd_args.str->sval[0], "%d", &num);
            ESP_LOGI(TAG, "set_volume: %s,%d", cmd_args.str->sval[0], num);
            if (num < 0 || num > 127) { 
                ESP_LOGE(TAG, "set_volume: %s,%d", cmd_args.str->sval[0], num);
                return 1;
            }
            _self->setVolume(0, (uint8_t)num);
            return 0;
        },
        .context = this
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd2));

    const esp_console_cmd_t cmd3 = {
        .command = "play",
        .help = "midi play melody",
        .hint = nullptr,
        .argtable = nullptr,
        .func_w_context = [](void *context,int argc, char** argv) -> int {
            auto self = (M5UnitSynth *)context;
            xTaskCreate([](void* arg) {
                auto _self = (M5UnitSynth*)arg;
                _self->MidiTask();
                vTaskDelete(NULL);
            }, "play task", 2048, self, configMAX_PRIORITIES - 1, &self->event_task_handle_);
            return 0;
        },
        .context = this
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd3));

    const esp_console_cmd_t cmd4 = {
        .command = "stop",
        .help = "midi stop melody",
        .hint = nullptr,
        .argtable = nullptr,
        .func_w_context = [](void *context,int argc, char** argv) -> int {
            auto self = (M5UnitSynth *)context;
            if(self->event_task_handle_) {
                vTaskDelete(self->event_task_handle_);
                self->event_task_handle_ = nullptr;
                self->setAllNotesOff(0);
            }
            return 0;
        },
        .context = this
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd4));
}



void M5UnitSynth::play(char data) {
    if(data == 0x0d)
        setNoteOff(0, data, 127);
    else
        setNoteOn(0, data, 63);
}

void M5UnitSynth::MidiTask(void) {
    int tempo = 80;
    // sizeof gives the number of bytes, each int value is composed of two bytes (16
    // bits) there are two values per note (pitch and duration), so for each note
    // there are four bytes
    int notes = sizeof(melody) / sizeof(melody[0]) / 2;

    // this calculates the duration of a whole note in ms
    int wholenote = (60000 * 4) / tempo;

    int divider = 0, noteDuration = 0;
    // iterate over the notes of the melody.
    // setInstrument(0, 0, MusicBox);
    // setVolume(0, 32);
    // Remember, the array is twice the number of notes (notes + durations)
    for (int thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2) {
        // calculates the duration of each note
        divider = melody[thisNote + 1];
        if (divider > 0) {
            // regular note, just proceed
            noteDuration = (wholenote) / divider;
        } else if (divider < 0) {
            // dotted notes are represented with negative durations!!
            noteDuration = (wholenote) / abs(divider);
            noteDuration *=
                1.5;  // increases the duration in half for dotted notes
        }

        // we only play the note for 90% of the duration, leaving 10% as a pause
        setNoteOn(0, melody[thisNote], 127);  // noteDuration * 0.9);
        // Wait for the specief duration before playing the next note.
        vTaskDelay(pdMS_TO_TICKS(noteDuration));
        // stop the waveform generation before the next note.
        // setNoteOff(0, melody[thisNote], 127);
    }
}

void M5UnitSynth::sendCMD(uint8_t *buffer, size_t size) {
    SendData((const char *)buffer, size);
}

void M5UnitSynth::setInstrument(uint8_t bank, uint8_t channel, uint8_t value) {
    uint8_t CMD_CONTROL_CHANGE_1[] = {
        (uint8_t)(MIDI_CMD_CONTROL_CHANGE | (channel & 0x0f)), 0x00, bank};

    sendCMD(CMD_CONTROL_CHANGE_1, sizeof(CMD_CONTROL_CHANGE_1));

    uint8_t CMD_PROGRAM_CHANGE_2[] = {
        (uint8_t)(MIDI_CMD_PROGRAM_CHANGE | (channel & 0x0f)), value};
    sendCMD(CMD_PROGRAM_CHANGE_2, sizeof(CMD_PROGRAM_CHANGE_2));
}

void M5UnitSynth::setNoteOn(uint8_t channel, uint8_t pitch, uint8_t velocity) {
    uint8_t CMD_NOTE_ON[] = {(uint8_t)(MIDI_CMD_NOTE_ON | (channel & 0x0f)),
                             pitch, velocity};
    sendCMD(CMD_NOTE_ON, sizeof(CMD_NOTE_ON));
}

void M5UnitSynth::setNoteOff(uint8_t channel, uint8_t pitch, uint8_t velocity) {
    uint8_t CMD_NOTE_OFF[] = {(uint8_t)(MIDI_CMD_NOTE_OFF | (channel & 0x0f)),
                              pitch, 0x00};
    sendCMD(CMD_NOTE_OFF, sizeof(CMD_NOTE_OFF));
}
void M5UnitSynth::setAllNotesOff(uint8_t channel) {
    uint8_t CMD_CONTROL_CHANGE[] = {
        (uint8_t)(MIDI_CMD_CONTROL_CHANGE | (channel & 0x0f)), 0x7b, 0x00};
    sendCMD(CMD_CONTROL_CHANGE, sizeof(CMD_CONTROL_CHANGE));
}

void M5UnitSynth::setPitchBend(uint8_t channel, int value) {
    value                    = value * 0x3FFF / 1023;//map(value, 0, 1023, 0, 0x3fff);
    uint8_t CMD_PITCH_BEND[] = {
        (uint8_t)(MIDI_CMD_PITCH_BEND | (channel & 0x0f)),
        (uint8_t)(value & 0xef), (uint8_t)((value >> 7) & 0xff)};
    sendCMD(CMD_PITCH_BEND, sizeof(CMD_PITCH_BEND));
}
void M5UnitSynth::setPitchBendRange(uint8_t channel, uint8_t value) {
    uint8_t CMD_CONTROL_CHANGE[] = {
        (uint8_t)(MIDI_CMD_CONTROL_CHANGE | (channel & 0x0f)),
        0x65,
        0x00,
        0x64,
        0x00,
        0x06,
        (uint8_t)(value & 0x7f)};
    sendCMD(CMD_CONTROL_CHANGE, sizeof(CMD_CONTROL_CHANGE));
}

void M5UnitSynth::setMasterVolume(uint8_t level) {
    uint8_t CMD_SYSTEM_EXCLUSIVE[] = {MIDI_CMD_SYSTEM_EXCLUSIVE,
                                      0x7f,
                                      0x7f,
                                      0x04,
                                      0x01,
                                      0x00,
                                      (uint8_t)(level & 0x7f),
                                      MIDI_CMD_END_OF_SYSEX};
    sendCMD(CMD_SYSTEM_EXCLUSIVE, sizeof(CMD_SYSTEM_EXCLUSIVE));
}
void M5UnitSynth::setVolume(uint8_t channel, uint8_t level) {
    uint8_t CMD_CONTROL_CHANGE[] = {
        (uint8_t)(MIDI_CMD_CONTROL_CHANGE | (channel & 0x0f)), 0x07, level};
    sendCMD(CMD_CONTROL_CHANGE, sizeof(CMD_CONTROL_CHANGE));
}

void M5UnitSynth::setExpression(uint8_t channel, uint8_t expression) {
    uint8_t CMD_CONTROL_CHANGE[] = {
        (uint8_t)(MIDI_CMD_CONTROL_CHANGE | (channel & 0x0f)), 0x0b, expression};
    sendCMD(CMD_CONTROL_CHANGE, sizeof(CMD_CONTROL_CHANGE));
}

void M5UnitSynth::setReverb(uint8_t channel, uint8_t program, uint8_t level,
                            uint8_t delayfeedback) {
    uint8_t CMD_CONTROL_CHANGE_1[] = {
        (uint8_t)(MIDI_CMD_CONTROL_CHANGE | (channel & 0x0f)), 0x50,
        (uint8_t)(program & 0x07)};
    sendCMD(CMD_CONTROL_CHANGE_1, sizeof(CMD_CONTROL_CHANGE_1));

    uint8_t CMD_CONTROL_CHANGE_2[] = {
        (uint8_t)(MIDI_CMD_CONTROL_CHANGE | (channel & 0x0f)), 0x5b,
        (uint8_t)(level & 0x7f)};
    sendCMD(CMD_CONTROL_CHANGE_2, sizeof(CMD_CONTROL_CHANGE_2));

    if (delayfeedback > 0) {
        uint8_t CMD_SYSTEM_EXCLUSIVE[] = {MIDI_CMD_SYSTEM_EXCLUSIVE,
                                          0x41,
                                          0x00,
                                          0x42,
                                          0x12,
                                          0x40,
                                          0x01,
                                          0x35,
                                          (uint8_t)(delayfeedback & 0x7f),
                                          0x00,
                                          MIDI_CMD_END_OF_SYSEX};
        sendCMD(CMD_SYSTEM_EXCLUSIVE, sizeof(CMD_SYSTEM_EXCLUSIVE));
    }
}

void M5UnitSynth::setChorus(uint8_t channel, uint8_t program, uint8_t level,
                            uint8_t feedback, uint8_t chorusdelay) {
    uint8_t CMD_CONTROL_CHANGE_1[] = {
        (uint8_t)(MIDI_CMD_CONTROL_CHANGE | (channel & 0x0f)), 0x51,
        (uint8_t)(program & 0x07)};
    sendCMD(CMD_CONTROL_CHANGE_1, sizeof(CMD_CONTROL_CHANGE_1));

    uint8_t CMD_CONTROL_CHANGE_2[] = {
        (uint8_t)(MIDI_CMD_CONTROL_CHANGE | (channel & 0x0f)), 0x5d,
        (uint8_t)(level & 0x7f)};
    sendCMD(CMD_CONTROL_CHANGE_2, sizeof(CMD_CONTROL_CHANGE_2));

    if (feedback > 0) {
        uint8_t CMD_SYSTEM_EXCLUSIVE_1[] = {MIDI_CMD_SYSTEM_EXCLUSIVE,
                                            0x41,
                                            0x00,
                                            0x42,
                                            0x12,
                                            0x40,
                                            0x01,
                                            0x3b,
                                            (uint8_t)(feedback & 0x7f),
                                            0x00,
                                            MIDI_CMD_END_OF_SYSEX};
        sendCMD(CMD_SYSTEM_EXCLUSIVE_1, sizeof(CMD_SYSTEM_EXCLUSIVE_1));
    }

    if (chorusdelay > 0) {
        uint8_t CMD_SYSTEM_EXCLUSIVE_2[] = {MIDI_CMD_SYSTEM_EXCLUSIVE,
                                            0x41,
                                            0x00,
                                            0x42,
                                            0x12,
                                            0x40,
                                            0x01,
                                            0x3c,
                                            (uint8_t)(feedback & 0x7f),
                                            0x00,
                                            MIDI_CMD_END_OF_SYSEX

        };
        sendCMD(CMD_SYSTEM_EXCLUSIVE_2, sizeof(CMD_SYSTEM_EXCLUSIVE_2));
    }
}

void M5UnitSynth::setPan(uint8_t channel, uint8_t value) {
    uint8_t CMD_CONTROL_CHANGE[] = {
        (uint8_t)(MIDI_CMD_CONTROL_CHANGE | (channel & 0x0f)), 0x0A, value};
    sendCMD(CMD_CONTROL_CHANGE, sizeof(CMD_CONTROL_CHANGE));
}

void M5UnitSynth::setEqualizer(uint8_t channel, uint8_t lowband,
                               uint8_t medlowband, uint8_t medhighband,
                               uint8_t highband, uint8_t lowfreq,
                               uint8_t medlowfreq, uint8_t medhighfreq,
                               uint8_t highfreq) {
    uint8_t CMD_CONTROL_CHANGE[] = {
        (uint8_t)(MIDI_CMD_CONTROL_CHANGE | (channel & 0x0f)),
        0x63,
        0x37,
        0x62,
        0x00,
        0x06,
        (uint8_t)(lowband & 0x7f)};
    sendCMD(CMD_CONTROL_CHANGE, sizeof(CMD_CONTROL_CHANGE));

    CMD_CONTROL_CHANGE[4] = 0x01;
    CMD_CONTROL_CHANGE[6] = (medlowband & 0x7f);

    sendCMD(CMD_CONTROL_CHANGE, sizeof(CMD_CONTROL_CHANGE));

    CMD_CONTROL_CHANGE[4] = 0x02;
    CMD_CONTROL_CHANGE[6] = (medhighband & 0x7f);
    sendCMD(CMD_CONTROL_CHANGE, sizeof(CMD_CONTROL_CHANGE));

    CMD_CONTROL_CHANGE[4] = 0x03;
    CMD_CONTROL_CHANGE[6] = (highband & 0x7f);
    sendCMD(CMD_CONTROL_CHANGE, sizeof(CMD_CONTROL_CHANGE));

    CMD_CONTROL_CHANGE[4] = 0x08;
    CMD_CONTROL_CHANGE[6] = (lowfreq & 0x7f);
    sendCMD(CMD_CONTROL_CHANGE, sizeof(CMD_CONTROL_CHANGE));

    CMD_CONTROL_CHANGE[4] = 0x09;
    CMD_CONTROL_CHANGE[6] = (medlowfreq & 0x7f);
    sendCMD(CMD_CONTROL_CHANGE, sizeof(CMD_CONTROL_CHANGE));

    CMD_CONTROL_CHANGE[4] = 0x0A;
    CMD_CONTROL_CHANGE[6] = (medhighfreq & 0x7f);
    sendCMD(CMD_CONTROL_CHANGE, sizeof(CMD_CONTROL_CHANGE));

    CMD_CONTROL_CHANGE[4] = 0x0B;
    CMD_CONTROL_CHANGE[6] = (highfreq & 0x7f);
    sendCMD(CMD_CONTROL_CHANGE, sizeof(CMD_CONTROL_CHANGE));
}

void M5UnitSynth::setTuning(uint8_t channel, uint8_t fine, uint8_t coarse) {
    uint8_t CMD_CONTROL_CHANGE[] = {
        (uint8_t)(MIDI_CMD_CONTROL_CHANGE | (channel & 0x0f)),
        0x65,
        0x00,
        0x64,
        0x01,
        0x06,
        (uint8_t)(fine & 0x7f)};

    sendCMD(CMD_CONTROL_CHANGE, sizeof(CMD_CONTROL_CHANGE));

    CMD_CONTROL_CHANGE[4] = 0x02;
    CMD_CONTROL_CHANGE[6] = (coarse & 0x7f);

    sendCMD(CMD_CONTROL_CHANGE, sizeof(CMD_CONTROL_CHANGE));
}
void M5UnitSynth::setVibrate(uint8_t channel, uint8_t rate, uint8_t depth,
                             uint8_t delay) {
    uint8_t CMD_CONTROL_CHANGE[] = {
        (uint8_t)(MIDI_CMD_CONTROL_CHANGE | (channel & 0x0f)),
        0x63,
        0x01,
        0x62,
        0x08,
        0x06,
        (uint8_t)(rate & 0x7f)};
    sendCMD(CMD_CONTROL_CHANGE, sizeof(CMD_CONTROL_CHANGE));

    CMD_CONTROL_CHANGE[4] = 0x09;
    CMD_CONTROL_CHANGE[6] = (depth & 0x7f);
    sendCMD(CMD_CONTROL_CHANGE, sizeof(CMD_CONTROL_CHANGE));

    CMD_CONTROL_CHANGE[4] = 0x0A;
    CMD_CONTROL_CHANGE[6] = (delay & 0x7f);
    sendCMD(CMD_CONTROL_CHANGE, sizeof(CMD_CONTROL_CHANGE));
}

void M5UnitSynth::setTvf(uint8_t channel, uint8_t cutoff, uint8_t resonance) {
    uint8_t CMD_CONTROL_CHANGE[] = {
        (uint8_t)(MIDI_CMD_CONTROL_CHANGE | (channel & 0x0f)),
        0x63,
        0x01,
        0x62,
        0x20,
        0x06,
        (uint8_t)(cutoff & 0x7f)};
    sendCMD(CMD_CONTROL_CHANGE, sizeof(CMD_CONTROL_CHANGE));

    CMD_CONTROL_CHANGE[4] = 0x21;
    CMD_CONTROL_CHANGE[6] = (resonance & 0x7f);
    sendCMD(CMD_CONTROL_CHANGE, sizeof(CMD_CONTROL_CHANGE));
}
void M5UnitSynth::setEnvelope(uint8_t channel, uint8_t attack, uint8_t decay,
                              uint8_t release) {
    uint8_t CMD_CONTROL_CHANGE[] = {

        (uint8_t)(MIDI_CMD_CONTROL_CHANGE | (channel & 0x0f)),
        0x63,
        0x01,
        0x62,
        0x63,
        0x06,
        (uint8_t)(attack & 0x7f)};
    sendCMD(CMD_CONTROL_CHANGE, sizeof(CMD_CONTROL_CHANGE));

    CMD_CONTROL_CHANGE[4] = 0x64;
    CMD_CONTROL_CHANGE[6] = (decay & 0x7f);
    sendCMD(CMD_CONTROL_CHANGE, sizeof(CMD_CONTROL_CHANGE));

    CMD_CONTROL_CHANGE[4] = 0x66;
    CMD_CONTROL_CHANGE[6] = (release & 0x7f);
    sendCMD(CMD_CONTROL_CHANGE, sizeof(CMD_CONTROL_CHANGE));
}

void M5UnitSynth::setModWheel(uint8_t channel, uint8_t pitch, uint8_t tvtcutoff,
                              uint8_t amplitude, uint8_t rate,
                              uint8_t pitchdepth, uint8_t tvfdepth,
                              uint8_t tvadepth) {
    uint8_t CMD_CONTROL_CHANGE[] = {MIDI_CMD_CONTROL_CHANGE,
                                    0x41,
                                    0x00,
                                    0x42,
                                    0x12,
                                    0x40,
                                    (uint8_t)(0x20 | (channel & 0x0f)),
                                    0x00,
                                    pitch,
                                    0x00,
                                    MIDI_CMD_END_OF_SYSEX};
    sendCMD(CMD_CONTROL_CHANGE, sizeof(CMD_CONTROL_CHANGE));

    CMD_CONTROL_CHANGE[8] = 0x01;
    CMD_CONTROL_CHANGE[9] = tvtcutoff;
    sendCMD(CMD_CONTROL_CHANGE, sizeof(CMD_CONTROL_CHANGE));

    CMD_CONTROL_CHANGE[8] = 0x02;
    CMD_CONTROL_CHANGE[9] = amplitude;
    sendCMD(CMD_CONTROL_CHANGE, sizeof(CMD_CONTROL_CHANGE));

    CMD_CONTROL_CHANGE[8] = 0x03;
    CMD_CONTROL_CHANGE[9] = rate;
    sendCMD(CMD_CONTROL_CHANGE, sizeof(CMD_CONTROL_CHANGE));

    CMD_CONTROL_CHANGE[8] = 0x04;
    CMD_CONTROL_CHANGE[9] = pitchdepth;
    sendCMD(CMD_CONTROL_CHANGE, sizeof(CMD_CONTROL_CHANGE));

    CMD_CONTROL_CHANGE[8] = 0x05;
    CMD_CONTROL_CHANGE[9] = tvfdepth;
    sendCMD(CMD_CONTROL_CHANGE, sizeof(CMD_CONTROL_CHANGE));

    CMD_CONTROL_CHANGE[8] = 0x06;
    CMD_CONTROL_CHANGE[9] = tvadepth;
    sendCMD(CMD_CONTROL_CHANGE, sizeof(CMD_CONTROL_CHANGE));
}

void M5UnitSynth::setAllInstrumentDrums() {
    uint8_t CMD_CONTROL_CHANGE[] = {MIDI_CMD_CONTROL_CHANGE,
                                    0x41,
                                    0x00,
                                    0x42,
                                    0x12,
                                    0x40,
                                    0x10,
                                    0x15,
                                    0x01,
                                    0x00,
                                    MIDI_CMD_END_OF_SYSEX};
    sendCMD(CMD_CONTROL_CHANGE, sizeof(CMD_CONTROL_CHANGE));

    for (uint8_t i = 1; i < 15; i++) {
        CMD_CONTROL_CHANGE[6] = i;
        sendCMD(CMD_CONTROL_CHANGE, sizeof(CMD_CONTROL_CHANGE));
    }
}

void M5UnitSynth::reset() {
    uint8_t CMD_SYSTEM_RESET[] = {MIDI_CMD_SYSTEM_RESET};
    sendCMD(CMD_SYSTEM_RESET, sizeof(CMD_SYSTEM_RESET));
}
