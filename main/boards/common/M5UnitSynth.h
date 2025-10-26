#ifndef _M5_UNIT_SYNTH_H
#define _M5_UNIT_SYNTH_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "uart_device.h"

#define NOTE_B0  23
#define NOTE_C1  24
#define NOTE_CS1 25
#define NOTE_D1  26
#define NOTE_DS1 27
#define NOTE_E1  28
#define NOTE_F1  29
#define NOTE_FS1 30
#define NOTE_G1  31
#define NOTE_GS1 32
#define NOTE_A1  33
#define NOTE_AS1 34
#define NOTE_B1  35
#define NOTE_C2  36
#define NOTE_CS2 37
#define NOTE_D2  38
#define NOTE_DS2 39
#define NOTE_E2  40
#define NOTE_F2  41
#define NOTE_FS2 42
#define NOTE_G2  43
#define NOTE_GS2 44
#define NOTE_A2  45
#define NOTE_AS2 46
#define NOTE_B2  47
#define NOTE_C3  48
#define NOTE_CS3 49
#define NOTE_D3  50
#define NOTE_DS3 51
#define NOTE_E3  52
#define NOTE_F3  53
#define NOTE_FS3 54
#define NOTE_G3  55
#define NOTE_GS3 56
#define NOTE_A3  57
#define NOTE_AS3 58
#define NOTE_B3  59
#define NOTE_C4  60
#define NOTE_CS4 61
#define NOTE_D4  62
#define NOTE_DS4 63
#define NOTE_E4  64
#define NOTE_F4  65
#define NOTE_FS4 66
#define NOTE_G4  67
#define NOTE_GS4 68
#define NOTE_A4  69
#define NOTE_AS4 70
#define NOTE_B4  71
#define NOTE_C5  72
#define NOTE_CS5 73
#define NOTE_D5  74
#define NOTE_DS5 75
#define NOTE_E5  76
#define NOTE_F5  77
#define NOTE_FS5 78
#define NOTE_G5  79
#define NOTE_GS5 80
#define NOTE_A5  81
#define NOTE_AS5 82
#define NOTE_B5  83
#define NOTE_C6  84
#define NOTE_CS6 85
#define NOTE_D6  86
#define NOTE_DS6 87
#define NOTE_E6  88
#define NOTE_F6  89
#define NOTE_FS6 90
#define NOTE_G6  91
#define NOTE_GS6 92
#define NOTE_A6  93
#define NOTE_AS6 94
#define NOTE_B6  95
#define NOTE_C7  96
#define NOTE_CS7 97
#define NOTE_D7  98
#define NOTE_DS7 99
#define NOTE_E7  100
#define NOTE_F7  101
#define NOTE_FS7 102
#define NOTE_G7  103
#define NOTE_GS7 104
#define NOTE_A7  105
#define NOTE_AS7 106
#define NOTE_B7  107
#define NOTE_C8  108
#define NOTE_CS8 109
#define NOTE_D8  110
#define NOTE_DS8 111
#define REST     0

typedef enum {
    GrandPiano_1 = 0,
    BrightPiano_2,
    ElGrdPiano_3,
    HonkyTonkPiano,
    ElPiano1,
    ElPiano2,
    Harpsichord,
    Clavi,
    Celesta,
    Glockenspiel,
    MusicBox,
    Vibraphone,
    Marimba,
    Xylophone,
    TubularBells,
    Santur,
    DrawbarOrgan,
    PercussiveOrgan,
    RockOrgan,
    ChurchOrgan,
    ReedOrgan,
    AccordionFrench,
    Harmonica,
    TangoAccordion,
    AcGuitarNylon,
    AcGuitarSteel,
    AcGuitarJazz,
    AcGuitarClean,
    AcGuitarMuted,
    OverdrivenGuitar,
    DistortionGuitar,
    GuitarHarmonics,
    AcousticBass,
    FingerBass,
    PickedBass,
    FretlessBass,
    SlapBass1,
    SlapBass2,
    SynthBass1,
    SynthBass2,
    Violin,
    Viola,
    Cello,
    Contrabass,
    TremoloStrings,
    PizzicatoStrings,
    OrchestralHarp,
    Timpani,
    StringEnsemble1,
    StringEnsemble2,
    SynthStrings1,
    SynthStrings2,
    ChoirAahs,
    VoiceOohs,
    SynthVoice,
    OrchestraHit,
    Trumpet,
    Trombone,
    Tuba,
    MutedTrumpet,
    FrenchHorn,
    BrassSection,
    SynthBrass1,
    SynthBrass2,
    SopranoSax,
    AltoSax,
    TenorSax,
    BaritoneSax,
    Oboe,
    EnglishHorn,
    Bassoon,
    Clarinet,
    Piccolo,
    Flute,
    Recorder,
    PanFlute,
    BlownBottle,
    Shakuhachi,
    Whistle,
    Ocarina,
    Lead1Square,
    Lead2Sawtooth,
    Lead3Calliope,
    Lead4Chiff,
    Lead5Charang,
    Lead6Voice,
    Lead7Fifths,
    Lead8BassLead,
    Pad1Fantasia,
    Pad2Warm,
    Pad3PolySynth,
    Pad4Choir,
    Pad5Bowed,
    Pad6Metallic,
    Pad7Halo,
    Pad8Sweep,
    FX1Rain,
    FX2Soundtrack,
    FX3Crystal,
    FX4Atmosphere,
    FX5Brightness,
    FX6Goblins,
    FX7Echoes,
    FX8SciFi,
    Sitar,
    Banjo,
    Shamisen,
    Koto,
    Kalimba,
    BagPipe,
    Fiddle,
    Shanai,
    TinkleBell,
    Agogo,
    SteelDrums,
    Woodblock,
    TaikoDrum,
    MelodicTom,
    SynthDrum,
    ReverseCymbal,
    GtFretNoise,
    BreathNoise,
    Seashore,
    BirdTweet,
    TelephRing,
    Helicopter,
    Applause,
    Gunshot,
} unit_synth_instrument_t;


#define UNIT_SYNTH_BAUD 31250

#define MIDI_CMD_NOTE_OFF 0x80  // Note Off
#define MIDI_CMD_NOTE_ON  0x90  // Note On
#define MIDI_CMD_POLYPHONIC_AFTERTOUCH \
    0xA0  // Polyphonic Aftertouch (or Key Pressure)
#define MIDI_CMD_CONTROL_CHANGE \
    0xB0                              // Control Change (or Channel Mode
                                      // Message)
#define MIDI_CMD_PROGRAM_CHANGE 0xC0  // Program Change
#define MIDI_CMD_CHANNEL_AFTERTOUCH \
    0xD0  // Channel Aftertouch (or Channel Pressure)
#define MIDI_CMD_PITCH_BEND       0xE0  // Pitch Bend
#define MIDI_CMD_SYSTEM_EXCLUSIVE 0xF0  // System Exclusive (SysEx) Start
#define MIDI_CMD_TIME_CODE        0xF1  // MIDI Time Code Quarter Frame
#define MIDI_CMD_SONG_POSITION    0xF2  // Song Position Pointer
#define MIDI_CMD_SONG_SELECT      0xF3  // Song Select
#define MIDI_CMD_TUNE_REQUEST     0xF6  // Tune Request
#define MIDI_CMD_END_OF_SYSEX     0xF7  // End of SysEx
#define MIDI_CMD_TIMING_CLOCK \
    0xF8  // Timing Clock (used in System Real-Time Messages)
#define MIDI_CMD_START    0xFA  // Start (used in System Real-Time Messages)
#define MIDI_CMD_CONTINUE 0xFB  // Continue (used in System Real-Time Messages)
#define MIDI_CMD_STOP     0xFC  // Stop (used in System Real-Time Messages)
#define MIDI_CMD_ACTIVE_SENSING \
    0xFE  // Active Sensing (used in System Real-Time Messages)
#define MIDI_CMD_SYSTEM_RESET 0xFF  // System Reset

class M5UnitSynth: public UartDevice{
   private:
    // HardwareSerial *_serial;
    TaskHandle_t event_task_handle_ = nullptr;
    void sendCMD(uint8_t *buffer, size_t size);
    void MidiTask(void);

   public:

    M5UnitSynth(gpio_num_t tx_pin);
    void play(char data);
    void setInstrument(uint8_t bank, uint8_t channel, uint8_t value);
    void setNoteOn(uint8_t channel, uint8_t pitch, uint8_t velocity);
    void setNoteOff(uint8_t channel, uint8_t pitch, uint8_t velocity);
    void setAllNotesOff(uint8_t channel);

    void setPitchBend(uint8_t channel, int value);
    void setPitchBendRange(uint8_t channel, uint8_t value);

    void setMasterVolume(uint8_t level);             // 0-127
    void setVolume(uint8_t channel, uint8_t level);  // 0-127
    
    void setExpression(uint8_t channel, uint8_t expression);  // 0-127

    void setReverb(uint8_t channel, uint8_t program, uint8_t level,
                   uint8_t delayfeedback);

    void setChorus(uint8_t channel, uint8_t program, uint8_t level,
                   uint8_t feedback, uint8_t chorusdelay);
    void setPan(uint8_t channel, uint8_t value);
    void setEqualizer(uint8_t channel, uint8_t lowband, uint8_t medlowband,
                      uint8_t medhighband, uint8_t highband, uint8_t lowfreq,
                      uint8_t medlowfreq, uint8_t medhighfreq,
                      uint8_t highfreq);

    void setTuning(uint8_t channel, uint8_t fine,
                   uint8_t coarse);  // 0-127, 64 is default t
    void setVibrate(uint8_t channel, uint8_t rate, uint8_t depth,
                    uint8_t delay);

    void setTvf(uint8_t channel, uint8_t cutoff, uint8_t resonance);

    void setEnvelope(uint8_t channel, uint8_t attack, uint8_t decay,
                     uint8_t release);

    void setModWheel(uint8_t channel, uint8_t pitch, uint8_t tvtcutoff,
                     uint8_t amplitude, uint8_t rate, uint8_t pitchdepth,
                     uint8_t tvfdepth, uint8_t tvadepth);

    void setAllInstrumentDrums();

    void reset();
};

#endif
