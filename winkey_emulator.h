#ifndef WINKEY_EMULATOR_H
#define WINKEY_EMULATOR_H

#include <stdbool.h>
#include <stdint.h>

//#define SEND_MIDI_FREQUENCY  // enable to allow frequent updates to PC about current sidetone frequency value

#define WINKEY_PERSISTENT_EEPROM_SIZE 256u

typedef enum {
    WINKEY_MODE_IAMBIC_B = 0,
    WINKEY_MODE_IAMBIC_A,
    WINKEY_MODE_ULTIMATIC,
    WINKEY_MODE_BUG
} winkey_mode_t;

typedef enum {
    WINKEY_MIDI_PTT_OFF = 0,
    WINKEY_MIDI_PTT_PIHPSDR,
    WINKEY_MIDI_PTT_THETIS
} winkey_midi_ptt_mode_t;

#ifdef __cplusplus
extern "C" {
#endif

void winkey_emulator_init(void);
void winkey_emulator_task(void);
void winkey_emulator_send_midi_wheel_step(int direction);
void winkey_emulator_set_speed(uint8_t wpm);
uint8_t winkey_emulator_get_speed(void);
bool winkey_emulator_get_ptt_output(void);
void winkey_emulator_set_sidetone_frequency(uint16_t frequency_hz);
uint16_t winkey_emulator_get_sidetone_frequency(void);
void winkey_emulator_set_mode(winkey_mode_t mode);
winkey_mode_t winkey_emulator_get_mode(void);
const char *winkey_emulator_mode_name(winkey_mode_t mode);
void winkey_emulator_set_paddle_swap(bool enabled);
bool winkey_emulator_get_paddle_swap(void);
void winkey_emulator_set_midi_ptt_mode(winkey_midi_ptt_mode_t mode);
winkey_midi_ptt_mode_t winkey_emulator_get_midi_ptt_mode(void);
const char *winkey_emulator_midi_ptt_mode_name(winkey_midi_ptt_mode_t mode);
void winkey_emulator_set_weight(uint8_t percent);
uint8_t winkey_emulator_get_weight(void);
bool winkey_emulator_is_paddle_keying(void);
void winkey_emulator_export_eeprom(
    uint8_t output[WINKEY_PERSISTENT_EEPROM_SIZE]
);
bool winkey_emulator_import_eeprom(
    const uint8_t input[WINKEY_PERSISTENT_EEPROM_SIZE]
);

#ifdef __cplusplus
}
#endif

#endif
