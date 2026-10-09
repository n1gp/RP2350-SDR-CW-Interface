/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * RP2350/Pico SDK port of the state-machine design in
 * dl1ycf/TeensyWinkeyEmulator, originally written by
 * Christoph van Wullen, DL1YCF (2020-2025).
 *
 * This port replaces Arduino Serial, EEPROM, GPIO, Audio and MIDI backends
 * with TinyUSB, Pico SDK GPIO and the local WM8960 audio renderer. The source
 * hardware provides a two-contact paddle on GPIO23/GPIO27 and a separate
 * straight key on GPIO22; buffered CW sent through the WinKey protocol is
 * fully generated locally.
 */

#include "winkey_emulator.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "audio_i2s_test.h"
#include "board_pins.h"
#include "cw_text_display.h"
#include "pico/stdlib.h"
#include "tusb.h"

#define WINKEY_VERSION             23u
#define WINKEY_PADDLE_DIT_PIN      BOARD_PADDLE_DIT_PIN
#define WINKEY_PADDLE_DAH_PIN      BOARD_PADDLE_DAH_PIN
#define WINKEY_STRAIGHT_KEY_PIN    BOARD_STRAIGHT_KEY_PIN
#define WINKEY_KEY_DEBOUNCE_MS     2u

#define WINKEY_MIDI_CHANNEL        10u
#define WINKEY_MIDI_KEY_NOTE       17u
#define WINKEY_MIDI_PTT_NOTE       18u
#define WINKEY_MIDI_WHEEL_CC       20u
#define WINKEY_MIDI_FREQUENCY_CC   3u

#define WINKEY_BUFFER_LENGTH       128u
#define WINKEY_BUFFER_MARGIN       85u
#define WINKEY_EEPROM_LENGTH       256u
#define WINKEY_TX_LENGTH           1024u

typedef enum {
    KEYER_IDLE = 0,
    KEYER_START_DIT,
    KEYER_SEND_DIT,
    KEYER_DIT_DELAY,
    KEYER_START_DAH,
    KEYER_SEND_DAH,
    KEYER_DAH_DELAY,
    KEYER_START_STRAIGHT,
    KEYER_SEND_STRAIGHT,
    KEYER_SEND_CHAR_PTT,
    KEYER_SEND_CHAR_ELEMENT,
    KEYER_SEND_CHAR_DELAY
} keyer_state_t;

typedef enum {
    WK_CMD_ADMIN       = 0x00,
    WK_CMD_SIDETONE    = 0x01,
    WK_CMD_SPEED       = 0x02,
    WK_CMD_WEIGHT      = 0x03,
    WK_CMD_PTT         = 0x04,
    WK_CMD_POTSET      = 0x05,
    WK_CMD_PAUSE       = 0x06,
    WK_CMD_GETPOT      = 0x07,
    WK_CMD_BACKSPACE   = 0x08,
    WK_CMD_PINCONFIG   = 0x09,
    WK_CMD_CLEAR       = 0x0a,
    WK_CMD_TUNE        = 0x0b,
    WK_CMD_HSCW        = 0x0c,
    WK_CMD_FARNSWORTH  = 0x0d,
    WK_CMD_MODE        = 0x0e,
    WK_CMD_LOADDEFAULT = 0x0f,
    WK_CMD_EXTENSION   = 0x10,
    WK_CMD_KEYCOMP     = 0x11,
    WK_CMD_PADSWITCH   = 0x12,
    WK_CMD_NULL        = 0x13,
    WK_CMD_SOFTPADDLE  = 0x14,
    WK_CMD_STATUS      = 0x15,
    WK_CMD_POINTER     = 0x16,
    WK_CMD_RATIO       = 0x17,
    WK_CMD_SETPTT      = 0x18,
    WK_CMD_KEYBUFFER   = 0x19,
    WK_CMD_WAIT        = 0x1a,
    WK_CMD_PROSIGN     = 0x1b,
    WK_CMD_BUFSPEED    = 0x1c,
    WK_CMD_HSCWSPEED   = 0x1d,
    WK_CMD_CANCELSPEED = 0x1e,
    WK_CMD_BUFNOP      = 0x1f,

    WK_STATE_FREE      = 0x20,
    WK_STATE_SWALLOW,
    WK_STATE_ECHO,
    WK_STATE_MESSAGE,
    WK_STATE_POINTER_1,
    WK_STATE_POINTER_2,
    WK_STATE_POINTER_3,
    WK_STATE_DUMP_EEPROM,
    WK_STATE_LOAD_EEPROM
} winkey_state_t;

typedef enum {
    WK_ADMIN_CALIBRATE = 0,
    WK_ADMIN_RESET     = 1,
    WK_ADMIN_OPEN      = 2,
    WK_ADMIN_CLOSE     = 3,
    WK_ADMIN_ECHO      = 4,
    WK_ADMIN_PAD_A2D   = 5,
    WK_ADMIN_SPD_A2D   = 6,
    WK_ADMIN_GETVALUES = 7,
    WK_ADMIN_DEBUG     = 8,
    WK_ADMIN_GETMAJOR  = 9,
    WK_ADMIN_SETWK1    = 10,
    WK_ADMIN_SETWK2    = 11,
    WK_ADMIN_DUMP      = 12,
    WK_ADMIN_LOAD      = 13,
    WK_ADMIN_SENDMSG   = 14,
    WK_ADMIN_LOADX1    = 15,
    WK_ADMIN_FWUPDATE  = 16,
    WK_ADMIN_LOWBAUD   = 17,
    WK_ADMIN_HIGHBAUD  = 18,
    WK_ADMIN_RTTY      = 19,
    WK_ADMIN_SETWK3    = 20,
    WK_ADMIN_VCC       = 21,
    WK_ADMIN_LOADX2    = 22,
    WK_ADMIN_GETMINOR  = 23,
    WK_ADMIN_GETTYPE   = 24,
    WK_ADMIN_VOLUME    = 25
} winkey_admin_command_t;

static uint8_t mode_register = 0x10u;
static uint8_t speed_wpm = 21u;
static uint8_t sidetone_parameter = 5u;
static uint16_t sidetone_frequency_hz = 750u;
static uint8_t weight = 50u;
static uint8_t ptt_lead_in = 0u;
static uint8_t ptt_tail = 0u;
static uint8_t minimum_wpm = 8u;
static uint8_t wpm_range = 20u;
static uint8_t extension = 0u;
static uint8_t compensation = 0u;
static uint8_t farnsworth = 10u;
static uint8_t paddle_point = 50u;
static uint8_t ratio = 50u;
static winkey_midi_ptt_mode_t midi_ptt_mode = WINKEY_MIDI_PTT_OFF;
/* Keep logical WinKey PTT enabled; MIDI note 18 has its own output mode. */
static uint8_t pin_config = 0x0fu;

#define SERIAL_ECHO_ENABLED    ((mode_register & 0x04u) != 0u)
#define PADDLE_ECHO_ENABLED    ((mode_register & 0x40u) != 0u)
#define PADDLE_SWAP            ((mode_register & 0x08u) != 0u)
#define IAMBIC_A               ((mode_register & 0x30u) == 0x10u)
#define BUG_MODE               ((mode_register & 0x30u) == 0x30u)
#define ULTIMATIC_MODE         ((mode_register & 0x30u) == 0x20u)
#define CONTEST_SPACING        ((mode_register & 0x01u) != 0u)
#define SIDETONE_ENABLED       ((pin_config & 0x02u) != 0u)
#define PTT_ENABLED            ((pin_config & 0x01u) != 0u)
#define PTT_HANG_BITS          ((pin_config >> 4) & 0x03u)

static uint8_t eeprom_data[WINKEY_EEPROM_LENGTH];

static uint8_t character_buffer[WINKEY_BUFFER_LENGTH];
static uint8_t buffer_read;
static uint8_t buffer_write;
static uint8_t buffer_count;

static uint8_t tx_buffer[WINKEY_TX_LENGTH];
static uint16_t tx_read;
static uint16_t tx_write;
static uint16_t tx_count;

static keyer_state_t keyer_state;
static winkey_state_t winkey_state;
static uint32_t now_ms;
static uint32_t deadline_ms;
static uint32_t last_key_up_ms;
static uint32_t straight_pressed_ms;
static uint32_t last_frequency_report_ms;

static bool paddle_dit;
static bool paddle_dah;
static bool straight_key;
static bool memory_dit;
static bool memory_dah;
static bool held_dit;
static bool held_dah;
static bool last_pressed_dah;
static bool key_output;
static bool ptt_output;
static bool host_mode;
static bool pausing;
static bool tuning;
static bool break_in;
static bool prosign;
static bool sent_space;

static uint8_t host_speed;
static uint8_t buffered_speed;
static uint8_t speed_pot;
static uint8_t winkey_status = 0xc0u;
static uint8_t last_reported_status = 0xffu;
static uint8_t last_reported_speed_pot = 0xffu;
static uint8_t sending = 1u;
static uint8_t collecting;
static uint8_t collecting_position;
static uint8_t replay_pointer;
static uint8_t parameter_index;
static uint8_t parameter_remaining;
static uint8_t prosign_first;
static uint16_t eeprom_transfer_index;

static bool deadline_reached(uint32_t deadline)
{
    return (int32_t)(now_ms - deadline) >= 0;
}

static bool host_tx_byte(uint8_t value)
{
    if (tx_count >= WINKEY_TX_LENGTH) {
        return false;
    }

    tx_buffer[tx_write] = value;
    tx_write = (uint16_t)((tx_write + 1u) % WINKEY_TX_LENGTH);
    ++tx_count;
    return true;
}

static void host_tx_task(void)
{
#if CFG_TUD_CDC > 0
    if (!tud_cdc_n_connected(0u)) {
        return;
    }

    bool wrote = false;
    while (tx_count > 0u && tud_cdc_n_write_available(0u) > 0u) {
        uint32_t available = tud_cdc_n_write_available(0u);
        uint32_t contiguous = WINKEY_TX_LENGTH - tx_read;
        uint32_t count = tx_count;
        if (count > contiguous) {
            count = contiguous;
        }
        if (count > available) {
            count = available;
        }
        if (count == 0u) {
            break;
        }

        uint32_t accepted = tud_cdc_n_write(0u, &tx_buffer[tx_read], count);
        if (accepted == 0u) {
            break;
        }
        tx_read = (uint16_t)((tx_read + accepted) % WINKEY_TX_LENGTH);
        tx_count = (uint16_t)(tx_count - accepted);
        wrote = true;
    }

    if (wrote) {
        tud_cdc_n_write_flush(0u);
    }
#endif
}

static bool host_byte_available(void)
{
#if CFG_TUD_CDC > 0
    return tud_cdc_n_available(0u) != 0u;
#else
    return false;
#endif
}

static uint8_t host_read_byte(void)
{
#if CFG_TUD_CDC > 0
    int32_t value = tud_cdc_n_read_char(0u);
    return value < 0 ? 0u : (uint8_t)value;
#else
    return 0u;
#endif
}

static void midi_send_note(uint8_t note, bool pressed)
{
#if CFG_TUD_MIDI > 0
    uint8_t packet[4] = {
        0x09u,
        (uint8_t)(0x90u | ((WINKEY_MIDI_CHANNEL - 1u) & 0x0fu)),
        (uint8_t)(note & 0x7fu),
        pressed ? 127u : 0u
    };
    (void)tud_midi_packet_write(packet);
#else
    (void)note;
    (void)pressed;
#endif
}

static void midi_send_control_change(uint8_t control, uint8_t value)
{
#if CFG_TUD_MIDI > 0
    uint8_t packet[4] = {
        0x0bu,
        (uint8_t)(0xb0u | ((WINKEY_MIDI_CHANNEL - 1u) & 0x0fu)),
        (uint8_t)(control & 0x7fu),
        (uint8_t)(value & 0x7fu)
    };
    (void)tud_midi_packet_write(packet);
#else
    (void)control;
    (void)value;
#endif
}

void winkey_emulator_send_midi_wheel_step(int direction)
{
    if (direction != 0) {
        midi_send_control_change(
            WINKEY_MIDI_WHEEL_CC,
            direction > 0 ? 127u : 1u
        );
    }
}

static uint8_t midi_frequency_value(uint16_t frequency_hz)
{
    /* DL1YCF piHPSDR maps an absolute MIDI slider 0..127 to 300..1000 Hz. */
    if (frequency_hz < 300u) {
        frequency_hz = 300u;
    } else if (frequency_hz > 1000u) {
        frequency_hz = 1000u;
    }
    return (uint8_t)((((uint32_t)frequency_hz - 300u) * 127u + 350u) /
                     700u);
}

static bool paddle_keying_state(void)
{
    return keyer_state == KEYER_START_DIT ||
           keyer_state == KEYER_SEND_DIT ||
           keyer_state == KEYER_START_DAH ||
           keyer_state == KEYER_SEND_DAH ||
           keyer_state == KEYER_START_STRAIGHT ||
           keyer_state == KEYER_SEND_STRAIGHT;
}

bool winkey_emulator_is_paddle_keying(void)
{
    return key_output && paddle_keying_state();
}

static void sync_sidetone_output(void)
{
    /*
     * WinKey host software is allowed to disable the protocol-controlled
     * sidetone through PINCONFIG.  That setting must not silence the local
     * monitor for the physical paddle: always feed paddle-generated elements
     * to the WM8960 sidetone mixer.
     */
    audio_i2s_set_sidetone_source(
        AUDIO_SIDETONE_SOURCE_WINKEY,
        key_output && (SIDETONE_ENABLED || paddle_keying_state())
    );
}

static void sync_usb_audio_mute(void)
{
    bool keyer_active = tuning || keyer_state != KEYER_IDLE ||
                        !deadline_reached(deadline_ms);

    audio_i2s_set_usb_playback_mute_source(
        AUDIO_USB_MUTE_SOURCE_WINKEY,
        keyer_active
    );
}

static void key_down(void)
{
    if (key_output) {
        return;
    }
    key_output = true;
    sync_sidetone_output();
    midi_send_note(WINKEY_MIDI_KEY_NOTE, true);
}

static void key_up(void)
{
    if (!key_output) {
        return;
    }
    key_output = false;
    sync_sidetone_output();
    midi_send_note(WINKEY_MIDI_KEY_NOTE, false);
}

static void ptt_on(void)
{
    if (ptt_output) {
        return;
    }
    ptt_output = true;
    if (midi_ptt_mode != WINKEY_MIDI_PTT_OFF) {
        midi_send_note(WINKEY_MIDI_PTT_NOTE, true);
    }
}

static void ptt_off(void)
{
    if (!ptt_output) {
        return;
    }
    ptt_output = false;
    if (midi_ptt_mode == WINKEY_MIDI_PTT_PIHPSDR) {
        midi_send_note(WINKEY_MIDI_PTT_NOTE, false);
    } else if (midi_ptt_mode == WINKEY_MIDI_PTT_THETIS) {
        /* Thetis MOX is a toggle: both PTT edges must be Note On. */
        midi_send_note(WINKEY_MIDI_PTT_NOTE, true);
    }
}

static void eeprom_write_defaults(void)
{
    memset(eeprom_data, 0xff, sizeof(eeprom_data));
    eeprom_data[0] = 0xa5u;
    eeprom_data[1] = mode_register;
    eeprom_data[2] = speed_wpm;
    eeprom_data[3] = sidetone_parameter;
    eeprom_data[4] = weight;
    eeprom_data[5] = ptt_lead_in;
    eeprom_data[6] = ptt_tail;
    eeprom_data[7] = minimum_wpm;
    eeprom_data[8] = wpm_range;
    eeprom_data[9] = extension;
    eeprom_data[10] = compensation;
    eeprom_data[11] = farnsworth;
    eeprom_data[12] = paddle_point;
    eeprom_data[13] = ratio;
    eeprom_data[14] = pin_config;
    eeprom_data[15] = 0u;
    eeprom_data[16] = 15u;
    eeprom_data[17] = 0x18u;
    for (unsigned i = 18u; i <= 23u; ++i) {
        eeprom_data[i] = 0u;
    }
}

static void settings_read_from_eeprom(void)
{
    if (eeprom_data[0] != 0xa5u) {
        return;
    }

    mode_register = eeprom_data[1];
    if (eeprom_data[2] >= 5u && eeprom_data[2] <= 99u) {
        speed_wpm = eeprom_data[2];
    }
    sidetone_parameter = eeprom_data[3];
    weight = eeprom_data[4];
    ptt_lead_in = eeprom_data[5];
    ptt_tail = eeprom_data[6];
    minimum_wpm = eeprom_data[7];
    wpm_range = eeprom_data[8] > 31u ? 31u : eeprom_data[8];
    extension = eeprom_data[9];
    compensation = eeprom_data[10];
    farnsworth = eeprom_data[11];
    paddle_point = eeprom_data[12];
    ratio = eeprom_data[13];
    pin_config = eeprom_data[14];
    sync_sidetone_output();
}

static void clear_character_buffer(void)
{
    buffer_read = 0u;
    buffer_write = 0u;
    buffer_count = 0u;
    pausing = false;
    buffered_speed = 0u;
}

static bool queue_bytes(uint8_t count, uint8_t a, uint8_t b, uint8_t c)
{
    if ((uint16_t)buffer_count + count > WINKEY_BUFFER_LENGTH) {
        return false;
    }

    const uint8_t values[3] = {a, b, c};
    for (uint8_t i = 0u; i < count; ++i) {
        character_buffer[buffer_write] = values[i];
        buffer_write = (uint8_t)((buffer_write + 1u) % WINKEY_BUFFER_LENGTH);
        ++buffer_count;
    }
    if (buffer_count > WINKEY_BUFFER_MARGIN) {
        winkey_status |= 0x01u;
    }
    return true;
}

static uint8_t buffer_get(void)
{
    if (buffer_count == 0u) {
        return 0u;
    }
    uint8_t value = character_buffer[buffer_read];
    buffer_read = (uint8_t)((buffer_read + 1u) % WINKEY_BUFFER_LENGTH);
    --buffer_count;
    if (buffer_count <= WINKEY_BUFFER_MARGIN) {
        winkey_status &= (uint8_t)~0x01u;
    }
    return value;
}

static void buffer_backspace(void)
{
    if (buffer_count == 0u) {
        return;
    }
    buffer_write = buffer_write == 0u
        ? (WINKEY_BUFFER_LENGTH - 1u)
        : (uint8_t)(buffer_write - 1u);
    --buffer_count;
    if (buffer_count <= WINKEY_BUFFER_MARGIN) {
        winkey_status &= (uint8_t)~0x01u;
    }
}

static const uint8_t morse_table[58] = {
    0x01, 0x52, 0x01, 0x01, 0x01, 0x01, 0x5e, 0x2d,
    0x6d, 0x01, 0x2a, 0x73, 0x61, 0x6a, 0x29, 0x3f,
    0x3e, 0x3c, 0x38, 0x30, 0x20, 0x21, 0x23, 0x27,
    0x2f, 0x47, 0x01, 0x01, 0x31, 0x01, 0x4c, 0x56,
    0x06, 0x11, 0x15, 0x09, 0x02, 0x14, 0x0b, 0x10,
    0x04, 0x1e, 0x0d, 0x12, 0x07, 0x05, 0x0f, 0x16,
    0x1b, 0x0a, 0x08, 0x03, 0x0c, 0x18, 0x0e, 0x19,
    0x1d, 0x13
};

static uint8_t ascii_to_morse(uint8_t value)
{
    if (value >= 'a' && value <= 'z') {
        value = (uint8_t)(value - ('a' - 'A'));
    }
    return value >= 33u && value <= 90u ? morse_table[value - 33u] : 1u;
}

static uint8_t morse_to_ascii(uint8_t pattern)
{
    for (uint8_t value = 33u; value <= 90u; ++value) {
        if (morse_table[value - 33u] == pattern) {
            return value;
        }
    }
    return ' ';
}

static void update_speed_pot(void)
{
    speed_pot = speed_wpm > minimum_wpm
        ? (uint8_t)(speed_wpm - minimum_wpm)
        : 0u;
    if (speed_pot > wpm_range) {
        speed_pot = wpm_range;
    }
}

static void update_sidetone_from_parameter(void)
{
    uint8_t divisor = sidetone_parameter & 0x0fu;
    if (divisor == 0u) {
        divisor = 1u;
    }
    sidetone_frequency_hz = (uint16_t)(4000u / divisor);
    audio_i2s_set_sidetone_frequency(sidetone_frequency_hz);
    sync_sidetone_output();
}

static void paddle_input_task(void)
{
    static bool initialized;
    static bool candidate_dit;
    static bool candidate_dah;
    static bool candidate_straight;
    static uint32_t candidate_dit_since;
    static uint32_t candidate_dah_since;
    static uint32_t candidate_straight_since;

    bool left_pressed = !gpio_get(WINKEY_PADDLE_DIT_PIN);
    bool right_pressed = !gpio_get(WINKEY_PADDLE_DAH_PIN);
    bool dit_pressed = PADDLE_SWAP ? right_pressed : left_pressed;
    bool dah_pressed = PADDLE_SWAP ? left_pressed : right_pressed;
    bool straight_pressed = !gpio_get(WINKEY_STRAIGHT_KEY_PIN);

    if (!initialized) {
        candidate_dit = dit_pressed;
        candidate_dah = dah_pressed;
        candidate_straight = straight_pressed;
        candidate_dit_since = now_ms;
        candidate_dah_since = now_ms;
        candidate_straight_since = now_ms;
        paddle_dit = dit_pressed;
        paddle_dah = dah_pressed;
        straight_key = straight_pressed;
        memory_dit = dit_pressed;
        memory_dah = dah_pressed;
        initialized = true;
        return;
    }

    if (dit_pressed != candidate_dit) {
        candidate_dit = dit_pressed;
        candidate_dit_since = now_ms;
    } else if (candidate_dit != paddle_dit &&
               (uint32_t)(now_ms - candidate_dit_since) >=
                   WINKEY_KEY_DEBOUNCE_MS) {
        paddle_dit = candidate_dit;
        if (paddle_dit) {
            memory_dit = true;
            last_pressed_dah = false;
        }
    }

    if (dah_pressed != candidate_dah) {
        candidate_dah = dah_pressed;
        candidate_dah_since = now_ms;
    } else if (candidate_dah != paddle_dah &&
               (uint32_t)(now_ms - candidate_dah_since) >=
                   WINKEY_KEY_DEBOUNCE_MS) {
        paddle_dah = candidate_dah;
        if (paddle_dah) {
            memory_dah = true;
            last_pressed_dah = true;
        }
    }

    if (straight_pressed != candidate_straight) {
        candidate_straight = straight_pressed;
        candidate_straight_since = now_ms;
    } else if (candidate_straight != straight_key &&
               (uint32_t)(now_ms - candidate_straight_since) >=
                   WINKEY_KEY_DEBOUNCE_MS) {
        straight_key = candidate_straight;
    }
}

static uint8_t effective_speed(void)
{
    uint8_t result = host_speed == 0u ? speed_wpm : host_speed;
    if (keyer_state >= KEYER_SEND_CHAR_PTT && buffered_speed != 0u) {
        result = buffered_speed;
    }
    if (result < 5u) {
        result = 5u;
    }
    return result;
}

static void keyer_task(void)
{
    uint8_t current_speed = effective_speed();
    uint32_t dot_length = 1200u / current_speed;
    int32_t dash_length = (int32_t)(3u * ratio * dot_length) / 50;
    int32_t element_pause = (int32_t)dot_length;
    uint32_t character_pause = 2u * dot_length;
    uint32_t word_pause = (CONTEST_SPACING ? 3u : 4u) * dot_length;
    bool effective_dit = paddle_dit;
    bool effective_dah = paddle_dah;
    bool manual_key = straight_key;

    if (BUG_MODE) {
        manual_key = manual_key || paddle_dah;
        memory_dah = false;
        effective_dah = false;
    } else if (ULTIMATIC_MODE && paddle_dit && paddle_dah) {
        if (last_pressed_dah) {
            effective_dit = false;
        } else {
            effective_dah = false;
        }
    }

    if (farnsworth > 10u && farnsworth < current_speed) {
        int32_t stretched = 3158 / farnsworth - (31 * (int32_t)dot_length) / 19;
        character_pause = (uint32_t)(3 * stretched - (int32_t)dot_length);
        word_pause = (uint32_t)((CONTEST_SPACING ? 6 : 7) * stretched -
                               (int32_t)character_pause);
    }

    if (weight != 50u) {
        int32_t correction = ((int32_t)weight - 50) * (int32_t)dot_length / 50;
        dot_length = (uint32_t)((int32_t)dot_length + correction);
        dash_length += correction;
        element_pause -= correction;
    }
    if (compensation != 0u) {
        dot_length += compensation;
        dash_length += compensation;
        element_pause -= compensation;
    }
    if (element_pause < 1) {
        element_pause = 1;
    }
    if (dash_length < 1) {
        dash_length = 1;
    }

    uint32_t hang_length;
    switch (PTT_HANG_BITS) {
        case 1u: hang_length = 9u * dot_length; break;
        case 2u: hang_length = 11u * dot_length; break;
        case 3u: hang_length = 15u * dot_length; break;
        default: hang_length = 8u * dot_length; break;
    }

    static uint8_t old_midi_frequency = 0xffu;
    uint8_t reported_frequency = midi_frequency_value(sidetone_frequency_hz);
    if (reported_frequency != old_midi_frequency ||
        (uint32_t)(now_ms - last_frequency_report_ms) >= 10000u) {
        old_midi_frequency = reported_frequency;
        last_frequency_report_ms = now_ms;
        #ifdef SEND_MIDI_FREQUENCY
        midi_send_control_change(
            WINKEY_MIDI_FREQUENCY_CC,
            reported_frequency
        );
        #endif
    }

    if ((effective_dit || effective_dah || manual_key) &&
        keyer_state >= KEYER_SEND_CHAR_PTT) {
        break_in = true;
        clear_character_buffer();
        replay_pointer = 0u;
        key_up();
        keyer_state = KEYER_IDLE;
        deadline_ms = now_ms + 10u;
    }

    switch (keyer_state) {
        case KEYER_IDLE:
            if (deadline_reached(deadline_ms)) {
                ptt_off();
            }

            if (collecting_position > 0u &&
                (uint32_t)(now_ms - last_key_up_ms) > 2u * dot_length) {
                collecting |= (uint8_t)(1u << collecting_position);
                if (PADDLE_ECHO_ENABLED && host_mode) {
                    (void)host_tx_byte(morse_to_ascii(collecting));
                }
                collecting = 0u;
                collecting_position = 0u;
            }
            if (collecting_position == 0u && !sent_space &&
                (uint32_t)(now_ms - last_key_up_ms) > 6u * dot_length) {
                if (PADDLE_ECHO_ENABLED && host_mode) {
                    (void)host_tx_byte(' ');
                }
                sent_space = true;
            }

            if (manual_key) {
                sent_space = false;
                deadline_ms = now_ms;
                if (!ptt_output && PTT_ENABLED) {
                    ptt_on();
                    deadline_ms = now_ms + 10u * ptt_lead_in;
                }
                keyer_state = KEYER_START_STRAIGHT;
                break;
            }

            if (effective_dit) {
                if (collecting_position < 7u) {
                    ++collecting_position;
                }
                sent_space = false;
                deadline_ms = now_ms;
                if (!ptt_output && PTT_ENABLED) {
                    ptt_on();
                    deadline_ms = now_ms + 10u * ptt_lead_in;
                }
                keyer_state = KEYER_START_DIT;
                break;
            }

            if (effective_dah) {
                if (collecting_position < 7u) {
                    collecting |= (uint8_t)(1u << collecting_position);
                    ++collecting_position;
                }
                sent_space = false;
                deadline_ms = now_ms;
                if (!ptt_output && PTT_ENABLED) {
                    ptt_on();
                    deadline_ms = now_ms + 10u * ptt_lead_in;
                }
                keyer_state = KEYER_START_DAH;
                break;
            }

            if (replay_pointer != 0u) {
                pausing = false;
                clear_character_buffer();
                sending = eeprom_data[replay_pointer++];
                if ((sending & 0x80u) != 0u) {
                    sending &= 0x7fu;
                    replay_pointer = 0u;
                }
                if (sending == 0x1cu) {
                    sending = 1u;
                    deadline_ms = now_ms + word_pause;
                    if (!ptt_output && PTT_ENABLED) {
                        ptt_on();
                    }
                    cw_text_display_put_char(' ', true);
                    keyer_state = KEYER_SEND_CHAR_DELAY;
                } else {
                    deadline_ms = now_ms;
                    if (!ptt_output && PTT_ENABLED) {
                        ptt_on();
                        deadline_ms = now_ms + 10u * ptt_lead_in;
                    }
                    cw_text_display_put_char((char)morse_to_ascii(sending), true);
                    keyer_state = KEYER_SEND_CHAR_PTT;
                }
                break;
            }

            if (buffer_count > 0u && !pausing) {
                uint8_t value = buffer_get();
                if (value >= 32u && value <= 127u && SERIAL_ECHO_ENABLED) {
                    if (host_mode) {
                        (void)host_tx_byte(value);
                    }
                    deadline_ms = now_ms + dot_length;
                }

                switch (value) {
                    case WK_CMD_PROSIGN:
                        prosign = true;
                        break;
                    case WK_CMD_BUFNOP:
                        break;
                    case WK_CMD_KEYBUFFER:
                    case WK_CMD_WAIT:
                    case WK_CMD_SETPTT:
                    case WK_CMD_HSCWSPEED:
                        (void)buffer_get();
                        break;
                    case WK_CMD_CANCELSPEED:
                        buffered_speed = 0u;
                        break;
                    case WK_CMD_BUFSPEED:
                        buffered_speed = buffer_get();
                        if (buffered_speed < 5u) buffered_speed = 5u;
                        if (buffered_speed > 99u) buffered_speed = 99u;
                        break;
                    case ' ':
                        sending = 1u;
                        deadline_ms = now_ms + word_pause;
                        cw_text_display_put_char(' ', true);
                        keyer_state = KEYER_SEND_CHAR_DELAY;
                        break;
                    case '[':
                        buffered_speed = 40u;
                        break;
                    case '$':
                        buffered_speed = 20u;
                        break;
                    case ']':
                        buffered_speed = 0u;
                        break;
                    case '|':
                        sending = 1u;
                        deadline_ms = now_ms + dot_length;
                        keyer_state = KEYER_SEND_CHAR_DELAY;
                        break;
                    case '{': case '}': case '^': case '_':
                    case '\\': case '`': case '~': case 0x7f:
                        break;
                    default:
                        sending = ascii_to_morse(value);
                        if (sending != 1u) {
                            deadline_ms = now_ms;
                            if (!ptt_output && PTT_ENABLED) {
                                ptt_on();
                                deadline_ms = now_ms + 10u * ptt_lead_in;
                            }
                            cw_text_display_put_char(
                                (char)(value >= 'a' && value <= 'z'
                                    ? value - ('a' - 'A')
                                    : value),
                                true);
                            keyer_state = KEYER_SEND_CHAR_PTT;
                        }
                        break;
                }
            }
            break;

        case KEYER_START_DIT:
            if (deadline_reached(deadline_ms)) {
                memory_dah = false;
                held_dah = effective_dah;
                deadline_ms = now_ms + dot_length;
                key_down();
                keyer_state = KEYER_SEND_DIT;
            }
            break;

        case KEYER_SEND_DIT:
            if (deadline_reached(deadline_ms)) {
                last_key_up_ms = now_ms;
                key_up();
                deadline_ms += (uint32_t)element_pause;
                keyer_state = KEYER_DIT_DELAY;
            }
            break;

        case KEYER_DIT_DELAY:
            if (deadline_reached(deadline_ms)) {
                if (!effective_dit && !effective_dah && IAMBIC_A) {
                    held_dah = false;
                }
                if (memory_dah || effective_dah || held_dah) {
                    if (collecting_position < 7u) {
                        collecting |= (uint8_t)(1u << collecting_position);
                        ++collecting_position;
                    }
                    keyer_state = KEYER_START_DAH;
                } else if (effective_dit) {
                    if (collecting_position < 7u) {
                        ++collecting_position;
                    }
                    keyer_state = KEYER_START_DIT;
                } else {
                    deadline_ms = now_ms + hang_length -
                                  (uint32_t)element_pause;
                    keyer_state = KEYER_IDLE;
                }
            }
            break;

        case KEYER_START_DAH:
            if (deadline_reached(deadline_ms)) {
                memory_dit = false;
                held_dit = effective_dit;
                deadline_ms = now_ms + (uint32_t)dash_length;
                key_down();
                keyer_state = KEYER_SEND_DAH;
            }
            break;

        case KEYER_SEND_DAH:
            if (deadline_reached(deadline_ms)) {
                last_key_up_ms = now_ms;
                key_up();
                deadline_ms += (uint32_t)element_pause;
                keyer_state = KEYER_DAH_DELAY;
            }
            break;

        case KEYER_DAH_DELAY:
            if (deadline_reached(deadline_ms)) {
                if (!effective_dit && !effective_dah && IAMBIC_A) {
                    held_dit = false;
                }
                if (memory_dit || effective_dit || held_dit) {
                    if (collecting_position < 7u) {
                        ++collecting_position;
                    }
                    keyer_state = KEYER_START_DIT;
                } else if (effective_dah) {
                    if (collecting_position < 7u) {
                        collecting |= (uint8_t)(1u << collecting_position);
                        ++collecting_position;
                    }
                    keyer_state = KEYER_START_DAH;
                } else {
                    deadline_ms = now_ms + hang_length -
                                  (uint32_t)element_pause;
                    keyer_state = KEYER_IDLE;
                }
            }
            break;

        case KEYER_START_STRAIGHT:
            if (deadline_reached(deadline_ms)) {
                memory_dit = false;
                memory_dah = false;
                held_dit = false;
                held_dah = false;
                if (manual_key) {
                    key_down();
                    straight_pressed_ms = now_ms;
                    keyer_state = KEYER_SEND_STRAIGHT;
                } else {
                    deadline_ms = now_ms + hang_length;
                    keyer_state = KEYER_IDLE;
                }
            }
            break;

        case KEYER_SEND_STRAIGHT:
            if (!manual_key) {
                last_key_up_ms = now_ms;
                key_up();
                if ((uint32_t)(now_ms - straight_pressed_ms) >
                    2u * (uint32_t)element_pause) {
                    collecting |= (uint8_t)(1u << collecting_position);
                }
                if (collecting_position < 7u) {
                    ++collecting_position;
                }
                deadline_ms = now_ms + hang_length;
                keyer_state = KEYER_IDLE;
            }
            break;

        case KEYER_SEND_CHAR_PTT:
            if (deadline_reached(deadline_ms)) {
                key_down();
                deadline_ms = now_ms +
                    (((sending & 1u) != 0u) ? (uint32_t)dash_length : dot_length);
                sending = (sending >> 1) & 0x7fu;
                keyer_state = KEYER_SEND_CHAR_ELEMENT;
            }
            break;

        case KEYER_SEND_CHAR_ELEMENT:
            if (deadline_reached(deadline_ms)) {
                key_up();
                deadline_ms = now_ms + (uint32_t)element_pause;
                if (sending == 1u) {
                    if (!prosign) {
                        deadline_ms += character_pause;
                    }
                    prosign = false;
                }
                keyer_state = KEYER_SEND_CHAR_DELAY;
            }
            break;

        case KEYER_SEND_CHAR_DELAY:
            if (deadline_reached(deadline_ms)) {
                if (sending == 1u) {
                    keyer_state = KEYER_IDLE;
                    deadline_ms += ptt_tail > 0u ? 10u * ptt_tail : 10u;
                } else {
                    key_down();
                    deadline_ms = now_ms +
                        (((sending & 1u) != 0u)
                            ? (uint32_t)dash_length
                            : dot_length);
                    sending = (sending >> 1) & 0x7fu;
                    keyer_state = KEYER_SEND_CHAR_ELEMENT;
                }
            }
            break;
    }
}

static void set_buffer_position(uint8_t position)
{
    buffer_write = position % WINKEY_BUFFER_LENGTH;
}

static void queue_buffer_zeroes(uint8_t count)
{
    while (count-- > 0u) {
        if (!queue_bytes(1u, 0u, 0u, 0u)) {
            break;
        }
    }
}

static void winkey_reset(void)
{
    key_up();
    ptt_off();
    settings_read_from_eeprom();
    host_mode = false;
    host_speed = 0u;
    tuning = false;
    clear_character_buffer();
    winkey_state = WK_STATE_FREE;
    keyer_state = KEYER_IDLE;
}

static void admin_command(uint8_t command)
{
    switch (command) {
        case WK_ADMIN_CALIBRATE:
            parameter_remaining = 1u;
            winkey_state = WK_STATE_SWALLOW;
            break;
        case WK_ADMIN_RESET:
            winkey_reset();
            break;
        case WK_ADMIN_OPEN:
            host_mode = true;
            clear_character_buffer();
            (void)host_tx_byte(WINKEY_VERSION);
            winkey_state = WK_STATE_FREE;
            break;
        case WK_ADMIN_CLOSE:
            host_speed = 0u;
            host_mode = false;
            clear_character_buffer();
            settings_read_from_eeprom();
            winkey_state = WK_STATE_FREE;
            break;
        case WK_ADMIN_ECHO:
            winkey_state = WK_STATE_ECHO;
            break;
        case WK_ADMIN_PAD_A2D:
            (void)host_tx_byte(0u);
            winkey_state = WK_STATE_FREE;
            break;
        case WK_ADMIN_SPD_A2D:
            (void)host_tx_byte(WINKEY_VERSION);
            winkey_state = WK_STATE_FREE;
            break;
        case WK_ADMIN_GETVALUES: {
            const uint8_t values[15] = {
                mode_register, host_speed, sidetone_parameter, weight,
                ptt_lead_in, ptt_tail, minimum_wpm, wpm_range, extension,
                compensation, farnsworth, paddle_point, ratio, pin_config, 0u
            };
            for (size_t i = 0u; i < sizeof(values); ++i) {
                (void)host_tx_byte(values[i]);
            }
            winkey_state = WK_STATE_FREE;
            break;
        }
        case WK_ADMIN_DEBUG:
            for (uint8_t i = 0u; i < 24u; ++i) {
                uint8_t value = i == 2u ? 0x49u :
                                i == 7u ? 0x4du :
                                i == 17u ? 0x4fu : 0x41u;
                (void)host_tx_byte(value);
            }
            winkey_state = WK_STATE_FREE;
            break;
        case WK_ADMIN_GETMAJOR:
            (void)host_tx_byte(WINKEY_VERSION);
            winkey_state = WK_STATE_FREE;
            break;
        case WK_ADMIN_SETWK1:
        case WK_ADMIN_SETWK2:
        case WK_ADMIN_SETWK3:
            winkey_state = WK_STATE_FREE;
            break;
        case WK_ADMIN_DUMP:
            eeprom_transfer_index = 0u;
            winkey_state = WK_STATE_DUMP_EEPROM;
            break;
        case WK_ADMIN_LOAD:
            eeprom_transfer_index = 0u;
            winkey_state = WK_STATE_LOAD_EEPROM;
            break;
        case WK_ADMIN_SENDMSG:
            winkey_state = WK_STATE_MESSAGE;
            break;
        case WK_ADMIN_LOADX1:
        case WK_ADMIN_LOADX2:
        case WK_ADMIN_VOLUME:
            parameter_remaining = 1u;
            winkey_state = WK_STATE_SWALLOW;
            break;
        case WK_ADMIN_RTTY:
            parameter_remaining = 2u;
            winkey_state = WK_STATE_SWALLOW;
            break;
        case WK_ADMIN_FWUPDATE:
            (void)host_tx_byte(0u);
            winkey_state = WK_STATE_FREE;
            break;
        case WK_ADMIN_LOWBAUD:
        case WK_ADMIN_HIGHBAUD:
            /* USB CDC has no hardware baud rate. */
            winkey_state = WK_STATE_FREE;
            break;
        case WK_ADMIN_VCC:
            (void)host_tx_byte(34u); /* Report approximately 3.3 V. */
            winkey_state = WK_STATE_FREE;
            break;
        case WK_ADMIN_GETMINOR:
        case WK_ADMIN_GETTYPE:
            (void)host_tx_byte(0u);
            winkey_state = WK_STATE_FREE;
            break;
        default:
            winkey_state = WK_STATE_FREE;
            break;
    }
}

static bool handle_immediate_state(void)
{
    switch (winkey_state) {
        case WK_CMD_GETPOT:
            (void)host_tx_byte((uint8_t)(128u + (speed_pot & 0x1fu)));
            winkey_state = WK_STATE_FREE;
            return true;
        case WK_CMD_BACKSPACE:
            buffer_backspace();
            winkey_state = WK_STATE_FREE;
            return true;
        case WK_CMD_CLEAR:
            clear_character_buffer();
            winkey_state = WK_STATE_FREE;
            return true;
        case WK_CMD_STATUS:
            (void)host_tx_byte(winkey_status);
            winkey_state = WK_STATE_FREE;
            return true;
        case WK_CMD_NULL:
            winkey_state = WK_STATE_FREE;
            return true;
        case WK_CMD_CANCELSPEED:
            (void)queue_bytes(1u, WK_CMD_CANCELSPEED, 0u, 0u);
            winkey_state = WK_STATE_FREE;
            return true;
        case WK_CMD_BUFNOP:
            (void)queue_bytes(1u, WK_CMD_BUFNOP, 0u, 0u);
            winkey_state = WK_STATE_FREE;
            return true;
        default:
            return false;
    }
}

static bool handle_eeprom_transfer(void)
{
    if (winkey_state == WK_STATE_DUMP_EEPROM) {
        while (eeprom_transfer_index < WINKEY_EEPROM_LENGTH &&
               tx_count < WINKEY_TX_LENGTH) {
            uint8_t value = eeprom_transfer_index == 0u
                ? 0xa5u
                : eeprom_data[eeprom_transfer_index];
            (void)host_tx_byte(value);
            ++eeprom_transfer_index;
        }
        if (eeprom_transfer_index == WINKEY_EEPROM_LENGTH) {
            winkey_state = WK_STATE_FREE;
        }
        return true;
    }

    if (winkey_state == WK_STATE_LOAD_EEPROM) {
        unsigned budget = 32u;
        while (budget-- > 0u && eeprom_transfer_index < WINKEY_EEPROM_LENGTH &&
               host_byte_available()) {
            uint8_t value = host_read_byte();
            eeprom_data[eeprom_transfer_index] =
                eeprom_transfer_index == 0u ? 0xa5u : value;
            ++eeprom_transfer_index;
        }
        if (eeprom_transfer_index == WINKEY_EEPROM_LENGTH) {
            settings_read_from_eeprom();
            update_sidetone_from_parameter();
            winkey_state = WK_STATE_FREE;
        }
        return true;
    }

    return false;
}

static void process_protocol_byte(uint8_t value)
{
    if (!host_mode && winkey_state == WK_STATE_FREE && value != WK_CMD_ADMIN) {
        return;
    }

    switch (winkey_state) {
        case WK_STATE_FREE:
            if (value >= 0x20u) {
                (void)queue_bytes(1u, value, 0u, 0u);
            } else {
                parameter_index = 0u;
                winkey_state = (winkey_state_t)value;
            }
            break;

        case WK_STATE_SWALLOW:
            if (parameter_remaining > 0u) {
                --parameter_remaining;
            }
            if (parameter_remaining == 0u) {
                winkey_state = WK_STATE_FREE;
            }
            break;

        case WK_STATE_ECHO:
            (void)host_tx_byte(value);
            winkey_state = WK_STATE_FREE;
            break;

        case WK_STATE_MESSAGE:
            if (value >= 1u && value <= 6u) {
                replay_pointer = eeprom_data[17u + value];
            }
            winkey_state = WK_STATE_FREE;
            break;

        case WK_CMD_ADMIN:
            admin_command(value);
            break;

        case WK_CMD_SIDETONE:
            sidetone_parameter = value;
            update_sidetone_from_parameter();
            winkey_state = WK_STATE_FREE;
            break;

        case WK_CMD_SPEED:
            host_speed = value > 99u ? 99u : value;
            winkey_state = WK_STATE_FREE;
            break;

        case WK_CMD_WEIGHT:
            weight = value < 10u ? 10u : value > 90u ? 90u : value;
            winkey_state = WK_STATE_FREE;
            break;

        case WK_CMD_PTT:
            if (parameter_index++ == 0u) {
                ptt_lead_in = value;
            } else {
                ptt_tail = value;
                winkey_state = WK_STATE_FREE;
            }
            break;

        case WK_CMD_POTSET:
            if (parameter_index == 0u) {
                minimum_wpm = value;
            } else if (parameter_index == 1u) {
                wpm_range = value > 31u ? 31u : value;
            } else {
                winkey_state = WK_STATE_FREE;
            }
            ++parameter_index;
            break;

        case WK_CMD_PAUSE:
            pausing = value != 0u;
            winkey_state = WK_STATE_FREE;
            break;

        case WK_CMD_PINCONFIG:
            pin_config = value;
            sync_sidetone_output();
            winkey_state = WK_STATE_FREE;
            break;

        case WK_CMD_TUNE:
            clear_character_buffer();
            tuning = value != 0u;
            if (tuning) {
                if (PTT_ENABLED) ptt_on();
                key_down();
            } else {
                key_up();
                ptt_off();
            }
            winkey_state = WK_STATE_FREE;
            break;

        case WK_CMD_HSCW:
        case WK_CMD_PADSWITCH:
        case WK_CMD_SOFTPADDLE:
            winkey_state = WK_STATE_FREE;
            break;

        case WK_CMD_FARNSWORTH:
            farnsworth = value < 10u ? 10u : value > 99u ? 99u : value;
            winkey_state = WK_STATE_FREE;
            break;

        case WK_CMD_MODE:
            mode_register = value;
            winkey_state = WK_STATE_FREE;
            break;

        case WK_CMD_LOADDEFAULT:
            switch (parameter_index++) {
                case 0u: mode_register = value; break;
                case 1u: host_speed = value; break;
                case 2u: sidetone_parameter = value; break;
                case 3u: weight = value; break;
                case 4u: ptt_lead_in = value; break;
                case 5u: ptt_tail = value; break;
                case 6u: minimum_wpm = value; break;
                case 7u: wpm_range = value > 31u ? 31u : value; break;
                case 8u: extension = value; break;
                case 9u: compensation = value; break;
                case 10u: farnsworth = value; break;
                case 11u: paddle_point = value; break;
                case 12u: ratio = value; break;
                case 13u: pin_config = value; break;
                default:
                    update_sidetone_from_parameter();
                    winkey_state = WK_STATE_FREE;
                    break;
            }
            break;

        case WK_CMD_EXTENSION:
            extension = value;
            winkey_state = WK_STATE_FREE;
            break;

        case WK_CMD_KEYCOMP:
            compensation = value;
            winkey_state = WK_STATE_FREE;
            break;

        case WK_CMD_POINTER:
            winkey_state = value == 1u ? WK_STATE_POINTER_1 :
                            value == 2u ? WK_STATE_POINTER_2 :
                            value == 3u ? WK_STATE_POINTER_3 : WK_STATE_FREE;
            if (value == 0u) {
                clear_character_buffer();
            }
            break;

        case WK_STATE_POINTER_1:
        case WK_STATE_POINTER_2:
            set_buffer_position(value > 0u ? (uint8_t)(value - 1u) : 0u);
            winkey_state = WK_STATE_FREE;
            break;

        case WK_STATE_POINTER_3:
            queue_buffer_zeroes(value);
            winkey_state = WK_STATE_FREE;
            break;

        case WK_CMD_RATIO:
            ratio = value < 33u ? 33u : value > 66u ? 66u : value;
            winkey_state = WK_STATE_FREE;
            break;

        case WK_CMD_SETPTT:
        case WK_CMD_KEYBUFFER:
        case WK_CMD_WAIT:
        case WK_CMD_BUFSPEED:
        case WK_CMD_HSCWSPEED:
            (void)queue_bytes(2u, (uint8_t)winkey_state, value, 0u);
            winkey_state = WK_STATE_FREE;
            break;

        case WK_CMD_PROSIGN:
            if (parameter_index++ == 0u) {
                prosign_first = value;
            } else {
                (void)queue_bytes(3u, WK_CMD_PROSIGN, prosign_first, value);
                winkey_state = WK_STATE_FREE;
            }
            break;

        default:
            winkey_state = WK_STATE_FREE;
            break;
    }
}

static void status_task(void)
{
    if (break_in) {
        winkey_status |= 0x02u;
        break_in = false;
    } else {
        winkey_status &= (uint8_t)~0x02u;
        if (keyer_state == KEYER_IDLE) {
            winkey_status &= (uint8_t)~0x04u;
        } else {
            winkey_status |= 0x04u;
        }
    }

    if (host_mode && winkey_status != last_reported_status) {
        (void)host_tx_byte(winkey_status);
        last_reported_status = winkey_status;
    }
    if (host_mode && speed_pot != last_reported_speed_pot) {
        (void)host_tx_byte((uint8_t)(128u + (speed_pot & 0x1fu)));
        last_reported_speed_pot = speed_pot;
    }
}

static void protocol_task(void)
{
    unsigned budget = 32u;
    while (budget-- > 0u) {
        if (handle_eeprom_transfer()) {
            if (winkey_state == WK_STATE_LOAD_EEPROM && !host_byte_available()) {
                break;
            }
            continue;
        }
        if (handle_immediate_state()) {
            continue;
        }
        if (!host_byte_available()) {
            break;
        }
        process_protocol_byte(host_read_byte());
    }
}

void winkey_emulator_init(void)
{
    gpio_init(WINKEY_PADDLE_DIT_PIN);
    gpio_set_dir(WINKEY_PADDLE_DIT_PIN, GPIO_IN);
    gpio_pull_up(WINKEY_PADDLE_DIT_PIN);

    gpio_init(WINKEY_PADDLE_DAH_PIN);
    gpio_set_dir(WINKEY_PADDLE_DAH_PIN, GPIO_IN);
    gpio_pull_up(WINKEY_PADDLE_DAH_PIN);

    gpio_init(WINKEY_STRAIGHT_KEY_PIN);
    gpio_set_dir(WINKEY_STRAIGHT_KEY_PIN, GPIO_IN);
    gpio_pull_up(WINKEY_STRAIGHT_KEY_PIN);

    paddle_dit = !gpio_get(WINKEY_PADDLE_DIT_PIN);
    paddle_dah = !gpio_get(WINKEY_PADDLE_DAH_PIN);
    straight_key = !gpio_get(WINKEY_STRAIGHT_KEY_PIN);
    memory_dit = paddle_dit;
    memory_dah = paddle_dah;
    keyer_state = KEYER_IDLE;
    winkey_state = WK_STATE_FREE;
    sent_space = true;
    now_ms = to_ms_since_boot(get_absolute_time());
    deadline_ms = now_ms;
    last_key_up_ms = now_ms;
    last_frequency_report_ms = now_ms - 10000u;

    eeprom_write_defaults();
    settings_read_from_eeprom();

    /* The project-specific default remains the requested exact 750 Hz. */
    sidetone_frequency_hz = 750u;
    audio_i2s_set_sidetone_frequency(sidetone_frequency_hz);
    sync_sidetone_output();
    sync_usb_audio_mute();

    midi_send_note(WINKEY_MIDI_KEY_NOTE, false);
}

void winkey_emulator_task(void)
{
    now_ms = to_ms_since_boot(get_absolute_time());
    host_tx_task();
    paddle_input_task();
    update_speed_pot();
    protocol_task();

    /* WinKey 2.3: touching either paddle exits tune mode immediately. */
    if (tuning && (paddle_dit || paddle_dah)) {
        key_up();
        ptt_off();
        tuning = false;
    }
    if (!tuning) {
        keyer_task();
    }
    status_task();
    sync_usb_audio_mute();
    host_tx_task();
}

void winkey_emulator_set_speed(uint8_t wpm)
{
    if (wpm < 5u) {
        wpm = 5u;
    } else if (wpm > 99u) {
        wpm = 99u;
    }

    speed_wpm = wpm;
    eeprom_data[2] = wpm;
    if (host_mode) {
        host_speed = wpm;
    }
    update_speed_pot();
    last_frequency_report_ms = now_ms - 10000u;
}

uint8_t winkey_emulator_get_speed(void)
{
    return effective_speed();
}

bool winkey_emulator_get_ptt_output(void)
{
    return ptt_output;
}

void winkey_emulator_set_sidetone_frequency(uint16_t frequency_hz)
{
    if (frequency_hz < 300u) {
        frequency_hz = 300u;
    } else if (frequency_hz > 1200u) {
        frequency_hz = 1200u;
    }

    sidetone_frequency_hz = frequency_hz;
    audio_i2s_set_sidetone_frequency(frequency_hz);
}

uint16_t winkey_emulator_get_sidetone_frequency(void)
{
    return sidetone_frequency_hz;
}

void winkey_emulator_set_mode(winkey_mode_t mode)
{
    uint8_t mode_bits = 0u;

    switch (mode) {
        case WINKEY_MODE_IAMBIC_A: mode_bits = 0x10u; break;
        case WINKEY_MODE_ULTIMATIC: mode_bits = 0x20u; break;
        case WINKEY_MODE_BUG: mode_bits = 0x30u; break;
        case WINKEY_MODE_IAMBIC_B:
        default: mode_bits = 0x00u; break;
    }

    mode_register = (uint8_t)((mode_register & ~0x30u) | mode_bits);
}

winkey_mode_t winkey_emulator_get_mode(void)
{
    switch (mode_register & 0x30u) {
        case 0x10u: return WINKEY_MODE_IAMBIC_A;
        case 0x20u: return WINKEY_MODE_ULTIMATIC;
        case 0x30u: return WINKEY_MODE_BUG;
        default: return WINKEY_MODE_IAMBIC_B;
    }
}

const char *winkey_emulator_mode_name(winkey_mode_t mode)
{
    switch (mode) {
        case WINKEY_MODE_IAMBIC_A: return "iambic-a";
        case WINKEY_MODE_ULTIMATIC: return "ultimatic";
        case WINKEY_MODE_BUG: return "bug";
        case WINKEY_MODE_IAMBIC_B:
        default: return "iambic-b";
    }
}

void winkey_emulator_set_paddle_swap(bool enabled)
{
    if (enabled) {
        mode_register |= 0x08u;
    } else {
        mode_register &= (uint8_t)~0x08u;
    }
}

bool winkey_emulator_get_paddle_swap(void)
{
    return PADDLE_SWAP;
}

void winkey_emulator_set_midi_ptt_mode(winkey_midi_ptt_mode_t mode)
{
    if (mode <= WINKEY_MIDI_PTT_THETIS) {
        midi_ptt_mode = mode;
    }
}

winkey_midi_ptt_mode_t winkey_emulator_get_midi_ptt_mode(void)
{
    return midi_ptt_mode;
}

const char *winkey_emulator_midi_ptt_mode_name(winkey_midi_ptt_mode_t mode)
{
    switch (mode) {
        case WINKEY_MIDI_PTT_OFF: return "off";
        case WINKEY_MIDI_PTT_PIHPSDR: return "onoff";
        case WINKEY_MIDI_PTT_THETIS: return "toggle";
        default: return "unknown";
    }
}

void winkey_emulator_set_weight(uint8_t percent)
{
    weight = percent > 100u ? 100u : percent;
}

uint8_t winkey_emulator_get_weight(void)
{
    return weight;
}

void winkey_emulator_export_eeprom(
    uint8_t output[WINKEY_PERSISTENT_EEPROM_SIZE]
)
{
    if (output == NULL) {
        return;
    }

    /* Keep the RAM-backed WinKey EEPROM image in sync with runtime values. */
    eeprom_data[0] = 0xa5u;
    eeprom_data[1] = mode_register;
    eeprom_data[2] = effective_speed();
    eeprom_data[3] = sidetone_parameter;
    eeprom_data[4] = weight;
    eeprom_data[5] = ptt_lead_in;
    eeprom_data[6] = ptt_tail;
    eeprom_data[7] = minimum_wpm;
    eeprom_data[8] = wpm_range;
    eeprom_data[9] = extension;
    eeprom_data[10] = compensation;
    eeprom_data[11] = farnsworth;
    eeprom_data[12] = paddle_point;
    eeprom_data[13] = ratio;
    eeprom_data[14] = pin_config;
    memcpy(output, eeprom_data, WINKEY_EEPROM_LENGTH);
}

bool winkey_emulator_import_eeprom(
    const uint8_t input[WINKEY_PERSISTENT_EEPROM_SIZE]
)
{
    if (input == NULL || input[0] != 0xa5u) {
        return false;
    }

    memcpy(eeprom_data, input, WINKEY_EEPROM_LENGTH);
    host_speed = 0u;
    settings_read_from_eeprom();
    update_speed_pot();
    update_sidetone_from_parameter();
    return true;
}
