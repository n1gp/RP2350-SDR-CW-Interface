#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "hardware/gpio.h"
#include "tusb.h"
#include <string.h>

#include "wm8960.h"
#include "board_pins.h"
#include "audio_i2s_test.h"
#include "front_panel_controls.h"
#include "control_console.h"
#include "cw_decoder.h"
#include "cw_text_display.h"
#include "settings_storage.h"
#include "usb_audio_callbacks.h"
#include "winkey_emulator.h"

#ifdef LCD
    #include "common.h"
    #include "lcd.h"
    #include "lcd_extra.h"
#endif

#define MIDI_CW_KEY_NOTE 17u
#define MIDI_PTT_NOTE    18u
wm8960_t codec;
board_type_t board_type = (board_type_t)4; // WAVESHARE_RP2350_PIZERO_WM8960_LCD_096

static void midi_task(void)
{
#if CFG_TUD_MIDI > 0
    uint8_t packet[4];
    static uint8_t last_echo[4];
    static bool last_echo_valid;
    static uint64_t last_echo_time_us;

    while (tud_midi_packet_read(packet)) {
        uint64_t now_us = time_us_64();
        bool immediate_echo_return =
            last_echo_valid &&
            (now_us - last_echo_time_us) < 15000u &&
            memcmp(packet, last_echo, sizeof(packet)) == 0;

        if (immediate_echo_return) {
            continue;
        }

        uint8_t message = packet[1] & 0xf0u;
        uint8_t note = packet[2] & 0x7fu;
        uint8_t velocity = packet[3] & 0x7fu;

        /*
         * The original CWKeyer uses note 17 on MIDI channel 10. Accept the
         * same note on every channel so it is also easy to test in MIDI-OX.
         */
        if (note == MIDI_CW_KEY_NOTE) {
            if (message == 0x90u) {
                bool pressed = velocity != 0u;
                audio_i2s_set_sidetone_source(
                    AUDIO_SIDETONE_SOURCE_MIDI,
                    pressed
                );
                audio_i2s_set_usb_playback_mute_source(
                    AUDIO_USB_MUTE_SOURCE_MIDI_KEY,
                    pressed
                );
            } else if (message == 0x80u) {
                audio_i2s_set_sidetone_source(
                    AUDIO_SIDETONE_SOURCE_MIDI,
                    false
                );
                audio_i2s_set_usb_playback_mute_source(
                    AUDIO_USB_MUTE_SOURCE_MIDI_KEY,
                    false
                );
            }
        } else if (note == MIDI_PTT_NOTE) {
            if (message == 0x90u) {
                audio_i2s_set_usb_playback_mute_source(
                    AUDIO_USB_MUTE_SOURCE_MIDI_PTT,
                    velocity != 0u
                );
            } else if (message == 0x80u) {
                audio_i2s_set_usb_playback_mute_source(
                    AUDIO_USB_MUTE_SOURCE_MIDI_PTT,
                    false
                );
            }
        }

        (void)tud_midi_packet_write(packet);
        memcpy(last_echo, packet, sizeof(packet));
        last_echo_valid = true;
        last_echo_time_us = now_us;
    }
#endif
}

#ifdef LCD
void lcd_initialize(){
    #if defined(RASPBERRYPI_PICO2)
        gpio_init(PICO_DEFAULT_LED_PIN);
        gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_IN);
        board_type = gpio_get(PICO_DEFAULT_LED_PIN) ? WAVESHARE_RP2350_LCD_096 : RASPBERRY_PI_PICO_2;
        gpio_set_dir((uint)PICO_DEFAULT_LED_PIN, (bool)GPIO_OUT);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
    #elif defined(WAVESHARE_RP2350_PIZERO)
        #define PICO_DEFAULT_LED_PIN 6 // GPIO6 (connected to driver of backlight via WM8960+0.96 LCD HAT)
        gpio_init(PICO_DEFAULT_LED_PIN);
        gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_IN);
        board_type = WAVESHARE_RP2350_PIZERO_WM8960_LCD_096;
        gpio_set_dir((uint)PICO_DEFAULT_LED_PIN, (bool)GPIO_OUT);
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
    #else
        gpio_init(PICO_DEFAULT_LED_PIN);
        gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_IN);
        board_type = gpio_get(PICO_DEFAULT_LED_PIN) ? WAVESHARE_RP2040_LCD_096 : RASPBERRY_PI_PICO;
        gpio_set_dir((uint)PICO_DEFAULT_LED_PIN, (bool)GPIO_OUT);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
    #endif

    pico_st7735_80x160_config_t lcd_cfg[] = {
        // LCD_CFG_CASE: 4
        SPI_CLK_FREQ_DEFAULT,
        spi1,
        PIN_LCD_SPI1_CS_WAVESHARE_A,
        PIN_LCD_SPI1_SCK_WAVESHARE_A,
        PIN_LCD_SPI1_MOSI_WAVESHARE_A,
        PIN_LCD_DC_WAVESHARE_A,
        PIN_LCD_RST_WAVESHARE_A,
        PIN_LCD_BLK_WAVESHARE_A,
        PWM_BLK_DEFAULT,
        INVERSION_DEFAULT,  // 0: non-color-inversion, 1: color-inversion
        RGB_ORDER_DEFAULT,  // 0: RGB, 1: BGR
        ROTATION_DEFAULT,
        H_OFS_DEFAULT,
        V_OFS_DEFAULT,
        X_MIRROR_DEFAULT
    };

    LCD_Config(lcd_cfg);
    LCD_Init();
    u8 bl_val = OLED_BLK_Get_PWM();
    u8 rotation = 2;
    const uint32_t TimeStay = 1000;

    LCD_SetRotation(rotation);
    LCD_Clear(BLACK);
    BACK_COLOR=BLACK;

    /* Separates the active screen from the surrounding hardware bezel. */
    #define SCREEN_BORDER_THICKNESS 1u
#ifdef HALDZEMO_ST7789_170x320
    #define SCREEN_BORDER_RIGHT_X 309u
#else
    #define SCREEN_BORDER_RIGHT_X 149u
#endif
    LCD_Fill(0, 0, SCREEN_BORDER_RIGHT_X, SCREEN_BORDER_THICKNESS - 1, RED);
    LCD_Fill(0, LCD_H() - SCREEN_BORDER_THICKNESS, SCREEN_BORDER_RIGHT_X, LCD_H() - 1, RED);
    LCD_Fill(0, 0, SCREEN_BORDER_THICKNESS - 1, LCD_H() - 1, RED);
    LCD_Fill(SCREEN_BORDER_RIGHT_X, 0, SCREEN_BORDER_RIGHT_X, LCD_H() - 1, RED);

    //sleep_ms(TimeStay);
    cw_text_display_init();
}
#endif

/*
* WM8960 I2C-control:
*
* GPIO2 = SDA
* GPIO3 = SCL
* I2C1  = 400 kHz
*/
void WM8960_initialize() {  
    printf("\r\n");
    printf("============================\r\n");
    printf("WM8960 hardware test\r\n");
    printf(
        "System clock: %lu Hz\r\n",
        (unsigned long)clock_get_hz(clk_sys)
    );

    printf("Starting I2C1 on GPIO2/GPIO3...\r\n");

    wm8960_bus_init(
        &codec,
        i2c1,
        BOARD_I2C_SDA_PIN,
        BOARD_I2C_SCL_PIN,
        400000
    );

    /*
     * WM8960 configureren.
     * De DAC blijft tijdens initialisatie nog muted.
     */
    printf("Initializing WM8960...\r\n");

    if (!wm8960_init_playback(
            &codec,
            WM8960_RATE_48000,
            16,
            WM8960_OUTPUT_BOTH)) {

        printf("ERROR: WM8960 initialization failed\r\n");

        while (true) {
            printf("WM8960 init error\r\n");
            stdio_flush();
            sleep_ms(1000);
        }
    }

    printf("WM8960 initialized\r\n");
}

/*
* PIO en DMA starten.
*
* GPIO18 = BCLK
* GPIO19 = LRCLK
* GPIO21 = DAC-data
*/
void i2s_initialize() {
    printf("Starting PIO I2S...\r\n");

    if (!audio_i2s_start()) {
        printf("ERROR: PIO/DMA I2S start failed\r\n");

        while (true) {
            printf("PIO/DMA error\r\n");
            stdio_flush();
            sleep_ms(1000);
        }
    }

    printf("PIO I2S started\r\n");

    /*
     * Tijdelijke diagnose. Mag later verwijderd worden.
     */
   // audio_i2s_debug();
}

/*
* Hoofdtelefoon- en luidsprekervolume instellen.
*
* Begin voor de test met 100%.
* Later kan dit bijvoorbeeld 70 of 80 worden.
*/
void audio_startup() {

    if (!wm8960_set_headphone_volume(&codec, 80)) {
        printf("ERROR: headphone volume failed\r\n");

        while (true) {
            stdio_flush();
            sleep_ms(1000);
        }
    }

    if (!wm8960_set_speaker_volume(&codec, 80)) {
        printf("ERROR: speaker volume failed\r\n");

        while (true) {
            stdio_flush();
            sleep_ms(1000);
        }
    }

    printf("Headphone and speaker volume set to 80%%\r\n");

    /*
    * De DAC pas unmuten nadat PIO en DMA lopen.
    */
    if (!wm8960_start_playback(&codec)) {
        printf("ERROR: WM8960 unmute failed\r\n");

        while (true) {
            stdio_flush();
            sleep_ms(1000);
        }
    }

    printf("WM8960 playback started\r\n");
    printf("USB speaker audio routed to WM8960\r\n");
    #if CFG_TUD_MIDI > 0
        printf("750 Hz sidetone: MIDI note 17 key-down/key-up\r\n");
    #endif
}

int main(void)
{
#if RP2350_SUPPORT_RP2040
    // Reconfigure PLL_SYS to generate exactly 122.88 MHz
    // Parameters: PLL_SYS, REF_DIV (1), VCO_FREQ (1474.56 MHz), POST_DIV1 (6), POST_DIV2 (2)
    // 12MHz (XOSC) / 1 * 123 = 1476 MHz (close enough to 1474.56 MHz target via SDK macros)
    set_sys_clock_pll(1474.56 * MHZ, 6, 2);
#else
    /*
     * Set sysclock to 153.6 MHz: VCO=768 MHz / postdiv=5.
     *
     * 153600000 / (48000 * 64) = 50 exactly.
     * PIO divider 50 is a pure integer -> no fractional jitter ->
     * no beat-frequency click between RP2350 BCLK and WM8960 MCLK/PLL.
     *
     * USB uses its own fixed 48 MHz PLL, unaffected by this change.
     */
    set_sys_clock_pll(768 * MHZ, 5, 1);
#endif
    /*
     * TinyUSB initialiseert het samengestelde Audio + CDC + MIDI-apparaat.
     * pico_stdio_usb staat uit omdat dat eigen USB-descriptors gebruikt.
     */
    stdio_init_all();
    tusb_init();
    
    // Wait until USB is connected (optional but useful for debugging)
    //while (!stdio_usb_connected()) {
    //    sleep_ms(100);
    //}

    // Wait before stable power-on for 750ms
    // to avoid unintended power-on when Headphone plug in
    for (int i = 0; i < 30; i++) {
        sleep_ms(25);
    }

    #ifdef LCD
    lcd_initialize();
    #endif

    WM8960_initialize();
    
    i2s_initialize();
    
    audio_startup();
    
    winkey_emulator_init();
    printf("WinKey 2.3 emulator active on CDC\r\n");
    printf("Paddle DIT: GPIO23 to GND, internal pull-up enabled\r\n");
    printf("Paddle DAH: GPIO27 to GND, internal pull-up enabled\r\n");
    printf("Straight key: GPIO22 to GND, internal pull-up enabled\r\n");

    if (settings_storage_init(&codec)) {
        printf("Persistent settings loaded; double-click encoder to save\r\n");
    } else {
        printf("WARNING: persistent settings storage unavailable\r\n");
    }

    if (!front_panel_controls_init(&codec)) {
        printf("ERROR: front-panel controls initialization failed\r\n");
        while (true) {
            tud_task();
            tight_loop_contents();
        }
    }
    printf("Encoder: GPIO15/GPIO6, button GPIO13\r\n");
    printf("Function LEDs: GPIO7/GPIO8/GPIO12/GPIO11/GPIO10\r\n");
    printf("Output LEDs: headphone GPIO24, speaker GPIO25\r\n");

    if (!control_console_init(&codec)) {
        printf("ERROR: control console initialization failed\r\n");
    } else {
        printf("Control console active on CDC interface 1\r\n");
    }

    if (cw_decoder_init()) {
        printf("CW decoder active on core 1 at 750 Hz\r\n");
    }

#if defined(RP2350_USB_AUDIO_DUPLEX_ONLY) || defined(RP2350_USB_AUDIO_DUPLEX_CDC) || defined(RP2350_USB_AUDIO_DUPLEX_CDC_MIDI)
    if (!wm8960_start_capture(&codec)) {
        printf("ERROR: WM8960 capture start failed\r\n");
        while (true) {
            tud_task();
            tight_loop_contents();
        }
    }
    printf("WM8960 microphone capture started on GPIO20\r\n");
    printf("TinyUSB RX + TX Audio ready\r\n");
#else
    printf("TinyUSB audio ready\r\n");
#endif

    while (true) {
        tud_task();
        midi_task();
        winkey_emulator_task();
        control_console_task();
        cw_decoder_task();
        front_panel_controls_task();
        settings_storage_task();
        audio_i2s_task();
        usb_audio_task();
        tight_loop_contents();
    }
}
