#include "audio_i2s_test.h"
#include "cw_decoder.h"
#include "cw_display_source.h"
#include "board_pins.h"
#include "winkey_emulator.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"

#include "wm8960_i2s_master_tx.pio.h"
#include "wm8960_i2s_slave_rx.pio.h"

#define I2S_SAMPLE_RATE_HZ     48000u
#define I2S_BCLK_PIN           BOARD_I2S_BCLK_PIN
#define I2S_LRCLK_PIN          BOARD_I2S_LRCLK_PIN
#define I2S_ADC_DATA_PIN       BOARD_I2S_ADC_DATA_PIN
#define I2S_DAC_DATA_PIN       BOARD_I2S_DAC_DATA_PIN

#define AUDIO_RING_FRAMES       2048u
#define AUDIO_RING_BYTES        (AUDIO_RING_FRAMES * sizeof(uint32_t))
#define AUDIO_START_LEAD_FRAMES 256u

#define SIDETONE_FREQUENCY_HZ   750u
#define SIDETONE_PERIOD_FRAMES  (I2S_SAMPLE_RATE_HZ / SIDETONE_FREQUENCY_HZ)
#define SIDETONE_LEVEL_DEFAULT  8192
#define SIDETONE_RAMP_STEP      256
#define HOST_AUDIO_GAIN_MAX     32767
#define HOST_AUDIO_RAMP_STEP    512

/*
 * Do not start a new USB stream from a nearly empty queue.  Four milliseconds
 * of PCM is enough to absorb normal USB/task scheduling jitter without adding
 * noticeable operator latency.
 */
#define HOST_AUDIO_START_FRAMES       192u
#define HOST_AUDIO_STREAM_RAMP_STEP   512
#define HOST_AUDIO_UNDERRUN_FADE_FRAMES 64u

/*
 * 256 stereo frames * 4 bytes = 1024 bytes.
 *
 * The buffer must be aligned to its complete size because the DMA read-address
 * ring wraps the lower 10 address bits.
 */
static uint32_t audio_ring[AUDIO_RING_FRAMES]
    __attribute__((aligned(AUDIO_RING_BYTES)));

/*
 * USB RX samples wait here until the fixed-lead DMA renderer consumes them.
 * Keeping them separate from the DMA ring lets the 750 Hz sidetone continue
 * when Windows is not sending a speaker stream.
 */
static int16_t host_audio_ring[AUDIO_RING_FRAMES];

static const int16_t sidetone_sine[SIDETONE_PERIOD_FRAMES] = {
         0,   3212,   6393,   9512,  12539,  15446,  18204,  20787,
     23170,  25329,  27245,  28898,  30273,  31356,  32137,  32609,
     32767,  32609,  32137,  31356,  30273,  28898,  27245,  25329,
     23170,  20787,  18204,  15446,  12539,   9512,   6393,   3212,
         0,  -3212,  -6393,  -9512, -12539, -15446, -18204, -20787,
    -23170, -25329, -27245, -28898, -30273, -31356, -32137, -32609,
    -32767, -32609, -32137, -31356, -30273, -28898, -27245, -25329,
    -23170, -20787, -18204, -15446, -12539,  -9512,  -6393,  -3212,
};

#define AUDIO_CAPTURE_FRAMES 2048u
#define AUDIO_CAPTURE_BYTES  (AUDIO_CAPTURE_FRAMES * sizeof(uint16_t))

static uint16_t capture_ring[AUDIO_CAPTURE_FRAMES]
    __attribute__((aligned(AUDIO_CAPTURE_BYTES)));

static uint32_t render_frame;
static uint32_t host_write_frame;
static uint32_t host_read_frame;
static uint32_t host_queued_frames;
static uint32_t last_dma_frame;
static volatile uint32_t sidetone_sources;
static volatile int32_t sidetone_level = SIDETONE_LEVEL_DEFAULT;
static volatile uint8_t sidetone_volume_percent = 25u;
static volatile uint32_t host_audio_mute_sources;
static int32_t host_audio_gain;
static int32_t host_stream_gain;
static int16_t host_last_stream_sample;
static int16_t host_underrun_sample;
static uint32_t host_underrun_fade_remaining;
static uint32_t host_underrun_count;
static bool host_stream_started;
static int32_t sidetone_envelope;
static uint32_t sidetone_phase;
static volatile uint32_t sidetone_phase_increment = 0x04000000u;

static int16_t sidetone_next_sample(void)
{
    int32_t target = sidetone_sources != 0u ? sidetone_level : 0;

    if (sidetone_envelope < target) {
        sidetone_envelope += SIDETONE_RAMP_STEP;
        if (sidetone_envelope > target) {
            sidetone_envelope = target;
        }
    } else if (sidetone_envelope > target) {
        sidetone_envelope -= SIDETONE_RAMP_STEP;
        if (sidetone_envelope < target) {
            sidetone_envelope = target;
        }
    }

    if (sidetone_envelope == 0) {
        sidetone_phase = 0;
        return 0;
    }

    int32_t sample =
        ((int32_t)sidetone_sine[sidetone_phase >> 26] * sidetone_envelope) /
        32767;
    sidetone_phase += sidetone_phase_increment;
    return (int16_t)sample;
}

static int16_t host_audio_apply_gain(int16_t sample)
{
    int32_t target = host_audio_mute_sources != 0u ? 0 : HOST_AUDIO_GAIN_MAX;

    if (host_audio_gain < target) {
        host_audio_gain += HOST_AUDIO_RAMP_STEP;
        if (host_audio_gain > target) {
            host_audio_gain = target;
        }
    } else if (host_audio_gain > target) {
        host_audio_gain -= HOST_AUDIO_RAMP_STEP;
        if (host_audio_gain < target) {
            host_audio_gain = target;
        }
    }

    return (int16_t)(((int32_t)sample * host_audio_gain) /
                     HOST_AUDIO_GAIN_MAX);
}

static int16_t host_audio_next_stream_sample(void)
{
    if (!host_stream_started) {
        /*
         * Conceal a depleted queue by continuing from the final real sample
         * and fading it to zero.  An abrupt real-sample -> zero transition is
         * the audible click that this path is intended to prevent.
         */
        if (host_underrun_fade_remaining > 0u) {
            int32_t sample =
                (int32_t)host_underrun_sample *
                (int32_t)host_underrun_fade_remaining /
                (int32_t)HOST_AUDIO_UNDERRUN_FADE_FRAMES;
            --host_underrun_fade_remaining;
            return (int16_t)sample;
        }

        if (host_queued_frames < HOST_AUDIO_START_FRAMES) {
            return 0;
        }

        host_stream_started = true;
        host_stream_gain = 0;
    }

    if (host_queued_frames == 0u) {
        host_stream_started = false;
        host_underrun_sample = host_last_stream_sample;
        host_underrun_fade_remaining = HOST_AUDIO_UNDERRUN_FADE_FRAMES;
        ++host_underrun_count;

        /* First concealment sample equals the last real output sample. */
        int16_t sample = host_underrun_sample;
        --host_underrun_fade_remaining;
        return sample;
    }

    int16_t sample = host_audio_ring[host_read_frame];
    host_read_frame = (host_read_frame + 1u) & (AUDIO_RING_FRAMES - 1u);
    --host_queued_frames;

    if (host_stream_gain < HOST_AUDIO_GAIN_MAX) {
        host_stream_gain += HOST_AUDIO_STREAM_RAMP_STEP;
        if (host_stream_gain > HOST_AUDIO_GAIN_MAX) {
            host_stream_gain = HOST_AUDIO_GAIN_MAX;
        }
    }

    sample = (int16_t)(((int32_t)sample * host_stream_gain) /
                       HOST_AUDIO_GAIN_MAX);
    host_last_stream_sample = sample;
    return sample;
}

static int16_t saturating_add_int16(int16_t left, int16_t right)
{
    int32_t mixed = (int32_t)left + (int32_t)right;
    if (mixed > INT16_MAX) {
        mixed = INT16_MAX;
    } else if (mixed < INT16_MIN) {
        mixed = INT16_MIN;
    }
    return (int16_t)mixed;
}

typedef struct {
    PIO pio;
    int sm_data;
    int sm_capture;
    int dma_channel;
    int dma_capture_channel;
    uint program_offset_data;
    uint program_offset_capture;
    bool running;
} i2s_test_state_t;

static i2s_test_state_t state = {
    .pio = pio0,
    .sm_data = -1,
    .sm_capture = -1,
    .dma_channel = -1,
    .dma_capture_channel = -1,
    .program_offset_data = 0,
    .program_offset_capture = 0,
    .running = false
};

static uint32_t capture_read_frame;

typedef struct {
    bool active;
    uint32_t interval_start_ms;
    uint32_t interval_start_dma_count;
    uint32_t samples;
    uint32_t fifo_min;
    uint32_t fifo_max;
    uint32_t fifo_empty_hits;
    uint32_t fifo_full_hits;
    uint32_t fdebug_or_mask;
} i2s_diag_state_t;

static i2s_diag_state_t diag = {
    .active = false,
    .interval_start_ms = 0,
    .interval_start_dma_count = 0,
    .samples = 0,
    .fifo_min = 0,
    .fifo_max = 0,
    .fifo_empty_hits = 0,
    .fifo_full_hits = 0,
    .fdebug_or_mask = 0
};

static void reset_diag_interval(uint32_t dma_count_now)
{
    diag.interval_start_ms =
        (uint32_t)to_ms_since_boot(get_absolute_time());

    diag.interval_start_dma_count = dma_count_now;
    diag.samples = 0;
    diag.fifo_min = 4u;
    diag.fifo_max = 0u;
    diag.fifo_empty_hits = 0;
    diag.fifo_full_hits = 0;
    diag.fdebug_or_mask = 0;
}

static uint32_t current_dma_frame(void)
{
    uintptr_t address = dma_channel_hw_addr((uint)state.dma_channel)->read_addr;
    return (uint32_t)((address - (uintptr_t)audio_ring) / sizeof(uint32_t)) &
           (AUDIO_RING_FRAMES - 1u);
}

static uint32_t current_capture_frame(void)
{
    uintptr_t address =
        dma_channel_hw_addr((uint)state.dma_capture_channel)->write_addr;
    return (uint32_t)((address - (uintptr_t)capture_ring) / sizeof(uint16_t)) &
           (AUDIO_CAPTURE_FRAMES - 1u);
}

static bool configure_pio(
    PIO pio,
    uint sm_data,
    uint data_offset)
{
    pio_gpio_init(pio, I2S_BCLK_PIN);
    pio_gpio_init(pio, I2S_LRCLK_PIN);
    pio_gpio_init(pio, I2S_DAC_DATA_PIN);
    pio_gpio_init(pio, I2S_ADC_DATA_PIN);

    pio_sm_config data_config =
        wm8960_i2s_master_tx_program_get_default_config(data_offset);

    sm_config_set_out_pins(&data_config, I2S_DAC_DATA_PIN, 1);
    sm_config_set_sideset_pins(&data_config, I2S_BCLK_PIN);

    /*
     * MSB first with autopull. Single-SM program outputs clock + data.
     */
    sm_config_set_out_shift(
        &data_config,
        false,  /* shift left: MSB first */
        true,   /* autopull */
        32
    );

    sm_config_set_fifo_join(&data_config, PIO_FIFO_JOIN_TX);

#ifdef RP2350_SUPPORT_RP2040
    /*
     * Sysclock = 122.88 MHz. Program uses 64 PIO cycles per stereo frame.
     */
    sm_config_set_clkdiv_int_frac(&data_config, 40, 0); // RP2040 using 122.88 MHz
#else
    /*
     * Sysclock = 153.6 MHz. Program uses 64 PIO cycles per stereo frame.
     * 153600000 / (48000 * 64) = 50 exactly -> integer divider, no jitter.
     */
    sm_config_set_clkdiv_int_frac(&data_config, 50, 0);
#endif

    pio_sm_init(pio, sm_data, data_offset, &data_config);

    /* Single-SM drives BCLK/LRCLK and DATA as outputs. */
    pio_sm_set_consecutive_pindirs(pio, sm_data, I2S_BCLK_PIN,  2, true);
    pio_sm_set_consecutive_pindirs(pio, sm_data, I2S_DAC_DATA_PIN, 1, true);

    /* Start all outputs low. */
    pio_sm_set_pins_with_mask(
        pio, sm_data,
        0u,
        (1u << I2S_BCLK_PIN) |
        (1u << I2S_LRCLK_PIN) |
        (1u << I2S_DAC_DATA_PIN)
    );

    pio_sm_clear_fifos(pio, sm_data);
    pio_sm_restart(pio, sm_data);

    return true;
}

static bool configure_capture_pio(PIO pio, uint sm, uint program_offset)
{
    pio_sm_config config =
        wm8960_i2s_slave_rx_program_get_default_config(program_offset);

    sm_config_set_in_pins(&config, I2S_ADC_DATA_PIN);
    sm_config_set_in_shift(
        &config,
        false, /* shift left: incoming MSB first */
        true,  /* autopush */
        16
    );
    sm_config_set_fifo_join(&config, PIO_FIFO_JOIN_RX);
    sm_config_set_clkdiv_int_frac(&config, 1, 0);

    pio_sm_init(pio, sm, program_offset, &config);
    pio_sm_set_consecutive_pindirs(pio, sm, I2S_ADC_DATA_PIN, 1, false);
    pio_sm_clear_fifos(pio, sm);
    pio_sm_restart(pio, sm);
    return true;
}


bool audio_i2s_start(void)
{
    if (state.running) {
        return true;
    }

    memset(audio_ring, 0, sizeof(audio_ring));
    memset(host_audio_ring, 0, sizeof(host_audio_ring));
    memset(capture_ring, 0, sizeof(capture_ring));

    if (!pio_can_add_program(
            state.pio,
            &wm8960_i2s_master_tx_program)) {
        printf("I2S: no room for data PIO program\r\n");
        return false;
    }

    if (!pio_can_add_program(
            state.pio,
            &wm8960_i2s_slave_rx_program)) {
        printf("I2S: no room for capture PIO program\r\n");
        return false;
    }

    /*
     * Eerst een PIO-state-machine claimen.
     */
    state.sm_data = pio_claim_unused_sm(state.pio, false);

    if (state.sm_data < 0) {
        printf("I2S: no free PIO data state machine\r\n");
        return false;
    }

    state.sm_capture = pio_claim_unused_sm(state.pio, false);
    if (state.sm_capture < 0) {
        printf("I2S: no free PIO capture state machine\r\n");
        pio_sm_unclaim(state.pio, (uint)state.sm_data);
        state.sm_data = -1;
        return false;
    }

    /*
     * Daarna een DMA-kanaal claimen.
     *
     * Dit moet vóór dma_channel_get_default_config().
     */
    state.dma_channel = dma_claim_unused_channel(false);

    if (state.dma_channel < 0) {
        printf("I2S: no free DMA channel\r\n");

        pio_sm_unclaim(
            state.pio,
            (uint)state.sm_data
        );
        pio_sm_unclaim(state.pio, (uint)state.sm_capture);

        state.sm_data = -1;
        state.sm_capture = -1;
        return false;
    }

    state.dma_capture_channel = dma_claim_unused_channel(false);
    if (state.dma_capture_channel < 0) {
        printf("I2S: no free capture DMA channel\r\n");
        dma_channel_unclaim((uint)state.dma_channel);
        pio_sm_unclaim(state.pio, (uint)state.sm_capture);
        pio_sm_unclaim(state.pio, (uint)state.sm_data);
        state.dma_channel = -1;
        state.sm_capture = -1;
        state.sm_data = -1;
        return false;
    }

    printf(
        "I2S: claimed TX_SM=%d RX_SM=%d TX_DMA=%d RX_DMA=%d\r\n",
        state.sm_data,
        state.sm_capture,
        state.dma_channel,
        state.dma_capture_channel
    );

    state.program_offset_data = pio_add_program(
        state.pio,
        &wm8960_i2s_master_tx_program
    );
    state.program_offset_capture = pio_add_program(
        state.pio,
        &wm8960_i2s_slave_rx_program
    );

    if (!configure_pio(
            state.pio,
            (uint)state.sm_data,
            state.program_offset_data)) {

        printf("I2S: PIO configuration failed\r\n");

        pio_remove_program(
            state.pio,
            &wm8960_i2s_master_tx_program,
            state.program_offset_data
        );
        pio_remove_program(
            state.pio,
            &wm8960_i2s_slave_rx_program,
            state.program_offset_capture
        );

        dma_channel_unclaim(
            (uint)state.dma_channel
        );
        dma_channel_unclaim((uint)state.dma_capture_channel);

        pio_sm_unclaim(
            state.pio,
            (uint)state.sm_data
        );
        pio_sm_unclaim(state.pio, (uint)state.sm_capture);

        state.dma_channel = -1;
        state.dma_capture_channel = -1;
        state.sm_data = -1;
        state.sm_capture = -1;
        state.program_offset_data = 0;
        state.program_offset_capture = 0;

        return false;
    }


    if (!configure_capture_pio(
            state.pio,
            (uint)state.sm_capture,
            state.program_offset_capture)) {
        printf("I2S: capture PIO configuration failed\r\n");
        audio_i2s_stop();
        return false;
    }

    /*
     * state.dma_channel bevat hier gegarandeerd een geldig kanaal.
     */
    dma_channel_config dma_config =
        dma_channel_get_default_config(
            (uint)state.dma_channel
        );

    channel_config_set_transfer_data_size(
        &dma_config,
        DMA_SIZE_32
    );

    /* Give audio DMA priority on the bus to reduce underrun risk. */
    channel_config_set_high_priority(
        &dma_config,
        true
    );

    channel_config_set_read_increment(
        &dma_config,
        true
    );

    channel_config_set_write_increment(
        &dma_config,
        false
    );

    channel_config_set_dreq(
        &dma_config,
        pio_get_dreq(
            state.pio,
            (uint)state.sm_data,
            true
        )
    );

    /*
     * false = ring op READ address.
     * 2^13 = 8192 bytes.
     */
    channel_config_set_ring(
        &dma_config,
        false,
        13
    );

#ifdef RP2350_SUPPORT_RP2040
    channel_config_set_chain_to(&dma_config, state.dma_channel);              // retrigger on completion
#endif
    /*
     * Configure DMA but do not start it yet.
     * We will synchronously enable both SMs first, then start DMA.
     */
    dma_channel_configure(
        (uint)state.dma_channel,
        &dma_config,

        /* Write address: PIO TX-FIFO. */
        &state.pio->txf[state.sm_data],

        /* Read address: aligned PCM ringbuffer. */
        audio_ring,

#ifdef RP2350_SUPPORT_RP2040
        0xFFFFFFFF,	/* RP2040: no endless mode — use max count */
#else
        /* RP2350 hardware endless mode; the read-address ring stays cyclic. */
        dma_encode_endless_transfer_count(),
#endif
        false
    );

    dma_channel_config capture_dma_config =
        dma_channel_get_default_config((uint)state.dma_capture_channel);
    channel_config_set_transfer_data_size(&capture_dma_config, DMA_SIZE_16);
#ifndef RP2350_SUPPORT_RP2040
    channel_config_set_high_priority(&capture_dma_config, true);
#endif
    channel_config_set_read_increment(&capture_dma_config, false);
    channel_config_set_write_increment(&capture_dma_config, true);
    channel_config_set_dreq(
        &capture_dma_config,
#ifndef RP2350_SUPPORT_RP2040
        pio_get_dreq(state.pio, (uint)state.sm_capture, false)
    );
    channel_config_set_ring(&capture_dma_config, true, 12);

    dma_channel_configure(
        (uint)state.dma_capture_channel,
        &capture_dma_config,
        capture_ring,
        &state.pio->rxf[state.sm_capture],
        /* RP2350 hardware endless mode; the write-address ring stays cyclic. */
        dma_encode_endless_transfer_count(),
#else
        pio_get_dreq(state.pio, (uint)state.sm_capture, false)
    );
    channel_config_set_ring(&capture_dma_config, true, __builtin_ctz(sizeof(capture_ring)));

    channel_config_set_chain_to(&capture_dma_config, state.dma_capture_channel);              // retrigger on completion

    dma_channel_configure(
        (uint)state.dma_capture_channel,
        &capture_dma_config,
        capture_ring,                        /* read: PCM ring     */
        &state.pio->rxf[state.sm_capture],   /* write: PIO TX FIFO */
        0xFFFFFFFF,                        /* RP2040: no endless mode — use max count */
#endif
        false
    );

    /*
     * Start DMA first so it pre-fills the TX FIFO while the SM is still
     * stopped. This provides maximum headroom against startup underruns.
     */
    dma_start_channel_mask(
        1u << (uint)state.dma_channel
    );

    while (pio_sm_get_tx_fifo_level(state.pio, (uint)state.sm_data) < 4u) {
        tight_loop_contents();
    }

    dma_start_channel_mask(1u << (uint)state.dma_capture_channel);

    /* Start capture before the master clock. It waits for an LRCLK edge. */
    pio_sm_set_enabled(state.pio, (uint)state.sm_capture, true);
    pio_sm_set_enabled(state.pio, (uint)state.sm_data, true);

    state.running = true;
    last_dma_frame = current_dma_frame();
    render_frame = (last_dma_frame + AUDIO_START_LEAD_FRAMES) &
                   (AUDIO_RING_FRAMES - 1u);
    host_write_frame = 0u;
    host_read_frame = 0u;
    host_queued_frames = 0u;
    sidetone_sources = 0u;
    sidetone_level = SIDETONE_LEVEL_DEFAULT;
    sidetone_volume_percent = 25u;
    host_audio_mute_sources = 0u;
    host_audio_gain = HOST_AUDIO_GAIN_MAX;
    host_stream_gain = 0;
    host_last_stream_sample = 0;
    host_underrun_sample = 0;
    host_underrun_fade_remaining = 0u;
    host_underrun_count = 0u;
    host_stream_started = false;
    sidetone_envelope = 0;
    sidetone_phase = 0u;
    sidetone_phase_increment = 0x04000000u;
    capture_read_frame = current_capture_frame();
    diag.active = false;

    printf("I2S: DMA and PIO enabled\r\n");

    return true;
}

size_t audio_i2s_read_mono16(int16_t *samples, size_t frame_count)
{
    if (!state.running || samples == NULL || frame_count == 0u) {
        return 0;
    }

    uint32_t write_frame_now = current_capture_frame();
    uint32_t available =
        (write_frame_now - capture_read_frame) & (AUDIO_CAPTURE_FRAMES - 1u);

    /* If USB was closed, discard stale audio and resume near live capture. */
    if (available > 256u) {
        capture_read_frame =
            (write_frame_now - 96u) & (AUDIO_CAPTURE_FRAMES - 1u);
        available = 96u;
    }

    if (frame_count > available) {
        frame_count = available;
    }

    for (size_t i = 0; i < frame_count; ++i) {
        samples[i] = (int16_t)capture_ring[capture_read_frame];
        capture_read_frame =
            (capture_read_frame + 1u) & (AUDIO_CAPTURE_FRAMES - 1u);
    }

    return frame_count;
}

void audio_i2s_task(void)
{
    if (!state.running) {
        return;
    }

    uint32_t dma_frame = current_dma_frame();
    uint32_t consumed = (dma_frame - last_dma_frame) & (AUDIO_RING_FRAMES - 1u);

    for (uint32_t i = 0; i < consumed; ++i) {
        int16_t host_sample = host_audio_next_stream_sample();
        host_sample = host_audio_apply_gain(host_sample);

        int16_t sidetone_sample = sidetone_next_sample();
        int16_t mixed = saturating_add_int16(host_sample, sidetone_sample);

        /*
         * Decoder input follows the on-screen source selection: WinKey text
         * never loops back into it (it displays directly), and RX/paddle can
         * be isolated from each other instead of always mixed together.
         */
        bool paddle_active = winkey_emulator_is_paddle_keying();
        cw_display_source_t decoder_source = cw_display_source_get();
        int16_t decoder_input = 0;
        bool feed_decoder = true;
        switch (decoder_source) {
            case CW_DISPLAY_SOURCE_RX_AUDIO:
                decoder_input = host_sample;
                break;
            case CW_DISPLAY_SOURCE_PADDLE_KEY:
                decoder_input = paddle_active ? sidetone_sample : 0;
                break;
            case CW_DISPLAY_SOURCE_ALL:
                decoder_input = host_sample;
                if (paddle_active) {
                    decoder_input = saturating_add_int16(decoder_input, sidetone_sample);
                }
                break;
            case CW_DISPLAY_SOURCE_WINKEYER:
            case CW_DISPLAY_SOURCE_NONE:
            default:
                /* Decoder is disabled for these modes; skip the cross-core write. */
                feed_decoder = false;
                break;
        }
        if (feed_decoder) {
            cw_decoder_submit_audio(&decoder_input, 1);
        }

        uint16_t sample = (uint16_t)mixed;
        audio_ring[render_frame] = ((uint32_t)sample << 16) | sample;
        render_frame = (render_frame + 1u) & (AUDIO_RING_FRAMES - 1u);
    }

    last_dma_frame = dma_frame;
}

size_t audio_i2s_write_mono16(const int16_t *samples, size_t frame_count)
{
    if (!state.running || samples == NULL) {
        return 0;
    }

    uint32_t free_frames = AUDIO_RING_FRAMES - host_queued_frames - 1u;
    if (frame_count > free_frames) {
        frame_count = free_frames;
    }

    for (size_t i = 0; i < frame_count; ++i) {
        host_audio_ring[host_write_frame] = samples[i];
        host_write_frame =
            (host_write_frame + 1u) & (AUDIO_RING_FRAMES - 1u);
    }

    host_queued_frames += (uint32_t)frame_count;
    return frame_count;
}

uint32_t audio_i2s_playback_queued_frames(void)
{
    return host_queued_frames;
}

uint32_t audio_i2s_get_host_underrun_count(void)
{
    return host_underrun_count;
}

void audio_i2s_set_sidetone(bool enabled)
{
    audio_i2s_set_sidetone_source(AUDIO_SIDETONE_SOURCE_LEGACY, enabled);
}

void audio_i2s_set_sidetone_source(uint32_t source_mask, bool enabled)
{
    if (enabled) {
        sidetone_sources |= source_mask;
    } else {
        sidetone_sources &= ~source_mask;
    }
}

void audio_i2s_set_sidetone_frequency(uint32_t frequency_hz)
{
    if (frequency_hz < 100u) {
        frequency_hz = 100u;
    } else if (frequency_hz > 8000u) {
        frequency_hz = 8000u;
    }

    sidetone_phase_increment =
        (uint32_t)(((uint64_t)frequency_hz << 32) / I2S_SAMPLE_RATE_HZ);
}

void audio_i2s_set_sidetone_volume(uint8_t percent)
{
    if (percent > 100u) {
        percent = 100u;
    }

    sidetone_volume_percent = percent;
    sidetone_level = (int32_t)(((uint32_t)INT16_MAX * percent) / 100u);
}

uint8_t audio_i2s_get_sidetone_volume(void)
{
    return sidetone_volume_percent;
}

void audio_i2s_set_usb_playback_mute_source(
    uint32_t source_mask,
    bool muted
)
{
    if (muted) {
        host_audio_mute_sources |= source_mask;
    } else {
        host_audio_mute_sources &= ~source_mask;
    }
}

void audio_i2s_stop(void)
{
    if (!state.running) {
        return;
    }

    /*
     * Set running false only after using it as the program ownership marker.
     */
    pio_sm_set_enabled(state.pio, (uint)state.sm_data, false);
    if (state.sm_capture >= 0) {
        pio_sm_set_enabled(state.pio, (uint)state.sm_capture, false);
    }

    dma_channel_abort((uint)state.dma_channel);
    dma_channel_unclaim((uint)state.dma_channel);
    state.dma_channel = -1;

    if (state.dma_capture_channel >= 0) {
        dma_channel_abort((uint)state.dma_capture_channel);
        dma_channel_unclaim((uint)state.dma_capture_channel);
        state.dma_capture_channel = -1;
    }

    pio_sm_clear_fifos(state.pio, (uint)state.sm_data);
    pio_sm_unclaim(state.pio, (uint)state.sm_data);
    state.sm_data = -1;

    if (state.sm_capture >= 0) {
        pio_sm_clear_fifos(state.pio, (uint)state.sm_capture);
        pio_sm_unclaim(state.pio, (uint)state.sm_capture);
        state.sm_capture = -1;
    }

    pio_remove_program(
        state.pio,
        &wm8960_i2s_master_tx_program,
        state.program_offset_data
    );
    pio_remove_program(
        state.pio,
        &wm8960_i2s_slave_rx_program,
        state.program_offset_capture
    );

    state.program_offset_data = 0;
    state.program_offset_capture = 0;
    state.running = false;
    diag.active = false;
}

// void audio_i2s_debug(void)
// {
//     if (!state.running ||
//         state.sm < 0 ||
//         state.dma_channel < 0) {

//         printf("I2S DEBUG: not running\r\n");
//         return;
//     }

//     const uint dma_channel = (uint)state.dma_channel;
//     const uint sm = (uint)state.sm;

//     uint32_t before =
//         dma_channel_hw_addr(dma_channel)->transfer_count;

//     sleep_ms(100);

//     uint32_t after =
//         dma_channel_hw_addr(dma_channel)->transfer_count;

//     uint32_t consumed = before - after;

//     printf("\r\n");
//     printf("----- I2S DEBUG -----\r\n");

//     printf(
//         "PIO=%u SM=%u DMA=%u\r\n",
//         pio_get_index(state.pio),
//         sm,
//         dma_channel
//     );

//     printf(
//         "DMA busy=%d\r\n",
//         dma_channel_is_busy(dma_channel)
//     );

//     printf(
//         "DMA count before=%lu after=%lu consumed=%lu/100ms\r\n",
//         (unsigned long)before,
//         (unsigned long)after,
//         (unsigned long)consumed
//     );

//     printf(
//         "PIO TX empty=%d full=%d\r\n",
//         pio_sm_is_tx_fifo_empty(state.pio, sm),
//         pio_sm_is_tx_fifo_full(state.pio, sm)
//     );

//     printf(
//         "Pinmux BCLK=%d LRCLK=%d DATA=%d (1 means PIO0)\r\n",
//         gpio_get_function(I2S_BCLK_PIN) == GPIO_FUNC_PIO0,
//         gpio_get_function(I2S_LRCLK_PIN) == GPIO_FUNC_PIO0,
//         gpio_get_function(I2S_DATA_PIN) == GPIO_FUNC_PIO0
//     );

//     printf(
//         "PIO direction configured as BCLK=input, LRCLK=input, DATA=output\r\n"
//     );

//     printf(
//         "PIO FDEBUG=0x%08lx\r\n",
//         (unsigned long)state.pio->fdebug
//     );

//     printf("---------------------\r\n");
// }

void audio_i2s_debug(void)
{
    if (!state.running ||
        state.sm_data < 0 ||
        state.dma_channel < 0) {

        printf("I2S DEBUG: not running\r\n");
        return;
    }

    const uint dma_channel = (uint)state.dma_channel;
    const uint sm_data = (uint)state.sm_data;

    const uint32_t dma_count_now =
        dma_channel_hw_addr(dma_channel)->transfer_count;

    const uint32_t fifo_level_now =
        pio_sm_get_tx_fifo_level(state.pio, sm_data);

    const uint32_t fdebug_now = state.pio->fdebug;

    if (!diag.active) {
        diag.active = true;
        reset_diag_interval(dma_count_now);
    }

    ++diag.samples;

    if (fifo_level_now < diag.fifo_min) {
        diag.fifo_min = fifo_level_now;
    }

    if (fifo_level_now > diag.fifo_max) {
        diag.fifo_max = fifo_level_now;
    }

    if (fifo_level_now == 0u) {
        ++diag.fifo_empty_hits;
    }

    if (fifo_level_now >= 4u) {
        ++diag.fifo_full_hits;
    }

    diag.fdebug_or_mask |= fdebug_now;

    /* FDEBUG bits are sticky; write back sampled bits to clear them. */
    state.pio->fdebug = fdebug_now;

    const uint32_t now_ms =
        (uint32_t)to_ms_since_boot(get_absolute_time());

    const uint32_t elapsed_ms = now_ms - diag.interval_start_ms;

    if (elapsed_ms < 1000u) {
        return;
    }

    uint32_t dma_words = 0u;

    if (diag.interval_start_dma_count >= dma_count_now) {
        dma_words = diag.interval_start_dma_count - dma_count_now;
    }

    const uint pc_data = pio_sm_get_pc(state.pio, sm_data);

    printf("\r\n");
    printf("----- I2S DIAG 1s -----\r\n");

    printf(
        "PIO=%u SM=%u PC=%u OFF=%u DMA=%u busy=%d\r\n",
        pio_get_index(state.pio),
        sm_data,
        pc_data,
        state.program_offset_data,
        dma_channel,
        dma_channel_is_busy(dma_channel)
    );

    printf(
        "FIFO level now=%lu min=%lu max=%lu empty_hits=%lu full_hits=%lu samples=%lu\r\n",
        (unsigned long)fifo_level_now,
        (unsigned long)diag.fifo_min,
        (unsigned long)diag.fifo_max,
        (unsigned long)diag.fifo_empty_hits,
        (unsigned long)diag.fifo_full_hits,
        (unsigned long)diag.samples
    );

    printf(
        "DMA words=%lu in %lums (~%lu words/s)\r\n",
        (unsigned long)dma_words,
        (unsigned long)elapsed_ms,
        elapsed_ms > 0u
            ? (unsigned long)((uint64_t)dma_words * 1000u / elapsed_ms)
            : 0ul
    );

    printf(
        "FDEBUG_OR=0x%08lx pinfunc BCLK/LRCLK/DATA=%d/%d/%d\r\n",
        (unsigned long)diag.fdebug_or_mask,
        gpio_get_function(I2S_BCLK_PIN),
        gpio_get_function(I2S_LRCLK_PIN),
        gpio_get_function(I2S_DAC_DATA_PIN)
    );

    printf(
        "USB playback queued=%lu started=%d underruns=%lu\r\n",
        (unsigned long)host_queued_frames,
        host_stream_started,
        (unsigned long)host_underrun_count
    );

    printf("-----------------------\r\n");

    reset_diag_interval(dma_count_now);
}
