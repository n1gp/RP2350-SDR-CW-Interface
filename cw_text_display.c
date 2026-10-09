#include "cw_text_display.h"

#include "board_pins.h"

#ifdef LCD

#include <stdint.h>
#include <string.h>

#include "cw_decoder.h"
#include "cw_display_source.h"
#include "winkey_emulator.h"
#include "lcd.h"
#include "pico/time.h"

/* 8x16 standard font bitmap, defined once in lcd.c (via oledfont.h). */
extern const u8 asc2_1608[1520];
#define FONT_SRC_W 8u
#define FONT_SRC_H 16u
#define FONT_GLYPH_COUNT 95u /* ' ' (0x20) through '~' (0x7E) */

/*
 * Scrolling CW text log, scaled up from the 8x16 standard font for
 * readability. No title bar and no LED row anymore, so the whole screen
 * (minus a slim right-hand meter/icon column) is available.
 */
#ifdef HALDZEMO_ST7789_170x320
#define CW_TEXT_ROWS       7u
#define CW_TEXT_COLS       19u
#else
#define CW_TEXT_ROWS       3u
#define CW_TEXT_COLS       9u
#endif
#define CW_TEXT_LEFT_X     2u
#define CW_TEXT_TOP_Y      2u
#define CW_TEXT_GLYPH_W    16u
#define CW_TEXT_GLYPH_H    22u
#define CW_TEXT_ROW_HEIGHT CW_TEXT_GLYPH_H
#define CW_TEXT_READER_COLOR      GREEN
#define CW_TEXT_PADDLE_COLOR      WHITE
#define CW_TEXT_ALL_ROWS_MASK ((1u << CW_TEXT_ROWS) - 1u)

/*
 * CW tuning indicator and output icons live just right of the content box,
 * whose right border is at x=149.  The meter's outline box starts at x=151
 * so its left edge sits just past the border with a 1px gap.
 */
#ifdef HALDZEMO_ST7789_170x320
#define CW_LEVEL_BAR_X          312u
#define CW_LEVEL_BAR_WIDTH      7u
#define CW_LEVEL_BAR_HEIGHT     115u
#else
#define CW_LEVEL_BAR_X          152u
#define CW_LEVEL_BAR_WIDTH      5u
#define CW_LEVEL_BAR_HEIGHT     56u
#endif
#define CW_LEVEL_BAR_TOP        2u
#define CW_LEVEL_BAR_UPDATE_MS  100u
#define CW_TUNE_FILL_COLOR      GREEN
#define CW_TUNE_BACK_COLOR      BLACK
#define CW_TUNE_CENTER_COLOR    WHITE

/* Output icons (headphone/speaker), stacked below the RX level bar. */
#ifdef HALDZEMO_ST7789_170x320
#define OUTPUT_ICON_X          311u
#define OUTPUT_ICON_WIDTH      11u
#define OUTPUT_ICON_HEIGHT     22u
#define OUTPUT_ICON_HP_Y       122u
#define OUTPUT_ICON_SPK_Y      147u
#else
#define OUTPUT_ICON_X          151u
#define OUTPUT_ICON_WIDTH      7u
#define OUTPUT_ICON_HEIGHT     9u
#define OUTPUT_ICON_HP_Y       60u
#define OUTPUT_ICON_SPK_Y      70u
#endif
#define OUTPUT_ICON_ON_COLOR   BLUE
#define OUTPUT_ICON_OFF_COLOR  LGRAY

/*
 * Popup overlay covering the scrolling text area while a function is being
 * viewed/adjusted (or a save-result message is shown).
 */
#define POPUP_X              2u
#define POPUP_Y              2u
#ifdef HALDZEMO_ST7789_170x320
#define POPUP_WIDTH          306u
#else
#define POPUP_WIDTH          146u
#endif
#define POPUP_HEIGHT         66u
#define POPUP_LABEL_Y        (POPUP_Y + 4u)
#define POPUP_MESSAGE_Y      (POPUP_Y + 12u)
#define POPUP_BAR_Y          (POPUP_Y + 34u)
#define POPUP_BAR_X          (POPUP_X + 4u)
#define POPUP_BAR_WIDTH      (POPUP_WIDTH - 8u)
#define POPUP_BAR_HEIGHT     24u
#define POPUP_TEXT_COLOR     WHITE
#define POPUP_BAR_FILL_COLOR GREEN
#define POPUP_BAR_BACK_COLOR DARKGRAY

static char lines[CW_TEXT_ROWS][CW_TEXT_COLS + 1u];
static uint16_t line_colors[CW_TEXT_ROWS][CW_TEXT_COLS + 1u];
static uint8_t cursor_column;
static volatile uint8_t dirty_rows;
static uint32_t level_bar_next_update_ms;
static bool suppressed;

static volatile bool popup_active;
static volatile bool popup_dirty;
static char popup_text[16];
static volatile float popup_fraction;
static volatile bool popup_show_bar;

static volatile bool icons_dirty;
static volatile bool icon_hp_on;
static volatile bool icon_spk_on;

/*
 * Nearest-neighbour scale of the 8x16 font to an arbitrary glyph box.
 *
 * Renders the whole glyph into a buffer and blits it in a single SPI burst
 * (LCD_BlitBuffer), instead of one LCD_WR_DATA()/LCD_DrawPoint() call per
 * pixel - each of which toggles chip-select separately and was slow enough
 * to starve the audio DMA ring and cause audible clicks during redraws.
 */
#define MAX_GLYPH_W 32u
#define MAX_GLYPH_H 32u
static u8 glyph_pixel_buffer[MAX_GLYPH_W * MAX_GLYPH_H * 2u];

static void draw_scaled_char(
    uint16_t x, uint16_t y, char ch,
    uint16_t glyph_w, uint16_t glyph_h, uint16_t color)
{
    uint8_t index = (uint8_t)((unsigned char)ch - ' ');
    if (index >= FONT_GLYPH_COUNT) {
        index = 0u; /* fall back to a blank cell for out-of-range input */
    }
    const u8 *glyph = &asc2_1608[(uint16_t)index * FONT_SRC_H];

    if (glyph_w > MAX_GLYPH_W) {
        glyph_w = MAX_GLYPH_W;
    }
    if (glyph_h > MAX_GLYPH_H) {
        glyph_h = MAX_GLYPH_H;
    }

    size_t out = 0u;
    for (uint16_t oy = 0u; oy < glyph_h; ++oy) {
        uint16_t sy = (uint16_t)((uint32_t)oy * FONT_SRC_H / glyph_h);
        u8 row_bits = glyph[sy];
        for (uint16_t ox = 0u; ox < glyph_w; ++ox) {
            uint16_t sx = (uint16_t)((uint32_t)ox * FONT_SRC_W / glyph_w);
            bool set = (row_bits & (u8)(1u << sx)) != 0u;
            uint16_t pixel = set ? color : BACK_COLOR;
            glyph_pixel_buffer[out++] = (u8)(pixel >> 8);
            glyph_pixel_buffer[out++] = (u8)pixel;
        }
    }
    LCD_BlitBuffer(x, y, glyph_w, glyph_h, glyph_pixel_buffer);
}

static void draw_large_text(uint16_t x, uint16_t y, const char *text, uint16_t color)
{
    while (*text != '\0') {
        draw_scaled_char(x, y, *text, CW_TEXT_GLYPH_W, CW_TEXT_GLYPH_H, color);
        x = (uint16_t)(x + CW_TEXT_GLYPH_W);
        ++text;
    }
}

static void draw_value_bar(
    uint16_t x, uint16_t y, uint16_t width, uint16_t height,
    float fraction, uint16_t fill_color, uint16_t back_color)
{
    if (fraction < 0.0f) {
        fraction = 0.0f;
    } else if (fraction > 1.0f) {
        fraction = 1.0f;
    }
    uint16_t fill_width = (uint16_t)(fraction * (float)width);

    if (fill_width != 0u) {
        LCD_Fill(x, y, (u16)(x + fill_width - 1u), (u16)(y + height - 1u), fill_color);
    }
    if (fill_width < width) {
        LCD_Fill(
            (u16)(x + fill_width),
            y,
            (u16)(x + width - 1u),
            (u16)(y + height - 1u),
            back_color
        );
    }
}

static void draw_row(uint8_t row) {
    // Clear/draw background row first or draw character-by-character
    uint16_t x = CW_TEXT_LEFT_X;
    uint16_t y = (uint16_t)(CW_TEXT_TOP_Y + row * CW_TEXT_ROW_HEIGHT);

    // First, clear the entire row box to BACK_COLOR to prevent ghosting
    char padded[CW_TEXT_COLS + 1u];
    memset(padded, ' ', CW_TEXT_COLS);
    padded[CW_TEXT_COLS] = '\0';
    memcpy(padded, lines[row], strlen(lines[row]));
    
    // Draw background block for the row (or per character)
    for (uint8_t col = 0u; col < CW_TEXT_COLS; ++col) {
        char ch = padded[col];
        uint16_t col_color = line_colors[row][col];
        if (col_color == 0u) col_color = CW_TEXT_READER_COLOR; // default fallback
        draw_scaled_char(x, y, ch, CW_TEXT_GLYPH_W, CW_TEXT_GLYPH_H, col_color);
        x = (uint16_t)(x + CW_TEXT_GLYPH_W);
    }
}

static void redraw_all(void)
{
    for (uint8_t row = 0u; row < CW_TEXT_ROWS; ++row) {
        draw_row(row);
    }
}

void cw_text_display_redraw(void)
{
    dirty_rows = CW_TEXT_ALL_ROWS_MASK;
}

void cw_text_display_set_suppressed(bool value)
{
    suppressed = value;
}

static void draw_level_bar(void)
{
    /*
     * Energy meter: fill height follows the decoder's own in-band signal
     * amplitude (dBFS), and a white center line marks the decoder's current
     * detection threshold.  A tone that passes the decoder's gate sits
     * above the threshold line, so the bar is full-scale exactly when the
     * decoder is capturing it.
     */
    float level_dbfs = cw_decoder_get_signal_level();
    float threshold_dbfs = cw_decoder_get_threshold();
    float min_dbfs = -60.0f;
    float max_dbfs = 0.0f;
    float span = max_dbfs - min_dbfs;

    if (level_dbfs < min_dbfs) level_dbfs = min_dbfs;
    if (level_dbfs > max_dbfs) level_dbfs = max_dbfs;
    uint16_t fill_pixels =
        (uint16_t)(((level_dbfs - min_dbfs) / span) * (float)CW_LEVEL_BAR_HEIGHT);
    uint16_t fill_top = (uint16_t)(CW_LEVEL_BAR_TOP + CW_LEVEL_BAR_HEIGHT - fill_pixels);

    LCD_Fill(
        CW_LEVEL_BAR_X,
        CW_LEVEL_BAR_TOP,
        CW_LEVEL_BAR_X + CW_LEVEL_BAR_WIDTH - 1u,
        CW_LEVEL_BAR_TOP + CW_LEVEL_BAR_HEIGHT - 1u,
        CW_TUNE_BACK_COLOR
    );
    if (fill_pixels != 0u) {
        LCD_Fill(
            CW_LEVEL_BAR_X,
            fill_top,
            CW_LEVEL_BAR_X + CW_LEVEL_BAR_WIDTH - 1u,
            CW_LEVEL_BAR_TOP + CW_LEVEL_BAR_HEIGHT - 1u,
            CW_TUNE_FILL_COLOR
        );
    }

    if (threshold_dbfs < min_dbfs) threshold_dbfs = min_dbfs;
    if (threshold_dbfs > max_dbfs) threshold_dbfs = max_dbfs;
    uint16_t tick_pixels =
        (uint16_t)(((threshold_dbfs - min_dbfs) / span) * (float)CW_LEVEL_BAR_HEIGHT);
    uint16_t tick_y =
        (uint16_t)(CW_LEVEL_BAR_TOP + CW_LEVEL_BAR_HEIGHT - 1u - tick_pixels);
    LCD_DrawLine(
        CW_LEVEL_BAR_X,
        tick_y,
        CW_LEVEL_BAR_X + CW_LEVEL_BAR_WIDTH - 1u,
        tick_y,
        CW_TUNE_CENTER_COLOR
    );
}

static void draw_output_icon(uint16_t y, bool on)
{
    LCD_Fill(
        OUTPUT_ICON_X,
        y,
        OUTPUT_ICON_X + OUTPUT_ICON_WIDTH - 1u,
        y + OUTPUT_ICON_HEIGHT - 1u,
        on ? OUTPUT_ICON_ON_COLOR : OUTPUT_ICON_OFF_COLOR
    );
}

static void draw_popup(void)
{
    LCD_Fill(
        POPUP_X, POPUP_Y,
        POPUP_X + POPUP_WIDTH - 1u, POPUP_Y + POPUP_HEIGHT - 1u,
        BLACK
    );
    if (popup_show_bar) {
        draw_large_text(POPUP_X + 4u, POPUP_LABEL_Y, popup_text, POPUP_TEXT_COLOR);
        draw_value_bar(
            POPUP_BAR_X, POPUP_BAR_Y, POPUP_BAR_WIDTH, POPUP_BAR_HEIGHT,
            popup_fraction, POPUP_BAR_FILL_COLOR, POPUP_BAR_BACK_COLOR
        );
    } else {
        draw_large_text(POPUP_X + 4u, POPUP_MESSAGE_Y, popup_text, POPUP_TEXT_COLOR);
    }
}

static void clear_popup(void)
{
    LCD_Fill(
        POPUP_X, POPUP_Y,
        POPUP_X + POPUP_WIDTH - 1u, POPUP_Y + POPUP_HEIGHT - 1u,
        BLACK
    );
}

void cw_text_display_show_popup(const char *text, float fraction, bool show_bar)
{
    strncpy(popup_text, text, sizeof(popup_text) - 1u);
    popup_text[sizeof(popup_text) - 1u] = '\0';
    popup_fraction = fraction;
    popup_show_bar = show_bar;
    popup_active = true;
    popup_dirty = true;
    suppressed = true;
}

void cw_text_display_hide_popup(void)
{
    if (!popup_active) {
        return;
    }
    popup_active = false;
    popup_dirty = true;
    suppressed = false;
    dirty_rows = CW_TEXT_ALL_ROWS_MASK;
}

void cw_text_display_set_output_icons(bool headphones_on, bool speakers_on)
{
    icon_hp_on = headphones_on;
    icon_spk_on = speakers_on;
    icons_dirty = true;
}

static void scroll_up(void)
{
    for (uint8_t row = 0u; row + 1u < CW_TEXT_ROWS; ++row) {
        memcpy(lines[row], lines[row + 1u], sizeof(lines[row]));
        memcpy(line_colors[row], line_colors[row + 1u], sizeof(line_colors[row]));
    }
    lines[CW_TEXT_ROWS - 1u][0] = '\0';
    cursor_column = 0u;
    dirty_rows = CW_TEXT_ALL_ROWS_MASK;
}

void cw_text_display_init(void)
{
    memset(lines, 0, sizeof(lines));
    cursor_column = 0u;
    redraw_all();
    LCD_DrawRectangle(
        CW_LEVEL_BAR_X - 1u,
        CW_LEVEL_BAR_TOP - 1u,
        CW_LEVEL_BAR_X + CW_LEVEL_BAR_WIDTH,
        CW_LEVEL_BAR_TOP + CW_LEVEL_BAR_HEIGHT,
        WHITE
    );
    dirty_rows = 0u;
    level_bar_next_update_ms = 0u;
    popup_active = false;
    popup_dirty = false;
    icons_dirty = false;
}

void cw_text_display_put_char(char c, bool from_winkeyer)
{
    uint16_t char_color = winkey_emulator_get_ptt_output() ? CW_TEXT_PADDLE_COLOR : CW_TEXT_READER_COLOR;
    cw_display_source_t source = cw_display_source_get();
    bool allowed = from_winkeyer
        ? (source == CW_DISPLAY_SOURCE_ALL || source == CW_DISPLAY_SOURCE_WINKEYER)
        : (source == CW_DISPLAY_SOURCE_ALL ||
           source == CW_DISPLAY_SOURCE_RX_AUDIO ||
           source == CW_DISPLAY_SOURCE_PADDLE_KEY);
    if (!allowed) {
        return;
    }

    if (c == '\n') {
        scroll_up();
        return;
    }
    if (cursor_column >= CW_TEXT_COLS) {
        scroll_up();
    }
    lines[CW_TEXT_ROWS - 1u][cursor_column] = c;
    line_colors[CW_TEXT_ROWS - 1u][cursor_column] = char_color;
    lines[CW_TEXT_ROWS - 1u][cursor_column + 1u] = '\0';
    ++cursor_column;
    dirty_rows |= (uint8_t)(1u << (CW_TEXT_ROWS - 1u));
}

/* Call only from core1 (cw_decoder.c's decoder_core loop). */
void cw_text_display_task(void)
{
    if (popup_dirty) {
        popup_dirty = false;
        if (popup_active) {
            draw_popup();
        } else {
            clear_popup();
        }
    }

    if (!suppressed && dirty_rows != 0u) {
        uint8_t rows = dirty_rows;
        dirty_rows = 0u;
        for (uint8_t row = 0u; row < CW_TEXT_ROWS; ++row) {
            if ((rows & (uint8_t)(1u << row)) != 0u) {
                draw_row(row);
            }
        }
    }

    if (icons_dirty) {
        icons_dirty = false;
        draw_output_icon(OUTPUT_ICON_HP_Y, icon_hp_on);
        draw_output_icon(OUTPUT_ICON_SPK_Y, icon_spk_on);
    }

    uint32_t now = to_ms_since_boot(get_absolute_time());
    if ((int32_t)(now - level_bar_next_update_ms) >= 0) {
        level_bar_next_update_ms = now + CW_LEVEL_BAR_UPDATE_MS;
        draw_level_bar();
    }
}

#else

void cw_text_display_init(void)
{
}

void cw_text_display_put_char(char c, bool from_winkeyer)
{
    (void)c;
    (void)from_winkeyer;
}

void cw_text_display_task(void)
{
}

void cw_text_display_redraw(void)
{
}

void cw_text_display_set_suppressed(bool suppressed)
{
    (void)suppressed;
}

void cw_text_display_show_popup(const char *text, float fraction, bool show_bar)
{
    (void)text;
    (void)fraction;
    (void)show_bar;
}

void cw_text_display_hide_popup(void)
{
}

void cw_text_display_set_output_icons(bool headphones_on, bool speakers_on)
{
    (void)headphones_on;
    (void)speakers_on;
}

#endif
