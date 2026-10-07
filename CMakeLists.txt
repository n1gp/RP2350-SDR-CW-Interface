# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.0)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PICO_SDK_PATH /home/mh/Desktop/SDR/HPSDR/PICO/pico-sdk)
#include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

#set(PICO_EXTRAS_PATH ${PICO_SDK_PATH}/pico-examples)
set(PICO_EXTRAS_PATH /home/mh/Desktop/SDR/HPSDR/PICO/pico-extras)
#include($ENV{PICO_EXTRAS_PATH}/external/pico_extras_import.cmake)
include(${PICO_EXTRAS_PATH}/external/pico_extras_import.cmake)

# Pico SDK importeren
#include(pico_sdk_import.cmake)

project(rp2350-sdr-cw-interface C CXX ASM)


option(RP2350_USB_CDC_ONLY "Build a temporary CDC-only USB test device" OFF)
option(RP2350_USB_CDC_MIDI_TEST "Build a temporary CDC + MIDI USB test device" OFF)
option(RP2350_USB_AUDIO_CDC_TEST "Build a temporary Audio + CDC USB test device" OFF)
option(RP2350_USB_MIC_CDC_TEST "Build TinyUSB's simple microphone + CDC test device" OFF)
option(RP2350_USB_SPK_CDC_TEST "Build TinyUSB's mono speaker + CDC test device" OFF)
option(RP2350_USB_AUDIO_OUT_ONLY "Build a minimal USB Audio OUT-only device" OFF)
option(RP2350_USB_AUDIO_DUPLEX_ONLY "Build RX speaker + TX microphone without CDC" OFF)
option(RP2350_USB_AUDIO_DUPLEX_CDC "Build RX speaker + TX microphone + CDC" OFF)
option(RP2350_USB_AUDIO_DUPLEX_CDC_MIDI "Build RX speaker + TX microphone + CDC + MIDI" ON)
option(RP2350_PATCH_TINYUSB_RP2_ISO "Apply TinyUSB RP2 ISO reactivation fix #2937" ON)
option(RP2350_LCD "Build LCD Feature" ON)
option(RP2350_SUPPORT_RP2040 "Support RP2040" ON)

if(RP2350_SUPPORT_RP2040)
    message(STATUS "Configuring project for RP2040 backward compatibility")
    set(PICO_BOARD pico CACHE STRING "Board type")
    set(PICO_PLATFORM rp2040)
else()
    # RP2350
    set(PICO_BOARD waveshare_rp2350_pizero CACHE STRING "Board type")
endif()

# Pico SDK initialiseren
pico_sdk_init()

#add_subdirectory(lib/file_menu)
#add_subdirectory(lib/LcdElementBox)
#add_subdirectory(lib/pico_audio_i2s_32b)
#add_subdirectory(lib/pico_audio_i2s_32b/src/pico_audio_32b)
#add_subdirectory(lib/pico_fatfs)
#add_subdirectory(lib/pico_flash_param)
add_subdirectory(lib/pico_st7735_80x160)
#add_subdirectory(lib/picojpeg)
#add_subdirectory(lib/PlayAudio)

# Pico SDK 2.3.0 contains the pre-#2937 RP2040/RP2350 DCD implementation.
# Windows UAC2 changes alt 1 -> alt 0 -> alt 1 while selecting the format.
# Without aborting the still-active ISO transfer, the second activation stalls.
# Generate a project-local corrected source so the installed SDK stays untouched.
if(RP2350_PATCH_TINYUSB_RP2_ISO AND TARGET tinyusb_device_base)
    set(RP2350_TINYUSB_DCD_SOURCE
        "${PICO_TINYUSB_PATH}/src/portable/raspberrypi/rp2040/dcd_rp2040.c")
    set(RP2350_TINYUSB_DCD_PATCHED
        "${CMAKE_CURRENT_BINARY_DIR}/tinyusb-patched/dcd_rp2040.c")

    file(READ "${RP2350_TINYUSB_DCD_SOURCE}" RP2350_TINYUSB_DCD_TEXT)

    set(RP2350_DCD_XFER_OLD [=[
static void hw_endpoint_xfer(uint8_t ep_addr, uint8_t* buffer, uint16_t total_bytes) {
  struct hw_endpoint* ep = hw_endpoint_get_by_addr(ep_addr);
  hw_endpoint_xfer_start(ep, buffer, total_bytes);
}
]=])
    set(RP2350_DCD_XFER_NEW [=[
static void hw_endpoint_xfer(uint8_t ep_addr, uint8_t* buffer, uint16_t total_bytes) {
  struct hw_endpoint* ep = hw_endpoint_get_by_addr(ep_addr);
  hw_endpoint_xfer_start(ep, buffer, total_bytes);
}

// Backport TinyUSB #2937: abort and reset an active ISO transfer before
// Windows reactivates the alternate streaming interface.
static void hw_endpoint_abort_xfer(struct hw_endpoint* ep) {
  const uint8_t dir = tu_edpt_dir(ep->ep_addr);
  const uint8_t epnum = tu_edpt_number(ep->ep_addr);
  const uint32_t abort_mask = TU_BIT((epnum << 1) | (dir ? 0 : 1));

  if (rp2040_chip_version() >= 2) {
    usb_hw_set->abort = abort_mask;
    while ((usb_hw->abort_done & abort_mask) != abort_mask) {}
  }

  uint32_t buf_ctrl = USB_BUF_CTRL_SEL;
  if (ep->next_pid) {
    buf_ctrl |= USB_BUF_CTRL_DATA1_PID;
  }
  _hw_endpoint_buffer_control_set_value32(ep, buf_ctrl);
  hw_endpoint_reset_transfer(ep);

  if (rp2040_chip_version() >= 2) {
    usb_hw_clear->abort_done = abort_mask;
    usb_hw_clear->abort = abort_mask;
  }
}
]=])

    set(RP2350_DCD_ISO_OLD [=[
bool dcd_edpt_iso_activate(uint8_t rhport, tusb_desc_endpoint_t const * ep_desc) {
  (void) rhport;
  const uint8_t ep_addr = ep_desc->bEndpointAddress;
  // Fill in endpoint control register with buffer offset
  struct hw_endpoint* ep = hw_endpoint_get_by_addr(ep_addr);
  TU_ASSERT(ep->hw_data_buf != NULL); // must be inited and buffer allocated
  ep->wMaxPacketSize = ep_desc->wMaxPacketSize;

  hw_endpoint_enable(ep);
  return true;
}
]=])
    set(RP2350_DCD_ISO_NEW [=[
bool dcd_edpt_iso_activate(uint8_t rhport, tusb_desc_endpoint_t const * ep_desc) {
  (void) rhport;
  struct hw_endpoint* ep = hw_endpoint_get_by_addr(ep_desc->bEndpointAddress);
  TU_ASSERT(ep->hw_data_buf != NULL); // allocated by dcd_edpt_iso_alloc()

  if (ep->active) {
    hw_endpoint_abort_xfer(ep);
  }

  ep->wMaxPacketSize = ep_desc->wMaxPacketSize;
  hw_endpoint_enable(ep);
  return true;
}
]=])

    string(REPLACE "${RP2350_DCD_XFER_OLD}" "${RP2350_DCD_XFER_NEW}"
        RP2350_TINYUSB_DCD_TEXT "${RP2350_TINYUSB_DCD_TEXT}")
    string(REPLACE "${RP2350_DCD_ISO_OLD}" "${RP2350_DCD_ISO_NEW}"
        RP2350_TINYUSB_DCD_TEXT "${RP2350_TINYUSB_DCD_TEXT}")

    string(FIND "${RP2350_TINYUSB_DCD_TEXT}" "hw_endpoint_abort_xfer" RP2350_DCD_PATCH_POS)
    if(RP2350_DCD_PATCH_POS EQUAL -1)
        message(FATAL_ERROR "Could not apply TinyUSB RP2 ISO fix #2937")
    endif()

    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/tinyusb-patched")
    file(WRITE "${RP2350_TINYUSB_DCD_PATCHED}" "${RP2350_TINYUSB_DCD_TEXT}")

    get_target_property(RP2350_TINYUSB_DEVICE_SOURCES tinyusb_device_base INTERFACE_SOURCES)
    list(REMOVE_ITEM RP2350_TINYUSB_DEVICE_SOURCES "${RP2350_TINYUSB_DCD_SOURCE}")
    set_property(TARGET tinyusb_device_base PROPERTY INTERFACE_SOURCES
        "${RP2350_TINYUSB_DEVICE_SOURCES}")
    target_sources(tinyusb_device_base INTERFACE "${RP2350_TINYUSB_DCD_PATCHED}")
    target_include_directories(tinyusb_device_base INTERFACE
        "${PICO_TINYUSB_PATH}/src/portable/raspberrypi/rp2040")
endif()

# Gebruik de project-lokale TinyUSB-configuratie voor het samengestelde
# USB-apparaat (Audio + CDC + MIDI).
add_compile_definitions(
    PICO_TUSB_CONFIG_HEADER=\"${CMAKE_CURRENT_LIST_DIR}/tusb_config.h\"
)

# Bronbestanden
add_executable(rp2350-sdr-cw-interface
    rp2350-sdr-cw-interface.c
    wm8960.c
    audio_i2s_test.c
    winkey_emulator.c
    front_panel_controls.c
    control_console.c
    cw_decoder.c
    cw_text_display.c
    cw_display_source.c
    settings_storage.c
    usb_os_guessing.c
    usb_descriptors.c
    usb_audio_callbacks.c
)

set_source_files_properties(
    winkey_emulator.c
    front_panel_controls.c
    control_console.c
    cw_decoder.c
    cw_text_display.c
    cw_display_source.c
    settings_storage.c
    usb_os_guessing.c
    PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra"
)

# Programmanaam en versie
pico_set_program_name(
    rp2350-sdr-cw-interface
    "rp2350-sdr-cw-interface"
)

set_target_properties(
    rp2350-sdr-cw-interface
    PROPERTIES OUTPUT_NAME "rp2350-sdr-cw-interface"
)

pico_set_program_version(
    rp2350-sdr-cw-interface
    "0.1"
)

# # Genereer de header uit het PIO-programma
# pico_generate_pio_header(
#     rp2350-sdr-cw-interface
#     ${CMAKE_CURRENT_LIST_DIR}/wm8960_i2s_tx.pio
# )

pico_generate_pio_header(
    rp2350-sdr-cw-interface
    ${CMAKE_CURRENT_LIST_DIR}/wm8960_i2s_master_tx.pio
)

pico_generate_pio_header(
    rp2350-sdr-cw-interface
    ${CMAKE_CURRENT_LIST_DIR}/wm8960_i2s_slave_rx.pio
)

# pico_generate_pio_header(
#     rp2350-sdr-cw-interface
#     ${CMAKE_CURRENT_LIST_DIR}/i2s_clock_gen.pio
# )

# Lokale headers zoals wm8960.h en audio_i2s_test.h
target_include_directories(
    rp2350-sdr-cw-interface
    PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

# Benodigde Pico SDK libraries
target_link_libraries(
    rp2350-sdr-cw-interface
    pico_stdlib

    hardware_i2c
    hardware_pio
    hardware_dma
    hardware_clocks
    hardware_flash

    pico_flash
    pico_multicore
    tinyusb_device
    tinyusb_board

    # for LCD
    pico_st7735_80x160

    m
)


# USB wordt door TinyUSB met onze eigen composite descriptors beheerd.
# pico_stdio_usb zou daarnaast een tweede USB-configuratie proberen te maken.
pico_enable_stdio_usb(rp2350-sdr-cw-interface 0)

if(RP2350_USB_CDC_ONLY)
    target_compile_definitions(rp2350-sdr-cw-interface PRIVATE RP2350_USB_CDC_ONLY=1)
elseif(RP2350_USB_CDC_MIDI_TEST)
    target_compile_definitions(rp2350-sdr-cw-interface PRIVATE RP2350_USB_CDC_MIDI_TEST=1)
elseif(RP2350_USB_AUDIO_CDC_TEST)
    target_compile_definitions(rp2350-sdr-cw-interface PRIVATE RP2350_USB_AUDIO_CDC_TEST=1)
elseif(RP2350_USB_MIC_CDC_TEST)
    target_compile_definitions(rp2350-sdr-cw-interface PRIVATE RP2350_USB_MIC_CDC_TEST=1)
elseif(RP2350_USB_AUDIO_DUPLEX_CDC_MIDI)
    target_compile_definitions(rp2350-sdr-cw-interface PRIVATE RP2350_USB_AUDIO_DUPLEX_CDC_MIDI=1)
elseif(RP2350_USB_AUDIO_DUPLEX_CDC)
    target_compile_definitions(rp2350-sdr-cw-interface PRIVATE RP2350_USB_AUDIO_DUPLEX_CDC=1)
elseif(RP2350_USB_AUDIO_DUPLEX_ONLY)
    target_compile_definitions(rp2350-sdr-cw-interface PRIVATE RP2350_USB_AUDIO_DUPLEX_ONLY=1)
elseif(RP2350_USB_AUDIO_OUT_ONLY)
    target_compile_definitions(rp2350-sdr-cw-interface PRIVATE RP2350_USB_AUDIO_OUT_ONLY=1)
elseif(RP2350_USB_SPK_CDC_TEST)
    target_compile_definitions(rp2350-sdr-cw-interface PRIVATE RP2350_USB_SPK_CDC_TEST=1)
elseif(RP2350_LCD)
    target_compile_definitions(rp2350-sdr-cw-interface PRIVATE RP2350_LCD=1)
endif()

if(RP2350_SUPPORT_RP2040)
    target_compile_definitions(rp2350-sdr-cw-interface PRIVATE RP2350_SUPPORT_RP2040=1)
endif()

# Geen UART-debugging via GPIO0/GPIO1
pico_enable_stdio_uart(rp2350-sdr-cw-interface 0)

# Eventueel kort wachten op het openen van de Serial Monitor
target_compile_definitions(
    rp2350-sdr-cw-interface
    PRIVATE
    PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=3000
)

# Genereer UF2, HEX en BIN
pico_add_extra_outputs(rp2350-sdr-cw-interface)
