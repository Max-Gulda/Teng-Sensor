cmake_minimum_required(VERSION 3.20.0)

# Add local devicetree bindings directory
list(APPEND DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ads131m04)

# Add your application source
target_sources(app PRIVATE
    src/main.c
    src/ads131m04_spi/ads131m04_spi.c
    src/lsm6dso_spi/lsm6dso_spi.c
    src/sampling/sampling.c
    src/bluetooth/bluetooth.c
    src/bluetooth/bt_channel.c
    src/bluetooth/bt_ecg.c
    src/bluetooth/bt_ecg_raw.c
    src/bluetooth/bt_imu.c
    src/bluetooth/bt_hr.c
    src/ble_log_backend/ble_log_backend.c
    src/circ_buffer/circ_buffer.c
    src/pan_tompkins/pan_tompkins.c
    src/pan_tompkins/heart_rate_finder.c
    src/filters/bw_filter.c
    src/filters/signal_processing.c
    src/filters/hr_filter.c
    src/ecg/ecg.c
)

# Add include directories for module headers
target_include_directories(app PRIVATE
    src/ads131m04_spi
    src/lsm6dso_spi
    src/sampling
    src/bluetooth
    src/ble_log_backend
    src/circ_buffer
    src/helper
    src/pan_tompkins
    src/filters
    src/ecg
)
