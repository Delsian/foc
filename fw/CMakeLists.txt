cmake_minimum_required(VERSION 3.20.0)

set(BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

if (NOT BUILD_TYPE)
    set(BUILD_TYPE debug)
endif()

# Set overlay file based on CONF_FILE environment/cache variable
# Users can build with: west build -b foc_431 -- -DCONF_FILE=prj_no_usb.conf
if(NOT DEFINED DTC_OVERLAY_FILE)
    if(DEFINED CONF_FILE AND "${CONF_FILE}" MATCHES "no_usb")
        set(DTC_OVERLAY_FILE ${CMAKE_CURRENT_SOURCE_DIR}/foc_431_no_usb.overlay)
    endif()
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(foc)

target_sources(app PRIVATE
    src/main.c
    src/oled.c
    src/motor_control.c
    src/foc/mt6701.c
    src/foc/foc_math.c
    src/foc/foc_pid.c
    src/foc/foc_motor.c
    src/foc/foc_current_sense.c
)

# USB CDC-ACM disabled (CONFIG_ENABLE_USB not defined)
# target_sources_ifdef(CONFIG_ENABLE_USB app PRIVATE src/cdcacm.c)

include_directories(inc)

# Add debug-specific compiler flags
if(BUILD_TYPE STREQUAL "debug")
    message("Set BUILD_TYPE to Debug")
    target_compile_options(app PRIVATE -O0 -g3)
    target_compile_definitions(app PRIVATE DEBUG=1)
endif()
