cmake_minimum_required(VERSION 3.20.0)

if(NOT DEFINED BOARD)
    # The ESP32-C6 Super Mini has the same 4 MB C6 module class and native USB
    # serial/JTAG path used by this Zephyr board definition. boards/esp32c6.overlay
    # overrides all user-facing pins with raw Super Mini GPIO numbers.
    set(BOARD xiao_esp32c6/esp32c6/hpcore)
endif()

if(BOARD STREQUAL "xiao_esp32c6/esp32c6/hpcore")
    set(DTC_OVERLAY_FILE "boards/esp32c6.overlay")
    set(EXTRA_CONF_FILE "boards/esp32c6.conf")
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(FieldSterilization
    VERSION
        2.0.0
    DESCRIPTION
        "Field sterilization temperature controller"
)

if(NOT EXISTS "${CMAKE_SOURCE_DIR}/lib/mlx90614/src/driver_mlx90614.c")
    message(FATAL_ERROR
        "Missing MLX90614 driver. Run: git submodule update --init --recursive")
endif()

# Creates the library target
add_library(mlx90614 STATIC lib/mlx90614/src/driver_mlx90614.c)
# Add the libraries header files
target_include_directories(mlx90614 PUBLIC
    lib/mlx90614/src
    lib/mlx90614/interface
    )
# Required to set Zephyr compile options
target_link_libraries(mlx90614 PRIVATE zephyr_interface)


# Locate the source code for the application
target_sources(app PRIVATE
    src/main.c
    src/mlx90614_zephyr_iface.c
    src/mlx90614_read.c
    src/heater_pwm_shell.c
)

# Link the MLX90614 driver library to the application
target_link_libraries(app PRIVATE mlx90614)
