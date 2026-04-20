cmake_minimum_required(VERSION 3.20.0)

if(NOT DEFINED BOARD)
    set(BOARD xiao_esp32c6/esp32c6/hpcore)
endif()

if(BOARD STREQUAL "xiao_esp32c6/esp32c6/hpcore")
    set(DTC_OVERLAY_FILE "boards/esp32c6.overlay")
    set(EXTRA_CONF_FILE "boards/esp32c6.conf")
    set(ZEPHYR_EXTRA_MODULES "${CMAKE_SOURCE_DIR}/../modules/HX711")
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(display_test
    VERSION
        1.0
    DESCRIPTION
        "Displays stuff on screen for 266K"
)

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
)

# Link the blink_hello library to the application
target_link_libraries(app PRIVATE mlx90614)