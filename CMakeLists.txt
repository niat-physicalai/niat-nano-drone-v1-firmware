cmake_minimum_required(VERSION 3.20)
project(niat_nano_drone C)
set(CMAKE_C_STANDARD 11)

option(PC_BUILD "Build host tests instead of target firmware" OFF)

set(WARN -Wall -Wextra -Wdouble-promotion -Wfloat-conversion -Werror=double-promotion)

if(PC_BUILD)
    enable_testing()
    add_executable(test_units tests/pc/test_units.c)
    target_include_directories(test_units PRIVATE interfaces)
    target_compile_options(test_units PRIVATE ${WARN})
    add_test(NAME units COMMAND test_units)
else()
    add_executable(fw app/main.c)
    target_include_directories(fw PRIVATE interfaces)
    target_compile_options(fw PRIVATE ${WARN}
        -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard
        -ffunction-sections -fdata-sections)
    target_link_options(fw PRIVATE
        -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard
        -Wl,--gc-sections --specs=nosys.specs)
endif()
