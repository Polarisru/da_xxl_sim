# usage:    cmake -G "MinGW Makefiles" . -Bbuild
#           cmake --build ./build
cmake_minimum_required(VERSION 3.5)
set(CMAKE_TOOLCHAIN_FILE arm-none-eabi-gcc.cmake)
project(DA-XL)

enable_language(C ASM)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
#set(CMAKE_C_EXTENSIONS OFF)

set(EXEC_58D DA-58-D)
set(EXEC_58D_SD DA-58-D-SD)
set(EXEC_58S DA-58-S)
set(EXEC_30D DA-30-D)
set(EXEC_30S DA-30-S)

add_definitions("-DFW_VERSION=\"$ENV{FW_VERSION}\"")
add_definitions("-DJOB_NAME=\"$ENV{JOB_NAME}\"")
add_definitions("-DBUILD_NUMBER=\"$ENV{BUILD_NUMBER}\"")
add_definitions("-DGIT_COMMIT=\"$ENV{GIT_COMMIT}\"")
add_definitions("-DSERVO_VARIANT=\"$ENV{SERVO_VARIANT}\"")

set(CRC_TOOL cortex_crc)

# set output directory for generated binary file
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY output)
# include directory with headers
include_directories(src 
	drivers 
	sys 
	CMSIS 
	config 
	hal 
	hal/include 
	include 
	include/component 
	include/instance 
	include/pio
	thirdparty/RTOS
	thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/include
	thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/portable/GCC/ARM_CM0
	)
# add all project files to sources
set(sources 
	src/analog.c
	src/canbus.c
	src/chain.c
	src/comm.c
	src/conversion.c
	src/counters.c
	src/crc16.c
	src/crc32.c
	src/drv8320.c
	src/eeprom.c
	src/global.c
	src/hall.c
	src/heartbeat.c
	src/logic.c
	src/magnet.c
	src/main.c
	src/monitor.c
	src/motor.c
	src/notch.c
	src/parser_can_volz.c
	src/parser_rs485.c
	src/power.c
	src/rs485.c
	src/timeouts.c
	src/utils.c
	src/version.c
	sys/startup_samc21.c
	sys/system_samc21.c
	drivers/driver_ac.c
	drivers/driver_adc.c
	drivers/driver_can.c
	drivers/driver_clk.c
	drivers/driver_dac.c
	drivers/driver_dma.c
	drivers/driver_dsu.c
	drivers/driver_flash.c
	drivers/driver_gpio.c
	drivers/driver_pwm.c
	drivers/driver_spi.c
	drivers/driver_temp.c
	drivers/driver_timer.c
	drivers/driver_uart.c
	drivers/driver_vref.c
	drivers/driver_wdt.c
	thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/croutine.c
	thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/event_groups.c
	thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/list.c
	thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/queue.c
	thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/stream_buffer.c
	thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/tasks.c
	thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/timers.c
	thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/croutine.c
	thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/portable/GCC/ARM_CM0/port.c
	thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/portable/MemMang/heap_1.c
	)

set(sources_d
	src/inetwork.c
	src/master.c
	src/redundancy.c
	src/voting.c
	)

# generate binary file from the list of sources
add_executable (${EXEC_58D} ${sources} ${sources_d})
add_executable (${EXEC_58D_SD} ${sources} ${sources_d})
add_executable (${EXEC_58S} ${sources})
add_executable (${EXEC_30D} ${sources} ${sources_d})
add_executable (${EXEC_30S} ${sources})

set(compile_definitions
    -D__ATSAMC21G18A__ 
    -D__STACK_SIZE=0x200
    -DDEF_RELEASE
    )
target_compile_definitions(${EXEC_58D} PRIVATE
    ${compile_definitions}
    -DDEF_DUPLEX
    -DDEF_DA58
    )
target_compile_definitions(${EXEC_58D_SD} PRIVATE
    ${compile_definitions}
    -DDEF_DUPLEX
    -DDEF_DA58
    -DDEF_DA58_SD
    )
target_compile_definitions(${EXEC_58S} PRIVATE
    ${compile_definitions}
    -DDEF_SINGLE
    -DDEF_DA58
    )
target_compile_definitions(${EXEC_30D} PRIVATE
    ${compile_definitions}
    -DDEF_DUPLEX
    -DDEF_DA30
    )
target_compile_definitions(${EXEC_30S} PRIVATE
    ${compile_definitions}
    -DDEF_SINGLE
    -DDEF_DA30
    )

set(compile_options 
    -mcpu=cortex-m0plus
    -march=armv6-m
    -mthumb
    -mfloat-abi=soft
    -fdata-sections
    -ffunction-sections
    -fno-strict-aliasing
    -W
    -Wall
    -Wextra
    -Wsign-compare 
    -Wshadow 
    -Wpointer-arith 
    -Winline 
    -Wcast-qual 
    -Wfloat-equal 
    -Wlogical-op 
    -Wlogical-not-parentheses
    -Wstrict-prototypes
    -pedantic
    -O2
    )

set(link_options 
    -T../sys/samc21g18a_release.ld
    -mcpu=cortex-m0plus
    -march=armv6-m
    -mthumb
    -mfloat-abi=soft
    -specs=nano.specs
    -Wl,--gc-sections
    -Wl,--defsym=__STACK_SIZE=0x200
    -lm
    -lc
    -lnosys
    )
    
function(set_dependencies target_name)
    target_compile_options(${target_name} PUBLIC ${compile_options})
    target_link_options(${target_name} PUBLIC ${link_options})
    target_link_libraries(${target_name} m)
    add_custom_command(TARGET ${target_name}
        POST_BUILD
        COMMAND ${CMAKE_SIZE_UTIL} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${target_name})
    add_custom_command(TARGET ${target_name}
        POST_BUILD
        #COMMAND ${CMAKE_OBJCOPY} -O ihex ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${target_name} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${target_name}.hex
        COMMAND ${CMAKE_OBJCOPY} -O binary ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${target_name} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${target_name}.bin
        COMMAND ${CRC_TOOL} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${target_name}.bin
        COMMAND ${CMAKE_OBJCOPY} -I binary -O ihex --change-addresses 0x2000 ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${target_name}.bin ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${target_name}.hex
    )
endfunction()

set_dependencies(${EXEC_58D})
set_dependencies(${EXEC_58D_SD})
set_dependencies(${EXEC_58S})
set_dependencies(${EXEC_30D})
set_dependencies(${EXEC_30S})
