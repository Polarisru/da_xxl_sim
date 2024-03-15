# usage:    cmake . -G "MinGW Makefiles" . -Bbuild
#           cmake --build ./build
cmake_minimum_required (VERSION 3.5)
project (da58_test)

enable_language(C ASM)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(EXECUTABLE_58D DA-58-D_UnitTest)
set(EXECUTABLE_58D_SD DA-58-D-SD_UnitTest)
set(EXECUTABLE_58S DA-58-S_UnitTest)
set(EXECUTABLE_30D DA-30-D_UnitTest)
set(EXECUTABLE_30S DA-30-S_UnitTest)

add_definitions("-DJOB_NAME=\"$ENV{JOB_NAME}\"")
add_definitions("-DBUILD_NUMBER=\"$ENV{BUILD_NUMBER}\"")
add_definitions("-DGIT_COMMIT=\"$ENV{GIT_COMMIT}\"")
add_definitions("-DSERVO_VARIANT=\"$ENV{SERVO_VARIANT}\"")

# add some more flags for compiler
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -O2 -s")

# set output directory for generated binary file
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY bin)

# include directory with headers
include_directories(../fw/src ../fw/hal ../fw/drivers cmocka .)

# add sources files to project
set(sources1
  ../fw/src/conversion.c
  ../fw/src/counters.c
  ../fw/src/crc16.c
  ../fw/src/crc32.c
  ../fw/src/eeprom.c
  ../fw/src/global.c
  ../fw/src/hall.c
  ../fw/src/inetwork.c
  ../fw/src/logic.c
  ../fw/src/magnet.c
  ../fw/src/monitor.c
  ../fw/src/motor.c
  ../fw/src/mvs.c
  ../fw/src/notch.c
  ../fw/src/parser_can_volz.c
  ../fw/src/parser_rs485.c
  ../fw/src/redundancy.c
  ../fw/src/timeouts.c
  ../fw/src/utils.c
  ../fw/src/version.c
  ../fw/src/voting.c
  cmocka/cmocka.c
  da58_test.c
  wrappers.c
)

set(sources2
  ../fw/src/conversion.c
  ../fw/src/counters.c
  ../fw/src/crc16.c
  ../fw/src/crc32.c
  ../fw/src/eeprom.c
  ../fw/src/global.c
  ../fw/src/hall.c
  ../fw/src/logic.c
  ../fw/src/magnet.c
  ../fw/src/monitor.c
  ../fw/src/motor.c
  ../fw/src/mvs.c
  ../fw/src/notch.c
  ../fw/src/parser_can_volz.c
  ../fw/src/parser_rs485.c
  ../fw/src/timeouts.c
  ../fw/src/utils.c
  ../fw/src/version.c
  cmocka/cmocka.c
  da58_test.c
  wrappers.c
)

# generate binary file from the list of sources
add_executable(${EXECUTABLE_58D} ${sources1})
add_executable(${EXECUTABLE_58D_SD} ${sources1})
add_executable(${EXECUTABLE_58S} ${sources2})
add_executable(${EXECUTABLE_30D} ${sources1})
add_executable(${EXECUTABLE_30S} ${sources2})

target_link_libraries(${EXECUTABLE_58D} m)
target_link_libraries(${EXECUTABLE_58D_SD} m)
target_link_libraries(${EXECUTABLE_58S} m)
target_link_libraries(${EXECUTABLE_30D} m)
target_link_libraries(${EXECUTABLE_30S} m)

target_compile_definitions(${EXECUTABLE_58D} PUBLIC -DDEF_UNITTEST -DDEF_DUPLEX -DHAVE_SIGNAL_H -DDEF_DA58)
target_compile_definitions(${EXECUTABLE_58D_SD} PUBLIC -DDEF_UNITTEST -DDEF_DUPLEX -DHAVE_SIGNAL_H -DDEF_DA58)
target_compile_definitions(${EXECUTABLE_58S} PUBLIC -DDEF_UNITTEST -DDEF_SINGLE -DHAVE_SIGNAL_H -DDEF_DA58)
target_compile_definitions(${EXECUTABLE_30D} PUBLIC -DDEF_UNITTEST -DDEF_DUPLEX -DHAVE_SIGNAL_H -DDEF_DA30)
target_compile_definitions(${EXECUTABLE_30S} PUBLIC -DDEF_UNITTEST -DDEF_SINGLE -DHAVE_SIGNAL_H -DDEF_DA30)
