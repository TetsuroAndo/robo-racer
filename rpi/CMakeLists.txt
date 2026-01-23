cmake_minimum_required(VERSION 3.16)
project(robo_racer_rpi LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(lib/mc_core)
add_subdirectory(lib/mc_proto)
add_subdirectory(lib/mc_serial)
add_subdirectory(lib/mc_ipc)

add_subdirectory(apps/seriald)
add_subdirectory(apps/serialctl)
add_subdirectory(src)
