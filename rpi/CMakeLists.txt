cmake_minimum_required(VERSION 3.16)
project(robo_racer_rpi LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(ROBO_RACER_TSAN "Enable ThreadSanitizer" OFF)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../shared/config/include)

if(ROBO_RACER_TSAN)
	if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
		add_compile_options(-fsanitize=thread -fno-omit-frame-pointer -g)
		add_link_options(-fsanitize=thread)
	endif()
endif()

add_subdirectory(lib/mc_core)
add_subdirectory(lib/mc_proto)
add_subdirectory(lib/mc_serial)
add_subdirectory(lib/mc_ipc)

add_subdirectory(apps/seriald)
add_subdirectory(apps/serialctl)
add_subdirectory(apps/sim_esp32d)
add_subdirectory(apps/metricsd)
add_subdirectory(apps/lidar_received)
add_subdirectory(src)
