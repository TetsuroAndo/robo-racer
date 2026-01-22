cmake_minimum_required(VERSION 3.16)
project(robo_racer_rpi LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release)
endif()

set(RPI_SRC_DIR "${CMAKE_CURRENT_LIST_DIR}/src")
set(RPI_LIB_DIR "${CMAKE_CURRENT_LIST_DIR}/lib")
set(RPLIDAR_SDK_DIR "${RPI_LIB_DIR}/rplidar_sdk")

file(GLOB_RECURSE SRC_RPI
	"${RPI_SRC_DIR}/*.c"
	"${RPI_SRC_DIR}/*.cc"
	"${RPI_SRC_DIR}/*.cpp"
)
file(GLOB_RECURSE SRC_LIB
	"${RPI_LIB_DIR}/mc_proto/src/*.c"
	"${RPI_LIB_DIR}/mc_proto/src/*.cc"
	"${RPI_LIB_DIR}/mc_proto/src/*.cpp"
	"${RPI_LIB_DIR}/mc_ipc/src/*.c"
	"${RPI_LIB_DIR}/mc_ipc/src/*.cc"
	"${RPI_LIB_DIR}/mc_ipc/src/*.cpp"
)

add_executable(robo-racer ${SRC_RPI} ${SRC_LIB})

target_include_directories(robo-racer SYSTEM PRIVATE
	"${RPLIDAR_SDK_DIR}/sdk/include"
	"${RPLIDAR_SDK_DIR}/sdk/src"
)
target_include_directories(robo-racer PRIVATE
	"${RPI_LIB_DIR}/mc_proto/include"
	"${RPI_LIB_DIR}/mc_ipc/include"
)

target_compile_options(robo-racer PRIVATE -Wall -Wextra -Wpedantic
	-Wno-zero-length-array)

find_package(Threads REQUIRED)
target_link_libraries(robo-racer PRIVATE Threads::Threads dl m)

set(RPLIDAR_LIB_DIR "${RPLIDAR_SDK_DIR}/output/${CMAKE_SYSTEM_NAME}/Release")
target_link_directories(robo-racer PRIVATE "${RPLIDAR_LIB_DIR}")
target_link_libraries(robo-racer PRIVATE sl_lidar_sdk)

set_target_properties(robo-racer PROPERTIES
	RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/.."
)
