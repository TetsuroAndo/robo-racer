add_executable(lidar_received
  main.cpp
  LidarScanData.cpp
  ../../src/Process.cpp
  ../../src/Sender.cpp
  ../../src/lidar_to_esp.cpp
)

target_include_directories(lidar_received PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/config
  ../../src
  ../../src/config
  ../../lib/mc_core/include
)
target_include_directories(lidar_received SYSTEM PRIVATE
  ../../lib/rplidar_sdk/sdk/include
)

target_compile_options(lidar_received PRIVATE -Wall -Wextra -Wpedantic
  -Wno-zero-length-array)

set(RPLIDAR_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../lib/rplidar_sdk/output/${CMAKE_SYSTEM_NAME}/Release")
find_library(RPLIDAR_LIB sl_lidar_sdk PATHS ${RPLIDAR_LIB_DIR})

target_link_libraries(lidar_received PRIVATE
  ${RPLIDAR_LIB}
  mc_ipc
  mc_proto
  pthread
)
