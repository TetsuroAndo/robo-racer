add_executable(lidar_receiver
  main.cpp
  ../../src/LidarReceiver.cpp
  ../../src/lidar_to_esp.cpp
)

target_include_directories(lidar_receiver PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ../../src
  ../../src/config
  ../../lib/rplidar_sdk/sdk/include
)

target_compile_options(lidar_receiver PRIVATE -Wall -Wextra -Wpedantic)

target_link_libraries(lidar_receiver PRIVATE
  rplidar_sdk
  pthread
)
