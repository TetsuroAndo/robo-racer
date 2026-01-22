file(GLOB RPI_SRCS CONFIGURE_DEPENDS
  "*.cpp"
)

add_executable(robo_racer ${RPI_SRCS})

target_include_directories(robo_racer PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/../lib/rplidar_sdk/sdk/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../lib/rplidar_sdk/sdk/src
  ${CMAKE_CURRENT_SOURCE_DIR}/../lib/mc_proto/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../lib/mc_ipc/include
)

target_compile_options(robo_racer PRIVATE -Wall -Wextra -Wpedantic)

set(RPLIDAR_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../lib/rplidar_sdk/output/${CMAKE_SYSTEM_NAME}/Release")
find_library(RPLIDAR_LIB sl_lidar_sdk PATHS ${RPLIDAR_LIB_DIR})
if (RPLIDAR_LIB)
  target_link_libraries(robo_racer PRIVATE ${RPLIDAR_LIB})
endif()

target_link_libraries(robo_racer PRIVATE pthread dl m)
