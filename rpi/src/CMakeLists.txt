file(GLOB_RECURSE RPI_SRCS CONFIGURE_DEPENDS
  "*.cpp"
)

set(ROBO_RACER_NAME "robo-racer" CACHE STRING "Output binary name")
add_executable(${ROBO_RACER_NAME} ${RPI_SRCS})

target_include_directories(${ROBO_RACER_NAME} SYSTEM PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../lib/rplidar_sdk/sdk/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../lib/rplidar_sdk/sdk/src
)
target_include_directories(${ROBO_RACER_NAME} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/../lib/mc_core/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../shared/proto/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../shared/proto
  ${CMAKE_CURRENT_SOURCE_DIR}/../lib/mc_ipc/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../shared/common/include
)

target_compile_options(${ROBO_RACER_NAME} PRIVATE -Wall -Wextra -Wpedantic
  -Wno-zero-length-array)

set(RPLIDAR_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../lib/rplidar_sdk/output/${CMAKE_SYSTEM_NAME}/Release")
find_library(RPLIDAR_LIB sl_lidar_sdk PATHS ${RPLIDAR_LIB_DIR})
if (RPLIDAR_LIB)
  target_link_libraries(${ROBO_RACER_NAME} PRIVATE ${RPLIDAR_LIB})
endif()

target_link_libraries(${ROBO_RACER_NAME} PRIVATE mc_proto mc_ipc dl m)

set_target_properties(${ROBO_RACER_NAME} PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../.."
)
