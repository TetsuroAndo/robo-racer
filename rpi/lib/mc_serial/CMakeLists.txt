add_library(mc_serial
  src/UartPosix.cpp
)

target_include_directories(mc_serial PUBLIC include)
target_compile_options(mc_serial PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(mc_serial PUBLIC mc_core)
