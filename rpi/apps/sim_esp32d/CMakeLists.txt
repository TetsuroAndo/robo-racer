add_executable(sim_esp32d
  src/main.cpp
)

target_link_libraries(sim_esp32d
  mc_proto
  mc_serial
  mc_core
)

target_compile_options(sim_esp32d PRIVATE -Wall -Wextra -Wpedantic)
