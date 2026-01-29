add_executable(metricsd
  src/main.cpp
)

target_link_libraries(metricsd
  mc_core
  mc_proto
  mc_ipc
)

target_compile_options(metricsd PRIVATE -Wall -Wextra -Wpedantic)
