add_executable(seriald
  src/main.cpp
)

target_include_directories(seriald PRIVATE include src)
target_compile_options(seriald PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(seriald PRIVATE mc_proto mc_ipc mc_serial mc_core)
