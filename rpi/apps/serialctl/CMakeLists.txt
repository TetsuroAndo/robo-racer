add_executable(serialctl main.cpp)

target_link_libraries(serialctl
  mc_proto
  mc_ipc
)

target_compile_options(serialctl PRIVATE -Wall -Wextra -Wpedantic)
