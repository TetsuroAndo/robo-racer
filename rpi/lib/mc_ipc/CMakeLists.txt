add_library(mc_ipc
  src/ipc_dgram.cpp
)

target_include_directories(mc_ipc PUBLIC include)
target_compile_options(mc_ipc PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(mc_ipc PUBLIC mc_core)
