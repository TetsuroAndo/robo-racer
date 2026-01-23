add_library(mc_core
  src/Log.cpp
  src/Time.cpp
  src/Process.cpp
)

target_include_directories(mc_core PUBLIC include)
target_compile_options(mc_core PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(mc_core PRIVATE pthread)
