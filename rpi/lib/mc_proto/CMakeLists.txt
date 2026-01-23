add_library(mc_proto
  ${CMAKE_SOURCE_DIR}/../shared/proto/src/mcproto.cpp
)

target_include_directories(mc_proto PUBLIC
	${CMAKE_SOURCE_DIR}/../shared/proto/include
	${CMAKE_SOURCE_DIR}/../shared/proto
)
target_compile_options(mc_proto PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(mc_proto PUBLIC mc_core)
