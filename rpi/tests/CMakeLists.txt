file(GLOB TEST_SOURCES
    tests_main.cpp
    test_planner_ftg_stop.cpp
    test_speed_policy.cpp
)

add_executable(rpi_tests
    ${TEST_SOURCES}
    ../src/planning/PlannerFTG.cpp
    ../src/control/SpeedPolicy.cpp
    ../src/control/SteeringSmoother.cpp
)

target_include_directories(rpi_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party
)
