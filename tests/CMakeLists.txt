set(ORD_TEST_SOURCES
  test_main.cpp
  test_tire.cpp
  test_drivetrain.cpp
  test_vehicle.cpp
  test_assists.cpp
  test_benchmarks.cpp
  test_host.cpp
  test_contact.cpp
  test_lod.cpp
  test_robustness.cpp
  test_baseline.cpp
  test_alloc.cpp
  test_single_track.cpp
)
add_executable(ord_tests ${ORD_TEST_SOURCES})
target_link_libraries(ord_tests PRIVATE ord_scenarios)
target_compile_options(ord_tests PRIVATE ${ORD_WARNINGS})
target_compile_definitions(ord_tests PRIVATE ORD_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

# The runner prints per-case results; pass a substring to run a subset: ord_tests "drivetrain:"
add_test(NAME ord_tests COMMAND ord_tests)
