# src/util/tests/CMakeLists.txt
get_target_property(
  CATCH_INCLUDE_DIRECTORY
  Catch
  INTERFACE_INCLUDE_DIRECTORIES
)

# tests-main {
add_library(
  tests-main
  OBJECT
  tests-main.cpp
)

target_compile_features(
  tests-main
  PUBLIC
  cxx_std_17
)

target_include_directories(
  tests-main
  PUBLIC
  ${CATCH_INCLUDE_DIRECTORY}
)
# }

set(test_targets testDatasetReader testUndistort testUndistort2)

foreach(test_target ${test_targets})

  add_executable(
    ${test_target}
    ${test_target}.cpp
    $<TARGET_OBJECTS:tests-main>
  )

  target_link_libraries(
    ${test_target}
    PUBLIC
    Catch
    dso
  )

  add_test(
    ${test_target}
    ${CMAKE_CURRENT_BINARY_DIR}/bin/${test_target}
  )

endforeach()

