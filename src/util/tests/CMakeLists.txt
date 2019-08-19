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

# DatasetReader {
add_executable(
  testDatasetReader
  testDatasetReader.cpp
  $<TARGET_OBJECTS:tests-main>
)

target_link_libraries(
  testDatasetReader
  PUBLIC
  Catch
  dso
)

add_test(
  testDatasetReader
  ${CMAKE_CURRENT_BINARY_DIR}/bin/testDatasetReader
)
# }

# Undistort {
add_executable(
  testUndistort
  testUndistort.cpp
  ../Undistort.cpp
  $<TARGET_OBJECTS:tests-main>
)

target_link_libraries(
  testUndistort 
  PUBLIC
  Catch
  dso
)

add_test(
  testUndistort 
  ${CMAKE_CURRENT_BINARY_DIR}/bin/testUndistort
)
# }

