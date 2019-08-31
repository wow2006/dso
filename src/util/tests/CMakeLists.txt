# src/util/tests/CMakeLists.txt
set(test_targets testDatasetReader testUndistort testUndistort2)

foreach(test_target ${test_targets})

  add_executable(
    ${test_target}
    ${test_target}.cpp
  )

  target_link_libraries(
    ${test_target}
    PUBLIC
    GTest::GTest
    GTest::Main
    dso
    $<$<CXX_COMPILER_ID:Clang>:-fsanitize=memory>
  )

  target_compile_options(
    ${test_target}
    PUBLIC
    $<$<CXX_COMPILER_ID:Clang>:-g;-Wno-global-constructors;-fsanitize=memory;-fno-omit-frame-pointer>
  )

  add_test(
    ${test_target}
    ${CMAKE_CURRENT_BINARY_DIR}/bin/${test_target}
  )

endforeach()

