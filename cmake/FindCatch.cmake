add_library(
  Catch
  INTERFACE
)

target_include_directories(
  Catch
  SYSTEM INTERFACE
  ${CMAKE_SOURCE_DIR}/thirdparty/Catch/include
)

