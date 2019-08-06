# /spikes/CMakeLists.txt

add_executable(
  spikesAddActiveFrame
  spikesAddActiveFrame.cpp
)

target_link_libraries(
  spikesAddActiveFrame
  PUBLIC
  dso
  fmt::fmt
  Boost::program_options
)

target_include_directories(
  spikesAddActiveFrame
  PUBLIC
  ${CMAKE_SOURCE_DIR}/thirdparty/GSL/include/
)

