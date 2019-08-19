# /spikes/CMakeLists.txt

set(executables spikesAddActiveFrame spikesPixelSelector)

foreach(executable ${executables})

  add_executable(
    ${executable}
    ${executable}.cpp
  )

  target_link_libraries(
    ${executable}
    PUBLIC
    dso
    fmt::fmt
    Boost::program_options
  )

  target_include_directories(
    ${executable}
    PUBLIC
    ${CMAKE_SOURCE_DIR}/thirdparty/GSL/include/
  )

endforeach()
