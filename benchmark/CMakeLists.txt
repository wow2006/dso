# /benchmark/CMakeLists.txt
add_executable(
  banchmarkPixelSelector
  banchmarkPixelSelector.cpp
)

target_link_libraries(
  banchmarkPixelSelector
  PRIVATE
  dso
  benchmark::benchmark
  benchmark::benchmark_main
)

