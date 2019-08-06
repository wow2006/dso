add_library(
  InsturctionSet
  INTERFACE
)

target_compile_options(
  InsturctionSet
  INTERFACE
  $<$<PLATFORM_ID:Linux>:-msse4.1;-march=native>
  $<$<PLATFORM_ID:Windows>:/arch:SSE4.1>
)

target_compile_definitions(
  InsturctionSet
  INTERFACE
  ENABLE_SSE
)

