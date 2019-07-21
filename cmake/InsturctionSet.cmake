add_library(
  InsturctionSet
  INTERFACE
)

target_compile_options(
  InsturctionSet
  INTERFACE
  $<$<PLATFORM_ID:Linux>:-msse2;-march=native>
)

target_compile_definitions(
  InsturctionSet
  INTERFACE
  ENABLE_SSE
)

