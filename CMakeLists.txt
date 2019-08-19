cmake_minimum_required(VERSION 3.9)
project(DSO CXX)

set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH    lib)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

include(${CMAKE_SOURCE_DIR}/cmake/Doxygen.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/InsturctionSet.cmake)

option(ENABLE_TESTING "Enable test cases" OFF)
option(ENABLE_SPIKES  "Enable spikes"     OFF)

# required libraries
find_package(SuiteParse    REQUIRED)
find_package(Eigen3        REQUIRED)
find_package(Boost         COMPONENTS system thread program_options filesystem REQUIRED)
find_package(Pangolin      REQUIRED)
find_package(OpenCV        REQUIRED imgcodecs highgui imgproc)
find_package(fmt           REQUIRED)
find_package(easy_profiler REQUIRED)

# ===========================================
# compile main library.

add_library(
  dso
  STATIC
)

target_sources(
  dso
  PRIVATE
  ${PROJECT_SOURCE_DIR}/src/FullSystem/FullSystem.cpp
  ${PROJECT_SOURCE_DIR}/src/FullSystem/FullSystemOptimize.cpp
  ${PROJECT_SOURCE_DIR}/src/FullSystem/FullSystemOptPoint.cpp
  ${PROJECT_SOURCE_DIR}/src/FullSystem/FullSystemDebugStuff.cpp
  ${PROJECT_SOURCE_DIR}/src/FullSystem/FullSystemMarginalize.cpp
  ${PROJECT_SOURCE_DIR}/src/FullSystem/Residuals.cpp
  ${PROJECT_SOURCE_DIR}/src/FullSystem/CoarseTracker.cpp
  ${PROJECT_SOURCE_DIR}/src/FullSystem/CoarseInitializer.cpp
  ${PROJECT_SOURCE_DIR}/src/FullSystem/ImmaturePoint.cpp
  ${PROJECT_SOURCE_DIR}/src/FullSystem/HessianBlocks.cpp
  ${PROJECT_SOURCE_DIR}/src/FullSystem/PixelSelector2.cpp
  # OptimizationBackend
  ${PROJECT_SOURCE_DIR}/src/OptimizationBackend/EnergyFunctional.cpp
  ${PROJECT_SOURCE_DIR}/src/OptimizationBackend/AccumulatedTopHessian.cpp
  ${PROJECT_SOURCE_DIR}/src/OptimizationBackend/AccumulatedSCHessian.cpp
  ${PROJECT_SOURCE_DIR}/src/OptimizationBackend/EnergyFunctionalStructs.cpp
  # util
  ${PROJECT_SOURCE_DIR}/src/util/settings.cpp
  ${PROJECT_SOURCE_DIR}/src/util/Undistort.cpp
  ${PROJECT_SOURCE_DIR}/src/util/globalCalib.cpp
  ${PROJECT_SOURCE_DIR}/src/util/DatasetReader.cpp
  # dso_pangolin_SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/src/IOWrapper/Pangolin/KeyFrameDisplay.cpp
  ${PROJECT_SOURCE_DIR}/src/IOWrapper/Pangolin/PangolinDSOViewer.cpp
  # dso_opencv_SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/src/IOWrapper/OpenCV/ImageDisplay_OpenCV.cpp
  ${PROJECT_SOURCE_DIR}/src/IOWrapper/OpenCV/ImageRW_OpenCV.cpp
)

target_include_directories(
  dso
  PUBLIC
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/thirdparty/Sophus
  SYSTEM PRIVATE
  ${CSPARSE_INCLUDE_DIR}
  ${CHOLMOD_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  # thirdparty
  ${PROJECT_SOURCE_DIR}/thirdparty/sse2neon
  # Pangolin
  ${Pangolin_INCLUDE_DIRS}
)

target_link_libraries(
  dso
  PUBLIC
  InsturctionSet
  Eigen3::Eigen
  Boost::thread
  Boost::filesystem
  ${OpenCV_LIBS}
  fmt::fmt
  easy_profiler
)

target_compile_features(
  dso
  PUBLIC
  cxx_std_17
)

target_compile_options(
  dso
  PUBLIC
  $<$<CXX_COMPILER_ID:Clang>:-Weverything;-Wno-c++98-compat-pedantic;-fcolor-diagnostics;-Wno-unused-macros>
  $<$<CXX_COMPILER_ID:GNU>:-Wall;-W>
)

# ===========================================

# ===========================================
# DEMO
add_executable(
  dso_dataset
  ${CMAKE_SOURCE_DIR}/src/main_dso_pangolin.cpp
)

target_link_libraries(
  dso_dataset
  PUBLIC
  dso
  ${Pangolin_LIBRARIES}
)

# TODO(Hussein): Get directory from target
set(
  RUN_EXEC
  ${CMAKE_BINARY_DIR}/bin/dso_dataset
)

configure_file(${CMAKE_SOURCE_DIR}/scripts/run.sh.in run.sh @ONLY)
# ===========================================

add_executable(
  dso_DEMO
  ${CMAKE_SOURCE_DIR}/src/main.cpp
)

target_link_libraries(
  dso_DEMO
  PUBLIC
  dso
  ${Pangolin_LIBRARIES}
  Boost::program_options
)

if(ENABLE_TESTING)
  find_package(Catch REQUIRED)

  enable_testing()

  add_subdirectory(${CMAKE_SOURCE_DIR}/src/util/tests)
endif()

if(ENABLE_SPIKES)
  add_subdirectory(spikes)
endif()

