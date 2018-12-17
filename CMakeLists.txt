cmake_minimum_required(VERSION 3.9)
project(DSO CXX)

set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH    lib)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# required libraries
find_package(SuiteParse REQUIRED)
find_package(Eigen3     REQUIRED)
find_package(Boost      COMPONENTS system thread program_options)
find_package(Pangolin   REQUIRED)
find_package(OpenCV     REQUIRED imgcodecs highgui)

# ===========================================
# compile main library.

# Sources files
set(
  SOURCE_FILES
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
  ${PROJECT_SOURCE_DIR}/src/OptimizationBackend/EnergyFunctional.cpp
  ${PROJECT_SOURCE_DIR}/src/OptimizationBackend/AccumulatedTopHessian.cpp
  ${PROJECT_SOURCE_DIR}/src/OptimizationBackend/AccumulatedSCHessian.cpp
  ${PROJECT_SOURCE_DIR}/src/OptimizationBackend/EnergyFunctionalStructs.cpp
  ${PROJECT_SOURCE_DIR}/src/util/settings.cpp
  ${PROJECT_SOURCE_DIR}/src/util/Undistort.cpp
  ${PROJECT_SOURCE_DIR}/src/util/globalCalib.cpp
  # dso_pangolin_SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/src/IOWrapper/Pangolin/KeyFrameDisplay.cpp
  ${PROJECT_SOURCE_DIR}/src/IOWrapper/Pangolin/PangolinDSOViewer.cpp
  # dso_opencv_SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/src/IOWrapper/OpenCV/ImageDisplay_OpenCV.cpp
  ${PROJECT_SOURCE_DIR}/src/IOWrapper/OpenCV/ImageRW_OpenCV.cpp
)

add_library(
  dso
  STATIC
  ${SOURCE_FILES}
)

target_include_directories(
  dso
  PUBLIC
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/thirdparty/Sophus
  PRIVATE
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
  Eigen3::Eigen
  Boost::thread
  ${OpenCV_LIBS}
)

target_compile_features(
  dso
  PUBLIC
  cxx_std_17
)

target_compile_options(
  dso
  PUBLIC
  $<$<CXX_COMPILER_ID:Clang>:-Weverything -Wno-c++98-compat-pedantic>
  $<$<CXX_COMPILER_ID:GNU>:-Wall -W>
)
# ===========================================

# ===========================================
# DEMO
add_executable(
  dso_dataset
  #${CMAKE_SOURCE_DIR}/src/main_dso_pangolin.cpp
  ${CMAKE_SOURCE_DIR}/main.cpp
)

target_link_libraries(
  dso_dataset
  PUBLIC
  dso
  #${Pangolin_LIBRARIES}
  Boost::program_options
)
# ===========================================

