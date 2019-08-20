// STL
#include <sstream>
// Catch2
#include <catch/catch.hpp>
// Internal
#include "util/Undistort2.hpp"

template<typename T>
inline bool near(T a, T b, T prec = static_cast<T>(0.001)) {
  return std::abs<T>(a - b) < prec;
}

TEST_CASE("load calibration from stream", "[Undistort2]" ) {
  constexpr unsigned int calibrationCount = 5;
  const Eigen::Vector2i inputDim(1280, 1024);
  const Eigen::Vector2i outputDim(640, 480);
  std::stringstream pinHoleCameraCalibration;

  const auto calibration = dso::Undistort2::loadCalibration(pinHoleCameraCalibration);

  REQUIRE(calibration.mType == dso::CalibrationType::PinHoleCamera);
  REQUIRE(calibration.mDistortion.size() == calibrationCount);
  REQUIRE(near(calibration.mDistortion(calibrationCount-1), 0.));
  REQUIRE(calibration.mInput == inputDim);
  REQUIRE(calibration.mOutput == outputDim);
}

