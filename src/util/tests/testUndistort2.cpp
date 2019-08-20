// STL
#include <sstream>
#include <iostream>
// Catch2
#include <catch/catch.hpp>
// Internal
#include "util/Undistort2.hpp"

template<typename T>
inline bool near(T a, T b, T prec = static_cast<T>(0.001)) {
  return std::abs<T>(a - b) < prec;
}

TEST_CASE("Convert String to VectorXd(4)", "[Undistort2]" ) {
  const auto data = (Eigen::VectorXd(4) << 1, 2, 3, 4).finished();

  std::stringstream inputString;
  inputString << data(0) << ' ' << data(1) << ' ' << data(2) << ' ' <<  data(3);

  const auto output = dso::Undistort2::fromStringToVectorX<double>(inputString.str());

  REQUIRE(output.size() == data.size());
  REQUIRE(near(output[0], data(0)));
  REQUIRE(near(output[1], data(1)));
  REQUIRE(near(output[2], data(2)));
  REQUIRE(near(output[3], data(3)));
}

TEST_CASE("Convert String to VectorXd(5)", "[Undistort2]" ) {
  const auto data = (Eigen::VectorXd(5) << 1, 2, 3, 4, 5).finished();

  std::stringstream inputString;
  inputString << data(0) << ' ' << data(1) << ' ' << data(2) << ' ' <<  data(3) << ' ' << data(4);

  const auto output = dso::Undistort2::fromStringToVectorX<double>(inputString.str());

  REQUIRE(output.size() == data.size());
  REQUIRE(near(output[0], data(0)));
  REQUIRE(near(output[1], data(1)));
  REQUIRE(near(output[2], data(2)));
  REQUIRE(near(output[3], data(3)));
  REQUIRE(near(output[4], data(4)));
}

TEST_CASE("Convert vector<doube>(4) to VectorXd(5)", "[Undistort2]" ) {
  auto vec = std::vector<double>{1, 2, 3, 4};
  Eigen::VectorXd result(vec.size());
  for(int i = 0; i < vec.size(); ++i) {
    result(i) = vec[i];
  }

  const auto output = dso::Undistort2::toEigen(vec);

  REQUIRE(output.size() == result.size());
  REQUIRE(near(output[0], result(0)));
  REQUIRE(near(output[1], result(1)));
  REQUIRE(near(output[2], result(2)));
  REQUIRE(near(output[3], result(3)));
}

TEST_CASE("load PinHole calibration from stream", "[Undistort2]" ) {
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

TEST_CASE("load RadTan calibration from stream", "[Undistort2]" ) {
  constexpr unsigned int calibrationCount = 5;
  const Eigen::Vector2i inputDim(1280, 1024);
  const Eigen::Vector2i outputDim(640, 480);
  std::stringstream RadTanCameraCalibration;

  const auto calibration = dso::Undistort2::loadCalibration(RadTanCameraCalibration);

  REQUIRE(calibration.mType == dso::CalibrationType::RadTanCamera);
  REQUIRE(calibration.mDistortion.size() == calibrationCount);
  REQUIRE(!near(calibration.mDistortion(calibrationCount-1), 0.));
  REQUIRE(calibration.mInput == inputDim);
  REQUIRE(calibration.mOutput == outputDim);
}

