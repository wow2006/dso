// STL
#include <sstream>
#include <iostream>
// GTest
#include <gtest/gtest.h>
// Internal
#include "util/Undistort2.hpp"

template<typename T>
inline bool near(T a, T b, T prec = static_cast<T>(0.001)) {
  return std::abs<T>(a - b) < prec;
}

TEST(Undistort2, ConvertStringToVectorXd_4) {
  const auto data = (Eigen::VectorXd(4) << 1, 2, 3, 4).finished();

  std::stringstream inputString;
  inputString << data(0) << ' ' << data(1) << ' ' << data(2) << ' ' <<  data(3);

  const auto output = dso::Undistort2::fromStringToVectorX<double>(inputString.str());

  ASSERT_TRUE(output.size() == data.size());
  ASSERT_TRUE(near(output[0], data(0)));
  ASSERT_TRUE(near(output[1], data(1)));
  ASSERT_TRUE(near(output[2], data(2)));
  ASSERT_TRUE(near(output[3], data(3)));
}

TEST(Undistort2, ConvertStringToVectorXd_5) {
  const auto data = (Eigen::VectorXd(5) << 1, 2, 3, 4, 5).finished();

  std::stringstream inputString;
  inputString << data(0) << ' ' << data(1) << ' ' << data(2) << ' ' <<  data(3) << ' ' << data(4);

  const auto output = dso::Undistort2::fromStringToVectorX<double>(inputString.str());

  ASSERT_TRUE(output.size() == data.size());
  ASSERT_TRUE(near(output[0], data(0)));
  ASSERT_TRUE(near(output[1], data(1)));
  ASSERT_TRUE(near(output[2], data(2)));
  ASSERT_TRUE(near(output[3], data(3)));
  ASSERT_TRUE(near(output[4], data(4)));
}

TEST(Undistort2, ConvertVector_doube_4_To_VectorXd_5) {
  auto vec = std::vector<double>{1, 2, 3, 4};
  Eigen::VectorXd result(vec.size());
  for(int i = 0; i < vec.size(); ++i) {
    result(i) = vec[i];
  }

  const auto output = dso::Undistort2::toEigen(vec);

  ASSERT_TRUE(output.size() == result.size());
  ASSERT_TRUE(near(output[0], result(0)));
  ASSERT_TRUE(near(output[1], result(1)));
  ASSERT_TRUE(near(output[2], result(2)));
  ASSERT_TRUE(near(output[3], result(3)));
}

TEST(Undistort2, LoadPinHoleCalibrationFromStream) {
  std::stringstream pinHoleCameraCalibration;

  constexpr unsigned int calibrationCount = 4;
  constexpr std::array<double, calibrationCount> calibrationValues{1, 2, 3, 4};
  Eigen::VectorXd calibrationEigen(calibrationCount);

  int index = 0;
  for(const auto value : calibrationValues) {
    calibrationEigen(index++) = value;
    pinHoleCameraCalibration << value << ' ';
  }
  pinHoleCameraCalibration << '\n';

  const Eigen::Vector2i inputDim(1280, 1024);
  pinHoleCameraCalibration << inputDim(0) << ' ' << inputDim(1) << '\n';
  pinHoleCameraCalibration << "crop\n";

  const Eigen::Vector2i outputDim(640, 480);
  pinHoleCameraCalibration << outputDim(0) << ' ' << outputDim(1) << '\n';


  const auto calibration = dso::Undistort2::loadCalibration(pinHoleCameraCalibration);

  ASSERT_TRUE(calibration.mType == dso::CalibrationType::PinHoleCamera);
  ASSERT_TRUE(calibration.mDistortion.size() == calibrationCount);
  ASSERT_TRUE(calibration.mInput == inputDim);
  ASSERT_TRUE(calibration.mOutput == outputDim);
}

TEST(Undistort2, LoadRadtanCalibrationFromStreamStartWithString) {
  std::stringstream RadTanCameraCalibration;

  constexpr unsigned int calibrationCount = 5;
  constexpr std::array<double, calibrationCount> calibrationValues{1, 2, 3, 4, 5};
  Eigen::VectorXd calibrationEigen(calibrationCount);

  RadTanCameraCalibration << "Pinhole ";
  int index = 0;
  for(const auto value : calibrationValues) {
    calibrationEigen(index++) = value;
    RadTanCameraCalibration << value << ' ';
  }
  RadTanCameraCalibration << '\n';

  const Eigen::Vector2i inputDim(1280, 1024);
  RadTanCameraCalibration << inputDim(0) << ' ' << inputDim(1) << '\n';
  RadTanCameraCalibration << "crop\n";

  const Eigen::Vector2i outputDim(640, 480);
  RadTanCameraCalibration << outputDim(0) << ' ' << outputDim(1) << '\n';


  const auto calibration = dso::Undistort2::loadCalibration(RadTanCameraCalibration);

  ASSERT_TRUE(calibration.mType == dso::CalibrationType::PinHoleCamera);
  ASSERT_TRUE(calibration.mDistortion.size() == calibrationCount);
  ASSERT_TRUE(!near(calibration.mDistortion(calibrationCount-1), 0.));
  ASSERT_TRUE(calibration.mInput == inputDim);
  ASSERT_TRUE(calibration.mOutput == outputDim);
}

TEST(Undistort2, LoadRadTanCalibrationFromStream) {
  std::stringstream RadTanCameraCalibration;

  constexpr unsigned int calibrationCount = 5;
  constexpr std::array<double, calibrationCount> calibrationValues{1, 2, 3, 4, 5};
  Eigen::VectorXd calibrationEigen(calibrationCount);

  int index = 0;
  for(const auto value : calibrationValues) {
    calibrationEigen(index++) = value;
    RadTanCameraCalibration << value << ' ';
  }
  RadTanCameraCalibration << '\n';

  const Eigen::Vector2i inputDim(1280, 1024);
  RadTanCameraCalibration << inputDim(0) << ' ' << inputDim(1) << '\n';
  RadTanCameraCalibration << "crop\n";

  const Eigen::Vector2i outputDim(640, 480);
  RadTanCameraCalibration << outputDim(0) << ' ' << outputDim(1) << '\n';


  const auto calibration = dso::Undistort2::loadCalibration(RadTanCameraCalibration);

  ASSERT_TRUE(calibration.mType == dso::CalibrationType::RadTanCamera);
  ASSERT_TRUE(calibration.mDistortion.size() == calibrationCount);
  ASSERT_TRUE(!near(calibration.mDistortion(calibrationCount-1), 0.));
  ASSERT_TRUE(calibration.mInput == inputDim);
  ASSERT_TRUE(calibration.mOutput == outputDim);
}

TEST(Undistort2, LoadRadTanCalibrationFromStreamStartWithString) {
  std::stringstream RadTanCameraCalibration;

  constexpr unsigned int calibrationCount = 5;
  constexpr std::array<double, calibrationCount> calibrationValues{1, 2, 3, 4, 5};
  Eigen::VectorXd calibrationEigen(calibrationCount);

  RadTanCameraCalibration << "RadTan ";
  int index = 0;
  for(const auto value : calibrationValues) {
    calibrationEigen(index++) = value;
    RadTanCameraCalibration << value << ' ';
  }
  RadTanCameraCalibration << '\n';

  const Eigen::Vector2i inputDim(1280, 1024);
  RadTanCameraCalibration << inputDim(0) << ' ' << inputDim(1) << '\n';
  RadTanCameraCalibration << "crop\n";

  const Eigen::Vector2i outputDim(640, 480);
  RadTanCameraCalibration << outputDim(0) << ' ' << outputDim(1) << '\n';


  const auto calibration = dso::Undistort2::loadCalibration(RadTanCameraCalibration);

  ASSERT_TRUE(calibration.mType == dso::CalibrationType::RadTanCamera);
  ASSERT_TRUE(calibration.mDistortion.size() == calibrationCount);
  ASSERT_TRUE(!near(calibration.mDistortion(calibrationCount-1), 0.));
  ASSERT_TRUE(calibration.mInput == inputDim);
  ASSERT_TRUE(calibration.mOutput == outputDim);
}

TEST(Undistort2, FromCalibrationStringTypeToEnumRadTanType) {
  constexpr std::string_view typeString = "RadTan";

  ASSERT_TRUE(dso::Undistort2::toCalibrationType(typeString) == dso::CalibrationType::RadTanCamera);
}

TEST(Undistort2, FromCalibrationStringTypeToEnumPinholeType) {
  constexpr std::string_view typeString = "Pinhole";

  ASSERT_TRUE(dso::Undistort2::toCalibrationType(typeString) == dso::CalibrationType::PinHoleCamera);
}

TEST(Undistort2, FromCalibrationStringTypeToEnumNoneType) {
  constexpr std::string_view typeString = "None";

  ASSERT_TRUE(dso::Undistort2::toCalibrationType(typeString) == dso::CalibrationType::None);
}

TEST(Undistort2, FromEmptyStringTypeToEnumNoneType) {
  constexpr std::string_view typeString;

  ASSERT_TRUE(dso::Undistort2::toCalibrationType(typeString) == dso::CalibrationType::None);
}
