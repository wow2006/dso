// STL
#include <vector>
#include <iostream>
// Internal
#include "util/Undistort2.hpp"

namespace dso {

Undistort2::Undistort2() = default;

Undistort2::~Undistort2() = default;

CalibrationData Undistort2::loadCalibration(std::istream& inputStream) {
  std::string line;
  std::getline(inputStream, line);

  CalibrationData calib;
  calib.mType = CalibrationType::PinHoleCamera;
  calib.mDistortion = Eigen::VectorXd::Zero(5);
  calib.mInput << 1280, 1024;
  calib.mOutput << 640, 480;
  return calib;
}

} // namespace dso

