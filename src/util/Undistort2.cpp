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
  if(std::isdigit(line.front())) {
    auto std_vec = Undistort2::fromStringToVectorX<double>(line);
    calib.mDistortion = Undistort2::toEigen(std_vec);

    const int index = static_cast<int>(calib.mDistortion.size());
    if(calib.mDistortion.size() == 4) {
      calib.mType = CalibrationType::PinHoleCamera;
    } else {
      calib.mType = CalibrationType::RadTanCamera;
    }
  } else {
    const auto itr = line.find(' ');
    const auto typeString = line.substr(0, itr);
    line = line.substr(itr+1);

    auto std_vec = Undistort2::fromStringToVectorX<double>(line);
    calib.mDistortion = Undistort2::toEigen(std_vec);
    if(typeString == "RadTan") {
      calib.mType = CalibrationType::RadTanCamera;
    } else if(typeString == "Pinhole") {
      calib.mType = CalibrationType::PinHoleCamera;
    } else {
      calib.mType = CalibrationType::None;
    }
  }

  std::getline(inputStream, line);

  calib.mInput << 1280, 1024;
  calib.mOutput << 640, 480;
  return calib;
}

CalibrationType Undistort2::toCalibrationType(std::string_view type) {
  if(type == "RadTan") {
    return CalibrationType::RadTanCamera;
  } if(type == "Pinhole") {
    return CalibrationType::PinHoleCamera;
  } else {
    return CalibrationType::None;
  }
}

} // namespace dso

