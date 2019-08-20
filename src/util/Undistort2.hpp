#pragma once
// STL
#include <istream>
// Eigen
#include <Eigen/Core>

namespace dso {

enum class CalibrationType {
  None = -1,
  PinHoleCamera
};

struct CalibrationData {
  Eigen::VectorXd mDistortion;
  CalibrationType mType;
  Eigen::Vector2i mInput;
  Eigen::Vector2i mOutput;

};

class Undistort2 {
public:
  Undistort2();

  ~Undistort2();

  static CalibrationData loadCalibration(std::istream& inputStream);

};

} // namespace dso

