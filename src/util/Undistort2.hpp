#pragma once
// STL
#include <istream>
#include <iostream>
// Eigen
#include <Eigen/Core>

namespace dso {

enum class CalibrationType {
  None = -1,
  PinHoleCamera,
  RadTanCamera
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

  template<typename T>
  static std::vector<T> fromStringToVectorX(const std::string& inputString) {
    std::vector<T> templateVector;
    std::stringstream inputStream{inputString};
    std::string value;

    while(std::getline(inputStream, value, ' ')) {
      templateVector.push_back(std::stod(value));
    }

    return templateVector;
  }

  template<typename T>
  static Eigen::Matrix<T, Eigen::Dynamic, 1> toEigen(std::vector<T>& vec) {
    Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, 1>> tempMatrix(vec.data(), vec.size(), 1);
    return tempMatrix;
  }

  static CalibrationType toCalibrationType(std::string_view type);

};

} // namespace dso
