/**
 * This file is part of DSO.
 *
 * Copyright 2016 Technical University of Munich and Intel.
 * Developed by Jakob Engel <engelj at in dot tum dot de>,
 * for more information see <http://vision.in.tum.de/dso>.
 * If you use this code, please cite the respective publications as
 * listed on the above website.
 *
 * DSO is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * DSO is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with DSO. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once
// STL
#include <string>
// Boost
#include <boost/filesystem.hpp>
// Eigen
#include <Eigen/Core>
// Internal
#include "util/ImageAndExposure.h"
#include "util/Undistort.hpp"

enum DirectoryReturn { NotExist = -1, Empty = 0 };

/**
 * @brief check end of string
 *
 * @param fullString input string
 * @param ending end string to check
 *
 * @return true compare
 */
bool hasEnding(std::string const &fullString, std::string const &ending);

/**
 * @brief read files names in directory
 *
 * @param dir   full path directory
 * @param files vector of string of files found in directory
 *
 * @return number of files found in directory
 *         @see DirectoryReturn
 */
int getdir(std::string_view dir, std::vector<std::string> &files);

struct PrepImageItem {
  int id;
  bool isQueud;
  dso::ImageAndExposure *pt;

  /**
   * @brief Consturctor take Image Id
   *
   * @param _id Image Id
   */
  PrepImageItem(int _id) : id{_id}, isQueud{false}, pt{nullptr} {}

  /**
   * @brief Release Image pointer
   */
  void release();
};

class ImageFolderReader {
public:
  /**
   * @brief Consturctor
   *
   * @param path_ images directory
   * @param calibFile_ calibration directory
   * @param gammaFile gamma file directory
   * @param vignetteFile vignette file directory
   */
  ImageFolderReader(std::string path_, std::string calibFile_,
                    std::string gammaFile, std::string vignetteFile);

  ~ImageFolderReader();

  static bool checkInputFile(const std::string& inputFileName, const std::string& extension);

  void readZipFile();

  void readImages();

  void readCalibration(const std::string& gammaFile, const std::string& vignetteFile);

  Eigen::VectorXf getOriginalCalib() {
    return undistort->getOriginalParameter().cast<float>();
  }

  Eigen::Vector2i getOriginalDimensions() {
    return undistort->getOriginalSize();
  }

  void getCalibMono(Eigen::Matrix3f &K, int &w, int &h) {
    K = undistort->getK().cast<float>();
    w = undistort->getSize()[0];
    h = undistort->getSize()[1];
  }

  void setGlobalCalibration();

  int getNumImages() { return static_cast<int>(files.size()); }

  double getTimestamp(int id) {
    if (timestamps.size() == 0) {
      return id * 0.1;
    }

    if (id >= static_cast<int>(timestamps.size())) {
      return 0;
    }

    if (id < 0) {
      return 0;
    }

    return timestamps[static_cast<std::size_t>(id)];
  }

  void prepImage(int id, bool as8U = false) {
    (void)id;
    (void)as8U;
  }

  dso::MinimalImageB *getImageRaw(int id) {
    return getImageRaw_internal(id, 0);
  }

  dso::ImageAndExposure *getImage(int id, bool forceLoadDirectly = false) {
    (void)forceLoadDirectly;
    return getImage_internal(id, 0);
  }

  float *getPhotometricGamma() {
    if (undistort == nullptr || undistort->photometricUndist == nullptr) {
      return nullptr;
    }

    return undistort->photometricUndist->getG();
  }

  // undistorter. [0] always exists, [1-2] only when MT is enabled.
  dso::Undistort *undistort;

#ifndef TESTING
private:
#endif
  dso::MinimalImageB *getImageRaw_internal(int id, int unused);

  dso::ImageAndExposure *getImage_internal(int id, int unused);

  void loadTimestamps();

  std::vector<dso::ImageAndExposure *> preloadedImages;
  std::vector<std::string> files;
  std::vector<double> timestamps;
  std::vector<float> exposures;

  int width, height;
  int widthOrg, heightOrg;

  std::string path;
  std::string calibfile;

  bool isZipped;

#ifdef HAS_ZIPLIB
  zip_t *ziparchive;
  char *databuffer;
#endif
};
