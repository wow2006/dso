// Catch2
#include <gtest/gtest.h>
// Internal
#include "testUtils.hpp"
#include "../Undistort.hpp"

using namespace dso;

TEST(getUndistorterForFile, PassEmptyCalibPath) {
  const std::string calibFile    = "";
  const std::string gammaFile    = "";
  const std::string vignetteFile = "";

  RedirectCerr redirect1;
  RedirectCout redirect2;
  const auto undistort = Undistort::getUndistorterForFile(
                         calibFile, gammaFile, vignetteFile);
  redirect1.release();
  redirect2.release();

  ASSERT_TRUE(undistort == nullptr);
}

TEST(getUndistorterForFile, PassNonExistingCalibPath) {
  const std::string calibFile    = "/shit/calib.txt";
  const std::string gammaFile    = "";
  const std::string vignetteFile = "";

  RedirectCerr redirect1;
  RedirectCout redirect2;
  const auto undistort = Undistort::getUndistorterForFile(
                         calibFile, gammaFile, vignetteFile);
  redirect1.release();
  redirect2.release();

  ASSERT_TRUE(undistort == nullptr);
}

TEST(getUndistorterForFile, PassEmptyCalibFile) {
  TempDirectory tempDirectory;
  const std::string calibFile = createFile(tempDirectory,
                                           "calib.txt", "");

  const std::string gammaFile    = "";
  const std::string vignetteFile = "";

  RedirectCerr redirect1;
  RedirectCout redirect2;
  const auto undistort = Undistort::getUndistorterForFile(
                         calibFile, gammaFile, vignetteFile);
  redirect1.release();
  redirect2.release();

  ASSERT_TRUE(undistort == nullptr);
}

TEST(getUndistorterForFile, PassRadTanCalibFile) {
  TempDirectory tempDirectory;

  std::string_view calibFileContent = 
    "RadTan 0.5 0.5 0.5 0.5 0 0 0 0\n"
    "640 480\n"
    "crop\n"
    "640 480\n";

  const std::string calibFile = createFile(tempDirectory,
                                           "calib.txt",
                                           calibFileContent);

  const std::string gammaFile    = "";
  const std::string vignetteFile = "";

  RedirectCerr redirect1;
  RedirectCout redirect2;
  const auto pUndistort = Undistort::getUndistorterForFile(
                         calibFile, gammaFile, vignetteFile);
  redirect1.release();
  redirect2.release();

  ASSERT_FALSE(pUndistort == nullptr);

  delete pUndistort;
}

