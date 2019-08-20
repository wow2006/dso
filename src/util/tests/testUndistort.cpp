// Catch2
#include <catch/catch.hpp>
// Internal
#include "testUtils.hpp"
#include "../Undistort.hpp"

using namespace dso;

TEST_CASE("pass empty calib path",
          "[Undistort, getUndistorterForFile]" ) {
  const std::string calibFile    = "";
  const std::string gammaFile    = "";
  const std::string vignetteFile = "";

  RedirectCerr redirect1;
  RedirectCout redirect2;
  const auto undistort = Undistort::getUndistorterForFile(
                         calibFile, gammaFile, vignetteFile);
  redirect1.release();
  redirect2.release();

  REQUIRE(undistort == nullptr);
}

TEST_CASE("pass non existing calib file",
          "[Undistort, getUndistorterForFile]" ) {
  const std::string calibFile    = "/shit/calib.txt";
  const std::string gammaFile    = "";
  const std::string vignetteFile = "";

  RedirectCerr redirect1;
  RedirectCout redirect2;
  const auto undistort = Undistort::getUndistorterForFile(
                         calibFile, gammaFile, vignetteFile);
  redirect1.release();
  redirect2.release();

  REQUIRE(undistort == nullptr);
}

TEST_CASE("pass empty calib file",
          "[Undistort, getUndistorterForFile]" ) {
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

  REQUIRE(undistort == nullptr);
}

TEST_CASE("pass RadTan calib file",
          "[Undistort, getUndistorterForFile]" ) {
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

  REQUIRE_FALSE(pUndistort == nullptr);

  delete pUndistort;
}

