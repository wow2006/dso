#include <catch/catch.hpp>

#include "testUtils.hpp"

#include "../Undistort.hpp"


using namespace dso;


TEST_CASE("pass empty calib file",
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

TEST_CASE("pass non existing calib file",
          "[Undistort, getUndistorterForFile]" ) {
}
