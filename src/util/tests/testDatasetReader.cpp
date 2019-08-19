#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>
// Catch
#define TESTING 1
#include <catch/catch.hpp>
// Internal
#include "util/DatasetReader.h"
//
#include "testUtils.hpp"

TEST_CASE("Pass input string ABC and endsWithC", "[hasEnding]") {
  const std::string fullString = "ABC";
  const std::string endsWithC  = "C";

  REQUIRE(hasEnding(fullString, endsWithC) == true);
}

TEST_CASE("Pass non existing directory", "[getdir]") {
  const std::string nonExistingDirectory = "/shit";
  std::vector<std::string> files;

  REQUIRE(getdir(nonExistingDirectory, files) == NotExist);
}

TEST_CASE("Pass Empty string", "[getdir]") {
  const std::string emptyString;
  std::vector<std::string> files;

  REQUIRE(getdir(emptyString, files) == NotExist);
}

TEST_CASE("Pass Empty directory", "[getdir]") {
  const TempDirectory tempDirectory;

  const std::string emptyDirectory = tempDirectory.mTempDirectory.string();
  std::vector<std::string> files;

  REQUIRE(getdir(emptyDirectory, files) == Empty);
}

TEST_CASE("Pass directory witch contains 3 files", "[getdir]") {
  const int imagesCount = 3;
  const TempDirectory imagesDirectoryPath;
  createFiles(imagesDirectoryPath, imagesCount, "png");

  const std::string imagesDirectory = imagesDirectoryPath.string();
  std::vector<std::string> files;

  REQUIRE(getdir(imagesDirectory, files) == imagesCount);
}

TEST_CASE("Pass directory witch contains 3 files and calib file", "[getdir]") {
  const int imagesCount = 3;
  const TempDirectory imagesDirectoryPath;
  createFiles(imagesDirectoryPath, imagesCount, "png");

  const std::string imagesDir = imagesDirectoryPath.string();
  const std::string calibFileName = "calib.txt";
  const std::string calibFile =
      createFile(imagesDirectoryPath, calibFileName, "");

  const std::string imagesDirectory = imagesDirectoryPath.string();
  std::vector<std::string> files;

  const int filesInDirectory = imagesCount + 1;
  REQUIRE(getdir(imagesDirectory, files) == filesInDirectory);
}

/// ImageFolderReader {

TEST_CASE("pass path to zip file is not exist", "[ImageFolderReader]") {
  const std::string zipFile = "a.zip";
  const std::string calibFile = "";
  const std::string gammaFile = "";
  const std::string vignetteFile = "";

  REQUIRE_THROWS([&] {
    ImageFolderReader imageFolderReader(zipFile, calibFile, gammaFile,
                                        vignetteFile);
  }());
}

TEST_CASE("pass path to zip file and ZIPLIB not exist", "[ImageFolderReader]") {
  const TempDirectory tempDirectory;
  createFiles(tempDirectory, 1, ".zip");

  const std::string zipFile = (tempDirectory.mTempDirectory / "0.zip").string();
  const std::string calibFile = "";
  const std::string gammaFile = "";
  const std::string vignetteFile = "";

  REQUIRE_THROWS([&] {
    ImageFolderReader imageFolderReader(zipFile, calibFile, gammaFile,
                                        vignetteFile);
  }());
}

TEST_CASE("pass empty directory", "[ImageFolderReader]") {
  const std::string imagesDir = "";
  const std::string calibFile = "";
  const std::string gammaFile = "";
  const std::string vignetteFile = "";

  REQUIRE_THROWS([&] {
    ImageFolderReader imageFolderReader(imagesDir, calibFile, gammaFile,
                                        vignetteFile);
  }());
}

TEST_CASE("pass empty string for calib file", "[ImageFolderReader]") {
  const TempDirectory tempDirectory;

  createFiles(tempDirectory, 3, ".png");

  const std::string imagesDir = tempDirectory.string();
  const std::string calibFile = "";
  const std::string gammaFile = "";
  const std::string vignetteFile = "";

  REQUIRE_THROWS([&] {
    ImageFolderReader imageFolderReader(imagesDir, calibFile, gammaFile,
                                        vignetteFile);
  }());
}

TEST_CASE("pass non existing calib file", "[ImageFolderReader]") {
  const TempDirectory tempDirectory;

  createFiles(tempDirectory, 3, ".png");

  const std::string imagesDir = tempDirectory.string();
  const std::string calibFile = "/shit/calib.txt";
  const std::string gammaFile = "";
  const std::string vignetteFile = "";

  RedirectCerr redirect;
  REQUIRE_THROWS([&] {
    ImageFolderReader imageFolderReader(imagesDir, calibFile, gammaFile,
                                        vignetteFile);
  }());
  redirect.release();
}

TEST_CASE("pass empty calib file", "[ImageFolderReader]") {
  const TempDirectory tempDirectory;

  createFiles(tempDirectory, 3, ".png");

  const std::string imagesDir = tempDirectory.string();
  const std::string calibFile = createFile(tempDirectory, "calib.txt", "");
  const std::string gammaFile = "";
  const std::string vignetteFile = "";

  RedirectCout redirect;
  RedirectCerr redirect1;
  REQUIRE_THROWS([&] {
    ImageFolderReader imageFolderReader(imagesDir, calibFile, gammaFile,
                                        vignetteFile);
  }());
  redirect.release();
  redirect1.release();
}

TEST_CASE("pass ATAN calib file", "[ImageFolderReader]") {
  const std::array<double, 5> calibation = {
    0.535719308086809, 0.669566858850269, 0.493248545285398,
    0.500408664348414, 0.897966326944875};
  const Eigen::Vector2i inputImageDim(1280, 1024);
  const Eigen::Vector2i outputImageDim(640, 480);

  const std::string calibFileName = "calib.txt";
  const std::string calibString = generateCalibString(calibation, inputImageDim, outputImageDim);
  const int FilesCount = 3;
  const TempDirectory tempDirectory;

  createFiles(tempDirectory, FilesCount, ".png");

  const auto imagesDir    = tempDirectory.string();
  const auto calibFile    = createFile(tempDirectory, calibFileName, calibString);
  const auto gammaFile    = "";
  const auto vignetteFile = "";

  RedirectStdout redirect;
  ImageFolderReader imageFolderReader(imagesDir, calibFile, gammaFile,
                                      vignetteFile);
  redirect.release();

  REQUIRE(imageFolderReader.width == outputImageDim(0));
  REQUIRE(imageFolderReader.height == outputImageDim(1));

  REQUIRE(imageFolderReader.widthOrg == inputImageDim(0));
  REQUIRE(imageFolderReader.heightOrg == inputImageDim(1));

  REQUIRE(dynamic_cast<dso::UndistortFOV*>(imageFolderReader.undistort) != nullptr);
}

TEST_CASE("pass Pinhole calib file", "[ImageFolderReader]") {
  const std::array<double, 5> calibation = {
    0.535719308086809, 0.669566858850269, 0.493248545285398,
    0.500408664348414, 0};
  const Eigen::Vector2i inputImageDim(1280, 1024);
  const Eigen::Vector2i outputImageDim(640, 480);

  const std::string calibFileName = "calib.txt";
  const std::string calibString = generateCalibString(calibation, inputImageDim, outputImageDim);
  const int FilesCount = 3;
  const TempDirectory tempDirectory;

  createFiles(tempDirectory, FilesCount, ".png");

  const auto imagesDir    = tempDirectory.string();
  const auto calibFile    = createFile(tempDirectory, calibFileName, calibString);
  const auto gammaFile    = "";
  const auto vignetteFile = "";

  RedirectStdout redirect;
  ImageFolderReader imageFolderReader(imagesDir, calibFile, gammaFile,
                                      vignetteFile);
  redirect.release();

  REQUIRE(imageFolderReader.width  == outputImageDim(0));
  REQUIRE(imageFolderReader.height == outputImageDim(1));

  REQUIRE(imageFolderReader.widthOrg  == inputImageDim(0));
  REQUIRE(imageFolderReader.heightOrg == inputImageDim(1));

  REQUIRE(dynamic_cast<dso::UndistortPinhole*>(imageFolderReader.undistort) != nullptr);
}

/// }
