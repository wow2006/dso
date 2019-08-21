#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>
// Catch
#define TESTING 1
#include <gtest/gtest.h>
// Internal
#include "util/DatasetReader.h"
//
#include "testUtils.hpp"

TEST(hasEnding, PassInputStringABC) {
  const std::string fullString = "ABC";
  const std::string endsWithC  = "C";

  ASSERT_TRUE(hasEnding(fullString, endsWithC));
}

TEST(getdir, PassNonExistingDirectory) {
  const std::string nonExistingDirectory = "/shit";
  std::vector<std::string> files;

  ASSERT_TRUE(getdir(nonExistingDirectory, files) == NotExist);
}

TEST(getdir, PassEmptyString) {
  const std::string emptyString;
  std::vector<std::string> files;

  ASSERT_EQ(getdir(emptyString, files), NotExist);
}

TEST(getdir, PassEmptyDirectory) {
  const TempDirectory tempDirectory;

  const std::string emptyDirectory = tempDirectory.mTempDirectory.string();
  std::vector<std::string> files;

  ASSERT_TRUE(getdir(emptyDirectory, files) == Empty);
}

TEST(getdir, PassDirectoryWitchContainsThresFiles) {
  const int imagesCount = 3;
  const TempDirectory imagesDirectoryPath;
  createFiles(imagesDirectoryPath, imagesCount, "png");

  const std::string imagesDirectory = imagesDirectoryPath.string();
  std::vector<std::string> files;

  ASSERT_TRUE(getdir(imagesDirectory, files) == imagesCount);
}

TEST(getdir, PassDirectoryWitchContainsThreeFilesAndCalibFile) {
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
  ASSERT_TRUE(getdir(imagesDirectory, files) == filesInDirectory);
}

/// ImageFolderReader {

TEST(ImageFolderReader, PassPathToZipFileIsNotExist) {
  const std::string zipFile = "a.zip";
  const std::string calibFile = "";
  const std::string gammaFile = "";
  const std::string vignetteFile = "";

  ASSERT_THROW([&] {
    ImageFolderReader imageFolderReader(zipFile, calibFile, gammaFile,
                                        vignetteFile);
  }(), std::runtime_error);
}

TEST(ImageFolderReader, PassPathToZipFileAndZIPLIBNotExist) {
  const TempDirectory tempDirectory;
  createFiles(tempDirectory, 1, ".zip");

  const std::string zipFile = (tempDirectory.mTempDirectory / "0.zip").string();
  const std::string calibFile = "";
  const std::string gammaFile = "";
  const std::string vignetteFile = "";

  ASSERT_THROW([&] {
    ImageFolderReader imageFolderReader(zipFile, calibFile, gammaFile,
                                        vignetteFile);
  }(), std::runtime_error);
}

TEST(ImageFolderReader, PassEmptyDirectory) {
  const std::string imagesDir = "";
  const std::string calibFile = "";
  const std::string gammaFile = "";
  const std::string vignetteFile = "";

  ASSERT_THROW([&] {
    ImageFolderReader imageFolderReader(imagesDir, calibFile, gammaFile,
                                        vignetteFile);
  }(), std::runtime_error);
}

TEST(ImageFolderReader, PassEmptyStringForCalibFile) {
  const TempDirectory tempDirectory;

  createFiles(tempDirectory, 3, ".png");

  const std::string imagesDir = tempDirectory.string();
  const std::string calibFile = "";
  const std::string gammaFile = "";
  const std::string vignetteFile = "";

  ASSERT_THROW([&] {
    ImageFolderReader imageFolderReader(imagesDir, calibFile, gammaFile,
                                        vignetteFile);
  }(), std::runtime_error);
}

TEST(ImageFolderReader, PassNonExistingCalibFile) {
  const TempDirectory tempDirectory;

  createFiles(tempDirectory, 3, ".png");

  const std::string imagesDir = tempDirectory.string();
  const std::string calibFile = "/shit/calib.txt";
  const std::string gammaFile = "";
  const std::string vignetteFile = "";

  RedirectCerr redirect;
  ASSERT_THROW([&] {
    ImageFolderReader imageFolderReader(imagesDir, calibFile, gammaFile,
                                        vignetteFile);
  }(), std::runtime_error);
  redirect.release();
}

TEST(ImageFolderReader, PassEmptyCalibFile) {
  const TempDirectory tempDirectory;

  createFiles(tempDirectory, 3, ".png");

  const std::string imagesDir = tempDirectory.string();
  const std::string calibFile = createFile(tempDirectory, "calib.txt", "");
  const std::string gammaFile = "";
  const std::string vignetteFile = "";

  RedirectCout redirect;
  RedirectCerr redirect1;
  ASSERT_THROW([&] {
    ImageFolderReader imageFolderReader(imagesDir, calibFile, gammaFile,
                                        vignetteFile);
  }(), std::runtime_error);
  redirect.release();
  redirect1.release();
}

TEST(ImageFolderReader, PassATANCalibFile) {
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

  ASSERT_TRUE(imageFolderReader.width == outputImageDim(0));
  ASSERT_TRUE(imageFolderReader.height == outputImageDim(1));

  ASSERT_TRUE(imageFolderReader.widthOrg == inputImageDim(0));
  ASSERT_TRUE(imageFolderReader.heightOrg == inputImageDim(1));

  ASSERT_TRUE(dynamic_cast<dso::UndistortFOV*>(imageFolderReader.undistort) != nullptr);
}

TEST(ImageFolderReader, PassPinholeCalibFile) {
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

  ASSERT_TRUE(imageFolderReader.width  == outputImageDim(0));
  ASSERT_TRUE(imageFolderReader.height == outputImageDim(1));

  ASSERT_TRUE(imageFolderReader.widthOrg  == inputImageDim(0));
  ASSERT_TRUE(imageFolderReader.heightOrg == inputImageDim(1));

  ASSERT_TRUE(dynamic_cast<dso::UndistortPinhole*>(imageFolderReader.undistort) != nullptr);
}

/// }
