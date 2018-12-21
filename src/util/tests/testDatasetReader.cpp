#include <string>
#include <vector>
#include <cstdlib>
#include <iostream>

#define TESTING 1
#include <catch/catch.hpp>

#include "util/DatasetReader.h"

#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;


struct TempDirectory {
  TempDirectory() {
    mTempDirectory = fs::temp_directory_path() / fs::unique_path();
    fs::create_directories(mTempDirectory);
  }

  ~TempDirectory() {
    fs::remove_all(mTempDirectory);
  }

  std::string string() const {
    return mTempDirectory.string();
  }

  fs::path mTempDirectory;
};

inline void createFiles(const TempDirectory& directory,
                        const int count,
                        std::string_view fileExtention) {
  for(int i = 0; i < count; ++i) {
    auto file = directory.mTempDirectory / (std::to_string(i) + fileExtention.data());

    std::ofstream(file.string()).close();
  }
}

inline std::string createFile(const TempDirectory& directory,
                              std::string_view fileName,
                              std::string_view fileContent) {
    auto file = directory.mTempDirectory / fileName.data();
    std::ofstream out{file};
    out << fileContent;
    out.close();

    return file.string();
}


TEST_CASE( "Pass non existing directory", "[getdir]" ) {
  const std::string nonExistingDirectory = "/shit";
  std::vector<std::string> files;

  REQUIRE(getdir(nonExistingDirectory, files) == NotExist);
}

TEST_CASE( "Pass Empty string", "[getdir]" ) {
  const std::string emptyString;
  std::vector<std::string> files;

  REQUIRE(getdir(emptyString, files) == NotExist);
}

TEST_CASE( "Pass Empty directory", "[getdir]" ) {
  const TempDirectory tempDirectory;

  const std::string emptyDirectory = tempDirectory.mTempDirectory.string();
  std::vector<std::string> files;

  REQUIRE(getdir(emptyDirectory, files) == Empty);
}

TEST_CASE( "Pass directory witch contains 3 files", "[getdir]" ) {
  const int count = 3;
  const TempDirectory emptyDirectoryPath;
  createFiles(emptyDirectoryPath, count, "");

  const std::string emptyDirectory = emptyDirectoryPath.string();
  std::vector<std::string> files;

  REQUIRE(getdir(emptyDirectory, files) == 3);
}

/// ImageFolderReader {

TEST_CASE("pass path to zip file is not exist", "[ImageFolderReader]" ) {
  const std::string zipFile      = "a.zip";
  const std::string calibFile    = "";
  const std::string gammaFile    = "";
  const std::string vignetteFile = "";

  REQUIRE_THROWS([&]{
    ImageFolderReader imageFolderReader(zipFile, calibFile, gammaFile, vignetteFile);
  }());
}

TEST_CASE("pass path to zip file and ZIPLIB not exist", "[ImageFolderReader]" ) {
  const TempDirectory tempDirectory;
  createFiles(tempDirectory, 1, ".zip");

  const std::string zipFile      = (tempDirectory.mTempDirectory / "0.zip").string();
  const std::string calibFile    = "";
  const std::string gammaFile    = "";
  const std::string vignetteFile = "";

  REQUIRE_THROWS([&]{
    ImageFolderReader imageFolderReader(zipFile, calibFile, gammaFile, vignetteFile);
  }());
}

TEST_CASE("pass empty directory", "[ImageFolderReader]" ) {
  const std::string imagesDir    = "";
  const std::string calibFile    = "";
  const std::string gammaFile    = "";
  const std::string vignetteFile = "";

  REQUIRE_THROWS([&]{
    ImageFolderReader imageFolderReader(imagesDir, calibFile, gammaFile, vignetteFile);
  }());
}

TEST_CASE("pass empty string for calib file", "[ImageFolderReader]" ) {
  const TempDirectory tempDirectory;

  createFiles(tempDirectory, 3, ".png");

  const std::string imagesDir    = tempDirectory.string();
  const std::string calibFile    = "";
  const std::string gammaFile    = "";
  const std::string vignetteFile = "";

  REQUIRE_THROWS([&]{
    ImageFolderReader imageFolderReader(imagesDir, calibFile, gammaFile, vignetteFile);
  }());
}

TEST_CASE("pass non existing calib file", "[ImageFolderReader]" ) {
  const TempDirectory tempDirectory;

  createFiles(tempDirectory, 3, ".png");

  const std::string imagesDir    = tempDirectory.string();
  const std::string calibFile    = "/shit/calib.txt";
  const std::string gammaFile    = "";
  const std::string vignetteFile = "";

  REQUIRE_THROWS([&]{
    ImageFolderReader imageFolderReader(imagesDir, calibFile, gammaFile, vignetteFile);
  }());
}

TEST_CASE("pass empty calib file", "[ImageFolderReader]" ) {
  const TempDirectory tempDirectory;

  createFiles(tempDirectory, 3, ".png");

  const std::string imagesDir    = tempDirectory.string();
  const std::string calibFile    = createFile(tempDirectory, "calib.txt", "");
  const std::string gammaFile    = "";
  const std::string vignetteFile = "";

  REQUIRE_THROWS([&]{
    ImageFolderReader imageFolderReader(imagesDir, calibFile, gammaFile, vignetteFile);
  }());
}

/// }
