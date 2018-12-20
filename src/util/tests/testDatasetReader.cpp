#include <string>
#include <vector>
#include <cstdlib>
#include <iostream>

#define TESTING 1
#include <catch/catch.hpp>

#include "util/DatasetReader.h"

#include <boost/filesystem.hpp>


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
  namespace fs = boost::filesystem;

  const auto emptyDirectoryPath = fs::temp_directory_path() / fs::unique_path();
  fs::create_directories(emptyDirectoryPath);

  const std::string emptyDirectory = emptyDirectoryPath.string();
  std::vector<std::string> files;

  REQUIRE(getdir(emptyDirectory, files) == Empty);

  fs::remove_all(emptyDirectoryPath);
}

TEST_CASE( "Pass directory witch contains 3 files", "[getdir]" ) {
  namespace fs = boost::filesystem;

  const auto emptyDirectoryPath = fs::temp_directory_path() / fs::unique_path();
  fs::create_directories(emptyDirectoryPath);
  for(int i = 0; i < 3; ++i) {
    auto file = emptyDirectoryPath / std::to_string(i);

    std::ofstream(file.string()).close();
  }

  const std::string emptyDirectory = emptyDirectoryPath.string();
  std::vector<std::string> files;

  REQUIRE(getdir(emptyDirectory, files) == 3);

  fs::remove_all(emptyDirectoryPath);
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
  const std::string zipFile = [](){ 
    namespace fs = boost::filesystem;

    const auto emptyDirectoryPath = fs::temp_directory_path() / fs::unique_path();

    fs::create_directories(emptyDirectoryPath);

    const auto fileString = (emptyDirectoryPath / "a.zip").string();

    std::ifstream f{fileString};
    f.close();

    return fileString;
  }();
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

/// }
