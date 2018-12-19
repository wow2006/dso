#include <string>
#include <vector>
#include <cstdlib>
#include <iostream>

#include <catch/catch.hpp>

#include "util/DatasetReader.h"

#include <boost/filesystem.hpp>


TEST_CASE( "Pass non existing directory", "[getdir]" ) {
  const std::string nonExistingDirectory = "";
  std::vector<std::string> files;

  REQUIRE(getdir(nonExistingDirectory, files) == -1);
}

TEST_CASE( "Pass Empty directory", "[getdir]" ) {
  namespace fs = boost::filesystem;

  const auto emptyDirectoryPath = fs::temp_directory_path() / fs::unique_path();
  fs::create_directories(emptyDirectoryPath);

  const std::string emptyDirectory = emptyDirectoryPath.string();
  std::vector<std::string> files;

  REQUIRE(getdir(emptyDirectory, files) == 0);

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

