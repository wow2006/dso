#include <iostream>

#include <catch/catch.hpp>

#include "../Undistort.hpp"


struct RedirectCerr {
  RedirectCerr() {
    m_pCerrBuffer = std::cerr.rdbuf();
    std::cerr.rdbuf(mStringStream.rdbuf());
  }
  
  void release() {
    if(m_pCerrBuffer != nullptr) {
      std::cerr.rdbuf(m_pCerrBuffer);
      m_pCerrBuffer = nullptr;
    }
  }

  ~RedirectCerr() {
    release();
  }

  std::streambuf *m_pCerrBuffer;
  std::stringstream mStringStream;
};

struct RedirectCout {
  RedirectCout() {
    m_pCoutBuffer = std::cout.rdbuf();
    std::cout.rdbuf(mStringStream.rdbuf());
  }
  
  void release() {
    if(m_pCoutBuffer != nullptr) {
      std::cout.rdbuf(m_pCoutBuffer);
      m_pCoutBuffer = nullptr;
    }
  }

  ~RedirectCout() {
    release();
  }

  std::streambuf *m_pCoutBuffer;
  std::stringstream mStringStream;
};

TEST_CASE("pass empty calib file",
          "[Undistort, getUndistorterForFile]" ) {
  using namespace dso;

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
  using namespace dso;

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
