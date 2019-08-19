#pragma once
// STL
#include <sstream>
#include <iostream>
// Eigen3
#include <Eigen/Core>
// Boost
#include <boost/filesystem.hpp>

#ifdef _WIN32
#define NULL_DEVICE "NUL:"
#else
#define NULL_DEVICE "/dev/null"
#endif

struct RedirectStdout {
  RedirectStdout() {
    fflush(stdout);
    mStdoutSave = dup(STDOUT_FILENO);
    freopen(NULL_DEVICE, "w", stdout);
  }

  void release() {
    fflush(stdout);
    if(mStdoutSave != 0) {
      dup2(mStdoutSave, STDOUT_FILENO);
      mStdoutSave = 0;
    }
  }

  ~RedirectStdout() {
    release();
  }

  int mStdoutSave = 0;
};

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

inline std::string generateCalibString(const std::array<double, 5>& calibation,
    const Eigen::Vector2i& inputImageDim, const Eigen::Vector2i &outputImageDim) {
  std::stringstream calibStream;

  for(auto cal : calibation) {
    calibStream << cal << ' ';
  }
  calibStream << '\n';
  calibStream << inputImageDim(0) << ' ' << inputImageDim(1) << '\n';
  calibStream << "crop\n";
  calibStream << outputImageDim(0) << ' ' << outputImageDim(1) << '\n';

  return calibStream.str();
}
