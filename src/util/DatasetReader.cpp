// STL
#include <algorithm>
#include <fstream>
#include <sstream>
#include <string_view>
// Boost
#include <boost/filesystem.hpp>
// Internal
#include "DatasetReader.h"
// Internal
#include "util/globalCalib.h"
#include "util/globalFuncs.h"
#include "util/settings.h"
// IOWrapper
#include "IOWrapper/ImageRW.h"
// ZIP
#ifdef HAS_ZIPLIB
#include "zip.h"
#endif

bool hasEnding(std::string const &fullString, std::string const &ending) {
  if (fullString.length() >= ending.length()) {
    return (0 == fullString.compare(fullString.length() - ending.length(),
                                    ending.length(), ending));
  } else {
    return false;
  }
}

int getdir(std::string_view dir, std::vector<std::string> &files) {
  namespace fs = boost::filesystem;

  const auto directory = fs::path{std::string{dir}};

  if (dir.empty() || !fs::is_directory(dir.data())) {
    return NotExist;
  }

  fs::directory_iterator filesIterators{directory};
  fs::directory_iterator filesIteratorsEnd;

  std::transform(
      filesIterators, filesIteratorsEnd, std::back_inserter(files),
      [](const fs::directory_entry &entry) { return entry.path().string(); });

  return static_cast<int>(files.size());
}

void PrepImageItem::release() {
  delete pt;
  pt = nullptr;
}

ImageFolderReader::ImageFolderReader(std::string path_, std::string calibFile_,
                                     std::string gammaFile,
                                     std::string vignetteFile)
    : path{path_}, calibfile{calibFile_} {
  namespace fs = boost::filesystem;

  if (checkInputFile(calibFile_, ".zip")) {
    readZipFile();
  } else {
    readImages();
  }

  readCalibration(gammaFile, vignetteFile);

  widthOrg = undistort->getOriginalSize()[0];
  heightOrg = undistort->getOriginalSize()[1];

  width = undistort->getSize()[0];
  height = undistort->getSize()[1];

  // load timestamps if possible.
  loadTimestamps();
  std::cout << "ImageFolderReader: got " << files.size() << " files in "
            << path.c_str() << '\n';
}

ImageFolderReader::~ImageFolderReader() {
#ifdef HAS_ZIPLIB
  if (ziparchive != 0)
    zip_close(ziparchive);
  if (databuffer != 0)
    delete databuffer;
#endif

  delete undistort;
}

dso::ImageAndExposure *ImageFolderReader::getImage_internal(int id,
                                                            int unused) {
  (void)unused;

  dso::MinimalImageB *minimg = getImageRaw_internal(id, 0);
  dso::ImageAndExposure *ret2 = undistort->undistort<unsigned char>(
      minimg, (exposures.size() == 0 ? 1.0f : exposures[id]),
      (timestamps.size() == 0 ? 0.0 : timestamps[id]));
  delete minimg;
  return ret2;
}

dso::MinimalImageB *ImageFolderReader::getImageRaw_internal(int id,
                                                            int unused) {
  (void)unused;

  if (!isZipped) {
    return dso::IOWrap::readImageBW_8U(files[id]);
  } else {
#ifdef HAS_ZIPLIB
    if (databuffer == 0) {
      databuffer = new char[widthOrg * heightOrg * 6 + 10000];
    }

    auto fle = zip_fopen(ziparchive, files[id].c_str(), 0);
    long readbytes =
        zip_fread(fle, databuffer, (long)widthOrg * heightOrg * 6 + 10000);

    if (readbytes > (long)widthOrg * heightOrg * 6) {
      printf("read %ld/%ld bytes for file %s. increase buffer!!\n", readbytes,
             (long)widthOrg * heightOrg * 6 + 10000, files[id].c_str());
      delete[] databuffer;
      databuffer = new char[(long)widthOrg * heightOrg * 30];
      fle = zip_fopen(ziparchive, files[id].c_str(), 0);
      readbytes =
          zip_fread(fle, databuffer, (long)widthOrg * heightOrg * 30 + 10000);

      if (readbytes > (long)widthOrg * heightOrg * 30) {
        printf("buffer still to small (read %ld/%ld). abort.\n", readbytes,
               (long)widthOrg * heightOrg * 30 + 10000);
        exit(1);
      }
    }

    return IOWrap::readStreamBW_8U(databuffer, readbytes);
#else
    printf("ERROR: cannot read .zip archive, as compile without ziplib!\n");
    exit(1);
#endif
  }
}

void ImageFolderReader::loadTimestamps() {
  std::ifstream tr;
  std::string timesFile = path.substr(0, path.find_last_of('/')) + "/times.txt";
  tr.open(timesFile.c_str());
  while (!tr.eof() && tr.good()) {
    std::string line;
    char buf[1000];
    tr.getline(buf, 1000);

    int id;
    double stamp;
    float exposure = 0;

    if (3 == sscanf(buf, "%d %lf %f", &id, &stamp, &exposure)) {
      timestamps.push_back(stamp);
      exposures.push_back(exposure);
    }

    else if (2 == sscanf(buf, "%d %lf", &id, &stamp)) {
      timestamps.push_back(stamp);
      exposures.push_back(exposure);
    }
  }
  tr.close();

  // check if exposures are correct, (possibly skip)
  bool exposuresGood =
      (exposures.size() == static_cast<std::size_t>(getNumImages()));
  for (std::size_t i = 0; i < exposures.size(); i++) {
    if (exposures[i] == 0) {
      // fix!
      float sum = 0, num = 0;
      if (i > 0 && exposures[i - 1] > 0) {
        sum += exposures[i - 1];
        num++;
      }

      if (i + 1 < exposures.size() && exposures[i + 1] > 0) {
        sum += exposures[i + 1];
        num++;
      }

      if (num > 0) {
        exposures[i] = sum / num;
      }
    }

    if (exposures[i] == 0) {
      exposuresGood = false;
    }
  }

  if (getNumImages() != static_cast<int>(timestamps.size())) {
    printf("set timestamps and exposures to zero!\n");
    exposures.clear();
    timestamps.clear();
  }

  if (getNumImages() != static_cast<int>(exposures.size()) || !exposuresGood) {
    printf("set EXPOSURES to zero!\n");
    exposures.clear();
  }

  std::cout << "got " << getNumImages() << " images and " << timestamps.size()
            << " timestamps and " << exposures.size() << '\n';
}

void ImageFolderReader::setGlobalCalibration() {
  int w_out, h_out;
  Eigen::Matrix3f K;
  getCalibMono(K, w_out, h_out);
  dso::setGlobalCalib(w_out, h_out, K);
}

bool ImageFolderReader::checkInputFile(const std::string &inputFileName,
                                       const std::string &extension) {
  return (inputFileName.length() > 4 &&
          inputFileName.substr(inputFileName.length() - 4) == extension);
}

void ImageFolderReader::readZipFile() {
#ifdef HAS_ZIPLIB
  ziparchive = 0;
  databuffer = 0;

  int ziperror = 0;
  ziparchive = zip_open(path.c_str(), ZIP_RDONLY, &ziperror);
  if (ziperror != 0) {
    printf("ERROR %d reading archive %s!\n", ziperror, path.c_str());
    exit(1);
  }

  files.clear();
  int numEntries = zip_get_num_entries(ziparchive, 0);
  for (int k = 0; k < numEntries; k++) {
    const char *name = zip_get_name(ziparchive, k, ZIP_FL_ENC_STRICT);
    std::string nstr = std::string(name);
    if (nstr == "." || nstr == "..")
      continue;
    files.push_back(name);
  }

  printf("got %d entries and %d files!\n", numEntries, (int)files.size());
  std::sort(files.begin(), files.end());
#else
  throw std::runtime_error(
      "ERROR: cannot read .zip archive, as compile without ziplib!");
#endif
}

void ImageFolderReader::readImages() {
  namespace fs = boost::filesystem;

  if (getdir(path, files) <= 0) {
    throw std::runtime_error("Can not found images");
  }

  // TODO(Hussien): Make sure files is just png
  files.erase(
      std::remove_if(std::begin(files), std::end(files),
                     [](auto filename) { return !hasEnding(filename, "jpg"); }),
      files.end());

  std::sort(std::begin(files), std::end(files), [](auto a, auto b) {
    const auto baseFileNameA = fs::path(a).stem().string();
    const auto baseFileNameB = fs::path(b).stem().string();

    return std::stoi(baseFileNameA) < std::stoi(baseFileNameB);
  });
}

void ImageFolderReader::readCalibration(const std::string& gammaFile, const std::string& vignetteFile) {
  namespace fs = boost::filesystem;

  if (calibfile.empty() || !fs::exists(calibfile)) {
    throw std::runtime_error("calib file is empty");
  }

  undistort =
      dso::Undistort::getUndistorterForFile(calibfile, gammaFile, vignetteFile);

  if (!undistort) {
    throw std::runtime_error("can not read undistorter file");
  }
}
