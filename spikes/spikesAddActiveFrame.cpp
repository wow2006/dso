// STL
#include <iostream>
#include <memory>
#include <vector>
// fmt
#include <fmt/color.h>
#include <fmt/printf.h>
#include <fmt/format.h>
// Boost
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
// Internal
#include "FullSystem/CoarseInitializer.h"
#include "FullSystem/HessianBlocks.h"
#include "util/DatasetReader.h"
#include "util/FrameShell.h"
#include "IOWrapper/ImageDisplay.h"
// OpenCV
#include <opencv2/opencv.hpp>

static bool initialized = false;
static CalibHessian Hcalib;
static CoarseInitializer *coarseInitializer;
static std::vector<FrameShell *> allFrameHistory;
static std::vector<FrameHessian *> allFrames;

std::vector<std::string> listImages(const std::string &inputDirectory);

FrameHessian *addNewFrameToHistory(ImageAndExposure *image, int id);

void makeImagesDerivatives(FrameHessian *fh, ImageAndExposure *image);

void initializing(FrameHessian *fh);

void debugPlotTracking();

int main(int argc, char *argv[]) {
  int startImageIndex = 0, stopImageIndex = 10;
  std::string inputDirectory, inputCalib, inputGamma;
  using namespace boost::program_options;
  try {
    options_description desc{"Options"};
    desc.add_options()("help,h", "Help screen")(
        "image,i", value<std::string>(&inputDirectory)->required(),
        "Input image directory")(
          "calib,c", value<std::string>(&inputCalib)->required(), "Input Calib")(
          "gamma,g", value<std::string>(&inputGamma)->required(), "Input Gamma");

    variables_map vm;
    store(parse_command_line(argc, argv, desc), vm);
    notify(vm);

    if (vm.count("help"))
      std::cout << desc << '\n';
  } catch (const error &ex) {
    std::cerr << ex.what() << '\n';
  }

  const auto images = listImages(inputDirectory);

  auto reader = std::make_unique<ImageFolderReader>(inputDirectory, inputCalib,
      inputGamma, "");
  // TODO(Hussien): remove global variables
  reader->setGlobalCalibration();

  coarseInitializer = new CoarseInitializer(wG[0], hG[0]);

  for (int i = startImageIndex; i < stopImageIndex; ++i) {
    std::cout << fmt::format("Processing image ({})\n", i);
    auto image = reader->getImage(i);

    // @{ addActiveFrame
    auto frame = addNewFrameToHistory(image, i);

    makeImagesDerivatives(frame, image);

    if (!initialized) {
      initializing(frame);
    } else {
      // frontEndOperation(fh, lock);
      break;
    }
    // @}

    delete image;
  }

  debugPlotTracking();

  return EXIT_SUCCESS;
}

std::vector<std::string> listImages(const std::string &inputDirectory) {
  std::vector<std::string> images;
  boost::filesystem::path imagesDirectory{inputDirectory};
  boost::filesystem::directory_iterator iteratorImages{imagesDirectory};
  for (; iteratorImages != boost::filesystem::directory_iterator{};
      ++iteratorImages) {
    images.push_back(iteratorImages->path().string());
  }

  std::sort(std::begin(images), std::end(images), [](auto a, auto b) {
      return std::stoi(boost::filesystem::path{a}.stem().string()) <
      std::stoi(boost::filesystem::path{b}.stem().string());
      });

  for (auto image : images) {
    std::cout << image << '\n';
  }

  return images;
}

FrameHessian *addNewFrameToHistory(ImageAndExposure *image, int id) {
  auto fh = new FrameHessian();
  auto shell = new FrameShell();
  // no lock required, as fh is not used anywhere yet.
  shell->camToWorld = SE3();
  shell->aff_g2l = AffLight(0, 0);
  shell->marginalizedAt = shell->id = static_cast<int>(allFrameHistory.size());
  shell->timestamp = image->timestamp;
  shell->incoming_id = id;
  fh->shell = shell;
  allFrameHistory.push_back(shell);

  allFrames.push_back(fh);

  return fh;
}

void makeImagesDerivatives(FrameHessian *fh, ImageAndExposure *image) {
  fh->ab_exposure = image->exposure_time;
  fh->makeImages(image->image, &Hcalib);
}

void initializing(FrameHessian *fh) {
  // use initializer!
  if (coarseInitializer->frameID < 0) {
    // first frame set. fh is kept by coarseInitializer.
    coarseInitializer->setFirst(&Hcalib, fh);
    fmt::print(fmt::fg(fmt::color::green), "Process shell ({})\n",
        fh->shell->incoming_id
        );
    return;
  } else {
    fmt::print(fmt::fg(fmt::color::red), "Process shell ({})\n",
        fh->shell->incoming_id
        );
  }
  // std::exit(0);
}

void debugPlotTracking() {
  const int wh = hG[0] * wG[0];

  std::vector<MinimalImageB3 *> images;

  for (auto frame2 : allFrames) {
    if (frame2->debugImage == 0) {
      frame2->debugImage = new MinimalImageB3(wG[0], hG[0]);
    }
  }

  for (auto frame2 : allFrames) {
    auto debugImage = frame2->debugImage;
    images.push_back(debugImage);

    auto fd = frame2->dI;

    for (int i = 0; i < wh; i++) {
      // BRIGHTNESS TRANSFER
      float colL = fd[i][0];
      if (colL < 0) {
        colL = 0;
      }

      if (colL > 255) {
        colL = 255;
      }

      debugImage->at(i) = Vec3b(colL, colL, colL);
    }
  }

  {
    std::vector<cv::Mat *> imagesCV;
    for (size_t i = 0; i < images.size(); i++) {
      auto image =
        new cv::Mat(images[i]->h, images[i]->w, CV_8UC3, images[i]->data);

      char imageText[100];
      sprintf(imageText, "Image(%d)", i);

      cv::putText(*image, imageText, cv::Point(30, 30),
          cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8,
          cv::Scalar(200, 200, 250), 1);
      imagesCV.push_back(image);
    }

    dso::IOWrap::displayImageStitch("Images", imagesCV, 0, 0);

    for (size_t i = 0; i < images.size(); i++) {
      delete imagesCV[i];
    }
  }

  IOWrap::waitKey(0);
}