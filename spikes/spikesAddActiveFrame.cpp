// STL
#include <iostream>
#include <memory>
#include <vector>
// fmt
#include <fmt/color.h>
#include <fmt/format.h>
#include <fmt/printf.h>
// Boost
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
// Internal
#include "FullSystem/CoarseInitializer.h"
#include "FullSystem/CoarseTracker.h"
#include "FullSystem/HessianBlocks.h"
// util
#include "util/DatasetReader.h"
#include "util/FrameShell.h"
// IOWrapper
#include "IOWrapper/ImageDisplay.h"
// OpenCV
#include <opencv2/opencv.hpp>

static bool initialized = false;
static CalibHessian Hcalib;
static CoarseInitializer *coarseInitializer;
static std::vector<FrameShell *> allFrameHistory;
static std::vector<FrameHessian *> allFrames;

static dso::CoarseTracker
    *coarseTracker_forNewKF;              // set as as reference. protected by
                                          // [coarseTrackerSwapMutex].
static dso::CoarseTracker *coarseTracker; // always used to track new frames.
                                          // protected by [trackMutex].

std::vector<std::string> listImages(const std::string &inputDirectory);

FrameHessian *addNewFrameToHistory(ImageAndExposure *image, int id);

void makeImagesDerivatives(FrameHessian *fh, ImageAndExposure *image);

void initializing(FrameHessian *fh);

void debugPlotTracking();

void swapTrackingReference() {
  if (coarseTracker_forNewKF->refFrameID > coarseTracker->refFrameID) {
    auto *tmp = coarseTracker;
    coarseTracker = coarseTracker_forNewKF;
    coarseTracker_forNewKF = tmp;
  }
}

int main(int argc, char *argv[]) {
  int startImageIndex = 0, stopImageIndex = 22;
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
  coarseInitializer->printDebug = true;

  for (int i = startImageIndex; i < stopImageIndex; ++i) {
    std::cout << fmt::format("Processing image ({})\n", i);
    auto image = reader->getImage(i);

    /// @{ addActiveFrame
    auto frame = addNewFrameToHistory(image, i);

    makeImagesDerivatives(frame, image);

    if (!initialized) {
      initializing(frame);
      continue;
    }

    // swapTrackingReference();

    /// @}

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
  auto frameHessian = new FrameHessian();
  auto shell = new FrameShell();
  // no lock required, as fh is not used anywhere yet.
  shell->camToWorld = SE3();
  shell->aff_g2l = AffLight(0, 0);
  shell->marginalizedAt = shell->id = static_cast<int>(allFrameHistory.size());
  shell->timestamp = image->timestamp;
  shell->incoming_id = id;
  frameHessian->shell = shell;
  //
  allFrameHistory.push_back(shell);

  return frameHessian;
}

void makeImagesDerivatives(FrameHessian *fh, ImageAndExposure *image) {
  fh->ab_exposure = image->exposure_time;
  fh->makeImages(image->image, &Hcalib);
}

void initializing(FrameHessian *pFrame) {
  static std::vector<IOWrap::Output3DWrapper *> outputWrapper;
  // use initializer!
  if (coarseInitializer->frameID < 0) {
    // first frame set. fh is kept by coarseInitializer.
    fmt::print(fmt::fg(fmt::color::red), "Process shell ({})\n",
               pFrame->shell->incoming_id);
    coarseInitializer->setFirst(&Hcalib, pFrame);
  } else if (coarseInitializer->trackFrame(pFrame, outputWrapper)) {
    // if SNAPPED
    fmt::print(fmt::fg(fmt::color::green), "Process shell ({})\n",
               pFrame->shell->incoming_id);
    // initializeFromInitializer(pFrame);
    // deliverTrackedFrame(pFrame, true);
  } else {
    // if still initializing
    fmt::print(fmt::fg(fmt::color::blue), "Process shell ({})\n",
               pFrame->shell->incoming_id);
    pFrame->shell->poseValid = false;
    delete pFrame;
  }
}

void debugPlotTracking() {
  const int wh = hG[0] * wG[0];

  std::vector<MinimalImageB3 *> images;

  for (auto frame2 : allFrames) {
    if (frame2->debugImage == nullptr) {
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
