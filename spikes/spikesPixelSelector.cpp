// STL
#include <iostream>
// OpenCV
#include <opencv2/opencv.hpp>
// Internal
#include "FullSystem/PixelSelector2.h"
#include "FullSystem/HessianBlocks.h"
// util
#include "util/globalFuncs.h"

int main(int argc, char* argv[]) {
  if(argc != 2) {
    std::cerr << "Usage:\n\t" << argv[0] << " input_image.png\n";
    return EXIT_FAILURE;
  }
  // Read and convert to float
  cv::Mat inputImage = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
  inputImage.convertTo(inputImage, CV_32FC1);

  constexpr bool plotImages        = true;
  const     int  imageWidth        = inputImage.cols;
  const     int  imageHeight       = inputImage.rows;
  const     auto imageSizeInPixels = static_cast<std::size_t>(imageWidth * imageHeight);

  dso::wG[0] = imageWidth;
  dso::hG[0] = imageHeight;

  dso::CalibHessian Hcalib;

  dso::FrameHessian dummyFrameHessian;
  dummyFrameHessian.makeImages(inputImage.ptr<float>(), &Hcalib);
  std::vector<float> dummyMapOutput(imageSizeInPixels);

  // CoarseInitializer.cpp+729
  dso::PixelSelector selector(imageWidth, imageHeight);
  selector.makeMaps(&dummyFrameHessian, dummyMapOutput.data(),
                    100.f, 1, plotImages, 2);

  // wait for key
  cv::waitKey();

  return EXIT_SUCCESS;
}

