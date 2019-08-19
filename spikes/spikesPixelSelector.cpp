// STL
#include <iostream>
// Internal
#include "FullSystem/PixelSelector2.h"
#include "FullSystem/HessianBlocks.h"
//
#include "util/globalFuncs.h"

int main(int argc, char* argv[]) {
  (void)argc; (void)argv;
  constexpr bool plotImages = true;
  constexpr int imageWidth = 128;
  constexpr int imageHeight = 128;
  constexpr int imageSizeInPixels = imageWidth * imageHeight;

  dso::wG[0] = imageWidth;
  dso::hG[0] = imageHeight;

  std::vector<float> imageDataInFloat(imageSizeInPixels);

  dso::PixelSelector selector(imageWidth, imageHeight);

  dso::CalibHessian Hcalib;

  dso::FrameHessian dummyFrameHessian;
  dummyFrameHessian.makeImages(imageDataInFloat.data(), &Hcalib);
  std::vector<float> dummyMapOutput(imageSizeInPixels);

  // CoarseInitializer.cpp+729
  selector.makeMaps(&dummyFrameHessian, dummyMapOutput.data(), 0, 1, plotImages, 2);

  return EXIT_SUCCESS;
}

