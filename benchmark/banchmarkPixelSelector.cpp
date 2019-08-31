// STL
#include <iostream>
// OpenCV
#include <opencv2/opencv.hpp>
// benchmark
#include <benchmark/benchmark.h>
// Internal
#include "FullSystem/PixelSelector2.h"
#include "FullSystem/HessianBlocks.h"
// util
#include "util/globalFuncs.h"

static void BM_SomeFunction(benchmark::State& state) {
  cv::Mat inputImage = cv::imread("Lenna.png", cv::IMREAD_GRAYSCALE);
  inputImage.convertTo(inputImage, CV_32FC1);

  constexpr bool plotImages        = false;
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
  // Perform setup here
  for (auto _ : state) {
  }
}
// Register the function as a benchmark
BENCHMARK(BM_SomeFunction);
// Run the benchmark
BENCHMARK_MAIN();
