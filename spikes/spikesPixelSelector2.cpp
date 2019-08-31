// STL
#include <iostream>
// OpenCV
#include <opencv2/opencv.hpp>
// Internal
#define private public
#include "FullSystem/PixelSelector2.h"
#undef private
#include "FullSystem/HessianBlocks.h"
// util
#include "util/globalFuncs.h"

void makeHists(const dso::FrameHessian *const fh);

void selectPoints(const dso::FrameHessian *const frameHessian, float *map_out,
                  int pot, float thFactor = 1);

int main(int argc, char *argv[]) {
  if (argc != 2) {
    std::cerr << "Usage:\n\t" << argv[0] << " input_image.png\n";
    return EXIT_FAILURE;
  }
  // Read and convert to float
  cv::Mat inputImage = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
  inputImage.convertTo(inputImage, CV_32FC1);

  //constexpr bool plotImages = true;
  const int imageWidth = inputImage.cols;
  const int imageHeight = inputImage.rows;
  const auto imageSizeInPixels =
      static_cast<std::size_t>(imageWidth * imageHeight);

  dso::wG[0] = imageWidth;
  dso::hG[0] = imageHeight;
  for (int i = 1; i < dso::pyrLevelsUsed; ++i) {
    dso::wG[i] = dso::wG[i - 1] / 2;
    dso::hG[i] = dso::hG[i - 1] / 2;
  }

  dso::CalibHessian Hcalib;

  dso::FrameHessian dummyFrameHessian;
  dummyFrameHessian.makeImages(inputImage.ptr<float>(), &Hcalib);
  std::vector<float> dummyMapOutput(imageSizeInPixels);

  // CoarseInitializer.cpp+729
  selectPoints(&dummyFrameHessian, dummyMapOutput.data(), 1);

  // wait for key
  cv::waitKey();

  return EXIT_SUCCESS;
}

void makeHists(const dso::FrameHessian *const fh) {
  (void)fh;
}

void selectPoints(const dso::FrameHessian *const frameHessian, float *map_out,
                  int pot, float thFactor) {

  // =======================
  // Members
  // =======================
  const auto imageSizeInPixel = static_cast<std::size_t>(dso::wG[0] * dso::hG[0]);
  std::vector<unsigned char> randomPattern(imageSizeInPixel);
  std::generate(std::begin(randomPattern), std::end(randomPattern),
                []() { return rand() & 0xFF; });

  std::vector<float> thsSmoothed;

  const auto thsStep = dso::wG[0]/ 32;
  // =======================

  Eigen::Vector3f const *const map0 = frameHessian->dI;

  const float *const mapmax0 = frameHessian->absSquaredGrad[0];
  const float *const mapmax1 = frameHessian->absSquaredGrad[1];
  const float *const mapmax2 = frameHessian->absSquaredGrad[2];

  const int w  = dso::wG[0];
  const int w1 = dso::wG[1];
  const int w2 = dso::wG[2];
  const int h  = dso::hG[0];

  // NOTE(Hussien): Directions?!
  const dso::Vec2f directions[16] = {
      dso::Vec2f(0, 1.0000),       dso::Vec2f(0.3827, 0.9239),
      dso::Vec2f(0.1951, 0.9808),  dso::Vec2f(0.9239, 0.3827),
      dso::Vec2f(0.7071, 0.7071),  dso::Vec2f(0.3827, -0.9239),
      dso::Vec2f(0.8315, 0.5556),  dso::Vec2f(0.8315, -0.5556),
      dso::Vec2f(0.5556, -0.8315), dso::Vec2f(0.9808, 0.1951),
      dso::Vec2f(0.9239, -0.3827), dso::Vec2f(0.7071, -0.7071),
      dso::Vec2f(0.5556, 0.8315),  dso::Vec2f(0.9808, -0.1951),
      dso::Vec2f(1.0000, 0.0000),  dso::Vec2f(0.1951, -0.9808)};

  // Clear map_out buffer
  std::fill(map_out, map_out + imageSizeInPixel, 0);

  const float dw1 = dso::setting_gradDownweightPerLevel;
  const float dw2 = dw1 * dw1;

  int n3 = 0, n2 = 0;//, n4 = 0;
  for (int y4 = 0; y4 < h; y4 += (4 * pot)) {
    for (int x4 = 0; x4 < w; x4 += (4 * pot)) {
      const int my3 = std::min((4 * pot), h - y4);
      const int mx3 = std::min((4 * pot), w - x4);

      int bestIdx4   = -1;
      float bestVal4 = 0;

      const dso::Vec2f dir4 = directions[randomPattern[n2] & 0xF];
      for (int y3 = 0; y3 < my3; y3 += (2 * pot)) {
        for (int x3 = 0; x3 < mx3; x3 += (2 * pot)) {
          const int x34 = x3 + x4;
          const int y34 = y3 + y4;

          const int my2 = std::min((2 * pot), h - y34);
          const int mx2 = std::min((2 * pot), w - x34);

          int bestIdx3 = -1;
          float bestVal3 = 0;

          const dso::Vec2f dir3 = directions[randomPattern[n2] & 0xF];

          for (int y2 = 0; y2 < my2; y2 += pot) {
            for (int x2 = 0; x2 < mx2; x2 += pot) {
              const int x234 = x2 + x34;
              const int y234 = y2 + y34;

              const int my1 = std::min(pot, h - y234);
              const int mx1 = std::min(pot, w - x234);

              int bestIdx2 = -1;
              float bestVal2 = 0;

              dso::Vec2f dir2 = directions[randomPattern[n2] & 0xF];
              for (int y1 = 0; y1 < my1; y1 += 1) {
                for (int x1 = 0; x1 < mx1; x1 += 1) {
                  assert(x1 + x234 < w);
                  assert(y1 + y234 < h);
                  const int idx = x1 + x234 + w * (y1 + y234);
                  const int xf = x1 + x234;
                  const int yf = y1 + y234;

                  if (xf < 4 || xf >= w - 5 || yf < 4 || yf > h - 4) {
                    continue;
                  }

                  const float pixelTH0 = thsSmoothed[(xf >> 5) + (yf >> 5) * thsStep];
                  const float pixelTH1 = pixelTH0 * dw1;
                  const float pixelTH2 = pixelTH1 * dw2;

                  const float ag0 = mapmax0[idx];
                  if (ag0 > pixelTH0 * thFactor) {
                    const dso::Vec2f ag0d = map0[idx].tail<2>();
                    float dirNorm = std::abs(ag0d.dot(dir2));
                    if (!dso::setting_selectDirectionDistribution) {
                      dirNorm = ag0;
                    }

                    if (dirNorm > bestVal2) {
                      bestVal2 = dirNorm;
                      bestIdx2 = idx;
                      bestIdx3 = -2;
                      bestIdx4 = -2;
                    }
                  }

                  if (bestIdx3 == -2) {
                    continue;
                  }

                  float ag1 = mapmax1[static_cast<int>(xf * 0.5f + 0.25f) +
                                      static_cast<int>(yf * 0.5f + 0.25f) * w1];
                  if (ag1 > pixelTH1 * thFactor) {
                    dso::Vec2f ag0d = map0[idx].tail<2>();
                    float dirNorm = std::abs(ag0d.dot(dir3));
                    if (!dso::setting_selectDirectionDistribution) {
                      dirNorm = ag1;
                    }

                    if (dirNorm > bestVal3) {
                      bestVal3 = dirNorm;
                      bestIdx3 = idx;
                      bestIdx4 = -2;
                    }
                  }

                  if (bestIdx4 == -2) {
                    continue;
                  }

                  const float ag2 = mapmax2[static_cast<int>(xf * 0.25f + 0.125) +
                                      static_cast<int>(yf * 0.25f + 0.125) * w2];
                  if (ag2 > pixelTH2 * thFactor) {
                    const dso::Vec2f ag0d = map0[idx].tail<2>();
                    float dirNorm = std::abs(ag0d.dot(dir4));
                    if (!dso::setting_selectDirectionDistribution) {
                      dirNorm = ag2;
                    }

                    if (dirNorm > bestVal4) {
                      bestVal4 = dirNorm;
                      bestIdx4 = idx;
                    }
                  }
                }
              }

              if (bestIdx2 > 0) {
                map_out[bestIdx2] = 1;
                bestVal3 = 1e10;
                n2++;
              }
            }
          }

          if (bestIdx3 > 0) {
            map_out[bestIdx3] = 2;
            bestVal4 = 1e10;
            n3++;
          }
        }
      }
    }
  }
}
