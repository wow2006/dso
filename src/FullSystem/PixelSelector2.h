/**
 * This file is part of DSO.
 *
 * Copyright 2016 Technical University of Munich and Intel.
 * Developed by Jakob Engel <engelj at in dot tum dot de>,
 * for more information see <http://vision.in.tum.de/dso>.
 * If you use this code, please cite the respective publications as
 * listed on the above website.
 *
 * DSO is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * DSO is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with DSO. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once
// Internal
#include "util/NumType.h"

namespace dso {

enum PixelSelectorStatus { PIXSEL_VOID = 0, PIXSEL_1, PIXSEL_2, PIXSEL_3 };

class FrameHessian;

class PixelSelector {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PixelSelector(int w, int h);

  ~PixelSelector();

  int makeMaps(const FrameHessian *const fh, float *map_out, float density,
               int recursionsLeft = 1, bool plot = false, float thFactor = 1);

  void draw(const FrameHessian *const fh, float *map_out);

  int currentPotential;

  bool allowFast;

private:
  void makeHists(const FrameHessian *const fh);

  Eigen::Vector3i select(const FrameHessian *const fh, float *map_out, int pot,
                         float thFactor = 1);

  int thsStep;
  std::vector<unsigned char> randomPattern;
  std::vector<int> gradHist;
  std::vector<float> ths;
  std::vector<float> thsSmoothed;
  const FrameHessian *gradHistFrame;
};

} // namespace dso
