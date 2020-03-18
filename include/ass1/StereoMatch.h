//
// Created by bsespede on 3/15/20.
//

#ifndef ROBOT_VISION_SRC_ASS1_STEREOMATCH_H_
#define ROBOT_VISION_SRC_ASS1_STEREOMATCH_H_

#include "ass1/StereoCalib.h"

class StereoMatch {
 public:
  StereoMatch(StereoCalib calibration);
  void computePointCloud(std::string inputPath);

 private:
  void computeRectifiedImages(std::string inputPath);
  void computeDisparityMaps(std::string inputPath);

  StereoCalib _calibration;
};

#endif //ROBOT_VISION_SRC_ASS1_STEREOMATCH_H_
