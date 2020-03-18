//
// Created by bsespede on 3/15/20.
//

#ifndef ROBOT_VISION_SRC_ASS1_STEREOMATCH_H_
#define ROBOT_VISION_SRC_ASS1_STEREOMATCH_H_

#include "ass1/StereoCalib.h"

class StereoMatch {
 public:
  void computeCalibration(std::string inputPath, cv::Size patternSize, float squareSize);
  void computeRectifiedImages(std::string inputPath, std::string outputPath);
  void computeDisparityMaps(std::string inputPath, std::string outputPath);
  void computePointCloud(std::string inputPath, std::string outputPath);
 private:
  bool _hasCalibratedCameras;
  StereoCalib _calibration;
};

#endif //ROBOT_VISION_SRC_ASS1_STEREOMATCH_H_
