//
// Created by bsespede on 3/15/20.
//

#ifndef ROBOT_VISION_SRC_ASS1_STEREOCALIB_H_
#define ROBOT_VISION_SRC_ASS1_STEREOCALIB_H_

#include "ass1/CameraCalib.h"

class StereoCalib {
  struct Extrinsics {
    cv::Mat rotationMatrix;
    cv::Mat transVector;
    float rmse;
  };

 public:
  StereoCalib(cv::Size patternSize, float squareSize);
  void computeExtrinsics(std::string inputPath);
  CameraCalib::Intrinsics getLeftIntrinsics();
  CameraCalib::Intrinsics getRightIntrinsics();
  Extrinsics getExtrinsics();

 private:
  cv::Size _patternSize;
  float _squareSize;
  bool _hasComputedExtrinsics;

  CameraCalib _leftCalibration;
  CameraCalib _rightCalibration;
  Extrinsics _extrinsics;
};

#endif //ROBOT_VISION_SRC_ASS1_STEREOCALIB_H_
