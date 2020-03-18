//
// Created by bsespede on 3/14/20.
//

#ifndef ROBOT_VISION_SRC_ASS1_CAMERACALIB_H_
#define ROBOT_VISION_SRC_ASS1_CAMERACALIB_H_

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <vector>
#include "ass1/ImageUtils.h"

class CameraCalib {
 public:
  struct Intrinsics {
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    float rmse;
  };
  struct ImagePoints {
    std::vector<cv::Point2f> points;
    std::string filename;
  };
  struct ObjectPoints {
    std::vector<cv::Point3f> points;
  };

  CameraCalib(cv::Size patternSize, float squareSize);
  void computeIntrinsics(std::string inputPath);
  Intrinsics getIntrinsics();
  std::vector<ImagePoints> getImagePoints();
  std::vector<ObjectPoints> getObjectPoints();

 private:
  void computeCornerPoints(std::string inputPath);
  cv::Size _patternSize;
  float _squareSize;
  bool _hasComputedIntrinsics;

  std::vector<ImagePoints> _imagePoints;
  std::vector<ObjectPoints> _objectPoints;
  Intrinsics _intrinsics;
};

#endif //ROBOT_VISION_SRC_ASS1_CAMERACALIB_H_
