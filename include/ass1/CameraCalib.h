//
// Created by bsespede on 3/14/20.
//

#ifndef ROBOT_VISION_SRC_ASS1_CAMERACALIB_H_
#define ROBOT_VISION_SRC_ASS1_CAMERACALIB_H_

#include <opencv2/core.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

class CameraCalib {
 public:
  CameraCalib(cv::Size boardSize, float squareLength);
  void computeIntrinsics(std::string inputPath, std::string outputPath, bool printResults);
 private:
  void printDetections(std::string outputPath);
  void detectCorners(cv::Mat boardImage);

  cv::Size boardSize;
  float squareLength;
  std::vector<std::vector<cv::Point2f>> detections;
};

#endif //ROBOT_VISION_SRC_ASS1_CAMERACALIB_H_
