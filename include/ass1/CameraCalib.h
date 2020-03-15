//
// Created by bsespede on 3/14/20.
//

#ifndef ROBOT_VISION_SRC_ASS1_CAMERACALIB_H_
#define ROBOT_VISION_SRC_ASS1_CAMERACALIB_H_

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/filesystem.hpp>

#include <vector>
#include <exception>

class CameraCalib {
 public:
  CameraCalib(cv::Size boardSize, float squareLength);
  void computeIntrinsics(std::string inputPath, std::string outputPath, bool printResults);
 private:
  std::vector<std::string> getImagesPath(std::string path);
  void printDetections(std::string outputPath);
  void detectCorners(cv::Mat boardImage);

  cv::Size boardSize;
  float squareLength;
  std::vector<std::vector<cv::Point2f>> corners2D;
  std::vector<std::vector<cv::Point3f>> corners3D;
};

#endif //ROBOT_VISION_SRC_ASS1_CAMERACALIB_H_
