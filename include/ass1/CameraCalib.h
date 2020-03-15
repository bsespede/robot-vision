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
  CameraCalib(cv::Size patternSize, float squareSize);
  void computeIntrinsics(std::string inputPath, bool printResults = false);
 private:
  cv::Size getImageSize(std::string inputPath);
  std::vector<std::vector<cv::Point2f>> getImagePoints(std::string inputPath, std::string outputPath, bool printResults);
  std::vector<std::vector<cv::Point3f>> getObjectPoints(int imagesNumber);
  std::vector<std::string> getImagesPath(std::string imagesPath);
  double getRMSE(std::vector<std::vector<cv::Point3f>> objectPoints, std::vector<std::vector<cv::Point2f>> imagePoints,
      cv::Mat cameraMatrix, cv::Mat distCoeffs, std::vector<cv::Mat> rvecs, std::vector<cv::Mat> tvecs);
  void storeResults(std::string outputPath, cv::Mat cameraMatrix, cv::Mat distCoeffs, double rmse);

  cv::Size patternSize;
  float squareSize;
};

#endif //ROBOT_VISION_SRC_ASS1_CAMERACALIB_H_
