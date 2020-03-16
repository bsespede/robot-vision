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
  struct Result {
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    double rmse;
  };

  struct ImagePoints {
    std::vector<cv::Point2f> points;
    std::string filename;
  };

  struct ObjectPoints {
    std::vector<cv::Point3f> points;
  };

 public:
  CameraCalib(cv::Size patternSize, float squareSize);
  void computeIntrinsics(std::string inputPath, bool printResults = false);
 private:
  void computeImagePoints(std::string inputPath, std::vector<ImagePoints>& imagePoints);
  void computeObjectPoints(std::vector<ObjectPoints>& objectPoints, std::vector<ImagePoints>& imagePoints);
  float computeError(std::vector<ObjectPoints>& objectPoints, std::vector<ImagePoints>& imagePoints, cv::Mat& cameraMatrix, cv::Mat& distCoeffs, std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs);

  std::vector<std::string> getImagesPath(std::string imagesPath);
  cv::Size getImagesSize(std::string inputPath);
  void saveResults(std::string outputPath, std::vector<ImagePoints>& imagePoints, CameraCalib::Result& result, bool printResults);

  cv::Size patternSize;
  float squareSize;
};

#endif //ROBOT_VISION_SRC_ASS1_CAMERACALIB_H_
