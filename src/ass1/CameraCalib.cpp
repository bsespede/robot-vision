//
// Created by bsespede on 3/14/20.
//

#include <iostream>
#include "ass1/CameraCalib.h"

CameraCalib::CameraCalib(cv::Size boardSize, float squareLength) : boardSize(boardSize), squareLength(squareLength),
corners2D(), corners3D() { }

void CameraCalib::computeIntrinsics(std::string inputPath, std::string outputPath, bool printResults) {
  if (!boost::filesystem::is_directory(outputPath)) {
    boost::filesystem::create_directories(outputPath);
    if (printResults) {
      boost::filesystem::create_directories(outputPath + "/corners");
      boost::filesystem::create_directories(outputPath + "/undistorted");
    }
  }
  std::vector<std::string> imagesPath = getImagesPath(inputPath);
  cv::Size imageSize;

  // Extract checkerboard corners
  for (std::string imagePath : imagesPath) {
    cv::Mat image = cv::imread(imagePath, cv::IMREAD_COLOR);
    imageSize = image.size();

    std::vector<cv::Point2f> corners;
    bool foundCorners = cv::findChessboardCorners(image, boardSize, corners);
    if (foundCorners) {
      cv::Mat imageGray;
      cvtColor(image, imageGray, cv::COLOR_BGR2GRAY);

      // TODO: try changing this values and see the effect in the RMSE
      int windowSize = 11;
      int maxIter = 30;
      float epsilon = 0.0001;
      cornerSubPix(imageGray, corners, cv::Size(windowSize, windowSize), cv::Size(-1,-1),
          cv::TermCriteria( cv::TermCriteria::EPS + cv::TermCriteria::COUNT, maxIter, epsilon));

      corners2D.push_back(corners);
    }

    if (printResults && foundCorners) {
      std::string outputImageFilename = boost::filesystem::path(imagePath).stem().string() + "_output.png";
      cv::drawChessboardCorners(image, boardSize, corners, foundCorners);
      cv::imwrite(outputPath + "/corners/" + outputImageFilename, image);
    }
  }

  // Corresponding points in the 3D coordinate system of the boards
  for (int imageIndex = 0; imageIndex < corners2D.size(); imageIndex++) {
    std::vector<cv::Point3f> corners;
    for( int i = 0; i < boardSize.height; ++i) {
      for( int j = 0; j < boardSize.width; ++j) {
        corners.push_back(cv::Point3f(j * squareLength, i * squareLength, 0));
      }
    }
    corners3D.push_back(corners);
  }

  // Perform the actual calibration
  cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
  std::vector<cv::Mat> rvecs = std::vector<cv::Mat>(corners2D.size());
  std::vector<cv::Mat> tvecs = std::vector<cv::Mat>(corners2D.size());
  cv::calibrateCamera(corners3D, corners2D, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs);

  std::cout << cameraMatrix << std::endl;
}

std::vector<std::string> CameraCalib::getImagesPath(std::string path) {
  std::vector<std::string> imagesPath;
  if (boost::filesystem::is_directory(path)) {
    for (auto& file : boost::filesystem::recursive_directory_iterator(path)) {
      if (file.path().extension() == ".png") {
        imagesPath.push_back(file.path().string());
      }
    }
  }
  else {
    throw std::runtime_error("Input folder doesn't exist");
  }
  return imagesPath;
}

void CameraCalib::printDetections(std::string outputPath) {

}

void CameraCalib::detectCorners(cv::Mat boardImage) {

}