//
// Created by bsespede on 3/14/20.
//

#include "ass1/CameraCalib.h"

CameraCalib::CameraCalib(cv::Size patternSize, float squareSize) : _patternSize(patternSize), _squareSize(squareSize),
_hasComputedIntrinsics(false) {}

void CameraCalib::computeIntrinsics(std::string inputPath, bool saveResults) {
  if (!boost::filesystem::is_directory(inputPath)) {
    throw std::runtime_error("Input folder does not exist");
  }

  printf("[DEBUG] Computing input and object points\n");
  computeCornerPoints(inputPath);
  cv::Size imageSize = ImageUtils::getImagesSize(inputPath);
  std::vector<cv::Mat> rvecs = std::vector<cv::Mat>(_imagePoints.size());
  std::vector<cv::Mat> tvecs = std::vector<cv::Mat>(_imagePoints.size());
  cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);

  printf("[DEBUG] Computing camera calibration\n");
  std::vector<std::vector<cv::Point2f>> imagePointsUnraveled;
  std::vector<std::vector<cv::Point3f>> objectPointsUnraveled;
  for (int imageNumber = 0; imageNumber < _imagePoints.size(); imageNumber++) {
    imagePointsUnraveled.push_back(_imagePoints[imageNumber].points);
    objectPointsUnraveled.push_back(_objectPoints[imageNumber].points);
  }

  float rmse = cv::calibrateCamera(objectPointsUnraveled, imagePointsUnraveled, imageSize, cameraMatrix, distCoeffs,
      rvecs, tvecs);

  _intrinsics = {cameraMatrix, distCoeffs, rmse};
  _hasComputedIntrinsics = true;

  if (saveResults) {
    printf("[DEBUG] Saving results\n");
    computeResultImages(inputPath);
  }
}

std::vector<CameraCalib::ImagePoints> CameraCalib::getImagePoints() {
  if (!_hasComputedIntrinsics) {
    throw std::runtime_error("Calibration has not been computed yet");
  }
  return _imagePoints;
}

std::vector<CameraCalib::ObjectPoints> CameraCalib::getObjectPoints() {
  if (!_hasComputedIntrinsics) {
    throw std::runtime_error("Calibration has not been computed yet");
  }
  return _objectPoints;
}

CameraCalib::Intrinsics CameraCalib::getIntrinsics() {
  if (!_hasComputedIntrinsics) {
    throw std::runtime_error("Calibration has not been computed yet");
  }
  return _intrinsics;
}

void CameraCalib::computeCornerPoints(std::string inputPath) {
  printf("[DEBUG] Computing corner points\n");
  _imagePoints = std::vector<ImagePoints>();
  _objectPoints = std::vector<ObjectPoints>();

  std::vector<cv::Point3f> curObjectPoints;
  for (int i = 0; i < _patternSize.height; ++i) {
    for (int j = 0; j < _patternSize.width; ++j) {
      curObjectPoints.push_back(cv::Point3f(j * _squareSize, i * _squareSize, 0));
    }
  }

  std::vector<std::string> imagesPath = ImageUtils::getImagesPath(inputPath);
  for (std::string imagePath : imagesPath) {
    cv::Mat image = cv::imread(imagePath, cv::IMREAD_COLOR);
    std::vector<cv::Point2f> curImagePoints;
    bool foundCorners = cv::findChessboardCorners(image, _patternSize, curImagePoints);
    if (foundCorners) {
      cv::Mat imageGray;
      cvtColor(image, imageGray, cv::COLOR_BGR2GRAY);
      cornerSubPix(imageGray, curImagePoints, cv::Size(11, 11), cv::Size(-1, -1),
                   cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.0001));
      _imagePoints.push_back({curImagePoints, boost::filesystem::path(imagePath).stem().string()});
      _objectPoints.push_back({curObjectPoints});
    }
  }
}

void CameraCalib::computeResultImages(std::string inputPath) {
  if (!_hasComputedIntrinsics) {
    throw std::runtime_error("Intrinsics have not been computed yet");
  }

  std::string outputPath = inputPath + "/output";
  std::string cornersFolderPath = outputPath + "/corners";
  std::string undistortedFolderPath = outputPath + "/undistorted";
  if (!boost::filesystem::is_directory(outputPath)) {
    boost::filesystem::create_directories(outputPath);
  }
  if (!boost::filesystem::is_directory(cornersFolderPath)) {
    boost::filesystem::create_directories(cornersFolderPath);
  }
  if (!boost::filesystem::is_directory(undistortedFolderPath)) {
    boost::filesystem::create_directories(undistortedFolderPath);
  }

  for (const ImagePoints& imagePoints : _imagePoints) {
    std::string filename = imagePoints.filename + ".png";
    cv::Mat patternImage = cv::imread(inputPath + "/" + filename, cv::IMREAD_COLOR);
    cv::Mat patternCorners = patternImage.clone();
    cv::Mat patternUndistorted = patternImage.clone();
    cv::drawChessboardCorners(patternCorners, _patternSize, imagePoints.points, true);
    cv::undistort(patternImage, patternUndistorted, _intrinsics.cameraMatrix, _intrinsics.distCoeffs);
    cv::imwrite(cornersFolderPath + "/" + filename, patternCorners);
    cv::imwrite(undistortedFolderPath + "/" + filename, patternUndistorted);
  }
}