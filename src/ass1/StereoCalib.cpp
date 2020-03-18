//
// Created by bsespede on 3/15/20.
//

#include "ass1/StereoCalib.h"

StereoCalib::StereoCalib(cv::Size patternSize, float squareSize) : _patternSize(patternSize), _squareSize(squareSize),
_leftCalibration(patternSize, squareSize), _rightCalibration(patternSize, squareSize), _hasComputedExtrinsics(false) {}

void StereoCalib::computeExtrinsics(std::string inputPath) {
  if (!boost::filesystem::is_directory(inputPath)) {
    throw std::runtime_error("Input folder does not exist");
  }

  std::string leftImagesPath = inputPath + "/left";
  std::string rightImagesPath = inputPath + "/right";
  if (!boost::filesystem::is_directory(leftImagesPath) || !boost::filesystem::is_directory(rightImagesPath)) {
    throw std::runtime_error("Input folder must contain left and right folders");
  }

  printf("[DEBUG] Checking for valid pairs\n");
  if (ImageUtils::getImagesSize(leftImagesPath) != ImageUtils::getImagesSize(rightImagesPath)) {
    throw std::runtime_error("Images should have the same size");
  }

  printf("[DEBUG] Computing intrinsics for left camera\n");
  _leftCalibration.computeIntrinsics(inputPath + "/left");

  printf("[DEBUG] Computing intrinsics for right camera\n");
  _rightCalibration.computeIntrinsics(inputPath + "/right");

  std::vector<std::vector<cv::Point2f>> leftImagePointsUnraveled;
  std::vector<std::vector<cv::Point2f>> rightImagePointsUnraveled;
  std::vector<std::vector<cv::Point3f>> objectPointsUnraveled;
  for (CameraCalib::ImagePoints leftImagePoints: _leftCalibration.getImagePoints()) {
    for (CameraCalib::ImagePoints rightImagePoints: _rightCalibration.getImagePoints()) {
      if (leftImagePoints.filename == rightImagePoints.filename) {
        leftImagePointsUnraveled.push_back(leftImagePoints.points);
        rightImagePointsUnraveled.push_back(rightImagePoints.points);
        objectPointsUnraveled.push_back(_leftCalibration.getObjectPoints()[0].points);
        break;
      }
    }
  }

  printf("[DEBUG] Computing extrinsics\n");
  cv::Mat leftIntrinsics = _leftCalibration.getIntrinsics().cameraMatrix;
  cv::Mat rightIntrinsics = _rightCalibration.getIntrinsics().cameraMatrix;
  cv::Mat leftDistortions = _leftCalibration.getIntrinsics().distCoeffs;
  cv::Mat rightDistortions = _rightCalibration.getIntrinsics().distCoeffs;
  cv::Size imageSize = ImageUtils::getImagesSize(leftImagesPath);
  cv::Mat rotationMatrix;
  cv::Mat translationMatrix;
  cv::Mat essentialMatrix;
  cv::Mat fundamentalMatrix;
  float rmse = cv::stereoCalibrate(objectPointsUnraveled, leftImagePointsUnraveled, rightImagePointsUnraveled, leftIntrinsics, leftDistortions,
  rightIntrinsics, rightDistortions, imageSize, rotationMatrix, translationMatrix, essentialMatrix, fundamentalMatrix);

  _extrinsics = {rotationMatrix, translationMatrix, rmse};
  _hasComputedExtrinsics = true;
}

CameraCalib::Intrinsics StereoCalib::getLeftIntrinsics() {
  if (!_hasComputedExtrinsics) {
    throw std::runtime_error("Stereo calibration has not been computed yet");
  }
  return _leftCalibration.getIntrinsics();
}
CameraCalib::Intrinsics StereoCalib::getRightIntrinsics() {
  if (!_hasComputedExtrinsics) {
    throw std::runtime_error("Stereo calibration has not been computed yet");
  }
  return _rightCalibration.getIntrinsics();
}

StereoCalib::Extrinsics StereoCalib::getExtrinsics() {
  if (!_hasComputedExtrinsics) {
    throw std::runtime_error("Stereo calibration has not been computed yet");
  }
  return _extrinsics;
}