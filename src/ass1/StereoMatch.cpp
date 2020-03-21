//
// Created by bsespede on 3/15/20.
//

#include <ass1/PointCloud.h>
#include "ass1/StereoMatch.h"

StereoMatch::StereoMatch(std::string inputPath, StereoCalib calibration) : _inputPath(inputPath),
_calibration(calibration) {}

void StereoMatch::computeRectification() {
  if (!_calibration.hasCalibrated()) {
    throw std::runtime_error("Calibration has not been computed");
  }

  std::string inputPathLeft = _inputPath + "/left";
  std::string inputPathRight = _inputPath + "/right";
  std::string outputPathLeft = inputPathLeft + "/rectified";
  std::string outputPathRight = inputPathRight + "/rectified";
  boost::filesystem::create_directories(outputPathLeft);
  boost::filesystem::create_directories(outputPathRight);

  cv::Mat leftRectification;
  cv::Mat rightRectification;
  cv::Mat leftProjection;
  cv::Mat rightProjection;
  cv::Mat disparityToDepth;
  CameraCalib::Intrinsics leftIntrinsics = _calibration.getLeftIntrinsics();
  CameraCalib::Intrinsics rightIntrinsics = _calibration.getRightIntrinsics();
  StereoCalib::Extrinsics extrinsics = _calibration.getExtrinsics();
  cv::Size imageSize = _calibration.getLeftIntrinsics().imageSize;

  printf("[DEBUG] Computing projection matrices\n");
  cv::stereoRectify(leftIntrinsics.cameraMatrix, leftIntrinsics.distCoeffs,
                    rightIntrinsics.cameraMatrix, rightIntrinsics.distCoeffs,
                    imageSize, extrinsics.rotationMatrix, extrinsics.transVector,
                    leftRectification, rightRectification, leftProjection, rightProjection, disparityToDepth);
  _projectionMatrix = disparityToDepth;

  printf("[DEBUG] Computing mapping matrices\n");
  cv::Mat leftMapX;
  cv::Mat leftMapY;
  cv::Mat rightMapX;
  cv::Mat rightMapY;
  cv::initUndistortRectifyMap(leftIntrinsics.cameraMatrix, leftIntrinsics.distCoeffs,
                              leftRectification, leftProjection, imageSize, CV_16SC2, leftMapX, leftMapY);
  cv::initUndistortRectifyMap(rightIntrinsics.cameraMatrix, rightIntrinsics.distCoeffs,
                              rightRectification, rightProjection, imageSize, CV_16SC2, rightMapX, rightMapY);

  printf("[DEBUG] Rectifying left camera images\n");
  std::vector<std::string> imagesPathLeft = ImageUtils::getImagesPath(inputPathLeft);
  for (std::string imagePath : imagesPathLeft) {
    cv::Mat leftImage = cv::imread(imagePath, cv::IMREAD_COLOR);
    cv::Mat leftImageUndistorted;
    cv::remap(leftImage, leftImageUndistorted, leftMapX, leftMapY, cv::INTER_LINEAR);

    std::string filename = boost::filesystem::path(imagePath).filename().string();
    cv::imwrite(outputPathLeft + "/" + filename, leftImageUndistorted);
  }

  printf("[DEBUG] Rectifying right camera images\n");
  std::vector<std::string> imagesPathRight = ImageUtils::getImagesPath(inputPathRight);
  for (std::string imagePath : imagesPathRight) {
    cv::Mat rightImage = cv::imread(imagePath, cv::IMREAD_COLOR);
    cv::Mat rightImageUndistorted;
    cv::remap(rightImage, rightImageUndistorted, rightMapX, rightMapY, cv::INTER_LINEAR);

    std::string filename = boost::filesystem::path(imagePath).filename().string();
    cv::imwrite(outputPathRight + "/" + filename, rightImageUndistorted);
  }
}

void StereoMatch::computeDisparityMaps() {
  if (!_hasRectified) {
    throw std::runtime_error("Rectification has not been computed");
  }

  std::string inputPathLeft = _inputPath + "/left/rectified";
  std::string inputPathRight = _inputPath + "/right/rectified";
  std::string outputPath = _inputPath + "/disparity";
  boost::filesystem::create_directories(outputPath);

  printf("[DEBUG] Computing disparity map\n");
  std::vector<std::string> imagesPathLeft = ImageUtils::getImagesPath(inputPathLeft);
  for (std::string leftImagePath : imagesPathLeft) {
    std::string filename = boost::filesystem::path(leftImagePath).filename().string();
    std::string rightImagePath = inputPathRight + "/" + filename;
    cv::Mat leftImage = cv::imread(leftImagePath, cv::IMREAD_ANYCOLOR);
    cv::Mat rightImage = cv::imread(rightImagePath, cv::IMREAD_ANYCOLOR);
    cv::Mat disparityMap;

    int channels = 3;
    int windowSize = 5;
    cv::Ptr<cv::StereoSGBM> stereoAlgorithm = cv::StereoSGBM::create();
    stereoAlgorithm->setMode(cv::StereoSGBM::MODE_HH);
    stereoAlgorithm->setP1(8 * channels * windowSize * windowSize);
    stereoAlgorithm->setP2(32 * channels * windowSize * windowSize);
    stereoAlgorithm->setBlockSize(windowSize);
    stereoAlgorithm->setMinDisparity(0);
    stereoAlgorithm->setNumDisparities(32);
    stereoAlgorithm->setDisp12MaxDiff(1);
    stereoAlgorithm->setUniquenessRatio(10);
    stereoAlgorithm->setSpeckleWindowSize(100);
    stereoAlgorithm->setSpeckleRange(32);
    stereoAlgorithm->compute(leftImage, rightImage, disparityMap);

    disparityMap.convertTo(disparityMap, CV_32F, 1.0f / 16.0f);
    cv::imwrite(outputPath + "/" + filename, disparityMap);
  }
}

void StereoMatch::computePointClouds() {
  if (!_hasRectified) {
    throw std::runtime_error("Rectification has not been computed");
  }

  std::string inputImagesPath = _inputPath + "/left";
  std::string inputDisparityPath = _inputPath + "/disparity";
  std::vector<std::string> imagesPath = ImageUtils::getImagesPath(inputImagesPath);
  for (std::string imagePath : imagesPath) {
    std::string filename = boost::filesystem::path(imagePath).stem().string();
    PointCloud pointCloud(_inputPath, filename, _projectionMatrix);
    pointCloud.computePointCloud();
  }
}
