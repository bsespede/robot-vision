//
// Created by bsespede on 3/15/20.
//

#include "ass1/StereoMatch.h"


void StereoCalib::computeResultImages(std::string inputPath) {
  if (!_hasComputedExtrinsics) {
    throw std::runtime_error("Stereo calibration has not been computed yet");
  }

  std::string outputPathLeft = inputPath + "/left/output";
  std::string outputPathRight = inputPath + "/right/output";
  std::string rectifiedFolderPathLeft = outputPathLeft + "/rectified";
  std::string rectifiedFolderPathRight = outputPathRight + "/rectified";

  if (!boost::filesystem::is_directory(outputPathLeft)) {
    boost::filesystem::create_directories(outputPathLeft);
  }
  if (!boost::filesystem::is_directory(outputPathRight)) {
    boost::filesystem::create_directories(outputPathRight);
  }
  if (!boost::filesystem::is_directory(rectifiedFolderPathLeft)) {
    boost::filesystem::create_directories(rectifiedFolderPathLeft);
  }
  if (!boost::filesystem::is_directory(rectifiedFolderPathRight)) {
    boost::filesystem::create_directories(rectifiedFolderPathRight);
  }

  cv::Mat leftRectification;
  cv::Mat rightRectification;
  cv::Mat leftProjection;
  cv::Mat rightProjection;
  cv::Mat disparityToDepth;
  cv::Mat leftMapX;
  cv::Mat leftMapY;
  cv::Mat rightMapX;
  cv::Mat rightMapY;
  cv::Size imageSize = ImageUtils::getImagesSize(inputPath + "/left");

  cv::stereoRectify(_leftCalibration.getIntrinsics().cameraMatrix, _leftCalibration.getIntrinsics().distCoeffs,
                    _rightCalibration.getIntrinsics().cameraMatrix, _rightCalibration.getIntrinsics().distCoeffs,
                    imageSize, _extrinsics.rotationMatrix, _extrinsics.transVector,
                    leftRectification, rightRectification, leftProjection, rightProjection, disparityToDepth);

  cv::initUndistortRectifyMap(_leftCalibration.getIntrinsics().cameraMatrix, _leftCalibration.getIntrinsics().distCoeffs,
                              leftRectification, leftProjection, imageSize, CV_32FC1, leftMapX, leftMapY);
  cv::initUndistortRectifyMap(_rightCalibration.getIntrinsics().cameraMatrix, _rightCalibration.getIntrinsics().distCoeffs,
                              rightRectification, rightProjection, imageSize, CV_32FC1, rightMapX, rightMapY);

  for (CameraCalib::ImagePoints leftImagePoints : _leftCalibration.getImagePoints()) {
    std::string filename = leftImagePoints.filename + ".png";
    cv::Mat patternImage = cv::imread(inputPath + "/left/" + filename, cv::IMREAD_COLOR);
    cv::Mat patternUndistorted = patternImage.clone();
    cv::remap(patternImage, patternUndistorted, leftMapX, leftMapY, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
    cv::imwrite(rectifiedFolderPathLeft + "/" + filename, patternUndistorted);
  }

  for (CameraCalib::ImagePoints rightImagePoints : _rightCalibration.getImagePoints()) {
    std::string filename = rightImagePoints.filename + ".png";
    cv::Mat patternImage = cv::imread(inputPath + "/right/" + filename, cv::IMREAD_COLOR);
    cv::Mat patternUndistorted = patternImage.clone();
    cv::remap(patternImage, patternUndistorted, rightMapX, rightMapY, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
    cv::imwrite(rectifiedFolderPathRight + "/" + filename, patternUndistorted);
  }
}