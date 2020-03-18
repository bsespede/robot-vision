//
// Created by bsespede on 3/15/20.
//

#include "ass1/StereoMatch.h"

StereoMatch::StereoMatch(StereoCalib calibration) : _calibration(calibration) {}

void StereoMatch::computeRectifiedImages(std::string inputPath) {
  if (!_calibration.hasCalibrated()) {
    throw std::runtime_error("Calibration has not been computed");
  }

  std::string inputPathLeft = inputPath + "/left";
  std::string inputPathRight = inputPath + "/right";
  std::string outputPathLeft = inputPathLeft + "/rectified";
  std::string outputPathRight = inputPathRight + "/rectified";
  boost::filesystem::create_directories(outputPathLeft);
  boost::filesystem::create_directories(outputPathRight);

  cv::Mat leftRectification;
  cv::Mat rightRectification;
  cv::Mat leftProjection;
  cv::Mat rightProjection;
  cv::Mat disparityToDepth;
  cv::Mat leftMapX;
  cv::Mat leftMapY;
  cv::Mat rightMapX;
  cv::Mat rightMapY;

  CameraCalib::Intrinsics leftIntrinsics = _calibration.getLeftIntrinsics();
  CameraCalib::Intrinsics rightIntrinsics = _calibration.getRightIntrinsics();
  StereoCalib::Extrinsics extrinsics = _calibration.getExtrinsics();
  cv::Size imageSize = _calibration.getLeftIntrinsics().imageSize;

  printf("[DEBUG] Computing projection matrices\n");
  cv::stereoRectify(leftIntrinsics.cameraMatrix, leftIntrinsics.distCoeffs,
                    rightIntrinsics.cameraMatrix, rightIntrinsics.distCoeffs,
                    imageSize, extrinsics.rotationMatrix, extrinsics.transVector,
                    leftRectification, rightRectification, leftProjection, rightProjection, disparityToDepth);

  printf("[DEBUG] Computing mapping matrices\n");
  cv::initUndistortRectifyMap(leftIntrinsics.cameraMatrix, leftIntrinsics.distCoeffs,
                              leftRectification, leftProjection, imageSize, CV_32FC1, leftMapX, leftMapY);
  cv::initUndistortRectifyMap(rightIntrinsics.cameraMatrix, rightIntrinsics.distCoeffs,
                              rightRectification, rightProjection, imageSize, CV_32FC1, rightMapX, rightMapY);

  printf("[DEBUG] Rectifying left camera images\n");
  std::vector<std::string> imagesPathLeft = ImageUtils::getImagesPath(inputPathLeft);
  for (std::string imagePath : imagesPathLeft) {
    cv::Mat patternImage = cv::imread(imagePath, cv::IMREAD_COLOR);
    cv::Mat patternUndistorted = patternImage.clone();
    cv::remap(patternImage, patternUndistorted, leftMapX, leftMapY, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
    std::string filename = boost::filesystem::path(imagePath).filename().string();
    cv::imwrite(outputPathLeft + "/" + filename, patternUndistorted);
  }

  printf("[DEBUG] Rectifying right camera images\n");
  std::vector<std::string> imagesPathRight = ImageUtils::getImagesPath(inputPathRight);
  for (std::string imagePath : imagesPathRight) {
    cv::Mat patternImage = cv::imread(imagePath, cv::IMREAD_COLOR);
    cv::Mat patternUndistorted = patternImage.clone();
    cv::remap(patternImage, patternUndistorted, leftMapX, leftMapY, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
    std::string filename = boost::filesystem::path(imagePath).filename().string();
    cv::imwrite(outputPathRight + "/" + filename, patternUndistorted);
  }
}

void StereoMatch::computeDisparityMaps(std::string inputPath) {
  std::string inputPathLeft = inputPath + "/left/rectified";
  std::string inputPathRight = inputPath + "/right/rectified";
  std::string outputPathLeftToRight = inputPath + "/disparity";
  boost::filesystem::create_directories(outputPathLeftToRight);

  int channels = 3;
  int windowSize = 5;
  cv::Ptr<cv::StereoSGBM> stereoAlgorithm = cv::StereoSGBM::create();

  printf("[DEBUG] Computing disparity map\n");
  std::vector<std::string> imagesPathLeft = ImageUtils::getImagesPath(inputPathLeft);
  for (std::string leftImagePath : imagesPathLeft) {
    std::string filename = boost::filesystem::path(leftImagePath).filename().string();
    std::string rightImagePath = inputPathRight + "/" + filename;

    cv::Mat leftImage = cv::imread(leftImagePath, cv::IMREAD_ANYCOLOR);
    cv::Mat rightImage = cv::imread(rightImagePath, cv::IMREAD_ANYCOLOR);
    cv::Mat disparityMap = cv::Mat(leftImage.size(), CV_16FC1);

    stereoAlgorithm->setP1(8 * channels * windowSize * windowSize);
    stereoAlgorithm->setP2(32 * channels * windowSize * windowSize);
    stereoAlgorithm->setMinDisparity(0);
    stereoAlgorithm->setNumDisparities(((leftImage.cols / 8) + 15) & -16);
    stereoAlgorithm->setUniquenessRatio(10);
    stereoAlgorithm->setSpeckleWindowSize(100);
    stereoAlgorithm->setSpeckleRange(32);
    stereoAlgorithm->setDisp12MaxDiff(1);
    stereoAlgorithm->compute(leftImage, rightImage, disparityMap);
    cv::imwrite(outputPathLeftToRight + "/disparity.png", disparityMap);
  }
}

void StereoMatch::computePointCloud(std::string inputPath) {
  computeRectifiedImages(inputPath);
  computeDisparityMaps(inputPath);
}
