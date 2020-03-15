//
// Created by bsespede on 3/14/20.
//

#include <iostream>
#include "ass1/CameraCalib.h"

CameraCalib::CameraCalib(cv::Size patternSize, float squareSize) : patternSize(patternSize), squareSize(squareSize) {}

void CameraCalib::computeIntrinsics(std::string inputPath, bool printResults) {
  printf("[DEBUG] Computing intrinsic camera parameters\n");
  std::string outputPath = inputPath + "/output";
  if (!boost::filesystem::is_directory(outputPath)) {
    boost::filesystem::create_directories(outputPath);
    if (printResults) {
      boost::filesystem::create_directories(outputPath + "/corners");
    }
  }

  cv::Size imageSize = getImageSize(inputPath);
  std::vector<std::vector<cv::Point2f>> imagePoints = getImagePoints(inputPath, outputPath + "/corners", printResults);
  std::vector<std::vector<cv::Point3f>> objectPoints = getObjectPoints(imagePoints.size());
  cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
  std::vector<cv::Mat> rvecs = std::vector<cv::Mat>(imagePoints.size());
  std::vector<cv::Mat> tvecs = std::vector<cv::Mat>(imagePoints.size());

  printf("[DEBUG] Computing camera calibration\n");
  cv::calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs);

  printf("[DEBUG] Computing RMSE\n");
  double rmse = getRMSE(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvecs, tvecs);

  printf("[DEBUG] Storing results\n");
  storeResults(outputPath, cameraMatrix, distCoeffs, rmse);
}

cv::Size CameraCalib::getImageSize(std::string inputPath) {
  printf("[DEBUG] Computing image size\n");
  bool sizeHasBeenSet = false;
  cv::Size imageSize;
  std::vector<std::string> imagesPath = getImagesPath(inputPath);
  for (std::string imagePath : imagesPath) {
    cv::Mat image = cv::imread(imagePath);
    if (!sizeHasBeenSet) {
      imageSize = image.size();
      sizeHasBeenSet = true;
      continue;
    }
    if (imageSize != image.size()) {
      throw std::runtime_error("Inconsistent image sizes");
    }
  }

  return imageSize;
}


std::vector<std::vector<cv::Point2f>> CameraCalib::getImagePoints(std::string inputPath, std::string outputPath, bool printResults) {
  printf("[DEBUG] Computing image points\n");
  std::vector<std::string> imagesPath = getImagesPath(inputPath);

  std::vector<std::vector<cv::Point2f>> allImagePoints;
  for (std::string imagePath : imagesPath) {
    cv::Mat image = cv::imread(imagePath, cv::IMREAD_COLOR);

    std::vector<cv::Point2f> imagePoints;
    bool foundCorners = cv::findChessboardCorners(image, patternSize, imagePoints);
    if (foundCorners) {
      cv::Mat imageGray;
      cvtColor(image, imageGray, cv::COLOR_BGR2GRAY);
      // TODO: try changing these values and see the effect in the RMSE
      cornerSubPix(imageGray, imagePoints, cv::Size(11, 11), cv::Size(-1, -1),
                   cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.0001));
      allImagePoints.push_back(imagePoints);
    }

    if (printResults && foundCorners) {
      std::string outputImageFilename = boost::filesystem::path(imagePath).stem().string() + "_output.png";
      cv::drawChessboardCorners(image, patternSize, imagePoints, foundCorners);
      if (boost::filesystem::is_directory(outputPath)) {
        cv::imwrite(outputPath + "/" + outputImageFilename, image);
      }
      else {
        throw std::runtime_error("Output folder doesn't exist");
      }
    }
  }

  return allImagePoints;
}

std::vector<std::vector<cv::Point3f>> CameraCalib::getObjectPoints(int imagesNumber) {
  printf("[DEBUG] Computing object points\n");
  std::vector<std::vector<cv::Point3f>> allObjectPoints;

  for (int imageNumber = 0; imageNumber < imagesNumber; imageNumber++) {
    std::vector<cv::Point3f> objectPoints;
    for (int i = 0; i < patternSize.height; ++i) {
      for (int j = 0; j < patternSize.width; ++j) {
        objectPoints.push_back(cv::Point3f(j * squareSize, i * squareSize, 0));
      }
    }
    allObjectPoints.push_back(objectPoints);
  }
  return allObjectPoints;
}

std::vector<std::string> CameraCalib::getImagesPath(std::string inputPath) {
  std::vector<std::string> imagesPath;
  if (boost::filesystem::is_directory(inputPath)) {
    for (auto& file : boost::filesystem::recursive_directory_iterator(inputPath)) {
      if (file.path().extension() == ".png") {
        imagesPath.push_back(file.path().string());
      }
    }
  }
  else {
    throw std::runtime_error("Input folder doesn't exist");
  }

  if (imagesPath.empty()) {
    throw new std::runtime_error("No images found in folder");
  }
  return imagesPath;
}

double CameraCalib::getRMSE(std::vector<std::vector<cv::Point3f>> objectPoints, std::vector<std::vector<cv::Point2f>> imagePoints,
              cv::Mat cameraMatrix, cv::Mat distCoeffs, std::vector<cv::Mat> rvecs, std::vector<cv::Mat> tvecs) {
  std::vector<cv::Point2f> imagePointsReprojected;
  int totalPoints = 0;
  double totalError = 0.0f;

  for(int imageNumber = 0; imageNumber < objectPoints.size(); imageNumber++) {
    cv::projectPoints(objectPoints[imageNumber], rvecs[imageNumber], tvecs[imageNumber], cameraMatrix, distCoeffs, imagePointsReprojected);
    double curError = norm(imagePoints[imageNumber], imagePointsReprojected, cv::NORM_L2);
    totalError += curError * curError;
    totalPoints += objectPoints[imageNumber].size();;
  }

  return std::sqrt(totalError / totalPoints);
}

void CameraCalib::storeResults(std::string outputPath, cv::Mat cameraMatrix, cv::Mat distCoeffs, double rmse) {
  boost::property_tree::ptree root;
  root.put("calibration.rmse", rmse);

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      root.put("calibration.camera_matrix." + std::to_string(i) + std::to_string(j), cameraMatrix.at<double>(i, j));
    }
  }

  for (int i = 0; i < 8; i++) {
    root.put("calibration.distortion_coeff." + std::to_string(i), distCoeffs.at<double>(i, 0));
  }

  boost::property_tree::write_json(outputPath + "/results.json", root);
}