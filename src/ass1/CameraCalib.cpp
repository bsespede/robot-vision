//
// Created by bsespede on 3/14/20.
//

#include "ass1/CameraCalib.h"

CameraCalib::CameraCalib(const cv::Size patternSize, const float squareSize) : patternSize(patternSize), squareSize(squareSize) {}

void CameraCalib::computeIntrinsics(const std::string inputPath, const bool printResults) {
  printf("[DEBUG] Computing input and object points\n");
  std::vector<ImagePoints> imagePoints;
  std::vector<ObjectPoints> objectPoints;
  computeImagePoints(inputPath, imagePoints);
  computeObjectPoints(objectPoints, imagePoints);
  cv::Size imageSize = getImagesSize(inputPath);
  cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
  std::vector<cv::Mat> rvecs = std::vector<cv::Mat>(imagePoints.size());
  std::vector<cv::Mat> tvecs = std::vector<cv::Mat>(imagePoints.size());

  printf("[DEBUG] Computing camera calibration\n");
  std::vector<std::vector<cv::Point2f>> imagePointsUnraveled;
  std::vector<std::vector<cv::Point3f>> objectPointsUnraveled;
  for (int imageNumber = 0; imageNumber < imagePoints.size(); imageNumber++) {
    imagePointsUnraveled.push_back(imagePoints[imageNumber].points);
    objectPointsUnraveled.push_back(objectPoints[imageNumber].points);
  }
  cv::calibrateCamera(objectPointsUnraveled, imagePointsUnraveled, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs);

  printf("[DEBUG] Computing calibration error\n");
  double rmse = computeError(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvecs, tvecs);

  printf("[DEBUG] Saving results\n");
  CameraCalib::Result result = {cameraMatrix, distCoeffs, rmse};
  saveResults(inputPath, imagePoints, result, printResults);
}

cv::Size CameraCalib::getImagesSize(std::string inputPath) {
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

void CameraCalib::computeImagePoints(std::string inputPath, std::vector<ImagePoints>& imagePoints) {
  printf("[DEBUG] Computing image points\n");
  std::vector<std::string> imagesPath = getImagesPath(inputPath);

  for (std::string imagePath : imagesPath) {
    cv::Mat image = cv::imread(imagePath, cv::IMREAD_COLOR);
    std::vector<cv::Point2f> curImagePoints;
    bool foundCorners = cv::findChessboardCorners(image, patternSize, curImagePoints);
    if (foundCorners) {
      cv::Mat imageGray;
      cvtColor(image, imageGray, cv::COLOR_BGR2GRAY);
      cornerSubPix(imageGray, curImagePoints, cv::Size(11, 11), cv::Size(-1, -1),
                   cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.0001));
      imagePoints.push_back({curImagePoints, boost::filesystem::path(imagePath).stem().string()});
    }
  }
}

void CameraCalib::computeObjectPoints(std::vector<ObjectPoints>& objectPoints, std::vector<ImagePoints>& imagePoints) {
  printf("[DEBUG] Computing object points\n");
  for (int imageNumber = 0; imageNumber < imagePoints.size(); imageNumber++) {
    std::vector<cv::Point3f> curObjectPoints;
    for (int i = 0; i < patternSize.height; ++i) {
      for (int j = 0; j < patternSize.width; ++j) {
        curObjectPoints.push_back(cv::Point3f(j * squareSize, i * squareSize, 0));
      }
    }
    objectPoints.push_back({curObjectPoints});
  }
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
    throw std::runtime_error("No images found in folder");
  }
  return imagesPath;
}

float CameraCalib::computeError(std::vector<ObjectPoints>& objectPoints, std::vector<ImagePoints>& imagePoints,
    cv::Mat& cameraMatrix, cv::Mat& distCoeffs, std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs) {
  std::vector<cv::Point2f> imagePointsReprojected;
  int totalPoints = 0;
  float totalError = 0.0f;
  for(int imageNumber = 0; imageNumber < objectPoints.size(); imageNumber++) {
    cv::projectPoints(objectPoints[imageNumber].points, rvecs[imageNumber], tvecs[imageNumber], cameraMatrix, distCoeffs, imagePointsReprojected);
    float curError = cv::norm(imagePoints[imageNumber].points, imagePointsReprojected, cv::NORM_L2);
    totalError += curError * curError;
    totalPoints += objectPoints[imageNumber].points.size();
  }
  return std::sqrt(totalError / totalPoints);
}

void CameraCalib::saveResults(std::string inputPath, std::vector<ImagePoints>& imagePoints, CameraCalib::Result& result, bool printResults) {
  std::string outputPath = inputPath + "/output";
  std::string cornersFolderPath = outputPath + "/corners";
  std::string undistortedFolderPath = outputPath + "/undistorted";
  if (!boost::filesystem::is_directory(outputPath)) {
    boost::filesystem::create_directories(outputPath);
    boost::filesystem::create_directories(cornersFolderPath);
    boost::filesystem::create_directories(undistortedFolderPath);
  }

  boost::property_tree::ptree imagePointsJson;
  for (ImagePoints imagePoint : imagePoints) {
    imagePointsJson.put("imagePoints." + imagePoint.filename + ".number", (int)imagePoint.points.size());
    for (int pointNumber = 0; pointNumber < imagePoint.points.size(); pointNumber++) {
      imagePointsJson.put("imagePoints." + imagePoint.filename + ".points." + std::to_string(pointNumber) + ".x", imagePoint.points[pointNumber].x);
      imagePointsJson.put("imagePoints." + imagePoint.filename + ".points." + std::to_string(pointNumber) + ".y", imagePoint.points[pointNumber].y);
    }

    if (printResults) {
      std::string filename = imagePoint.filename + ".png";
      cv::Mat patternImage = cv::imread(inputPath + "/" + filename, cv::IMREAD_COLOR);
      cv::Mat patternCorners = patternImage.clone();
      cv::Mat patternUndistorted = patternImage.clone();
      cv::drawChessboardCorners(patternCorners, patternSize, imagePoint.points, true);
      cv::undistort(patternImage, patternUndistorted, result.cameraMatrix, result.distCoeffs);
      cv::imwrite(cornersFolderPath + "/" + filename, patternCorners);
      cv::imwrite(undistortedFolderPath + "/" + filename, patternUndistorted);
    }
  }
  boost::property_tree::write_json(outputPath + "/imagePoints.json", imagePointsJson);

  boost::property_tree::ptree calibJson;
  calibJson.put("calibration.rmse", result.rmse);
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      calibJson.put("calibration.camera_matrix." + std::to_string(i) + std::to_string(j), result.cameraMatrix.at<double>(i, j));
    }
  }
  for (int i = 0; i < 8; i++) {
    calibJson.put("calibration.distortion_coeff." + std::to_string(i), result.distCoeffs.at<double>(i, 0));
  }
  boost::property_tree::write_json(outputPath + "/calibration.json", calibJson);
}