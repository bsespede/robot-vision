//
// Created by bsespede on 3/15/20.
//

#include "ass1/PointCloud.h"

PointCloud::PointCloud(std::string inputPath, std::string filename, cv::Mat projectionMatrix) : _inputPath(inputPath),
_filename(filename), _projectionMatrix(projectionMatrix) {}

void PointCloud::computePointCloud() {
  cv::Mat rgbImage = cv::imread(_inputPath + "/left/rectified/" + _filename + ".png", cv::IMREAD_ANYCOLOR);
  cvtColor(rgbImage, rgbImage, cv::COLOR_BGRA2RGB);
  cv::Mat disparityMap = cv::imread(_inputPath + "/disparity/" + _filename + ".png", cv::IMREAD_ANYDEPTH);
  disparityMap.convertTo(disparityMap, CV_32F);
  cv::Mat pointCloud;
  cv::reprojectImageTo3D(disparityMap, pointCloud, _projectionMatrix);

  // TODO: compute mask with min/max disparity
  std::string cloudsPath = _inputPath + "/clouds/";
  boost::filesystem::create_directories(cloudsPath);

  int totalValidPixels = 0;
  for (int u = 0; u < disparityMap.cols; u++) {
    for (int v = 0; v < disparityMap.rows; v++) {
      if (disparityMap.at<float>(v, u) > 0.0f &&  disparityMap.at<float>(v, u) < 1000.0f) {
        totalValidPixels++;
      }
    }
  }

  std::string pointCloudPath = cloudsPath + _filename + ".ply";
  std::ofstream pointCloudFile;
  pointCloudFile.open (pointCloudPath);
  pointCloudFile << "ply\n";
  pointCloudFile << "format ascii 1.0\n";
  pointCloudFile << "element vertex " << totalValidPixels << "\n";
  pointCloudFile << "property float x\n";
  pointCloudFile << "property float y\n";
  pointCloudFile << "property float z\n";
  pointCloudFile << "property uchar red\n";
  pointCloudFile << "property uchar green\n";
  pointCloudFile << "property uchar blue\n";
  pointCloudFile << "end_header\n";

  for (int u = 0; u < disparityMap.cols; u++) {
    for (int v = 0; v < disparityMap.rows; v++) {
      if (disparityMap.at<float>(v, u) > 0.0f &&  disparityMap.at<float>(v, u) < 1000.0f){
        cv::Point3f vertex = pointCloud.at<cv::Point3f>(v, u);
        cv::Vec3b color = rgbImage.at<cv::Vec3b>(v, u);
        pointCloudFile << vertex.x << " " << vertex.y << " " << vertex.z << " ";
        pointCloudFile << (int)color(0) << " " << (int)color(1) << " " << (int)color(2) << "\n";
      }
    }
  }
  pointCloudFile.close();
}