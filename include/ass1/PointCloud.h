//
// Created by bsespede on 3/15/20.
//

#ifndef ROBOT_VISION_SRC_ASS1_POINTCLOUD_H_
#define ROBOT_VISION_SRC_ASS1_POINTCLOUD_H_

#include "ass1/CameraCalib.h"
#include <iostream>
#include <fstream>

class PointCloud {
 public:
  PointCloud(std::string inputPath, std::string filename, cv::Mat projectionMatrix);
  void computePointCloud();

 private:
  std::string _inputPath;
  std::string _filename;
  cv::Mat _projectionMatrix;
};

#endif //ROBOT_VISION_SRC_ASS1_POINTCLOUD_H_
