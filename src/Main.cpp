//
// Created by bsespede on 3/14/20.
//

#include <string>
#include <ass1/CameraCalib.h>

int main(int argc, char *argv[])
{
  cv::Size boardSize(8, 5);
  float squareLength = 30;
  CameraCalib calibration(boardSize, squareLength);
  calibration.computeIntrinsics("../data/CALIB_DATA/left", "../data/CALIB_DATA/left/output", true);

  return 0;
}

