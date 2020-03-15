//
// Created by bsespede on 3/14/20.
//

#include <string>
#include <ass1/CameraCalib.h>

int main(int argc, char *argv[])
{
  CameraCalib calibration(cv::Size(8, 5), 30);
  calibration.computeIntrinsics("../data/CALIB_DATA/left", true);

  return 0;
}

