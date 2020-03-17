//
// Created by bsespede on 3/14/20.
//

#include <string>
#include <ass1/StereoCalib.h>

int main(int argc, char *argv[])
{
  StereoCalib calibration(cv::Size(8, 5), 30);
  calibration.computeExtrinsics("../data/CALIB_DATA", true);

  return 0;
}

