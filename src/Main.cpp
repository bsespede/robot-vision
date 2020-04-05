//
// Created by bsespede on 3/14/20.
//

#include <string>
#include <ass1/StereoCalib.h>
#include <ass1/StereoMatch.h>

int main(int argc, char *argv[])
{
  std::string calibImagesPath = "../data/CALIB_DATA";
  StereoCalib calibration(calibImagesPath, cv::Size(8, 5), 30);
  calibration.computeCalibration();

  std::string sceneImagesPath = "../data/STEREO_DATA_ADD/stereo_data_add";
  StereoMatch stereoMatcher(sceneImagesPath, calibration);
  stereoMatcher.computeRectification();
  stereoMatcher.computeDepth(true, true);

  return 0;
}

