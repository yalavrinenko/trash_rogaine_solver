//
// Created by yalavrinenko on 22.05.2021.
//
#include "ocv_image_loader.hpp"
#include <utils/exceptions.hpp>
#include <utils/logger.hpp>
#include <opencv2/opencv.hpp>

cv::Mat trs::ocv_image_loader::load(const std::filesystem::path &img_path) {
  auto img = cv::imread(img_path.generic_string(), cv::IMREAD_COLOR);

  if (img.empty()){
    LOGE << "Fail to load image " << img_path << ". Reason unknown. Check your path";
    throw trs::load_failure("Fail to load image");
  }

  return img;
}
