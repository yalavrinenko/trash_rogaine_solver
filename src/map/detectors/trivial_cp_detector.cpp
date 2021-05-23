//
// Created by yalavrinenko on 23.05.2021.
//

#include "trivial_cp_detector.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/dnn/dnn.hpp>

struct trs::general_detector::dnn_core{
  dnn_core(){
  }

  std::string detect(cv::Mat const& frame){
    cv::Mat extended;
    cv::cvtColor(frame, extended, cv::COLOR_GRAY2BGR);
    auto blob = cv::dnn::blobFromImage(extended, 1.0, cv::Size(320, 320));

    return {};
  }

private:
  std::string const detector_model_path = "../../dnn/frozen_east_text_detection.pb";
  //cv::dnn::TextDetectionModel_EAST net_;
};

trs::general_detector::general_detector() {
  core_ = std::make_unique<dnn_core>();
}

trs::general_detector::~general_detector() = default;



std::vector<trs::check_point> trs::general_detector::extract_point(const cv::Mat &filtered) {
  std::vector<cv::Vec3f> circles;

  auto const minDist = 50;
  auto try_add = [&circles, minDist](cv::Point_<float> const& center, float r){
    auto sqr = [](auto x) { return x * x; };
    auto it = std::ranges::find_if(circles, [minDist, center, sqr](cv::Vec3f const &cp){
      return sqr(center.x - cp[0]) + sqr(center.y - cp[1]) < minDist * minDist;
    });
    if (it == std::end(circles)) {
      circles.emplace_back(center.x, center.y, r);
      return true;
    } else
      return false;
  };

  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;

  cv::findContours(filtered, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  for (auto & contour : contours){
    cv::Point_<float> center;
    float radius;

    cv::minEnclosingCircle(contour, center, radius);
    if (10 <= radius && radius <= 25) {
      try_add(center, radius);
    }
  }

  return get_check_point(filtered, circles);
}

cv::Mat trs::general_detector::smooth(cv::Mat const &in, int iteration) {
  cv::Mat smoothed = in.clone();
  cv::Mat_<double> kernel(5, 5);
  kernel << 1, 1, 1, 1, 1,
      1, 1, 1, 1, 1,
      1, 1, 1, 1, 1,
      1, 1, 1, 1, 1,
      1, 1, 1, 1, 1;
  kernel /= 25.0;

  for (auto i = 0; i <  iteration; ++i) {
    cv::filter2D(smoothed, smoothed, 0, kernel);
  }
  return smoothed;
}

std::string trs::general_detector::digit_recognition(const cv::Mat &frame) {
  return core_->detect(frame);
}

std::vector<trs::check_point>
trs::general_detector::get_check_point(const cv::Mat &frame, const std::vector<cv::Vec3f> &circles) {
  std::vector<check_point> cp;

  auto frame_size = 45;

  for (auto &c : circles){
    cv::circle(frame, cv::Point{static_cast<int>(c[0]), static_cast<int>(c[1])}, static_cast<int>(c[2]), cv::Scalar(255),
           3, cv::LINE_AA);
    auto left_up = cv::Point {std::max(0, static_cast<int>(c[0]) - frame_size), std::max(0, static_cast<int>(c[1]) - frame_size)};
    auto right_bot = cv::Point {std::min(frame.cols, left_up.x + 2 * frame_size),
                                std::min(frame.rows, left_up.y + 2 * frame_size)};
    //cv::rectangle(frame, left_up, right_bot, 255, 1);

    auto subframe = frame(cv::Rect(left_up, right_bot));
    auto id_str = digit_recognition(frame);
  }

  std::cout << "Total: " << circles.size() << "\n";

  cv::imshow("checks", frame);

  return cp;
}