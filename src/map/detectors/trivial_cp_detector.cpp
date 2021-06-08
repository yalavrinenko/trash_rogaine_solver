//
// Created by yalavrinenko on 23.05.2021.
//

#include "trivial_cp_detector.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/dnn/dnn.hpp>
#include <fstream>
#include <iterator>
#include <span>

namespace {
  using namespace cv;
  void fourPointsTransform(const Mat& frame, const Point2f vertices[], Mat& result)  {
    const Size outputSize = Size(100, 32);

    Point2f targetVertices[4] = {
        Point(0, outputSize.height - 1),
        Point(0, 0), Point(outputSize.width - 1, 0),
        Point(outputSize.width - 1, outputSize.height - 1)
    };
    Mat rotationMatrix = getPerspectiveTransform(vertices, targetVertices);

    warpPerspective(frame, result, rotationMatrix, outputSize);
  }
}

struct trs::general_detector::dnn_core{
  dnn_core():
    detection_net_{detector_model_path.generic_string()},
    recognition_net_{recognition_model.generic_string()} {
    float confThreshold = 0.5;
    float nmsThreshold = 0.4;
    detection_net_.setConfidenceThreshold(confThreshold).setNMSThreshold(nmsThreshold);
    double detScale = 1.0;
    cv::Size detInputSize = cv::Size(320, 320);

    cv::Scalar detMean = cv::Scalar(50, 50, 50);
    bool swapRB = false;
    detection_net_.setInputParams(detScale, detInputSize, detMean, swapRB);

    //recognition

    recognition_net_.setDecodeType("CTC-greedy");

    std::ifstream vocFile;
    vocFile.open(text_alphabet);
    std::vector<std::string> vocabulary(std::istream_iterator<std::string>{vocFile}, std::istream_iterator<std::string>{});
    //std::ranges::transform(vocabulary, vocabulary.begin(), [](auto const &c) { return (std::isdigit(c[0])) ? c : "."; });
    recognition_net_.setVocabulary(vocabulary);

    double scale = 1.0 / 127.5;
    cv::Scalar mean = cv::Scalar(127.5, 127.5, 127.5);

    cv::Size inputSize = cv::Size(100, 32);
    recognition_net_.setInputParams(scale, inputSize, mean);
  }

  std::string detect(cv::Mat const& frame){
    cv::Mat extended;
    cv::cvtColor(frame, extended, cv::COLOR_GRAY2BGR);
    cv::resize(extended, extended, cv::Size(320, 320));

    std::vector<std::vector<cv::Point>> detections;
    detection_net_.detect(smooth(extended, 2), detections);

    cv::cvtColor(extended, extended, cv::COLOR_BGR2GRAY);

    if (detections.size() > 1 || detections.empty())
      return "";

    auto const &points = detections.front();

    try {
      cv::Mat roi;

      auto vertices = points | std::views::transform([](auto v) { return cv::Point2f(v); }) | std::views::common;

      fourPointsTransform(extended, std::vector(vertices.begin(), vertices.end()).data(), roi);

      auto number = recognition_net_.recognize(roi);

      return std::ranges::all_of(number, [](auto const& c) { return std::isdigit(c); }) ? number : "";
    }
    catch (cv::Exception &e) {
      return "";
    }

    return {};
  }

private:
  std::filesystem::path const detector_model_path = "../../dnn/frozen_east_text_detection.pb";
  std::filesystem::path const recognition_model = "../../dnn/crnn.onnx";
  std::filesystem::path const text_alphabet = "../../dnn/alphabet_36.txt";
  cv::dnn::TextDetectionModel_EAST detection_net_;
  cv::dnn::TextRecognitionModel recognition_net_;
};

trs::general_detector::general_detector() {
  core_ = std::make_unique<dnn_core>();
}

trs::general_detector::~general_detector() = default;



std::unordered_map<size_t, trs::check_point> trs::general_detector::extract_point(const cv::Mat &filtered) {
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
    if (9 <= radius && radius <= 25) {
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

std::unordered_map<size_t, trs::check_point>
trs::general_detector::get_check_point(const cv::Mat &frame, const std::vector<cv::Vec3f> &circles) {
  std::unordered_map<size_t, check_point> cp;

  auto frame_size = 45;

  for (auto &c : circles){
    cv::circle(frame, cv::Point{static_cast<int>(c[0]), static_cast<int>(c[1])}, static_cast<int>(c[2]), cv::Scalar(0),
           4, cv::LINE_AA);

    auto left_up = cv::Point {std::max(0, static_cast<int>(c[0]) - frame_size), std::max(0, static_cast<int>(c[1]) - frame_size)};
    auto right_bot = cv::Point {std::min(frame.cols, left_up.x + 2 * frame_size),
                                std::min(frame.rows, left_up.y + 2 * frame_size)};

    auto subframe = frame(cv::Rect(left_up, right_bot));
    auto id_str = digit_recognition(subframe);
    if (id_str != "")
      cp.emplace(std::stoull(id_str), check_point{
        .position = {static_cast<size_t>(c[0]), static_cast<size_t >(c[1])},
        .uid = std::stoull(id_str)
      });
  }
  return cp;
}