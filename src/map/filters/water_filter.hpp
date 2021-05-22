//
// Created by yalavrinenko on 22.05.2021.
//

#ifndef TRASH_ROGAINE_SOLVER_WATER_FILTER_HPP
#define TRASH_ROGAINE_SOLVER_WATER_FILTER_HPP

namespace cv{
  class Mat;
}

namespace trs {
  class water_filter {
  public:
    static cv::Mat apply(cv::Mat const& src);

  private:
    static cv::Mat& smooth(cv::Mat &in);
  };
}


#endif //TRASH_ROGAINE_SOLVER_WATER_FILTER_HPP
