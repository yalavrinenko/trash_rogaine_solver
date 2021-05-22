//
// Created by yalavrinenko on 23.05.2021.
//

#ifndef TRASH_ROGAINE_SOLVER_TARGET_FILTER_HPP
#define TRASH_ROGAINE_SOLVER_TARGET_FILTER_HPP

namespace cv{
  class Mat;
}

namespace trs {
  class target_filter {
  public:
    static cv::Mat apply(cv::Mat const &src);
  };
}


#endif //TRASH_ROGAINE_SOLVER_TARGET_FILTER_HPP
