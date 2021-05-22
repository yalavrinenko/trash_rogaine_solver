//
// Created by yalavrinenko on 22.05.2021.
//

#ifndef TRASH_ROGAINE_SOLVER_OCV_IMAGE_LOADER_HPP
#define TRASH_ROGAINE_SOLVER_OCV_IMAGE_LOADER_HPP

#include <filesystem>

namespace cv{
  class Mat;
}
namespace trs {
  class ocv_image_loader {
  public:
    static cv::Mat load(std::filesystem::path const& img_path);
  private:
  };
}


#endif //TRASH_ROGAINE_SOLVER_OCV_IMAGE_LOADER_HPP
