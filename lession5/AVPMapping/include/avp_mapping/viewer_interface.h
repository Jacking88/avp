#pragma once
#include "avp_mapping/map.h"

class ViewerInterface {
 public:
  // display map
  virtual void displayAvpMap(const TimedPose &pose, const Map &avp_map, const std::vector<Slot> &cur_slots)= 0;
  // display ipm detection result
  void displayIpmDetection(const std::vector<cv::Point2f> &corners,
                           const std::vector<cv::Point2f> &slots_corners,
                           const cv::Mat &ipm_image) {
    std::cout << "\tipm detect : " << corners.size() << " corners, " << slots_corners.size() / 4 << " slots" << std::endl;
    if (imshow_) {
      cv::imshow("ipm_detection_result", drawIPMImg(corners, slots_corners, ipm_image));
      int key = cv::waitKey(delay_);
      if ('r' == key) {
        delay_ = 1;  // set auto run
      }
      if ('s' == key) {
        delay_ = 0;  // run image by image
      }
    }
  }

 private:
  cv::Mat drawIPMImg(const std::vector<cv::Point2f> &corners,
                     const std::vector<cv::Point2f> &slots_corners,
                     const cv::Mat &ipm_image){
    auto ret = ipm_image.clone();
    for (const auto &center : corners) {
      cv::circle(ret, center, 50, 255, 2);
    }
    for (int i = 0; i < slots_corners.size(); i += 4) {
      cv::line(ret, slots_corners[i], slots_corners[i + 1], 0, 2);
      cv::line(ret, slots_corners[i + 2], slots_corners[i + 1], 0, 2);
      cv::line(ret, slots_corners[i + 2], slots_corners[i + 3], 0, 2);
      cv::line(ret, slots_corners[i], slots_corners[i + 3], 0, 2);
    }
    return ret;
  }

 protected:
  const bool imshow_; // do cv imshow
  int delay_;
  ViewerInterface(bool imshow, bool autorun) : imshow_(imshow), delay_(0) {
    if (imshow_) {
      cv::namedWindow("ipm_detection_result", cv::WINDOW_NORMAL);
    }
    if (autorun){
      delay_ = 1;
    }
  }
  ~ViewerInterface() {
    cv::destroyAllWindows();
  }
};