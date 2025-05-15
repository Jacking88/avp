#include "avp_mapping/slot.h"

namespace {
double computeCenterDistance(cv::Vec4i line1, cv::Vec4i line2) {
  cv::Point2f p1 ((line1[0] + line1[2]) / 2.0f,
                  (line1[1] + line1[3]) / 2.0f);
  cv::Point2f p2 ((line2[0] + line2[2]) / 2.0f,
                  (line2[1] + line2[3]) / 2.0f);
  return cv::norm(p1 - p2);
}

cv::Point2f computeIntersect(cv::Vec4i line1, cv::Vec4i line2) {
  int x1 = line1[0], y1 = line1[1], x2 = line1[2], y2 = line1[3];
  int x3 = line2[0], y3 = line2[1], x4 = line2[2], y4 = line2[3];
  float denom = (float)((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));
  cv::Point2f intersect( (x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4),
                         (x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4));
  return intersect / denom;
}
} // end of namespace

// 保留边角、细小区域 和骨架
cv::Mat skeletonize(const cv::Mat &img) {
  cv::Mat skel(img.size(), CV_8UC1, cv::Scalar(0));
  cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
  cv::Mat temp, eroded;
  do {
    cv::erode(img, eroded, element);
    cv::dilate(eroded, temp, element);
    cv::subtract(img, temp, temp);  // 细小区域
    cv::bitwise_or(skel, temp, skel);
    eroded.copyTo(img);
  } while (0 != cv::countNonZero(img));
  return skel;
}

void removeIsolatedPixels(cv::Mat &src, int min_neighbors) {
  CV_Assert(src.type() == CV_8UC1);  // 确保单通道灰度图
  cv::Mat dst = src.clone();
  for (int y = 1; y < src.rows - 1; ++y) {
    for (int x = 1; x < src.cols - 1; ++x) {
      if (src.at<uchar>(y, x) > 0) {  // 如果是前景图像
        int neighbor_count = 0;       // 相邻前景像素的计数器
        // 检查8邻域内的像素
        for (int ny = -1; ny <= 1; ++ny) {
          for (int nx = -1; nx <= 1; ++nx) {
            if (ny == 0 && nx == 0) continue;  // 跳过自己
            if (src.at<uchar>(y + ny, x + nx) > 0) {
              ++neighbor_count;  // 增加邻居数
            }
          }
        }
        // 如果相邻前景像素小于阈值，则认为该点是孤立的，并将其去除
        if (neighbor_count < min_neighbors) {
          dst.at<uchar>(y, x) = 0;
        }
      }
    }
  }
  src = dst;
}

bool isSlotShortLine(const cv::Point2f &point1,
                     const cv::Point2f &point2,
                     const cv::Mat &image) {
  cv::LineIterator it(image, point1, point2, 4);
  int positiveIndex = 0;
  const double len = cv::norm(point1 - point2);
  int delta = 10;
  if (std::fabs(len - kShortLinePixelDistance) < delta) {
    for (int i = 0; i < it.count; ++i, ++it) {
      int color = image.at<uchar>(std::round(it.pos().y), std::round(it.pos().x));
      if (color > 0) {
        positiveIndex++;
      }
    }
    if (positiveIndex > kShortLineDetectPixelNumberMin) {
      return true;
    }
  }
  return false;
}

bool isSlotLongLine(const cv::Mat &line_img,
                    const cv::Point2f &start,
                    const cv::Point2f &dir) {
  int cnt{0};
  cv::Point2f pt(start);
  for (int l = 1; l < kLongLinePixelDistance; ++l) {
    pt += dir;
    if (pt.y <= 0 || pt.y >= kIPMImgHeight ||
        pt.x <= 0 || pt.x >= kIPMImgWidth) {
      continue;
    }
    if (line_img.at<uchar>(pt) > 0) {
      ++cnt;
    }
  }
  return cnt > kLongLineDetectPixelNumberMin;
}

std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point2f>> detectSlot(
    const cv::Mat &slot_img, const std::vector<cv::Vec4i> &lines) {
  /////////////// TODO 1 detect corners from lines ///////////////
  std::vector<cv::Point2f> corners; // corners detected from lines
 // 设定参数
 constexpr double distance_threshold = 20.0;  // 交点去重阈值（像素）
 constexpr double min_angle = 85.0;  // 交点最小角度
 constexpr double max_angle = 95.0;  // 交点最大角度

 // 遍历所有线段，计算交点
 for (size_t i = 0; i < lines.size(); ++i) {
   for (size_t j = i + 1; j < lines.size(); ++j) {
     // 提取两条线段的端点
     cv::Point2f p1(lines[i][0], lines[i][1]), p2(lines[i][2], lines[i][3]);
     cv::Point2f p3(lines[j][0], lines[j][1]), p4(lines[j][2], lines[j][3]);

     // 计算交点
     float denom = (p1.x - p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x - p4.x);
     if (std::abs(denom) < 1e-6) continue; // 避免平行线或接近平行

     cv::Point2f intersection;
     intersection.x = ((p1.x * p2.y - p1.y * p2.x) * (p3.x - p4.x) - (p1.x - p2.x) * (p3.x * p4.y - p3.y * p4.x)) / denom;
     intersection.y = ((p1.x * p2.y - p1.y * p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x * p4.y - p3.y * p4.x)) / denom;

     // 确保交点在图像范围内
     if (intersection.x < 0 || intersection.x >= slot_img.cols || intersection.y < 0 || intersection.y >= slot_img.rows)
       continue;

     // 计算交点处的夹角
     cv::Point2f dir1 = p2 - p1, dir2 = p4 - p3;
     double dot = dir1.x * dir2.x + dir1.y * dir2.y;
     double norm1 = std::sqrt(dir1.x * dir1.x + dir1.y * dir1.y);
     double norm2 = std::sqrt(dir2.x * dir2.x + dir2.y * dir2.y);
     double angle = std::acos(dot / (norm1 * norm2)) * 180.0 / CV_PI; // 转换为角度

     // 过滤掉非直角交点
     if (angle < min_angle || angle > max_angle)
       continue;

     // 检查交点是否属于车位标记区域
     if (slot_img.at<uchar>(cv::Point(intersection.x, intersection.y)) < 128)
       continue; // 不是有效车位标记，跳过

     // 交点去重（合并过近角点）
     bool too_close = false;
     for (const auto &existing_corner : corners) {
       if (cv::norm(existing_corner - intersection) < distance_threshold) {
         too_close = true;
         break;
       }
     }
     if (!too_close)
       corners.push_back(intersection);
   }
 }

  /////////////// TODO 2 detect slots in slot_img ///////////////
  std::vector<cv::Point2f> slot_points; // Every 4 consecutive points compose a slot
  
  // 设定参数
  constexpr double short_side_m = 4.2; // 车位短边（米）
  constexpr double long_side_m = 6.4; // 车位长边（米）
  constexpr double pixel_scale = 50.0; // 1m ≈ 50 像素
  constexpr double short_side_px = short_side_m * pixel_scale; // 车位短边 ≈ 210 像素
  constexpr double long_side_px = long_side_m * pixel_scale; // 车位长边 ≈ 320 像素
  constexpr double min_visible_long_edge_ratio = 0.3;  // 至少 30% 长边像素可见

  constexpr double line_density_threshold = 0.6; // 线上的库位线点比例阈值

  // 遍历所有角点，两两配对
  for (size_t i = 0; i < corners.size(); ++i) {
    for (size_t j = i + 1; j < corners.size(); ++j) {
      double dist = cv::norm(corners[i] - corners[j]);

      // 1. 检查两点距离是否接近 4.2m（210 像素）
      if (std::abs(dist - short_side_px) > distance_threshold)
        continue; // 不满足短边长度要求，跳过

      // 2. 统计短边上的像素点数量，确保大部分是库位线
      int line_pixels = 0, slot_line_pixels = 0;
      int steps = 20; // 采样点数
      for (int k = 0; k <= steps; ++k) {
        cv::Point2f sample_point = corners[i] + (corners[j] - corners[i]) * (k / (double)steps);
        if (slot_img.at<uchar>(sample_point) > 128) slot_line_pixels++; // 计数库位线像素
        line_pixels++;
      }
      if (slot_line_pixels < line_density_threshold * line_pixels)
        continue; // 库位线比例不足，跳过

      // 3. 计算短边对应的垂线方向
      cv::Point2f dir = corners[j] - corners[i]; // 计算方向向量
      cv::Point2f normal(-dir.y, dir.x); // 旋转 90° 计算垂线方向
      // cv::normalize(normal, normal); // 归一化

      // 计算单位向量
      double length = std::sqrt(normal.x * normal.x + normal.y * normal.y);
      if (length > 1e-6) { // 避免除以 0
        normal.x /= length;
        normal.y /= length;
      }

      // 4. 计算两条可能的长边（正方向 & 反方向）
      cv::Point2f candidate1_a = corners[i] + normal * long_side_px;
      cv::Point2f candidate1_b = corners[j] + normal * long_side_px;
      cv::Point2f candidate2_a = corners[i] - normal * long_side_px;
      cv::Point2f candidate2_b = corners[j] - normal * long_side_px;

      // 5. 统计垂线上是否有足够多的库位线像素
      // auto check_perpendicular_line = [&](const cv::Point2f &p1, const cv::Point2f &p2) {
      //   int line_pixels = 0, slot_line_pixels = 0;
      //   for (int k = 0; k <= steps; ++k) {
      //     cv::Point2f sample_point = p1 + (p2 - p1) * (k / (double)steps);
      //     if (sample_point.x >= 0 && sample_point.x < slot_img.cols &&
      //         sample_point.y >= 0 && sample_point.y < slot_img.rows) {
      //       if (slot_img.at<uchar>(sample_point) > 128) slot_line_pixels++;
      //     }
      //     line_pixels++;
      //   }
      //   return slot_line_pixels >= line_density_threshold * line_pixels;
      // };

      auto check_perpendicular_line = [&](const cv::Point2f &p1, const cv::Point2f &p2) {
        int line_pixels = 0, slot_line_pixels = 0;
        int visible_pixels = 0;  
        for (int k = 0; k <= steps; ++k) {
          cv::Point2f sample_point = p1 + (p2 - p1) * (k / (double)steps);
          if (sample_point.x >= 0 && sample_point.x < slot_img.cols &&
              sample_point.y >= 0 && sample_point.y < slot_img.rows) {
            visible_pixels++;
            if (slot_img.at<uchar>(sample_point) > 128) slot_line_pixels++;
          }
          line_pixels++;
        }
        return visible_pixels >= min_visible_long_edge_ratio * steps && 
               slot_line_pixels >= line_density_threshold * visible_pixels;
      };

      bool valid1 = check_perpendicular_line(corners[i], candidate1_a) &&
                    check_perpendicular_line(corners[j], candidate1_b);
      bool valid2 = check_perpendicular_line(corners[i], candidate2_a) &&
                    check_perpendicular_line(corners[j], candidate2_b);

      // 6. 选择符合条件的长边推测车位四个角
      if (valid1) {
        slot_points.push_back(corners[i]);
        slot_points.push_back(corners[j]);
        slot_points.push_back(candidate1_b);
        slot_points.push_back(candidate1_a);
      } else if (valid2) {
        slot_points.push_back(corners[i]);
        slot_points.push_back(corners[j]);
        slot_points.push_back(candidate2_b);
        slot_points.push_back(candidate2_a);
      }
    }
  }

  return std::make_tuple(corners, slot_points);
}
