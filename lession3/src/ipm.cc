#include "ipm.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

MEICamera::MEICamera(const std::string& intrinsicFile,
                     const std::string& extrinsicFile) {
  K_ = cv::Mat::eye(3, 3, CV_64F);
  D_ = cv::Mat::zeros(4, 1, CV_64F);

  {  // load intrinsic
    cv::FileStorage fs(intrinsicFile, cv::FileStorage::READ);
    if (!fs.isOpened()) {
      std::cerr << "Fail to open intrinsic file: " << intrinsicFile
                << std::endl;
      return;
    }
    cv::FileNode m_param = fs["mirror_parameters"];
    cv::FileNode D_param = fs["distortion_parameters"];
    cv::FileNode K_param = fs["projection_parameters"];
    if (m_param.empty() || D_param.empty() || K_param.empty()) {
      std::cerr << "Error intrinsic file: " << intrinsicFile << std::endl;
      return;
    }
    fs["image_width"] >> width_;
    fs["image_height"] >> height_;
    m_param["xi"] >> xi_;
    K_param["gamma1"] >> K_.at<double>(0, 0);
    K_param["gamma2"] >> K_.at<double>(1, 1);
    K_param["u0"] >> K_.at<double>(0, 2);
    K_param["v0"] >> K_.at<double>(1, 2);
    D_param["k1"] >> D_.at<double>(0, 0);
    D_param["k2"] >> D_.at<double>(1, 0);
    D_param["p1"] >> D_.at<double>(2, 0);
    D_param["p2"] >> D_.at<double>(3, 0);
  }

  {  // load extrinsic
    cv::FileStorage fs(extrinsicFile, cv::FileStorage::READ);
    if (!fs.isOpened()) {
      std::cerr << "Fail to open extrinsic file: " << extrinsicFile
                << std::endl;
      return;
    }
    cv::FileNode trans_param = fs["transform"]["translation"];
    cv::FileNode rot_param = fs["transform"]["rotation"];
    if (trans_param.empty() || rot_param.empty()) {
      std::cerr << "Error extrinsic file: " << extrinsicFile << std::endl;
      return;
    }
    fs["frame_id"] >> id_;

    // T_vehicle_cam_ represents the transformation from the camera coordinate system to the vehicle coordinate system,
    // the origin of the vehicle coordinate system is the center of the vehicle.
    trans_param["x"] >> T_vehicle_cam_.translation().x();
    trans_param["y"] >> T_vehicle_cam_.translation().y();
    trans_param["z"] >> T_vehicle_cam_.translation().z();
    Eigen::Quaterniond q;
    rot_param["x"] >> q.x();
    rot_param["y"] >> q.y();
    rot_param["z"] >> q.z();
    rot_param["w"] >> q.w();
    T_vehicle_cam_.linear() = q.toRotationMatrix();
  }

  is_valid_ = true;
}

void MEICamera::DebugString() const {
  std::cout << id_ << " : " << is_valid_ << std::endl;
  std::cout << "height: " << height_ << std::endl;
  std::cout << "width: " << width_ << std::endl;
  std::cout << "xi: " << xi_ << std::endl;
  std::cout << "K : " << K_ << std::endl;
  std::cout << "D : " << D_ << std::endl;
  std::cout << "cam_extrinsic : " << T_vehicle_cam_.matrix() << std::endl;
}

IPM::IPM() {}

void IPM::AddCamera(const std::string& intrinsicsFile,
                    const std::string& extrinsicFile) {
  cameras_.emplace_back(intrinsicsFile, extrinsicFile);
  std::cout << "---------ipm add new camera ---------" << std::endl;
  cameras_.back().DebugString();
}

cv::Mat IPM::GenerateIPMImage(const std::vector<cv::Mat>& images) const {
  // Initialize a black IPM image with dimensions ipm_img_h_ x ipm_img_w_ and 3 channels (RGB)
  cv::Mat ipm_image = cv::Mat::zeros(ipm_img_h_, ipm_img_w_, CV_8UC3);

  // Check if the number of input images matches the number of cameras
  if (images.size() != cameras_.size()) {
    // If not, print an error message and return the initialized black IPM image
    std::cout << "IPM not init normaly !" << std::endl;
    return ipm_image;
  }
  
  // Iterate over each pixel in the IPM image
  for (int u = 0; u < ipm_img_w_; ++u) {
    for (int v = 0; v < ipm_img_h_; ++v) {
      // Calculate the point p_v in vehicle coordinates, p_v is corresponding to the current pixel (u, v).
      // Assume the height of the ipm_image in vehicle coordinate is 0.
      Eigen::Vector3d p_v(-(0.5 * ipm_img_h_ - u) * pixel_scale_,
                          (0.5 * ipm_img_w_ - v) * pixel_scale_, 0);

      // Iterate over each camera
      for (size_t i = 0; i < cameras_.size(); ++i) {
        // Project the vehile point p_v into the image plane uv
        ////////////////////////TODO begin///////////////////////
        // // pinhole camera model
        // Eigen::Vector3d p_c = cameras_[i].T_vehicle_cam_.inverse() * p_v;

        // double x = p_c.x() / p_c.z();
        // double y = p_c.y() / p_c.z();

        // double r2 = x * x + y * y;
        // double k1 = cameras_[i].D_.at<double>(0, 0);
        // double k2 = cameras_[i].D_.at<double>(1, 0);
        // double p1 = cameras_[i].D_.at<double>(2, 0);
        // double p2 = cameras_[i].D_.at<double>(3, 0);
        // double dist_factor = 1.0 + k1 * r2 + k2 * r2 * r2;
        // double dx = 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
        // double dy = p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;

        // x = x * dist_factor + dx;
        // y = y * dist_factor + dy;

        // int uv0 = static_cast<int>(cameras_[i].K_.at<double>(0, 0) * x + cameras_[i].K_.at<double>(0, 2));
        // int uv1 = static_cast<int>(cameras_[i].K_.at<double>(1, 1) * y + cameras_[i].K_.at<double>(1, 2));

        // MEI camera model
        Eigen::Vector3d p_c = cameras_[i].T_vehicle_cam_.inverse() * p_v;

        double x = p_c.x();
        double y = p_c.y();
        double z = p_c.z();
        if(z<=1e-6) continue;

        double p = std::sqrt(x * x + y * y + z * z); 
        double x1 = x/p;
        double y1 = y/p;
        double z1 = z/p;

        double z2 = z1 + cameras_[i].xi_;
        double x3 = x1/z2;
        double y3 = y1/z2;

        double r2 = x3 * x3 + y3 * y3;
        double k1 = cameras_[i].D_.at<double>(0, 0);
        double k2 = cameras_[i].D_.at<double>(1, 0);
        double p1 = cameras_[i].D_.at<double>(2, 0);
        double p2 = cameras_[i].D_.at<double>(3, 0);
        double dist_factor = 1.0 + k1 * r2 + k2 * r2 * r2;
        double dx = 2 * p1 * x3 * y3 + p2 * (r2 + 2 * x3 * x3);
        double dy = p1 * (r2 + 2 * y3 * y3) + 2 * p2 * x3 * y3;

        // Apply distortion correction
        x = x3 * dist_factor + dx;
        y = y3 * dist_factor + dy;

        // Step 4: Convert normalized coordinates to pixel coordinates using the camera's intrinsic matrix
        int uv0 = static_cast<int>(cameras_[i].K_.at<double>(0, 0) * x + cameras_[i].K_.at<double>(0, 2));
        int uv1 = static_cast<int>(cameras_[i].K_.at<double>(1, 1) * y + cameras_[i].K_.at<double>(1, 2));

        ////////////////////////TODO end/////////////////////
        // (uv0, uv1) is the projected pixel from p_v to cameras_[i]
        // Skip this point if the projected coordinates are out of bounds of the camera image
        if (uv0 < 0 || uv0 >= cameras_[i].width_ || uv1 < 0 ||
            uv1 >= cameras_[i].height_) {
          continue;
        }

        // Get the pixel color from the camera image and set it to the IPM image
        // If the IPM image pixel is still black (not yet filled), directly assign the color
        if (ipm_image.at<cv::Vec3b>(v, u) == cv::Vec3b(0, 0, 0)) {
          ipm_image.at<cv::Vec3b>(v, u) = images[i].at<cv::Vec3b>(uv1, uv0);
        } else {
          // Otherwise, average the existing color with the new color
          ipm_image.at<cv::Vec3b>(v, u) = (ipm_image.at<cv::Vec3b>(v, u) +
                                           images[i].at<cv::Vec3b>(uv1, uv0)) /
                                          2;
        }
      }
    }
  }

  // Return the generated IPM image
  return ipm_image;
}

cv::Mat IPM::GenerateIPMImage_process(const std::vector<cv::Mat> &images) const
{
  // Initialize a black IPM image with the dimensions ipm_img_h_ x ipm_img_w_ and 3 channels (RGB)
  cv::Mat ipm_image = cv::Mat::zeros(ipm_img_h_, ipm_img_w_, CV_8UC3);

  // Check if the number of input images matches the number of cameras
  if (images.size() != cameras_.size()) {
    std::cout << "IPM not initialized properly!" << std::endl;
    return ipm_image;
  }

  // Iterate over each pixel in the IPM image
  for (int u = 0; u < ipm_img_w_; ++u) {
    for (int v = 0; v < ipm_img_h_; ++v) {
      // Calculate the point p_v in vehicle coordinates, p_v corresponds to the current pixel (u, v)
      Eigen::Vector3d p_v(-(0.5 * ipm_img_h_ - u) * pixel_scale_,
                          (0.5 * ipm_img_w_ - v) * pixel_scale_, 0);

      // Iterate over each camera
      for (size_t i = 0; i < cameras_.size(); ++i) {
        // Project the vehicle point p_v into the camera coordinate system
        Eigen::Vector3d p_c = cameras_[i].T_vehicle_cam_.inverse() * p_v;

        // Normalize the coordinates
        double x = p_c.x();
        double y = p_c.y();
        double z = p_c.z();
        if(z<=1e-6) continue;

        // Calculate the angle of the point in the camera's field of view (FOV)
        double angle = std::atan2(std::sqrt(x * x + y * y), z);  // angle from camera's optical axis

        // Use the camera's pixel directly (if within FOV)
        double p = std::sqrt(x * x + y * y + z * z); 
        double x1 = x / p;
        double y1 = y / p;
        double z1 = z / p;

        // MEI model: Add xi (a constant) and project to the plane at z = 1
        double z2 = z1 + cameras_[i].xi_;
        double x3 = x1 / z2;  // Normalized coordinates
        double y3 = y1 / z2;

        // Apply distortion model
        double r2 = x3 * x3 + y3 * y3;
        double k1 = cameras_[i].D_.at<double>(0, 0);
        double k2 = cameras_[i].D_.at<double>(1, 0);
        double p1 = cameras_[i].D_.at<double>(2, 0);
        double p2 = cameras_[i].D_.at<double>(3, 0);
        
        double radial = 1.0 + k1 * r2 + k2 * r2 * r2;
        double x_tan = 2.0 * p1 * x3 * y3 + p2 * (r2 + 2.0 * x3 * x3);
        double y_tan = p1 * (r2 + 2.0 * y3 * y3) + 2.0 * p2 * x3 * y3;

        // Distorted coordinates
        double x_distorted = x3 * radial + x_tan;
        double y_distorted = y3 * radial + y_tan;

        // Convert to pixel coordinates using the camera's intrinsic matrix
        int uv0 = static_cast<int>(cameras_[i].K_.at<double>(0, 0) * x_distorted + cameras_[i].K_.at<double>(0, 2));
        int uv1 = static_cast<int>(cameras_[i].K_.at<double>(1, 1) * y_distorted + cameras_[i].K_.at<double>(1, 2));
        
        if (uv0 < 0 || uv0 >= cameras_[i].width_ || uv1 < 0 ||
          uv1 >= cameras_[i].height_) {
        continue;
      }
        // Check if the point is within the 90° FOV of the camera (half of 90° = 45°)
        if (angle <= M_PI / 4) {
          // Skip if the pixel is out of bounds
          // if (uv0 >= 0 && uv0 < cameras_[i].width_ && uv1 >= 0 && uv1 < cameras_[i].height_) {
            // If pixel is in the field of view, directly use the camera pixel
            if (ipm_image.at<cv::Vec3b>(v, u) == cv::Vec3b(0, 0, 0)) {
              ipm_image.at<cv::Vec3b>(v, u) = images[i].at<cv::Vec3b>(uv1, uv0);
            }
          // }
        } else {
          // For points outside the camera's 90° FOV, use weighted averaging
          if (ipm_image.at<cv::Vec3b>(v, u) == cv::Vec3b(0, 0, 0)) {
            ipm_image.at<cv::Vec3b>(v, u) = images[i].at<cv::Vec3b>(uv1, uv0);
          } else {
            // Simple averaging, you can apply weights based on distance or other factors if needed
            ipm_image.at<cv::Vec3b>(v, u) = (ipm_image.at<cv::Vec3b>(v, u) + images[i].at<cv::Vec3b>(uv1, uv0)) / 2;
          }
        }
      }
    }
  }

  // Return the generated IPM image
  return ipm_image;
}
