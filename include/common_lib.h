/* 
Developer: Chunran Zheng <zhengcr@connect.hku.hk>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef COMMON_LIB_H
#define COMMON_LIB_H
#define PCL_NO_PRECOMPILE

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <vector>
#include <string>
#include <algorithm>
#include <cassert>
#include "toml.hpp"

#define LOG_INFO(msg) std::cout << BOLDCYAN << "[INFO] " << msg << RESET << std::endl
#define LOG_WARN(msg) std::cout << BOLDYELLOW << "[WARN] " << msg << RESET << std::endl
#define LOG_ERROR(msg) std::cerr << BOLDRED << "[ERROR] " << msg << RESET << std::endl
#define LOG_SUCCESS(msg) std::cout << BOLDGREEN << "[SUCCESS] " << msg << RESET << std::endl

// 控制台颜色宏定义（替代原来的color.h）
#define RESET       "\033[0m"
#define BOLD        "\033[1m"
#define RED         "\033[31m"
#define GREEN       "\033[32m"
#define YELLOW      "\033[33m"
#define BLUE        "\033[34m"
#define MAGENTA     "\033[35m"
#define CYAN        "\033[36m"
#define WHITE       "\033[37m"
#define BOLDRED     "\033[1m\033[31m"
#define BOLDGREEN   "\033[1m\033[32m"
#define BOLDYELLOW  "\033[1m\033[33m"
#define BOLDBLUE    "\033[1m\033[34m"
#define BOLDMAGENTA "\033[1m\033[35m"
#define BOLDCYAN    "\033[1m\033[36m"
#define BOLDWHITE   "\033[1m\033[37m"

using namespace std;
using namespace cv;
using namespace pcl;

#define TARGET_NUM_CIRCLES 4
#define DEBUG 1
#define GEOMETRY_TOLERANCE 0.06

// ===== 自定义点类型：XYZ + ring =====
namespace Common 
{
  struct Point
  {
    PCL_ADD_POINT4D;            // quad-word XYZ + padding
    std::uint16_t ring = 0;     // 线号（机械雷达/多线雷达）
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  } EIGEN_ALIGN16;
}
POINT_CLOUD_REGISTER_POINT_STRUCT(Common::Point,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (std::uint16_t, ring, ring)
);

// 参数结构体
struct Params {
  double x_min, x_max, y_min, y_max, z_min, z_max;
  double fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, k5, k6;
  double marker_size, delta_width_qr_center, delta_height_qr_center;
  double delta_width_circles, delta_height_circles, circle_radius;
  int min_detected_markers;
  string image_path;
  string pointcloud_path;  
  string output_path;
  bool use_mech_lidar;
};

// 从配置文件读取参数（替代ROS参数服务器）
Params loadParameters(const string& config_file) {
  Params params;
  
  // 默认参数值
  params.fx = 1215.31801774424;
  params.fy = 1214.72961288138;
  params.cx = 1047.86571859677;
  params.cy = 745.068353101898;
  params.k1 = -0.33574781188503;
  params.k2 = 0.10996870793601;
  params.p1 = 0.000157303079833973;
  params.p2 = 0.000544930726278493;
  params.marker_size = 0.2;
  params.delta_width_qr_center = 0.55;
  params.delta_height_qr_center = 0.35;
  params.delta_width_circles = 0.5;
  params.delta_height_circles = 0.4;
  params.min_detected_markers = 3;
  params.circle_radius = 0.12;
  params.image_path = "./data/image.png";
  params.pointcloud_path = "./data/input.pcd";  // 改为PCD文件
  params.output_path = "./output";
  params.x_min = 1.5;
  params.x_max = 3.0;
  params.y_min = -1.5;
  params.y_max = 2.0;
  params.z_min = -0.5;
  params.z_max = 2.0;
  params.use_mech_lidar = true;
  
  // 如果提供了配置文件，则从文件读取
  ifstream config(config_file);
  if (config.is_open()) {
    string line;
    while (getline(config, line)) {
      // 跳过注释行和空行
      if (line.empty() || line[0] == '#') continue;
      
      size_t delimiter = line.find(':');
      if (delimiter != string::npos) {
        string key = line.substr(0, delimiter);
        string value_str = line.substr(delimiter + 1);
        
        // 去除前后空格
        key.erase(0, key.find_first_not_of(" \t"));
        key.erase(key.find_last_not_of(" \t") + 1);
        value_str.erase(0, value_str.find_first_not_of(" \t"));
        value_str.erase(value_str.find_last_not_of(" \t") + 1);
        
        double value_double;
        int value_int;
        
        try {
          if (key == "fx") params.fx = stod(value_str);
          else if (key == "fy") params.fy = stod(value_str);
          else if (key == "cx") params.cx = stod(value_str);
          else if (key == "cy") params.cy = stod(value_str);
          else if (key == "k1") params.k1 = stod(value_str);
          else if (key == "k2") params.k2 = stod(value_str);
          else if (key == "p1") params.p1 = stod(value_str);
          else if (key == "p2") params.p2 = stod(value_str);
          else if (key == "k3") params.k3 = stod(value_str);
          else if (key == "k4") params.k4 = stod(value_str);
          else if (key == "k5") params.k5 = stod(value_str);
          else if (key == "k6") params.k6 = stod(value_str);
          else if (key == "marker_size") params.marker_size = stod(value_str);
          else if (key == "delta_width_qr_center") params.delta_width_qr_center = stod(value_str);
          else if (key == "delta_height_qr_center") params.delta_height_qr_center = stod(value_str);
          else if (key == "delta_width_circles") params.delta_width_circles = stod(value_str);
          else if (key == "delta_height_circles") params.delta_height_circles = stod(value_str);
          else if (key == "circle_radius") params.circle_radius = stod(value_str);
          else if (key == "min_detected_markers") params.min_detected_markers = stoi(value_str);
          else if (key == "image_path") params.image_path = value_str;
          else if (key == "pointcloud_path") params.pointcloud_path = value_str;
          else if (key == "output_path") params.output_path = value_str;
          else if (key == "x_min") params.x_min = stod(value_str);
          else if (key == "x_max") params.x_max = stod(value_str);
          else if (key == "y_min") params.y_min = stod(value_str);
          else if (key == "y_max") params.y_max = stod(value_str);
          else if (key == "z_min") params.z_min = stod(value_str);
          else if (key == "z_max") params.z_max = stod(value_str);
        } catch (const std::exception& e) {
          cerr << "Error parsing parameter '" << key << "' with value '" << value_str << "': " << e.what() << endl;
        }
      }
    }
    config.close();
    cout << BOLDGREEN << "[Config] Loaded parameters from: " << config_file << RESET << endl;
  } else {
    cout << BOLDYELLOW << "[Config] Using default parameters" << RESET << endl;
  }

  std::cout<<"params.k6:"<<params.k6<<std::endl;
  
  return params;
}




Eigen::Matrix4f LodaMatrix(const std::string &lidar_name)
{
    std::string str_tmp = "TongLi5_Calib_Param_v1.toml";
    Eigen::Matrix4f ret_map = Eigen::Matrix4f::Identity();

    try
    {
        std::ifstream ifs(str_tmp, std::ios_base::binary);
        const auto cfg_file = toml::parse(ifs);

        if (ifs.good())
        {
            const auto &tfmatrix = toml::find(cfg_file, lidar_name);
            const auto rotation_vec = toml::find<std::vector<float>>(tfmatrix, "rotation");
            const auto transform_vec = toml::find<std::vector<float>>(tfmatrix, "transform");

            Eigen::Matrix3f rotation;
            rotation << rotation_vec[0], rotation_vec[1], rotation_vec[2],
                rotation_vec[3], rotation_vec[4], rotation_vec[5], rotation_vec[6],
                rotation_vec[7], rotation_vec[8];

            Eigen::Vector3f transform;
            transform << transform_vec[0], transform_vec[1], transform_vec[2];

            ret_map.block<3, 1>(0, 3) = transform;
            ret_map.block<3, 3>(0, 0) = rotation;
        }
        std::cout << "----loadRTMatrix----" << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cout << "矩阵为空或不符合规定" << std::endl;
        return ret_map;
    }

    std::cout << lidar_name << " , ret_map:" << ret_map << std::endl;
    return ret_map;
}

/**
 * @brief 将所有ring的点云合并为一个彩色点云（不同ring用不同颜色）
 * 
 * @param cloud_common 输入点云
 * @param output_dir 输出目录
 */
void createColoredRingPointCloud(
    const pcl::PointCloud<Common::Point>::Ptr& cloud_common,
    const std::string& output_dir)
{
    if (!cloud_common || cloud_common->empty())
        return;
    
    // 创建RGB点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    colored_cloud->resize(cloud_common->size());
    
    // 预定义32种颜色（彩虹色系）
    std::vector<std::tuple<int, int, int>> colors = {
        {255, 0, 0},      // 红色 ring 1
        {255, 64, 0},     // 橙红
        {255, 128, 0},    // 橙色
        {255, 191, 0},    // 橙黄
        {255, 255, 0},    // 黄色 ring 5
        {191, 255, 0},    // 黄绿
        {128, 255, 0},    // 绿黄
        {64, 255, 0},     // 黄绿
        {0, 255, 0},      // 绿色 ring 9
        {0, 255, 64},     // 绿青
        {0, 255, 128},    // 青绿
        {0, 255, 191},    // 青
        {0, 255, 255},    // 青色 ring 13
        {0, 191, 255},    // 青蓝
        {0, 128, 255},    // 蓝青
        {0, 64, 255},     // 蓝青
        {0, 0, 255},      // 蓝色 ring 17
        {64, 0, 255},     // 蓝紫
        {128, 0, 255},    // 紫蓝
        {191, 0, 255},    // 紫
        {255, 0, 255},    // 紫色 ring 21
        {255, 0, 191},    // 紫红
        {255, 0, 128},    // 红紫
        {255, 0, 64},     // 红紫
        {128, 128, 128},  // 灰色 ring 25
        {160, 160, 160},  // 浅灰
        {192, 192, 192},  // 银灰
        {224, 224, 224},  // 浅银
        {255, 255, 255},  // 白色 ring 29
        {255, 200, 200},  // 浅粉
        {200, 255, 200},  // 浅绿
        {200, 200, 255}   // 浅蓝 ring 32
    };
    
    // 分配颜色
    for (size_t i = 0; i < cloud_common->size(); ++i)
    {
        const Common::Point& src_point = cloud_common->points[i];
        pcl::PointXYZRGB& dst_point = colored_cloud->points[i];
        
        dst_point.x = src_point.x;
        dst_point.y = src_point.y;
        dst_point.z = src_point.z;
        
        int ring = src_point.ring;
        if (ring >= 1 && ring <= 32)
        {
            const auto& color = colors[ring - 1];  // 索引从0开始
            dst_point.r = std::get<0>(color);
            dst_point.g = std::get<1>(color);
            dst_point.b = std::get<2>(color);
        }
        else
        {
            // 无效ring用黑色
            dst_point.r = 0;
            dst_point.g = 0;
            dst_point.b = 0;
        }
    }
    
    colored_cloud->width = colored_cloud->size();
    colored_cloud->height = 1;
    colored_cloud->is_dense = true;
    
    // 保存彩色点云
    std::string colored_file = output_dir + "/all_rings_colored.pcd";
    if (pcl::io::savePCDFileBinaryCompressed(colored_file, *colored_cloud) == 0)
    {
        std::cout << "彩色点云已保存到: " << colored_file << std::endl;
        std::cout << "可在CloudCompare等软件中查看，不同ring显示不同颜色" << std::endl;
    }
}



pcl::PointCloud<Common::Point>::Ptr convertToCommonPointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_input)
{
    // 创建输出点云
    pcl::PointCloud<Common::Point>::Ptr cloud_output(new pcl::PointCloud<Common::Point>);
    
    // 定义ring角度映射表 (ring编号 -> 垂直角度)
    // 注意：这里的角度应该是垂直角度（相对于水平面的夹角）
    std::map<int, float> ring_angle_map = {
        {1, 15.0f},     // 通道1: 15°
        {2, 13.0f},     // 通道2: 13°
        {3, 11.0f},     // 通道3: 11°
        {4, 9.0f},      // 通道4: 9°
        {5, 7.0f},      // 通道5: 7°
        {6, 5.5f},      // 通道6: 5.5°
        {7, 4.0f},      // 通道7: 4°
        {8, 2.67f},     // 通道8: 2.67°
        {9, 1.33f},     // 通道9: 1.33°
        {10, 0.0f},     // 通道10: 0°
        {11, -1.33f},   // 通道11: -1.33°
        {12, -2.67f},   // 通道12: -2.67°
        {13, -4.0f},    // 通道13: -4°
        {14, -5.33f},   // 通道14: -5.33°
        {15, -6.67f},   // 通道15: -6.67°
        {16, -8.0f},    // 通道16: -8°
        {17, -10.0f},   // 通道17: -10°
        {18, -16.0f},   // 通道18: -16°
        {19, -13.0f},   // 通道19: -13°
        {20, -19.0f},   // 通道20: -19°
        {21, -22.0f},   // 通道21: -22°
        {22, -28.0f},   // 通道22: -28°
        {23, -25.0f},   // 通道23: -25°
        {24, -31.0f},   // 通道24: -31°
        {25, -34.0f},   // 通道25: -34°
        {26, -37.0f},   // 通道26: -37°
        {27, -40.0f},   // 通道27: -40°
        {28, -43.0f},   // 通道28: -43°
        {29, -46.0f},   // 通道29: -46°
        {30, -49.0f},   // 通道30: -49°
        {31, -52.0f},   // 通道31: -52°
        {32, -55.0f}    // 通道32: -55°
    };
    
    // 设置输出点云的大小和属性
    cloud_output->resize(cloud_input->size());
    cloud_output->width = cloud_input->width;
    cloud_output->height = cloud_input->height;
    cloud_output->is_dense = cloud_input->is_dense;
    
    // 遍历所有点进行转换
    for (size_t i = 0; i < cloud_input->size(); ++i)
    {
        const auto& src_point = cloud_input->points[i];
        auto& dst_point = cloud_output->points[i];
        
        // 复制坐标
        dst_point.x = src_point.x;
        dst_point.y = src_point.y;
        dst_point.z = src_point.z;
        
        // 计算垂直角度（从水平面到点的角度，单位：度）
        // 注意：这里假设雷达坐标系中，Z轴向上为正
        float range_xy = std::sqrt(src_point.x * src_point.x + src_point.y * src_point.y);
        
        // 处理特殊情况：避免除以0
        if (range_xy < 1e-6)
        {
            // 如果点在Z轴上，根据Z的正负决定角度
            if (src_point.z > 0)
                dst_point.ring = 1;  // 接近15度，选择最高通道
            else if (src_point.z < 0)
                dst_point.ring = 32; // 接近-55度，选择最低通道
            else
                dst_point.ring = 10; // Z=0，对应0度
            continue;
        }
        
        // 计算俯仰角（垂直角度），从水平面到点的角度
        // atan2(z, xy_plane_distance) * 180 / M_PI
        float vertical_angle = std::atan2(src_point.z, range_xy) * 180.0f / M_PI;
        
        // 查找最接近的ring角度
        int best_ring = 1;  // 默认值
        float min_angle_diff = std::numeric_limits<float>::max();
        
        for (const auto& pair : ring_angle_map)
        {
            float angle_diff = std::fabs(vertical_angle - pair.second);
            if (angle_diff < min_angle_diff)
            {
                min_angle_diff = angle_diff;
                best_ring = pair.first;
            }
        }
        
        dst_point.ring = static_cast<std::uint16_t>(best_ring);
    }
    
    return cloud_output;
}




double computeRMSE(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud1, 
                   const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud2) 
{
    if (cloud1->size() != cloud2->size()) 
    {
      std::cerr << BOLDRED << "[computeRMSE] Point cloud sizes do not match, cannot compute RMSE." << RESET << std::endl;
      return -1.0;
    }

    double sum = 0.0;
    for (size_t i = 0; i < cloud1->size(); ++i) 
    {
      double dx = cloud1->points[i].x - cloud2->points[i].x;
      double dy = cloud1->points[i].y - cloud2->points[i].y;
      double dz = cloud1->points[i].z - cloud2->points[i].z;

      sum += dx * dx + dy * dy + dz * dz;
    }

    double mse = sum / cloud1->size();
    return std::sqrt(mse);
}

// 将 LiDAR 点云转换到 QR 码坐标系
void alignPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
  pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud, const Eigen::Matrix4f &transformation) 
{
  output_cloud->clear();
  for (const auto &pt : input_cloud->points) 
  {
    Eigen::Vector4f pt_homogeneous(pt.x, pt.y, pt.z, 1.0);
    Eigen::Vector4f transformed_pt = transformation * pt_homogeneous;
    output_cloud->push_back(pcl::PointXYZ(transformed_pt(0), transformed_pt(1), transformed_pt(2)));
  }
}

void comb(int N, int K, std::vector<std::vector<int>> &groups) {
  int upper_factorial = 1;
  int lower_factorial = 1;

  for (int i = 0; i < K; i++) {
    upper_factorial *= (N - i);
    lower_factorial *= (K - i);
  }
  int n_permutations = upper_factorial / lower_factorial;

  if (DEBUG)
    cout << N << " centers found. Iterating over " << n_permutations
         << " possible sets of candidates" << endl;

  std::string bitmask(K, 1);  // K leading 1's
  bitmask.resize(N, 0);       // N-K trailing 0's

  // print integers and permute bitmask
  do {
    std::vector<int> group;
    for (int i = 0; i < N; ++i)  // [0..N-1] integers
    {
      if (bitmask[i]) {
        group.push_back(i);
      }
    }
    groups.push_back(group);
  } while (std::prev_permutation(bitmask.begin(), bitmask.end()));

  assert(groups.size() == n_permutations);
}


void projectPointCloudToImage(const pcl::PointCloud<Common::Point>::Ptr& cloud,
                             const Eigen::Matrix4f& transformation,
                             const cv::Mat& cameraMatrix,
                             const cv::Mat& distCoeffs,
                             const cv::Mat& image,
                             cv::Mat& image_out) 
{
    // 复制输入图像到输出
    image_out = image.clone();
    
    // 去畸变参数（零畸变）
    cv::Mat zeroDistCoeffs = cv::Mat::zeros(5, 1, CV_32F);
    
    for (const auto& point : *cloud) 
    {
        // 变换到相机坐标系
        Eigen::Vector4f point_vec(point.x, point.y, point.z, 1.0f);
        Eigen::Vector4f transformed = transformation * point_vec;
        
        // 跳过相机后的点
        if (transformed[2] <= 0) continue;
        
        // 投影到图像平面
        std::vector<cv::Point3f> objectPoints = {cv::Point3f(transformed[0], transformed[1], transformed[2])};
        std::vector<cv::Point2f> imagePoints;
        
        cv::projectPoints(objectPoints, 
                         cv::Mat::zeros(3, 1, CV_32F),  // 旋转向量（零）
                         cv::Mat::zeros(3, 1, CV_32F),  // 平移向量（零）
                         cameraMatrix, 
                         distCoeffs, 
                         imagePoints);
        
        int u = static_cast<int>(imagePoints[0].x);
        int v = static_cast<int>(imagePoints[0].y);
        
        // 检查图像边界
        if (u >= 0 && u < image.cols && v >= 0 && v < image.rows) 
        {
            // 在图像上绘制点（红色）
            cv::circle(image_out, cv::Point(u, v), 2, cv::Scalar(0, 0, 255), -1);
        }
    }
}


void printCalibrationResult(const Eigen::Matrix4f& transformation, double rmse) {
    // 输出变换矩阵
    std::cout << BOLDYELLOW << "\n[Calibration Result] Extrinsic parameters T_camera_lidar:" 
              << RESET << std::endl;
    std::cout << BOLDCYAN << std::fixed << std::setprecision(6) 
              << transformation << RESET << std::endl;
    
    // 分解变换矩阵
    LOG_INFO("\n[Calibration Details]");
    Eigen::Matrix3f rotation = transformation.block<3, 3>(0, 0);
    Eigen::Vector3f translation = transformation.block<3, 1>(0, 3);
    
    std::cout << "Transformation matrix inverse:\n" 
              << std::fixed << std::setprecision(6) 
              << transformation.inverse() << std::endl;
    
    std::cout << "Rotation matrix R:\n" 
              << std::fixed << std::setprecision(6) 
              << rotation << std::endl;
    
    std::cout << "Translation vector t: [" 
              << std::fixed << std::setprecision(6)
              << translation[0] << ", "
              << translation[1] << ", "
              << translation[2] << "]^T" << std::endl;
    
    // 计算欧拉角
    Eigen::Vector3f euler_angles = rotation.eulerAngles(2, 1, 0); // ZYX顺序
    std::cout << "Euler angles (ZYX order): [" 
              << std::fixed << std::setprecision(4)
              << euler_angles[0] * 180.0 / M_PI << "°, "  // yaw
              << euler_angles[1] * 180.0 / M_PI << "°, "  // pitch
              << euler_angles[2] * 180.0 / M_PI << "°]" << std::endl;
    
    // 输出RMSE
    std::cout << "Alignment RMSE: " << std::fixed << std::setprecision(6) 
              << rmse << std::endl;
}

void projectPointCloudToImage(const pcl::PointCloud<Common::Point>::Ptr& cloud,
  const Eigen::Matrix4f& transformation,
  const cv::Mat& cameraMatrix,
  const cv::Mat& distCoeffs,
  const cv::Mat& image,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr& colored_cloud) 
{
  colored_cloud->clear();
  colored_cloud->reserve(cloud->size());

  // Undistort the entire image (preprocess outside if possible)
  cv::Mat undistortedImage;
  cv::undistort(image, undistortedImage, cameraMatrix, distCoeffs);

  // Precompute rotation and translation vectors (zero for this case)
  cv::Mat rvec = cv::Mat::zeros(3, 1, CV_32F);
  cv::Mat tvec = cv::Mat::zeros(3, 1, CV_32F);
  cv::Mat zeroDistCoeffs = cv::Mat::zeros(5, 1, CV_32F);

  // Preallocate memory for projection
  std::vector<cv::Point3f> objectPoints(1);
  std::vector<cv::Point2f> imagePoints(1);

  for (const auto& point : *cloud) 
  {
    // Transform the point
    Eigen::Vector4f homogeneous_point(point.x, point.y, point.z, 1.0f);
    Eigen::Vector4f transformed_point = transformation * homogeneous_point;

    // Skip points behind the camera
    if (transformed_point(2) < 0) continue;

    // Project the point to the image plane
    objectPoints[0] = cv::Point3f(transformed_point(0), transformed_point(1), transformed_point(2));
    cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix, zeroDistCoeffs, imagePoints);

    int u = static_cast<int>(imagePoints[0].x);
    int v = static_cast<int>(imagePoints[0].y);

    // Check if the point is within the image bounds
    if (u >= 0 && u < undistortedImage.cols && v >= 0 && v < undistortedImage.rows) 
    {
      // Get the color from the undistorted image
      cv::Vec3b color = undistortedImage.at<cv::Vec3b>(v, u);

      // Create a colored point and add it to the cloud
      pcl::PointXYZRGB colored_point;
      colored_point.x = transformed_point(0);
      colored_point.y = transformed_point(1);
      colored_point.z = transformed_point(2);
      colored_point.r = color[2];
      colored_point.g = color[1];
      colored_point.b = color[0];
      colored_cloud->push_back(colored_point);
    }
  }
}

void saveTargetHoleCenters(const pcl::PointCloud<pcl::PointXYZ>::Ptr& lidar_centers,
                      const pcl::PointCloud<pcl::PointXYZ>::Ptr& qr_centers,
                      const Params& params)
{
    if (lidar_centers->size() != 4 || qr_centers->size() != 4) {
      std::cerr << "[saveTargetHoleCenters] The number of points in lidar_centers or qr_centers is not 4, skip saving." << std::endl;
      return;
    }
    
    std::string saveDir = params.output_path;
    if (saveDir.back() != '/') saveDir += '/';
    
    // 确保输出目录存在
    system(("mkdir -p " + saveDir).c_str());
    
    std::ofstream saveFile(saveDir + "circle_center_record.txt", std::ios::app);

    if (!saveFile.is_open()) {
        std::cerr << "[saveTargetHoleCenters] Cannot open file: " << saveDir + "circle_center_record.txt" << std::endl;
        return;
    }

    // 获取当前系统时间
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    char buffer[80];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", std::localtime(&now_time));
    saveFile << "time: " << buffer << std::endl;

    saveFile << "lidar_centers:";
    for (const auto& pt : lidar_centers->points) {
        saveFile << " {" << pt.x << "," << pt.y << "," << pt.z << "}";
    }
    saveFile << std::endl;
    saveFile << "qr_centers:";
    for (const auto& pt : qr_centers->points) {
        saveFile << " {" << pt.x << "," << pt.y << "," << pt.z << "}";
    }
    saveFile << std::endl;
    saveFile.close();
    std::cout << BOLDGREEN << "[Record] Saved four pairs of circular hole centers to " << BOLDWHITE << saveDir << "circle_center_record.txt" << RESET << std::endl;
}

void saveCalibrationResults(const Params& params, const Eigen::Matrix4f& transformation, 
     const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& colored_cloud, const cv::Mat& img_input)
{
  if(colored_cloud->empty()) 
  {
    std::cerr << BOLDRED << "[saveCalibrationResults] Colored point cloud is empty!" << RESET << std::endl;
    return;
  }
  std::string outputDir = params.output_path;
  if (outputDir.back() != '/') outputDir += '/';
  
  // 确保输出目录存在
  system(("mkdir -p " + outputDir).c_str());

  // std::ofstream outFile(outputDir + "single_calib_result.txt");
  // if (outFile.is_open()) 
  // {
  //   outFile << "# FAST-LIVO2 calibration format\n";
  //   outFile << "cam_model: Pinhole\n";
  //   outFile << "cam_width: " << img_input.cols << "\n";
  //   outFile << "cam_height: " << img_input.rows << "\n";
  //   outFile << "scale: 1.0\n";
  //   outFile << "cam_fx: " << params.fx << "\n";
  //   outFile << "cam_fy: " << params.fy << "\n";
  //   outFile << "cam_cx: " << params.cx << "\n";
  //   outFile << "cam_cy: " << params.cy << "\n";
  //   outFile << "cam_d0: " << params.k1 << "\n";
  //   outFile << "cam_d1: " << params.k2 << "\n";
  //   outFile << "cam_d2: " << params.p1 << "\n";
  //   outFile << "cam_d3: " << params.p2 << "\n";

  //   outFile << "\nRcl: [" << std::fixed << std::setprecision(6);
  //   outFile << std::setw(10) << transformation(0, 0) << ", " << std::setw(10) << transformation(0, 1) << ", " << std::setw(10) << transformation(0, 2) << ",\n";
  //   outFile << "      " << std::setw(10) << transformation(1, 0) << ", " << std::setw(10) << transformation(1, 1) << ", " << std::setw(10) << transformation(1, 2) << ",\n";
  //   outFile << "      " << std::setw(10) << transformation(2, 0) << ", " << std::setw(10) << transformation(2, 1) << ", " << std::setw(10) << transformation(2, 2) << "]\n";

  //   outFile << "Pcl: [";
  //   outFile << std::setw(10) << transformation(0, 3) << ", " << std::setw(10) << transformation(1, 3) << ", " << std::setw(10) << transformation(2, 3) << "]\n";

  //   outFile.close();
  //   std::cout << BOLDYELLOW << "[Result] Single-scene calibration results saved to " << BOLDWHITE << outputDir << "single_calib_result.txt" << RESET << std::endl;
  // } 
  // else
  // {
  //   std::cerr << BOLDRED << "[Error] Failed to open single_calib_result.txt for writing!" << RESET << std::endl;
  // }
  
  if (pcl::io::savePCDFileASCII(outputDir + "colored_cloud.pcd", *colored_cloud) == 0) 
  {
    std::cout << BOLDYELLOW << "[Result] Saved colored point cloud to: " << BOLDWHITE << outputDir << "colored_cloud.pcd" << RESET << std::endl;
  } 
  else 
  {
    std::cerr << BOLDRED << "[Error] Failed to save colored point cloud to " << outputDir << "colored_cloud.pcd" << "!" << RESET << std::endl;
  }
 
  imwrite( "qr_detect.png", img_input);
}




void sortPatternCentersGeometric(pcl::PointCloud<pcl::PointXYZ>::Ptr pc,
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr v,
                                 const std::string& sensor_type = "camera") 
{
    if (pc->size() != 4) {
        std::cerr << "Error: Need exactly 4 points." << std::endl;
        return;
    }

    // 1. 计算三维质心
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*pc, centroid);
    float cx = centroid[0], cy = centroid[1], cz = centroid[2];

    // 2. 选择一个投影平面。对于大致水平的标定板，可以忽略高度变化最小的轴。
    //    这里使用PCA找到变化最大的两个主轴，更通用。
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZ>);
    *cloud_2d = *pc; // 实际应用中，这里应进行PCA，但为简化，假设XY平面变化最大

    // 3. 计算各点相对于质心的极角 (使用忽略Z值的2D投影)
    std::vector<std::pair<float, int>> angle_index_pairs; // (angle, original_index)
    for (size_t i = 0; i < cloud_2d->size(); ++i) {
        const auto& p = cloud_2d->points[i];
        float dx = p.x - cx;
        float dy = p.y - cy;
        float angle = atan2(dy, dx); // 范围 [-pi, pi]
        angle_index_pairs.emplace_back(angle, i);
    }

    // 4. 按极角排序（逆时针）
    std::sort(angle_index_pairs.begin(), angle_index_pairs.end(),
              [](const std::pair<float, int>& a, const std::pair<float, int>& b) {
                  return a.first < b.first;
              });

    // 5. 统一化起点：找到X最小的点作为起点，保证不同传感器排序一致性
    // 注意：现在按极角排序后，起点是角度最小的点。但为了更鲁棒，可以强制以“最左点”为起点。
    int start_idx = 0;
    float min_x = std::numeric_limits<float>::max();
    for (int i = 0; i < 4; ++i) {
        int orig_idx = angle_index_pairs[i].second;
        if (pc->points[orig_idx].x < min_x) {
            min_x = pc->points[orig_idx].x;
            start_idx = i;
        }
    }

    // 6. 重新组织顺序，以选定的起点开始
    v->resize(4);
    for (int i = 0; i < 4; ++i) {
        int src_idx = (start_idx + i) % 4;
        int orig_point_idx = angle_index_pairs[src_idx].second;
        v->points[i] = pc->points[orig_point_idx];
    }

    // 可选：验证并确保是逆时针顺序（计算叉积）
    const auto& p0 = v->points[0];
    const auto& p1 = v->points[1];
    const auto& p2 = v->points[2];
    Eigen::Vector3f v01(p1.x - p0.x, p1.y - p0.y, 0);
    Eigen::Vector3f v12(p2.x - p1.x, p2.y - p1.y, 0);
    if (v01.cross(v12).z() < 0) { // 如果是顺时针
        // 反转后3个点的顺序
        std::swap((*v)[1], (*v)[3]);
    }

    // 输出调试信息
    std::cout << "[DEBUG] Sorted for " << sensor_type << " (indices after start_idx normalization): ";
    for (int i = 0; i < 4; ++i) {
        int src_idx = (start_idx + i) % 4;
        std::cout << angle_index_pairs[src_idx].second << " ";
    }
    std::cout << std::endl;
}




void sortPatternCenters(pcl::PointCloud<pcl::PointXYZ>::Ptr pc,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr v,
                        const std::string& axis_mode = "camera") 
{
  if (pc->size() != 4) {
    std::cerr << BOLDRED << "[sortPatternCenters] Number of " << axis_mode << " center points to be sorted is not 4." << RESET << std::endl;
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr work_pc(new pcl::PointCloud<pcl::PointXYZ>());

  // Coordinate transformation (LiDAR -> Camera)
  if (axis_mode == "lidar") {
    for (const auto& p : *pc) {
      pcl::PointXYZ pt;
      pt.x = -p.y;   // LiDAR Y -> Cam -X
      pt.y = -p.z;   // LiDAR Z -> Cam -Y
      pt.z = p.x;    // LiDAR X -> Cam Z
      work_pc->push_back(pt);
    }
  } else {
    *work_pc = *pc;
  }

  // --- Sorting based on the local coordinate system of the pattern ---
  // 1. Calculate the centroid of the points
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*work_pc, centroid);
  pcl::PointXYZ ref_origin(centroid[0], centroid[1], centroid[2]);

  // 2. Project points to the XY plane relative to the centroid and calculate angles
  std::vector<std::pair<float, int>> proj_points;
  for (size_t i = 0; i < work_pc->size(); ++i) {
    const auto& p = work_pc->points[i];
    Eigen::Vector3f rel_vec(p.x - ref_origin.x, p.y - ref_origin.y, p.z - ref_origin.z);
    proj_points.emplace_back(atan2(rel_vec.y(), rel_vec.x()), i);
  }

  // 3. Sort points based on the calculated angle
  std::sort(proj_points.begin(), proj_points.end());

  // 4. Output the sorted points into the result vector 'v'
  v->resize(4);
  for (int i = 0; i < 4; ++i) {
    (*v)[i] = work_pc->points[proj_points[i].second];
  }

  // 5. Verify the order (ensure it's counter-clockwise) and fix if necessary
  const auto& p0 = v->points[0];
  const auto& p1 = v->points[1];
  const auto& p2 = v->points[2];
  Eigen::Vector3f v01(p1.x - p0.x, p1.y - p0.y, 0);
  Eigen::Vector3f v12(p2.x - p1.x, p2.y - p1.y, 0);
  if (v01.cross(v12).z() > 0) {
    std::swap((*v)[1], (*v)[3]);
  }

  // 6. If the original input was in the lidar frame, transform the sorted points back
  if (axis_mode == "lidar") {
    for (auto& point : v->points) {
      float x_new = point.z;    // Cam Z -> LiDAR X
      float y_new = -point.x;   // Cam -X -> LiDAR Y
      float z_new = -point.y;   // Cam -Y -> LiDAR Z
      point.x = x_new;
      point.y = y_new;
      point.z = z_new;
    }
  }
}

<<<<<<< HEAD
=======


>>>>>>> master
class Square 
{
  private:
    pcl::PointXYZ _center;
    std::vector<pcl::PointXYZ> _candidates;
    float _target_width, _target_height, _target_diagonal;
 
  public:
    Square(std::vector<pcl::PointXYZ> candidates, float width, float height) {
      _candidates = candidates;
      _target_width = width;
      _target_height = height;
      _target_diagonal = sqrt(pow(width, 2) + pow(height, 2));
 
      // Compute candidates centroid
      _center.x = _center.y = _center.z = 0;
      for (int i = 0; i < candidates.size(); ++i) {
        _center.x += candidates[i].x;
        _center.y += candidates[i].y;
        _center.z += candidates[i].z;
      }
 
      _center.x /= candidates.size();
      _center.y /= candidates.size();
      _center.z /= candidates.size();
    }
 
    float distance(pcl::PointXYZ pt1, pcl::PointXYZ pt2) {
      return sqrt(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2) +
                  pow(pt1.z - pt2.z, 2));
    }
 
    pcl::PointXYZ at(int i) {
      assert(0 <= i && i < 4);
      return _candidates[i];
    }
 
    // ==================================================================================================
    // The original is_valid() was too rigid. This version is more robust by checking for two possible
    // orderings of the side lengths (width-height vs. height-width) after angular sorting.
    // ==================================================================================================
    bool is_valid() 
    {
      if (_candidates.size() != 4) return false;

      pcl::PointCloud<pcl::PointXYZ>::Ptr candidates_cloud(new pcl::PointCloud<pcl::PointXYZ>());
      for(const auto& p : _candidates) candidates_cloud->push_back(p);

      // Check if candidates are at a reasonable distance from their centroid
      for (int i = 0; i < _candidates.size(); ++i) {
        float d = distance(_center, _candidates[i]);
        // Check if distance from center to corner is close to half the diagonal length
        if (fabs(d - _target_diagonal / 2.) / (_target_diagonal / 2.) > GEOMETRY_TOLERANCE * 2.0) { // Loosened tolerance slightly
          return false;
        }
      }
      
      // Sort the corners counter-clockwise
      pcl::PointCloud<pcl::PointXYZ>::Ptr sorted_centers(new pcl::PointCloud<pcl::PointXYZ>());
      sortPatternCenters(candidates_cloud, sorted_centers, "camera");
      
      // Get the four side lengths from the sorted points
      float s01 = distance(sorted_centers->points[0], sorted_centers->points[1]);
      float s12 = distance(sorted_centers->points[1], sorted_centers->points[2]);
      float s23 = distance(sorted_centers->points[2], sorted_centers->points[3]);
      float s30 = distance(sorted_centers->points[3], sorted_centers->points[0]);

      // Check for pattern 1: width, height, width, height
      bool pattern1_ok = 
        (fabs(s01 - _target_width) / _target_width < GEOMETRY_TOLERANCE) &&
        (fabs(s12 - _target_height) / _target_height < GEOMETRY_TOLERANCE) &&
        (fabs(s23 - _target_width) / _target_width < GEOMETRY_TOLERANCE) &&
        (fabs(s30 - _target_height) / _target_height < GEOMETRY_TOLERANCE);

      // Check for pattern 2: height, width, height, width
      bool pattern2_ok = 
        (fabs(s01 - _target_height) / _target_height < GEOMETRY_TOLERANCE) &&
        (fabs(s12 - _target_width) / _target_width < GEOMETRY_TOLERANCE) &&
        (fabs(s23 - _target_height) / _target_height < GEOMETRY_TOLERANCE) &&
        (fabs(s30 - _target_width) / _target_width < GEOMETRY_TOLERANCE);

      if (!pattern1_ok && !pattern2_ok) {
        return false;
      }
      
      // Final check on perimeter
      float perimeter = s01 + s12 + s23 + s30;
      float ideal_perimeter = 2 * (_target_width + _target_height);
      if (fabs(perimeter - ideal_perimeter) / ideal_perimeter > GEOMETRY_TOLERANCE) {
        return false;
      }
 
      return true;
    }


};

#endif
