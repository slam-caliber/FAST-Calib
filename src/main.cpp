/* 
Developer: Chunran Zheng <zhengcr@connect.hku.hk>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

<<<<<<< HEAD
#include "qr_detect.hpp"
#include "lidar_detect.hpp"
#include "data_preprocess.hpp"

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "mono_qr_pattern");
    ros::NodeHandle nh;

    // 读取参数
    Params params = loadParameters(nh);

    // 初始化 QR 检测和 LiDAR 检测
    QRDetectPtr qrDetectPtr;
    qrDetectPtr.reset(new QRDetect(nh, params));

    LidarDetectPtr lidarDetectPtr;
    lidarDetectPtr.reset(new LidarDetect(nh, params));

    DataPreprocessPtr dataPreprocessPtr;
    dataPreprocessPtr.reset(new DataPreprocess(params));

    // 读取图像和点云
    cv::Mat img_input = dataPreprocessPtr->img_input_;
    pcl::PointCloud<Common::Point>::Ptr cloud_input = dataPreprocessPtr->cloud_input_;
    
    // 检测 QR 码
    PointCloud<PointXYZ>::Ptr qr_center_cloud(new PointCloud<PointXYZ>);
    qr_center_cloud->reserve(4);
    qrDetectPtr->detect_qr(img_input, qr_center_cloud);

    // 检测 LiDAR 数据
    PointCloud<PointXYZ>::Ptr lidar_center_cloud(new PointCloud<PointXYZ>);
    lidar_center_cloud->reserve(4);
    
    switch (dataPreprocessPtr->lidar_type_)
    {
        case LiDARType::Solid:
            lidarDetectPtr->detect_solid_lidar(cloud_input, lidar_center_cloud);
            break;

        case LiDARType::Mech:
            lidarDetectPtr->detect_mech_lidar(cloud_input, lidar_center_cloud);
            break;

        default:
            std::cerr << BOLDYELLOW 
                    << "[Main] Unknown LiDAR type." 
                    << RESET << std::endl;
            break;
    }

    // 对 QR 和 LiDAR 检测到的圆心进行排序
    PointCloud<PointXYZ>::Ptr qr_centers(new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr lidar_centers(new PointCloud<PointXYZ>);
    sortPatternCenters(qr_center_cloud, qr_centers, "camera");
    sortPatternCenters(lidar_center_cloud, lidar_centers, "lidar");

    // 保存中间结果：排序后的 LiDAR 圆心和 QR 圆心
    saveTargetHoleCenters(lidar_centers, qr_centers, params);

    // 计算外参
    Eigen::Matrix4f transformation;
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> svd;
    svd.estimateRigidTransformation(*lidar_centers, *qr_centers, transformation);

    // 将 LiDAR 点云转换到 QR 码坐标系
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_lidar_centers(new pcl::PointCloud<pcl::PointXYZ>);
    aligned_lidar_centers->reserve(lidar_centers->size());
    alignPointCloud(lidar_centers, aligned_lidar_centers, transformation);
    
    double rmse = computeRMSE(qr_centers, aligned_lidar_centers);
    if (rmse > 0) 
    {
      std::cout << BOLDYELLOW << "[Result] RMSE: " << BOLDRED << std::fixed << std::setprecision(4)
      << rmse << " m" << RESET << std::endl;
    }

    std::cout << BOLDYELLOW << "[Result] Single-scene calibration: extrinsic parameters T_cam_lidar = " << RESET << std::endl;
    std::cout << BOLDCYAN << std::fixed << std::setprecision(6) << transformation << RESET << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    projectPointCloudToImage(cloud_input, transformation, qrDetectPtr->cameraMatrix_, qrDetectPtr->distCoeffs_, img_input, colored_cloud);

    saveCalibrationResults(params, transformation, colored_cloud, qrDetectPtr->imageCopy_);

    ros::Publisher colored_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("colored_cloud", 1);
    ros::Publisher aligned_lidar_centers_pub = nh.advertise<sensor_msgs::PointCloud2>("aligned_lidar_centers", 1);

    // 主循环
    ros::Rate rate(1);
    while (ros::ok()) 
    {
      if (DEBUG) 
      {
        // 发布 QR 检测结果
        sensor_msgs::PointCloud2 qr_centers_msg;
        pcl::toROSMsg(*qr_centers, qr_centers_msg);
        qr_centers_msg.header.stamp = ros::Time::now();
        qr_centers_msg.header.frame_id = "map";
        qrDetectPtr->qr_pub_.publish(qr_centers_msg);

        // 发布 LiDAR 检测结果
        sensor_msgs::PointCloud2 lidar_centers_msg;
        pcl::toROSMsg(*lidar_centers, lidar_centers_msg);
        lidar_centers_msg.header = qr_centers_msg.header;
        lidarDetectPtr->center_pub_.publish(lidar_centers_msg);

        // 发布中间结果
        sensor_msgs::PointCloud2 filtered_cloud_msg;
        pcl::toROSMsg(*lidarDetectPtr->getFilteredCloud(), filtered_cloud_msg);
        filtered_cloud_msg.header = qr_centers_msg.header;
        lidarDetectPtr->filtered_pub_.publish(filtered_cloud_msg);

        sensor_msgs::PointCloud2 plane_cloud_msg;
        pcl::toROSMsg(*lidarDetectPtr->getPlaneCloud(), plane_cloud_msg);
        plane_cloud_msg.header = qr_centers_msg.header;
        lidarDetectPtr->plane_pub_.publish(plane_cloud_msg);

        sensor_msgs::PointCloud2 aligned_cloud_msg;
        pcl::toROSMsg(*lidarDetectPtr->getAlignedCloud(), aligned_cloud_msg);
        aligned_cloud_msg.header = qr_centers_msg.header;
        lidarDetectPtr->aligned_pub_.publish(aligned_cloud_msg);

        sensor_msgs::PointCloud2 edge_cloud_msg;
        pcl::toROSMsg(*lidarDetectPtr->getEdgeCloud(), edge_cloud_msg);
        edge_cloud_msg.header = qr_centers_msg.header;
        lidarDetectPtr->edge_pub_.publish(edge_cloud_msg);

        sensor_msgs::PointCloud2 lidar_centers_z0_msg;
        pcl::toROSMsg(*lidarDetectPtr->getCenterZ0Cloud(), lidar_centers_z0_msg);
        lidar_centers_z0_msg.header = qr_centers_msg.header;
        lidarDetectPtr->center_z0_pub_.publish(lidar_centers_z0_msg);

        // 发布外参变换后的LiDAR点云
        sensor_msgs::PointCloud2 aligned_lidar_centers_msg;
        pcl::toROSMsg(*aligned_lidar_centers, aligned_lidar_centers_msg);
        aligned_lidar_centers_msg.header = qr_centers_msg.header;
        aligned_lidar_centers_pub.publish(aligned_lidar_centers_msg);

        // 发布彩色点云
        sensor_msgs::PointCloud2 colored_cloud_msg;
        pcl::toROSMsg(*colored_cloud, colored_cloud_msg);
        colored_cloud_msg.header = qr_centers_msg.header;
        colored_cloud_pub.publish(colored_cloud_msg);

        // cv::imshow("result", qrDetectPtr->imageCopy_);
      }
      // cv::waitKey(1);
      ros::spinOnce();
      rate.sleep();
    }

=======
#include "lidar_detect.hpp"
#include "qr_detect.hpp"
#include "common_lib.h"
#include <iostream>
#include <iomanip>
#include <string>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <chrono>

// 日志宏定义
#define LOG_INFO(msg) std::cout << BOLDCYAN << "[INFO] " << msg << RESET << std::endl
#define LOG_WARN(msg) std::cout << BOLDYELLOW << "[WARN] " << msg << RESET << std::endl
#define LOG_ERROR(msg) std::cerr << BOLDRED << "[ERROR] " << msg << RESET << std::endl
#define LOG_SUCCESS(msg) std::cout << BOLDGREEN << "[SUCCESS] " << msg << RESET << std::endl

int main(int argc, char **argv) 
{
    LOG_INFO("Starting Camera-LiDAR  Calibration...");
    // 检查命令行参数
    if (argc < 1) {
        LOG_INFO("Example: " << argv[0] << " config.yaml input.pcd");
        return 1;
    }
    std::string config_file = argv[1];

    
    try {
        // 1. 读取参数
        LOG_INFO("Loading parameters from: " << config_file);
        Params params = loadParameters(config_file);
        LOG_INFO("Loading point cloud from: " << params.pointcloud_path);
        //加载雷达到车体标定参数
        Eigen::Matrix4f right_lidar_to_car = LodaMatrix("RightLidarToCar");
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZ>);
        
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(params.pointcloud_path, *cloud_input) == -1) {
            LOG_ERROR("Could not read PCD file: " << params.pointcloud_path);
            return 1;
        }
        LOG_INFO("Loaded " << cloud_input->size() << " points");

        
        //2. 雷达检测
        LOG_INFO("LiDAR detector...");
        LidarDetectPtr lidarDetectPtr(new LidarDetect(params));
        pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_center_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if(params.use_mech_lidar)
        {
            pcl::PointCloud<Common::Point>::Ptr cloud_input_ring;
            pcl::PointCloud<Common::Point>::Ptr cloud_common = convertToCommonPointCloud(cloud_input);
            createColoredRingPointCloud(cloud_common, params.output_path);
            LOG_INFO("Using mech LiDAR detection method...");
            lidarDetectPtr->detect_mech_lidar(cloud_common, lidar_center_cloud);
        }
        else
        {
            LOG_INFO("Using solid-state LiDAR detection method...");
            lidarDetectPtr->detect_solid_lidar(cloud_input, lidar_center_cloud);
        }


        //3. 相机检测
        LOG_INFO("Camera QR detector...");
        LOG_INFO("Loading image from: " << params.image_path);
        cv::Mat img_input = cv::imread(params.image_path);
        if (img_input.empty()) {
            LOG_ERROR("Could not read image: " << params.image_path);
            return 1;
        }
        LOG_INFO("Image loaded - Size: " << img_input.cols << "x" << img_input.rows << ", Channels: " << img_input.channels());
        QRDetectPtr qrDetectPtr(new QRDetect(params));
        pcl::PointCloud<pcl::PointXYZ>::Ptr qr_center_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        qrDetectPtr->detect_qr(img_input, qr_center_cloud);


        //4. 标定
        bool can_calibrate = (qr_center_cloud->size() >= 3 && lidar_center_cloud->size() >= 3);
        if (can_calibrate) {
            LOG_INFO("Both camera and LiDAR detections have enough points for calibration.");
            LOG_INFO("Camera points: " << qr_center_cloud->size() << ", LiDAR points: " << lidar_center_cloud->size());
            
            //4.1. 排序
            pcl::PointCloud<pcl::PointXYZ>::Ptr qr_centers_sorted(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_centers_sorted(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud(*lidar_center_cloud, *lidar_center_cloud, right_lidar_to_car);
            sortPatternCenters(qr_center_cloud, qr_centers_sorted, "camera");
            sortPatternCenters(lidar_center_cloud, lidar_centers_sorted, "lidar");
            
            if (DEBUG) {
                LOG_INFO("Sorted Camera centers:");
                for (size_t i = 0; i < qr_centers_sorted->size(); ++i) {
                    const auto& center = qr_centers_sorted->points[i];
                    std::cout << "  Sorted Camera " << i << ": (" 
                              << std::fixed << std::setprecision(4)
                              << center.x << ", " << center.y << ", " << center.z << ")" << std::endl;
                }
                
                LOG_INFO("Sorted LiDAR centers:");
                for (size_t i = 0; i < lidar_centers_sorted->size(); ++i) {
                    const auto& center = lidar_centers_sorted->points[i];
                    std::cout << "  Sorted LiDAR " << i << ": (" 
                              << std::fixed << std::setprecision(4)
                              << center.x << ", " << center.y << ", " << center.z << ")" << std::endl;
                }

                LOG_INFO("Saving detection results...");
                saveTargetHoleCenters(lidar_centers_sorted, qr_centers_sorted, params);
            }
            

            //4.2. 标定
            Eigen::Matrix4f transformation;
            pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> svd;
            svd.estimateRigidTransformation(*lidar_centers_sorted, *qr_centers_sorted, transformation);
            LOG_SUCCESS("Extrinsic calibration computed successfully,lidar->camera!");
            
            pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_lidar_centers(new pcl::PointCloud<pcl::PointXYZ>);
            alignPointCloud(lidar_centers_sorted, aligned_lidar_centers, transformation);
            double rmse = computeRMSE(qr_centers_sorted, aligned_lidar_centers);
            printCalibrationResult(transformation, rmse);
            LOG_INFO("Generating colored point cloud (projecting LiDAR to image)...");
            

            cv::Mat cameraMatrix = (cv::Mat_<float>(3, 3) << params.fx, 0, params.cx,
                                                              0, params.fy, params.cy,
                                                              0, 0, 1);
            cv::Mat distCoeffs = (cv::Mat_<float>(1, 5) << params.k1, params.k2, params.p1, params.p2, 0);
            
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<Common::Point>::Ptr cloud_common_for_projection(new pcl::PointCloud<Common::Point>);
            cloud_common_for_projection->resize(cloud_input->size());
            for (size_t i = 0; i < cloud_input->size(); ++i) {
                cloud_common_for_projection->points[i].x = cloud_input->points[i].x;
                cloud_common_for_projection->points[i].y = cloud_input->points[i].y;
                cloud_common_for_projection->points[i].z = cloud_input->points[i].z;
                cloud_common_for_projection->points[i].ring = 0;
            }
            
            pcl::PointCloud<Common::Point>::Ptr plane_cloud_common_for_projection(new pcl::PointCloud<Common::Point>);

            plane_cloud_common_for_projection = lidarDetectPtr->getPlaneCommonCloud();
            pcl::transformPointCloud(*cloud_common_for_projection, *cloud_common_for_projection, right_lidar_to_car);
            // 投影图像到点云
            projectPointCloudToImage(cloud_common_for_projection, transformation, 
                                     cameraMatrix, distCoeffs, img_input, colored_cloud);
            cv::Mat projected_image;
            // 投影点云到图像
            projectPointCloudToImage(cloud_common_for_projection, transformation, 
                            cameraMatrix, distCoeffs, img_input, projected_image);
            cv::imwrite("Projected.png", projected_image);

            LOG_INFO("Saving calibration results...");
            saveCalibrationResults(params, transformation, colored_cloud, qrDetectPtr->imageCopy_);
            
            LOG_SUCCESS("\n=== CALIBRATION COMPLETED SUCCESSFULLY ===");
            
        } else {
            LOG_WARN("Cannot perform calibration due to insufficient points.");
            LOG_WARN("Camera points: " << qr_center_cloud->size() << ", LiDAR points: " << lidar_center_cloud->size());
            LOG_WARN("Need at least 3 points from each sensor for calibration.");
        }
        
    } catch (const std::exception& e) {
        LOG_ERROR("Exception occurred: " << e.what());
        return 1;
    }
    
>>>>>>> master
    return 0;
}