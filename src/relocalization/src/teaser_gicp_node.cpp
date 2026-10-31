#include <chrono>
#include <memory>
#include <string>
#include <vector>

// ROS2 Headers
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <pcl_conversions/pcl_conversions.h>

// PCL Headers
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

// TEASER++ Headers
#include <teaser/registration.h>
#include <teaser/matcher.h>
#include <teaser/fpfh.h>

// small_gicp Headers
#include <small_gicp/pcl/pcl_registration.hpp>
#include <small_gicp/util/downsampling_omp.hpp>

// Livox Support
#ifdef USE_LIVOX
#include <livox_ros_driver2/msg/custom_msg.hpp>
#endif

// 定位状态枚举
enum class LocalizationState {
    TEASER_PHASE,   // TEASER 全局定位阶段
    GICP_PHASE      // GICP 精细配准阶段
};

class TeaserGicpNode : public rclcpp::Node
{
public:
    TeaserGicpNode()
        : Node("teaser_gicp_node"), state_(LocalizationState::TEASER_PHASE)
    {
        // ================= 参数声明 =================
        
        // 1. 基础配置
        this->declare_parameter("map_path", "");
        this->declare_parameter("map_frame_id", "map");
        this->declare_parameter("pcl_type", "livox");
        // 【本地修复】成功后是否退出。上游硬编码"LOCALIZATION SUCCESS! 就 rclcpp::shutdown()"，
        // 是一次性重定位。默认 false（常驻，持续输出位姿）；要恢复上游行为设 true。
        this->declare_parameter("shutdown_on_success", false);

        // 【本地修复】GICP 连续失败多少次才回退去做 TEASER 全局搜索。
        //
        // 上游写死 5/10，太小：TEASER 是**全局搜索**，每次回退都可能落到另一个
        // 局部最优（实测在两个解之间反复翻转、z 相差 0.3 m），并让
        // transform_publisher 的 map→camera_init 跟着跳，看起来就是"地图乱飘"。
        // 真机实测：4 分钟内回退 48 次、TEASER↔GICP 切换 49 次。
        // 调大后，偶发的坏帧不会再掀桌子，GICP 能一直锁定在同一个解上。
        this->declare_parameter("gicp_fail_thre_not_converged", 30);
        this->declare_parameter("gicp_fail_thre_high_fitness", 30);

        // 2. 降采样参数
        this->declare_parameter("map_voxel_leaf_size", 0.4);     // TEASER 用较大体素
        this->declare_parameter("cloud_voxel_leaf_size", 0.4);   // TEASER 用较大体素
        this->declare_parameter("gicp_map_voxel_leaf_size", 0.1);  // GICP 用较小体素
        this->declare_parameter("gicp_cloud_voxel_leaf_size", 0.1);

        // 3. TEASER 参数
        this->declare_parameter("fpfh_normal_radius", 0.6);
        this->declare_parameter("fpfh_feature_radius", 0.9);
        this->declare_parameter("noise_bound", 0.2);
        this->declare_parameter("teaser_solver_max_iter", 100);
        this->declare_parameter("rotation_gnc_factor", 1.4);
        this->declare_parameter("teaser_inlier_threshold", 30);    // TEASER 内点阈值
        this->declare_parameter("teaser_success_count", 3);        // TEASER 连续成功次数后切换

        // 4. GICP 参数
        this->declare_parameter("gicp_solver_max_iter", 50);
        this->declare_parameter("num_threads", 4);
        this->declare_parameter("max_correspondence_distance", 0.5);
        this->declare_parameter("fitness_score_thre", 0.1);
        this->declare_parameter("converged_count_thre", 20);
        // 【本地修复】确认多少帧后开始“每帧都发”位姿。
        // 上游是“攒满 converged_count_thre 帧才发一次、然后清零”，常驻时每 21 帧
        // 才输出一次，下游 reloc_bridge 根本凑不够稳定样本（实测整段只发 2 次）。
        this->declare_parameter("publish_after_converged", 3);
        this->declare_parameter("registration_type", "VGICP");

        // ================= 获取参数 =================
        this->get_parameter("map_path", map_path_);
        this->get_parameter("map_frame_id", map_frame_id_);
        this->get_parameter("pcl_type", pcl_type_);
        this->get_parameter("shutdown_on_success", shutdown_on_success_);
        this->get_parameter("gicp_fail_thre_not_converged", gicp_fail_thre_not_converged_);
        this->get_parameter("gicp_fail_thre_high_fitness", gicp_fail_thre_high_fitness_);
        
        this->get_parameter("map_voxel_leaf_size", teaser_map_voxel_);
        this->get_parameter("cloud_voxel_leaf_size", teaser_cloud_voxel_);
        this->get_parameter("gicp_map_voxel_leaf_size", gicp_map_voxel_);
        this->get_parameter("gicp_cloud_voxel_leaf_size", gicp_cloud_voxel_);

        this->get_parameter("fpfh_normal_radius", fpfh_normal_radius_);
        this->get_parameter("fpfh_feature_radius", fpfh_feature_radius_);
        this->get_parameter("noise_bound", noise_bound_);
        this->get_parameter("teaser_solver_max_iter", teaser_solver_max_iter_);
        this->get_parameter("rotation_gnc_factor", rotation_gnc_factor_);
        int inlier_thr;
        this->get_parameter("teaser_inlier_threshold", inlier_thr);
        teaser_inlier_threshold_ = static_cast<size_t>(inlier_thr);
        this->get_parameter("teaser_success_count", teaser_success_count_thre_);

        this->get_parameter("gicp_solver_max_iter", gicp_solver_max_iter_);
        this->get_parameter("num_threads", num_threads_);
        this->get_parameter("max_correspondence_distance", max_correspondence_distance_);
        this->get_parameter("fitness_score_thre", fitness_score_thre_);
        this->get_parameter("converged_count_thre", gicp_converged_count_thre_);
        this->get_parameter("publish_after_converged", publish_after_converged_);
        this->get_parameter("registration_type", registration_type_);

        // ================= 初始化 ROS 接口 =================
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("icp_result", 10);
        map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "prior_map", rclcpp::QoS(1).transient_local().reliable());
        transformed_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("transformed_cloud", 10);

#ifdef USE_LIVOX
        if (pcl_type_ == "livox") {
            lvx_cloud_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
                "/livox/lidar", 10, std::bind(&TeaserGicpNode::lvx_cloud_callback, this, std::placeholders::_1));
        } else {
            cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/pointcloud2", 10, std::bind(&TeaserGicpNode::cloud_callback, this, std::placeholders::_1));
        }
#else
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/pointcloud2", 10, std::bind(&TeaserGicpNode::cloud_callback, this, std::placeholders::_1));
#endif
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", 10, std::bind(&TeaserGicpNode::pose_callback, this, std::placeholders::_1));

        // ================= 加载地图 =================
        if (!load_map()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load map. Node will not function properly.");
            return;
        }

        // ================= 初始化 TEASER =================
        init_teaser();

        // ================= 初始化 GICP =================
        init_gicp();

        RCLCPP_INFO(this->get_logger(), "=== TeaserGicpNode Initialized ===");
        RCLCPP_INFO(this->get_logger(), "Starting in TEASER phase for global localization...");
    }

private:
    // ===================== 地图加载 =====================
    bool load_map()
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr raw_map(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path_, *raw_map) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Could not read map file: %s", map_path_.c_str());
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Loaded map with %zu points", raw_map->size());

        // 为 TEASER 降采样地图 (较大体素)
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(raw_map);
        sor.setLeafSize(teaser_map_voxel_, teaser_map_voxel_, teaser_map_voxel_);
        sor.filter(*teaser_map_pcl_);
        RCLCPP_INFO(this->get_logger(), "TEASER map downsampled to %zu points", teaser_map_pcl_->size());

        // 为 GICP 降采样地图 (较小体素)
        gicp_map_pcl_ = small_gicp::voxelgrid_sampling_omp(*raw_map, gicp_map_voxel_, num_threads_);
        RCLCPP_INFO(this->get_logger(), "GICP map downsampled to %zu points", gicp_map_pcl_->size());

        // 发布地图
        sensor_msgs::msg::PointCloud2 map_msg;
        pcl::toROSMsg(*gicp_map_pcl_, map_msg);  // 发布更精细的地图用于可视化
        map_msg.header.stamp = this->now();
        map_msg.header.frame_id = map_frame_id_;
        map_pub_->publish(map_msg);

        return true;
    }

    // ===================== TEASER 初始化 =====================
    void init_teaser()
    {
        // 转换为 TEASER 格式
        convert_pcl_to_teaser(teaser_map_pcl_, teaser_map_cloud_);

        // 计算地图 FPFH
        RCLCPP_INFO(this->get_logger(), "Computing Map FPFH features...");
        teaser::FPFHEstimation fpfh;
        teaser_map_descriptors_ = fpfh.computeFPFHFeatures(teaser_map_cloud_, fpfh_normal_radius_, fpfh_feature_radius_);
        RCLCPP_INFO(this->get_logger(), "Map FPFH features computed.");
    }

    // ===================== GICP 初始化 =====================
    void init_gicp()
    {
        gicp_reg_.setRegistrationType(registration_type_);
        gicp_reg_.setNumThreads(num_threads_);
        gicp_reg_.setMaxCorrespondenceDistance(max_correspondence_distance_);
        gicp_reg_.setCorrespondenceRandomness(20);
        gicp_reg_.setInputTarget(gicp_map_pcl_);
        
        gicp_init_guess_ = Eigen::Matrix4f::Identity();
    }

    // ===================== 核心处理函数 =====================
    void process_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
    {
        if (teaser_map_cloud_.empty() || gicp_map_pcl_->empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Map not ready.");
            return;
        }

        switch (state_) {
            case LocalizationState::TEASER_PHASE:
                process_teaser(input_cloud);
                break;
            case LocalizationState::GICP_PHASE:
                process_gicp(input_cloud);
                break;
        }
    }

    // ===================== TEASER 处理 =====================
    void process_teaser(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
    {
        // 1. 降采样
        pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(input_cloud);
        sor.setLeafSize(teaser_cloud_voxel_, teaser_cloud_voxel_, teaser_cloud_voxel_);
        sor.filter(*src_cloud);

        if (src_cloud->size() < 50) {
            RCLCPP_WARN(this->get_logger(), "[TEASER] Input cloud too small: %zu", src_cloud->size());
            return;
        }

        // 2. 计算 FPFH
        teaser::PointCloud src_teaser;
        convert_pcl_to_teaser(src_cloud, src_teaser);
        teaser::FPFHEstimation fpfh;
        auto src_descriptors = fpfh.computeFPFHFeatures(src_teaser, fpfh_normal_radius_, fpfh_feature_radius_);

        // 3. 特征匹配
        teaser::Matcher matcher;
        auto correspondences = matcher.calculateCorrespondences(
            src_teaser, teaser_map_cloud_, *src_descriptors, *teaser_map_descriptors_, 
            false, true, false, 0.95);

        RCLCPP_INFO(this->get_logger(), "[TEASER] Correspondences found: %zu (threshold: %zu)", 
            correspondences.size(), teaser_inlier_threshold_);

        if (correspondences.size() < 10) {  // 至少需要一些对应点才能求解
            RCLCPP_WARN(this->get_logger(), "[TEASER] Too few correspondences for solving");
            teaser_success_count_ = 0;
            // 发布原始点云用于调试
            publish_debug_cloud(*src_cloud);
            return;
        }

        // 4. TEASER 求解
        teaser::RobustRegistrationSolver::Params params;
        params.noise_bound = noise_bound_;
        params.cbar2 = 1;
        params.estimate_scaling = false;
        params.rotation_max_iterations = teaser_solver_max_iter_;
        params.rotation_gnc_factor = rotation_gnc_factor_;
        params.rotation_estimation_algorithm =
            teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
        params.rotation_cost_threshold = 0.005;

        teaser::RobustRegistrationSolver solver(params);
        auto start = std::chrono::high_resolution_clock::now();
        solver.solve(src_teaser, teaser_map_cloud_, correspondences);
        auto end = std::chrono::high_resolution_clock::now();

        auto solution = solver.getSolution();

        // 5. 结果验证
        if (solution.valid) {
            auto inliers = solver.getTranslationInliers();
            RCLCPP_INFO(this->get_logger(), "[TEASER] Solved in %.2f ms. Inliers: %zu/%zu (threshold: %zu)", 
                std::chrono::duration<double, std::milli>(end - start).count(), 
                inliers.size(), correspondences.size(), teaser_inlier_threshold_);

            // 构建变换矩阵
            Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
            transform.block<3, 3>(0, 0) = solution.rotation.cast<float>();
            transform.block<3, 1>(0, 3) = solution.translation.cast<float>();

            // 发布变换后的点云用于可视化（无论是否达到阈值）
            pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
            pcl::transformPointCloud(*src_cloud, transformed_cloud, transform);
            publish_debug_cloud(transformed_cloud);

            if (inliers.size() >= teaser_inlier_threshold_) {
                teaser_success_count_++;
                RCLCPP_INFO(this->get_logger(), "[TEASER] Success count: %d / %d", 
                    teaser_success_count_, teaser_success_count_thre_);

                // 更新 GICP 初值
                gicp_init_guess_ = transform;

                // 检查是否达到切换条件
                if (teaser_success_count_ >= teaser_success_count_thre_) {
                    RCLCPP_INFO(this->get_logger(), "====================================");
                    RCLCPP_INFO(this->get_logger(), "TEASER phase complete! Switching to GICP phase...");
                    RCLCPP_INFO(this->get_logger(), "Initial pose from TEASER: x=%.3f, y=%.3f, z=%.3f", 
                        transform(0, 3), transform(1, 3), transform(2, 3));
                    RCLCPP_INFO(this->get_logger(), "====================================");
                    
                    state_ = LocalizationState::GICP_PHASE;
                    gicp_converged_count_ = 0;
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "[TEASER] Inliers (%zu) below threshold (%zu), resetting count",
                    inliers.size(), teaser_inlier_threshold_);
                teaser_success_count_ = 0;
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "[TEASER] Solution invalid.");
            teaser_success_count_ = 0;
            // 发布原始点云
            publish_debug_cloud(*src_cloud);
        }
    }

    // ===================== GICP 处理 =====================
    void process_gicp(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
    {
        // 1. 降采样
        auto src_cloud = small_gicp::voxelgrid_sampling_omp(*input_cloud, gicp_cloud_voxel_, num_threads_);

        // 2. 设置 Source
        gicp_reg_.setInputSource(src_cloud);

        // 3. 执行配准
        pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
        gicp_reg_.align(aligned_cloud, gicp_init_guess_);

        // 4. 获取结果
        bool converged = gicp_reg_.hasConverged();
        double fitness_score = gicp_reg_.getFitnessScore();
        Eigen::Matrix4f final_transform = gicp_reg_.getFinalTransformation();

        RCLCPP_INFO(this->get_logger(), "[GICP] Fitness: %.4f, Converged: %d", fitness_score, converged);

        if (converged && fitness_score < fitness_score_thre_) {
            gicp_converged_count_++;
            gicp_init_guess_ = final_transform;      // 接力给下一帧当初值
            if (gicp_converged_count_ > 1000000) gicp_converged_count_ = 1000000;

            // 【本地修复】确认 publish_after_converged_ 帧后**每帧都发**；
            // 不再“攒满 21 帧发一次然后清零”，否则下游根本拿不到连续样本。
            if (has_published_ || gicp_converged_count_ >= publish_after_converged_) {
                if (!has_published_) {
                    has_published_ = true;
                    RCLCPP_INFO(this->get_logger(), "====================================");
                    RCLCPP_INFO(this->get_logger(), "LOCALIZATION SUCCESS! (continuous publishing)");
                    RCLCPP_INFO(this->get_logger(), "First pose: x=%.3f, y=%.3f, z=%.3f",
                        final_transform(0, 3), final_transform(1, 3), final_transform(2, 3));
                    RCLCPP_INFO(this->get_logger(), "====================================");
                }
                publish_pose(final_transform);
                if (shutdown_on_success_) {
                    rclcpp::shutdown();
                    return;
                }
            } else {
                RCLCPP_INFO(this->get_logger(), "[GICP] Converged count: %d / %d",
                    gicp_converged_count_, publish_after_converged_);
            }
        } else {
            if (converged && fitness_score >= fitness_score_thre_) {
                // 收敛但分数高，可能是局部最优，仍然更新
                gicp_init_guess_ = final_transform;
                RCLCPP_WARN(this->get_logger(), "[GICP] High fitness score (%.3f > %.3f), may need better init",
                    fitness_score, fitness_score_thre_);
                
                // 如果连续多次 fitness score 过高，考虑回退到 TEASER
                gicp_fail_count_++;
                if (gicp_fail_count_ > gicp_fail_thre_high_fitness_) {
                    RCLCPP_WARN(this->get_logger(), "GICP failing too often (%d), switching back to TEASER...", gicp_fail_count_);
                    state_ = LocalizationState::TEASER_PHASE;
                    teaser_success_count_ = 0;
                    gicp_fail_count_ = 0;
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "[GICP] Not converged, resetting...");
                gicp_fail_count_++;
                
                if (gicp_fail_count_ > gicp_fail_thre_not_converged_) {
                    RCLCPP_WARN(this->get_logger(), "GICP failing (%d), switching back to TEASER...", gicp_fail_count_);
                    state_ = LocalizationState::TEASER_PHASE;
                    teaser_success_count_ = 0;
                    gicp_fail_count_ = 0;
                }
            }
            gicp_converged_count_ = 0;
        }

        // 发布调试点云
        pcl::PointCloud<pcl::PointXYZ> debug_cloud;
        pcl::transformPointCloud(*src_cloud, debug_cloud, gicp_init_guess_);
        publish_debug_cloud(debug_cloud);
    }

    // ===================== 辅助函数 =====================
    void convert_pcl_to_teaser(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud, teaser::PointCloud& teaser_cloud)
    {
        teaser_cloud.clear();
        teaser_cloud.reserve(pcl_cloud->size());
        for (const auto& p : *pcl_cloud) {
            teaser_cloud.push_back({p.x, p.y, p.z});
        }
    }

    void publish_pose(const Eigen::Matrix4f& transform)
    {
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = map_frame_id_;

        pose_msg.pose.pose.position.x = transform(0, 3);
        pose_msg.pose.pose.position.y = transform(1, 3);
        pose_msg.pose.pose.position.z = transform(2, 3);

        Eigen::Matrix3f rotation = transform.block<3, 3>(0, 0);
        Eigen::Quaternionf q(rotation);
        pose_msg.pose.pose.orientation.x = q.x();
        pose_msg.pose.pose.orientation.y = q.y();
        pose_msg.pose.pose.orientation.z = q.z();
        pose_msg.pose.pose.orientation.w = q.w();

        for (int i = 0; i < 36; i++) pose_msg.pose.covariance[i] = 0.0;
        pose_msg.pose.covariance[0] = 0.01;
        pose_msg.pose.covariance[7] = 0.01;
        pose_msg.pose.covariance[35] = 0.01;

        publisher_->publish(pose_msg);
    }

    void publish_debug_cloud(const pcl::PointCloud<pcl::PointXYZ>& cloud)
    {
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(cloud, cloud_msg);
        cloud_msg.header.stamp = this->now();
        cloud_msg.header.frame_id = map_frame_id_;
        transformed_cloud_pub_->publish(cloud_msg);
    }

    // ===================== ROS 回调 =====================
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *input_cloud);
        process_cloud(input_cloud);
    }

#ifdef USE_LIVOX
    void lvx_cloud_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        input_cloud->reserve(msg->point_num);
        for (uint i = 0; i < msg->point_num; i++) {
            if (std::abs(msg->points[i].x) < 0.01 && std::abs(msg->points[i].y) < 0.01 && std::abs(msg->points[i].z) < 0.01) 
                continue;
            input_cloud->push_back(pcl::PointXYZ(msg->points[i].x, msg->points[i].y, msg->points[i].z));
        }
        process_cloud(input_cloud);
    }
#endif

    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        // 手动设置初始位姿，直接跳转到 GICP 阶段
        gicp_init_guess_ = Eigen::Matrix4f::Identity();
        gicp_init_guess_(0, 3) = msg->pose.pose.position.x;
        gicp_init_guess_(1, 3) = msg->pose.pose.position.y;
        gicp_init_guess_(2, 3) = msg->pose.pose.position.z;

        tf2::Quaternion q;
        tf2::fromMsg(msg->pose.pose.orientation, q);
        tf2::Matrix3x3 rot_mat(q);
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                gicp_init_guess_(i, j) = rot_mat[i][j];
            }
        }

        double r, p, yaw;
        rot_mat.getRPY(r, p, yaw);
        RCLCPP_INFO(this->get_logger(), "Manual pose received: x=%.3f, y=%.3f, z=%.3f, yaw=%.3f",
            msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z, yaw);
        RCLCPP_INFO(this->get_logger(), "Switching to GICP phase with manual initial pose...");

        state_ = LocalizationState::GICP_PHASE;
        gicp_converged_count_ = 0;
        gicp_fail_count_ = 0;
    }

    // ===================== 成员变量 =====================
    
    // ROS 接口
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transformed_cloud_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
#ifdef USE_LIVOX
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lvx_cloud_sub_;
#endif
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;

    // 状态
    LocalizationState state_;

    // TEASER 数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr teaser_map_pcl_{new pcl::PointCloud<pcl::PointXYZ>};
    teaser::PointCloud teaser_map_cloud_;
    std::shared_ptr<teaser::FPFHCloud> teaser_map_descriptors_;
    int teaser_success_count_ = 0;
    int teaser_success_count_thre_;

    // GICP 数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr gicp_map_pcl_{new pcl::PointCloud<pcl::PointXYZ>};
    small_gicp::RegistrationPCL<pcl::PointXYZ, pcl::PointXYZ> gicp_reg_;
    Eigen::Matrix4f gicp_init_guess_;
    int gicp_converged_count_ = 0;
    int gicp_converged_count_thre_;
    int publish_after_converged_ = 3;   // 【本地修复】确认几帧后开始每帧发布
    bool has_published_ = false;        // 【本地修复】是否已经发布过（之后不再重新确认）
    int gicp_fail_count_ = 0;

    // 参数
    std::string map_path_, map_frame_id_, pcl_type_;
    bool shutdown_on_success_ = false;   // 【本地修复】
    int gicp_fail_thre_not_converged_ = 30;   // 【本地修复】连续不收敛多少次才回退 TEASER
    int gicp_fail_thre_high_fitness_ = 30;    // 【本地修复】连续高 fitness 多少次才回退
    double teaser_map_voxel_, teaser_cloud_voxel_;
    double gicp_map_voxel_, gicp_cloud_voxel_;
    double fpfh_normal_radius_, fpfh_feature_radius_, noise_bound_, rotation_gnc_factor_;
    int teaser_solver_max_iter_;
    size_t teaser_inlier_threshold_;
    int gicp_solver_max_iter_, num_threads_;
    double max_correspondence_distance_, fitness_score_thre_;
    std::string registration_type_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeaserGicpNode>());
    rclcpp::shutdown();
    return 0;
}
