/*
把接收到的 icp_result（PoseWithCovarianceStamped）当作 map → odom 的静态变换发布。

【本地修复】原版的问题
----------------------
原版对**每一条** /icp_result 都调用 sendTransform()。而重定位节点是常驻的、
以雷达帧率（10Hz）逐帧独立做全局搜索 + GICP，输出天然会抖
（实测相邻帧最多跳 0.57 m，偶尔还会在两个局部最优之间翻转，z 相差 0.3 m）。

而 map→camera_init 是**整个 odom 坐标系相对 map 的位姿**，它一抖，
RViz 里所有挂在 camera_init 系的东西（最主要就是 /cloud_registered）
就跟着在 map 里每秒跳 10 次 —— 现象就是"地图乱飘 / 一偏雷达地图大偏移"。

正确做法：这个变换是"重定位给出的常量偏移"，应当**只在得到可信结果时更新一次**，
而不是追着逐帧噪声跑。于是引入 update_policy：

    first  : 只用第一条（最省事，但可能锁到抖动样本里的错误解）
    stable : 先立即用第一条把 TF 建起来，之后**只在一批位姿彼此一致时**才更新
             （默认；既不会追噪声，又能在真正重新定位后修正）
    always : 上游行为（每条都更新，会抖）
*/

#include <cmath>
#include <deque>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

class TransformPublisherNode : public rclcpp::Node
{
public:
    TransformPublisherNode()
        : Node("transform_publisher_node")
    {
        this->declare_parameter<std::string>("odom_frame_id", "camera_init");
        this->declare_parameter<std::string>("map_frame_id", "map");
        this->declare_parameter<std::string>("update_policy", "stable");
        this->declare_parameter<int>("stable_count", 5);
        this->declare_parameter<double>("stable_trans_thre", 0.25);
        this->declare_parameter<double>("stable_rot_thre_deg", 8.0);

        this->get_parameter_or<std::string>("odom_frame_id", odom_frame_id, "camera_init");
        this->get_parameter_or<std::string>("map_frame_id", map_frame_id, "map");
        this->get_parameter_or<std::string>("update_policy", update_policy, "stable");
        this->get_parameter_or<int>("stable_count", stable_count, 5);
        this->get_parameter_or<double>("stable_trans_thre", stable_trans_thre, 0.25);
        this->get_parameter_or<double>("stable_rot_thre_deg", stable_rot_thre_deg, 8.0);

        RCLCPP_INFO(this->get_logger(),
            "transform_publisher: %s -> %s, update_policy=%s (stable: %d 帧内 %.2f m / %.1f deg)",
            map_frame_id.c_str(), odom_frame_id.c_str(), update_policy.c_str(),
            stable_count, stable_trans_thre, stable_rot_thre_deg);

        subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "icp_result", 10, std::bind(&TransformPublisherNode::callback, this, std::placeholders::_1));
        broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    }

private:
    static double rotDiffDeg(const Eigen::Matrix3d &R1, const Eigen::Matrix3d &R2)
    {
        double c = ((R1.transpose() * R2).trace() - 1.0) * 0.5;
        c = std::min(1.0, std::max(-1.0, c));
        return std::acos(c) * 180.0 / M_PI;
    }

    void send(const geometry_msgs::msg::Pose &p)
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = map_frame_id;
        transform.child_frame_id = odom_frame_id;
        transform.transform.translation.x = p.position.x;
        transform.transform.translation.y = p.position.y;
        transform.transform.translation.z = p.position.z;
        transform.transform.rotation = p.orientation;
        broadcaster_->sendTransform(transform);
        ++published_;
    }

    void callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        const auto &p = msg->pose.pose;

        if (update_policy == "always")
        {
            send(p);
            return;
        }

        // 第一条：无论如何先把 TF 建起来，否则 RViz 连 map 系都没有
        if (published_ == 0)
        {
            send(p);
            last_ = p;
            RCLCPP_INFO(this->get_logger(), "已发布初始 %s -> %s", map_frame_id.c_str(), odom_frame_id.c_str());
            if (update_policy == "first") return;
        }
        else if (update_policy == "first")
        {
            return;
        }

        // stable 策略：维护最近 N 条，只有它们彼此一致时才更新 TF
        const auto &o = p.orientation;
        Eigen::Quaterniond q(o.w, o.x, o.y, o.z);
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = q.toRotationMatrix();
        T.block<3, 1>(0, 3) = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);

        hist_.push_back(T);
        while (hist_.size() > static_cast<size_t>(std::max(2, stable_count))) hist_.pop_front();
        if (hist_.size() < static_cast<size_t>(std::max(2, stable_count))) return;

        double worst_t = 0.0, worst_r = 0.0;
        for (const auto &Tk : hist_)
        {
            worst_t = std::max(worst_t, (Tk.block<3, 1>(0, 3) - T.block<3, 1>(0, 3)).norm());
            worst_r = std::max(worst_r, rotDiffDeg(Tk.block<3, 3>(0, 0), T.block<3, 3>(0, 0)));
        }
        if (worst_t > stable_trans_thre || worst_r > stable_rot_thre_deg)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "重定位输出不稳定（最近 %zu 条相差 %.2f m / %.1f deg），暂不更新 TF，"
                "避免整个 odom 系点云跟着乱跳。", hist_.size(), worst_t, worst_r);
            return;
        }

        // 稳定了：只有明显不同于已发布值才更新（避免无意义重发）
        Eigen::Quaterniond ql(last_.orientation.w, last_.orientation.x,
                              last_.orientation.y, last_.orientation.z);
        Eigen::Vector3d tl(last_.position.x, last_.position.y, last_.position.z);
        double dt = (T.block<3, 1>(0, 3) - tl).norm();
        double dr = rotDiffDeg(T.block<3, 3>(0, 0), ql.toRotationMatrix());
        if (dt < 0.05 && dr < 2.0) return;

        send(p);
        last_ = p;
        RCLCPP_WARN(this->get_logger(),
            "已用稳定的重定位结果更新 %s -> %s（变化 %.2f m / %.1f deg）",
            map_frame_id.c_str(), odom_frame_id.c_str(), dt, dr);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
    std::string odom_frame_id, map_frame_id, update_policy;
    int stable_count = 5;
    double stable_trans_thre = 0.25, stable_rot_thre_deg = 8.0;
    std::deque<Eigen::Matrix4d> hist_;
    geometry_msgs::msg::Pose last_;
    int published_ = 0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TransformPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
