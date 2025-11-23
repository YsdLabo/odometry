#include<rclcpp/rclcpp.hpp>
#include<sensor_msgs/msg/joint_state.hpp>
#include<nav_msgs/msg/odometry.hpp>
#include<tf2_ros/transform_broadcaster.h>
#include<tf2/LinearMath/Quaternion.h>
#include<tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class OdometryCalculator : public rclcpp::Node
{
public:
    OdometryCalculator()
      : Node("odometry_node"), first_time_(true)
    {
        get_remote_parameter_with_wait<double>("diff_robot_node", "wheel_radius", wheel_radius_, std::chrono::seconds(10));
        get_remote_parameter_with_wait<double>("diff_robot_node", "wheel_separation", wheel_separation_, std::chrono::seconds(10));
    
        sub_joint_states_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 
            10,
            std::bind(&OdometryCalculator::callback_joint_states, this, std::placeholders::_1)
        );
        
        pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "odom", 
            1
        );
        
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        
        RCLCPP_INFO(this->get_logger(), "get_parameter : wheel_radius = %f", wheel_radius_);
        RCLCPP_INFO(this->get_logger(), "get_parameter : wheel_separation = %f", wheel_separation_);
        RCLCPP_INFO(this->get_logger(), "OdomCalculator Node Initialized.");
    }
private:
    rclcpp::Time cur_time_;
    double cur_x_;
    double cur_y_;
    double cur_th_;
    double cur_v_x_;
    double cur_v_th_;
    tf2::Quaternion cur_q_;
    double cur_pos_left_;
    double cur_pos_right_;

    rclcpp::Time last_time_;
    double last_pos_right_;
    double last_pos_left_;
    
    double wheel_radius_;
    double wheel_separation_;

    bool first_time_;
    
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    void callback_joint_states(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if(msg->position.size() != 2 or msg->name.size() != 2) return;
        
        size_t left_idx = (msg->name[0] == "wheel_left_joint") ? 0 : 1;
        size_t right_idx = (msg->name[1] == "wheel_right_joint") ? 1 : 0;
        if(msg->name[left_idx] != "wheel_left_joint" or msg->name[right_idx] != "wheel_right_joint") return;

        cur_pos_left_ = msg->position[left_idx];
        cur_pos_right_ = msg->position[right_idx];
        cur_time_ = rclcpp::Time(msg->header.stamp);
        
        odometry_update();

        publish_odom();
        publish_tf();
    }
    
    void odometry_update()
    {
        if(first_time_)
        {
            cur_x_ = 0.0;
            cur_y_ = 0.0;
            cur_th_ = 0.0;
            cur_v_x_ = 0.0;
            cur_v_th_ = 0.0;
            cur_q_.setRPY(0.0, 0.0, 0.0);
            cur_q_.normalize();
            last_pos_left_ = cur_pos_left_;
            last_pos_right_ = cur_pos_right_;
            last_time_ = cur_time_;
            first_time_ = false;
            return;
        }
        
        // 車輪の微小移動距離 (m)
        double delta_Ll = wheel_radius_ * (cur_pos_left_ - last_pos_left_);
        double delta_Lr = wheel_radius_ * (cur_pos_right_ - last_pos_right_);
        
        // 時間差分(s)
        double dt = (cur_time_ - last_time_).seconds();
        
        last_pos_left_ = cur_pos_left_;
        last_pos_right_ = cur_pos_right_;
        last_time_ = cur_time_;
        
        // ロボット中心の微小移動距離(m)
        double delta_L = (delta_Lr + delta_Ll) / 2.0;
        
        // 座標更新
        double dx, dy, dth;
        if(std::fabs(delta_Ll - delta_Lr) < 0.000001)
        {
            dx = delta_L * std::cos(cur_th_);
            dy = delta_L * std::sin(cur_th_);
            dth = 0.0;
        }
        else
        {
            dth = (delta_Lr - delta_Ll) / wheel_separation_;
            double R = delta_L / dth;
            double dth_half = dth * 0.5;
            double d_Lp = 2.0 * R * std::sin(dth_half);
            double th_h = cur_th_ + dth_half;
            dx = d_Lp * std::cos(th_h);
            dy = d_Lp * std::sin(th_h); 
        }
        
        cur_x_ += dx;
        cur_y_ += dy;
        cur_th_ = normalize_angle(cur_th_ + dth);
        cur_q_.setRPY(0, 0, cur_th_);
        cur_q_.normalize();
        
        if(dt < 0.000001) {
            cur_v_x_ = 0.0;
            cur_v_th_ = 0.0;
        }
        else {
            cur_v_x_ = delta_L / dt;
            cur_v_th_ = dth / dt;
        }
    }
    
    void publish_odom()
    {
        auto odom = nav_msgs::msg::Odometry();
        odom.header.stamp = cur_time_;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";
        
        odom.pose.pose.position.x = cur_x_;
        odom.pose.pose.position.y = cur_y_;
        odom.pose.pose.orientation = tf2::toMsg(cur_q_);
        
        odom.pose.covariance[0] = 0.01;
        odom.pose.covariance[7] = 0.01;
        odom.pose.covariance[14] = FLT_MAX;
        odom.pose.covariance[21]= FLT_MAX;
        odom.pose.covariance[28] = FLT_MAX;
        odom.pose.covariance[35] = 0.1;
        
        odom.twist.twist.linear.x = cur_v_x_;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = cur_v_th_;
        
        odom.twist.covariance[0] = 0.01;
        odom.twist.covariance[35] = 0.1;
        
        pub_odom_->publish(odom);
    }
    
    void publish_tf()
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = cur_time_;
        t.header.frame_id = "odom";
        t.child_frame_id = "base_footprint";
        t.transform.translation.x = cur_x_;
        t.transform.translation.y = cur_y_;
        t.transform.rotation = tf2::toMsg(cur_q_);
        
        tf_broadcaster_->sendTransform(t);
    }
    
    double normalize_angle(double angle)
    {
        while(angle > M_PI) angle -= 2.0 * M_PI;
        while(angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
    
    template <typename T>
    bool get_remote_parameter_with_wait(
        const std::string& node_name,
        const std::string& param_name,
        T& output_value,
        std::chrono::seconds timeout)
    {
        auto param_client = std::make_shared<rclcpp::SyncParametersClient>(this, node_name);
        RCLCPP_INFO(this->get_logger(), "Waiting for service %s...", node_name.c_str());
        
        if(!param_client->wait_for_service(timeout)) {
            RCLCPP_ERROR(this->get_logger(), "Service for %s not available.", node_name.c_str());
            return false;
        }
        
        auto params = param_client->get_parameters({param_name});
        
        if(params.empty()) {
            RCLCPP_WARN(this->get_logger(), "Parameter '%s' not found on %s.", param_name.c_str(), node_name.c_str());
            return false;
        }
        
        const auto& param = params[0];
        if(param.get_type() == rclcpp::ParameterValue(output_value).get_type()) {
            output_value = param.get_value<T>();
            RCLCPP_INFO(this->get_logger(), "Parameter '%s' retrieved successfully.", param_name.c_str());
            return true;
        }
        else {
            RCLCPP_ERROR(this->get_logger(), "Parameter '%s' type mismatch.", param_name.c_str());
            return false;
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryCalculator>());
    rclcpp::shutdown();
    return 0;
}


