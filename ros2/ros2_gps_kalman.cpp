#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/float64.hpp"

class KalmanFilter {
public:
    KalmanFilter() {
        // Initialize Kalman filter matrices (for simplicity, using identity matrices)
        // These should be fine-tuned depending on the system dynamics.
        A_ = Eigen::MatrixXd(4, 4); // State transition matrix
        H_ = Eigen::MatrixXd(2, 4); // Observation matrix
        P_ = Eigen::MatrixXd(4, 4); // Covariance matrix
        Q_ = Eigen::MatrixXd(4, 4); // Process noise covariance
        R_ = Eigen::MatrixXd(2, 2); // Measurement noise covariance
        K_ = Eigen::MatrixXd(4, 2); // Kalman gain

        // Initialize the matrices (simplified for this example)
        A_ << 1, 0, 1, 0,
              0, 1, 0, 1,
              0, 0, 1, 0,
              0, 0, 0, 1;
        H_ << 1, 0, 0, 0,
              0, 1, 0, 0;

        P_ = Eigen::MatrixXd::Identity(4, 4);
        Q_ = Eigen::MatrixXd::Identity(4, 4) * 0.1;
        R_ = Eigen::MatrixXd::Identity(2, 2) * 1.0;
    }

    void predict() {
        // Predict the next state (state update step)
        x_ = A_ * x_;  // Predicted state
        P_ = A_ * P_ * A_.transpose() + Q_; // Predicted covariance
    }

    void update(const Eigen::Vector2d& measurement) {
        // Update with new measurements
        Eigen::Vector2d y = measurement - H_ * x_; // Measurement residual
        Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_; // Residual covariance
        K_ = P_ * H_.transpose() * S.inverse(); // Kalman gain
        x_ = x_ + K_ * y; // Updated state estimate
        P_ = P_ - K_ * H_ * P_; // Updated covariance
    }

    Eigen::Vector4d getState() const {
        return x_;
    }

private:
    Eigen::MatrixXd A_, H_, P_, Q_, R_, K_;
    Eigen::Vector4d x_; // State vector (x, y, vx, vy)
};

class GPSSensorFusionNode : public rclcpp::Node {
public:
    GPSSensorFusionNode()
    : Node("gps_sensor_fusion_node") {
        // Create a Kalman Filter instance
        kalman_filter_ = std::make_shared<KalmanFilter>();

        // Subscribers to multiple GPS modules
        gps_sub_1_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "gps1/fix", 10, std::bind(&GPSSensorFusionNode::gpsCallback, this, std::placeholders::_1));

        gps_sub_2_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "gps2/fix", 10, std::bind(&GPSSensorFusionNode::gpsCallback, this, std::placeholders::_1));

        // Publisher for the fused state
        fused_state_pub_ = this->create_publisher<geometry_msgs::msg::Point>("fused_state", 10);
    }

private:
    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        // Convert GPS data into the measurement vector (x, y)
        Eigen::Vector2d measurement(msg->longitude, msg->latitude);

        // Prediction step
        kalman_filter_->predict();

        // Update step with new GPS data
        kalman_filter_->update(measurement);

        // Get the fused state estimate
        Eigen::Vector4d fused_state = kalman_filter_->getState();

        // Publish the fused position (x, y)
        geometry_msgs::msg::Point point;
        point.x = fused_state(0); // Estimated X position
        point.y = fused_state(1); // Estimated Y position

        fused_state_pub_->publish(point);
    }

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_1_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_2_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr fused_state_pub_;

    std::shared_ptr<KalmanFilter> kalman_filter_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GPSSensorFusionNode>());
    rclcpp::shutdown();
    return 0;
}
