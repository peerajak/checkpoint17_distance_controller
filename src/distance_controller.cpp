#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <chrono>
#include <memory>
#include <tuple>
#include <vector>
#include <cmath>
#include <nav_msgs/msg/odometry.hpp>
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include <Eigen/Dense>

#define pi 3.1417
 
using Eigen::MatrixXd;
using namespace std::chrono_literals;
using std::placeholders::_1;
//std::chrono::nanoseconds fifty_milisec = 5000000;
class DistanceController : public rclcpp::Node {
public:

  DistanceController() : Node("distance_controller") {
    // ---- 1. publisher to cmd_vel
      publisher_1_twist =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    //------- 2. timer_1 related -----------//
    timer_1_ = this->create_wall_timer(
        1000ms, std::bind(&DistanceController::timer1_callback, this));//nothing about 1 sec
    //------- 3. Odom related  -----------//
    callback_group_3_odom = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions options3_odom;
    options3_odom.callback_group = callback_group_3_odom;
    subscription_3_odom = this->create_subscription<nav_msgs::msg::Odometry>(
        "/rosbot_xl_base_controller/odom", 10,
        std::bind(&DistanceController::odom_callback, this,
                  std::placeholders::_1), options3_odom);

    //ref_points.push_back(std::make_tuple(0,0));
    double cur_ref_phi = 0;
    double cur_ref_x = 0;
    double cur_ref_y = 0;
    //RCLCPP_DEBUG(this->get_logger(), "initialize ref_point (x,y) = %f,%f ", cur_ref_x, cur_ref_y);
    for (auto iter = waypoints.begin();
     iter != waypoints.end(); iter++){
        std::tuple<double,double,double> i_th = *iter;
        double dphi = std::get<0>(i_th);
        double dx = std::get<1>(i_th);
        double dy = std::get<2>(i_th);
        cur_ref_phi -= dphi;
        cur_ref_x += dx;
        cur_ref_y += dy;
        ref_points.push_back(std::make_tuple(cur_ref_phi,cur_ref_x,cur_ref_y));
        RCLCPP_INFO(this->get_logger(), "initialize ref_point phi %f, (x,y) = %f,%f ", cur_ref_phi, cur_ref_x, cur_ref_y);
    }
  
  }

private:
    //int max_iter = 20;

  void timer1_callback() {
    RCLCPP_DEBUG(this->get_logger(), "Timer Callback ");
    timer_1_->cancel();
    //assert(false);
    std::thread{std::bind(&DistanceController::execute, this)}.detach();
  }
  void execute() {
    auto message = std_msgs::msg::Float32MultiArray();
    double error_tolerance = 0.01; 
    //rclcpp::Rate loop_rate(0.01);
    while(!ref_points.empty()){
        std::tuple<double,double,double> it2 = ref_points.front();
         RCLCPP_INFO(this->get_logger(), "ref_point (x,y) = %f,%f ", std::get<1>(it2), std::get<2>(it2));
        double dphi = std::get<0>(it2); 
        double dx = std::get<1>(it2);
        double dy = std::get<2>(it2);
        go_to(dx,dy, dphi, error_tolerance);
        sleep(1);
        timer1_counter++;
        waypoints.pop_front();
        ref_points.pop_front(); 
    }
    RCLCPP_DEBUG(get_logger(), "No more waypoints");  
    rclcpp::shutdown();
  }

  MatrixXd velocity2twist(double dphi, double dx, double dy){
    RCLCPP_DEBUG(get_logger(), "velocity2twist current_yaw_rad_ %f",current_yaw_rad_);  
    MatrixXd R(3,3);
    R(0,0) = 1; R(0,1) = 0; R(0,2) = 0;
    R(1,0) = 0; R(1,1) = cos(current_yaw_rad_); R(1,2) =  sin(current_yaw_rad_); 
    R(2,0) = 0; R(2,1) = -sin(current_yaw_rad_); R(2,2) =  cos(current_yaw_rad_);        
    MatrixXd v(3,1);
    v(0,0) = dphi;
    v(1,0) = dx;
    v(2,0) = dy;

    MatrixXd twist = R*v; 
    return twist;
  }
 std::vector<float> twist2wheels(MatrixXd twist){
    std::vector<float> u_vector;

    MatrixXd H(4,3);
    H(0,0) = (-l-w)/r; H(0,1) =  1/r; H(0,2) = -1/r;
    H(1,0) = (l+w)/r;  H(1,1) =  1/r; H(1,2) =  1/r;
    H(2,0) = (l+w)/r;  H(2,1) =  1/r; H(2,2) = -1/r;
    H(3,0) = (-l-w)/r; H(3,1) =  1/r; H(3,2) =  1/r;
    MatrixXd u = H * twist;
    u_vector.push_back(u(0,0));
    u_vector.push_back(u(1,0));
    u_vector.push_back(u(2,0));
    u_vector.push_back(u(3,0));
    return u_vector;
  
  }
  std::vector<float> twist2wheels(double wz, double vx, double vy){
    std::vector<float> u_vector;
    MatrixXd twist(3,1);
    twist(0,0) = wz;
    twist(1,0) = vx;
    twist(2,0) = vy;
    MatrixXd H(4,3);
    H(0,0) = (-l-w)/r; H(0,1) =  1/r; H(0,2) = -1/r;
    H(1,0) = (l+w)/r;  H(1,1) =  1/r; H(1,2) =  1/r;
    H(2,0) = (l+w)/r;  H(2,1) =  1/r; H(2,2) = -1/r;
    H(3,0) = (-l-w)/r; H(3,1) =  1/r; H(3,2) =  1/r;
    MatrixXd u = H * twist;
    u_vector.push_back(u(0,0));
    u_vector.push_back(u(1,0));
    u_vector.push_back(u(2,0));
    u_vector.push_back(u(3,0));
    return u_vector;
  
  }

  double normalize_angle(double angle_radian){
    double sign_multiplier, normalized_angle;

    if(angle_radian >= -pi && angle_radian <= pi)
        return angle_radian;

    if(angle_radian < -pi)
        sign_multiplier = 1;

    if(angle_radian > pi)
        sign_multiplier = -1;

    normalized_angle = angle_radian;
    for (int i=0; i<20; i++){
        normalized_angle += sign_multiplier*2*pi;
        if( normalized_angle >= -pi && normalized_angle <= pi)
            return normalized_angle;
    }
    return -100;  
  }

  void go_to(double x_goal, double y_goal, double theta_goal_radian, double tolerance){
    auto message = std_msgs::msg::Float32MultiArray();
    double rho = std::numeric_limits<double>::max();
    double theta_goal = normalize_angle(theta_goal_radian);
    RCLCPP_INFO(get_logger(), "x_goal %f, y_goal %f,theta_goal %f",x_goal,y_goal,theta_goal);  
    int hz_inverse_us = 10000;//10 Hz = 0.01 sec = 10000 microsec 
    while(rho>tolerance){
        double delta_x = x_goal - current_pos_.x;
        double delta_y = y_goal - current_pos_.y;
        rho = sqrt(delta_x*delta_x + delta_y*delta_y);
        double alpha = 0;
        double beta  = normalize_angle(-theta_goal - (alpha + current_yaw_rad_));
        double w = k_alpha*alpha + k_beta*beta;
        RCLCPP_INFO(get_logger(), "rho %f current_yaw_rad_ %f, beta %f ",rho,current_yaw_rad_,beta);  
        // positive w is ccw ,negative w is cw
        RCLCPP_INFO(get_logger(), "theta_goal %f, w %f, delta_x %f, delta_y %f",theta_goal,w,delta_x, delta_y); 
        ling.angular.z = w;
        ling.linear.x = delta_x;
        ling.linear.y = delta_y;
        this->move_robot(ling);
        usleep(hz_inverse_us);
    }

  }



MatrixXd KinematicLeastSquareNormalEq(MatrixXd & u){
    MatrixXd H(4,3);
    H(0,0) = (-l-w)/r; H(0,1) =  1/r; H(0,2) = -1/r;
    H(1,0) = (l+w)/r;  H(1,1) =  1/r; H(1,2) =  1/r;
    H(2,0) = (l+w)/r;  H(2,1) =  1/r; H(2,2) = -1/r;
    H(3,0) = (-l-w)/r; H(3,1) =  1/r; H(3,2) =  1/r;
    MatrixXd HTH_inv = (H.transpose()* H).inverse();
    MatrixXd HTHinv_least_square  = HTH_inv * H.transpose();
    MatrixXd twist = HTHinv_least_square * u;
    return twist;
}
void move_robot(geometry_msgs::msg::Twist &msg) {
    publisher_1_twist->publish(msg);
}
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_1_twist;
  geometry_msgs::msg::Twist ling;
  //------- 3. Odom related  Functions -----------//  
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {

    current_pos_ = msg->pose.pose.position;
    current_angle_ = msg->pose.pose.orientation;
    current_yaw_rad_ = yaw_theta_from_quaternion(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

    RCLCPP_INFO(this->get_logger(), "current pos=['%f','%f','%f'",
                 current_pos_.x, current_pos_.y, current_yaw_rad_);
  }
  double yaw_theta_from_quaternion(double qx, double qy, double qz, double qw) {
    double roll_rad, pitch_rad, yaw_rad;
    tf2::Quaternion odom_quat(qx, qy, qz, qw);
    tf2::Matrix3x3 matrix_tf(odom_quat);
    matrix_tf.getRPY(roll_rad, pitch_rad, yaw_rad);
    return yaw_rad; // In radian
  }


  //------- 3. Odom related private variables  -----------//
  rclcpp::CallbackGroup::SharedPtr callback_group_3_odom;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_3_odom;
  geometry_msgs::msg::Point current_pos_;
  geometry_msgs::msg::Quaternion current_angle_;
  double current_yaw_rad_;

  //--------  Kinematic related private variables --------// 
  double l = 0.500/2;
  double r = 0.254/2;
  double w = 0.548/2;
  //------- feedback loop private varibales -------//
  double k_rho = 0.3;   
  double k_alpha = 0.8;
  double k_beta = 1;// -0.15;
  std::list<std::tuple<double, double, double>> waypoints {std::make_tuple(0,1,-1),std::make_tuple(0,1,1),
                                std::make_tuple(0,1,1),std::make_tuple(1.5708, 1, -1),std::make_tuple(-3.1415, -1, -1),
                                std::make_tuple(0.0, -1, 1),std::make_tuple(0.0, -1, 1),std::make_tuple(0.0, -1, -1)};

  std::list<std::tuple<double,double,double>> ref_points;
 
  rclcpp::TimerBase::SharedPtr timer_1_;
  int timer1_counter;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;

};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto eight_trajectory_subscriber = std::make_shared<DistanceController>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(eight_trajectory_subscriber);
  executor.spin();



  //rclcpp::spin(std::make_shared<DistanceController>());
  rclcpp::shutdown();
  return 0;
}