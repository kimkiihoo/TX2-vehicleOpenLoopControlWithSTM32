#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <cmath>
#include "Uwb_Location/uwb.h" // 新增，包含UWB消息类型




// 轨迹点结构体，用于存储目标轨迹的 x 和 y 坐标
struct Point {
    double x;
    double y;
};

// PID 控制器类
class PID {
public:
    // 构造函数，初始化 PID 控制器的参数：比例 (kp)、积分 (ki)、微分 (kd) 系数
    PID(double kp, double ki, double kd)
        : kp_(kp), ki_(ki), kd_(kd), prev_error_(0.0), integral_(0.0) {}

    // 计算控制量的函数，基于目标值、当前值以及时间步长 (dt)
    double compute(double target, double current, double dt) {
        // 计算误差
        double error = target - current;
        // 累加积分项
        integral_ += error * dt;
        // 计算误差的微分项
        double derivative = (error - prev_error_) / dt;
        // 更新上一次的误差
        prev_error_ = error;
        // 返回 PID 控制器的输出值
        return kp_ * error + ki_ * integral_ + kd_ * derivative;
    }

private:
    // PID 参数
    double kp_, ki_, kd_;
    // 前一次的误差值和积分项
    double prev_error_;
    double integral_;
};

int index_xie = 0;
// 车辆控制器类，包含车辆状态订阅和控制计算
class VehicleController {
public:
    // 构造函数，初始化 ROS 订阅者、发布者，并设置 PID 控制器参数
    VehicleController(ros::NodeHandle& nh)
        : speed_pid_(1.0, 0.1, 0.05), // 初始化速度控制的PID控制器
          steer_pid_(1.0, 0.1, 0.05), // 初始化转向控制的PID控制器
          target_speed_(5.0),         // 设置目标速度
          current_speed_(0.0),        // 初始化当前速度
          current_steering_angle_(0.0), // 初始化当前转向角
          current_x_(0.0),            // 初始化当前车辆x坐标
          current_y_(0.0),            // 初始化当前车辆y坐标
          current_yaw_(0.0),          // 初始化航向角
          dt_(0.1) {                  // 设置时间步长

        // 订阅 UWB 位置信息
        xy_sub_ = nh.subscribe("/uwb/data", 10, &VehicleController::xyCallback, this);
        // 订阅车辆的电机速度信息
        speed_sub_ = nh.subscribe("/vehicle/speed", 10, &VehicleController::speedCallback, this);
        // 订阅车辆的转向角度信息
        steering_sub_ = nh.subscribe("/vehicle/steering_angle", 10, &VehicleController::steeringCallback, this);

        // 发布电机速度控制信号
        motor_pub_ = nh.advertise<std_msgs::Float64>("/vehicle/motor_speed_control", 10);
        // 发布转向角度控制信号
        steering_pub_ = nh.advertise<std_msgs::Float64>("/vehicle/steering_control", 10);
    }

    // 控制更新函数，每次迭代调用，传入目标轨迹点序列
    void update(const std::vector<Point>& trajectory) {
        if (trajectory.empty()) return; // 如果轨迹为空则返回

        // 查找离当前车辆位置最近的目标轨迹点
        Point target_point = findClosestPoint(current_x_, current_y_, trajectory);

        // 计算期望的航向角（车辆需要朝向的方向）
        double target_yaw = 10;
        if(index_xie <= 60){
        index_xie = index_xie + 1;
        target_yaw = 0;
        }
        else if (index_xie > 60 && index_xie <= 100) {
        index_xie = index_xie + 1;
        target_yaw = 12;
        }
        else {
        index_xie = index_xie + 1;
        target_yaw = 0;
        }


        // 计算当前航向角和期望航向角之间的误差
        // double yaw_error = target_yaw - current_steering_angle_;

        // 计算当前航向角与期望航向角度，用的是tag1，tag2算出来的
        double yaw_error = target_yaw - current_yaw_;
        

        // 使用 PID 控制器计算转向控制量
        // double steer_control = steer_pid_.compute(0.0, yaw_error, dt_);
        //double steer_control = 0.0;
        // 使用 PID 控制器计算转向控制量
        double steer_control = steer_pid_.compute(target_yaw, current_yaw_, dt_);


        // 使用 PID 控制器计算速度控制量
        // double speed_control = speed_pid_.compute(target_speed_, current_speed_, dt_);
        double speed_control = 10;
        if(index_xie <= 50){
            speed_control = 0;
        }
        else if (index_xie > 48 && index_xie <= 100) {
            speed_control = 400;
        }
        else {
            speed_control = 0;
        }


        // 发布电机速度控制信号
        std_msgs::Float64 speed_msg;
        speed_msg.data = speed_control;
        motor_pub_.publish(speed_msg);

        // 发布转向角度控制信号
        std_msgs::Float64 steer_msg;
        steer_msg.data = steer_control;
        steering_pub_.publish(steer_msg);
    }

private:
    // 最近轨迹点的查找函数，考虑到x方向递增的密集轨迹
    Point findClosestPoint(double x, double y, const std::vector<Point>& trajectory) {
        double min_dist = std::numeric_limits<double>::max(); // 初始化最小距离为无穷大
        Point closest_point;

        // 遍历轨迹点，找到距离当前车辆位置最近的点
        for (const auto& point : trajectory) {
            // 如果当前点的x坐标超过了车辆的x位置，并且点间距较小，可以提前退出
            if (point.x > x && std::abs(point.x - x) > 0.2) {
                break;
            }
            // 计算当前轨迹点和车辆位置的距离
            double dist = std::hypot(point.x - x, point.y - y); // 使用欧几里得距离公式
            if (dist < min_dist) { // 更新最近的点
                min_dist = dist;
                closest_point = point;
            }
        }

        return closest_point; // 返回最近的轨迹点
    }

    // UWB 位置信息回调函数，更新当前车辆的 x 和 y 位置
    void xyCallback(const Uwb_Location::uwb::ConstPtr& msg) {
        if (!received_first_tag_) {
            // 存储第一个tag的x,y
            first_tag_x_ = msg->x;
            first_tag_y_ = msg->y;
            received_first_tag_ = true;
        } else {
            // 存储第二个tag的x,y
            double second_tag_x = msg->x;
            double second_tag_y = msg->y;

            // 根据两个tag计算横摆角
            current_yaw_ = std::atan2(second_tag_y - first_tag_y_, second_tag_x - first_tag_x_);

            // 假设车辆位置在tag之间
            current_x_ = (first_tag_x_ + second_tag_x) / 2.0;
            current_y_ = (first_tag_y_ + second_tag_y) / 2.0;

            received_first_tag_ = false; 
        }
    }

    // 电机速度回调函数，更新当前车辆的速度
    void speedCallback(const std_msgs::Float64::ConstPtr& msg) {
        current_speed_ = msg->data;
    }

    // 转向角回调函数，更新当前车辆的转向角
    void steeringCallback(const std_msgs::Float64::ConstPtr& msg) {
        current_steering_angle_ = msg->data;
    }

    // PID 控制器实例
    PID speed_pid_;
    PID steer_pid_;
    // 目标速度和当前状态
    double target_speed_;
    double current_speed_;
    double current_steering_angle_;
    double current_x_;
    double current_y_;
    double current_yaw_;
    double dt_;
    double first_tag_x_, first_tag_y_;
    bool received_first_tag_;

    // ROS 订阅者和发布者
    ros::Subscriber xy_sub_;
    ros::Subscriber speed_sub_;
    ros::Subscriber steering_sub_;
    ros::Publisher motor_pub_;
    ros::Publisher steering_pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "vehicle_controller"); // 初始化ROS节点
    ros::NodeHandle nh; // 创建节点句柄

    std::vector<Point> trajectory;
    double y = 0.0; // 轨迹在 y 轴上保持为0
    for (double x = 0.0; x <= 100.0; x += 0.2) { // 生成0.2m间隔的点，长度为100m的轨迹
        trajectory.push_back({x, y});
    }

    // 创建车辆控制器实例
    VehicleController controller(nh);

    ros::Rate loop_rate(10);  // 设置循环频率为 10Hz

    // 主循环，持续执行控制更新
    while (ros::ok()) {
        controller.update(trajectory); // 更新控制器
        ros::spinOnce(); // 处理回调函数
        loop_rate.sleep(); // 保持10Hz循环
    }

    return 0; // 程序结束
}
