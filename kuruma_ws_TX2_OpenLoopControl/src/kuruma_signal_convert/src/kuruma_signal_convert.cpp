#include "ros/ros.h"                    // 包含ROS的基本头文件
#include "std_msgs/Float64.h"           // 包含用于发布和订阅64位浮点数的消息类型
#include <iostream>                     // C++标准输入输出库
#include <iomanip>                      // 用于格式化输出

using namespace std;

// 自定义clamp函数，确保值在指定的最小和最大范围内
template<typename T>
T clamp(T val, T minVal, T maxVal) {
    return max(minVal, min(maxVal, val));  // 返回三者中的中间值
}

// 将车辆的转向角度转换为PWM信号的函数
double angleToPwm(double angle) {
    angle = clamp(angle, -36.0, 36.0);  // 限制角度在-36到36度之间
    return 9.0 + (angle / 36.0) * 3.0;  // 线性插值计算PWM值
}

// 将车辆的速度转换为油门PWM信号的函数
double speedToThrottlePwm(double speed) {
    speed = clamp(speed, -1000.0, 1000.0);  // 限制速度在-1000到1000 RPM之间
    double pwm = 9.0;  // 默认PWM值

    if (speed > 0.05) {
        pwm = 9.3 + (speed / 1000.0);  // 正速映射到9.3以上的PWM值
    } else if (speed < -0.05) {
        pwm = 8.7 + (speed / 1000.0);  // 负速映射到8.7以下的PWM值
    }
    else
    {
        pwm=9.0;
    }
    

    return pwm;  // 返回计算得到的PWM值
}

// 接收转向控制消息的回调函数
void steeringControlCallback(const std_msgs::Float64::ConstPtr& msg, ros::Publisher angle_pub) {
    double anglePwmOutput = angleToPwm(msg->data);  // 计算转向PWM输出值
    std_msgs::Float64 output;  // 创建输出消息
    output.data = anglePwmOutput;  // 设置输出数据
    angle_pub.publish(output);  // 发布输出消息
    ROS_INFO("ANGLE_PWM: %.2f%%", anglePwmOutput);  // 打印信息到ROS日志
}

// 接收电机速度控制消息的回调函数
void motorSpeedControlCallback(const std_msgs::Float64::ConstPtr& msg, ros::Publisher speed_pub) {
    double speedPwmOutput = speedToThrottlePwm(msg->data);  // 计算电机PWM输出值
    std_msgs::Float64 output;  // 创建输出消息
    output.data = speedPwmOutput;  // 设置输出数据
    speed_pub.publish(output);  // 发布输出消息
    ROS_INFO("SPEED_PWM: %.2f%%", speedPwmOutput);  // 打印信息到ROS日志
}

// 主函数
int main(int argc, char **argv) {
    ros::init(argc, argv, "vehicle_control");  // 初始化ROS节点，节点名为"vehicle_control"
    ros::NodeHandle nh;  // 创建节点句柄

    // 创建两个发布者，用于发布角度和速度的PWM信号
    ros::Publisher angle_pub = nh.advertise<std_msgs::Float64>("angle_pwm_output", 10);
    ros::Publisher speed_pub = nh.advertise<std_msgs::Float64>("speed_pwm_output", 10);

    // 创建两个订阅者，用于订阅角度和速度的控制指令
    ros::Subscriber steering_sub = nh.subscribe<std_msgs::Float64>("/vehicle/steering_control", 10, 
        boost::bind(steeringControlCallback, _1, angle_pub));
    ros::Subscriber motor_speed_sub = nh.subscribe<std_msgs::Float64>("/vehicle/motor_speed_control", 10,
        boost::bind(motorSpeedControlCallback, _1, speed_pub));

    ros::spin();  // 进入ROS事件循环，等待消息到来

    return 0;  // 返回0，正常退出程序
}
