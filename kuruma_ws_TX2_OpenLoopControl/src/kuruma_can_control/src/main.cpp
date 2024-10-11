// kuruma_can_control.cpp

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include "kuruma_can_control/controlcan.h"
#include <sstream>
#include <boost/bind.hpp>
#include <mutex>

// 全局变量
VCI_BOARD_INFO pInfo;
VCI_BOARD_INFO pInfo1[50];
int num = 0;
int running = 1;             // 控制接收线程运行
pthread_mutex_t can_mutex;   // 用于 CAN 操作的互斥锁

ros::Publisher can_pub;
ros::Publisher speed_pub;    // 发布速度的 ROS 话题
ros::Publisher yaw_pub;      // 发布 yaw（转向角）的 ROS 话题

// 线程安全的互斥量，用于访问 combined_message
std::mutex data_mutex;

// 定义结构体用于存储角度和速度数据
struct CombinedMessage {
    float angle;          // 角度，浮点数
    float speed;          // 速度，浮点数
    bool angle_ready;     // 标志，表示角度数据是否已准备好
    bool speed_ready;     // 标志，表示速度数据是否已准备好

    // 构造函数，初始化数据
    CombinedMessage() : angle(0.0), speed(0.0), angle_ready(false), speed_ready(false) {}
};

// 全局实例化 CombinedMessage
CombinedMessage combined_message;

// 角度数据的回调函数
void angleCallback(const std_msgs::Float64::ConstPtr& msg, CombinedMessage& combined_msg) {
    std::lock_guard<std::mutex> lock(data_mutex);
    combined_msg.angle = msg->data;
    combined_msg.angle_ready = true;
    ROS_INFO("Received angle data: %f", combined_msg.angle);
}

// 速度数据的回调函数
void speedCallback(const std_msgs::Float64::ConstPtr& msg, CombinedMessage& combined_msg) {
    std::lock_guard<std::mutex> lock(data_mutex);
    combined_msg.speed = msg->data;
    combined_msg.speed_ready = true;
    ROS_INFO("Received speed data: %f", combined_msg.speed);
}

// 接收线程函数
void *receive_func(void *param) {
    VCI_CAN_OBJ rec[3000]; // 接收缓冲区
    int reclen = 0;
    int i, j;

    while (running) {
        pthread_mutex_lock(&can_mutex);
        reclen = VCI_Receive(VCI_USBCAN2, 0, 0, rec, 3000, 100);
        pthread_mutex_unlock(&can_mutex);

        if (reclen > 0) {
            for (j = 0; j < reclen; j++) {
                // 打印接收到的消息信息
                ROS_INFO("Message received:");
                ROS_INFO("  CAN1 RX ID: 0x%08X", rec[j].ID);
                ROS_INFO("  Frame format: %s", rec[j].ExternFlag ? "Extended frame" : "Standard frame");

                std::stringstream data_ss;
                data_ss << "  Data: ";
                for (i = 0; i < rec[j].DataLen; i++) {
                    data_ss << "0x" << std::hex << (int)rec[j].Data[i] << " ";
                }
                ROS_INFO("%s", data_ss.str().c_str());
                ROS_INFO("  Timestamp: 0x%08X", rec[j].TimeStamp);

                // 处理频率数据 (ID = 0x433)
                if (rec[j].ID == 0x433) {
                    if (rec[j].DataLen >= 4) { // 确保数据长度足够
                        // 从 Data[2] 和 Data[3] 提取频率数据
                        uint16_t frequency_raw = (rec[j].Data[2] << 8) | rec[j].Data[3];
                        float frequency_value = static_cast<float>(frequency_raw); // 根据需要进行缩放

                        // 创建并发布频率消息
                        std_msgs::Float64 frequency_msg;
                        frequency_msg.data = frequency_value;
                        speed_pub.publish(frequency_msg);

                        ROS_INFO("Published frequency data: %f to /vehicle/speed", frequency_value);
                    } else {
                        ROS_WARN("Received 0x433 message with insufficient data length: %d", rec[j].DataLen);
                    }
                }

                // 处理转向角数据 (ID = 0x503)
                if (rec[j].ID == 0x503) {
                    if (rec[j].DataLen >= 4) { // 确保数据长度足够
                        // 使用联合体将四个字节转换为 float
                        union {
                            float f;
                            uint32_t i;
                        } float_union;

                        // 假设发送端使用大端字节序 (先高字节后低字节)
                        float_union.i = ((uint32_t)rec[j].Data[0] << 24) |
                                        ((uint32_t)rec[j].Data[1] << 16) |
                                        ((uint32_t)rec[j].Data[2] << 8) |
                                        ((uint32_t)rec[j].Data[3]);

                        float yaw_value = float_union.f;

                        // 创建并发布转向角消息
                        std_msgs::Float64 yaw_msg;
                        yaw_msg.data = yaw_value;
                        yaw_pub.publish(yaw_msg);

                        ROS_INFO("Published yaw data: %f degrees to /vehicle/yaw", yaw_value);
                    } else {
                        ROS_WARN("Received 0x503 message with insufficient data length: %d", rec[j].DataLen);
                    }
                }

                // 可选：将接收到的消息发布到 ROS 话题
                std_msgs::String msg;
                std::stringstream ss;
                ss << "ID: 0x" << std::hex << rec[j].ID << " Data: ";
                for (i = 0; i < rec[j].DataLen; i++) {
                    ss << "0x" << std::hex << (int)rec[j].Data[i] << " ";
                }
                msg.data = ss.str();
                can_pub.publish(msg);
            }
        }
        usleep(100000); // 100ms
    }

    ROS_INFO("Receiving thread exiting.");
    pthread_exit(NULL);
}

int main(int argc, char **argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "kuruma_can_control");
    ros::NodeHandle nh;

    // 创建 ROS 发布者
    can_pub = nh.advertise<std_msgs::String>("can_received_messages", 1000);
    speed_pub = nh.advertise<std_msgs::Float64>("/vehicle/speed", 10);
    yaw_pub = nh.advertise<std_msgs::Float64>("/vehicle/yaw", 10); // 新增的 yaw 发布者

    // 订阅角度和速度 PWM 控制消息
    ros::Subscriber angle_sub = nh.subscribe<std_msgs::Float64>("angle_pwm_output", 10, boost::bind(angleCallback, _1, boost::ref(combined_message)));
    ros::Subscriber speed_sub = nh.subscribe<std_msgs::Float64>("speed_pwm_output", 10, boost::bind(speedCallback, _1, boost::ref(combined_message)));

    // 启动一个异步 spinner 以处理回调
    ros::AsyncSpinner spinner(2); // 使用 2 个线程处理回调
    spinner.start();

    ROS_INFO("Program started, preparing to initialize CAN device.");

    // 初始化互斥锁
    if (pthread_mutex_init(&can_mutex, NULL) != 0) {
        ROS_ERROR("Failed to initialize mutex.");
        return -1;
    }

    // 在打开设备之前查找可用的 USB-CAN 设备
    num = VCI_FindUsbDevice2(pInfo1);
    ROS_INFO("Found %d USBCAN devices.", num);

    if (num < 1)
    {
        ROS_ERROR("No USBCAN device detected, please check the connection.");
        pthread_mutex_destroy(&can_mutex);
        return -1;
    }

    // 打印每个设备的信息
    for(int i = 0; i < num; i++)
    {
        ROS_INFO("Device %d information:", i);
        ROS_INFO("  Serial Number: %.20s", pInfo1[i].str_Serial_Num);
        ROS_INFO("  Hardware Type: %.10s", pInfo1[i].str_hw_Type);
        ROS_INFO("  Firmware Version: V%d.%d%d",
                 (pInfo1[i].fw_Version & 0xF00) >> 8,
                 (pInfo1[i].fw_Version & 0xF0) >> 4,
                 pInfo1[i].fw_Version & 0xF);
    }

    // 打开设备
    if(VCI_OpenDevice(VCI_USBCAN2, 0, 0) != 1)
    {
        ROS_ERROR("Failed to open device, please check if the device is properly connected.");
        pthread_mutex_destroy(&can_mutex);
        return -1;
    }
    ROS_INFO("Device opened successfully.");

    // 读取设备信息
    memset(&pInfo, 0, sizeof(VCI_BOARD_INFO)); // 初始化 pInfo
    if(VCI_ReadBoardInfo(VCI_USBCAN2, 0, &pInfo) != 1)
    {
        ROS_ERROR("Failed to read device information.");
        VCI_CloseDevice(VCI_USBCAN2, 0);
        pthread_mutex_destroy(&can_mutex);
        return -1;
    }

    ROS_INFO("Device information read successfully.");
    ROS_INFO("  Serial Number: %.20s", pInfo.str_Serial_Num);
    ROS_INFO("  Hardware Type: %.10s", pInfo.str_hw_Type);
    ROS_INFO("  Firmware Version: V%d.%d%d",
             (pInfo.fw_Version & 0xF00) >> 8,
             (pInfo.fw_Version & 0xF0) >> 4,
             pInfo.fw_Version & 0xF);

    // 初始化 CAN 参数
    VCI_INIT_CONFIG config;
    memset(&config, 0, sizeof(VCI_INIT_CONFIG)); // 初始化 config
    config.AccCode = 0;
    config.AccMask = 0xFFFFFFFF;
    config.Filter = 1; // 接收所有帧
    config.Timing0 = 0x00; // 波特率 500 Kbps (0x00 0x1C)
    config.Timing1 = 0x1C;
    config.Mode = 0; // 正常模式

    if(VCI_InitCAN(VCI_USBCAN2, 0, 0, &config) != 1)
    {
        ROS_ERROR("Failed to initialize CAN1.");
        VCI_CloseDevice(VCI_USBCAN2, 0);
        pthread_mutex_destroy(&can_mutex);
        return -1;
    }
    ROS_INFO("CAN1 initialized successfully.");

    // 启动 CAN1
    if(VCI_StartCAN(VCI_USBCAN2, 0, 0) != 1)
    {
        ROS_ERROR("Failed to start CAN1.");
        VCI_CloseDevice(VCI_USBCAN2, 0);
        pthread_mutex_destroy(&can_mutex);
        return -1;
    }
    ROS_INFO("CAN1 started successfully.");

    // 创建接收线程
    pthread_t recv_thread;
    int ret = pthread_create(&recv_thread, NULL, receive_func, NULL);
    if(ret != 0)
    {
        ROS_ERROR("Failed to create receiving thread.");
        VCI_CloseDevice(VCI_USBCAN2, 0);
        pthread_mutex_destroy(&can_mutex);
        return -1;
    }
    ROS_INFO("Receiving thread started.");

    // 主循环，用于在数据准备好时发送 CAN 消息
    ros::Rate loop_rate(100); // 主循环频率 100 Hz
    while (ros::ok())
    {
        bool ready_to_send = false;
        float angle = 0.0;
        float speed = 0.0;

        // 锁定互斥量以访问共享数据
        {
            std::lock_guard<std::mutex> lock(data_mutex);
            if (combined_message.angle_ready && combined_message.speed_ready)
            {
                ready_to_send = true;
                angle = combined_message.angle;
                speed = combined_message.speed;

                // 重置准备标志
                combined_message.angle_ready = false;
                combined_message.speed_ready = false;
            }
        }

        if (ready_to_send)
        {
            // 将角度和速度转换为整数
            int angle_int = static_cast<int>(angle * 100);
            int speed_int = static_cast<int>(speed * 100);

            // 准备发送帧
            VCI_CAN_OBJ send;
            memset(&send, 0, sizeof(VCI_CAN_OBJ)); // 初始化 send
            send.ID = 0x233;          // 设置 ID 为 0x233
            send.SendType = 0;        // 正常发送
            send.RemoteFlag = 0;      // 数据帧
            send.ExternFlag = 0;      // 标准帧（11 位 ID）
            send.DataLen = 8;         // 数据长度 8 字节

            // 设置发送的数据
            send.Data[0] = (angle_int >> 8) & 0xFF;
            send.Data[1] = angle_int & 0xFF;
            send.Data[2] = (speed_int >> 8) & 0xFF;
            send.Data[3] = speed_int & 0xFF;
            send.Data[4] = 0x00;
            send.Data[5] = 0x00;
            send.Data[6] = 0x00;
            send.Data[7] = 0x48;

            // 发送 CAN 消息
            pthread_mutex_lock(&can_mutex);
            int send_ret = VCI_Transmit(VCI_USBCAN2, 0, 0, &send, 1);
            pthread_mutex_unlock(&can_mutex);

            if(send_ret == 1)
            {
                ROS_INFO("  CAN1 TX ID: 0x%08X", send.ID);
                ROS_INFO("  Frame format: %s", send.ExternFlag ? "Extended frame" : "Standard frame");

                std::stringstream data_ss;
                data_ss << "  Data: ";
                for(int j = 0; j < send.DataLen; j++) {
                    data_ss << "0x" << std::hex << (int)send.Data[j] << " ";
                }
                ROS_INFO("%s", data_ss.str().c_str());
            }
            else
            {
                ROS_ERROR("Send failed, error code: %d", send_ret);
                // 根据错误码进行进一步处理
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("Sending completed, preparing to stop receiving thread.");
    running = 0;

    // 等待接收线程结束
    pthread_join(recv_thread, NULL);
    ROS_INFO("Receiving thread has ended.");

    // 复位 CAN1
    pthread_mutex_lock(&can_mutex);
    if(VCI_ResetCAN(VCI_USBCAN2, 0, 0) != 1)
    {
        ROS_ERROR("Failed to reset CAN1.");
    }
    else
    {
        ROS_INFO("CAN1 reset successfully.");
    }
    pthread_mutex_unlock(&can_mutex);

    // 关闭设备
    if(VCI_CloseDevice(VCI_USBCAN2, 0) != 1)
    {
        ROS_ERROR("Failed to close device.");
        pthread_mutex_destroy(&can_mutex);
        return -1;
    }
    ROS_INFO("Device closed successfully.");

    // 销毁互斥锁
    pthread_mutex_destroy(&can_mutex);

    ROS_INFO("Node is about to shut down.");
    ros::shutdown();

    return 0;
}
