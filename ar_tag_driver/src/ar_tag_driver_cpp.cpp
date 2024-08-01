#include <ros/ros.h>
#include <xycar_msgs/xycar_motor.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <math.h>
#include <vector>
#include <algorithm>
#include <iostream>

class ArTag {
public:
    ArTag(const ar_track_alvar_msgs::AlvarMarker& marker)
        : x(marker.pose.pose.position.x), y(marker.pose.pose.position.y), z(marker.pose.pose.position.z) {}

    float x, y, z;
};

class ArTagDriver {
public:
    ArTagDriver() {
        ros::NodeHandle nh;
        ar_sub = nh.subscribe("ar_pose_marker", 1, &ArTagDriver::arCB, this);
        ctrl_cmd_pub = nh.advertise<xycar_msgs::xycar_motor>("/xycar_motor_ar", 1);

        k = 2;
        l = 30;
        angle_cal = 0.017;

        speed = 0;
        flag = false;
        rate = new ros::Rate(30); // 30 Hz
    }

    void run() {
        while (ros::ok()) {
            std::cout << "AR Tag self.angle: " << angle << std::endl;
            speed = 4;
            publishCtrlCmd(speed, angle, flag);
            rate->sleep();
            ros::spinOnce();
        }
    }

private:
    ros::Subscriber ar_sub;
    ros::Publisher ctrl_cmd_pub;
    xycar_msgs::xycar_motor ctrl_cmd_msg_ar;

    std::vector<ArTag> sorted_ar_list;
    int k;
    int l;
    double angle_cal;

    int speed;
    bool flag;
    double angle;

    ros::Rate* rate;

    void publishCtrlCmd(int motor_msg, double servo_msg, bool flag) {
        ctrl_cmd_msg_ar.speed = motor_msg;
        ctrl_cmd_msg_ar.angle = servo_msg;
        ctrl_cmd_msg_ar.flag = flag;
        ctrl_cmd_pub.publish(ctrl_cmd_msg_ar);
    }

    void arCB(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {
        if (msg->markers.empty()) {
            flag = false;
            return;
        }

        flag = true;
        sorted_ar_list.clear();
        for (const auto& marker : msg->markers) {
            sorted_ar_list.push_back(ArTag(marker));
        }

        std::sort(sorted_ar_list.begin(), sorted_ar_list.end(), [](const ArTag& a, const ArTag& b) {
            return a.z < b.z;
        });

        ArTag closest_ar = sorted_ar_list[0];
        angle = static_cast<int>(k * std::round((closest_ar.x - angle_cal) * 180.0 / M_PI)) - l;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ar_tag_driver_node");
    try {
        ArTagDriver node;
        node.run();
    } catch (ros::Exception& e) {
        ROS_ERROR("ROS Exception: %s", e.what());
    }

    return 0;
}
