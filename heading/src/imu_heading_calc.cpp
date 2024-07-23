#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float64.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <math.h>

class ImuHeadingCalculator
{
public:
    ImuHeadingCalculator()
    {
        heading = 0.0;
        adjust_heading = 136.64;  // 필요에 따라 조정값을 설정합니다.
        heading_pub = nh.advertise<std_msgs::Float64>("/heading", 10);
        imu_sub = nh.subscribe("/imu", 10, &ImuHeadingCalculator::imuCB, this);
    }

    void imuCB(const sensor_msgs::Imu::ConstPtr& msg)
    {
        // 쿼터니언을 사용하여 roll, pitch, yaw 계산
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w
        );

        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        heading = yaw * 180.0 / M_PI;

        // print('BEFORE: ', heading);
        heading = heading + adjust_heading;
        // print('AFTER: ', heading);

        if (heading > 180)
        {
            heading -= 360;
        }
        else if (heading < -180)
        {
            heading += 360;
        }

        // Heading을 퍼블리시
        std_msgs::Float64 heading_msg;
        heading_msg.data = heading;
        heading_pub.publish(heading_msg);
    }

private:
    ros::NodeHandle nh;
    ros::Publisher heading_pub;
    ros::Subscriber imu_sub;
    double heading;
    double adjust_heading;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_heading_calculator");
    ImuHeadingCalculator imuHeadingCalculator;
    ros::spin();
    return 0;
}
