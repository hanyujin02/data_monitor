#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <data_monitor/BatteryPoseVel.h>

class BatteryPoseVelNode
{
public:
    BatteryPoseVelNode()
    {
        battery_sub_ = nh_.subscribe("/mavros/battery", 10, &BatteryPoseVelNode::batteryCallback, this);
        pose_sub_ = nh_.subscribe("/mavros/local_position/pose", 10, &BatteryPoseVelNode::poseCallback, this);
        velocity_sub_ = nh_.subscribe("/mavros/local_position/velocity_local", 10, &BatteryPoseVelNode::velocityCallback, this);

        pub_ = nh_.advertise<data_monitor::BatteryPoseVel>("/battery_pose_velocity", 10);

        battery_ready_ = pose_ready_ = velocity_ready_ = false;
    }

    void spin()
    {
        ros::Rate rate(10);
        while (ros::ok())
        {
            if (battery_ready_ && pose_ready_ && velocity_ready_)
            {
                data_monitor::BatteryPoseVel msg;
                msg.battery_percentage = battery_percentage_;
                msg.pose = pose_;
                msg.velocity = velocity_;
                pub_.publish(msg);
            }

            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber battery_sub_, pose_sub_, velocity_sub_;
    ros::Publisher pub_;

    float battery_percentage_;
    geometry_msgs::Pose pose_;
    geometry_msgs::Twist velocity_;

    bool battery_ready_, pose_ready_, velocity_ready_;

    void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg)
    {
        battery_percentage_ = msg->percentage;
        battery_ready_ = true;
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        pose_ = msg->pose;
        pose_ready_ = true;
    }

    void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
    {
        velocity_ = msg->twist;
        velocity_ready_ = true;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "battery_pose_vel_node");
    BatteryPoseVelNode node;
    node.spin();
    return 0;
}
