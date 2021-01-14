#include "../../May/May.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Int16.h"
#include <ros/ros.h>

class BoxMoving
{
public:
    BoxMoving();
    ~BoxMoving();

    void StartStrategy();
    void MainStrategy();

private:
    May *CMay;
    ros::NodeHandle n_;
    ros::Subscriber yuan_subber_;
    thread *yuan_sub_thread_;

    ros::Publisher yuan_pubber_;
    std_msgs::Int16MultiArray yuan_msg_;
    thread *yuan_pub_thread_;

    int yuan_state_;

    void YuanCallback(const std_msgs::Int16::ConstPtr &yuan_msg);
    void YuanPub(void);
    void YuanSub(void);
};