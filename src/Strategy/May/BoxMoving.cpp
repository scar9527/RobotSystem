#include "BoxMoving.h"

BoxMoving::BoxMoving()
{
    CMay = May::getMay();
    StartStrategy();
}

BoxMoving::~BoxMoving()
{
    yuan_pub_thread_->join();
    yuan_sub_thread_->join();
    delete yuan_pub_thread_;
    delete yuan_sub_thread_;  
}

void BoxMoving::StartStrategy()
{
    /* Yuan */
    yuan_state_ = 0;
    yuan_pubber_ = n_.advertise<std_msgs::Int16MultiArray>("SixArm_data", 1);
    yuan_pub_thread_ = new std::thread(&BoxMoving::YuanPub, this);
    yuan_sub_thread_ = new std::thread(&BoxMoving::YuanSub, this);
    MainStrategy();
}

void BoxMoving::MainStrategy()
{
    /* Initialize */
    CMay->TrajectoryPlanning(0, 0, 0, 300, 0, 300);
    
    while(true)
    {
        /* On conveyor, Up */
        CMay->TrajectoryPlanning(0, 0, 0, 0, 550, 300);

        /* On conveyor, Down */
        CMay->TrajectoryPlanning(0, 0, 0, 0, 550, 210);

        /* Pneumatic On */
        CMay->PneumaticOn();

        /* On conveyor, Up */
        CMay->TrajectoryPlanning(0, 0, 0, 0, 550, 300);

        /* Off conveyor, Up */
        CMay->TrajectoryPlanning(0, 0, 0, 600, 0, 300);

        /* Off conveyor, Down */
        CMay->TrajectoryPlanning(0, 0, 0, 600, 0, -180);

        /* Pneumatic Off */
        CMay->PneumaticOff();

        /* Off conveyor, Up */
        CMay->TrajectoryPlanning(0, 0, 0, 600, 0, 300);
    }
}

void BoxMoving::YuanPub()
{
    while (true)
    {   
        // ID_Light 0-5
        for (int a=0; a < 6; a++)
            yuan_msg_.data.push_back(CMay->GetMotor_Connected(a));

        // Value 6-11
        for (int e=0; e < 6; e++)
        {
            yuan_msg_.data.push_back((int)CMay->GetMotor_PresentAngle(e));
            yuan_msg_.data.push_back((int)CMay->GetMotor_Velocity(e));
            yuan_msg_.data.push_back((int)CMay->GetMotor_PresentTorque(e));
        }

        //Hand_Pos x_y_z_ox_oy_oz 12-17
        for (int i=0; i < 3; i++)
            yuan_msg_.data.push_back((int)CMay->GetCurrentPosition(i));
        for (int j=0; j < 3; j++)
            yuan_msg_.data.push_back((int)CMay->GetCurrentOrientation(j));

        this->yuan_pubber_.publish(yuan_msg_);
        ros::spinOnce();

        yuan_msg_.data.clear();

        std::this_thread::sleep_for(std::chrono::milliseconds(int(10)));
    }
}

void BoxMoving::YuanSub()
{
    yuan_subber_ = this->n_.subscribe("/SixArm_data", 1, &BoxMoving::YuanCallback, this);
    ros::spin();
}

void BoxMoving::YuanCallback(const std_msgs::Int16::ConstPtr &yuan_msg)
{
    /* Update state */
    yuan_state_ = yuan_msg->data;

    /* Default State */
    if (yuan_state_ == 3)
    {
    }
    /* Stop */
    else if (yuan_state_ == 0)
    {
        CMay->Stop();
    }
    /* Start */
    else if (yuan_state_ ==1)
    {
        CMay->Start();
    }
}