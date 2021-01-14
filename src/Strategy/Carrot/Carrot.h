#include "../../Robot/Robot.h"
#include "std_msgs/Int16.h"

class Carrot
{
public:
    Carrot(const int &strategy_index);
    ~Carrot();

    void StartStrategy(void);

    void SAMove(void);

    void Debug(void);

    void ForwardData(void);
    void ShiftData(void);
    void SelfTurnData(void);

    void LeftMove(const float &J0, const float &ox, const float &oy, const float &oz, const float &px, const float &py, const float &pz);
    void RightMove(const float &J0, const float &ox, const float &oy, const float &oz, const float &px, const float &py, const float &pz);

    void ImpedenceControl(const float &ox, const float &oy, const float &oz, const float &px, const float &py, const float &pz);
    void DualArmAdapt(void);
    Eigen::Matrix<float, 6, 1> MotorVelocityCheck(Eigen::Matrix<float, 6, 1> motor_velocity);

    Eigen::Matrix<float, 3, 2> OrientationMapping(const int &arm_index);

private:
    Robot *CRobot;
    const int strategy_index_;
    std::thread *left_thread_;
    std::thread *right_thread_;

    bool is_out_of_limit_;
    const float delay_time_;

private:
    /* Listener */
    ros::NodeHandle n_;
    ros::Subscriber subber_;
    thread *sub_thread_;

    ros::Subscriber yuan_subber_joy_;
    ros::Subscriber yuan_subber_all_;
    thread *yuan_sub_thread_;

    /* Talker */
    ros::Publisher pubber_;
    ros::Publisher yuan_pubber_;
    std_msgs::Int16MultiArray msg_;
    std_msgs::Int16MultiArray yuan_msg_;
    thread *pub_thread_;
    thread *yuan_pub_thread_;
    
    int received_state_;
    int yuan_state_;

    void CallbackFunction(const std_msgs::Int16::ConstPtr &msg);
    void YuanCallback(const std_msgs::Int16::ConstPtr &yuan_msg);
    void Sub(void);
    void Pub(void);
    void YuanPub(void);
    void YuanSub(void);
};