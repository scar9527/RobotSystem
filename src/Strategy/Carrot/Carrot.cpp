#include "Carrot.h"

Carrot::Carrot(const int &strategy_index) : strategy_index_(strategy_index),
                                            delay_time_(10)
{
    CRobot = Robot::getRobot();
    StartStrategy();
}

Carrot::~Carrot()
{
    yuan_pub_thread_->join();
    yuan_sub_thread_->join();
    delete yuan_pub_thread_;
    delete yuan_sub_thread_;    
}

void Carrot::StartStrategy()
{
    is_out_of_limit_ = false;

    if (strategy_index_ == 0)
    {
        ForwardData();
    }
    else if (strategy_index_ == 1)
    {
        ShiftData();
    }
    else if (strategy_index_ == 2)
    {
        SelfTurnData();
    }
    else if (strategy_index_ == 3)
    {
        ImpedenceControl(0, 0, 0, 350, 120, -300);
    }
    else if (strategy_index_ == 4)
    {
        SAMove();
    }
    else if (strategy_index_ == 5)
    {
        DualArmAdapt();
    }
    else if (strategy_index_ == -1)
        Debug();
    else
        ;
}

/* Subscriber */
void Carrot::CallbackFunction(const std_msgs::Int16::ConstPtr &msg)
{
    received_state_ = msg->data;
}

void Carrot::Sub(void)
{
    subber_ = this->n_.subscribe("dominated_state", 1, &Carrot::CallbackFunction, this);
    ros::spinOnce();
}

/* Publisher */
void Carrot::Pub(void)
{
    Eigen::Matrix<float, 3, 2> left_compensated_data;
    Eigen::Matrix<float, 3, 2> right_compensated_data;

    while (true)
    {
        left_compensated_data = OrientationMapping(0);
        right_compensated_data = OrientationMapping(1);

        msg_.data.push_back(left_compensated_data.col(0)(0));
        msg_.data.push_back(left_compensated_data.col(0)(1));
        msg_.data.push_back(left_compensated_data.col(0)(2));
        msg_.data.push_back(left_compensated_data.col(1)(0));
        msg_.data.push_back(left_compensated_data.col(1)(1));
        msg_.data.push_back(left_compensated_data.col(1)(2));

        msg_.data.push_back(right_compensated_data.col(0)(0));
        msg_.data.push_back(right_compensated_data.col(0)(1));
        msg_.data.push_back(right_compensated_data.col(0)(2));
        msg_.data.push_back(right_compensated_data.col(1)(0));
        msg_.data.push_back(right_compensated_data.col(1)(1));
        msg_.data.push_back(right_compensated_data.col(1)(2));

        this->pubber_.publish(msg_);
        ros::spinOnce();

        msg_.data.clear();

        std::this_thread::sleep_for(std::chrono::milliseconds(int(delay_time_)));
    }
}

void Carrot::SAMove()
{
    /* Yuan */
    yuan_state_ = 0;
    yuan_pubber_ = n_.advertise<std_msgs::Int16MultiArray>("Joy_data", 1);
    yuan_pub_thread_ = new std::thread(&Carrot::YuanPub, this);
    yuan_sub_thread_ = new std::thread(&Carrot::YuanSub, this);

    /* Carrot */
    sub_thread_ = new std::thread(&Carrot::Sub, this);

    pubber_ = n_.advertise<std_msgs::Int16MultiArray>("DataFetch", 1);
    pub_thread_ = new std::thread(&Carrot::Pub, this);

    std::thread *arm_thread = new std::thread(&Carrot::DualArmAdapt, this);

    bool is_changed_recently = false;
    int time_counter = 0;
    int mobile_state = 0;

    Eigen::Matrix<float, 3, 1> left_compensated_force;
    Eigen::Matrix<float, 3, 1> right_compensated_force;

    CRobot->CMobile->CWheel->LockOn();

    int average_velocity = 0;

    while (true)
    {
        CRobot->CLeftArm->CalculateJacobianMatrix();
        CRobot->CRightArm->CalculateJacobianMatrix();

        left_compensated_force = OrientationMapping(0).col(0);
        right_compensated_force = OrientationMapping(1).col(0);

        // Disturbance
        if (received_state_ == 0)
        {
        }
        // Forward
        else if ((received_state_ == 1 || received_state_ == 2) && is_changed_recently == false)
        {
            if (mobile_state != 1)
            {
                is_changed_recently = true;
                CRobot->CMobile->CWheel->MoveByConstant(0);
                CRobot->CMobile->CWheel->Wait();
                CRobot->CMobile->CSteering->TurnStraight();
                CRobot->CMobile->CSteering->Wait();
                mobile_state = 1;
            }
        }
        // Shift
        else if ((received_state_ == 3 || received_state_ == 4) && is_changed_recently == false)
        {
            if (mobile_state != 2)
            {
                is_changed_recently = true;
                CRobot->CMobile->CWheel->MoveByConstant(0);
                CRobot->CMobile->CWheel->Wait();
                CRobot->CMobile->CSteering->TurnAll(90);
                CRobot->CMobile->CSteering->Wait();
                mobile_state = 2;
            }
        }
        // Selfturn
        else if ((received_state_ == 5 || received_state_ == 6) && is_changed_recently == false)
        {
            if (mobile_state != 3)
            {
                is_changed_recently = true;
                CRobot->CMobile->CWheel->MoveByConstant(0);
                CRobot->CMobile->CWheel->Wait();
                CRobot->CMobile->CSteering->Self_Turn();
                CRobot->CMobile->CSteering->Wait();
                mobile_state = 3;
            }
        }

        // Buffering time before changing state
        if (is_changed_recently == true)
        {
            if (time_counter >= 100)
            {
                is_changed_recently = false;
                time_counter = 0;
            }
            else
            {
                time_counter++;
            }
        }
        /* Wheel Control */
        int move_threshold = 150;
        int move_limit = 300;

        if (mobile_state == 1)
        {
            if (abs(left_compensated_force(0)) > move_threshold && abs(right_compensated_force(0)) > move_threshold && left_compensated_force(0) * right_compensated_force(0) >= 0)
            {
                average_velocity = (left_compensated_force(0) + right_compensated_force(0)) / 2;
                CRobot->CMobile->CWheel->MoveByConstant(average_velocity * 2);
            }
            else
            {
                average_velocity = 0;
                CRobot->CMobile->CWheel->MoveByConstant(0);
            }
        }
        else if (mobile_state == 2)
        {
            if (abs(left_compensated_force(1)) > move_threshold && abs(right_compensated_force(1)) > move_threshold && left_compensated_force(1) * right_compensated_force(1) >= 0)
            {
                average_velocity = (left_compensated_force(1) + right_compensated_force(1)) / 2;
                CRobot->CMobile->CWheel->MoveByConstant(average_velocity * 2);
            }
            else
            {
                average_velocity = 0;
                CRobot->CMobile->CWheel->MoveByConstant(0);
            }
        }
        else if (mobile_state == 3)
        {
            if (abs(left_compensated_force(0)) > move_threshold && abs(right_compensated_force(0)) > move_threshold && left_compensated_force(0) * right_compensated_force(0) <= 0)
            {
                average_velocity = (abs(left_compensated_force(0)) + abs(right_compensated_force(0))) / 2;

                if (left_compensated_force(0) <= 0)
                    CRobot->CMobile->CWheel->SelfTurnByConstant(average_velocity);
                else if (left_compensated_force(0) > 0)
                    CRobot->CMobile->CWheel->SelfTurnByConstant(-average_velocity);
            }
            else
            {
                average_velocity = 0;
                CRobot->CMobile->CWheel->SelfTurnByConstant(0);
            }                
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(int(delay_time_)));
    }    
}

void Carrot::Debug()
{
    pubber_ = n_.advertise<std_msgs::Int16MultiArray>("DataFetch", 1);
    pub_thread_ = new std::thread(&Carrot::Pub, this);

    left_thread_ = new thread(&Carrot::LeftMove, this, 0, 0, 0, 0, 450, 120, -150);
    right_thread_ = new thread(&Carrot::RightMove, this, 0, 0, 0, 0, 450, -120, -150);
    left_thread_->join();
    right_thread_->join();
    delete left_thread_;
    delete right_thread_;

    CRobot->CLeftForceSensor->DataNormalization();
    CRobot->CRightForceSensor->DataNormalization();

    while (true)
    {
        std::cout << "YA" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}

void Carrot::LeftMove(const float &J0, const float &ox, const float &oy, const float &oz, const float &px, const float &py, const float &pz)
{
    CRobot->CLeftArm->TrajectoryPlanning(J0, ox, oy, oz, px, py, pz);
}

void Carrot::RightMove(const float &J0, const float &ox, const float &oy, const float &oz, const float &px, const float &py, const float &pz)
{
    CRobot->CRightArm->TrajectoryPlanning(J0, ox, oy, oz, px, py, pz);
}

/*
 * arm_index: 0 for left and 1 for right
 * data_index: 0 for force and 1 for torque
 */
Eigen::Matrix<float, 3, 2> Carrot::OrientationMapping(const int &arm_index)
{
    /* Orientation Mapping */
    float ox = 0.0;
    float oy = 0.0;
    float oz = 0.0;

    Eigen::Matrix<float, 3, 2> data;

    if (arm_index == 0) // Left Arm Data
    {
        ox = CRobot->CLeftArm->GetCurrentOrientation(0) * Angle2Rad;
        oy = CRobot->CLeftArm->GetCurrentOrientation(1) * Angle2Rad;
        oz = CRobot->CLeftArm->GetCurrentOrientation(2) * Angle2Rad;

        data.col(0) << CRobot->CLeftForceSensor->GetNormalizedFx(),
            CRobot->CLeftForceSensor->GetNormalizedFy(),
            CRobot->CLeftForceSensor->GetNormalizedFz();

        data.col(1) << CRobot->CLeftForceSensor->GetNormalizedMx(),
            CRobot->CLeftForceSensor->GetNormalizedMy(),
            CRobot->CLeftForceSensor->GetNormalizedMz();
    }
    else if (arm_index == 1) // Right Arm Data
    {
        ox = CRobot->CRightArm->GetCurrentOrientation(0) * Angle2Rad;
        oy = CRobot->CRightArm->GetCurrentOrientation(1) * Angle2Rad;
        oz = CRobot->CRightArm->GetCurrentOrientation(2) * Angle2Rad;

        data.col(0) << CRobot->CRightForceSensor->GetNormalizedFx(),
            CRobot->CRightForceSensor->GetNormalizedFy(),
            CRobot->CRightForceSensor->GetNormalizedFz();
        data.col(1) << CRobot->CRightForceSensor->GetNormalizedMx(),
            CRobot->CRightForceSensor->GetNormalizedMy(),
            CRobot->CRightForceSensor->GetNormalizedMz();
    }

    /* GetRotationMatrix is actually wrote in the Arm class and there is no difference as subclass goes */
    Eigen::Matrix<float, 3, 3> x_orientation_rotation_matrix = CRobot->CLeftArm->GetRotationMatrix(0, ox);
    Eigen::Matrix<float, 3, 3> y_orientation_rotation_matrix = CRobot->CLeftArm->GetRotationMatrix(1, oy);
    Eigen::Matrix<float, 3, 3> z_orientation_rotation_matrix = CRobot->CLeftArm->GetRotationMatrix(2, oz);

    /* Force Sensor Installation Mapping */
    Eigen::Matrix<float, 3, 3> z_installation_rotation_matrix = CRobot->CLeftArm->GetRotationMatrix(2, M_PI_2);
    Eigen::Matrix<float, 3, 3> x_installation_rotation_matrix = CRobot->CLeftArm->GetRotationMatrix(0, M_PI_2);

    return z_orientation_rotation_matrix * y_orientation_rotation_matrix * x_orientation_rotation_matrix * z_installation_rotation_matrix * x_installation_rotation_matrix * data;
}

void Carrot::ForwardData()
{
    pubber_ = n_.advertise<std_msgs::Int16MultiArray>("DataFetch", 1);
    pub_thread_ = new std::thread(&Carrot::Pub, this);    

    float move_threshold = 150;
    float move_limit = 300;
    CRobot->CMobile->CSteering->TurnStraight();
    CRobot->CMobile->CWheel->LockOn();

    left_thread_ = new thread(&Carrot::LeftMove, this, 0, 0, 0, 0, 450, 120, -150);
    right_thread_ = new thread(&Carrot::RightMove, this, 0, 0, 0, 0, 450, -120, -150);
    left_thread_->join();
    right_thread_->join();
    delete left_thread_;
    delete right_thread_;

    CRobot->CLeftForceSensor->DataNormalization();
    CRobot->CRightForceSensor->DataNormalization();

    Eigen::Matrix<float, 3, 2> left_compensated_data;
    Eigen::Matrix<float, 3, 2> right_compensated_data;

    float lfx = 0.0;
    float rfx = 0.0;

    std::thread *arm_thread = new std::thread(&Carrot::DualArmAdapt, this);

    while (true)
    {
        left_compensated_data = OrientationMapping(0);
        right_compensated_data = OrientationMapping(1);

        lfx = left_compensated_data.col(0)(0);
        rfx = right_compensated_data.col(0)(0);

        if (abs(lfx) > move_threshold && abs(rfx) > move_threshold && (lfx * rfx) >= 0)
        {
            float average_vel = (lfx + rfx) / 2;
            std::cout << average_vel << std::endl;

            if (abs(average_vel) > move_limit)
            {
                if (average_vel > 0)
                    CRobot->CMobile->CWheel->MoveByConstant(move_limit * 2);
                else
                    CRobot->CMobile->CWheel->MoveByConstant(-move_limit * 2);
            }
            else if (abs(average_vel) > move_threshold && abs(average_vel) < move_limit)
            {
                CRobot->CMobile->CWheel->MoveByConstant(average_vel * 2);
            }
        }
        else
            CRobot->CMobile->CWheel->MoveByConstant(0);

        std::this_thread::sleep_for(std::chrono::milliseconds(int(delay_time_)));
    }
}

void Carrot::ShiftData()
{
    pubber_ = n_.advertise<std_msgs::Int16MultiArray>("DataFetch", 1);
    pub_thread_ = new std::thread(&Carrot::Pub, this);

    float move_threshold = 150;
    float move_limit = 300;

    /* move forward */
    CRobot->CMobile->CSteering->TurnAll(90);
    CRobot->CMobile->CWheel->LockOn();

    left_thread_ = new thread(&Carrot::LeftMove, this, 0, 0, 0, 0, 450, 120, -150);
    right_thread_ = new thread(&Carrot::RightMove, this, 0, 0, 0, 0, 450, -120, -150);
    left_thread_->join();
    right_thread_->join();
    delete left_thread_;
    delete right_thread_;

    CRobot->CLeftForceSensor->DataNormalization();
    CRobot->CRightForceSensor->DataNormalization();

    Eigen::Matrix<float, 3, 2> left_compensated_data;
    Eigen::Matrix<float, 3, 2> right_compensated_data;

    float lfy = 0.0;
    float rfy = 0.0;

    std::thread *arm_thread = new std::thread(&Carrot::DualArmAdapt, this);

    while (true)
    {
        left_compensated_data = OrientationMapping(0);
        right_compensated_data = OrientationMapping(1);

        lfy = left_compensated_data.col(0)(1);
        rfy = right_compensated_data.col(0)(1);

        if (abs(lfy) > move_threshold && abs(rfy) > move_threshold && (lfy * rfy) >= 0)
        {
            float average_vel = (lfy + rfy) / 2;
            std::cout << average_vel << std::endl;
            if (abs(average_vel) > move_limit)
            {
                if (average_vel > 0)
                    CRobot->CMobile->CWheel->MoveByConstant(move_limit * 2);
                else
                    CRobot->CMobile->CWheel->MoveByConstant(-move_limit * 2);
            }
            else if (abs(average_vel) > move_threshold && abs(average_vel) < move_limit)
            {
                CRobot->CMobile->CWheel->MoveByConstant(average_vel * 2);
            }
        }
        else
            CRobot->CMobile->CWheel->MoveByConstant(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(int(delay_time_)));
    }
}

void Carrot::SelfTurnData()
{
    pubber_ = n_.advertise<std_msgs::Int16MultiArray>("DataFetch", 1);
    pub_thread_ = new std::thread(&Carrot::Pub, this);

    float move_threshold = 150;
    float move_limit = 300;

    /* move forward */
    CRobot->CMobile->CSteering->Self_Turn();
    CRobot->CMobile->CWheel->LockOn();

    left_thread_ = new thread(&Carrot::LeftMove, this, 0, 0, 0, 0, 450, 120, -150);
    right_thread_ = new thread(&Carrot::RightMove, this, 0, 0, 0, 0, 450, -120, -150);
    left_thread_->join();
    right_thread_->join();
    delete left_thread_;
    delete right_thread_;

    CRobot->CLeftForceSensor->DataNormalization();
    CRobot->CRightForceSensor->DataNormalization();

    Eigen::Matrix<float, 3, 2> left_compensated_data;
    Eigen::Matrix<float, 3, 2> right_compensated_data;

    float lfx = 0.0;
    float rfx = 0.0;

     std::thread *arm_thread = new std::thread(&Carrot::DualArmAdapt, this);

    while (true)
    {
        left_compensated_data = OrientationMapping(0);
        right_compensated_data = OrientationMapping(1);

        lfx = left_compensated_data.col(0)(0);
        rfx = right_compensated_data.col(0)(0);

        if (abs(lfx) > move_threshold && abs(rfx) > move_threshold && (lfx * rfx) <= 0)
        {
            float average_vel = (abs(lfx) + abs(rfx)) / 2;
            std::cout << average_vel << std::endl;
            if (abs(average_vel) > move_limit)
            {
                if (lfx <= 0 && rfx >= 0)
                    CRobot->CMobile->CWheel->SelfTurnByConstant(move_limit);
                else if (lfx >= 0 && rfx <= 0)
                    CRobot->CMobile->CWheel->SelfTurnByConstant(-move_limit);
            }
            else if (abs(average_vel) > move_threshold && abs(average_vel) < move_limit)
            {
                if (lfx <= 0 && rfx >= 0)
                    CRobot->CMobile->CWheel->SelfTurnByConstant(average_vel);
                else if (lfx >= 0 && rfx <= 0)
                    CRobot->CMobile->CWheel->SelfTurnByConstant(-average_vel);
            }
        }
        else
            CRobot->CMobile->CWheel->SelfTurnByConstant(0);

        std::this_thread::sleep_for(std::chrono::milliseconds(int(delay_time_)));
    }
}

void Carrot::ImpedenceControl(const float &ox, const float &oy, const float &oz, const float &px, const float &py, const float &pz)
{
    /* Define the coefficients of Motion(M), Damping(B) and Stiffness(K) on different axis xyz */
    /* LINEAR */
    Eigen::Matrix<float, 3, 3> M_linear = Eigen::Matrix3f::Identity(3, 3);
    M_linear *= 0.001;

    Eigen::Matrix<float, 3, 3> B_linear = Eigen::Matrix3f::Identity(3, 3);
    B_linear *= 2;

    Eigen::Matrix<float, 3, 3> K_linear = Eigen::Matrix3f::Identity(3, 3);
    K_linear *= 3;

    /* ANGULAR */
    Eigen::Matrix<float, 3, 3> M_angular = Eigen::Matrix3f::Identity(3, 3);
    M_angular *= 0.001;

    Eigen::Matrix<float, 3, 3> B_angular = Eigen::Matrix3f::Identity(3, 3);
    B_angular *= 1.5;

    Eigen::Matrix<float, 3, 3> K_angular = Eigen::Matrix3f::Identity(3, 3);
    K_angular *= 2;

    float time_step = delay_time_ / 1000;

    /* 
     * For following two velocity matrix, col defines time step and row defines the axis
     * It should look like:
     * 
     * Vx_t1, Vx_t2, Vx_t3
     * Vy_t1, Vy_t2, Vy_t3
     * Vz_t1, Vz_t2, Vz_t3 
     */
    Eigen::Matrix3f linear_velocity = Eigen::Matrix3f::Zero(3, 3);
    Eigen::Matrix3f angular_velocity = Eigen::Matrix3f::Zero(3, 3);

    Eigen::Matrix<float, 3, 1> compensated_force;
    Eigen::Matrix<float, 3, 1> compensated_torque;

    CRobot->CLeftArm->TrajectoryPlanning(0, ox, oy, oz, px, py, pz);
    CRobot->CLeftForceSensor->DataNormalization();

    while (true)
    {
        CRobot->CLeftArm->CalculateJacobianMatrix();

        compensated_force = OrientationMapping(0).col(0);
        compensated_torque = OrientationMapping(0).col(1) / 500;

        for (int i = 0; i < 3; i++)
        {
            if (abs(compensated_force(i)) < 50)
                compensated_force(i) = 0;
            if (abs(compensated_torque(i)) < 0.1)
                compensated_torque(i) = 0;
        }

        Eigen::Matrix<float, 3, 1> current_position;
        current_position << CRobot->CLeftArm->GetCurrentPosition(0), CRobot->CLeftArm->GetCurrentPosition(1), CRobot->CLeftArm->GetCurrentPosition(2);

        Eigen::Matrix<float, 3, 1> target_position;
        target_position << px, py, pz;

        Eigen::Matrix<float, 3, 1> current_orientation;
        current_orientation << CRobot->CLeftArm->GetCurrentOrientation(0), CRobot->CLeftArm->GetCurrentOrientation(1), CRobot->CLeftArm->GetCurrentOrientation(2);
        current_orientation = current_orientation * Angle2Rad;

        Eigen::Matrix<float, 3, 1> target_orientation;
        target_orientation << ox, oy, oz;

        /* Calculate linear velocity in t1, t2 and t3 */
        linear_velocity.col(2) = B_linear.lu().solve(compensated_force -
                                                     M_linear * (linear_velocity.col(1) - linear_velocity.col(0)) / time_step -
                                                     K_linear * (current_position - target_position));
        linear_velocity.col(0) = linear_velocity.col(1);
        linear_velocity.col(1) = linear_velocity.col(2);

        /* Calculate angular velocity in t1, t2 and t3 */
        angular_velocity.col(2) = B_angular.lu().solve(compensated_torque -
                                                       M_angular * (angular_velocity.col(1) - angular_velocity.col(0)) / time_step -
                                                       K_angular * (current_orientation - target_orientation));
        angular_velocity.col(0) = angular_velocity.col(1);
        angular_velocity.col(1) = angular_velocity.col(2);

        Eigen::Matrix<float, 6, 1> end_effector_velocity;

        /* Zero for both, one for linear only, two for angular only */
        int test_mode = 0;
        if (test_mode == 0)
        {
            end_effector_velocity << angular_velocity(0, 2),
                angular_velocity(1, 2),
                angular_velocity(2, 2),
                linear_velocity(0, 2),
                linear_velocity(1, 2),
                linear_velocity(2, 2);
        }
        else if (test_mode == 1)
        {
        }
        else if (test_mode == 2)
        {
            Eigen::Matrix<float, 3, 1> eazy_linear_velocity = K_linear * (target_position - current_position);
            end_effector_velocity << angular_velocity(0, 2),
                angular_velocity(1, 2),
                angular_velocity(2, 2),
                eazy_linear_velocity(0),
                eazy_linear_velocity(1),
                eazy_linear_velocity(2);
        }

        Eigen::Matrix<float, 6, 1> motor_velocity;
        motor_velocity = CRobot->CLeftArm->GetJacobianMatrix().lu().solve(end_effector_velocity);

        motor_velocity = MotorVelocityCheck(motor_velocity);
        if (is_out_of_limit_)
        {
            std::cout << "\t[WARNING] Dangerous velocity! Fail to arrive." << std::endl;
            CRobot->CLeftArm->SetArmVelocity(0, 0, 0, 0, 0, 0, 0);
            is_out_of_limit_ = false;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            break;
        }
        else
        {
            CRobot->CLeftArm->SetArmVelocity(0,
                                             motor_velocity(0, 0),
                                             motor_velocity(1, 0),
                                             motor_velocity(2, 0),
                                             motor_velocity(3, 0),
                                             motor_velocity(4, 0),
                                             motor_velocity(5, 0));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(int(delay_time_)));
    }
}

Eigen::Matrix<float, 6, 1> Carrot::MotorVelocityCheck(Eigen::Matrix<float, 6, 1> motor_velocity)
{
    /* Determine the index of max velocity */
    float max_speed = 0;
    float max_index = 0;
    for (int i = 0; i < 6; i++)
    {
        if (abs(motor_velocity(i, 0)) >= max_speed)
        {
            max_speed = abs(motor_velocity(i, 0));
            max_index = i;
        }
    }

    if (max_speed >= M_PI) // dangerous zone
    {
        is_out_of_limit_ = true;
    }
    else // limited zone
    {
        if (max_speed >= (M_PI / 6))
            motor_velocity *= abs((M_PI / 6) / motor_velocity(max_index, 0));
    }
    return motor_velocity;
}

void Carrot::DualArmAdapt()
{
    /* LINEAR */
    Eigen::Matrix<float, 3, 3> M_linear = Eigen::Matrix3f::Identity(3, 3);
    M_linear *= 0.001;

    Eigen::Matrix<float, 3, 3> B_linear = Eigen::Matrix3f::Identity(3, 3);
    B_linear *= 1;

    Eigen::Matrix<float, 3, 3> K_linear = Eigen::Matrix3f::Identity(3, 3);
    K_linear *= 2;

    Eigen::Matrix<float, 3, 3> S_factor = Eigen::Matrix3f::Identity(3, 3);
    S_factor *= 1;

    float time_step = delay_time_ / 1000;

    Eigen::Matrix<float, 3, 6> linear_velocity;
    linear_velocity.setZero(3, 6);
    Eigen::Matrix<float, 3, 6> angular_velocity;
    angular_velocity.setZero(3, 6);

    Eigen::Matrix<float, 3, 2> left_compensated_data;
    Eigen::Matrix<float, 3, 2> right_compensated_data;

    Eigen::Matrix<float, 3, 2> temp_position;

    Eigen::Matrix<float, 3, 2> current_orientation;
    Eigen::Matrix<float, 3, 2> target_orientation;
    target_orientation.setZero(3, 2);

    Eigen::Matrix<float, 3, 2> eazy_angular_velocity;

    Eigen::Matrix<float, 6, 2> end_effector_velocity;
    Eigen::Matrix<float, 6, 2> motor_velocity;

    while (true)
    {
        CRobot->CLeftArm->CalculateJacobianMatrix();
        CRobot->CRightArm->CalculateJacobianMatrix();

        left_compensated_data = OrientationMapping(0);
        right_compensated_data = OrientationMapping(1);

        Eigen::Matrix<float, 3, 1> mix_data;
        mix_data = (left_compensated_data.col(0) + right_compensated_data.col(0)) / 2;

        mix_data.row(0) *= 0;
        mix_data.row(1) *= 0;

        if (abs(mix_data(2)) >= 150)
            mix_data(2) = 150 * mix_data(2) / abs(mix_data(2));
        else if (abs(mix_data(2)) < 100)
            mix_data(2) = 0;

        temp_position.col(0) << CRobot->CLeftArm->GetCurrentPosition(0),
            CRobot->CLeftArm->GetCurrentPosition(1) - 120,
            CRobot->CLeftArm->GetCurrentPosition(2);

        temp_position.col(1) << CRobot->CRightArm->GetCurrentPosition(0),
            CRobot->CRightArm->GetCurrentPosition(1) + 120,
            CRobot->CRightArm->GetCurrentPosition(2);

        current_orientation.col(0) << CRobot->CLeftArm->GetCurrentOrientation(0), CRobot->CLeftArm->GetCurrentOrientation(1), CRobot->CLeftArm->GetCurrentOrientation(2);
        current_orientation.col(1) << CRobot->CRightArm->GetCurrentOrientation(0), CRobot->CRightArm->GetCurrentOrientation(1), CRobot->CRightArm->GetCurrentOrientation(2);
        current_orientation *= Angle2Rad;

        /* Iterate linear velocity */
        linear_velocity.col(2) = B_linear.lu().solve(mix_data.col(0) -
                                                     M_linear * (linear_velocity.col(1) - linear_velocity.col(0)) / time_step -
                                                     K_linear * (linear_velocity.col(1) + linear_velocity.col(0)) * time_step / 2 -
                                                     S_factor * (temp_position.col(0) - temp_position.col(1)));
        linear_velocity.col(0) = linear_velocity.col(1);
        linear_velocity.col(1) = linear_velocity.col(2);

        linear_velocity.col(5) = B_linear.lu().solve(mix_data.col(0) -
                                                     M_linear * (linear_velocity.col(4) - linear_velocity.col(3)) / time_step -
                                                     K_linear * (linear_velocity.col(4) + linear_velocity.col(3)) * time_step / 2 -
                                                     S_factor * (temp_position.col(1) - temp_position.col(0)));
        linear_velocity.col(3) = linear_velocity.col(4);
        linear_velocity.col(4) = linear_velocity.col(5);    
        
        eazy_angular_velocity = (target_orientation - current_orientation) * 0.3;
        
        end_effector_velocity.col(0) << eazy_angular_velocity.col(0), linear_velocity.col(2); // Left
        end_effector_velocity.col(1) << eazy_angular_velocity.col(1), linear_velocity.col(5); // Right
      
        motor_velocity.col(0) = CRobot->CLeftArm->GetJacobianMatrix().lu().solve(end_effector_velocity.col(0));
        motor_velocity.col(1) = CRobot->CRightArm->GetJacobianMatrix().lu().solve(end_effector_velocity.col(1));

        /* Checking motor velocity limitation */
        motor_velocity.col(0) = MotorVelocityCheck(motor_velocity.col(0));
        motor_velocity.col(1) = MotorVelocityCheck(motor_velocity.col(1));

        if (is_out_of_limit_)
        {
            std::cout << "\t[WARNING] Dangerous velocity! Fail to arrive." << std::endl;
            CRobot->CLeftArm->SetArmVelocity(0, 0, 0, 0, 0, 0, 0);
            CRobot->CRightArm->SetArmVelocity(0, 0, 0, 0, 0, 0, 0);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            is_out_of_limit_ = false;
            break;
        }
        else
        {
            CRobot->CLeftArm->SetArmVelocity(0,
                                             motor_velocity(0, 0),
                                             motor_velocity(1, 0),
                                             motor_velocity(2, 0),
                                             motor_velocity(3, 0),
                                             motor_velocity(4, 0),
                                             motor_velocity(5, 0));

            CRobot->CRightArm->SetArmVelocity(0,
                                              motor_velocity(0, 1),
                                              motor_velocity(1, 1),
                                              motor_velocity(2, 1),
                                              motor_velocity(3, 1),
                                              motor_velocity(4, 1),
                                              motor_velocity(5, 1));
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(int(delay_time_)));
    }
}

void Carrot::YuanPub()
{
    while (true)
    {   
        // ID_Light 0-21 LA=7 RA=7 W=4 S=4
        for (int a=0; a < 7; a++)
            yuan_msg_.data.push_back(CRobot->CLeftArm->GetMotor_Connected(a));
        for (int b=0; b < 7; b++)
            yuan_msg_.data.push_back(CRobot->CRightArm->GetMotor_Connected(b));
        for (int c=0; c < 4; c++)
            yuan_msg_.data.push_back(CRobot->CMobile->CWheel->GetMotor_Connected(c));
        for (int d=0; d < 4; d++)
            yuan_msg_.data.push_back(CRobot->CMobile->CSteering->GetMotor_Connected(d));

        // Value 22-87 LA=21 RA=21 W=12 S=12
        for (int e=0; e < 7; e++)
        {
            yuan_msg_.data.push_back((int)CRobot->CLeftArm->GetMotor_PresentAngle(e));
            yuan_msg_.data.push_back((int)CRobot->CLeftArm->GetMotor_Velocity(e));
            yuan_msg_.data.push_back((int)CRobot->CLeftArm->GetMotor_PresentTorque(e));
        }
        for (int f=0; f < 7; f++)
        {
            yuan_msg_.data.push_back((int)CRobot->CRightArm->GetMotor_PresentAngle(f));
            yuan_msg_.data.push_back((int)CRobot->CRightArm->GetMotor_Velocity(f));
            yuan_msg_.data.push_back((int)CRobot->CRightArm->GetMotor_PresentTorque(f));
        }
        for (int g=0; g < 4; g++)
        {
            yuan_msg_.data.push_back((int)CRobot->CMobile->CWheel->GetMotor_PresentAngle(g));
            yuan_msg_.data.push_back((int)CRobot->CMobile->CWheel->GetMotor_Velocity(g));
            yuan_msg_.data.push_back((int)CRobot->CMobile->CWheel->GetMotor_PresentTorque(g));
        }
        for (int h=0; h < 4; h++)
        {
            yuan_msg_.data.push_back((int)CRobot->CMobile->CSteering->GetMotor_PresentAngle(h));
            yuan_msg_.data.push_back((int)CRobot->CMobile->CSteering->GetMotor_Velocity(h));
            yuan_msg_.data.push_back((int)CRobot->CMobile->CSteering->GetMotor_PresentTorque(h));
        }

        //Hand_Pos x_y_z_ox_oy_oz 88-99 LA=6 RA=6
        for (int i=0; i < 3; i++)
            yuan_msg_.data.push_back((int)CRobot->CLeftArm->GetCurrentPosition(i));
        for (int j=0; j < 3; j++)
            yuan_msg_.data.push_back((int)CRobot->CLeftArm->GetCurrentOrientation(j));
        for (int k=0; k < 3; k++)
            yuan_msg_.data.push_back((int)CRobot->CRightArm->GetCurrentPosition(k));
        for (int l=0; l < 3; l++)
            yuan_msg_.data.push_back((int)CRobot->CRightArm->GetCurrentOrientation(l));

        this->yuan_pubber_.publish(yuan_msg_);
        ros::spinOnce();

        yuan_msg_.data.clear();

        std::this_thread::sleep_for(std::chrono::milliseconds(int(delay_time_)));
    }
}

void Carrot::YuanSub()
{
    yuan_subber_joy_ = this->n_.subscribe("/Joy_control", 1, &Carrot::YuanCallback, this);
    yuan_subber_all_ = this->n_.subscribe("/Every_control", 1, &Carrot::YuanCallback, this);
    ros::spin();
}

void Carrot::YuanCallback(const std_msgs::Int16::ConstPtr &yuan_msg)
{
    /* Update state */
    yuan_state_ = yuan_msg->data;

    /* Default State */
    if (yuan_state_ == 3)
    {
    }
    /* Start */
    else if (yuan_state_ == 0)
    {
        CRobot->CLeftArm->Stop();
        CRobot->CRightArm->Stop();
        CRobot->CMobile->CWheel->Stop();
    }
    /* Stop */
    else if (yuan_state_ ==1)
    {
        CRobot->CLeftArm->Start();
        CRobot->CRightArm->Start();
        CRobot->CMobile->CSteering->Start();
    }
    /* Reconnection */
    else if (yuan_state_ == 2)
        CRobot->Reconnect();
}
