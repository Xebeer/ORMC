#include "vector"

enum RobotType
{
    RR,
    SCARA,
    ArticulatedArm,
    Stewart,
    Delta,
    CNC
};

enum KinematicsType
{
    Serial,
    Parallel,
    Hybrid
};

enum JointType
{
    Revolute,
    Prismatic
};

struct Joint
{
    public:

    Joint(JointType type, double lower_limit, double upper_limit, double home, double max_speed, double max_acceleration, double max_jerk, double rad2pulse)
    {
        this->type = type;
        this->lower_limit = lower_limit;
        this->upper_limit = upper_limit;
        this->home = home;
        this->max_speed = max_speed;
        this->max_acceleration = max_acceleration;
        this->max_jerk = max_jerk;
        this->q2pulse = q2pulse;
        this->q = 0;
    };

    Joint()
    {
        this->type = Revolute;
        this->lower_limit = 0;
        this->upper_limit = 0;
        this->home = 0;
        this->max_speed = 0;
        this->max_acceleration = 0;
        this->max_jerk = 0;
        this->q2pulse = 0;
        this->q = 0;
    };

    int getPulse()
    {
        return (int)(q * q2pulse);
    }

    int getPulse(double& q)
    {
        return (int) (q * q2pulse);
    }

    JointType type = Revolute;
    double lower_limit;
    double upper_limit;
    double home;
    double max_speed;
    double max_acceleration;
    double max_jerk;
    double q2pulse;
    double q;
};

class Robot
{
    public:
    Robot(RobotType type_)
    {
        robot_type = type_;

        switch (type_)
        {
        case RR:
        number_of_axes = 2;
        kinematics_type = Serial;
            break;

        case SCARA:
        number_of_axes = 4;
        kinematics_type = Serial;
            break;
        
        case ArticulatedArm:
        number_of_axes = 6;
        kinematics_type = Serial;
            break;
        
        case Stewart:
        number_of_axes = 6;
        kinematics_type = Parallel;
            break;
        
        case Delta:
        number_of_axes = 4;
        kinematics_type = Hybrid;
            break;
        
        case CNC:
        number_of_axes = 3;
        kinematics_type = Serial;
            break;
        
        default:
            break;
        }

        for (int i = 0; i < number_of_axes; i++)
        {
            joints.push_back(Joint());
        }
    }

    int number_of_axes = 0;
    RobotType robot_type = RobotType::RR;
    KinematicsType kinematics_type = KinematicsType::Serial;

    std::vector<Joint> joints;

}