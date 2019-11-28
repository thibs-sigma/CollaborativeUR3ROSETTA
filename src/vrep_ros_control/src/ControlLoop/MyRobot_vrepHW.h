#pragma once

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>


namespace MR
{


enum MrJointsEnum
{
    SHOULDER_PAN_JOINT = 0,
    SHOULDER_LIFT_JOINT,
    ELBOW_JOINT,
    WRIST_1_JOINT,
    WRIST_2_JOINT,
    WRIST_3_JOINT,
    
    UR3_2_SHOULDER_PAN_JOINT,
    UR3_2_SHOULDER_LIFT_JOINT,
    UR3_2_ELBOW_JOINT,
    UR3_2_WRIST_1_JOINT,
    UR3_2_WRIST_2_JOINT,
    UR3_2_WRIST_3_JOINT,

    MR_JOINTS_NUM
};


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief This is the hardware interface for MyRobot simulated in vrep.
class MyRobot_vrepHW : public hardware_interface::RobotHW
{
public:
    MyRobot_vrepHW();

    bool init();

    bool read();
    bool write();

protected:
    static std::string sm_jointsName[MR_JOINTS_NUM];

    // Vrep handles.
    int m_vrepJointsHandle[MR_JOINTS_NUM];

    // Interfaces.
    double m_cmd[MR_JOINTS_NUM];
    double m_pos[MR_JOINTS_NUM];
    double m_vel[MR_JOINTS_NUM];
    double m_eff[MR_JOINTS_NUM];

    hardware_interface::JointStateInterface m_jointState_interface;
    hardware_interface::PositionJointInterface m_jointPosition_interface;

    void registerHardwareInterfaces();
};


} // namespace MR.
