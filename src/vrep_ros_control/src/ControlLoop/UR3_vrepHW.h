#pragma once

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>


namespace UR3
{


enum UR3JointsEnum
{
    ELBOW_JOINT = 0,
    SHOULDER_LIFT_JOINT,
    SHOULDER_PAN_JOINT,
    WRIST_1_JOINT,
    WRIST_2_JOINT,
    WRIST_3_JOINT,

    UR3_JOINTS_NUM
};


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief This is the hardware interface for UR3 simulated in vrep.
class UR3_vrepHW : public hardware_interface::RobotHW
{
public:
    UR3_vrepHW();

    bool init();

    bool read();
    bool write();

protected:
    static std::string sm_jointsName[UR3_JOINTS_NUM];

    // Vrep handles.
    int m_vrepJointsHandle[UR3_JOINTS_NUM];

    // Interfaces.
    double m_cmd[UR3_JOINTS_NUM];
    double m_pos[UR3_JOINTS_NUM];
    double m_vel[UR3_JOINTS_NUM];
    double m_eff[UR3_JOINTS_NUM];

    hardware_interface::JointStateInterface m_jointState_interface;
    hardware_interface::VelocityJointInterface m_jointVelocity_interface;

    void registerHardwareInterfaces();
};


} // namespace UR3.
