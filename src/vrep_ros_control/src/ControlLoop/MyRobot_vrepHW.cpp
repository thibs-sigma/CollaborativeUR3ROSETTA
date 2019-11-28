#include "MyRobot_vrepHW.h"

#include "../v_repLib.h"

#include <string>
#include <iostream>


namespace MR
{


std::string MyRobot_vrepHW::sm_jointsName[MR_JOINTS_NUM] = {
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
    "UR3_2_shoulder_pan_joint",
    "UR3_2_shoulder_lift_joint",
    "UR3_2_elbow_joint",
    "UR3_2_wrist_1_joint",
    "UR3_2_wrist_2_joint",
    "UR3_2_wrist_3_joint"
};


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
MyRobot_vrepHW::MyRobot_vrepHW() :
    hardware_interface::RobotHW()
{
    // Init arrays m_cmd[], m_pos[], m_vel[], m_eff[].
    for (int i = 0; i < MR_JOINTS_NUM; ++i)
    {
        m_cmd[i] = 0.0;
        m_pos[i] = 0.0;
        m_vel[i] = 0.0;
        m_eff[i] = 0.0;
    }

    // Init and get handles of the joints to control.
    for (int i = 0; i < MR_JOINTS_NUM; ++i)
        m_vrepJointsHandle[i] = -1;

    // Register joint interfaces.
    registerHardwareInterfaces();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool MyRobot_vrepHW::init()
{
    // Get joint handles.
    for (int i = 0; i < MR_JOINTS_NUM; ++i)
    {
        int vrepJointsHandle = simGetObjectHandle(sm_jointsName[i].c_str());

        if (vrepJointsHandle == -1)
        {
            ROS_ERROR_STREAM("MR robot interface not able to get handle for '" << sm_jointsName[i] << "'." << std::endl);

            return false;
        }

        m_vrepJointsHandle[i] = vrepJointsHandle;
    }

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MyRobot_vrepHW::registerHardwareInterfaces()
{
    for (int i = 0; i < MR_JOINTS_NUM; ++i)
    {
        // Joint state interface.
        hardware_interface::JointStateHandle jointStateHandle(sm_jointsName[i], &m_pos[i], &m_vel[i], &m_eff[i]);
        m_jointState_interface.registerHandle(jointStateHandle);

        // Joint command interface (in UR case this is a position interface).
        hardware_interface::JointHandle jointPositionHandle(jointStateHandle, &m_cmd[i]);
        m_jointPosition_interface.registerHandle(jointPositionHandle);
    }

    registerInterface(&m_jointState_interface);
    registerInterface(&m_jointPosition_interface);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool MyRobot_vrepHW::read()
{
    for (int i = 0; i < MR_JOINTS_NUM; ++i)
    {
        float pos,
              vel,
              eff;

        if (simGetJointPosition(m_vrepJointsHandle[i], &pos) == -1 ||
            simGetObjectFloatParameter(m_vrepJointsHandle[i], 2012, &vel) == -1 || // Velocity.
            simGetJointForce(m_vrepJointsHandle[i], &eff) == -1)
        {
            ROS_ERROR_STREAM("MR robot interface not able to get state for '" << sm_jointsName[i] << "'." << std::endl);

            return false;
        }

        m_pos[i] = pos;
        m_vel[i] = vel;
        m_eff[i] = eff;
    }

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool MyRobot_vrepHW::write()
{
    for (int i = 0; i < MR_JOINTS_NUM; ++i)
    {
        if (simSetJointTargetPosition(m_vrepJointsHandle[i], m_cmd[i]) == -1)
        {
            ROS_ERROR_STREAM("MR robot interface not able to get state for '" << sm_jointsName[i] << "'." << std::endl);

            return false;
        }
    }

    return true;
}

} // namespace MR.
