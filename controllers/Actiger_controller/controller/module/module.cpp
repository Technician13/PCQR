#include "module.hpp"


using namespace quadruped;


Module::Module(RobotModel* p_robotModel, Parameters* p_para, LegControl* p_legControl)
{
    legControl = p_legControl;
    robotModel = p_robotModel;
    para = p_para;
}


Module::~Module(void)
{

}

void Module::ResetLeg(void)
{
   
}


void Module::SwingTest(Vec3<double> p_pos_des,Vec3<double> p_vel_des)
{
    Vec3<double> torque = Vec3<double>::Zero();
}


void Module::TorqueLimit(Vec3<double>& p_torque)
{
    #define MAX_TORQUE_SHOULDER (0.4)
    #define MAX_TORQUE_HIP (0.4)
    #define MAX_TORQUE_KNEE (0.5)
    
    if(isnan(p_torque(0)))
        p_torque(0) = 0.0;
    if(isnan(p_torque(1)))
        p_torque(1) = 0.0;
    if(isnan(p_torque(2)))
        p_torque(2) = 0.0;

    if(abs(p_torque(0)) > MAX_TORQUE_SHOULDER)
        p_torque(0) = p_torque(0)/abs(p_torque(0))*MAX_TORQUE_SHOULDER;
    
    if(abs(p_torque(1)) > MAX_TORQUE_HIP)
        p_torque(1) = p_torque(1)/abs(p_torque(1))*MAX_TORQUE_HIP;

    if(abs(p_torque(2)) > MAX_TORQUE_KNEE)
        p_torque(2) = p_torque(2)/abs(p_torque(2))*MAX_TORQUE_KNEE;


}