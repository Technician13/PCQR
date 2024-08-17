#ifndef MODULE_HPP
#define MODULE_HPP

#include "parameters.hpp"
#include "legControl.hpp"
#include "robotModel.hpp"

namespace quadruped
{
    class Module
    {
    public:
        explicit Module(RobotModel* p_robotModel, Parameters* p_para, LegControl* p_legControl);
        virtual ~Module(void);

        LegControl* legControl;
        RobotModel* robotModel;
        Parameters* para;

        void ResetLeg(void);
        void SwingTest(Vec3<double> p_pos_des,Vec3<double> p_vel_des);      
        void TorqueLimit(Vec3<double>& p_torque);
 
    };




}











#endif
