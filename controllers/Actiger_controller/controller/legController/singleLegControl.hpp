#ifndef SINGLELEGCONTROL_HPP
#define SINGLELEGCONTROL_HPP

#include "cppTypes.h"
#include "robotModel.hpp"

namespace quadruped 
{
    class SingleLeg
    {
    private:

    protected:

    public:
        explicit SingleLeg(LegStateEstimate* p_legStste, Parameters* p_para);
        virtual ~SingleLeg(void);

        legTorque_t SingleStanceControl(Mat3<double> p_rotation, Vec3<double> p_grav, Vec3<double> coriolis, Vec3<double> p_reactForce);
        legTorque_t SingleSwingControl(Mat3<double> p_rotation,Vec3<double> p_grav, Vec3<double> coriolis, Vec3<double> p_desPos,Vec3<double> p_desVel);

        Vec3<double> torqueFeedForward;
        Vec3<double> qDes;
        Vec3<double> qdDes;

        LegStateEstimate* legStste;
        Parameters* para;
    };
}

#endif // !SINGLELEGCONTROL_HPP