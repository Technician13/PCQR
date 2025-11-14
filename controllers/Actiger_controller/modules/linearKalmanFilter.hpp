#ifndef LINEARKALMANFILTER_HPP
#define LINEARKALMANFILTER_HPP

#include "cppTypes.h"
#include "commonFunction.hpp"
#include <array>

namespace quadruped
{
    class LinearKalmanFilter
    {
    private:

    protected:

    public:
        explicit LinearKalmanFilter(void);
		virtual ~LinearKalmanFilter(void);

        void Init();
        void Run(Vec3<double> p_bodyEuler , Vec3<double> p_aBody , Vec3<double> p_actul_body_angularVel ,
                 Vec3<double> p_lf_hip_pos_body , Vec3<double> p_lr_hip_pos_body , Vec3<double> p_rf_hip_pos_body , Vec3<double> p_rr_hip_pos_body ,
                 Vec3<double> p_lf_endfoot_hip , Vec3<double> p_lr_endfoot_hip , Vec3<double> p_rf_endfoot_hip , Vec3<double> p_rr_endfoot_hip ,
                 Vec3<double> p_lf_endfoot_vel_hip , Vec3<double> p_lr_endfoot_vel_hip , Vec3<double> p_rf_endfoot_vel_hip , Vec3<double> p_rr_endfoot_vel_hip ,
                 bool p_lf_contact_state , bool p_lr_contact_state , bool p_rf_contact_state , bool p_rr_contact_state);
        
        double dt;
        Vec18<double> xhat;
        Mat18<double> A;
        Mat18x3<double> B;
        Mat18<double> Q_init;
        Mat18<double> Q;
        Mat18<double> P;
        Mat28x18<double> H;
        Vec28<double> z;
        Mat28<double> R_init;
        Mat28<double> R;
        Vec12<double> ps;
        Vec12<double> vs;
        Vec4<double> pzs;
    };
}

#endif // !LINEARKALMANFILTER_HPP