#ifndef VELOCITYESTIMATE_HPP
#define VELOCITYESTIMATE_HPP 

#include "KalmanFilterForVE.hpp"
#include "cppTypes.h"
#include "parameters.hpp"
#include <vector>

class VelocityEstimate
{
public:
    VelocityEstimate(void);
    virtual ~VelocityEstimate();
    
    void vector2crossmatrix(const Vec3<double> & vector_input, Mat3<double> & matrix_input);
    Vec3<double> CalVelocity(bool LF_CONTACT, bool LR_CONTACT, bool RF_CONTACT, bool RR_CONTACT, 
                             Vec3<double> lf_endfoot_vel, Vec3<double> lr_endfoot_vel, Vec3<double> rf_endfoot_vel, Vec3<double> rr_endfoot_vel,
                             Vec3<double> lf_endfoot, Vec3<double> lr_endfoot, Vec3<double> rf_endfoot, Vec3<double> rr_endfoot,
                             DVec<double> f_k, DVec<double> f_k_1, Mat3<double> rotation,
                             Vec3<double> actul_body_angularVel, Vec3<double> actul_body_acc, Vec3<double> actul_body_pos);
    
    KalmanFilterForVE *KF = new KalmanFilterForVE((double)BODY_MASS, period);
    std::vector<bool> contactList{1,1,1,1};
    int contactNum;
    Mat3<double> omega_cross;
    Vec3<double> b_xdot_lf;
    Vec3<double> b_xdot_lr;
	Vec3<double> b_xdot_rf;
	Vec3<double> b_xdot_rr;
    double LF_probabilistic;
	double LR_probabilistic;
	double RF_probabilistic;
	double RR_probabilistic;
    Vec3<double> velocityPrimary;
    Vec3<double> world_velocityPrimary;
    Vec3<double> a_k;
    Vec3<double> a_k_1;
    Vec3<double> xdot;
    Vec3<double> v_k_1;
    Vec3<double> p_k;
    Vec3<double> p_k_1;

    double period;

    int KalmanCnt;
    double beta;
	double beta0;
    bool LF_CONTACT_LAST;
    bool LR_CONTACT_LAST;
    bool RF_CONTACT_LAST;
    bool RR_CONTACT_LAST;
};

#endif //VELOCITYESTIMATE_HPP