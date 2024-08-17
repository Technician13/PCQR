/**************************************************************************
Copyright:Action
Author: 
Date:2021-03-27
Description:Velocity Estimate of AcTiger
**************************************************************************/

#ifndef VELOCITYESTIMATE_HPP
#define VELOCITYESTIMATE_HPP 

#include "KalmanFilterForVE.hpp"
#include "cppTypes.h"
#include "parameters.hpp"
#include <vector>

//**************************************************************************
//these parameters should be replaced by real value or parameters in project
//robot mass

//state of footend:if stance,1;if swing,0.
//*     bool LF_CONTACT;
//*     bool LR_CONTACT;
//*     bool RF_CONTACT;
//*     bool RR_CONTACT;
//current footend force
//*     Eigen::VectorXd f_k;
//last footend force, should be update by f_k_1 = f_k after one iteration
//*     Eigen::VectorXd f_k_1;
//*     Eigen::VectorXd lf_endfoot_vel;
//*     Eigen::VectorXd lf_endfoot;
//*     Eigen::VectorXd lr_endfoot_vel;
//*     Eigen::VectorXd lr_endfoot;
//*     Eigen::VectorXd rf_endfoot_vel;
//*     Eigen::VectorXd rf_endfoot;
//*     Eigen::VectorXd rr_endfoot_vel;
//*     Eigen::VectorXd rr_endfoot;
//*     Eigen::MatrixXd rotation;
//*     Eigen::VectorXd actul_body_angularVel;
//*     Eigen::VectorXd actul_body_acc;
//*     Eigen::VectorXd actul_body_pos;
//**************************************************************************

class VelocityEstimate
{
public:
    VelocityEstimate(void);
    virtual ~VelocityEstimate();
    
    void vector2crossmatrix(const Vec3<double> & vector_input, Mat3<double> & matrix_input);
    Vec3<double> CalVelocity(bool LF_CONTACT, \
                         bool LR_CONTACT, \
                         bool RF_CONTACT, \
                         bool RR_CONTACT, \
                         Vec3<double> lf_endfoot_vel, \
                         Vec3<double> lr_endfoot_vel, \
                         Vec3<double> rf_endfoot_vel, \
                         Vec3<double> rr_endfoot_vel, \
                         Vec3<double> lf_endfoot, \
                         Vec3<double> lr_endfoot, \
                         Vec3<double> rf_endfoot, \
                         Vec3<double> rr_endfoot, \
                         DVec<double> f_k, \
                         DVec<double> f_k_1, \
                         Mat3<double> rotation, \
                         Vec3<double> actul_body_angularVel, \
                         Vec3<double> actul_body_acc, \
                         Vec3<double> actul_body_pos);
    
    KalmanFilterForVE *KF = new KalmanFilterForVE((double)BODY_MASS, period);
    //footend contact list
    std::vector<bool> contactList{1,1,1,1};
    //a cnt used to determine whether to start the KF
    int contactNum;
    //robot's angualr velocity cross matrix
    Mat3<double> omega_cross;
    //lf footend velocity contribution under body frame
    Vec3<double> b_xdot_lf;
    //lr footend velocity contribution under body frame
    Vec3<double> b_xdot_lr;
    //rf footend velocity contribution under body frame
	Vec3<double> b_xdot_rf;
    //rr footend velocity contribution under body frame
	Vec3<double> b_xdot_rr;
    //probability of lf footend contact to the ground
    double LF_probabilistic;
    //probability of lr footend contact to the ground
	double LR_probabilistic;
    //probability of rf footend contact to the ground
	double RF_probabilistic;
    //probability of rr footend contact to the ground
	double RR_probabilistic;
    //velocity that without filter
    Vec3<double> velocityPrimary;
    //velocity that without filter under world frame
    Vec3<double> world_velocityPrimary;
    //current robot's acceleration
    Vec3<double> a_k;
    //last robot's acceleration
    Vec3<double> a_k_1;
    //equal to world_velocityPrimary
    Vec3<double> xdot;
    //current robot's velocity
    Vec3<double> v_k_1;
    //current robot's position
    Vec3<double> p_k;
    //last robot's position
    Vec3<double> p_k_1;


    double period;

    int KalmanCnt;
    //coefficients used to calculate the probability of footend contact to the ground
    double beta;
	double beta0;
    //last contact status
    bool LF_CONTACT_LAST;
    bool LR_CONTACT_LAST;
    bool RF_CONTACT_LAST;
    bool RR_CONTACT_LAST;
};

#endif //VELOCITYESTIMATE_HPP