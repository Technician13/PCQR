#include "bodyStateEstimate.hpp"
#include <iostream>

using namespace quadruped;

BodyStateEstimate::BodyStateEstimate(void)
{
	velocityEstimate = new VelocityEstimate();

	position_x_ = 0.0;
    position_y_ = 0.0;
    position_z_ = 0.0;
    cnt = 0;

	z_offset = 0.0;

	std::cout << "BodyStateEstimate Init" << std::endl;
}

BodyStateEstimate::~BodyStateEstimate()
{
	delete velocityEstimate;
}


void BodyStateEstimate::GetRobotBodyPos(bool LF_CONTACT_, bool LR_CONTACT_, bool RF_CONTACT_, bool RR_CONTACT_,
                                		Vec3<double> lf_endfoot_vel_, Vec3<double> lr_endfoot_vel_, Vec3<double> rf_endfoot_vel_, Vec3<double> rr_endfoot_vel_,
                                		Vec3<double> lf_endfoot_, Vec3<double> lr_endfoot_, Vec3<double> rf_endfoot_, Vec3<double> rr_endfoot_,
                                		DVec<double> p_f_k, DVec<double> p_f_k_1, Vec3<double> p_actul_body_acc)
{
	actul_body_vel = velocityEstimate->CalVelocity(LF_CONTACT_, LR_CONTACT_, RF_CONTACT_, RR_CONTACT_,
                				                   lf_endfoot_vel_, lr_endfoot_vel_, rf_endfoot_vel_, rr_endfoot_vel_,
                				                   lf_endfoot_, lr_endfoot_, rf_endfoot_, rr_endfoot_,
                				                   p_f_k, p_f_k_1, rotation, actul_body_angularVel, p_actul_body_acc, actul_body_pos);
								 
	if(cnt < 6)
    {
        cnt++;
    }
    else if(cnt < 7)
    {
        cnt++;
        z_offset = -0.25 * (lf_endfoot_[2] + lr_endfoot_[2] + rf_endfoot_[2] + rr_endfoot_[2]);
    }
    else
    {
        position_x_ = position_x_ + actul_body_vel[0] * 0.005;
        position_y_ = position_y_ + actul_body_vel[1] * 0.005;
        position_z_ = position_z_ + actul_body_vel[2] * 0.005;

        bodyPosWorld << position_x_ , position_y_ , position_z_+z_offset;
    }

	actul_body_pos = bodyPosWorld;				 
}


void BodyStateEstimate::GetRobotBodyAngle(void)
{
	imu_t imu={0};
	imu=GetRobotEuler();

	actul_body_angle(0)=imu.roll;
	actul_body_angle(1)=imu.pitch;
	actul_body_angle(2)=imu.yaw;

	actul_body_angularVel(0) = imu.omega_roll;
	actul_body_angularVel(1) = imu.omega_pitch;
	actul_body_angularVel(2) = imu.omega_yaw;
}

void BodyStateEstimate::GetRobotMsg(LegStateEstimate* p_lf_legState,LegStateEstimate* p_lr_legState,
									LegStateEstimate* p_rf_legState,LegStateEstimate* p_rr_legState,
									Vec12<double> p_f_k,Vec12<double> p_f_k_1,Vec3<double> p_actul_body_acc)
{
	Vec3<double> lf_offset;
	lf_offset << 0.5 * BODY_L , 0.5 * BODY_W_F , (double)0.0;
	Vec3<double> lr_offset;
	lr_offset << -0.5 * BODY_L , 0.5 * BODY_W_R , (double)0.0;
	Vec3<double> rf_offset;
	rf_offset << 0.5 * BODY_L , -0.5 * BODY_W_F , (double)0.0;
	Vec3<double> rr_offset;
	rr_offset << -0.5 * BODY_L , -0.5 * BODY_W_R , (double)0.0;

	GetRobotBodyAngle();

	rotation=Common::RotationMatrix(actul_body_angle);

	GetRobotBodyPos(p_lf_legState->contact_state,
					p_lr_legState->contact_state,
					p_rf_legState->contact_state,
					p_rr_legState->contact_state,
					p_lf_legState->endfoot_vel_hip,
					p_lr_legState->endfoot_vel_hip,
					p_rf_legState->endfoot_vel_hip,
					p_rr_legState->endfoot_vel_hip,
					(p_lf_legState->endfoot_hip)+lf_offset,
					(p_lr_legState->endfoot_hip)+lr_offset,
					(p_rf_legState->endfoot_hip)+rf_offset,
					(p_rr_legState->endfoot_hip)+rr_offset,
					p_f_k,
					p_f_k_1,
					p_actul_body_acc);

}
