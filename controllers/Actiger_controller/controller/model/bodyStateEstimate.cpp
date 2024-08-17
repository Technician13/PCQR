/**
 * @file bodyStateEstimate.cpp
 * @brief Actiger机体状态信息类BodyStateEstimate
 * @author Bin & Tech13
 * @version 1.1
 * @date 2021-08-19
 * 
 * @copyright  Copyright  (c)  2021  Action
 * 
 */

#include "bodyStateEstimate.hpp"
#include <iostream>

using namespace quadruped;

/**
 * @brief BodyStateEstimate类的构造函数，完成所有对象成员的内存分配以及数据成员的初值装载       
 */
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

/**
 * @brief BodyStateEstimate类的析构函数，释放申请的内存
 */
BodyStateEstimate::~BodyStateEstimate()
{
	delete velocityEstimate;
}


void BodyStateEstimate::GetRobotBodyPos(bool LF_CONTACT_,
                                		   bool LR_CONTACT_,
                                		   bool RF_CONTACT_,
                                		   bool RR_CONTACT_,
                                		   Vec3<double> lf_endfoot_vel_,
                                		   Vec3<double> lr_endfoot_vel_,
                                		   Vec3<double> rf_endfoot_vel_,
                                		   Vec3<double> rr_endfoot_vel_,
                                		   Vec3<double> lf_endfoot_,
                                		   Vec3<double> lr_endfoot_,
                                		   Vec3<double> rf_endfoot_,
                                		   Vec3<double> rr_endfoot_,
                                		   DVec<double> p_f_k,
                                		   DVec<double> p_f_k_1,
                                		   Vec3<double> p_actul_body_acc)
{

	// double contactNum = (double)LF_CONTACT_ + (double)LR_CONTACT_ + (double)RF_CONTACT_ + (double)RR_CONTACT_;
	// actul_body_pos =  (rotation.transpose() * lf_endfoot_ * LF_CONTACT_ + rotation.transpose() * lr_endfoot_ * LR_CONTACT_ + rotation.transpose() * rf_endfoot_ * RF_CONTACT_ + rotation.transpose() * rr_endfoot_ * RR_CONTACT_) / (double)contactNum;
	// actul_body_pos = -actul_body_pos;

	// actul_body_pos(0) = actul_body_pos(0) + (double)(actul_body_vel(0) * TIME_STEP);
	// actul_body_pos(1) = actul_body_pos(1) + (double)(actul_body_vel(1) * TIME_STEP);
	// actul_body_pos(2) = actul_body_pos(2) + (double)(actul_body_vel(2) * TIME_STEP);

	
	// std::cout<<"%%%%%%%%%%%%%%%%%%%%%%"<<lf_endfoot_[2]<<"  "<<lr_endfoot_[2]<<"  "<<rf_endfoot_[2]<<"  "<<rr_endfoot_[2]<<std::endl;
	actul_body_vel = velocityEstimate->CalVelocity(LF_CONTACT_,
                				 LR_CONTACT_,
                				 RF_CONTACT_,
                				 RR_CONTACT_,
                				 lf_endfoot_vel_,
                				 lr_endfoot_vel_,
                				 rf_endfoot_vel_,
                				 rr_endfoot_vel_,
                				 lf_endfoot_,
                				 lr_endfoot_,
                			     rf_endfoot_,
                				 rr_endfoot_,
                				 p_f_k,
                				 p_f_k_1,
                				 rotation,
                				 actul_body_angularVel,
                				 p_actul_body_acc,
                				 actul_body_pos);
								 

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
	// std::cout<<"######################## "<<z_offset<<" #########################"<<std::endl;
	
								 
}

/**
 * @brief 将IMU的数据装载到BodyStateEstimate数据成员
 */
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
