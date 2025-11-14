#include "legStateEstimate.hpp"
#include <iostream>
 
using namespace quadruped;


LegStateEstimate::LegStateEstimate(int p_legNum)
{
	legNum = p_legNum;
	L0 = 0.0;
	contact_state = 1;

	if (p_legNum == LF || p_legNum == LR)
		L0 = L0_L;
	else
		L0 = L0_R;

	switch(p_legNum)
	{
		case LF:
			hip_pos_body << 0.33,0.07,0.0; 
		break;

		case LR:
			hip_pos_body << -0.33,0.055,0.0; 	
		break;

		case RF:
			hip_pos_body << 0.33,-0.07,0.0;
		break;

		case RR:
			hip_pos_body << -0.33,-0.055,0.0;
		break;

		default:break;
	}

	std::cout << "legStateEstimate Init" << std::endl;
}


LegStateEstimate::~LegStateEstimate(void) {}


void LegStateEstimate::GetLegMsg(legMsg_t p_legMsg)
{
	Resize();
	GetJointMsg(p_legMsg);
	GetJacobian();
	CaluEndfootMsg();
}


void LegStateEstimate::Resize(void)
{
	endfoot_hip.setZero();
	endfoot_vel_hip.setZero();
	joint_angle.setZero();
	joint_anglularVel.setZero();
}


void LegStateEstimate::GetJacobian(void)
{
	jacobian(0, 0) = 0.0;
	jacobian(0, 1) = (-L1 * cos(joint_angle(1)) -L2 * cos(joint_angle(1) + joint_angle(2)) );
	jacobian(0, 2) = (-L2 * cos(joint_angle(1) + joint_angle(2)));
					 	
	jacobian(1, 0) = (cos(joint_angle(0)) * (L2 * cos(joint_angle(1) + joint_angle(2)) + L1 * cos(joint_angle(1))) - L0 * sin(joint_angle(0)));
	jacobian(1, 1) = (-sin(joint_angle(0)) * (L2 * sin(joint_angle(1) + joint_angle(2)) + L1 * sin(joint_angle(1))));
	jacobian(1, 2) = (-L2 * sin(joint_angle(1) + joint_angle(2)) * sin(joint_angle(0)));
					
	jacobian(2, 0) = (sin(joint_angle(0)) * (L2 * cos(joint_angle(1) + joint_angle(2)) + L1 * cos(joint_angle(1))) + L0 * cos(joint_angle(0)));
	jacobian(2, 1) = (cos(joint_angle(0)) * (L2 * sin(joint_angle(1) + joint_angle(2)) + L1 * sin(joint_angle(1))));
	jacobian(2, 2) = (L2 * sin(joint_angle(1) + joint_angle(2)) * cos(joint_angle(0)));
}


void LegStateEstimate::CaluEndfootMsg(void)
{
	endfoot_hip(0) = (-(L1 * sin(joint_angle(1)) + L2 * sin(joint_angle(1) + joint_angle(2))));
	endfoot_hip(1) = (L0 * cos(joint_angle(0)) + sin(joint_angle(0)) * (L1 * cos(joint_angle(1)) + L2 * cos(joint_angle(1) + joint_angle(2))));
	endfoot_hip(2) = (L0 * sin(joint_angle(0)) - cos(joint_angle(0)) * (L1 * cos(joint_angle(1)) + L2 * cos(joint_angle(1) + joint_angle(2))));
	
	endfoot_vel_hip = jacobian * joint_anglularVel;
}


void LegStateEstimate::GetJointMsg(legMsg_t p_legMsg)
{
	joint_angle(0) = (p_legMsg.shoulder_pos);
	joint_angle(1) = (p_legMsg.hip_pos);
	joint_angle(2) = (p_legMsg.knee_pos);
	
	joint_anglularVel(0)= (p_legMsg.shoulder_vel);
	joint_anglularVel(1)= (p_legMsg.hip_vel);
	joint_anglularVel(2)= (p_legMsg.knee_vel);

	if(handle.standReady == 1)
	{
		if(handle.stepOver == 0)
		{	
			if((handle.is_lf_rr_stand == 0 && (legNum == 0 || legNum == 3)) || (handle.is_lf_rr_stand == 1 && (legNum == 1 || legNum == 2)))
				contact_state =  0;
			else
				contact_state =  (bool)(p_legMsg.contact);
		}
		else
			contact_state =  (bool)(p_legMsg.contact);
	}	
	else
		contact_state = 1;
}


void LegStateEstimate::GetJdot(void)
{
	Mat3<double> jacobian_dot = Mat3<double>::Zero();
	jacobian_dot(0, 0) = 0.0;

	jacobian_dot(0, 1) = (L1 * sin(joint_angle(1)) * joint_anglularVel(1) \
		                     + L2 * sin(joint_angle(1) + joint_angle(2)) * (joint_anglularVel(1) + joint_anglularVel(2)));

	jacobian_dot(0, 2) = (L2 * sin(joint_angle(1) + joint_angle(2)) * (joint_anglularVel(1) + joint_anglularVel(2)));

	jacobian_dot(1, 0) = (- L0 * cos(joint_angle(0))* joint_anglularVel(0) \
		                     - sin(joint_angle(0)) * (L2 * cos(joint_angle(1) + joint_angle(2)) + L1 * cos(joint_angle(1))) * joint_anglularVel(0)\
		                     - cos(joint_angle(0)) * (L1 * sin(joint_angle(1)) * joint_anglularVel(1) + L2 * sin(joint_angle(1) + joint_angle(2)) * (joint_anglularVel(1) + joint_anglularVel(2))));

	jacobian_dot(1, 1) = (- cos(joint_angle(0)) * (L2 * sin(joint_angle(1) + joint_angle(2)) + L1 * sin(joint_angle(1))) * joint_anglularVel(0)\
		                     - sin(joint_angle(0)) * (L1 * cos(joint_angle(1)) * joint_anglularVel(1) + L2 * cos(joint_angle(1) + joint_angle(2)) * (joint_anglularVel(1) + joint_anglularVel(2))));

	jacobian_dot(1, 2) = (- cos(joint_angle(0)) * (L2 * sin(joint_angle(1) + joint_angle(2))) * joint_anglularVel(0)\
		                     - sin(joint_angle(0)) * (L2 * cos(joint_angle(1) + joint_angle(2)) * (joint_anglularVel(1) + joint_anglularVel(2))));

	jacobian_dot(2, 0) = (- L0 * sin(joint_angle(0)) * joint_anglularVel(0) \
		                     + cos(joint_angle(0)) * (L2 * cos(joint_angle(1) + joint_angle(2)) + L1 * cos(joint_angle(1))) * joint_anglularVel(0)\
		                     - sin(joint_angle(0)) * (L1 * sin(joint_angle(1)) * joint_anglularVel(1) + L2 * sin(joint_angle(1) + joint_angle(2)) * (joint_anglularVel(1) + joint_anglularVel(2))));

	jacobian_dot(2, 1) = (- sin(joint_angle(0)) * (L2 * sin(joint_angle(1) + joint_angle(2)) + L1 * sin(joint_angle(1))) * joint_anglularVel(0)\
		                     + cos(joint_angle(0)) * (L1 * cos(joint_angle(1)) * joint_anglularVel(1) + L2 * cos(joint_angle(1) + joint_angle(2)) * (joint_anglularVel(1) + joint_anglularVel(2))));

	jacobian_dot(2, 2) = (- sin(joint_angle(0)) * (L2 * sin(joint_angle(1) + joint_angle(2))) * joint_anglularVel(0)\
		                     + sin(joint_angle(0)) * (L2 * cos(joint_angle(1) + joint_angle(2)) * (joint_anglularVel(1) + joint_anglularVel(2))));


	jdqd = jacobian_dot * joint_anglularVel;
}