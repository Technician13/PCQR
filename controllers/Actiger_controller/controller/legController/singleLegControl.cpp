/**
 * @file singleLegControl.cpp
 * @brief Actiger单腿控制器SingleLeg
 * @author Bin & Tech13
 * @version 1.1
 * @date 2021-08-20
 * 
 * @copyright  Copyright  (c)  2021  Action
 * 
 */

#include "singleLegControl.hpp"
#include <iostream>

using namespace quadruped;

/**
 * @brief SingleLeg类的构造函数，完成所有对象成员的初始化
 * @param  p_legStste  LegStateEstimate类的实例化对象，存储机器人单腿状态信息，用来初始化对象成员legStste    
 * @param  p_para  Parameters类的实例化对象，存储一些被频繁使用的参数，用来初始化对象成员para          
 */
SingleLeg::SingleLeg(LegStateEstimate *p_legStste, Parameters *p_para)
{
	legStste = p_legStste;
	para = p_para;

	std::cout << "SingleLeg Init" << std::endl;
}

/**
 * @brief SingleLeg类的析构函数
 */
SingleLeg::~SingleLeg() {}

/**
 * @brief Actiger支撑腿控制函数
 * @param  p_rotation  惯性坐标系到机体坐标系的旋转矩阵       
 * @param  p_grav  拉格朗日动力学方程中的重力项            
 * @param  coriolis  拉格朗日动力学方程中的离心力-哥氏力项        
 * @param  p_reactForce  Actiger模型预测控制器计算得到的地面反作用力[GRF]    
 * @return legTorque_t 
 */
legTorque_t SingleLeg::SingleStanceControl(Mat3<double> p_rotation, Vec3<double> p_grav, Vec3<double> coriolis, Vec3<double> p_reactForce)
{
	legTorque_t return_torque = {0};
	Vec3<double> torque = Vec3<double>::Zero();

	torque = -legStste->jacobian.transpose() * p_rotation * p_reactForce + p_grav + coriolis;

	return_torque.shoulder_torque = torque(0);
	return_torque.hip_torque = torque(1);
	return_torque.knee_torque = torque(2);

	// if(return_torque.shoulder_torque > 20.0)
	// {
	// 	return_torque.shoulder_torque = 20.0;
	// }
	// else if(return_torque.shoulder_torque < -20.0)
	// {
	// 	return_torque.shoulder_torque = -20.0;
	// }

	// if(return_torque.hip_torque > 50.0)
	// {
	// 	return_torque.hip_torque = 50.0;
	// }
	// else if(return_torque.hip_torque < -50.0)
	// {
	// 	return_torque.hip_torque = -50.0;
	// }

	// if(return_torque.knee_torque > 50.0)
	// {
	// 	return_torque.knee_torque = 50.0;
	// }
	// else if(return_torque.knee_torque < -50.0)
	// {
	// 	return_torque.knee_torque = -50.0;
	// }

	// if(legStste->joint_angle[0] > PI*12.0/180.0  ||  legStste->joint_angle[0] < -PI*12.0/180.0)
	// {
	// 	return_torque.shoulder_torque = 0.0;
	// }
	// if(legStste->joint_angle[1] > PI*90.0/180.0  ||  legStste->joint_angle[1] < 0.0)
	// {
	// 	return_torque.hip_torque = 0.0;
	// }
	// if(legStste->joint_angle[2] > -PI*70.0/180.0  ||  legStste->joint_angle[2] < -PI*120.0/180.0)
	// {
	// 	return_torque.knee_torque = 0.0;
	// }

	return return_torque;
}

/**
 * @brief Actiger摆动腿控制函数
 * @param  p_rotation  惯性坐标系到机体坐标系的旋转矩阵       
 * @param  p_grav  拉格朗日动力学方程中的重力项           
 * @param  coriolis  拉格朗日动力学方程中的离心力-哥氏力项        
 * @param  p_desPos  摆动腿足端在机体坐标系下的期望位置        
 * @param  p_desVel  摆动腿足端在机体坐标系下的期望速度      
 * @return legTorque_t 
 */
legTorque_t SingleLeg::SingleSwingControl(Mat3<double> p_rotation, Vec3<double> p_grav, Vec3<double> coriolis, Vec3<double> p_desPos, Vec3<double> p_desVel)
{

	legTorque_t return_torque = {0};
	Vec3<double> toreuqe = Vec3<double>::Zero();
	Vec3<double> force = Vec3<double>::Zero();

	Vec3<double> desPos = p_rotation * (p_desPos)-legStste->hip_pos_body;
	Vec3<double> desVel = p_rotation * p_desVel;
	// cout<< "[  "<<legStste->legNum <<"  ] ";

	// cout<< "desPos: ";
	// cout<< desPos(0)<<" "<<desPos(1)<<" "<<desPos(2)<<" ";
	// cout<< "actPos: ";
	// cout<< legStste->endfoot_hip(0)<<" "<<legStste->endfoot_hip(1)<<" "<<legStste->endfoot_hip(2)<<" ";

	// cout<< "desVel_w: ";
	// cout<< p_desVel(0)<<" "<<p_desVel(1)<<" "<<p_desVel(2)<<" ";

	// cout<< "desVel: ";
	// cout<< desVel(0)<<" "<<desVel(1)<<" "<<desVel(2)<<" ";

	// cout<< "actVel: ";
	// cout<< legStste->endfoot_vel_hip(0)<<" "<<legStste->endfoot_vel_hip(1)<<" "<<legStste->endfoot_vel_hip(2)<<" ";

	force(0) = para->leg_kpJoint(0) * (desPos(0) - legStste->endfoot_hip(0)) + para->leg_kdJoint(0) * (desPos(0) - legStste->endfoot_vel_hip(0));
	force(1) = para->leg_kpJoint(1) * (desPos(1) - legStste->endfoot_hip(1)) + para->leg_kdJoint(1) * (desPos(1) - legStste->endfoot_vel_hip(1));
	force(2) = para->leg_kpJoint(2) * (desPos(2) - legStste->endfoot_hip(2)) + para->leg_kdJoint(2) * (desPos(2) - legStste->endfoot_vel_hip(2));

	// cout<< "force-"<<legStste->legNum<< ": "<<force(0) << " " <<force(1) <<  " "<<force(2)<< endl;

	toreuqe = legStste->jacobian.transpose() * force + p_grav + coriolis;

	return_torque.shoulder_torque = toreuqe(0);
	return_torque.hip_torque = toreuqe(1);
	return_torque.knee_torque = toreuqe(2);

	// if(return_torque.shoulder_torque > 20.0)
	// {
	// 	return_torque.shoulder_torque = 20.0;
	// }
	// else if(return_torque.shoulder_torque < -20.0)
	// {
	// 	return_torque.shoulder_torque = -20.0;
	// }

	// if(return_torque.hip_torque > 50.0)
	// {
	// 	return_torque.hip_torque = 50.0;
	// }
	// else if(return_torque.hip_torque < -50.0)
	// {
	// 	return_torque.hip_torque = -50.0;
	// }

	// if(return_torque.knee_torque > 50.0)
	// {
	// 	return_torque.knee_torque = 50.0;
	// }
	// else if(return_torque.knee_torque < -50.0)
	// {
	// 	return_torque.knee_torque = -50.0;
	// }

	// if(legStste->joint_angle[0] > PI*12.0/180.0  ||  legStste->joint_angle[0] < -PI*12.0/180.0)
	// {
	// 	return_torque.shoulder_torque = 0.0;
	// }
	// if(legStste->joint_angle[1] > PI*90.0/180.0  ||  legStste->joint_angle[1] < 0.0)
	// {
	// 	return_torque.hip_torque = 0.0;
	// }
	// if(legStste->joint_angle[2] > -PI*70.0/180.0  ||  legStste->joint_angle[2] < -PI*120.0/180.0)
	// {
	// 	return_torque.knee_torque = 0.0;
	// }

	return return_torque;
}