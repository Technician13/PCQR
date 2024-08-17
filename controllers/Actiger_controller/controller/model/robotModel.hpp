/**
 * @file robotModel.hpp
 * @brief Actiger机体模型类RobotModel头文件
 * @author Bin & Tech13
 * @version 1.1
 * @date 2021-08-19
 * 
 * @copyright  Copyright  (c)  2021  Action
 * 
 */

#ifndef ROBOTMODEL_HPP
#define ROBOTMODEL_HPP

#include <vector>
#include "floatingBaseModel.hpp"
#include "bodyStateEstimate.hpp"
#include "legStateEstimate.hpp"
#include "cppStruct.h"
#include "cppTypes.h"
#include "linearKalmanFilter.hpp"

#ifdef ROBOT
    #include "embedded.hpp"
#else

#endif

namespace quadruped
{
	/**
	 * @brief Actiger机体模型类
	 */
	class RobotModel
	{
	private:
		void Resize(void);
		void CalLegMsg(void);

	protected:

	public:
		explicit RobotModel(Parameters* p_para);
		virtual ~RobotModel(void);

		void UpdataModel(Vec12<double> mpc_force_, Vec12<double> mpc_force_1_);
		Vec3<double> CalEndfootPos2Body(void);
		Vec3<double> CalEndfootVel2Body(void);

		/** @brief 拉格朗日动力学方程中的惯性项 */
		Mat18<double> A;
		/** @brief 拉格朗日动力学方程中的惯性项的逆 */
		Mat18<double> Ainv;
		/** @brief 拉格朗日动力学方程中的离心力-哥氏力项 */
		Vec18<double> coriolis;
		/** @brief 拉格朗日动力学方程中的重力项 */
		Vec18<double> grav;
		/** @brief Actiger机体的惯性 */
		Mat6<double> cenInertia;	
		/** @brief Actiger足端在惯性坐标系下的位置向量 */
		std::vector<Vec3<double>> linkEndPos_world;
		/** @brief Actiger足端在惯性坐标系下的速度向量 */
		std::vector<Vec3<double>> linkEndVel_world;
		/** @brief Actiger左前髋关节在机体坐标系下的位置 */
		Vec3<double> lf_hip_pos_body;
		/** @brief Actiger左后髋关节在机体坐标系下的位置 */
		Vec3<double> lr_hip_pos_body;
		/** @brief Actiger右前髋关节在机体坐标系下的位置 */
		Vec3<double> rf_hip_pos_body;
		/** @brief Actiger右后髋关节在机体坐标系下的位置 */
		Vec3<double> rr_hip_pos_body;
		/** @brief Actiger当前的全状态 */
		Vec19<double> q_now;
		/** @brief Actiger当前的全一阶微分状态 */
		Vec18<double> q_dot_now;
		/** @brief Actiger空间姿态角的四元数表示 */
		Quat<double> qua;
		/** @brief Actiger在全局坐标系下的位置 */
		Vec3<double> bodyPosition;
		/** @brief Actiger在全局坐标系下的速度 */
		Vec3<double> bodyVelocity;
		/** @brief Actiger在机体坐标系下的加速度 */
		Vec3<double> bodyAcc;
		/** @brief Actiger的欧拉角 */
		Vec3<double> bodyEuler;
		/** @brief Actiger的欧拉角速度 */
		Vec3<double> bodyEulerAngularVel;
		/** @brief Actiger机体到世界的旋转矩阵 */
		Mat3<double> body2WorldRotation;
		/** @brief Actiger世界到机体的旋转矩阵 */
		Mat3<double> world2BodyRotation;

		/** @brief BodyStateEstimate类的实例化对象，Actiger状态信息 */
		BodyStateEstimate* bodyState;
		/** @brief LegStateEstimate类的实例化对象向量，Actiger腿部信息 */
		std::vector<LegStateEstimate*> legState;
		/** @brief LegStateEstimate类的实例化对象，Actiger左前腿单腿状态信息 */
		LegStateEstimate* lf_state;
		/** @brief LegStateEstimate类的实例化对象，Actiger左后腿单腿状态信息 */
		LegStateEstimate* lr_state;
		/** @brief LegStateEstimate类的实例化对象，Actiger右前腿单腿状态信息 */
		LegStateEstimate* rf_state;
		/** @brief LegStateEstimate类的实例化对象，Actiger右后腿单腿状态信息 */
		LegStateEstimate* rr_state;
		/** @brief FloatingBaseModel类的实例化对象，Actiger浮动基座模型 */
		FloatingBaseModel* model;

		LinearKalmanFilter* pos_vel_estimator;

		/** @brief WBC控制器预留接口 */
		std::vector<Vec3<double>> linkEndPos;
		/** @brief WBC控制器预留接口 */
		std::vector<Vec3<double>> linkEndVel;
		/** @brief WBC控制器预留接口 */
		std::vector<Mat3x18<double>> J;
		/** @brief WBC控制器预留接口 */
		std::vector<Vec3<double>> Jdqd;	
		/** @brief WBC控制器预留接口 */
		Vec18<double> state_now;
	};
}

#endif // !ROBOTMODEL_HPP