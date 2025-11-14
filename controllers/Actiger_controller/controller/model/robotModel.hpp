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

		Mat18<double> A;
		Mat18<double> Ainv;
		Vec18<double> coriolis;
		Vec18<double> grav;
		Mat6<double> cenInertia;	
		std::vector<Vec3<double>> linkEndPos_world;
		std::vector<Vec3<double>> linkEndVel_world;
		Vec3<double> lf_hip_pos_body;
		Vec3<double> lr_hip_pos_body;
		Vec3<double> rf_hip_pos_body;
		Vec3<double> rr_hip_pos_body;
		Vec19<double> q_now;
		Vec18<double> q_dot_now;
		Quat<double> qua;
		Vec3<double> bodyPosition;
		Vec3<double> bodyVelocity;
		Vec3<double> bodyAcc;
		Vec3<double> bodyEuler;
		Vec3<double> bodyEulerAngularVel;
		Mat3<double> body2WorldRotation;
		Mat3<double> world2BodyRotation;

		BodyStateEstimate* bodyState;
		std::vector<LegStateEstimate*> legState;
		LegStateEstimate* lf_state;
		LegStateEstimate* lr_state;
		LegStateEstimate* rf_state;
		LegStateEstimate* rr_state;
		FloatingBaseModel* model;

		LinearKalmanFilter* pos_vel_estimator;

		std::vector<Vec3<double>> linkEndPos;
		std::vector<Vec3<double>> linkEndVel;
		std::vector<Mat3x18<double>> J;
		std::vector<Vec3<double>> Jdqd;	
		Vec18<double> state_now;
	};
}

#endif // !ROBOTMODEL_HPP