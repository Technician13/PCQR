#ifndef MOTIONPLAN_HPP
#define MOTIONPLAN_HPP

#include <vector>
#include "queue.hpp"
#include "gamepad.hpp"
#include "stateMachine.hpp"
#include "bodyTrajectory.hpp"
#include "endfootTrajectory.hpp"
#include "foothold.hpp"
#include "robotModel.hpp"
#include "legPlan.hpp"

#ifndef ROBOT
	#include "webotsApi.hpp"
#endif

namespace quadruped
{
	class MotionPlan
	{
	private:	
		void Resize(void);
		void MotionOptimize(void);
		void Gait(void);

		Vec6<double> spcial;
		Vec3<double> swing_front;
		Vec3<double> swing_rear;
		Vec3<double> stance_front;
		Vec3<double> stance_rear;
		double desire_z;
		double desire_yaw;
		Vec3<double> normal_vector;
		Vec3<double> curVelocity;
		Vec3<double> cmdVelocity;
	
	protected:

	public:
		explicit MotionPlan(RobotModel* p_robotModel, Parameters* p_para);
		virtual ~MotionPlan(void);
	
		void RobotMotionPlan(void);
		void SetCmd(void);
		void UpdataInfo(void);
		void SetTerrain(uint8_t p_terrainType);
		void Start(void);

		#ifndef ROBOT
			void DisplayBodyTraj(void);
			void DisplayLFTraj(void);
			void DisplayLRTraj(void);
			void DisplayRFTraj(void);
			void DisplayRRTraj(void);
		#endif

		double Ts;
		int grid;
		bool is_lf_rr_stand;
		bool planFlag = 0;
		uint8_t gaitNow;
		Queue* pBody_euler_des;
		Queue* vBody_euler_des;
		Queue* pBody_des;
		Queue* vBody_des;
		Queue* aBody_des;
		Gamepad* gamepad;
		StateMachine* stateMachine;
		RobotModel* robotModel;
		Parameters* para;
		Foothold* foothold;
		BodyTrajectory* bodyTrajectory;
		EndfootTrajectory* endfootTrajectory;
		std::vector<LegPlan*> legPlan;
		LegPlan* lf;
		LegPlan* lr;
		LegPlan* rf;
		LegPlan* rr;
		Terrain* terrain_now; 
        FlatGround* flatGround;
        Slope* slope;
        Stage* stage;
        Sill* sill;
        Stairs* stairs;
        Calandria* calandria;
        Hurdles* hurdles;
        Ballpool* ballpool;
        Seesaw* seesaw;
        Bridge* bridge;

		Vec3<double> desEulerEnd;
		Vec3<double> desPosEnd;
		Vec3<double> swingFrontEnd;
		Vec3<double> swingRearEnd;
		Vec3<double> stanceFrontEnd;
		Vec3<double> stanceRearEnd;
		Vec3<double> lfPosEnd;
		Vec3<double> lrPosEnd;
		Vec3<double> rfPosEnd;
		Vec3<double> rrPosEnd;
	};
}

void* MotionPlanRun(void* p_motionPlan);

#endif // !MOTIONPLAN_HPP