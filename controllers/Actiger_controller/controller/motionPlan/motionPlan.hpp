/**
 * @file motionPlan.hpp
 * @brief Actiger规划器Motionlan头文件
 * @author Bin & Tech13
 * @version 1.1
 * @date 2021-08-19
 * 
 * @copyright  Copyright  (c)  2021  Action
 * 
 */

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
	/**
	 * @brief Actiger规划器
	 */
	class MotionPlan
	{
	private:	
		void Resize(void);
		void MotionOptimize(void);
		void Gait(void);

		/** @brief Actiger机体姿态 6D：0～2 Euler | 3～5 Pos */
		Vec6<double> spcial;
		/** @brief Actiger前摆动腿足端在全局坐标系下的位置 */
		Vec3<double> swing_front;
		/** @brief Actiger后摆动腿足端在全局坐标系下的位置 */
		Vec3<double> swing_rear;
		/** @brief Actiger前支撑腿足端在全局坐标系下的位置 */
		Vec3<double> stance_front;
		/** @brief Actiger后支撑腿足端在全局坐标系下的位置 */
		Vec3<double> stance_rear;
		/** @brief Actiger期望的机体高度 */
		double desire_z;
		/** @brief Actiger期望的偏航角 */
		double desire_yaw;
		/** @brief 法向量 */
		Vec3<double> normal_vector;
		/** @brief Actiger当前在全局坐标系下的速度 */
		Vec3<double> curVelocity;
		/** @brief 速度指令 */
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

		/** @brief 一个完整步态周期中的支撑相所占时间 */
		double Ts;
		/** @brief 把一个Ts时间跨度分割成grid个MPC周期 */
		int grid;
		/** @brief 表示支撑腿组合的标志位 */
		bool is_lf_rr_stand;
		/** @brief 判断是否进行一次规划的标志位 */
		bool planFlag = 0;
		/** @brief Actiger当前步态 */
		uint8_t gaitNow;
		/** @brief 期望的机体欧拉角队列 */
		Queue* pBody_euler_des;
		/** @brief 期望的机体欧拉角速度队列 */
		Queue* vBody_euler_des;
		/** @brief 期望的机体位置队列 */
		Queue* pBody_des;
		/** @brief 期望的机体速度队列 */
		Queue* vBody_des;
		/** @brief 期望的机体加速度队列 */
		Queue* aBody_des;
		/** @brief Gamepad类的实例化对象，Actiger遥控器 */
		Gamepad* gamepad;
		/** @brief StateMachine类的实例化对象，Actiger状态机 */
		StateMachine* stateMachine;
		/** @brief RobotModel类的实例化对象，存储机器人模型信息 */
		RobotModel* robotModel;
		/** @brief Parameters类的实例化对象，存储一些被频繁使用的参数 */
		Parameters* para;
		/** @brief Foothold类的实例化对象，Actiger落足点规划器 */
		Foothold* foothold;
		/** @brief BodyTrajectory类的实例化对象，Actiger机体轨迹规划器 */
		BodyTrajectory* bodyTrajectory;
		/** @brief EndfootTrajectory类的实例化对象，Actiger摆动腿足端轨迹规划器 */
		EndfootTrajectory* endfootTrajectory;
		/** @brief LegPlan类的实例化对象向量，存储Actiger腿部的规划结果 */
		std::vector<LegPlan*> legPlan;
		/** @brief LegPlan类的实例化对象，存储Actiger左前腿的规划结果 */
		LegPlan* lf;
		/** @brief LegPlan类的实例化对象，存储Actiger左后腿的规划结果 */
		LegPlan* lr;
		/** @brief LegPlan类的实例化对象，存储Actiger右前腿的规划结果 */
		LegPlan* rf;
		/** @brief LegPlan类的实例化对象，存储Actiger右后腿的规划结果 */
		LegPlan* rr;
		/** @brief Terrain类的实例化对象，表示当前的地形 */
		Terrain* terrain_now; 
		/** @brief FlatGround类的实例化对象，表示平地 */
        FlatGround* flatGround;
		/** @brief Slope类的实例化对象，表示坡面 */
        Slope* slope;
		/** @brief Stage类的实例化对象，表示平台 */
        Stage* stage;
		/** @brief Sill类的实例化对象，表示门槛 */
        Sill* sill;
		/** @brief Stairs类的实例化对象，表示阶梯 */
        Stairs* stairs;
		/** @brief Calandria类的实例化对象，表示排管 */
        Calandria* calandria;
		/** @brief Hurdles类的实例化对象，表示横杆 */
        Hurdles* hurdles;
		/** @brief Ballpool类的实例化对象，表示球池 */
        Ballpool* ballpool;
		/** @brief Seesaw类的实例化对象，表示跷跷板 */
        Seesaw* seesaw;
		/** @brief Bridge类的实例化对象，表示双木桥 */
        Bridge* bridge;

		/** @brief 一步规划后的各个状态的最终值 */
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