/**
 * @file motionPlan.cpp
 * @brief Actiger规划器Motionlan
 * @author Bin & Tech13
 * @version 1.1
 * @date 2021-08-19
 * 
 * @copyright  Copyright  (c)  2021  Action
 * 
 */

#include "motionPlan.hpp"
#include "commonFunction.hpp"
#include <cstdlib>
#include <fstream>

using namespace quadruped;


/**
 * @brief MotionPlan类的构造函数，完成所有对象成员的内存分配及数据成员的初值装载
 * @param  p_robotModel  RobotModel类的实例化对象，存储机器人模型信息，用来初始化对象成员robotModel   
 * @param  p_para  Parameters类的实例化对象，存储一些被频繁使用的参数，用来初始化对象成员para       
 */
MotionPlan::MotionPlan(RobotModel *p_robotModel, Parameters *p_para) : robotModel(p_robotModel),para(p_para)
{
	gamepad = new Gamepad();
	stateMachine = new StateMachine();
	flatGround = new FlatGround(); 
	slope = new Slope();
	stage = new Stage();
	sill = new Sill();
    stairs = new Stairs(); 
    calandria = new Calandria(); 
    hurdles = new Hurdles(); 
    ballpool = new Ballpool(); 
    seesaw = new Seesaw(); 
    bridge = new Bridge(); 

    terrain_now = flatGround;
	foothold = new Foothold(p_para,terrain_now);
	endfootTrajectory = new EndfootTrajectory(p_para,terrain_now);
	bodyTrajectory = new BodyTrajectory(p_para);
	stateMachine = new StateMachine();
	pBody_euler_des = new Queue(251);
	vBody_euler_des = new Queue(251);
	pBody_des = new Queue(251);
	vBody_des = new Queue(251);
	aBody_des = new Queue(251);
	lf = new LegPlan();
	lr = new LegPlan();
	rf = new LegPlan();
	rr = new LegPlan();

	legPlan.emplace_back(lf);
	legPlan.emplace_back(lr);
	legPlan.emplace_back(rf);
	legPlan.emplace_back(rr);
	curVelocity = Vec3<double>::Zero();
	cmdVelocity = Vec3<double>::Zero();
	swing_front = Vec3<double>::Zero();
	swing_rear = Vec3<double>::Zero();
	stance_front = Vec3<double>::Zero();
	stance_rear = Vec3<double>::Zero();
	spcial = Vec6<double>::Zero();
	normal_vector << 0.0,0.0,1.0;
	desire_z = INIT_HEIGHT;
	desire_yaw =0.0;
	is_lf_rr_stand = 1;
	Ts = 0.0;
	grid = 0;
	planFlag = 0;
	gaitNow = STAND;

	std::cout << "MotionPlan Init" << std::endl;
}

/**
 * @brief MotionPlan类的析构函数，释放申请的内存
 */
MotionPlan::~MotionPlan(void)
{
	delete gamepad;
	delete stateMachine;
	delete flatGround;
	delete stairs;
	delete calandria;
	delete hurdles;
	delete ballpool;
	delete seesaw;
	delete bridge;
	delete foothold;
	delete endfootTrajectory;
	delete bodyTrajectory;
	delete[] pBody_euler_des;
	delete[] vBody_euler_des;
	delete[] pBody_des;
	delete[] vBody_des;
	delete[] aBody_des;
	delete lf;
	delete lr;
	delete rf;
	delete rr;
}

/**
 * @brief 部分数据成员重置
 */
void MotionPlan::Resize(void)
{
	pBody_euler_des->Clear();
	vBody_euler_des->Clear();
	pBody_des->Clear();
	vBody_des->Clear();
	aBody_des->Clear();

	lf->pFoot_des->Clear();
	lf->vFoot_des->Clear();
	lf->aFoot_des->Clear();

	lr->pFoot_des->Clear();
	lr->vFoot_des->Clear();
	lr->aFoot_des->Clear();

	rf->pFoot_des->Clear();
	rf->vFoot_des->Clear();
	rf->aFoot_des->Clear();

	rr->pFoot_des->Clear();
	rr->vFoot_des->Clear();
	rr->aFoot_des->Clear();
}

/**
 * @brief 空函数  |  预留接口  |  当前Actiger步态
 */
void MotionPlan::Gait(void) {}

/**
 * @brief 通过求解优化问题完成Actiger规划
 */
void MotionPlan::MotionOptimize(void)
{
	static int cnt = 0;
	static double yaw = 0.0;
	if (gaitNow == TROT || gaitNow == STOP)
	{
		fstream ofs;
		ofs.open("planningperiod.txt");
		clock_t start, end;
		start = clock();

		yaw = yaw + handle.yaw_omega * 0.2;
		desire_yaw = yaw;

		//std::cout << "======================== desire yaw: " << desire_yaw << std::endl;

		/* 落足点规划 */
		foothold->updataInfo->UpdateInfo(is_lf_rr_stand, robotModel->bodyEuler, robotModel->bodyPosition,
										 curVelocity, desire_yaw, desire_z, gaitNow, 
										 swing_front, swing_rear, stance_front, stance_rear);
		foothold->GetFootHold();

		/* 由于在摆动腿落足点的X方向上采取了相对宽松的策略[GetOtherCst中的(2)和(5)]，因此重新计算Z方向的值 */
		foothold->updataInfo->return_desEndfoot_pos_front[2] = terrain_now->GetHight(foothold->updataInfo->return_desEndfoot_pos_front[0] , 0.0, robotModel->bodyEuler[1]);
		foothold->updataInfo->return_desEndfoot_pos_rear[2] = terrain_now->GetHight(foothold->updataInfo->return_desEndfoot_pos_rear[0] , 0.0, robotModel->bodyEuler[1]);

		/* 由于上一步重新计算了落足点的Z方向的值，因此这里有必要对期望的机体俯仰角进行重新计算 */
		if(abs(foothold->updataInfo->return_desEndfoot_pos_front[2] - foothold->updataInfo->return_desEndfoot_pos_rear[2]) <= 0.01)
		{
			foothold->updataInfo->desEuler[1] = 0.0;
		}
		else
		{
			foothold->updataInfo->desEuler[1] = -1.0 * atan2((double)(foothold->updataInfo->return_desEndfoot_pos_front[2] - foothold->updataInfo->return_desEndfoot_pos_rear[2]), 
																	(double)(foothold->updataInfo->return_desEndfoot_pos_front[0] - foothold->updataInfo->return_desEndfoot_pos_rear[0]));
		}

		/* 保守 or 激进？ */
		for(int i = 0 ; i < terrain_now->numSplit ; i++)
		{
			/* 一旦求解失败，保证前摆动腿落足点位于安全区范围 */
			if(foothold->updataInfo->return_desEndfoot_pos_front[0] >= (terrain_now->split[i] - terrain_now->averageDangerArea) && 
			   foothold->updataInfo->return_desEndfoot_pos_front[0] < terrain_now->split[i])
			{
				foothold->updataInfo->return_desEndfoot_pos_front[0] = terrain_now->split[i] - terrain_now->averageDangerArea;	
			}
			else if(foothold->updataInfo->return_desEndfoot_pos_front[0] >= terrain_now->split[i] && 
			        foothold->updataInfo->return_desEndfoot_pos_front[0] < (terrain_now->split[i] + terrain_now->averageDangerArea))
			{
				foothold->updataInfo->return_desEndfoot_pos_front[0] = terrain_now->split[i] + terrain_now->averageDangerArea;
			}

			/* 一旦求解失败，保证后摆动腿落足点位于安全区范围 */
			if(foothold->updataInfo->return_desEndfoot_pos_rear[0] >= (terrain_now->split[i] - terrain_now->averageDangerArea) && 
			   foothold->updataInfo->return_desEndfoot_pos_rear[0] < terrain_now->split[i])
			{
				foothold->updataInfo->return_desEndfoot_pos_rear[0] = terrain_now->split[i] - terrain_now->averageDangerArea;
			}
			else if(foothold->updataInfo->return_desEndfoot_pos_rear[0] >= terrain_now->split[i] && 
			        foothold->updataInfo->return_desEndfoot_pos_rear[0] < (terrain_now->split[i] + terrain_now->averageDangerArea))
			{
				foothold->updataInfo->return_desEndfoot_pos_rear[0] = terrain_now->split[i] + terrain_now->averageDangerArea;
			}
		}

		Ts = para->Ts;

		Vec6<double> spatial_des = foothold->updataInfo->return_spatial_desire;
		Vec6<double> spatial_dot = Vec6<double>::Zero();
		Vec6<double> spatial_dot_des = Vec6<double>::Zero();

		//**********************************************************************************************
		spatial_des[5] = foothold->updataInfo->H;
		//***********************************************************************************************

		spatial_dot.block(0, 0, 3, 1) = robotModel->bodyEulerAngularVel;
		spatial_dot.block(3, 0, 3, 1) = curVelocity;
		//spatial_dot_des.block(0,0,3,1) = foothold->updataInfo->desOmega;
		spatial_dot_des.block(3, 0, 3, 1) = foothold->updataInfo->desVelocity;

		/* 机体轨迹规划 */
		bodyTrajectory->UpdateInfo(is_lf_rr_stand, Ts, spcial, spatial_dot, spatial_des, spatial_dot_des);
		/* TODO8.14 */
		bodyTrajectory->GetBodyTrajectory();

		grid = bodyTrajectory->grid;

		/* 摆动腿足端轨迹规划 */
		endfootTrajectory->GetEndfootTrajectory(grid, swing_front, swing_rear,
												foothold->updataInfo->return_desEndfoot_pos_front,
												foothold->updataInfo->return_desEndfoot_pos_rear,
												bodyTrajectory->return_spatial_list);

		desEulerEnd = (foothold->updataInfo->return_spatial_desire).block(0,0,3,1);
		desPosEnd = (foothold->updataInfo->return_spatial_desire).block(3,0,3,1);
		swingFrontEnd = foothold->updataInfo->return_desEndfoot_pos_front;
		swingRearEnd =foothold->updataInfo->return_desEndfoot_pos_rear;
		stanceFrontEnd = stance_front;
		stanceRearEnd = stance_rear;

		// std::cout<<"++++++++++++++++++++++++++++++++++++++++++++++++++++++++ opt result ++++++++++++++++++++++++++++++++++++++++++++++++++++++++"<<std::endl;
		// std::cout<<"[ Euler ] "<<desEulerEnd[0]<<"    "<<desEulerEnd[1]<<"    "<<desEulerEnd[2]<<std::endl;
		// std::cout<<"[ Pos ]"<<desPosEnd[0]<<"    "<<desPosEnd[1]<<"    "<<desPosEnd[2]<<std::endl;
		// std::cout<<"[ Swing Front ]"<<swingFrontEnd[0]<<"    "<<swingFrontEnd[1]<<"    "<<swingFrontEnd[2]<<std::endl;
		// std::cout<<"[ Swing Rear ]"<<swingRearEnd[0]<<"    "<<swingRearEnd[1]<<"    "<<swingRearEnd[2]<<std::endl;
		// std::cout<<"[ Stance Front ]"<<stanceFrontEnd[0]<<"    "<<stanceFrontEnd[1]<<"    "<<stanceFrontEnd[2]<<std::endl;
		// std::cout<<"[ Stance Rear ]"<<stanceRearEnd[0]<<"    "<<stanceRearEnd[1]<<"    "<<stanceRearEnd[2]<<std::endl;

		if(is_lf_rr_stand)
		{
			lfPosEnd = stanceFrontEnd;
			rrPosEnd = stanceRearEnd;
			rfPosEnd = swingFrontEnd;
			lrPosEnd = swingRearEnd;
		}
		else
		{
			rfPosEnd = stanceFrontEnd;
			lrPosEnd = stanceRearEnd;
			lfPosEnd = swingFrontEnd;
			rrPosEnd = swingRearEnd;
		}
		end = clock();
		std::cout<<"clock: "<<(double)(end-start)/CLOCKS_PER_SEC<<std::endl;
	}
}

/**
 * @brief Actiger核心规划函数
 */
void MotionPlan::RobotMotionPlan(void)
{
	planFlag = 0;
	UpdataInfo();
	MotionOptimize();
	SetCmd();
}

/**
 * @brief Actiger规划标志位处理函数
 */
void MotionPlan::Start(void)
{
	/* 设置地形 */
	SetTerrain(handle.terrain);
	static int cnt = 20;
	if (cnt >= 20)
	{
		cnt = 20;
		handle.stepOver = 1;
	}
	else
		cnt++;

	pthread_mutex_lock(&model_mutex);
	/* 四个足端同时触地时规划一次 */
	if (robotModel->legState[0]->contact_state == 1 && robotModel->legState[1]->contact_state == 1 &&
		robotModel->legState[2]->contact_state == 1 && robotModel->legState[3]->contact_state == 1 && cnt >= 10)
	{
		planFlag = 1;
		cnt = 0;
	}
	/* 站立完成标志位转换 */
	if (handle.standReady == 0 && robotModel->bodyPosition(2) >= 0.9 * INIT_HEIGHT)
	{
		handle.standReady = 1;
	}
	pthread_mutex_unlock(&model_mutex);
}

/**
 * @brief 根据形参p_terrainType设置当前地形
 * @param  p_terrainType  当前地形，配合感知得到 
 */
void MotionPlan::SetTerrain(uint8_t p_terrainType)
{
	switch (p_terrainType)
	{
	case FLAT:
		// std::cout<<"[ terrain now ]  FLAT"<<std::endl;
		terrain_now = flatGround;
		foothold->updataInfo->terrain_now = flatGround;
		endfootTrajectory->terrain_now = flatGround;
		break;

	case SLOPE:
		// std::cout<<"[ terrain now ]  SLOPE"<<std::endl;
		terrain_now = slope;
		foothold->updataInfo->terrain_now = slope;
		endfootTrajectory->terrain_now = slope;
		break;

	case STAGE:
		// std::cout<<"[ terrain now ]  STAGE"<<std::endl;
		terrain_now = stage;
		foothold->updataInfo->terrain_now = stage;
		endfootTrajectory->terrain_now = stage;
		break;

	case SILL:
		// std::cout<<"[ terrain now ]  SILL"<<std::endl;
		terrain_now = sill;
		foothold->updataInfo->terrain_now = sill;
		endfootTrajectory->terrain_now = sill;
		break;

	case CALANDRIA:
		// std::cout<<"[ terrain now ]  CALANDRIA"<<std::endl;
		terrain_now = calandria;
		foothold->updataInfo->terrain_now = calandria;
		endfootTrajectory->terrain_now = calandria;
		break;

	case STAIRS:
		// std::cout<<"[ terrain now ]  STAIRS"<<std::endl;
		terrain_now = stairs;
		foothold->updataInfo->terrain_now = stairs;
		endfootTrajectory->terrain_now = stairs;
		break;

	case SEESAW:
		// std::cout<<"[ terrain now ]  SEESAW"<<std::endl;
		terrain_now = seesaw;
		foothold->updataInfo->terrain_now = seesaw;
		endfootTrajectory->terrain_now = seesaw;
		break;

	case BRIDGE:
		// std::cout<<"[ terrain now ]  BRIDGE"<<std::endl;
		terrain_now = bridge;
		foothold->updataInfo->terrain_now = bridge;
		// endfootTrajectory->terrain_now = bridge;
		break;












	

	case HURDLES:
		// std::cout<<"[ terrain now ]  HURDLES"<<std::endl;		
		terrain_now = hurdles;
		//foothold->updataInfo->terrain_now = hurdles;
		// endfootTrajectory->terrain_now = hurdles;
		break;

	case BALL_POOL:
		// std::cout<<"[ terrain now ]  BALL_POOL"<<std::endl;
		terrain_now = ballpool;
		//foothold->updataInfo->terrain_now = ballpool;
		// endfootTrajectory->terrain_now = ballpool;
		break;

	
	}
}

/**
 * @brief 更新Actiger规划信息
 */
void MotionPlan::UpdataInfo(void)
{
	desire_z = handle.z_height;
	// desire_yaw = handle.yaw_omega*0.3;

	// cmdVelocity(0) = handle.x_speed;
	// cmdVelocity(1) = handle.y_speed;

	// if (handle.unlock == 1)
	// {
	// 	gaitNow = TROT;
	// }

	// std::cout<<"######################################################################  "<<(int)(gaitNow)<<std::endl;

	if (gaitNow == TROT || gaitNow == STOP)
	{
		is_lf_rr_stand = !is_lf_rr_stand;
		handle.is_lf_rr_stand = is_lf_rr_stand;
		grid = 0;
	}
	else
	{
		grid = 60;
	}

	pthread_mutex_lock(&model_mutex);
	curVelocity = robotModel->bodyVelocity;
	/* 根据支撑腿组合的不同定义前后的摆动腿和支撑腿 */
	if (is_lf_rr_stand)
	{
		swing_front = robotModel->linkEndPos_world[2];
		swing_rear = robotModel->linkEndPos_world[1];
		stance_front = robotModel->linkEndPos_world[0];
		stance_rear = robotModel->linkEndPos_world[3];
	}
	else
	{
		swing_front = robotModel->linkEndPos_world[0];
		swing_rear = robotModel->linkEndPos_world[3];
		stance_front = robotModel->linkEndPos_world[2];
		stance_rear = robotModel->linkEndPos_world[1];
	}

	/* 向量spcial中装载值 */
	spcial.block(0, 0, 3, 1) = robotModel->bodyEuler;
	spcial.block(3, 0, 3, 1) = robotModel->bodyPosition;
	pthread_mutex_unlock(&model_mutex);

	// SetTerrain(handle.terrain);
}

/**
 * @brief 将Actiger规划器求得的期望序列写入队列
 */
void MotionPlan::SetCmd(void)
{
	Vec3<double> zero = Vec3<double>::Zero();
	Vec3<double> dEuler = Vec3<double>::Zero();
	Vec3<double> dPos = Vec3<double>::Zero();
	Vec3<double> dEulerDot = Vec3<double>::Zero();
	Vec3<double> dPosDot = Vec3<double>::Zero();
	Vec3<double> pFoot = Vec3<double>::Zero();
	Vec3<double> vFoot = Vec3<double>::Zero();

	pthread_mutex_lock(&ref_mutex);

	Resize();

	for (int i = 0; i < 250; i++)
	{
		if (gaitNow == TROT)
		{
			handle.stepOver = 0;
			if (i < grid)
			{
				dEuler = bodyTrajectory->return_spatial_list.block(0, i, 3, 1);
				dEulerDot = bodyTrajectory->return_spatial_dot_list.block(0, i, 3, 1);
				dPos = bodyTrajectory->return_spatial_list.block(3, i, 3, 1);
				dPosDot = bodyTrajectory->return_spatial_dot_list.block(3, i, 3, 1);

				if (is_lf_rr_stand)
				{
					/* 摆动腿 */
					pFoot = endfootTrajectory->endfoot_pos_front_list.block(0, i, 3, 1) - dPos;
					vFoot = endfootTrajectory->endfoot_vel_front_list.block(0, i, 3, 1) - dPosDot;

					rf->pFoot_des->Push(pFoot);
					rf->vFoot_des->Push(vFoot);
					rf->aFoot_des->Push(zero);

					// std::cout<<"************************************************************************************************************************"<<std::endl;
					// std::cout<<pFoot(0)<<"	"<<pFoot(1)<<"	"<<pFoot(2)<<"	"<<dPos(0)<<" "<<dPos(1)<<" "<<dPos(2)<<std::endl;

					pFoot = endfootTrajectory->endfoot_pos_rear_list.block(0, i, 3, 1) - dPos;
					vFoot = endfootTrajectory->endfoot_vel_rear_list.block(0, i, 3, 1) - dPosDot;

					lr->pFoot_des->Push(pFoot);
					lr->vFoot_des->Push(vFoot);
					lr->aFoot_des->Push(zero);

					/* 支撑腿 */
					lf->pFoot_des->Push(robotModel->linkEndPos_world[0] - dPos);
					lf->vFoot_des->Push(zero);
					lf->aFoot_des->Push(zero);

					rr->pFoot_des->Push(robotModel->linkEndPos_world[3] - dPos);
					rr->vFoot_des->Push(zero);
					rr->aFoot_des->Push(zero);			
				}
				else
				{
					/* 摆动腿 */
					pFoot = endfootTrajectory->endfoot_pos_front_list.block(0, i, 3, 1) - dPos;
					vFoot = endfootTrajectory->endfoot_vel_front_list.block(0, i, 3, 1) - dPosDot;

					lf->pFoot_des->Push(pFoot);
					lf->vFoot_des->Push(vFoot);
					lf->aFoot_des->Push(zero);

					pFoot = endfootTrajectory->endfoot_pos_rear_list.block(0, i, 3, 1) - dPos;
					vFoot = endfootTrajectory->endfoot_vel_rear_list.block(0, i, 3, 1) - dPosDot;

					rr->pFoot_des->Push(pFoot);
					rr->vFoot_des->Push(vFoot);
					rr->aFoot_des->Push(zero);

					/* 支撑腿 */
					rf->pFoot_des->Push(robotModel->linkEndPos_world[2] - dPos);
					rf->vFoot_des->Push(zero);
					rf->aFoot_des->Push(zero);

					lr->pFoot_des->Push(robotModel->linkEndPos_world[1] - dPos);
					lr->vFoot_des->Push(zero);
					lr->aFoot_des->Push(zero);
				}
			}
			else
			{
				dEuler = bodyTrajectory->return_spatial_list.block(0, grid - 1, 3, 1);
				dEulerDot = bodyTrajectory->return_spatial_dot_list.block(0, grid - 1, 3, 1);
				dPos = bodyTrajectory->return_spatial_list.block(3, grid - 1, 3, 1);
				dPosDot = bodyTrajectory->return_spatial_dot_list.block(3, grid - 1, 3, 1);

				/* 加入延迟触地应对机制 */
				double length = 250.0 - (double)grid;
				Vec3<double> delayContactTargetFront = endfootTrajectory->endfoot_pos_front_list.block(0, grid - 1, 3, 1);
				Vec3<double> delayContactTargetRear = endfootTrajectory->endfoot_pos_rear_list.block(0, grid - 1, 3, 1);
				delayContactTargetFront[2] = delayContactTargetFront[2] - (double)(i - grid) / length;
				delayContactTargetRear[2] = delayContactTargetRear[2] - (double)(i - grid) / length;

				if (is_lf_rr_stand)
				{
					/* 摆动腿 */
					//pFoot = endfootTrajectory->endfoot_pos_front_list.block(0, grid - 1, 3, 1) - dPos;
					pFoot = delayContactTargetFront - dPos;
					vFoot = endfootTrajectory->endfoot_pos_front_list.block(0, grid - 1, 3, 1) - dPosDot;

					rf->pFoot_des->Push(pFoot);
					rf->vFoot_des->Push(vFoot);
					rf->aFoot_des->Push(zero);

					//pFoot = endfootTrajectory->endfoot_pos_rear_list.block(0, grid - 1, 3, 1) - dPos;
					pFoot = delayContactTargetRear - dPos;
					vFoot = endfootTrajectory->endfoot_pos_rear_list.block(0, grid - 1, 3, 1) - dPosDot;

					lr->pFoot_des->Push(pFoot);
					lr->vFoot_des->Push(vFoot);
					lr->aFoot_des->Push(zero);

					/* 支撑腿 */
					lf->pFoot_des->Push(robotModel->linkEndPos_world[0] - dPos);
					lf->vFoot_des->Push(zero);
					lf->aFoot_des->Push(zero);

					rr->pFoot_des->Push(robotModel->linkEndPos_world[3] - dPos);
					rr->vFoot_des->Push(zero);
					rr->aFoot_des->Push(zero);
				}
				else
				{
					/* 摆动腿 */
					//pFoot = endfootTrajectory->endfoot_pos_front_list.block(0, grid - 1, 3, 1) - dPos;
					pFoot = delayContactTargetFront - dPos;
					vFoot = endfootTrajectory->endfoot_pos_front_list.block(0, grid - 1, 3, 1) - dPosDot;

					lf->pFoot_des->Push(pFoot);
					lf->vFoot_des->Push(vFoot);
					lf->aFoot_des->Push(zero);

					//pFoot = endfootTrajectory->endfoot_pos_rear_list.block(0, grid - 1, 3, 1) - dPos;
					pFoot = delayContactTargetRear - dPos;
					vFoot = endfootTrajectory->endfoot_pos_rear_list.block(0, grid - 1, 3, 1) - dPosDot;

					rr->pFoot_des->Push(pFoot);
					rr->vFoot_des->Push(vFoot);
					rr->aFoot_des->Push(zero);

					/* 支撑腿 */
					rf->pFoot_des->Push(robotModel->linkEndPos_world[2] - dPos);
					rf->vFoot_des->Push(zero);
					rf->aFoot_des->Push(zero);

					lr->pFoot_des->Push(robotModel->linkEndPos_world[1] - dPos);
					lr->vFoot_des->Push(zero);
					lr->aFoot_des->Push(zero);
				}
			}
		}
		else if(gaitNow == STOP)
		{
			dEuler = desEulerEnd;
			dPos = desPosEnd;

			lf->pFoot_des->Push(lfPosEnd);
			lf->vFoot_des->Push(vFoot);
			lf->aFoot_des->Push(zero);

			lr->pFoot_des->Push(lrPosEnd);
			lr->vFoot_des->Push(vFoot);
			lr->aFoot_des->Push(zero);

			rf->pFoot_des->Push(rfPosEnd);
			rf->vFoot_des->Push(vFoot);
			rf->aFoot_des->Push(zero);

			rr->pFoot_des->Push(rrPosEnd);
			rr->vFoot_des->Push(vFoot);
			rr->aFoot_des->Push(zero);
		}
		else if (gaitNow == STAND)
		{
			dEuler(0) = handle.s_roll;
			dEuler(1) = handle.s_pitch;
			dEuler(2) = handle.s_yaw;
			dEulerDot = zero;
			dPos(2) = desire_z;
			dPosDot = zero;
		}
		else if (gaitNow == LYING)
		{
			dEuler(0) = handle.s_roll;
			dEuler(1) = handle.s_pitch;
			dEuler(2) = handle.s_yaw;
			dEulerDot = zero;
			dPos(2) = 0.3;
			dPosDot = zero;
		}
		
		pBody_euler_des->Push(dEuler);
		vBody_euler_des->Push(dEulerDot);
		pBody_des->Push(dPos);
		vBody_des->Push(dPosDot);
		aBody_des->Push(zero);
	}

	pthread_mutex_unlock(&ref_mutex);
}

/**
 * @brief Actiger规划器运行函数
 * @param  p_motionPlan  准备运行的机器人规划器   
 * @return void* 
 */
void *MotionPlanRun(void *p_motionPlan)
{
	MotionPlan *motionPlan = (MotionPlan *)p_motionPlan;
	while (1)
	{
		/* 5ms进一次，也就是主循环迭代一次进一次 */
		if (GetTimeFlagMotionPlan())
		{
			motionPlan->gamepad->GetKey(&(motionPlan->gaitNow));
			motionPlan->Start();
			if (motionPlan->planFlag == 1)
					motionPlan->RobotMotionPlan();

			#ifndef ROBOT
				#if VISIBLE_BODY_TRAJ
					motionPlan->DisplayBodyTraj();
				#endif
				#if VISIBLE_LF_TRAJ
					motionPlan->DisplayLFTraj();
				#endif
				#if VISIBLE_LR_TRAJ
					motionPlan->DisplayLRTraj();
				#endif
				#if VISIBLE_RF_TRAJ
					motionPlan->DisplayRFTraj();
				#endif
				#if VISIBLE_RR_TRAJ
					motionPlan->DisplayRRTraj();
				#endif
			#endif

		}
	}
	return (void *)0;
}

#ifndef ROBOT
	/**
	 * @brief 机体轨迹可视化
	 */
	void MotionPlan::DisplayBodyTraj(void)
	{
		const double body_translation[3] = {robotModel->bodyState->actul_body_pos(1),
											robotModel->bodyState->actul_body_pos(2) ,
											robotModel->bodyState->actul_body_pos(0)};
		bodyTrajPointField->setMFVec3f(bodyTrajIndex, body_translation);
		if (bodyTrajIndex > 0) 
		{
			bodyTrajCoordIndexField->setMFInt32(3 * (bodyTrajIndex - 1), bodyTrajIndex - 1);
			bodyTrajCoordIndexField->setMFInt32(3 * (bodyTrajIndex - 1) + 1, bodyTrajIndex);
		}
		else if (bodyTrajIndex == 0 && bodyTrajFirstStep == false) 
		{
			bodyTrajCoordIndexField->setMFInt32(3 * (MAX_NUM_DISPLAY_DOT - 1), 0);
			bodyTrajCoordIndexField->setMFInt32(3 * (MAX_NUM_DISPLAY_DOT - 1) + 1, MAX_NUM_DISPLAY_DOT - 1);
		}
		bodyTrajCoordIndexField->setMFInt32(3 * bodyTrajIndex, bodyTrajIndex);
		bodyTrajCoordIndexField->setMFInt32(3 * bodyTrajIndex + 1, bodyTrajIndex);

		bodyTrajFirstStep = false;
		bodyTrajIndex++;
		bodyTrajIndex = bodyTrajIndex % MAX_NUM_DISPLAY_DOT;
	}

	/**
	 * @brief 左前腿足端轨迹可视化
	 */
	void MotionPlan::DisplayLFTraj(void)
	{
		const double lf_translation[3] = {robotModel->linkEndPos_world[0](1),
										  robotModel->linkEndPos_world[0](2) ,
										  robotModel->linkEndPos_world[0](0)};
		lfTrajPointField->setMFVec3f(lfTrajIndex, lf_translation);
		if (lfTrajIndex > 0) 
		{
			lfTrajCoordIndexField->setMFInt32(3 * (lfTrajIndex - 1), lfTrajIndex - 1);
			lfTrajCoordIndexField->setMFInt32(3 * (lfTrajIndex - 1) + 1, lfTrajIndex);
		}
		else if (lfTrajIndex == 0 && lfTrajFirstStep == false) 
		{
			lfTrajCoordIndexField->setMFInt32(3 * (MAX_NUM_DISPLAY_DOT - 1), 0);
			lfTrajCoordIndexField->setMFInt32(3 * (MAX_NUM_DISPLAY_DOT - 1) + 1, MAX_NUM_DISPLAY_DOT - 1);
		}
		lfTrajCoordIndexField->setMFInt32(3 * lfTrajIndex, lfTrajIndex);
		lfTrajCoordIndexField->setMFInt32(3 * lfTrajIndex + 1, lfTrajIndex);

		lfTrajFirstStep = false;
		lfTrajIndex++;
		lfTrajIndex = lfTrajIndex % MAX_NUM_DISPLAY_DOT;
	}

	/**
	 * @brief 左后腿足端轨迹可视化
	 */
	void MotionPlan::DisplayLRTraj(void)
	{
		const double lr_translation[3] = {robotModel->linkEndPos_world[1](1),
										  robotModel->linkEndPos_world[1](2) ,
										  robotModel->linkEndPos_world[1](0)};
		lrTrajPointField->setMFVec3f(lrTrajIndex, lr_translation);
		if (lrTrajIndex > 0) 
		{
			lrTrajCoordIndexField->setMFInt32(3 * (lrTrajIndex - 1), lrTrajIndex - 1);
			lrTrajCoordIndexField->setMFInt32(3 * (lrTrajIndex - 1) + 1, lrTrajIndex);
		}
		else if (lrTrajIndex == 0 && lrTrajFirstStep == false) 
		{
			lrTrajCoordIndexField->setMFInt32(3 * (MAX_NUM_DISPLAY_DOT - 1), 0);
			lrTrajCoordIndexField->setMFInt32(3 * (MAX_NUM_DISPLAY_DOT - 1) + 1, MAX_NUM_DISPLAY_DOT - 1);
		}
		lrTrajCoordIndexField->setMFInt32(3 * lrTrajIndex, lrTrajIndex);
		lrTrajCoordIndexField->setMFInt32(3 * lrTrajIndex + 1, lrTrajIndex);

		lrTrajFirstStep = false;
		lrTrajIndex++;
		lrTrajIndex = lrTrajIndex % MAX_NUM_DISPLAY_DOT;
	}

	/**
	 * @brief 右前腿足端轨迹可视化
	 */
	void MotionPlan::DisplayRFTraj(void)
	{
		const double rf_translation[3] = {robotModel->linkEndPos_world[2](1),
										  robotModel->linkEndPos_world[2](2) ,
										  robotModel->linkEndPos_world[2](0)};
		rfTrajPointField->setMFVec3f(rfTrajIndex, rf_translation);
		if (rfTrajIndex > 0) 
		{
			rfTrajCoordIndexField->setMFInt32(3 * (rfTrajIndex - 1), rfTrajIndex - 1);
			rfTrajCoordIndexField->setMFInt32(3 * (rfTrajIndex - 1) + 1, rfTrajIndex);
		}
		else if (rfTrajIndex == 0 && rfTrajFirstStep == false) 
		{
			rfTrajCoordIndexField->setMFInt32(3 * (MAX_NUM_DISPLAY_DOT - 1), 0);
			rfTrajCoordIndexField->setMFInt32(3 * (MAX_NUM_DISPLAY_DOT - 1) + 1, MAX_NUM_DISPLAY_DOT - 1);
		}
		rfTrajCoordIndexField->setMFInt32(3 * rfTrajIndex, rfTrajIndex);
		rfTrajCoordIndexField->setMFInt32(3 * rfTrajIndex + 1, rfTrajIndex);

		rfTrajFirstStep = false;
		rfTrajIndex++;
		rfTrajIndex = rfTrajIndex % MAX_NUM_DISPLAY_DOT;
	}

	/**
	 * @brief 右后腿足端轨迹可视化
	 */
	void MotionPlan::DisplayRRTraj(void)
	{
		const double rr_translation[3] = {robotModel->linkEndPos_world[3](1),
										  robotModel->linkEndPos_world[3](2) ,
										  robotModel->linkEndPos_world[3](0)};
		rrTrajPointField->setMFVec3f(rrTrajIndex, rr_translation);
		if (rrTrajIndex > 0) 
		{
			rrTrajCoordIndexField->setMFInt32(3 * (rrTrajIndex - 1), rrTrajIndex - 1);
			rrTrajCoordIndexField->setMFInt32(3 * (rrTrajIndex - 1) + 1, rrTrajIndex);
		}
		else if (rrTrajIndex == 0 && rrTrajFirstStep == false) 
		{
			rrTrajCoordIndexField->setMFInt32(3 * (MAX_NUM_DISPLAY_DOT - 1), 0);
			rrTrajCoordIndexField->setMFInt32(3 * (MAX_NUM_DISPLAY_DOT - 1) + 1, MAX_NUM_DISPLAY_DOT - 1);
		}
		rrTrajCoordIndexField->setMFInt32(3 * rrTrajIndex, rrTrajIndex);
		rrTrajCoordIndexField->setMFInt32(3 * rrTrajIndex + 1, rrTrajIndex);

		rrTrajFirstStep = false;
		rrTrajIndex++;
		rrTrajIndex = rrTrajIndex % MAX_NUM_DISPLAY_DOT;
	}

#endif