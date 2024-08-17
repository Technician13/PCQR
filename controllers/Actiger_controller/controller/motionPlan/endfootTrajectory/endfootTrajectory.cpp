/**
 * @file endfootTrajectory.cpp
 * @brief Actiger摆动腿足端轨迹规划器EndfootTrajectory
 * @author Bin & Tech13
 * @version 1.1
 * @date 2021-11-01
 * 
 * @copyright  Copyright  (c)  2021  Action
 * 
 */

#include "endfootTrajectory.hpp"
#include <iostream>

using namespace quadruped;

/**
 * @brief EndfootTrajectory类的构造函数，完成当前地形的设定以及Mininap类对象的实例化
 * @param  p_para           
 * @param  p_terrain_now  当前地形    
 */
EndfootTrajectory::EndfootTrajectory(Parameters *p_para, Terrain *p_terrain_now)
{
	terrain_now = p_terrain_now;
	minisnap = new MiniSnap(2);
}

/**
 * @brief EndfootTrajectory类的析构函数，释放申请的内存
 */
EndfootTrajectory::~EndfootTrajectory(void)
{
	delete minisnap;
}

/**
 * @brief 获取摆动腿足端轨迹
 * @param  p_grid  一次摆动过程中MPC周期数目           
 * @param  p_front_startPoint  前摆动腿的起始点
 * @param  p_rear_startPoint  后摆动腿的起始点
 * @param  p_front_endPoint  前摆动腿的终止点
 * @param  p_rear_endPoint  后摆动腿的终止点
 * @param  p_spatial_list
 */
void EndfootTrajectory::GetEndfootTrajectory(int p_grid, 
											 Vec3<double> p_front_startPoint,
											 Vec3<double> p_rear_startPoint,
											 Vec3<double> p_front_endPoint,
											 Vec3<double> p_rear_endPoint,
											 DMat<double> p_spatial_list)
{
	#ifdef DISPLAY_RES
		std::cout<<"[ ====================================================== 足端轨迹规划 ====================================================== ]"<<std::endl;
	#endif

	Vec3<double> front_another = Vec3<double>::Zero();
	Vec3<double> rear_another = Vec3<double>::Zero();

	grid = p_grid;
	front_startPoint = p_front_startPoint;
	front_endPoint = p_front_endPoint;
	rear_startPoint = p_rear_startPoint;
	rear_endPoint = p_rear_endPoint;

	/* 减小计算复杂度，保留三位小数 */
	front_startPoint[0] = (double)((double)((int)(front_startPoint[0]*1000))/1000.0);
	front_startPoint[1] = (double)((double)((int)(front_startPoint[1]*1000))/1000.0);
	front_startPoint[2] = (double)((double)((int)(front_startPoint[2]*1000))/1000.0);

	front_endPoint[0] = (double)((double)((int)(front_endPoint[0]*1000))/1000.0);
	front_endPoint[1] = (double)((double)((int)(front_endPoint[1]*1000))/1000.0);
	front_endPoint[2] = (double)((double)((int)(front_endPoint[2]*1000))/1000.0);

	rear_startPoint[0] = (double)((double)((int)(rear_startPoint[0]*1000))/1000.0);
	rear_startPoint[1] = (double)((double)((int)(rear_startPoint[1]*1000))/1000.0);
	rear_startPoint[2] = (double)((double)((int)(rear_startPoint[2]*1000))/1000.0);

	rear_endPoint[0] = (double)((double)((int)(rear_endPoint[0]*1000))/1000.0);
	rear_endPoint[1] = (double)((double)((int)(rear_endPoint[1]*1000))/1000.0);
	rear_endPoint[2] = (double)((double)((int)(rear_endPoint[2]*1000))/1000.0);

	/* 设定前、后摆动腿轨迹的中点
	*  位置：起始点和落足点在X-Y平面连线的中点
	*  高度：抬腿高度 */
	front_another(0) = (front_endPoint(0) + front_startPoint(0)) * 0.5;
	front_another(1) = (front_endPoint(1) + front_startPoint(1)) * 0.5;
	front_another(2) = (front_endPoint(2) + front_startPoint(2)) * 0.5 + INIT_SWING_HEIGHT;

	rear_another(0) = (rear_endPoint(0) + rear_startPoint(0)) * 0.5;
	rear_another(1) = (rear_endPoint(1) + rear_startPoint(1)) * 0.5;
	rear_another(2) = (rear_endPoint(2) + rear_startPoint(2)) * 0.5 + INIT_SWING_HEIGHT;

	front_another[0] = (double)((double)((int)(front_another[0]*1000))/1000.0);
	front_another[1] = (double)((double)((int)(front_another[1]*1000))/1000.0);
	front_another[2] = (double)((double)((int)(front_another[2]*1000))/1000.0);

	rear_another[0] = (double)((double)((int)(rear_another[0]*1000))/1000.0);
	rear_another[1] = (double)((double)((int)(rear_another[1]*1000))/1000.0);
	rear_another[2] = (double)((double)((int)(rear_another[2]*1000))/1000.0);

	/* 设定第三点处的足端速度 */
	Vec3<double> front_another_vel = Vec3<double>::Zero();
	Vec3<double> rear_another_vel = Vec3<double>::Zero();

	front_another_vel(0) = (front_endPoint(0) - front_startPoint(0)) / (grid * 0.5 * MPC_PERIOD);
	front_another_vel(1) = (front_endPoint(1) - front_startPoint(1)) / (grid * 0.5 * MPC_PERIOD);
	front_another_vel(2) = (front_endPoint(2) - front_startPoint(2)) / (grid * 0.5 * MPC_PERIOD);

	rear_another_vel(0) = (rear_endPoint(0) - rear_startPoint(0)) / (grid * 0.5 * MPC_PERIOD);
	rear_another_vel(1) = (rear_endPoint(1) - rear_startPoint(1)) / (grid * 0.5 * MPC_PERIOD);
	rear_another_vel(2) = (rear_endPoint(2) - rear_startPoint(2)) / (grid * 0.5 * MPC_PERIOD);

	front_another_vel[0] = (double)((double)((int)(front_another_vel[0]*1000))/1000.0);
	front_another_vel[1] = (double)((double)((int)(front_another_vel[1]*1000))/1000.0);
	front_another_vel[2] = (double)((double)((int)(front_another_vel[2]*1000))/1000.0);

	rear_another_vel[0] = (double)((double)((int)(rear_another_vel[0]*1000))/1000.0);
	rear_another_vel[1] = (double)((double)((int)(rear_another_vel[1]*1000))/1000.0);
	rear_another_vel[2] = (double)((double)((int)(rear_another_vel[2]*1000))/1000.0);

	/* 设定每段子轨迹的时间占比 */
	std::vector<double> proportion{0.5 , 0.5};
	double T = grid * MPC_PERIOD;

	std::vector<double> para_front_X;
	std::vector<double> para_front_Y;
	std::vector<double> para_front_Z;
	std::vector<double> para_rear_X;
	std::vector<double> para_rear_Y;
	std::vector<double> para_rear_Z;

	/* 前摆动腿的X方向规划 */
	std::vector<double> front_X{front_startPoint[0] , front_another[0] , front_endPoint[0]};
    std::vector<double> front_X_vel{0.0 , front_another_vel[0] , 0.0};
	minisnap->SetParas(proportion , T , front_X , front_X_vel);
	minisnap->SolveOpt(para_front_X);

	#ifdef DISPLAY_RES
		std::cout<<"[ ============================ 前摆动腿的X方向规划 ============================ ]"<<std::endl;
		std::cout<<proportion[0]<<"	"<<proportion[1]<<std::endl;
		std::cout<<T<<std::endl;
		std::cout<<front_X[0]<<"	"<<front_X[1]<<"	"<<front_X[2]<<std::endl;
		std::cout<<front_X_vel[0]<<"	"<<front_X_vel[1]<<"	"<<front_X_vel[2]<<std::endl;
	#endif
	
	/* 前摆动腿的Y方向规划 */
	std::vector<double> front_Y{front_startPoint[1] , front_another[1] , front_endPoint[1]};
    std::vector<double> front_Y_vel{0.0 , front_another_vel[1] , 0.0};
	minisnap->SetParas(proportion , T , front_Y , front_Y_vel);
	minisnap->SolveOpt(para_front_Y);

	#ifdef DISPLAY_RES
		std::cout<<"[ ============================ 前摆动腿的Y方向规划 ============================ ]"<<std::endl;
		std::cout<<proportion[0]<<"	"<<proportion[1]<<std::endl;
		std::cout<<T<<std::endl;
		std::cout<<front_Y[0]<<"	"<<front_Y[1]<<"	"<<front_Y[2]<<std::endl;
		std::cout<<front_Y_vel[0]<<"	"<<front_Y_vel[1]<<"	"<<front_Y_vel[2]<<std::endl;
	#endif

	/* 前摆动腿的Z方向规划 */
	std::vector<double> front_Z{front_startPoint[2] , front_another[2] , front_endPoint[2]};
    std::vector<double> front_Z_vel{0.0 , front_another_vel[2] , 0.0};
	minisnap->SetParas(proportion , T , front_Z , front_Z_vel);
	minisnap->SolveOpt(para_front_Z);

	#ifdef DISPLAY_RES
		std::cout<<"[ ============================ 前摆动腿的Z方向规划 ============================ ]"<<std::endl;
		std::cout<<proportion[0]<<"	"<<proportion[1]<<std::endl;
		std::cout<<T<<std::endl;
		std::cout<<front_Z[0]<<"	"<<front_Z[1]<<"	"<<front_Z[2]<<std::endl;
		std::cout<<front_Z_vel[0]<<"	"<<front_Z_vel[1]<<"	"<<front_Z_vel[2]<<std::endl;
	#endif

	/* 后摆动腿的X方向规划 */
	std::vector<double> rear_X{rear_startPoint[0] , rear_another[0] , rear_endPoint[0]};
    std::vector<double> rear_X_vel{0.0 , rear_another_vel[0] , 0.0};
	minisnap->SetParas(proportion , T , rear_X , rear_X_vel);
	minisnap->SolveOpt(para_rear_X);

	#ifdef DISPLAY_RES
		std::cout<<"[ ============================ 后摆动腿的X方向规划 ============================ ]"<<std::endl;
		std::cout<<proportion[0]<<"	"<<proportion[1]<<std::endl;
		std::cout<<T<<std::endl;
		std::cout<<rear_X[0]<<"	"<<rear_X[1]<<"	"<<rear_X[2]<<std::endl;
		std::cout<<rear_X_vel[0]<<"	"<<rear_X_vel[1]<<"	"<<rear_X_vel[2]<<std::endl;
	#endif

	/* 后摆动腿的Y方向规划 */
	std::vector<double> rear_Y{rear_startPoint[1] , rear_another[1] , rear_endPoint[1]};
    std::vector<double> rear_Y_vel{0.0 , rear_another_vel[1] , 0.0};
	minisnap->SetParas(proportion , T , rear_Y , rear_Y_vel);
	minisnap->SolveOpt(para_rear_Y);

	#ifdef DISPLAY_RES
		std::cout<<"[ ============================ 后摆动腿的Y方向规划 ============================ ]"<<std::endl;
		std::cout<<proportion[0]<<"	"<<proportion[1]<<std::endl;
		std::cout<<T<<std::endl;
		std::cout<<rear_Y[0]<<"	"<<rear_Y[1]<<"	"<<rear_Y[2]<<std::endl;
		std::cout<<rear_Y_vel[0]<<"	"<<rear_Y_vel[1]<<"	"<<rear_Y_vel[2]<<std::endl;
	#endif

	/* 后摆动腿的Z方向规划 */
	std::vector<double> rear_Z{rear_startPoint[2] , rear_another[2] , rear_endPoint[2]};
    std::vector<double> rear_Z_vel{0.0 , rear_another_vel[2] , 0.0};
	minisnap->SetParas(proportion , T , rear_Z , rear_Z_vel);
	minisnap->SolveOpt(para_rear_Z);

	#ifdef DISPLAY_RES
		std::cout<<"[ ============================ 后摆动腿的Z方向规划 ============================ ]"<<std::endl;
		std::cout<<proportion[0]<<"	"<<proportion[1]<<std::endl;
		std::cout<<T<<std::endl;
		std::cout<<rear_Z[0]<<"	"<<rear_Z[1]<<"	"<<rear_Z[2]<<std::endl;
		std::cout<<rear_Z_vel[0]<<"	"<<rear_Z_vel[1]<<"	"<<rear_Z_vel[2]<<std::endl;
	#endif

	/* 装载规划结果 */
	endfoot_pos_front_list.resize(3, grid);
	endfoot_pos_front_list.setZero();
	endfoot_vel_front_list.resize(3, grid);
	endfoot_vel_front_list.setZero();
	endfoot_pos_rear_list.resize(3, grid);
	endfoot_pos_rear_list.setZero();
	endfoot_vel_rear_list.resize(3, grid);
	endfoot_vel_rear_list.setZero();
	
	endfoot_pos_front_list.block(0, grid - 1, 3, 1) = front_endPoint;
	endfoot_pos_rear_list.block(0, grid - 1, 3, 1) = rear_endPoint;
	
	for (int i = 0; i < grid - 1; i++)
	{
	 	double t = i * MPC_PERIOD;

		double t5 = pow(t , 5);
		double t4 = pow(t , 4);
		double t3 = pow(t , 3);
		double t2 = pow(t , 2);
		double t1 = pow(t , 1);

		if(i < grid * 0.5)
		{
			endfoot_pos_front_list(0, i) = para_front_X[0] * t5 + para_front_X[1] * t4 + para_front_X[2] * t3 + para_front_X[3] * t2 + para_front_X[4] * t1 + para_front_X[5];
			endfoot_pos_front_list(1, i) = para_front_Y[0] * t5 + para_front_Y[1] * t4 + para_front_Y[2] * t3 + para_front_Y[3] * t2 + para_front_Y[4] * t1 + para_front_Y[5];
			endfoot_pos_front_list(2, i) = para_front_Z[0] * t5 + para_front_Z[1] * t4 + para_front_Z[2] * t3 + para_front_Z[3] * t2 + para_front_Z[4] * t1 + para_front_Z[5];
			
			endfoot_pos_rear_list(0, i) = para_rear_X[0] * t5 + para_rear_X[1] * t4 + para_rear_X[2] * t3 + para_rear_X[3] * t2 + para_rear_X[4] * t1 + para_rear_X[5];
			endfoot_pos_rear_list(1, i) = para_rear_Y[0] * t5 + para_rear_Y[1] * t4 + para_rear_Y[2] * t3 + para_rear_Y[3] * t2 + para_rear_Y[4] * t1 + para_rear_Y[5];
			endfoot_pos_rear_list(2, i) = para_rear_Z[0] * t5 + para_rear_Z[1] * t4 + para_rear_Z[2] * t3 + para_rear_Z[3] * t2 + para_rear_Z[4] * t1 + para_rear_Z[5];
			
			endfoot_vel_front_list(0, i) = 5 * para_front_X[0] * t4 + 4 * para_front_X[1] * t3 + 3 * para_front_X[2] * t2 + 2 * para_front_X[3] * t + para_front_X[4];
			endfoot_vel_front_list(1, i) = 5 * para_front_Y[0] * t4 + 4 * para_front_Y[1] * t3 + 3 * para_front_Y[2] * t2 + 2 * para_front_Y[3] * t + para_front_Y[4];
			endfoot_vel_front_list(2, i) = 5 * para_front_Z[0] * t4 + 4 * para_front_Z[1] * t3 + 3 * para_front_Z[2] * t2 + 2 * para_front_Z[3] * t + para_front_Z[4];

			endfoot_vel_rear_list(0, i) = 5 * para_rear_X[0] * t4 + 4 * para_rear_X[1] * t3 + 3 * para_rear_X[2] * t2 + 2 * para_rear_X[3] * t + para_rear_X[4];
			endfoot_vel_rear_list(1, i) = 5 * para_rear_Y[0] * t4 + 4 * para_rear_Y[1] * t3 + 3 * para_rear_Y[2] * t2 + 2 * para_rear_Y[3] * t + para_rear_Y[4];
			endfoot_vel_rear_list(2, i) = 5 * para_rear_Z[0] * t4 + 4 * para_rear_Z[1] * t3 + 3 * para_rear_Z[2] * t2 + 2 * para_rear_Z[3] * t + para_rear_Z[4];
		}	
		else
		{
			endfoot_pos_front_list(0, i) = para_front_X[6] * t5 + para_front_X[7] * t4 + para_front_X[8] * t3 + para_front_X[9] * t2 + para_front_X[10] * t1 + para_front_X[11];
			endfoot_pos_front_list(1, i) = para_front_Y[6] * t5 + para_front_Y[7] * t4 + para_front_Y[8] * t3 + para_front_Y[9] * t2 + para_front_Y[10] * t1 + para_front_Y[11];
			endfoot_pos_front_list(2, i) = para_front_Z[6] * t5 + para_front_Z[7] * t4 + para_front_Z[8] * t3 + para_front_Z[9] * t2 + para_front_Z[10] * t1 + para_front_Z[11];
			
			endfoot_pos_rear_list(0, i) = para_rear_X[6] * t5 + para_rear_X[7] * t4 + para_rear_X[8] * t3 + para_rear_X[9] * t2 + para_rear_X[10] * t1 + para_rear_X[11];
			endfoot_pos_rear_list(1, i) = para_rear_Y[6] * t5 + para_rear_Y[7] * t4 + para_rear_Y[8] * t3 + para_rear_Y[9] * t2 + para_rear_Y[10] * t1 + para_rear_Y[11];
			endfoot_pos_rear_list(2, i) = para_rear_Z[6] * t5 + para_rear_Z[7] * t4 + para_rear_Z[8] * t3 + para_rear_Z[9] * t2 + para_rear_Z[10] * t1 + para_rear_Z[11];
			
			endfoot_vel_front_list(0, i) = 5 * para_front_X[6] * t4 + 4 * para_front_X[7] * t3 + 3 * para_front_X[8] * t2 + 2 * para_front_X[9] * t + para_front_X[10];
			endfoot_vel_front_list(1, i) = 5 * para_front_Y[6] * t4 + 4 * para_front_Y[7] * t3 + 3 * para_front_Y[8] * t2 + 2 * para_front_Y[9] * t + para_front_Y[10];
			endfoot_vel_front_list(2, i) = 5 * para_front_Z[6] * t4 + 4 * para_front_Z[7] * t3 + 3 * para_front_Z[8] * t2 + 2 * para_front_Z[9] * t + para_front_Z[10];

			endfoot_vel_rear_list(0, i) = 5 * para_rear_X[6] * t4 + 4 * para_rear_X[7] * t3 + 3 * para_rear_X[8] * t2 + 2 * para_rear_X[9] * t + para_rear_X[10];
			endfoot_vel_rear_list(1, i) = 5 * para_rear_Y[6] * t4 + 4 * para_rear_Y[7] * t3 + 3 * para_rear_Y[8] * t2 + 2 * para_rear_Y[9] * t + para_rear_Y[10];
			endfoot_vel_rear_list(2, i) = 5 * para_rear_Z[6] * t4 + 4 * para_rear_Z[7] * t3 + 3 * para_rear_Z[8] * t2 + 2 * para_rear_Z[9] * t + para_rear_Z[10];
		}
	}
}