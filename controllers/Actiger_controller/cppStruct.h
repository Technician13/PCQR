/**
 * @file cppStruct.h
 * @brief Actiger工程中结构体的定义
 * @author Bin & Tech13
 * @version 1.1
 * @date 2021-08-19
 * 
 * @copyright  Copyright  (c)  2021  Action
 * 
 */

#ifndef CPPSTRUCT_HPP
#define CPPSTRUCT_HPP

#include "cppTypes.h"
#include <iostream>

/**
 * @brief IMU信息结构体：欧拉角 | 欧拉角速度
 */
typedef struct
{
	float pitch;
	float roll;
	float yaw;

	float omega_pitch;
	float omega_roll;
	float omega_yaw;
} imu_t;

/**
 * @brief 单腿信息结构体：关节角度 | 关节角速度 | 足端触地状态
 */
typedef struct
{
	float shoulder_pos;
	float shoulder_vel;
	float hip_pos;
	float hip_vel;
	float knee_pos;
	float knee_vel;
	uint8_t contact;
} legMsg_t;

/**
 * @brief 机器人整体腿部状态封装
 */
typedef struct
{
	legMsg_t lf;
	legMsg_t lr;
	legMsg_t rf;
	legMsg_t rr;
} robotLegMsg_t;

/**
 * @brief 单腿扭矩输出结构体：关节扭矩
 */
typedef struct
{
	float shoulder_torque;
	float hip_torque;
	float knee_torque;
} legTorque_t;

/**
 * @brief 机器人整体腿部扭矩封装
 */
typedef struct
{
	legTorque_t lf_torque;
	legTorque_t lr_torque;
	legTorque_t rf_torque;
	legTorque_t rr_torque;
} robotLegTorque_t;

/**
 * @brief 标志位结构体
 */
typedef struct
{
	bool start;
	bool unlock;
	uint8_t terrain;
	double x_speed;
	double y_speed;
	double z_height;
	double yaw_omega;
	double s_yaw;
	double s_pitch;
	double s_roll;
	/** @brief 站立完成的标志位 */
	bool standReady;
	/** @brief 一步转换完成的标志位 */
	bool stepOver;
	/** @brief 判断支撑腿的标志位 */
	bool is_lf_rr_stand;
} handle_t;

/**
 * @brief 摆动腿足端轨迹规划相关结构体
 */
typedef struct
{
	double lineA;
	double lineB;
	double lineC;
} beeline_t;

/**
 * @brief 遥控器键值映射结构体
 */
typedef struct xbox_map
{
	int time;
	int a;
	int b;
	int x;
	int y;
	int left;
	int right;
	int up;
	int down;
	int l;
	int r;
	int lz;
	int rz;
	int plus;
	int minus;
	int home;
	int lo;
	int ro;
	int lx;
	int ly;
	int rx;
	int ry;
} xbox_map_t;

#endif // !CPPSTRUCT_HPP