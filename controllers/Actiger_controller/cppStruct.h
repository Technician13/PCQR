#ifndef CPPSTRUCT_HPP
#define CPPSTRUCT_HPP

#include "cppTypes.h"
#include <iostream>

typedef struct
{
	float pitch;
	float roll;
	float yaw;

	float omega_pitch;
	float omega_roll;
	float omega_yaw;
} imu_t;

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

typedef struct
{
	legMsg_t lf;
	legMsg_t lr;
	legMsg_t rf;
	legMsg_t rr;
} robotLegMsg_t;

typedef struct
{
	float shoulder_torque;
	float hip_torque;
	float knee_torque;
} legTorque_t;

typedef struct
{
	legTorque_t lf_torque;
	legTorque_t lr_torque;
	legTorque_t rf_torque;
	legTorque_t rr_torque;
} robotLegTorque_t;

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
	bool standReady;
	bool stepOver;
	bool is_lf_rr_stand;
} handle_t;

typedef struct
{
	double lineA;
	double lineB;
	double lineC;
} beeline_t;

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