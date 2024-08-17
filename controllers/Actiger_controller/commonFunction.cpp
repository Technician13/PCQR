/**
 * @file commonFunction.cpp
 * @brief Actiger通用函数类Common
 * @author Bin & Tech13
 * @version 1.1
 * @date 2021-08-19
 * 
 * @copyright  Copyright  (c)  2021  Action
 * 
 */

#include "commonFunction.hpp"

using namespace quadruped;

/**
 * @brief Common类的构造函数
 */
Common::Common(void) {}

/**
 * @brief Common类的析构函数
 */
Common::~Common(void) {}

/**
 * @brief 构建三维旋转矩阵[World -> Body | roll + pitch + yaw]，需要右乘一个World坐标系下的位置向量，得到一个Body坐标系下的位置向量
 * @param  p_actul_body_angle  Actiger实际的欧拉角
 * @return Mat3<double> 
 */
Mat3<double> Common::RotationMatrix(Vec3<double> p_actul_body_angle)
{
	double roll = p_actul_body_angle(0);
	double pitch = p_actul_body_angle(1);
	double yaw = p_actul_body_angle(2);

	Mat3<double> rotation;

	rotation(0, 0) = cos(pitch) * cos(yaw);
	rotation(0, 1) = cos(pitch) * sin(yaw);
	rotation(0, 2) = -sin(pitch);
	rotation(1, 0) = cos(yaw) * sin(pitch) * sin(roll) - cos(roll) * sin(yaw);
	rotation(1, 1) = cos(roll) * cos(yaw) + sin(pitch) * sin(roll) * sin(yaw);
	rotation(1, 2) = cos(pitch) * sin(roll);
	rotation(2, 0) = sin(roll) * sin(yaw) + cos(roll) * cos(yaw) * sin(pitch);
	rotation(2, 1) = cos(roll) * sin(pitch) * sin(yaw) - cos(yaw) * sin(roll);
	rotation(2, 2) = cos(pitch) * cos(roll);

	return rotation;
}

/**
 * @brief 构建三维旋转矩阵[World -> Body | yaw]，需要右乘一个World坐标系下的位置向量，得到一个Body坐标系下的位置向量
 * @param  yaw  Actiger实际的偏航角           
 * @return Mat3<double> 
 */
Mat3<double> Common::RotationOfZMatrix(double yaw)
{
	Mat3<double> rotation;

	rotation(0, 0) = cos(yaw);
	rotation(0, 1) = sin(yaw);
	rotation(0, 2) = 0.0;
	rotation(1, 0) = -sin(yaw);
	rotation(1, 1) = cos(yaw);
	rotation(1, 2) = 0.0;
	rotation(2, 0) = 0.0;
	rotation(2, 1) = 0.0;
	rotation(2, 2) = 1.0;

	return rotation;
}

/**
 * @brief 将向量转化为叉乘矩阵
 * @param  vector  待转化的向量        
 * @return Mat3<double> 
 */
Mat3<double> Common::GetSkewSymmetricMatrix(Vec3<double> vector)
{
	Mat3<double> returnMatrix = Mat3<double>::Zero(3, 3);

	returnMatrix(0, 1) = -vector(2);
	returnMatrix(0, 2) = vector(1);
	returnMatrix(1, 0) = vector(2);
	returnMatrix(1, 2) = -vector(0);
	returnMatrix(2, 0) = -vector(1);
	returnMatrix(2, 1) = vector(0);

	return returnMatrix;
}

/**
 * @brief 矩阵求伪逆
 * @param  matrix 待求伪逆矩阵         
 * @return DMat<double> 
 */
DMat<double> Common::Pinv(DMat<double> matrix)
{
	Eigen::JacobiSVD<DMat<double>> svd(matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
	double pinvtoler = 1.e-8;
	int row = matrix.rows();
	int col = matrix.cols();
	int k = std::min(row, col);
	DMat<double> X = DMat<double>::Zero(col, row);
	DMat<double> singularValues_inv = svd.singularValues();
	DMat<double> singularValues_inv_mat = DMat<double>::Zero(col, row);
	for (long i = 0; i < k; ++i)
	{
		if (singularValues_inv(i) > pinvtoler)
			singularValues_inv(i) = 1.0 / singularValues_inv(i);
		else
			singularValues_inv(i) = 0;
	}
	for (long i = 0; i < k; ++i)
	{
		singularValues_inv_mat(i, i) = singularValues_inv(i);
	}
	X = (svd.matrixV()) * (singularValues_inv_mat) * (svd.matrixU().transpose());

	return X;
}

/**
 * @brief 
 * @param  J                
 * @param  Winv             
 * @return DMat<double> 
 */
DMat<double> Common::WeightedInverse(DMat<double> J, DMat<double> Winv)
{
	DMat<double> Jinv;
	DMat<double> lambda = J * Winv * J.transpose();
	DMat<double> lambda_inv;
	lambda_inv = Pinv(lambda);
	Jinv = Winv * J.transpose() * lambda_inv;

	return Jinv;
}