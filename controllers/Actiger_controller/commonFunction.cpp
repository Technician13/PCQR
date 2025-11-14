#include "commonFunction.hpp"

using namespace quadruped;


Common::Common(void) {}


Common::~Common(void) {}


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


DMat<double> Common::WeightedInverse(DMat<double> J, DMat<double> Winv)
{
	DMat<double> Jinv;
	DMat<double> lambda = J * Winv * J.transpose();
	DMat<double> lambda_inv;
	lambda_inv = Pinv(lambda);
	Jinv = Winv * J.transpose() * lambda_inv;

	return Jinv;
}