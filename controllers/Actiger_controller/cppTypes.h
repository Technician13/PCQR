#ifndef CPPTYPES_H
#define CPPTYPES_H

#include <Eigen/Dense>
#include <Eigen/Geometry>


template <typename T>
using Vec2 = Eigen::Matrix<T, 2, 1>;

template <typename T>
using Vec3 = Eigen::Matrix<T, 3, 1>;

template <typename T>
using Vec4 = Eigen::Matrix<T, 4, 1>;

template <typename T>
using Vec5 = Eigen::Matrix<T, 5, 1>;

template <typename T>
using Vec6 = Eigen::Matrix<T, 6, 1>;

template <typename T>
using Vec8 = Eigen::Matrix<T, 8, 1>;

template <typename T>
using Vec9 = Eigen::Matrix<T, 9, 1>;

template <typename T>
using Vec10 = Eigen::Matrix<T, 10, 1>;

template <typename T>
using Vec11 = Eigen::Matrix<T, 11, 1>;

template <typename T>
using Vec12 = Eigen::Matrix<T, 12, 1>;

template <typename T>
using Vec13 = Eigen::Matrix<T, 13, 1>;

template <typename T>
using Vec18 = Eigen::Matrix<T, 18, 1>;

template <typename T>
using Vec19 = Eigen::Matrix<T, 19, 1>;

template <typename T>
using Vec20 = Eigen::Matrix<T, 20, 1>;

template <typename T>
using Vec23 = Eigen::Matrix<T, 23, 1>;

template <typename T>
using Vec28 = Eigen::Matrix<T, 28, 1>;

template <typename T>
using Quat = typename Eigen::Quaternion<T>;

template <typename T>
using DVec = Eigen::Matrix<T, Eigen::Dynamic, 1>;

template <typename T>
using Mat2 = Eigen::Matrix<T, 2, 2>;

template <typename T>
using Mat2x13 = Eigen::Matrix<T, 2, 13>;

template <typename T>
using Mat3 = Eigen::Matrix<T, 3, 3>;

template <typename T>
using Mat3x6 = Eigen::Matrix<T, 3, 6>;

template <typename T>
using Mat3x18 = Eigen::Matrix<T, 3, 18>;

template <typename T>
using Mat4 = Eigen::Matrix<T, 4, 4>;

template <typename T>
using Mat5x3 = Eigen::Matrix<T, 5, 3>;

template <typename T>
using Mat6 = Eigen::Matrix<T, 6, 6>;

template <typename T>
using Mat6x3 = Eigen::Matrix<T, 6, 3>;

template <typename T>
using Mat11x13 = Eigen::Matrix<T, 11, 13>;

template <typename T>
using Mat12 = Eigen::Matrix<T, 12, 12>;

template <typename T>
using Mat13 = Eigen::Matrix<T, 13, 13>;

template <typename T>
using Mat13x12 = Eigen::Matrix<T, 13, 12>;

template <typename T>
using Mat18 = Eigen::Matrix<T, 18, 18>;

template <typename T>
using Mat18x3 = Eigen::Matrix<T, 18, 3>;

template <typename T>
using Mat18x13 = Eigen::Matrix<T, 18, 13>;

template <typename T>
using Mat18x28 = Eigen::Matrix<T, 18, 28>;

template <typename T>
using Mat20x12 = Eigen::Matrix<T, 20, 12>;

template <typename T>
using Mat23x12 = Eigen::Matrix<T, 23, 12>;

template <typename T>
using Mat28 = Eigen::Matrix<T, 28, 28>;

template <typename T>
using Mat28x18 = Eigen::Matrix<T, 28, 18>;

template <typename T>
using DMat = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

#endif // ！CPPTYPES_H