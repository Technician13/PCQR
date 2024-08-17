/**
 * @file LinearKalmanFilter.hpp
 * @brief 线性Kalman滤波器头文件
 * @author Bin & Tech13
 * @version 1.1
 * @date 2021-09-11
 * 
 * @copyright  Copyright  (c)  2021  Action
 * 
 */

#ifndef LINEARKALMANFILTER_HPP
#define LINEARKALMANFILTER_HPP

#include "cppTypes.h"
#include "commonFunction.hpp"
#include <array>

namespace quadruped
{
    /**
	 * @brief 线性Kalman滤波器
	 */
    class LinearKalmanFilter
    {
    private:

    protected:

    public:
        explicit LinearKalmanFilter(void);
		virtual ~LinearKalmanFilter(void);

        void Init();
        void Run(Vec3<double> p_bodyEuler , Vec3<double> p_aBody , Vec3<double> p_actul_body_angularVel ,
                 Vec3<double> p_lf_hip_pos_body , Vec3<double> p_lr_hip_pos_body , Vec3<double> p_rf_hip_pos_body , Vec3<double> p_rr_hip_pos_body ,
                 Vec3<double> p_lf_endfoot_hip , Vec3<double> p_lr_endfoot_hip , Vec3<double> p_rf_endfoot_hip , Vec3<double> p_rr_endfoot_hip ,
                 Vec3<double> p_lf_endfoot_vel_hip , Vec3<double> p_lr_endfoot_vel_hip , Vec3<double> p_rf_endfoot_vel_hip , Vec3<double> p_rr_endfoot_vel_hip ,
                 bool p_lf_contact_state , bool p_lr_contact_state , bool p_rf_contact_state , bool p_rr_contact_state);
        
        /** @brief 线性Kalman滤波器的采样时间间隔 */
        double dt;
        /** @brief 线性Kalman滤波器中的状态向量 [ P | v | P_lf | P_lr | P_rf | R_rr ] */
        Vec18<double> xhat;
        /** @brief 状态转移矩阵 */
        Mat18<double> A;
        /** @brief 输入矩阵 */
        Mat18x3<double> B;
        /** @brief 初始过程噪声矩阵 */
        Mat18<double> Q_init;
        /** @brief 过程噪声矩阵 */
        Mat18<double> Q;
        /** @brief 状态协方差矩阵 */
        Mat18<double> P;
        /** @brief 观测矩阵 */
        Mat28x18<double> H;
        /** @brief 测量值 */
        Vec28<double> z;
        /** @brief 初始观测噪声矩阵 */
        Mat28<double> R_init;
        /** @brief 观测噪声矩阵 */
        Mat28<double> R;
        /** @brief 构建观测值向量的辅助向量 */
        Vec12<double> ps;
        /** @brief 构建观测值向量的辅助向量 */
        Vec12<double> vs;
        /** @brief 构建观测值向量的辅助向量 */
        Vec4<double> pzs;
    };
}

#endif // !LINEARKALMANFILTER_HPP