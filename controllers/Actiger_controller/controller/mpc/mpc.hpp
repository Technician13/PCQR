/**
 * @file mpc.hpp
 * @brief Actiger模型预测控制器头文件
 * @author Bin & Tech13
 * @version 1.1
 * @date 2021-08-20
 * 
 * @copyright  Copyright  (c)  2021  Action
 * 
 */

#ifndef MPC_HPP
#define MPC_HPP

#include <osqp.h>
#include <vector> 
#include <math.h>
#include "parameters.hpp"
#include "motionPlan.hpp"
#include "robotModel.hpp"
#include "qpSolver.hpp"
#include "legControl.hpp"
#include "cppTypes.h"

namespace quadruped
{
    /**
     * @brief Actiger模型预测控制器
     */
    class Mpc
    {
    private:
        void CalculateQPData(void);
        void ResizeQpMats(void);
        void SetState(void);
        void GetLimit(void);
        void GetMPCModel(void);
        void SetStateRef(void);
        void IntegralAdjuest(void);

        /** @brief Actiger模型预测控制器的预测时域 */
        int horizon;
        /** @brief 构建目标函数时使用 */
        double alpha;
        /** @brief Actiger腿的数目 */
        int dim_leg;
        /** @brief 预留接口  |  设置机体期望状态时限幅使用 */
        Vec6<double> adjust;
        /** @brief 二次规划求解器的输出 */
        DVec<double> Rf_Desire;
        /** @brief 状态空间方程的系统矩阵 */
        Mat13<double> matrix_a;
        /** @brief 状态空间方程的控制矩阵 */
        Mat13x12<double> matrix_b;
        /** @brief Actiger模型预测控制器的权重矩阵 */
        Mat13<double> matrix_q;
        /** @brief Actiger模型预测控制器的初始状态 */
        Vec13<double> x_initial;
        /** @brief Actiger模型预测控制器的期望状态向量 */
        DVec<double> x_ref;
        /** @brief 不等式约束的下界 */
        Vec20<double> u_lower;
        /** @brief 不等式约束的上界 */
        Vec20<double> u_upper;
        /** @brief 整体的系统矩阵 */
        DMat<double> matrix_A;
        /** @brief 整体的控制矩阵 */
        DMat<double> matrix_B;
        /** @brief 整体的权重矩阵 */
        DMat<double> matrix_Q;
        /** @brief 二次规划目标函数中二次项系数矩阵 */
        DMat<double> qp_H;
        /** @brief 二次规划目标函数中一次项系数矩阵 */
        DVec<double> qp_G;
        /** @brief 二次规划中不等式约束矩阵 */
        DMat<double> qp_A;
        /** @brief 二次规划中不等式约束上界 */
        DVec<double> qp_U;
        /** @brief 二次规划中不等式约束下界 */
        DVec<double> qp_L;
        /** @brief 单位阵 */
        DMat<double> eye_12h;
        /** @brief Parameters类的实例化对象，存储一些被频繁使用的参数 */
        Parameters* para;
        /** @brief RobotModel类的实例化对象，存储机器人模型信息 */
        RobotModel* robotModel;
        /** @brief MotionPlan类的实例化对象，Actiger规划器 */
        MotionPlan* ref;
        /** @brief QpSolver类的实例化对象，二次规划求解器 */
        QpSolver* qpSolver;

    protected:

    public:
        explicit Mpc(RobotModel* p_robotModel, MotionPlan* p_ref, Parameters* p_para);
        virtual ~Mpc();

        void GetReactForce(void);
    };
}

void* MpcRun(void* p_mpc);

#endif // !MPC_HPP