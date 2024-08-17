/**
 * @file singleLegControl.hpp
 * @brief Actiger单腿控制器SingleLeg头文件
 * @author Bin & Tech13
 * @version 1.1
 * @date 2021-08-20
 * 
 * @copyright  Copyright  (c)  2021  Action
 * 
 */

#ifndef SINGLELEGCONTROL_HPP
#define SINGLELEGCONTROL_HPP

#include "cppTypes.h"
#include "robotModel.hpp"

namespace quadruped 
{
    /**
     * @brief Actiger单腿控制器
     */
    class SingleLeg
    {
    private:

    protected:

    public:
        explicit SingleLeg(LegStateEstimate* p_legStste, Parameters* p_para);
        virtual ~SingleLeg(void);

        legTorque_t SingleStanceControl(Mat3<double> p_rotation, Vec3<double> p_grav, Vec3<double> coriolis, Vec3<double> p_reactForce);
        legTorque_t SingleSwingControl(Mat3<double> p_rotation,Vec3<double> p_grav, Vec3<double> coriolis, Vec3<double> p_desPos,Vec3<double> p_desVel);

        /** @brief WBC预留接口 */
        Vec3<double> torqueFeedForward;
        /** @brief WBC预留接口 */
        Vec3<double> qDes;
        /** @brief WBC预留接口 */
        Vec3<double> qdDes;

        /** @brief LegStateEstimate类的实例化对象，存储机器人单腿状态信息 */
        LegStateEstimate* legStste;
        /** @brief Parameters类的实例化对象，存储一些被频繁使用的参数 */
        Parameters* para;
    };
}

#endif // !SINGLELEGCONTROL_HPP