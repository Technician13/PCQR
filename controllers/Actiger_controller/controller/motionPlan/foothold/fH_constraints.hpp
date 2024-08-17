#ifndef FH_CONSTRAINS_HPP
#define FH_CONSTRAINS_HPP

#include "fH_updataInfo.hpp"
#include <cmath>

namespace quadruped
{
    
    class FH_constraints
    {

        public:
            explicit FH_constraints(FH_updataInfo* p_fH_updataInfo);
            virtual ~FH_constraints(void);

            void GetKinematicsCst(Vec18<double>* p_cst,Vec3<double> p_bodyPos,Vec3<double> p_bodyVel,Vec3<double> p_foothold_front,Vec3<double> p_foothold_rear);
            void GetKinematicsCstBound(double* ub, double* lb);
            void GetKinematicsCstJacobian(Mat18x13<double>* p_cst_jacobian_value);
            void GetKinematicsCstJacobianPos(Mat18x13<double>* p_cst_jacobian_pos);

            void GetOtherCst(Vec23<double>* p_cst,Vec3<double> p_bodyPos,Vec3<double> p_bodyVel,Vec3<double> p_foothold_front,Vec3<double> p_foothold_rear);
            void GetOtherCstBound(double* ub, double* lb);
            void GetOtherCstJacobian(Mat23x12<double>* p_cst_jacobian_value,Vec3<double> p_bodyPos,Vec3<double> p_bodyVel,Vec3<double> p_foothold_front,Vec3<double> p_foothold_rear);
            void GetOtherCstJacobianPos(Mat23x12<double>* p_cst_jacobian_pos);
            void GetOtherCstHessian(double* lambda, Mat12<double>* p_cst_hessian_value);
            void GetOtherCstHessianPos(Mat12<double>* p_cst_hessian_pos);


        private:
            FH_updataInfo* updataInfo;
    };



}


#endif 



