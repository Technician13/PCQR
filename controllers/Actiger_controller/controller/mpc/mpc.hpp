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

        int horizon;
        double alpha;
        int dim_leg;
        Vec6<double> adjust;
        DVec<double> Rf_Desire;
        Mat13<double> matrix_a;
        Mat13x12<double> matrix_b;
        Mat13<double> matrix_q;
        Vec13<double> x_initial;
        DVec<double> x_ref;
        Vec20<double> u_lower;
        Vec20<double> u_upper;
        DMat<double> matrix_A;
        DMat<double> matrix_B;
        DMat<double> matrix_Q;
        DMat<double> qp_H;
        DVec<double> qp_G;
        DMat<double> qp_A;
        DVec<double> qp_U;
        DVec<double> qp_L;
        DMat<double> eye_12h;
        Parameters* para;
        RobotModel* robotModel;
        MotionPlan* ref;
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