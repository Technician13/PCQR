#include "mpc.hpp"
#include <iostream>

using namespace std;
using namespace quadruped;


Mpc::Mpc(RobotModel *p_robotModel, MotionPlan *p_ref, Parameters *p_para)
{
    robotModel = p_robotModel;
    ref = p_ref;
    para = p_para;
    dim_leg = num_leg;
    qpSolver = new QpSolver();

    horizon = 10;
    alpha = 5e-6;

    cout << "Mpc Init" << endl;
}


Mpc::~Mpc(void)
{
    delete qpSolver;
}


void Mpc::GetLimit(void)
{
    Vec5<double> ieq_vec_l;
    ieq_vec_l << -1.0 * LIMIT_INFINITE,
                 0.0,
                 -1.0 * LIMIT_INFINITE,
                 0.0,
                 5.0;

    Vec5<double> ieq_vec_u;
    ieq_vec_u << 0.0,
                 LIMIT_INFINITE,
                 0.0,
                 LIMIT_INFINITE,
                 300.0;

    for (size_t leg(0); leg < dim_leg; ++leg)
    {
        if (robotModel->legState[leg]->contact_state == 1)
        {
            u_upper.block(leg * 5, 0, 5, 1) = ieq_vec_u;
            u_lower.block(leg * 5, 0, 5, 1) = ieq_vec_l;
        }
        else
        {
            u_upper.block(leg * 5, 0, 5, 1) = Vec5<double>::Zero();
            u_lower.block(leg * 5, 0, 5, 1) = Vec5<double>::Zero();
        }
    }
    for (int i = 0; i < horizon; i++)
    {
        qp_U.block(i * 20, 0, 20, 1) = u_upper;
        qp_L.block(i * 20, 0, 20, 1) = u_lower;
    }
}


void Mpc::SetStateRef()
{
    for (int i = 0; i < horizon; i++)
    {
        x_ref.block(i * 13, 0, 3, 1) = ref->pBody_euler_des->GetQueueValue(i);
        x_ref.block(i * 13 + 3, 0, 3, 1) = ref->pBody_des->GetQueueValue(i);
        x_ref.block(i * 13 + 6, 0, 3, 1) = ref->vBody_euler_des->GetQueueValue(i);
        x_ref.block(i * 13 + 9, 0, 3, 1) = ref->vBody_des->GetQueueValue(i);
        x_ref(i * 13 + 12) = 1.0;
    }

    ref->pBody_euler_des->Pop();
    ref->pBody_des->Pop();
    ref->vBody_euler_des->Pop();
    ref->vBody_des->Pop();
}


void Mpc::IntegralAdjuest(void) {}


void Mpc::SetState(void)
{
    x_initial.block(0, 0, 3, 1) = robotModel->bodyEuler;
    x_initial.block(3, 0, 3, 1) = robotModel->bodyPosition;
    x_initial.block(6, 0, 3, 1) = robotModel->bodyEulerAngularVel;
    x_initial.block(9, 0, 3, 1) = robotModel->bodyVelocity;
    x_initial(12) = 1.0;
}


void Mpc::ResizeQpMats(void)
{
    matrix_A.resize(13 * horizon, 13);
    matrix_B.resize(13 * horizon, 12 * horizon);
    matrix_Q.resize(13 * horizon, 13 * horizon);
    qp_H.resize(12 * horizon, 12 * horizon);
    qp_G.resize(12 * horizon, 1);
    qp_A.resize(20 * horizon, 12 * horizon);
    qp_U.resize(20 * horizon, 1);
    qp_L.resize(20 * horizon, 1);
    eye_12h.resize(12 * horizon, 12 * horizon);
    x_ref.resize(13 * horizon, 1);
    Rf_Desire.resize(12 * horizon, 1);

    eye_12h.setIdentity();
    matrix_a.setIdentity();
    matrix_b.setZero();
    matrix_A.setZero();
    matrix_B.setZero();
    matrix_Q.setZero();
    qp_H.setZero();
    qp_G.setZero();
    qp_A.setZero();
    qp_U.setZero();
    qp_L.setZero();
    x_ref.setZero();
    Rf_Desire.setZero();
}


void Mpc::GetMPCModel(void)
{
    int contactLegNum = 0;
    for (int i = 0; i < 4; i++)
    {
        if (robotModel->legState[i]->contact_state == 1)
            contactLegNum++;
    }
    if (contactLegNum > 3)
    {
        alpha = 1e-7;

        matrix_q.setZero();

        matrix_q(0, 0) = 1.0;
        matrix_q(1, 1) = 1.0;
        matrix_q(2, 2) = 4.0;

        matrix_q(3, 3) = 4.0;
        matrix_q(4, 4) = 6.0;
        matrix_q(5, 5) = 8.0;

        matrix_q(6, 6) = 0.02;
        matrix_q(7, 7) = 0.02;
        matrix_q(8, 8) = 0.1;

        matrix_q(9, 9) = 0.2;
        matrix_q(10, 10) = 0.5;
        matrix_q(11, 11) = 0.2;

        matrix_q(12, 12) = 0.0;
    }
    else
    {
        alpha = 1e-7;

        matrix_q.setZero();

        matrix_q(0, 0) = 1.5;
        matrix_q(1, 1) = 1.0;
        matrix_q(2, 2) = 3.0;

        matrix_q(3, 3) = 6.0;
        matrix_q(4, 4) = 6.0;
        matrix_q(5, 5) = 1.0;

        matrix_q(6, 6) = 0.08;
        matrix_q(7, 7) = 0.08;
        matrix_q(8, 8) = 0.2;

        matrix_q(9, 9) = 0.5;
        matrix_q(10, 10) = 5.5;
        matrix_q(11, 11) = 0.5;

        matrix_q(12, 12) = 0.0;
    }

    double leg_contact = 0.;
    double period = (double)0.005;
    Mat3<double> skewSymmetricMatrix;

    Mat3<double> rotation = dynacore::RotZ_Matrix(robotModel->bodyEuler[2]);
    Mat3<double> inertia_world = robotModel->body2WorldRotation * robotModel->cenInertia.block(3, 3, 3, 3);

    matrix_a.block(0, 6, 3, 3) = Mat3<double>::Identity(3, 3) * period;
    matrix_a.block(3, 9, 3, 3) = Mat3<double>::Identity(3, 3) * period;
    matrix_a(11, 12) = -period * GRAVITY;
    for (int i = 0; i < dim_leg; ++i)
    {
        Vec3<double> endPos = robotModel->linkEndPos_world[i] - robotModel->bodyPosition;

        skewSymmetricMatrix = Common::GetSkewSymmetricMatrix(endPos);
        if (robotModel->legState[i]->contact_state == 1)
            leg_contact = (double)1.0;
        matrix_b.block(dim_floating, i * num_leg_joint, num_leg_joint, num_leg_joint) =
            inertia_world.inverse() * skewSymmetricMatrix * period * leg_contact;
        matrix_b.block(dim_floating + 3, i * num_leg_joint, num_leg_joint, num_leg_joint) =
            Mat3<double>::Identity(3, 3) * period / (double)BODY_MASS * leg_contact;
        leg_contact = (double)0.0;
    }
}


void Mpc::CalculateQPData(void)
{
    Mat13<double> powerMats[21];
    for (int i = 0; i < 21; i++)
    {
        powerMats[i].setZero();
    }
    powerMats[0].setIdentity();
    for (int i = 1; i < horizon + 1; i++)
    {
        powerMats[i] = matrix_a * powerMats[i - 1];
    }
    for (uint16_t r = 0; r < horizon; r++)
    {
        matrix_Q.block(13 * r, 13 * r, 13, 13) = matrix_q;

        matrix_A.block(13 * r, 0, 13, 13) = powerMats[r + 1]; 
        for (uint16_t c = 0; c < horizon; c++)
        {
            if (r >= c)
            {
                uint16_t a_num = r - c;
                matrix_B.block(13 * r, 12 * c, 13, 12) = powerMats[a_num] * matrix_b;
            }
        }
    }


    qp_H = 2.0 * (matrix_B.transpose() * matrix_Q * matrix_B + alpha * eye_12h);
    qp_G = 2.0 * matrix_B.transpose() * matrix_Q * (matrix_A * x_initial - x_ref);

    Mat5x3<double> single_control_constraint_mat;
    single_control_constraint_mat << 1.0, 0.0, -MU * 0.707,
        1.0, 0.0, MU * 0.707,
        0.0, 1.0, -MU * 0.707,
        0.0, 1.0, MU * 0.707,
        0.0, 0.0, 1.0;
    Mat20x12<double> control_constraint_mat = Mat20x12<double>::Zero();

    for (size_t i = 0; i < 4; i++)
    {
        control_constraint_mat.block(i * 5, i * 3, 5, 3) = single_control_constraint_mat;
    }

    for (size_t i = 0; i < horizon; i++)
    {
        qp_A.block(i * 20, i * 12, 20, 12) = control_constraint_mat;
    }
}


void Mpc::GetReactForce(void)
{
    Vec12<double> return_force = Vec12<double>::Zero();
    ResizeQpMats();
    pthread_mutex_lock(&ref_mutex);
    SetStateRef();
    pthread_mutex_unlock(&ref_mutex);
    pthread_mutex_lock(&model_mutex);
    GetLimit();
    SetState();
    GetMPCModel();
    pthread_mutex_unlock(&model_mutex);
    CalculateQPData();

    Rf_Desire = qpSolver->Solve(qp_H, qp_G, qp_A, qp_L, qp_U);
    for (int i(0); i < 12 * horizon; i++)
    {
        if (isnan(Rf_Desire(i)))
            Rf_Desire(i) = 0.0;
    }
    DMat<double> matX;
    matX.resize(horizon * 13, 1);
    matX = matrix_A * x_initial + matrix_B * Rf_Desire;

    for (int i(0); i < 12; i++)
        return_force(i) = Rf_Desire(i);

    pthread_mutex_lock(&f_des_mutex);
    for (int leg(0); leg < 4; leg++)
    {
        ref->legPlan[leg]->fr_des = return_force.block(leg * 3, 0, 3, 1);
    }
    pthread_mutex_unlock(&f_des_mutex);
}


void *MpcRun(void *p_mpc)
{
    Mpc *mpc = (Mpc *)p_mpc;
    while (1)
    {
        if (GetTimeFlagMpc())
            mpc->GetReactForce();
    }
    return (void *)0;
}