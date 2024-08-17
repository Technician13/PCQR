/**
 * @file LinearKalmanFilter.cpp
 * @brief 线性Kalman滤波器
 * @author Bin & Tech13
 * @version 1.1
 * @date 2021-09-11
 * 
 * @copyright  Copyright  (c)  2021  Action
 * 
 */

#include "linearKalmanFilter.hpp"

/**
 * @brief LinearKalmanFilter类的构造函数
 */
quadruped::LinearKalmanFilter::LinearKalmanFilter(void)
{

}

/**
 * @brief LinearKalmanFilter类的析构函数
 */
quadruped::LinearKalmanFilter::~LinearKalmanFilter(void)
{

}

/**
 * @brief 线性Kalman滤波器的参数初始化
 */
void quadruped::LinearKalmanFilter::Init()
{
    dt = TIME_STEP;
    xhat.setZero();
    ps.setZero();
    vs.setZero();
    pzs.setZero();

    /* 状态转移矩阵初始化 */
    A.setZero();
    A.block(0,0,3,3) = Mat3<double>::Identity();
    A.block(0,3,3,3) = dt * Mat3<double>::Identity();
    A.block(3,3,3,3) = Mat3<double>::Identity();
    A.block(6,6,12,12) = Mat12<double>::Identity();

    /* 输入矩阵初始化 */
    B.setZero();
    B.block(0,0,3,3) = 0.5 * dt * dt * Mat3<double>::Identity();
    B.block(3,0,3,3) = dt * Mat3<double>::Identity();

    /* 观测矩阵初始化 */
    Mat3x6<double> H1 , H2;
    H1 << Mat3<double>::Identity() , Mat3<double>::Zero();
    H2 << Mat3<double>::Zero() , Mat3<double>::Identity();
    H.setZero();
    H.block(0,0,3,6) = H1;
    H.block(3,0,3,6) = H1;
    H.block(6,0,3,6) = H1;
    H.block(9,0,3,6) = H1;
    H.block(12,0,3,6) = H2;
    H.block(15,0,3,6) = H2;
    H.block(18,0,3,6) = H2;
    H.block(21,0,3,6) = H2;
    H.block(0,6,12,12) = -1.0 * Mat12<double>::Identity();
    H(27,17) = 1.0;
    H(26,14) = 1.0;
    H(25,11) = 1.0;
    H(24,8) = 1.0;

    /* 状态协方差矩阵初始化 */
    P.setZero();
    P = Mat18<double>::Identity();

    /* 初始过程噪声矩阵初始化 */
    Q_init.setZero();
    Q_init.block(0,0,3,3) = (dt / 20.0) * Mat3<double>::Identity();
    Q_init.block(3,3,3,3) = (dt * 9.8 / 20.0) * Mat3<double>::Identity();
    Q_init.block(6,6,12,12) = dt * Mat12<double>::Identity();

    /* 初始观测噪声矩阵初始化 */
    R_init.setIdentity();
}

/**
 * @brief 运行线性Kalman滤波器
 */
void quadruped::LinearKalmanFilter::Run(Vec3<double> p_bodyEuler , Vec3<double> p_aBody , Vec3<double> p_actul_body_angularVel ,
                                        Vec3<double> p_lf_hip_pos_body , Vec3<double> p_lr_hip_pos_body , Vec3<double> p_rf_hip_pos_body , Vec3<double> p_rr_hip_pos_body ,
                                        Vec3<double> p_lf_endfoot_hip , Vec3<double> p_lr_endfoot_hip , Vec3<double> p_rf_endfoot_hip , Vec3<double> p_rr_endfoot_hip ,
                                        Vec3<double> p_lf_endfoot_vel_hip , Vec3<double> p_lr_endfoot_vel_hip , Vec3<double> p_rf_endfoot_vel_hip , Vec3<double> p_rr_endfoot_vel_hip ,
                                        bool p_lf_contact_state , bool p_lr_contact_state , bool p_rf_contact_state ,  bool p_rr_contact_state)
{
    std::array<Vec3<double> , 4> hipPosBody = {p_lf_hip_pos_body , p_lr_hip_pos_body , p_rf_hip_pos_body , p_rr_hip_pos_body};
    std::array<Vec3<double> , 4> endfootHip = {p_lf_endfoot_hip , p_lr_endfoot_hip , p_rf_endfoot_hip , p_rr_endfoot_hip};
    std::array<Vec3<double> , 4> endfootVelHip = {p_lf_endfoot_vel_hip , p_lr_endfoot_vel_hip , p_rf_endfoot_vel_hip , p_rr_endfoot_vel_hip};
    std::array<bool , 4> contactState = {p_lf_contact_state , p_lr_contact_state , p_rf_contact_state , p_rr_contact_state};

    /* 过程噪声配置 */
    Q = Mat18<double>::Identity();
    Q.block(0,0,3,3) = Q_init.block(0,0,3,3) * PROCESS_NOISE_BODY_POS;
    Q.block(3,3,3,3) = Q_init.block(3,3,3,3) * PROCESS_NOISE_BODY_VEL;
    Q.block(6,6,12,12) = Q_init.block(6,6,12,12) * PROCESS_NOISE_FOOT_POS;

    /* 观测噪声配置 */
    R = Mat28<double>::Identity();
    R.block(0,0,12,12) = R_init.block(0,0,12,12) * MEASURE_NOISE_FOOT_POS;
    R.block(12,12,12,12) = R_init.block(12,12,12,12) * MEASURE_NOISE_FOOT_VEL;
    R.block(24,24,4,4) = R_init.block(24,24,4,4) *  MEASURE_NOISE_FOOT_Z;

    int qindex = 0;
    int rindex1 = 0;
    int rindex2 = 0;
    int rindex3 = 0;

    /* 重力向量 */
    Vec3<double> g = Vec3<double>::Zero();
    g[2] = -1.0 * GRAVITY;

    /* 惯性坐标系到机身坐标系的旋转矩阵 */
    Mat3<double> Rw2b = Common::RotationMatrix(p_bodyEuler);

    /* 输入量为惯性坐标系下的机体加速度 */
    Vec3<double> aWorld = Rw2b.transpose() * (p_aBody) + g;

    /* 信任度列表 */
    Vec4<double> trusts = Vec4<double>::Zero();

    /* 初始的位置和速度 */
    Vec3<double> p0 , v0;
    p0 << xhat[0] , xhat[1] , xhat[2];
    v0 << xhat[3] , xhat[4] , xhat[5];

    /* 参数装载 */
    for(int i = 0 ; i < 4 ; i++)
    {

        int i1 = 3 * i;

        /* 髋关节相对于CoM的位置 */
        Vec3<double> ph = Vec3<double>::Zero();
        /* 足端在机体坐标系下的位置 */
        Vec3<double> p_rel = Vec3<double>::Zero();
        /* 足端在机体坐标系下的速度 */
        Vec3<double> dp_rel = Vec3<double>::Zero();
        /* 足端在惯性坐标系下的位置 */
        Vec3<double> p_f = Vec3<double>::Zero();
        /* 足端速度在惯性坐标系下的描述 */
        Vec3<double> dp_f = Vec3<double>::Zero();

        

        ph = hipPosBody[i];
        p_rel = ph + endfootHip[i];
        dp_rel = endfootVelHip[i];
        p_f = Rw2b.transpose() * p_rel;
        dp_f = Rw2b.transpose() * (p_actul_body_angularVel.cross(p_rel) + dp_rel);

        

        /* 更新索引 */
        qindex = 6 + i1;
        rindex1 = i1;
        rindex2 = 12 + i1;
        rindex3 = 24 + i;

        /* 得到信任程度，有待优化 */
        double trust = 0.0;
        if(contactState[i])
        {
            trust = 1.0;
        }

        double high_suspect_number = 100.0;

        

        Q.block(qindex,qindex,3,3) = (1.0 + (1.0 - trust) * high_suspect_number) * Q.block(qindex,qindex,3,3);

        R.block(rindex1,rindex1,3,3) = 1.0 * R.block(rindex1,rindex1,3,3);
        R.block(rindex2,rindex2,3,3) = (1.0 + (1.0 - trust) * high_suspect_number) * R.block(rindex2,rindex2,3,3);
        R(rindex3,rindex3) = (1.0 + (1.0 - trust) * high_suspect_number) * R(rindex3,rindex3);


        trusts[i] = trust;

        /* 准备生成观测值向量 */
        ps.segment(i1 , 3) = -p_f;
        vs.segment(i1 , 3) = (1.0 - trust) * v0 + trust * (-dp_f);
        pzs[i] = (1.0 - trust) * (p0[2] + p_f(2));

        
    }

    z << ps , vs , pzs;

    /* 线性Kalman滤波器的运行主体 */
    xhat = A * xhat + B * aWorld;
    Mat18<double> At = A.transpose();
    Mat18<double> Pm = A * P * At + Q;
    Mat18x28<double> Ht = H.transpose();
    Vec28<double> y = z - H * xhat;
    Mat28<double> S = H * Pm * Ht + R;
    Vec28<double> S_y = S.lu().solve(y);
    xhat += Pm * Ht * S_y;
    Mat28x18<double> S_H = S.lu().solve(H);
    P = (Mat18<double>::Identity() - Pm * Ht * S_H) * Pm;

    /* MIT的迷之操作 */
    // Mat18<double> Pt = P.transpose();
    // P = (P + Pt) / 2.0;
}