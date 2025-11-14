#include "fH_updataInfo.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>

#ifndef ROBOT
    #include "webotsApi.hpp"
#endif

using namespace quadruped;

FH_updataInfo::FH_updataInfo(Parameters *p_para, Terrain *p_terrain_now)
{
    terrain_now = p_terrain_now;
    foothold_Q = p_para->plan_foothold_Q;
    foothold_R = p_para->plan_foothold_R;
    Ts = p_para->Ts;
    k_v = p_para->plan_k_v;
    endfoot_max = p_para->plan_endfoot_max;
    body_length = BODY_L;
    body_width = BODY_W_F / 2.0 + BODY_W_R / 2.0;

    lf_hip_pos_body = p_para->lf_hip_pos_body;
    lr_hip_pos_body = p_para->lr_hip_pos_body;
    rf_hip_pos_body = p_para->rf_hip_pos_body;
    rr_hip_pos_body = p_para->rr_hip_pos_body;
}

FH_updataInfo::~FH_updataInfo(void) {}


void FH_updataInfo::GetXInital(double *x)
{
    x[0] = Ts / 2.0 * (desVelocity(0) + curVelocity(0)) + curPos(0);
    x[1] = Ts / 2.0 * (desVelocity(1) + curVelocity(1)) + curPos(1);
    x[2] = desire_z;

    x[3] = desVelocity(0);
    x[4] = desVelocity(1);
    x[5] = 0.0;

    x[6] = endfoot_desire_front(0);
    x[7] = endfoot_desire_front(1);
    x[8] = endfoot_desire_front(2);
    x[9] = endfoot_desire_rear(0);
    x[10] = endfoot_desire_rear(1);
    x[11] = endfoot_desire_rear(2);

    x[2] = desire_z + (x[8] + x[11]) / 2.0;
}


void FH_updataInfo::UpdateInfo(int p_is_lf_rr_stand, Vec3<double> p_curEuler, Vec3<double> p_curPos, Vec3<double> p_curVelocity,
                               double p_desire_yaw, double p_desire_z,uint8_t p_gaitNow,
                               Vec3<double> p_endfoot_now_front, Vec3<double> p_endfoot_now_rear,
                               Vec3<double> p_stance_now_front, Vec3<double> p_stance_now_rear)
{       
    is_lf_rr_stand = p_is_lf_rr_stand;

    desire_z = p_desire_z;

    curVelocity = p_curVelocity;
    curPos = p_curPos;
    curEuler = p_curEuler;

    if (desire_z > 0.55)
        desire_z = 0.55;
    else if (desire_z < 0.35)
        desire_z = 0.35;

    if (abs(p_desire_yaw - curEuler(2)) > 0.2)
        desire_yaw = abs(p_desire_yaw - curEuler(2)) / (p_desire_yaw - curEuler(2)) * 0.2 + curEuler(2);
    else
        desire_yaw = p_desire_yaw;

    cmdVelocity << 0.0, 0.0, 0.0;

    if(terrain_now->terrain_id == SILL || terrain_now->terrain_id == STAGE)
    {
        cmdVelocity[0] = ZOOM_FACTOR_SILL_STAGE * handle.x_speed * cos(curEuler(1));
    }
    else if(terrain_now->terrain_id == SLOPE)
    {
        cmdVelocity[0] = ZOOM_FACTOR_SLOPE * handle.x_speed * cos(curEuler(1));
    }
    else if(terrain_now->terrain_id == CALANDRIA)
    {
        cmdVelocity[0] = ZOOM_FACTOR_CALANDRIA * handle.x_speed * cos(curEuler(1));
    }
    else if(terrain_now->terrain_id == STAIRS)
    {
        cmdVelocity[0] = ZOOM_FACTOR_STAIRS * handle.x_speed * cos(curEuler(1));
    }
    else if(terrain_now->terrain_id == SEESAW)
    {
        cmdVelocity[0] = ZOOM_FACTOR_SEESAW * handle.x_speed * cos(curEuler(1));
    }
    else if(terrain_now->terrain_id == BRIDGE)
    {
        cmdVelocity[0] = ZOOM_FACTOR_BRIDGE * handle.x_speed * cos(curEuler(1));
    }
    else
    {
        cmdVelocity[0] = handle.x_speed * cos(curEuler(1));
    }
    cmdVelocity[1] = handle.y_speed;

    if (abs(cmdVelocity(0) - curVelocity(0)) < MAX_ACC_X * Ts)
        desVelocity(0) = cmdVelocity(0);
    else
        desVelocity(0) = curVelocity(0) + MAX_ACC_X * Ts * abs(cmdVelocity(0) - curVelocity(0)) / (cmdVelocity(0) - curVelocity(0));

    if (abs(cmdVelocity(1) - curVelocity(1)) < MAX_ACC_Y * Ts)
        desVelocity(1) = cmdVelocity(1);
    else
        desVelocity(1) = curVelocity(1) + MAX_ACC_Y * Ts * abs(cmdVelocity(1) - curVelocity(1)) / (cmdVelocity(1) - curVelocity(1));

    if (abs(cmdVelocity(2) - curVelocity(2)) < MAX_ACC_Z * Ts)
        desVelocity(2) = cmdVelocity(2);
    else
        desVelocity(2) = curVelocity(2) + MAX_ACC_Z * Ts * abs(cmdVelocity(2) - curVelocity(2)) / (cmdVelocity(2) - curVelocity(2));

    endfoot_now_front = p_endfoot_now_front;
    endfoot_now_rear = p_endfoot_now_rear;
    stance_front_pos = p_stance_now_front;
    stance_rear_pos = p_stance_now_rear;

    desire_hight = desire_z;

    bodyPos_desire = Vec3<double>::Zero();
    bodyPos_desire(0) = Ts / 2.0 * (desVelocity(0) + curVelocity(0)) + curPos(0);
    bodyPos_desire(1) = Ts / 2.0 * (desVelocity(1) + curVelocity(1)) + curPos(1);
    
    Mat3<double> rotation = dynacore::RotZ_Matrix(curEuler(2));
    double desOmega_yaw = (desire_yaw - curEuler(2)) / Ts;
    double adjust;
    if(p_gaitNow == TROT)
    {
        adjust = 0.0;
    }
    else if(p_gaitNow == STOP)
    {
        adjust = 0.0;
    }
        if (is_lf_rr_stand)
        {
            endfoot_desire_front(0) = Ts * desVelocity(0) / 2.0 + k_v(0) * (curVelocity(0) - desVelocity(0)) +
                                      0.5 * sqrt(desire_hight / GRAVITY) * (curVelocity(1) * desOmega_yaw) +
                                      bodyPos_desire(0) + rf_hip_pos_body(0) * rotation(0, 0) + (rf_hip_pos_body(1) - adjust) * rotation(0, 1);

            endfoot_desire_rear(0) = Ts * desVelocity(0) / 2.0 + k_v(0) * (curVelocity(0) - desVelocity(0)) +
                                     0.5 * sqrt(desire_hight / GRAVITY) * (curVelocity(1) * desOmega_yaw) +
                                     bodyPos_desire(0) + lr_hip_pos_body(0) * rotation(0, 0) + (lr_hip_pos_body(1) + adjust) * rotation(0, 1);

            endfoot_desire_front(1) = Ts * desVelocity(1) / 2.0 + k_v(1) * (curVelocity(1) - desVelocity(1)) +
                                      0.5 * sqrt(desire_hight / GRAVITY) * (-1.0 * curVelocity(0) * desOmega_yaw) +
                                      bodyPos_desire(1) + rf_hip_pos_body(0) * rotation(1, 0) + (rf_hip_pos_body(1) - adjust) * rotation(1, 1);

            endfoot_desire_rear(1) = Ts * desVelocity(1) / 2.0 + k_v(1) * (curVelocity(1) - desVelocity(1)) +
                                     0.5 * sqrt(desire_hight / GRAVITY) * (-1.0 * curVelocity(0) * desOmega_yaw) +
                                     bodyPos_desire(1) + lr_hip_pos_body(0) * rotation(1, 0) + (lr_hip_pos_body(1) + adjust) * rotation(1, 1);
        }
        else
        {
            endfoot_desire_front(0) = Ts * desVelocity(0) / 2.0 + k_v(0) * (curVelocity(0) - desVelocity(0)) +
                                      0.5 * sqrt(desire_hight / GRAVITY) * (curVelocity(1) * desOmega_yaw) +
                                      bodyPos_desire(0) + lf_hip_pos_body(0) * rotation(0, 0) + (lf_hip_pos_body(1) + adjust) * rotation(0, 1);

            endfoot_desire_rear(0) = Ts * desVelocity(0) / 2.0 + k_v(0) * (curVelocity(0) - desVelocity(0)) +
                                     0.5 * sqrt(desire_hight / GRAVITY) * (curVelocity(1) * desOmega_yaw) +
                                     bodyPos_desire(0) + rr_hip_pos_body(0) * rotation(0, 0) + (rr_hip_pos_body(1) - adjust) * rotation(0, 1);

            endfoot_desire_front(1) = Ts * desVelocity(1) / 2.0 + k_v(1) * (curVelocity(1) - desVelocity(1)) +
                                      0.5 * sqrt(desire_hight / GRAVITY) * (-1.0 * curVelocity(0) * desOmega_yaw) +
                                      bodyPos_desire(1) + lf_hip_pos_body(0) * rotation(1, 0) + (lf_hip_pos_body(1) + adjust) * rotation(1, 1);

            endfoot_desire_rear(1) = Ts * desVelocity(1) / 2.0 + k_v(1) * (curVelocity(1) - desVelocity(1)) +
                                     0.5 * sqrt(desire_hight / GRAVITY) * (-1.0 * curVelocity(0) * desOmega_yaw) +
                                     bodyPos_desire(1) + rr_hip_pos_body(0) * rotation(1, 0) + (rr_hip_pos_body(1) - adjust) * rotation(1, 1);
        }


        if(terrain_now->terrain_id == FLAT)
        {
            getEdgeFlg = 0;
        }

        if(terrain_now->terrain_id == SLOPE)
        {
            #ifndef ROBOT
                terrain_now->UpdataEdge(GetSlopeFirstEdge() , 0.0);
            #else
            #endif
            getEdgeFlg = 1;
        }
        else if(terrain_now->terrain_id == STAGE)
        {
            #ifndef ROBOT
                terrain_now->UpdataEdge(GetStageFirstEdge() , 0.0);
            #else
            #endif
            getEdgeFlg = 1;
        }
        else if(terrain_now->terrain_id == SILL)
        {
            #ifndef ROBOT
                terrain_now->UpdataEdge(GetSillFirstEdge() , 0.0);
            #else
            #endif
            getEdgeFlg = 1;
        }
        else if(terrain_now->terrain_id == CALANDRIA)
        {
            #ifndef ROBOT
                terrain_now->UpdataEdge(GetCalandriaFirstEdge() , 0.0);
            #else
            #endif
            getEdgeFlg = 1;
        }
        else if(terrain_now->terrain_id == STAIRS)
        {
            #ifndef ROBOT
                terrain_now->UpdataEdge(GetStairsFirstEdge() , 0.0);
            #else
            #endif
            getEdgeFlg = 1;
        }
        else if(terrain_now->terrain_id == SEESAW)
        {
            #ifndef ROBOT
                terrain_now->UpdataEdge(GetSeesawFirstEdge() , 0.0);
            #else
            #endif
            getEdgeFlg = 1;
        }
        else if(terrain_now->terrain_id == BRIDGE)
        {
            #ifndef ROBOT
                terrain_now->UpdataEdge(GetBridgeFirstEdge() , 0.0);
            #else
            #endif
            getEdgeFlg = 1;
        }
        
        endfoot_desire_front(2) = terrain_now->GetHight(endfoot_desire_front(0), endfoot_desire_front(1),curEuler[1]);
        endfoot_desire_rear(2) = terrain_now->GetHight(endfoot_desire_rear(0), endfoot_desire_rear(1),curEuler[1]);

    if (abs(endfoot_desire_front(2) - endfoot_desire_rear(2)) <= 0.01)
    {
        desEuler(0) = 0.0;
        desEuler(1) = 0.0;
    }
    else
    {
        desEuler(0) = 0.0;
        desEuler(1) = -1.0 * atan2((endfoot_desire_front(2) - endfoot_desire_rear(2)), (endfoot_desire_front(0) - endfoot_desire_rear(0)));
    }
    desEuler(2) = desire_yaw;

    double height;
    
    if(abs(stance_front_pos(2) - stance_rear_pos(2)) <= 0.01)
    {
        desEuler(1) = 0.0;
    }
    else
    {
        desEuler(1) = -1.0 * atan2((double)(stance_front_pos(2) - stance_rear_pos(2)), (double)(stance_front_pos(0) - stance_rear_pos(0)));
    }

    height = 0.5 * (stance_front_pos(2) + stance_rear_pos(2));

    if(terrain_now->terrain_id == SEESAW)
    {
        H = height + BODY_HEIGHT_FACTOR_SEESAW * desire_z * cos(desEuler(1));
    }
    else
    {
        H = height + desire_z * cos(desEuler(1));
    }
    
    desOmega = (desEuler - curEuler) / Ts;

    return_spatial_desire.setZero();
    return_spatial_dot_desire.setZero();
    return_desEndfoot_pos_front.setZero();
    return_desEndfoot_pos_rear.setZero();

}


void FH_updataInfo::GetXBound(double *ub, double *lb)
{
    ub[0] = curPos(0) + 2.0;
    ub[1] = curPos(1) + 2.0;
    ub[2] = 10.0;
    ub[3] = 2.0;
    ub[4] = 1.5;
    ub[5] = 0.0;
    ub[6] = curPos(0) + 3.0;
    ub[7] = curPos(1) + 3.0;
    ub[8] = 10.0;
    ub[9] = curPos(0) + 2.0;
    ub[10] = curPos(1) + 2.0;
    ub[11] = 5.0;

    lb[0] = curPos(0) - 2.0;
    lb[1] = curPos(1) - 2.0;
    lb[2] = -10.0;
    lb[3] = -2.0;
    lb[4] = -1.5;
    lb[5] = 0.0;
    lb[6] = curPos(0) - 2.0;
    lb[7] = curPos(1) - 2.0;
    lb[8] = -10.0;
    lb[9] = curPos(0) - 3.0;
    lb[10] = curPos(1) - 3.0;
    lb[11] = -5.0;
}
