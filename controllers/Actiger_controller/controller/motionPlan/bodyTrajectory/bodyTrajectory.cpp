#include "bodyTrajectory.hpp"
#include <iostream>
#include <cassert> 

using namespace quadruped;

BodyTrajectory::BodyTrajectory(Parameters* p_para)
{ 
   
    lf_hip_pos_body = p_para->lf_hip_pos_body;
    lr_hip_pos_body = p_para->lr_hip_pos_body;
    rf_hip_pos_body = p_para->rf_hip_pos_body;
    rr_hip_pos_body = p_para->rr_hip_pos_body;
}

BodyTrajectory::~BodyTrajectory()  {}


void BodyTrajectory::GetBodyTrajectory(void)
{   
    return_spatial_list.resize(dim_floating,grid);
    return_spatial_list.setZero();    
    return_spatial_dot_list.resize(dim_floating,grid);
    return_spatial_dot_list.setZero();

    Vec6<double> para_a = Vec6<double>::Zero();
    Vec6<double> para_b = Vec6<double>::Zero();
    Vec6<double> para_c = Vec6<double>::Zero();
    Vec6<double> para_d = Vec6<double>::Zero();

    double time = 0.0;

    para_a = spatial_now;
    para_b = spatial_dot_now;
    para_c = -1.0 / Ts / Ts * (3.0 * (spatial_now-spatial_des) + Ts * (2.0 * spatial_dot_now + spatial_dot_des));
    para_d = 1.0 / Ts / Ts / Ts * (2.0 * (spatial_now-spatial_des) + Ts * (spatial_dot_now + spatial_dot_des));

    for(int i=0;i<grid;i++)
    {      
        time = (double)(i+1)*MPC_PERIOD;
        return_spatial_list.block(0,i,dim_floating,1) = para_a + para_b * time + para_c * time * time + para_d * time * time * time;
        return_spatial_dot_list.block(0,i,dim_floating,1) = para_b + 2.0 * para_c * time + 3.0 * para_d * time * time;
    }
}


void BodyTrajectory::UpdateInfo(bool p_is_lf_rr_stand, double p_Ts,
                                Vec6<double> p_spatial_now,Vec6<double> p_spatial_dot_now,\
                                Vec6<double> p_spatial_des,Vec6<double> p_spatial_dot_des)
{
    Ts = p_Ts;
    if(p_Ts / MPC_PERIOD - (double)((int)(p_Ts / MPC_PERIOD)) > 0.5)
        grid = (int)(p_Ts/MPC_PERIOD)+1; 
    else
        grid = (int)(p_Ts/MPC_PERIOD); 

    is_lf_rr_stand = p_is_lf_rr_stand;
    spatial_now = p_spatial_now;
    spatial_des = p_spatial_des;
    spatial_dot_now = p_spatial_dot_now;
    spatial_dot_des = p_spatial_dot_des;
}
