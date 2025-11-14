#ifndef FH_UPDATAINFO_HPP
#define FH_UPDATAINFO_HPP

#include <vector>
#include "terrain.hpp"
#include "commonFunction.hpp"
#include "parameters.hpp"
#include "robotModel.hpp"

namespace quadruped
{
    class FH_updataInfo
    {
    private:
		
	protected:

    public:
        explicit FH_updataInfo(Parameters* p_para,Terrain* p_terrain_now);
        virtual ~FH_updataInfo(void);

        void GetXBound(double* ub,double* lb);
        void SetTerrain(uint8_t p_terrainType);
        void GetXInital(double* x);
        void UpdateInfo(int p_is_lf_rr_stand,Vec3<double> p_curEuler,Vec3<double> p_curPos,Vec3<double> p_curVelocity,\
                        double p_desire_yaw,double p_desire_z,uint8_t p_gaitNow, \
                        Vec3<double> p_endfoot_now_front,Vec3<double> p_endfoot_now_rear,\
                        Vec3<double> p_stance_now_front,Vec3<double> p_stance_now_rear);

        bool is_lf_rr_stand;

        double desire_z;
        double desire_yaw;
        
        double desire_hight;
        double desEndfoot_pos_z_front;
        double desEndfoot_pos_z_rear;
        Vec3<double> cmdVelocity;      
        Vec3<double> desVelocity;
        Vec3<double> desOmega;  
        Vec3<double> desEuler;     
        Vec3<double> curPos;
        Vec3<double> curEuler;
        Vec3<double> curVelocity;
        Vec3<double> bodyPos_desire;

        Vec3<double> endfoot_now_front;
        Vec3<double> endfoot_desire_front;
        Vec2<double> foothold_front_min;
        Vec2<double> foothold_front_max;

        Vec3<double> endfoot_now_rear;
        Vec3<double> endfoot_desire_rear;    
        Vec2<double> foothold_rear_min;
        Vec2<double> foothold_rear_max;

        Vec3<double> stance_front_pos;
        Vec3<double> stance_rear_pos;

        Vec2<double> hip_2_body_front;
        Vec2<double> hip_2_body_rear;

        double body_length;
        double body_width;

        double Ts;
        
        Vec2<double> k_v;
        Vec2<double> k_swing;

        Vec2<double> foothold_Q;
        Vec2<double> foothold_R;    
        
        Vec2<double> endfoot_max;

        Vec3<double> lf_hip_pos_body;
        Vec3<double> lr_hip_pos_body;
        Vec3<double> rf_hip_pos_body;
        Vec3<double> rr_hip_pos_body;

        Terrain* terrain_now;
 
        Vec6<double> return_spatial_dot_desire;
        Vec6<double> return_spatial_desire;
        Vec3<double> return_desEndfoot_pos_front;
        Vec3<double> return_desEndfoot_pos_rear;

	    double H;
        bool getEdgeFlg;
    };
}

#endif // !FH_UPDATAINFO_HPP