#ifndef COMMONFUNCTION_HPP
#define COMMONFUNCTION_HPP

#include "cppTypes.h"
#include "cppStruct.h"
#include <vector>
#include <list>
#include <string>
#include <cmath>
#include <ctime>
#include <iostream>
#include <utility>
#include <fstream>

using namespace std;

extern std::ofstream OutFile8;                                                                                                    

namespace robot_dim
{
    constexpr int num_q = 19;
    constexpr int num_qdot = 18;
    constexpr int num_act_joint = 12;
    constexpr int num_virtual = 6;
};

namespace robot_link
{
    constexpr int body = 0;
    constexpr int LF_foot = 1;
    constexpr int LR_foot = 2;
    constexpr int RF_foot = 3;
    constexpr int RR_foot = 4;
};

namespace quadruped
{	
	#define MAX_NUM_DISPLAY_DOT   (500)		
	#define VISIBLE_BODY_TRAJ     (0)			
	#define VISIBLE_LF_TRAJ       (0)				
	#define VISIBLE_LR_TRAJ       (0)		
	#define VISIBLE_RF_TRAJ       (0)				
	#define VISIBLE_RR_TRAJ       (0)							

	#define FORWARD_DEADZONE        (0.1)
    #define BACKWARD_DEADZONE       (0.1)
    #define ERR_HIGHT               (-5.0)

	#define SLOPE_MID_LENGTH      	(3.0)                    
	#define SLOPE_MID_HEIGHT      	(0.3)               
	#define SLOPE_UP_LENGTH         (1.2)					
	#define SLOPE_DOWN_LENGTH       (1.2)				
	#define SLOPE_UP_ANGLE          (-0.2527)			
	#define SLOPE_DOWN_ANGLE        (0.2527)		

	#define STAGE_HEIGHT            (0.1)		
	#define STAGE_LENGTH            (2.5)	

	#define SILL_HEIGHT             (0.05)			
	#define SILL_WIDTH              (0.05)		

	#define CALANDRIA_HEIGHT        (0.05)		
	#define CALANDRIA_WIDTH         (0.05)		
	#define CALANDRIA_DISTANCE      (0.4)		

	#define STAIRS_HEIGHT           (0.1)					
	#define STAIRS_DEPTH            (0.3)						

	#define BRIDGE_HEIGHT           (0.1)                      
	#define BRIDGE_WIDTH            (0.2)          
	#define BRIDGE_LENGTH           (2.5)      

	#define SEESAW_LENGTH           (2.4)          
	#define SEESAW_MID_HEIGHT       (0.3)				
	#define SEESAW_UP_ANGLE         (-0.2527)         
	#define SEESAW_DOWN_ANGLE       (0.2527)           

    #define PI          			(3.14159265358979323846) 	
	#define MU          			(0.5)						
	#define TIME_STEP   			(0.005)					  	
	#define MPC_PERIOD  			(0.005)					  	
	#define GRAVITY     			(9.81)                                 

	#define MAX_ACC_X   			(1.0)						
	#define MAX_ACC_Y   			(0.2)						
	#define MAX_ACC_Z   			(1.0)						
	#define MAX_CMD_VEL_X 			(0.8)						
	#define MAX_CMD_VEL_Y 			(0.25)						
	#define MAX_CMD_HEIGHT_Z 		(0.53)						
	#define MIN_CMD_HEIGHT_Z 		(0.35)						
	#define MAX_CMD_OMEGA_YAW 		(PI / 6.0)					
	#define MAX_CMD_STANCE_ROLL 	(PI / 6.0)					
	#define MAX_CMD_STANCE_PITCH 	(PI / 6.0)					
	#define MAX_CMD_STANCE_YAW 		(PI / 6.0)					

	#define INIT_SWING_HEIGHT		(0.2)						
	#define INIT_HEIGHT 			(0.5) 						

	#define ZOOM_FACTOR_SILL_STAGE  (0.4)						
	#define ZOOM_FACTOR_SLOPE       (0.5)                       
	#define ZOOM_FACTOR_CALANDRIA   (0.3)						
	#define ZOOM_FACTOR_STAIRS      (0.25)                      
	#define ZOOM_FACTOR_SEESAW      (0.3)						
	#define ZOOM_FACTOR_BRIDGE      (0.1)                       

	#define HEIGHT_FACTOR_STAIRS_STAGE   (0.8)					
	#define HEIGHT_FACTOR_CALANDRIA      (1.25)					
	#define HEIGHT_FACTOR_SEESAW         (0.8)                  

	#define BODY_HEIGHT_FACTOR_SEESAW    (0.8)                 

	#define LYING 					(0)							
	#define STAND 					(1)
	#define WALK 					(2)
	#define TROT 					(3)
	#define FLYINF_TROT 			(4)
	#define STOP                    (5)

	#define FLAT 					(0)							
	#define SLOPE 					(1)							
	#define STAGE					(2)							
	#define SILL					(3)							
	#define CALANDRIA 				(4)							
	#define STAIRS 					(5)							
	#define BRIDGE 					(6)							
	#define HURDLES 				(7)							
	#define BALL_POOL 				(8)							
	#define SEESAW 					(9)							
	
	#define BODY_L 					(0.66)						
	#define BODY_W_F 				(0.14)						
	#define BODY_W_R 				(0.11)						
	#define BODY_MASS 				(29.4)						
	#define L0_L 					(0.096)						
	#define L0_R 					(-0.096)					
	#define L1 						(0.34)						
	#define L2 						(0.34)						

	#define M0 						(1.67)						
	#define M1 						(2.0)						
	#define M2 						(0.5)						

	#define MM0 					(0.0)
	#define MM1 					(0.0)
	#define MM2 					(0.0)

	#define KR0 					(1)
	#define KR1 					(1)
	#define KR2 					(1)

	#define PROCESS_NOISE_BODY_POS  (0.02)								
	#define PROCESS_NOISE_BODY_VEL  (0.02)						
	#define PROCESS_NOISE_FOOT_POS  (0.002)						
	#define MEASURE_NOISE_FOOT_POS  (0.001)						
	#define MEASURE_NOISE_FOOT_VEL  (0.1)						
	#define MEASURE_NOISE_FOOT_Z    (0.001)										

	#define YES 					(1)
	#define NO 						(0)
	#define LIMIT_0					(0.000001)					
	#define LIMIT_INFINITE			(100000.0)					

	#define FH_VAR_NUM_SINGLE  		(12)						
	#define FH_KIN_CST_NUM      	(18)
	#define FH_OTH_CST_NUM      	(23)
	#define FH_KIN_CST_JAC_NUM      (62)
	#define FH_OTH_CST_JAC_NUM      (33)

	#define XBOX_BUTTON_A           (0x00)     				    
	#define XBOX_BUTTON_B           (0x01)        				
	#define XBOX_BUTTON_Y           (0x03)        				
	#define XBOX_BUTTON_X           (0x02)        				
	#define XBOX_BUTTON_L           (0x04)        				
	#define XBOX_BUTTON_R           (0x05)        				
	#define XBOX_BUTTON_MINUS       (0x06)        				
	#define XBOX_BUTTON_PLUS        (0x07)        				
	#define XBOX_BUTTON_LO          (0x09)        				
	#define XBOX_BUTTON_RO          (0x0a)        				
	#define XBOX_AXIS_LX            (0x00)        				
	#define XBOX_AXIS_LY            (0x01)        				
	#define XBOX_AXIS_RX            (0x03)        				
	#define XBOX_AXIS_RY            (0x04)        				 
	#define XBOX_AXIS_VAL_MIN       (-32767.0)  				
	#define XBOX_AXIS_VAL_MAX       (32767.0)  					

	#define XBOX_BUTTON_LZ          (0xff)        				
	#define XBOX_BUTTON_RZ          (0xfe)        				
	#define XBOX_BUTTON_UP          (0xfd)        				
	#define XBOX_BUTTON_DOWN        (0xfc)        				
	#define XBOX_BUTTON_LEFT        (0xfb)        				
	#define XBOX_BUTTON_RIGHT       (0xfa)        				
	#define XBOX_BUTTON_HOME        (0xf9)       				

	//#define   MINISNAP_TEST										
	//#define   DISPLAY_RES										

	constexpr size_t dim_floating = 6;							
	constexpr size_t dim_config = 18;							
	constexpr size_t num_leg = 4;								
	constexpr size_t num_leg_joint = 3;							

	constexpr size_t LF = 0; 									
	constexpr size_t LR = 1; 									
	constexpr size_t RF = 2; 									
	constexpr size_t RR = 3; 									

    class Common
    {
    private:
		
	protected:

	public:
            explicit Common(void);
            virtual ~Common(void);
           
            static Mat3<double> RotationMatrix(Vec3<double> p_actul_body_angle);
            static Mat3<double> RotationOfZMatrix(double yaw);
            static Mat3<double> GetSkewSymmetricMatrix(Vec3<double> vector);
            static DMat<double> Pinv(DMat<double>  matrix);
            static DMat<double> WeightedInverse(DMat<double> J, DMat<double> Winv);
    };
}

#endif // !COMMONFUNCTION_HPP