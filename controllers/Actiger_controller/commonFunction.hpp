/**
 * @file commonFunction.hpp
 * @brief Actiger通用函数类Common以及一些通用的函数和变量头文件
 * @author Bin & Tech13
 * @version 1.1
 * @date 2021-08-19
 * 
 * @copyright  Copyright  (c)  2021  Action
 * 
 */

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
	/* -------------------------------------- 仿真相关 -------------------------------------- */
	#define MAX_NUM_DISPLAY_DOT   (500)						    /**< webots中显示轨迹点的最大数目 */
	#define VISIBLE_BODY_TRAJ     (0)							/**< 是否在webots中显示机体轨迹 */
	#define VISIBLE_LF_TRAJ       (0)							/**< 是否在webots中显示左前腿足端轨迹 */
	#define VISIBLE_LR_TRAJ       (0)							/**< 是否在webots中显示左后腿足端轨迹 */
	#define VISIBLE_RF_TRAJ       (0)							/**< 是否在webots中显示右前腿足端轨迹 */
	#define VISIBLE_RR_TRAJ       (0)							/**< 是否在webots中显示右后腿足端轨迹 */

	/* -------------------------------------- 障碍相关 -------------------------------------- */
	#define FORWARD_DEADZONE        (0.1)
    #define BACKWARD_DEADZONE       (0.1)
    #define ERR_HIGHT               (-5.0)

	#define SLOPE_MID_LENGTH      	(3.0)                       /**< 坡面中段长度 */
	#define SLOPE_MID_HEIGHT      	(0.3)                       /**< 坡面中段高度 */
	#define SLOPE_UP_LENGTH         (1.2)						/**< 坡面上升段长度 */
	#define SLOPE_DOWN_LENGTH       (1.2)						/**< 坡面下降段长度 */
	#define SLOPE_UP_ANGLE          (-0.2527)					/**< 坡面上升段角度 */
	#define SLOPE_DOWN_ANGLE        (0.2527)					/**< 坡面下降段角度 */

	#define STAGE_HEIGHT            (0.1)						/**< 平台高度 */
	#define STAGE_LENGTH            (2.5)						/**< 平台长度 */

	#define SILL_HEIGHT             (0.05)						/**< 门槛高度 */
	#define SILL_WIDTH              (0.05)						/**< 门槛宽度 */

	#define CALANDRIA_HEIGHT        (0.05)						/**< 排管高度 */
	#define CALANDRIA_WIDTH         (0.05)						/**< 排管宽度 */
	#define CALANDRIA_DISTANCE      (0.4)						/**< 排管轴线间距 */

	#define STAIRS_HEIGHT           (0.1)						/**< 每一级阶梯高度 */
	#define STAIRS_DEPTH            (0.3)						/**< 每一级阶梯深度 */

	#define BRIDGE_HEIGHT           (0.1)                       /**< 双木桥高度 */
	#define BRIDGE_WIDTH            (0.2)                       /**< 双木桥宽度 */
	#define BRIDGE_LENGTH           (2.5)                       /**< 双木桥长度 */

	#define SEESAW_LENGTH           (2.4)                       /**< 跷跷板长度 */
	#define SEESAW_MID_HEIGHT       (0.3)						/**< 跷跷板中点高度 */
	#define SEESAW_UP_ANGLE         (-0.2527)                   /**< 跷跷板上升角度 */
	#define SEESAW_DOWN_ANGLE       (0.2527)                    /**< 跷跷板下降角度 */

	/* -------------------------------------- 环境相关 -------------------------------------- */	
    #define PI          			(3.14159265358979323846) 	/**< 圆周率 */
	#define MU          			(0.5)						/**< 摩擦系数 */
	#define TIME_STEP   			(0.005)					  	/**< 一次主循环时间 */
	#define MPC_PERIOD  			(0.005)					  	/**< MPC周期 */
	#define GRAVITY     			(9.81)                      /**< 重力加速度数值 */             

	/* -------------------------------------- 运动相关 -------------------------------------- */
	#define MAX_ACC_X   			(1.0)						/**< 机体X方向最大加速度 */
	#define MAX_ACC_Y   			(0.2)						/**< 机体Y方向最大加速度 */
	#define MAX_ACC_Z   			(1.0)						/**< 机体Z方向最大加速度 */
	#define MAX_CMD_VEL_X 			(0.1)						/**< 机体X方向最大速度 */
	#define MAX_CMD_VEL_Y 			(0.25)						/**< 机体Y方向最大速度 */
	#define MAX_CMD_HEIGHT_Z 		(0.53)						/**< 机体Z方向最大高度 */
	#define MIN_CMD_HEIGHT_Z 		(0.35)						/**< 机体X方向最小高度 */
	#define MAX_CMD_OMEGA_YAW 		(PI / 6.0)					/**< 机体偏航角最大转速 */
	#define MAX_CMD_STANCE_ROLL 	(PI / 6.0)					/**< 机体静态调整姿态时最大横滚角 */
	#define MAX_CMD_STANCE_PITCH 	(PI / 6.0)					/**< 机体静态调整姿态时最大俯仰角 */
	#define MAX_CMD_STANCE_YAW 		(PI / 6.0)					/**< 机体静态调整姿态时最大偏航角 */

	#define INIT_SWING_HEIGHT		(0.2)						/**< 摆动腿默认抬腿高度 */
	#define INIT_HEIGHT 			(0.5) 						/**< 机体默认站立高度 */

	#define ZOOM_FACTOR_SILL_STAGE  (0.4)						/**< 门槛和平台障碍下机体的满速占正常情况下的比例 */
	#define ZOOM_FACTOR_SLOPE       (0.5)                       /**< 坡面障碍下机体的满速占正常情况下的比例 */
	#define ZOOM_FACTOR_CALANDRIA   (0.3)						/**< 排管障碍下机体的满速占正常情况下的比例 */
	#define ZOOM_FACTOR_STAIRS      (0.25)                      /**< 阶梯障碍下机体的满速占正常情况下的比例 */
	#define ZOOM_FACTOR_SEESAW      (0.3)						/**< 跷跷板障碍下机体的满速占正常情况下的比例 */
	#define ZOOM_FACTOR_BRIDGE      (0.1)                       /**< 双木桥障碍下机体的满速占正常情况下的比例 */

	#define HEIGHT_FACTOR_STAIRS_STAGE   (0.8)					/**< 阶梯和平台障碍下机体抬腿高度占正常情况下的比例 */
	#define HEIGHT_FACTOR_CALANDRIA      (1.25)					/**< 排管障碍下机体抬腿高度占正常情况下的比例 */
	#define HEIGHT_FACTOR_SEESAW         (0.8)                  /**< 跷跷板障碍下机体抬腿高度占正常情况下的比例 */

	#define BODY_HEIGHT_FACTOR_SEESAW    (0.8)                  /**< 跷跷板障碍下机体高度占正常情况下的比例 */

	/* -------------------------------------- 步态相关 -------------------------------------- */	
	#define LYING 					(0)							
	#define STAND 					(1)
	#define WALK 					(2)
	#define TROT 					(3)
	#define FLYINF_TROT 			(4)
	#define STOP                    (5)

	/* -------------------------------------- 地形相关 -------------------------------------- */	
	#define FLAT 					(0)							/**< 平地 */
	#define SLOPE 					(1)							/**< 坡面 */
	#define STAGE					(2)							/**< 平台 */
	#define SILL					(3)							/**< 门槛 */
	#define CALANDRIA 				(4)							/**< 排管 */
	#define STAIRS 					(5)							/**< 阶梯 */
	#define BRIDGE 					(6)							/**< 双木桥 */
	#define HURDLES 				(7)							/**< 横杆 */
	#define BALL_POOL 				(8)							/**< 球池 */
	#define SEESAW 					(9)							/**< 跷跷板 */
	
	/* -------------------------------------- 机体属性 -------------------------------------- */	
	#define BODY_L 					(0.66)						/**< 机体长度 */
	#define BODY_W_F 				(0.14)						/**< 机体前宽 */
	#define BODY_W_R 				(0.11)						/**< 机体后宽 */
	#define BODY_MASS 				(29.4)						/**< 机体质量 */
	#define L0_L 					(0.096)						/**< 机体左侧第一段连杆 */
	#define L0_R 					(-0.096)					/**< 机体右侧第一段连杆 */
	#define L1 						(0.34)						/**< 大腿长度 */
	#define L2 						(0.34)						/**< 小腿长度 */

	#define M0 						(1.67)						/**< 第一段连杆质量 */
	#define M1 						(2.0)						/**< 大腿质量 */
	#define M2 						(0.5)						/**< 小腿质量 */

	#define MM0 					(0.0)
	#define MM1 					(0.0)
	#define MM2 					(0.0)

	#define KR0 					(1)
	#define KR1 					(1)
	#define KR2 					(1)

	#define PROCESS_NOISE_BODY_POS  (0.02)						/**< 机体位置过程噪声 */				
	#define PROCESS_NOISE_BODY_VEL  (0.02)						/**< 机体速度过程噪声 */
	#define PROCESS_NOISE_FOOT_POS  (0.002)						/**< 足端位置过程噪声 */
	#define MEASURE_NOISE_FOOT_POS  (0.001)						/**< 足端位置观测噪声 */
	#define MEASURE_NOISE_FOOT_VEL  (0.1)						/**< 足端速度观测噪声 */
	#define MEASURE_NOISE_FOOT_Z    (0.001)						/**< 足端高度测量噪声 */					

	/* -------------------------------------- 计算相关 -------------------------------------- */
	#define YES 					(1)
	#define NO 						(0)
	#define LIMIT_0					(0.000001)					/**< 无穷小 */
	#define LIMIT_INFINITE			(100000.0)					/**< 无穷大 */

	/* -------------------------------------- 规划相关 -------------------------------------- */
	#define FH_VAR_NUM_SINGLE  		(12)						/**< 落足点规划中决策变量的数目 */
	#define FH_KIN_CST_NUM      	(18)
	#define FH_OTH_CST_NUM      	(23)
	#define FH_KIN_CST_JAC_NUM      (62)
	#define FH_OTH_CST_JAC_NUM      (33)
	
	/* -------------------------------------- 遥控器相关 -------------------------------------- */
	// #define XBOX_BUTTON_A           (0x00)     				    /**< 按键A */ 
	// #define XBOX_BUTTON_B           (0x01)        				/**< 按键B */
	// #define XBOX_BUTTON_Y           (0x02)        				/**< Y */
	// #define XBOX_BUTTON_X           (0x03)        				/**< x */
	// #define XBOX_BUTTON_L           (0x04)        				/**< L */
	// #define XBOX_BUTTON_R           (0x05)        				/**< R */
	// #define XBOX_BUTTON_LZ          (0x06)        				/**< LZ */
	// #define XBOX_BUTTON_RZ          (0x07)        				/**< RZ */
	// #define XBOX_BUTTON_UP          (0x0d)        				/**< 方向键上 */
	// #define XBOX_BUTTON_DOWN        (0x0e)        				/**< 方向键下 */
	// #define XBOX_BUTTON_LEFT        (0x0f)        				/**< 方向键左 */
	// #define XBOX_BUTTON_RIGHT       (0x10)        				/**< 方向键右 */
	// #define XBOX_BUTTON_MINUS       (0x08)        				/**< 减号 */
	// #define XBOX_BUTTON_PLUS        (0x09)        				/**< 加号 */
	// #define XBOX_BUTTON_HOME        (0x0a)       				/**< home键 */
	// #define XBOX_BUTTON_LO          (0x0b)        				/**< 左摇杆按键 */
	// #define XBOX_BUTTON_RO          (0x0c)        				/**< 右摇杆按键 */
	// #define XBOX_AXIS_LX            (0x00)        				/**< 左摇杆X轴 */  
	// #define XBOX_AXIS_LY            (0x01)        				/**< 左摇杆Y轴 */  
	// #define XBOX_AXIS_RX            (0x03)        				/**< 右摇杆X轴 */  
	// #define XBOX_AXIS_RY            (0x04)        				/**< 右摇杆Y轴 */   
	// #define XBOX_AXIS_VAL_MIN       (-32767.0)  				/**< 摇杆的下限 */
	// #define XBOX_AXIS_VAL_MAX       (32767.0)  					/**< 摇杆的上限 */

	#define XBOX_BUTTON_A           (0x00)     				    /**< 按键A */ 
	#define XBOX_BUTTON_B           (0x01)        				/**< 按键B */
	#define XBOX_BUTTON_Y           (0x03)        				/**< Y */
	#define XBOX_BUTTON_X           (0x02)        				/**< x */
	#define XBOX_BUTTON_L           (0x04)        				/**< L */
	#define XBOX_BUTTON_R           (0x05)        				/**< R */
	#define XBOX_BUTTON_MINUS       (0x06)        				/**< 减号 */
	#define XBOX_BUTTON_PLUS        (0x07)        				/**< 加号 */
	#define XBOX_BUTTON_LO          (0x09)        				/**< 左摇杆按键 */
	#define XBOX_BUTTON_RO          (0x0a)        				/**< 右摇杆按键 */
	#define XBOX_AXIS_LX            (0x00)        				/**< 左摇杆X轴 */  
	#define XBOX_AXIS_LY            (0x01)        				/**< 左摇杆Y轴 */  
	#define XBOX_AXIS_RX            (0x03)        				/**< 右摇杆X轴 */  
	#define XBOX_AXIS_RY            (0x04)        				/**< 右摇杆Y轴 */   
	#define XBOX_AXIS_VAL_MIN       (-32767.0)  				/**< 摇杆的下限 */
	#define XBOX_AXIS_VAL_MAX       (32767.0)  					/**< 摇杆的上限 */

	#define XBOX_BUTTON_LZ          (0xff)        				/**< LZ */
	#define XBOX_BUTTON_RZ          (0xfe)        				/**< RZ */
	#define XBOX_BUTTON_UP          (0xfd)        				/**< 方向键上 */
	#define XBOX_BUTTON_DOWN        (0xfc)        				/**< 方向键下 */
	#define XBOX_BUTTON_LEFT        (0xfb)        				/**< 方向键左 */
	#define XBOX_BUTTON_RIGHT       (0xfa)        				/**< 方向键右 */
	#define XBOX_BUTTON_HOME        (0xf9)       				/**< home键 */

	/* -------------------------------------- 可选项 -------------------------------------- */
	//#define   MINISNAP_TEST										/**< 打印MinimumSnap规划的中间变量 */
	//#define   DISPLAY_RES										/**< 打印足端规划的结果 */

	constexpr size_t dim_floating = 6;							/**< Actiger浮动基座的自由度 */
	constexpr size_t dim_config = 18;							/**< Actiger整体的自由度 */
	constexpr size_t num_leg = 4;								/**< Actiger腿的数目 */
	constexpr size_t num_leg_joint = 3;							/**< Actiger单腿关节的数目 */

	constexpr size_t LF = 0; 									/**< 左前腿索引 */
	constexpr size_t LR = 1; 									/**< 左后腿索引 */
	constexpr size_t RF = 2; 									/**< 右前腿索引 */
	constexpr size_t RR = 3; 									/**< 右后腿索引 */

	/**
	 * @brief Actiger通用函数类
	 */
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