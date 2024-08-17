/**
 * @file main.cpp
 * @brief 主函数文件
 * @author Bin & Tech13
 * @version 1.1
 * @date 2021-08-18 2023-07-18
 * 
 * @copyright  Copyright  (c)  2021  Action
 * 
 */

#include "cppTypes.h"
#include "locomotionController.hpp"
#include "positionEstimate.hpp"
#include <fstream>
#include <string>
#include <fstream>

#ifdef ROBOT
    #include "embedded.hpp"
#else
	#include "webotsApi.hpp"
#endif

using namespace quadruped;

/* 文本输出 */
// std::ofstream outfile;
// std::string str="result.txt";
std::ofstream OutFile("result.txt");
std::ofstream OutFile8("planningperiod.txt");

void SendDebugInfo(LocomotionCtrl* quadrupedCtrl_);

int main(void)
{	
	#ifndef ROBOT
		#if VISIBLE_BODY_TRAJ
			BodyTrajSet();
		#endif
		#if VISIBLE_LF_TRAJ
			LFTrajSet();
		#endif
		#if VISIBLE_LR_TRAJ
			LRTrajSet();
		#endif
		#if VISIBLE_RF_TRAJ
			RFTrajSet();
		#endif
		#if VISIBLE_RR_TRAJ
			RRTrajSet();
		#endif
	#endif	
	LocomotionCtrl* quadrupedCtrl = new LocomotionCtrl();
	Init();
	quadrupedCtrl->robotModel->UpdataModel(Vec12<double>::Zero(), Vec12<double>::Zero());

	// quadrupedCtrl->motionPlan->SetTerrain(STAIRS);
	// quadrupedCtrl->motionPlan->stairs->UpdataEdge(1.25,-0.45);
	/*****************************************************开辟线程运行MotionPlan**************************************************/
	pthread_t motionPlan_pid;	
	int motionPlanIsok = pthread_create(&motionPlan_pid, NULL, MotionPlanRun,(void*)(quadrupedCtrl->motionPlan));
	if(motionPlanIsok == 0)
		pthread_detach(motionPlan_pid);
	else
		std::cout<<"pthread_create err!!!!!!"<<std::endl;
	/*******************************************************************************************************************************/

	// outfile.open(str);
	while(1)	
	{		
		/* 程序周期5ms */
		if(GetTimeFlag())
		{			
			#ifdef ROBOT
				Embedded();
			#else
				WebotsApi();
			#endif			
			
        	quadrupedCtrl->RobotControl();
			
			
			SendDebugInfo(quadrupedCtrl);
			OutFile<<" x "<< quadrupedCtrl->robotModel->bodyPosition[0]
					<< " z "<< quadrupedCtrl->robotModel->bodyPosition[2]
					<< " pitch "<< quadrupedCtrl->robotModel->bodyEuler[1]
					<< " roll "<< quadrupedCtrl->robotModel->bodyEuler[0] << "\n";
					
		}
	}
	OutFile.close();
	delete quadrupedCtrl;
	return (0);
}

/**
 * @description: 打印调试数据
 * @param {LocomotionCtrl*} quadrupedCtrl_
 * @return {*}
 */
void SendDebugInfo(LocomotionCtrl* quadrupedCtrl_)
{
	// std::cout<<"-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------"<<std::endl;
	// std::cout<<"[JOINT ANGLE] -------------------------------------------------------------------------------------------------------------------------------------------"<<std::endl;
	// std::cout<<"LF_SHOULDER ： "<<quadrupedCtrl_->robotModel->legState[0]->joint_angle[0]<<"     "
	// 		 <<"LF_HIP ： "     <<quadrupedCtrl_->robotModel->legState[0]->joint_angle[1]<<"     "
	// 		 <<"LF_KNEE ： "    <<quadrupedCtrl_->robotModel->legState[0]->joint_angle[2]<<"     "
	// 		 <<"LF_CONTACT ： " <<quadrupedCtrl_->robotModel->legState[0]->contact_state <<"     "<<std::endl;

	// std::cout<<"==============================================================================================="<<std::endl;
	// std::cout<<"LF : "<<quadrupedCtrl_->robotModel->legState[0]->joint_anglularVel[0]<<"	"
	// 				  <<quadrupedCtrl_->robotModel->legState[0]->joint_anglularVel[1]<<"	"
	// 				  <<quadrupedCtrl_->robotModel->legState[0]->joint_anglularVel[2]<<"	"<<std::endl;

	// std::cout<<"LR : "<<quadrupedCtrl_->robotModel->legState[1]->joint_anglularVel[0]<<"	"
	// 				  <<quadrupedCtrl_->robotModel->legState[1]->joint_anglularVel[1]<<"	"
	// 				  <<quadrupedCtrl_->robotModel->legState[1]->joint_anglularVel[2]<<"	"<<std::endl;
	
	// std::cout<<"RF : "<<quadrupedCtrl_->robotModel->legState[2]->joint_anglularVel[0]<<"	"
	// 				  <<quadrupedCtrl_->robotModel->legState[2]->joint_anglularVel[1]<<"	"
	// 				  <<quadrupedCtrl_->robotModel->legState[2]->joint_anglularVel[2]<<"	"<<std::endl;

	// std::cout<<"RR : "<<quadrupedCtrl_->robotModel->legState[3]->joint_anglularVel[0]<<"	"
	// 				  <<quadrupedCtrl_->robotModel->legState[3]->joint_anglularVel[1]<<"	"
	// 				  <<quadrupedCtrl_->robotModel->legState[3]->joint_anglularVel[2]<<"	"<<std::endl;

	// std::cout<<"LR_SHOULDER ： "<<quadrupedCtrl_->robotModel->legState[1]->joint_angle[0]<<"     "
	// 		 <<"LR_HIP ： "	    <<quadrupedCtrl_->robotModel->legState[1]->joint_angle[1]<<"     "
	// 		 <<"LR_KNEE ： "	<<quadrupedCtrl_->robotModel->legState[1]->joint_angle[2]<<"     "
	// 		 <<"LR_CONTACT ： " <<quadrupedCtrl_->robotModel->legState[1]->contact_state <<"     "<<std::endl;

	// std::cout<<"RF_SHOULDER ： "<<quadrupedCtrl_->robotModel->legState[2]->joint_angle[0]<<"     "
	// 		 <<"RF_HIP ： "		<<quadrupedCtrl_->robotModel->legState[2]->joint_angle[1]<<"     "
	// 		 <<"RF_KNEE ： "	<<quadrupedCtrl_->robotModel->legState[2]->joint_angle[2]<<"     "
	// 		 <<"RF_CONTACT ： " <<quadrupedCtrl_->robotModel->legState[2]->contact_state <<"     "<<std::endl;

	// std::cout<<"RR_SHOULDER ： "<<quadrupedCtrl_->robotModel->legState[3]->joint_angle[0]<<"     "
	// 		 <<"RR_HIP ： "		<<quadrupedCtrl_->robotModel->legState[3]->joint_angle[1]<<"     "
	// 		 <<"RR_KNEE ： "	<<quadrupedCtrl_->robotModel->legState[3]->joint_angle[2]<<"     "
	// 		 <<"RR_CONTACT ： " <<quadrupedCtrl_->robotModel->legState[3]->contact_state <<"     "<<std::endl;

	// std::cout<<"[ENDFOOT POS] -------------------------------------------------------------------------------------------------------------------------------------------"<<std::endl;
	// std::cout<<"LF_X ： "<<quadrupedCtrl_->robotModel->legState[0]->endfoot_hip[0]<<"     "
	// 		 <<"LF_Y ： "<<quadrupedCtrl_->robotModel->legState[0]->endfoot_hip[1]<<"     "
	// 		 <<"LF_Z ： "<<quadrupedCtrl_->robotModel->legState[0]->endfoot_hip[2]<<"     "<<std::endl;

	// std::cout<<"LR_X ： "<<quadrupedCtrl_->robotModel->legState[1]->endfoot_hip[0]<<"     "
	// 		 <<"LR_Y ： "<<quadrupedCtrl_->robotModel->legState[1]->endfoot_hip[1]<<"     "
	// 		 <<"LR_Z ： "<<quadrupedCtrl_->robotModel->legState[1]->endfoot_hip[2]<<"     "<<std::endl;

	// std::cout<<"RF_X ： "<<quadrupedCtrl_->robotModel->legState[2]->endfoot_hip[0]<<"     "
	// 		 <<"RF_Y ： "<<quadrupedCtrl_->robotModel->legState[2]->endfoot_hip[1]<<"     "
	// 		 <<"RF_Z ： "<<quadrupedCtrl_->robotModel->legState[2]->endfoot_hip[2]<<"     "<<std::endl;
	
	// std::cout<<"RR_X ： "<<quadrupedCtrl_->robotModel->legState[3]->endfoot_hip[0]<<"     "
	// 		 <<"RR_Y ： "<<quadrupedCtrl_->robotModel->legState[3]->endfoot_hip[1]<<"     "
	// 		 <<"RR_Z ： "<<quadrupedCtrl_->robotModel->legState[3]->endfoot_hip[2]<<"     "<<std::endl;

	// std::cout<<"[ENDFOOT POS] -------------------------------------------------------------------------------------------------------------------------------------------"<<std::endl;
	// std::cout<<"LF_X ： "<<quadrupedCtrl_->robotModel->linkEndPos_world[0](0)<<"     "
	// 		 <<"LF_Y ： "<<quadrupedCtrl_->robotModel->linkEndPos_world[0](1)<<"     "
	// 		 <<"LF_Z ： "<<quadrupedCtrl_->robotModel->linkEndPos_world[0](2)<<"     "<<std::endl;

	// std::cout<<"LR_X ： "<<quadrupedCtrl_->robotModel->linkEndPos_world[1](0)<<"     "
	// 		 <<"LR_Y ： "<<quadrupedCtrl_->robotModel->linkEndPos_world[1](1)<<"     "
	// 		 <<"LR_Z ： "<<quadrupedCtrl_->robotModel->linkEndPos_world[1](2)<<"     "<<std::endl;

	// std::cout<<"RF_X ： "<<quadrupedCtrl_->robotModel->linkEndPos_world[2](0)<<"     "
	// 		 <<"RF_Y ： "<<quadrupedCtrl_->robotModel->linkEndPos_world[2](1)<<"     "
	// 		 <<"RF_Z ： "<<quadrupedCtrl_->robotModel->linkEndPos_world[2](2)<<"     "<<std::endl;
	
	// std::cout<<"RR_X ： "<<quadrupedCtrl_->robotModel->linkEndPos_world[3](0)<<"     "
	// 		 <<"RR_Y ： "<<quadrupedCtrl_->robotModel->linkEndPos_world[3](1)<<"     "
	// 		 <<"RR_Z ： "<<quadrupedCtrl_->robotModel->linkEndPos_world[3](2)<<"     "<<std::endl;

	// std::cout<<"[MOTIONPLAN] -------------------------------------------------------------------------------------------------------------------------------------------"<<std::endl;
	// std::cout<<"gaitNow ： "<<(int)quadrupedCtrl_->motionPlan->gaitNow<<"     "
	// 		 <<"unlock ： "<<handle.unlock<<"     "
	// 		 <<"z_height ： "<<handle.z_height<<"     "
	// 		 <<"stepOver ： "<<handle.stepOver<<"     "
	// 		 <<"is_lf_rr_stand ： "<<handle.is_lf_rr_stand<<"     "<<std::endl;

	// std::cout<<"[ENDFOOT FORCE] -------------------------------------------------------------------------------------------------------------------------------------------"<<std::endl;
	// std::cout<<"LF_X ： "<<quadrupedCtrl_->motionPlan->legPlan[0]->fr_des[0]<<"     "
	// 		 <<"LF_Y ： "<<quadrupedCtrl_->motionPlan->legPlan[0]->fr_des[1]<<"     "
	// 		 <<"LF_Z ： "<<quadrupedCtrl_->motionPlan->legPlan[0]->fr_des[2]<<"     "<<std::endl;

	// std::cout<<"LR_X ： "<<quadrupedCtrl_->motionPlan->legPlan[1]->fr_des[0]<<"     "
	// 		 <<"LR_Y ： "<<quadrupedCtrl_->motionPlan->legPlan[1]->fr_des[1]<<"     "
	// 		 <<"LR_Z ： "<<quadrupedCtrl_->motionPlan->legPlan[1]->fr_des[2]<<"     "<<std::endl;

	// std::cout<<"RF_X ： "<<quadrupedCtrl_->motionPlan->legPlan[2]->fr_des[0]<<"     "
	// 		 <<"RF_Y ： "<<quadrupedCtrl_->motionPlan->legPlan[2]->fr_des[1]<<"     "
	// 		 <<"RF_Z ： "<<quadrupedCtrl_->motionPlan->legPlan[2]->fr_des[2]<<"     "<<std::endl;
	
	// std::cout<<"RR_X ： "<<quadrupedCtrl_->motionPlan->legPlan[3]->fr_des[0]<<"     "
	// 		 <<"RR_Y ： "<<quadrupedCtrl_->motionPlan->legPlan[3]->fr_des[1]<<"     "
	// 		 <<"RR_Z ： "<<quadrupedCtrl_->motionPlan->legPlan[3]->fr_des[2]<<"     "<<std::endl;

	// std::cout<<"[GAMEPAD] -------------------------------------------------------------------------------------------------------------------------------------------"<<std::endl;
	// std::cout<<"GAMEPAD_a：" <<quadrupedCtrl_->motionPlan->gamepad->map.a<<std::endl;
	// std::cout<<"GAMEPAD_b：" <<quadrupedCtrl_->motionPlan->gamepad->map.b<<std::endl;
	// std::cout<<"GAMEPAD_x：" <<quadrupedCtrl_->motionPlan->gamepad->map.x<<std::endl;
	// std::cout<<"GAMEPAD_y：" <<quadrupedCtrl_->motionPlan->gamepad->map.y<<std::endl;

	// std::cout<<"GAMEPAD_left：" <<quadrupedCtrl_->motionPlan->gamepad->map.left<<std::endl;
	// std::cout<<"GAMEPAD_right：" <<quadrupedCtrl_->motionPlan->gamepad->map.right<<std::endl;
	// std::cout<<"GAMEPAD_up：" <<quadrupedCtrl_->motionPlan->gamepad->map.up<<std::endl;
	// std::cout<<"GAMEPAD_down：" <<quadrupedCtrl_->motionPlan->gamepad->map.down<<std::endl;

	// std::cout<<"GAMEPAD_l：" <<quadrupedCtrl_->motionPlan->gamepad->map.l<<std::endl;
	// std::cout<<"GAMEPAD_r：" <<quadrupedCtrl_->motionPlan->gamepad->map.r<<std::endl;

	// std::cout<<"GAMEPAD_lz：" <<quadrupedCtrl_->motionPlan->gamepad->map.lz<<std::endl;
	// std::cout<<"GAMEPAD_rz：" <<quadrupedCtrl_->motionPlan->gamepad->map.rz<<std::endl;

	// std::cout<<"GAMEPAD_plus：" <<quadrupedCtrl_->motionPlan->gamepad->map.plus<<std::endl;
	// std::cout<<"GAMEPAD_minus：" <<quadrupedCtrl_->motionPlan->gamepad->map.minus<<std::endl;
	// std::cout<<"GAMEPAD_home：" <<quadrupedCtrl_->motionPlan->gamepad->map.home<<std::endl;
	// std::cout<<"GAMEPAD_lo：" <<quadrupedCtrl_->motionPlan->gamepad->map.lo<<std::endl;
	// std::cout<<"GAMEPAD_ro：" <<quadrupedCtrl_->motionPlan->gamepad->map.ro<<std::endl;

	// std::cout<<"GAMEPAD_lx：" <<quadrupedCtrl_->motionPlan->gamepad->map.lx<<std::endl;
	// std::cout<<"GAMEPAD_ly：" <<quadrupedCtrl_->motionPlan->gamepad->map.ly<<std::endl;
	// std::cout<<"GAMEPAD_rx：" <<quadrupedCtrl_->motionPlan->gamepad->map.rx<<std::endl;
	// std::cout<<"GAMEPAD_ry：" <<quadrupedCtrl_->motionPlan->gamepad->map.ry<<std::endl;

	// std::cout<<"[JOINT TORQUE] -------------------------------------------------------------------------------------------------------------------------------------------"<<std::endl;
	// std::cout<<"LF_SHOULDER_TORQUE ： "<<quadrupedCtrl_->legControl->lf_leg->legTorque.shoulder_torque<<"     "
	// 		 <<"LF_HIP_TORQUE ： "	   <<quadrupedCtrl_->legControl->lf_leg->legTorque.hip_torque     <<"     "
	// 		 <<"LF_KNEE_TORQUE ： "	   <<quadrupedCtrl_->legControl->lf_leg->legTorque.knee_torque    <<"     "<<std::endl;

	// std::cout<<"LR_SHOULDER_TORQUE ： "<<quadrupedCtrl_->legControl->lr_leg->legTorque.shoulder_torque<<"     "
	// 		 <<"LR_HIP_TORQUE ： "	   <<quadrupedCtrl_->legControl->lr_leg->legTorque.hip_torque     <<"     "
	// 		 <<"LR_KNEE_TORQUE ： "	   <<quadrupedCtrl_->legControl->lr_leg->legTorque.knee_torque    <<"     "<<std::endl;

	// std::cout<<"RF_SHOULDER_TORQUE ： "<<quadrupedCtrl_->legControl->rf_leg->legTorque.shoulder_torque<<"     "
	// 		 <<"RF_HIP_TORQUE ： "	   <<quadrupedCtrl_->legControl->rf_leg->legTorque.hip_torque     <<"     "
	// 		 <<"RF_KNEE_TORQUE： "	   <<quadrupedCtrl_->legControl->rf_leg->legTorque.knee_torque    <<"     "<<std::endl;

	// std::cout<<"RR_SHOULDER_TORQUE ： "<<quadrupedCtrl_->legControl->rr_leg->legTorque.shoulder_torque<<"     "
	// 		 <<"RR_HIP_TORQUE ： "	   <<quadrupedCtrl_->legControl->rr_leg->legTorque.hip_torque     <<"     "
	// 		 <<"RR_KNEE_TORQUE ： "	   <<quadrupedCtrl_->legControl->rr_leg->legTorque.knee_torque    <<"     "<<std::endl;

	// std::cout<<"SHOULDERTORQUE: "<<quadrupedCtrl_->legControl->lf_leg->legStste->joint_torque[0]<<" "
	// 																	   <<quadrupedCtrl_->legControl->lr_leg->legStste->joint_torque[0]<<" "
	// 																	   <<quadrupedCtrl_->legControl->rf_leg->legStste->joint_torque[0]<<" "
	// 																	   <<quadrupedCtrl_->legControl->rr_leg->legStste->joint_torque[0]<<" "<<std::endl;

	// std::cout<<"[ROBOT STATE] -------------------------------------------------------------------------------------------------------------------------------------------"<<std::endl;
	// std::cout<<"ROBOT_POS_X ： "<<quadrupedCtrl_->robotModel->bodyPosition[0]<<"     "
	// 		 <<"ROBOT_POS_Y ： "<<quadrupedCtrl_->robotModel->bodyPosition[1]<<"     "
	// 		 <<"ROBOT_POS_Z ： "<<quadrupedCtrl_->robotModel->bodyPosition[2]<<"     ";

	// std::cout<<"ROBOT_ROLL ： " <<quadrupedCtrl_->robotModel->bodyEuler[0]<<"     "
	// 		 <<"ROBOT_PITCH ： "<<quadrupedCtrl_->robotModel->bodyEuler[1]<<"     "
	// 		 <<"ROBOT_YAW ： "  <<quadrupedCtrl_->robotModel->bodyEuler[2]<<"     ";

	// std::cout<<"ROBOT_VEL_X ： "<<quadrupedCtrl_->robotModel->bodyVelocity[0]<<"     "
	// 		 <<"ROBOT_VEL_Y ： "<<quadrupedCtrl_->robotModel->bodyVelocity[1]<<"     "
	// 		 <<"ROBOT_VEL_Z ： "<<quadrupedCtrl_->robotModel->bodyVelocity[2]<<"     ";

	// std::cout<<"ROBOT_VEL_ROLL ： "<<quadrupedCtrl_->robotModel->bodyEulerAngularVel[0]<<"     "
	// 		 <<"ROBOT_VEL_PITCH ： "<<quadrupedCtrl_->robotModel->bodyEulerAngularVel[1]<<"     "
	// 		 <<"ROBOT_VEL_YAW ： "<<quadrupedCtrl_->robotModel->bodyEulerAngularVel[2]<<"     "<<std::endl;

	// outfile<<"@@@@ x: "<<quadrupedCtrl_->robotModel->bodyPosition[0]
	// 	   <<" z: "<<quadrupedCtrl_->robotModel->bodyPosition[2]
	// 	   <<" pitch: "<<quadrupedCtrl_->robotModel->bodyEuler[1]<<std::endl;
}