#include "cppTypes.h"
#include "locomotionController.hpp"
#include <fstream>
#include <string>
#include <fstream>

#ifdef ROBOT
    #include "embedded.hpp"
#else
	#include "webotsApi.hpp"
#endif

using namespace quadruped;


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


	pthread_t motionPlan_pid;	
	int motionPlanIsok = pthread_create(&motionPlan_pid, NULL, MotionPlanRun,(void*)(quadrupedCtrl->motionPlan));
	if(motionPlanIsok == 0)
		pthread_detach(motionPlan_pid);
	else
		std::cout<<"pthread_create err!!!!!!"<<std::endl;


	while(1)	
	{		
		if(GetTimeFlag())
		{			
			#ifdef ROBOT
				Embedded();
			#else
				WebotsApi();
			#endif			
			
        	quadrupedCtrl->RobotControl();
			SendDebugInfo(quadrupedCtrl);	
		}
	}
	OutFile.close();
	delete quadrupedCtrl;
	return (0);
}

void SendDebugInfo(LocomotionCtrl* quadrupedCtrl_)
{
	
}