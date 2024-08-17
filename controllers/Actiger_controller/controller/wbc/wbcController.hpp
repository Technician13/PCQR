#ifndef WBC_CONTROLLER_H
#define WBC_CONTROLLER_H

#include <vector>

#include "parameters.hpp"
#include "taskSet/bodyPosTask.hpp"
#include "taskSet/bodyOriTask.hpp"
#include "taskSet/linkPosTask.hpp"
#include "contact/singleContact.hpp"
#include "WBIC/kinWBC.hpp"
#include "WBIC/dynWBC.hpp"
#include "mpc.hpp"
#include "robotModel.hpp"
#include "motionPlan.hpp"

namespace quadruped
{
    
    class WBC_Ctrl {
    public:
        int cnt = 20;
        BodyPosTask* body_pos_task;
        BodyOriTask* body_ori_task;

        LinkPosTask* link_task[4];
        ContactSpec* foot_contact[4];

        RobotModel* robotModel;
        MotionPlan* ref;
        LegControl* legControl; 
        KinWBC* kin_wbc;
        DynWBC* dyn_wbc;
        Parameters* para;
        
        std::vector<ContactSpec* > contact_list;
        std::vector<Task* > task_list;
        
        DVec<double> full_config;
        DVec<double> torque_ff;
        DVec<double> des_jpos;
        DVec<double> des_jvel;
             
        explicit WBC_Ctrl(RobotModel* p_robotModel, MotionPlan* p_ref, Parameters* p_para, LegControl* p_legControl);
        virtual ~WBC_Ctrl(void);

        void Run(Vec12<double> & p_mpc_force);
        void ParameterSetup(void);
        void UpdateLegCMD(void);
        void CleanUp(void);
        void UpdatTask(void);
        void ComputeWBC();
        
    };
}
#endif
