 #ifndef TASK_HPP
#define TASK_HPP
#include "cppTypes.h"


namespace quadruped
{
    
    class Task {
    public:
        Task(size_t dim)
            : b_set_task(false),
            dim_task(dim),
            acc_cmd(dim),
            pos_err(dim),
            vel_des(dim),
            acc_des(dim) {}  
        virtual ~Task() {}

        void GetCommand(DVec<double>& p_acc_cmd) { p_acc_cmd = acc_cmd; }
        void GetTaskJacobian(DMat<double>& p_Jt) { p_Jt = Jt; }
        void GetTaskJacobianDotQdot(DVec<double>& p_JtDotQdot) { p_JtDotQdot = JtDotQdot; }

        bool UpdateTask(const DVec<double>& p_pos_des, const DVec<double>& p_vel_des,
            const DVec<double>& p_acc_des) {
            UpdateTaskJacobian();
            UpdateTaskJDotQdot();
            UpdateCommand(p_pos_des, p_vel_des, p_acc_des);
            AdditionalUpdate();
            b_set_task = true;
            return true;
        }

        bool IsTaskSet() { return b_set_task; }
        size_t GetDim() { return dim_task; }
        void UnsetTask() { b_set_task = false; }

        const DVec<double>& GetPosError() { return pos_err; }
        const DVec<double>& GetDesVel() { return vel_des; }
        const DVec<double>& GetDesAcc() { return acc_des; }

    protected:
        // Update acc_cmd
        virtual bool UpdateCommand(const DVec<double>& p_pos_des, const DVec<double>& p_vel_des,
            const DVec<double>& p_acc_des) = 0;
        // Update Jt
        virtual bool UpdateTaskJacobian() = 0;
        // Update JtDotQdot
        virtual bool UpdateTaskJDotQdot() = 0;
        // Additional Update (defined in child classes)
        virtual bool AdditionalUpdate() = 0;

        bool b_set_task;
        size_t dim_task;

        DVec<double> acc_cmd;
        DVec<double> JtDotQdot;
        DMat<double> Jt;

        DVec<double> pos_err;
        DVec<double> vel_des;
        DVec<double> acc_des;
    };
}
#endif // !REF_HPP







