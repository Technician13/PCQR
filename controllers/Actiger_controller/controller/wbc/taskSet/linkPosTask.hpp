#ifndef LINKPOSTASK_HPP
#define LINKPOSTASK_HPP

#include "task.hpp"
#include "robotModel.hpp"

enum linkNum_t {lfLink=0, lrLink, rfLink, rrLink};

namespace quadruped
{
	
	class LinkPosTask : public Task
	{
	private:

	public:
		Vec3<double> kp_kin, kp, kd;
		int linkId;

		explicit LinkPosTask(RobotModel* p_robotModel,const int p_linkId);
		virtual ~LinkPosTask();

	protected:
		RobotModel* robotModel;
		// Update acc_cmd
		virtual bool UpdateCommand(const DVec<double>& p_pos_des, const DVec<double>& p_vel_des, const DVec<double>& p_acc_des);
		// Update Jt
		virtual bool UpdateTaskJacobian(void);
		// Update JtDotQdot
		virtual bool UpdateTaskJDotQdot(void);
		// Additional Update (defined in child classes)
		virtual bool AdditionalUpdate(void) { return true; }

	};
}
#endif // !REF_HPP







