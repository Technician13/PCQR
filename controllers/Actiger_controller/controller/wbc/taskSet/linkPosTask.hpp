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
		virtual bool UpdateCommand(const DVec<double>& p_pos_des, const DVec<double>& p_vel_des, const DVec<double>& p_acc_des);
		virtual bool UpdateTaskJacobian(void);
		virtual bool UpdateTaskJDotQdot(void);
		virtual bool AdditionalUpdate(void) { return true; }

	};
}
#endif // !REF_HPP







