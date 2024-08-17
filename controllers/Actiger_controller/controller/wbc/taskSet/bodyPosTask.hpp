#ifndef BODYPOSTASK_HPP
#define BODYPOSTASK_HPP

#include "robotModel.hpp"
#include "task.hpp"


namespace quadruped
{
	
	class BodyPosTask : public Task
	{
	private:

	public:
		Vec3<double> kp_kin;
		Vec3<double> kp, kd;

		explicit BodyPosTask(RobotModel* p_robotModel);
		virtual ~BodyPosTask(void);

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








