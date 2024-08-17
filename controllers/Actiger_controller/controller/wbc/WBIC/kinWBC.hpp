#ifndef KINWBC_HPP
#define KINWBC_HPP

#include <vector>
#include "robotModel.hpp"
#include "contact/contactSpec.hpp"
#include "taskSet/task.hpp"

namespace quadruped
{
	
	class KinWBC
	{

	public:
		explicit KinWBC(size_t num_qdot_);
		virtual ~KinWBC();	

		bool FindConfiguration(const DVec<double>& p_curr_config,
			const std::vector<Task*>& p_task_list,
			const std::vector<ContactSpec*>& p_contact_list,
			DVec<double>* p_jpos_cmd, DVec<double>* p_jvel_cmd);

	private:

		size_t num_qdot;
		size_t num_act_joint;
		DMat<double> I_mtx;

		void BuildProjectionMatrix(const DMat<double>& p_J, DMat<double>& p_N);
	
	};
}
#endif // !REF_HPP







