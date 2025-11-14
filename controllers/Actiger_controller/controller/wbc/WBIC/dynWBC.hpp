#ifndef DYNWBC_HPP
#define DYNWBC_HPP

#include <osqp.h>
#include "robotModel.hpp"
#include <contact/contactSpec.hpp>
#include <taskSet/task.hpp>
#include "qpSolver.hpp"
#include <vector>
namespace quadruped
{
	
	class DynWBC
	{
	public:
		DMat<double> w_floating;
		DMat<double> w_rf;
		const std::vector<ContactSpec*>* contact_list;
		const std::vector<Task*>* task_list;

		explicit DynWBC(size_t p_num_qdot, const std::vector<ContactSpec*>* p_contact_list,
						const std::vector<Task*>* p_task_list);
		virtual ~DynWBC();

		DVec<double> MakeTorque(void);
		void UpdateSetting(const DMat<double>& p_A, const DMat<double>& p_Ainv,
			const DVec<double>& p_cori, const DVec<double>& p_grav);


	private:
		size_t dim_opt;      

		size_t dim_rf;  
		size_t dim_cstrMtx;

		size_t dim_floating = 6;

		DMat<double> qp_H;
		DVec<double> qp_G;
		DMat<double> qp_A;
		DVec<double> qp_U;
		DVec<double> qp_L;

		bool b_updatesetting;

		DMat<double> A;
		DMat<double> Ainv;
		DVec<double> cori;
		DVec<double> grav;

		size_t num_act_joint;
		size_t num_qdot;

		DMat<double> Sv; 
		
		Mat18<double> eye;
		DMat<double> eye_floating;

		DMat<double> S_delta;
		DMat<double> cstrMtx;
		DVec<double> cstr_ieq_vec_l;
		DVec<double> cstr_ieq_vec_u;

		DMat<double> Jc;
		DVec<double> JcDotQdot;
		DVec<double> Fr_des;
	
		DVec<double> opt_result;
		DVec<double> Fr;

		RobotModel* robotModel;
		QpSolver* qpSolver;
		void CalculateQPData(const DVec<double>& p_qddot);
		void ContactBuilding();

		DVec<double> GetSolution(const DVec<double>& p_qddot, DVec<double> p_add);
		void SetOptimizationSize();

	};
}
#endif // !REF_HPP







