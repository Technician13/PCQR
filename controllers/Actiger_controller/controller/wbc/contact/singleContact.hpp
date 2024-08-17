#ifndef SINGLE_CONTACT_HPP
#define SINGLE_CONTACT_HPP

#include "taskSet/task.hpp"
#include "robotModel.hpp"
#include "contactSpec.hpp"


namespace quadruped
{
	
	class SingleContact : public ContactSpec
	{
	public:
		RobotModel* robotModel;
		int contact_id;
		int dim_cstr;


		explicit SingleContact(RobotModel* p_robotModel, const int p_linkId);
		virtual ~SingleContact(void);


		virtual bool UpdateJc();
		virtual bool UpdateJcDotQdot();
		virtual bool UpdateUf();
		virtual bool UpdateInequalityVector();

		

	protected:

	};


}


#endif
