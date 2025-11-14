#ifndef LEGPLAN_HPP
#define LEGPLAN_HPP

#include <vector>
#include "queue.hpp"
#include "robotModel.hpp"
#include "cppTypes.h"

namespace quadruped 
{
	class LegPlan
	{
	private:

	protected:

	public:
		explicit LegPlan(void);
		virtual ~LegPlan(void);

		uint32_t horizon;
		Vec3<double> fr_des;
		Queue* pFoot_des;
		Queue* vFoot_des;
		Queue* aFoot_des;
	};
}

#endif // !LEGPLAN_HPP