#include "legPlan.hpp"

using namespace std;
using namespace quadruped;

LegPlan::LegPlan()
{
	horizon = 251;
	fr_des.setZero();

	pFoot_des = new Queue(horizon);
	vFoot_des = new Queue(horizon);
	aFoot_des = new Queue(horizon);
}


LegPlan::~LegPlan()
{
	delete pFoot_des;
	delete vFoot_des;
	delete aFoot_des;
}