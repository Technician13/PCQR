
#ifndef STATEMACHINE_HPP
#define STATEMACHINE_HPP

#include <vector>
// #include "robotModel.hpp"
#include "cppTypes.h"

namespace quadruped {


	
	class StateMachine
	{

	public:
		uint32_t period = 200;
		uint32_t stanceTime = 0;
		explicit StateMachine(void);
		virtual ~StateMachine(void);
	
	};

}

#endif // !STATEMACHINE_HPP
