#include "foothold.hpp"
#include <iostream>
using namespace quadruped;

Foothold::Foothold(Parameters* p_para,Terrain* p_terrain_now)
{
	updataInfo = new FH_updataInfo(p_para,p_terrain_now);
	fH_nlp = new FH_nlp(updataInfo); 

	app = IpoptApplicationFactory();
	app->RethrowNonIpoptException(true);

	app->Options()->SetIntegerValue("print_level", 3);
	app->Options()->SetIntegerValue("max_iter", 200);
	app->Options()->SetNumericValue("tol", 1e-3);
	app->Options()->SetNumericValue("compl_inf_tol", 1e-4);
	app->Options()->SetNumericValue("constr_viol_tol", 1e-3);
	app->Options()->SetStringValue("mu_strategy", "adaptive");
	app->Options()->SetStringValue("output_file", "ipopt.out");

	status = app->Initialize();
	if (status != Solve_Succeeded)
	{
		std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
	}
}

Foothold::~Foothold(void){}


void Foothold::GetFootHold(void)
{
	status = app->OptimizeTNLP(fH_nlp);
	if (status != Solve_Succeeded)
	{
		std::cout << std::endl << std::endl << "*** The problem FAILED!" << std::endl <<std::endl;
	}
}

