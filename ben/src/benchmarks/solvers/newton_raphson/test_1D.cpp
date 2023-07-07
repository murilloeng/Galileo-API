//std
#include <cmath>

//mat
#include "solvers/newton_raphson.h"

//ben
#include "benchmarks/solvers/solvers.h"

static void fun_update(void)
{
	return;
}
static void fun_restore(void)
{
	return;
}
static void fun_system(double* fi, double* Kt, const double* x)
{
	fi[0] = atan(x[0]);
	Kt[0] = 1 / (1 + x[0] * x[0]);
}

void tests::solvers::newton_raphson::test_1D(void)
{
	//solver
	mat::solvers::newton_raphson solver;

	//setup
	solver.size(1);
	solver.m_fe[0] = 1;
	solver.m_x_new[0] = 0;
	solver.m_step_max = 400;
	solver.m_system = fun_system;
	solver.m_update = fun_update;
	solver.m_restore = fun_restore;
	solver.m_strategy = mat::solvers::strategy::control_state;

	//solve
	solver.solve();
}