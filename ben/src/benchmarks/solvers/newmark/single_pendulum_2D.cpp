//std
#include <cmath>

//mat
#include "solvers/newmark.h"

//ben
#include "benchmarks/solvers/solvers.h"

//data
const static int n = 5;
const static double g = 9.81;
const static double m = 1.00;
const static double l = 1.00;

static void external(double* f, double t)
{
	f[0] = 0;
}
static void internal(double* f, const double* x, const double* v)
{
	f[0] = g / l * sin(x[0]);
}

static void inertia(double* M, const double* x)
{
	M[0] = 1;
}
static void damping(double* C, const double* x, const double* v)
{
	C[0] = 0;
}
static void stifness(double* K, const double* x, const double* v)
{
	K[0] = g / l * cos(x[0]);
}

void tests::solvers::newmark::single_pendulum_2D(void)
{
	//solver
	mat::solvers::newmark solver(1, true);

	//setup
	solver.m_ns = 1000;
	solver.m_T = 20 * sqrt(g / l);

	//initials
	solver.m_x[0] = 0;
	solver.m_v[0] = 2 * sqrt(g / l);

	//system
	solver.m_inertia = inertia;
	solver.m_damping = damping;
	solver.m_internal = internal;
	solver.m_external = external;
	solver.m_stiffness = stifness;

	//solve
	solver.solve();
}