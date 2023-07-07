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
const static double v = 0.00;
const static double q = M_PI / 6;
const static double f = g / l * pow(sin(q), 2) / cos(q);
const static double h = f * pow(sin(q), 2);

static void external(double* f, double t)
{
	f[0] = 0;
}
static void internal(double* f, const double* x, const double* v)
{
	f[0] = g / l * sin(x[0]) - h * cos(x[0]) / pow(sin(x[0]), 3);
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
	K[0] = g / l * cos(x[0]) + h * (1 + 2 * pow(cos(x[0]), 2)) / pow(sin(x[0]), 4);
}

void tests::solvers::newmark::single_pendulum_3D(void)
{
	//solver
	mat::solvers::newmark solver(1, true);

	//setup
	solver.m_ns = 1000;
	solver.m_T = 20 * sqrt(g / l);

	//initials
	solver.m_x[0] = q;
	solver.m_v[0] = v;

	//system
	solver.m_inertia = inertia;
	solver.m_damping = damping;
	solver.m_internal = internal;
	solver.m_external = external;
	solver.m_stiffness = stifness;

	//solve
	solver.solve();
}