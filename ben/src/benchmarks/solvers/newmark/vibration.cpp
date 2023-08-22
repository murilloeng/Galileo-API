//std
#include <cmath>

//mat
#include "mat/inc/solvers/newmark.h"

//ben
#include "ben/inc/benchmarks/solvers/solvers.h"

//data
const static int n = 5;
const static double g = 9.81;
const static double m = 1.00;
const static double c = 0.00;
const static double k = 1.00;
const static double p = 1.00;
const static double w = 2.00;
const static double wn = sqrt(k / m);
const static double Tn = 2 * M_PI / wn;

static void external(double* f, double t)
{
	f[0] = p * sin(w * t);
}
static void internal(double* f, const double* x, const double* v)
{
	f[0] = k * x[0] + c * v[0];
}

static void inertia(double* M, const double* x)
{
	M[0] = m;
}
static void damping(double* C, const double* x, const double* v)
{
	C[0] = c;
}
static void stifness(double* K, const double* x, const double* v)
{
	K[0] = k;
}

void tests::solvers::newmark::vibration(void)
{
	//solver
	mat::solvers::newmark solver(1, true);

	//setup
	solver.m_ns = 1000;
	solver.m_T = n * Tn;

	//initials
	solver.m_x[0] = 0;
	solver.m_v[0] = 0;

	//system
	solver.m_inertia = inertia;
	solver.m_damping = damping;
	solver.m_internal = internal;
	solver.m_external = external;
	solver.m_stiffness = stifness;

	//solve
	solver.solve();
}