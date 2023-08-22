//std
#include <cmath>
#include <cstring>

//mat
#include "mat/inc/linear/dense.h"
#include "mat/inc/solvers/newmark.h"

//ben
#include "ben/inc/benchmarks/solvers/solvers.h"

//data (sun x earth)
//const static double G = 6.674e-11;
//const static double R = 1.496e+11;
//const static double m1 = 5.972e+24;
//const static double m2 = 1.989e+30;
//const static double Tn = 365.25 * 24 * 60 * 60;

//data (sun x earth)
const static double R = 3.844e+8;
const static double G = 6.674e-11;
const static double m1 = 7.342e+22;
const static double m2 = 5.972e+24;
const static double Tn = 27.45 * 24 * 60 * 60;

static void external(double* f, double t)
{
	f[0] = 0;
	f[1] = 0;
	f[2] = 0;
}
static void internal(double* f, const double* x, const double* v)
{
	const double r = mat::norm(x, 3);
	f[0] = G * m1 * m2 / (r * r * r) * x[0];
	f[1] = G * m1 * m2 / (r * r * r) * x[1];
	f[2] = G * m1 * m2 / (r * r * r) * x[2];
}

static void inertia(double* M, const double* x)
{
	M[1] = M[2] = M[3] = 0;
	M[5] = M[6] = M[7] = 0;
	M[0] = M[4] = M[8] = m1;
}
static void damping(double* C, const double* x, const double* v)
{
	memset(C, 0, 9 * sizeof(double));
}
static void stifness(double* K, const double* x, const double* v)
{
	double n[3];
	mat::normalize(n, x, 3);
	const double r = mat::norm(x, 3);
	for(unsigned i = 0; i < 3; i++)
	{
		for(unsigned j = 0; j < 3; j++)
		{
			K[i + 3 * j] = G * m1 * m2 / (r * r * r) * ((i == j) - 3 * n[i] * n[j]);
		}
	}
}

void tests::solvers::newmark::gravity(void)
{
	//solver
	mat::solvers::newmark solver(3, true);

	//setup
	solver.m_T = Tn;
	solver.m_ns = 1000;

	//initials
	solver.m_x[0] = R;
	solver.m_x[1] = 0;
	solver.m_x[2] = 0;
	solver.m_v[0] = 0;
	solver.m_v[2] = 0;
	solver.m_v[1] = 2 * M_PI * R / Tn;

	//system
	solver.m_inertia = inertia;
	solver.m_damping = damping;
	solver.m_internal = internal;
	solver.m_external = external;
	solver.m_stiffness = stifness;

	//solve
	solver.solve();
}