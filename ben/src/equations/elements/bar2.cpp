//std
#include <cmath>
#include <ctime>
#include <cstdio>
#include <cstdlib>
#include <cstring>

//mat
#include "mat/inc/misc/util.h"
#include "mat/inc/linear/vector.h"

//ben
#include "ben/inc/equations/elements/bar2.h"

//data
static bool m_strain = true;
static bool m_geometric = true;
static mat::vector X1(2), X2(2);
static double d[4], f[4], k[16];
static double E, v, G, A, L, U, m_f, m_k, m_e, m_de, m_he;

static void fune(double* Ux, const double* dx)
{
	memcpy(d, dx, 4 * sizeof(double));
	equations::bar2::apply();
	equations::bar2::internal_energy();
	memcpy(Ux, &U, 1 * sizeof(double));

}
static void funf(double* fx, const double* dx)
{
	memcpy(d, dx, 4 * sizeof(double));
	equations::bar2::apply();
	equations::bar2::internal_force();
	memcpy(fx, f, 4 * sizeof(double));
}
static void funs(double* kx, const double* dx)
{
	memcpy(d, dx, 4 * sizeof(double));
	equations::bar2::apply();
	equations::bar2::stiffness();
	memcpy(kx, k, 16 * sizeof(double));
}

void equations::bar2::test(void)
{
	setup();
	double dx[4];
	srand(time(nullptr));
	const bool mode = true;
	const unsigned n = 100000;
	for(unsigned i = 0; i < n; i++)
	{
		printf("%04d ", i);
		mat::vector(dx, 4).randu();
		if(mode)
		{
			if(!mat::drift(fune, funf, dx, 1, 4, 1e-2, 1e-8))
			{
				break;
			}
		}
		else
		{
			if(!mat::drift(funf, funs, dx, 4, 4, 1e-2, 1e-8))
			{
				break;
			}
		}
	}
}
void equations::bar2::setup(void)
{
	//section
	A = 1.00e-02;
	//material
	E = 2.00e+11;
	v = 3.00e-01;
	G = E / 2 / (1 + v);
	//random
	srand(time(nullptr));
	//kinematics
	X1.zeros();
	X2.zeros();
	X2(0) = 1;
	// X1.randu();
	// X2.randu();
	L = (X2 - X1).norm();
}

void equations::bar2::apply(void)
{
	//kinematics
	const mat::vector x1 = X1 + mat::vector(d + 0, 2);
	const mat::vector x2 = X2 + mat::vector(d + 2, 2);
	//strech
	const double l = (x2 - x1).norm();
	const double a = m_geometric ? l / L : 1;
	//direction
	const mat::vector s1 = (X2 - X1) / L;
	//measures
	const double eh = log(a);
	const double eg = (a * a - 1) / 2;
	const double el = s1.inner((x2 - X2) - (x1 - X1)) / L;
	//strain
	m_e = m_geometric ? m_strain ? eh : eg : el;
	m_de = m_geometric ? m_strain ? +1 / a : a : 1;
	m_he = m_geometric ? m_strain ? -1 / (a * a) : 1 : 0;
	//local
	m_f = E * A * m_e * m_de;
	m_k = E * A / L * (m_de * m_de + m_e * m_he);
}
void equations::bar2::stiffness(void)
{
	//kinematics
	const mat::vector x1 = X1 + mat::vector(d + 0, 2);
	const mat::vector x2 = X2 + mat::vector(d + 2, 2);
	//directions
	const double l = (x2 - x1).norm();
	mat::vector t1 = m_geometric ? (x2 - x1) / l : (X2 - X1) / L;
	//stiffness
	const double kg = m_geometric * m_f / l;
	k[0 + 4 * 0] = k[2 + 4 * 2] = +(m_k - kg) * t1[0] * t1[0] + kg;
	k[1 + 4 * 1] = k[3 + 4 * 3] = +(m_k - kg) * t1[1] * t1[1] + kg;
	k[2 + 4 * 0] = k[0 + 4 * 2] = -(m_k - kg) * t1[0] * t1[0] - kg;
	k[3 + 4 * 1] = k[1 + 4 * 3] = -(m_k - kg) * t1[1] * t1[1] - kg;
	k[1 + 4 * 0] = k[0 + 4 * 1] = k[3 + 4 * 2] = k[2 + 4 * 3] = +(m_k - kg) * t1[0] * t1[1];
	k[3 + 4 * 0] = k[0 + 4 * 3] = k[2 + 4 * 1] = k[1 + 4 * 2] = -(m_k - kg) * t1[0] * t1[1];
}
void equations::bar2::internal_force(void)
{
	//kinematics
	const mat::vector x1 = X1 + mat::vector(d + 0, 2);
	const mat::vector x2 = X2 + mat::vector(d + 2, 2);
	//directions
	const double l = (x2 - x1).norm();
	mat::vector t1 = m_geometric ? (x2 - x1) / l : (X2 - X1) / L;
	//internal force
	f[0] = -m_f * t1[0];
	f[1] = -m_f * t1[1];
	f[2] = +m_f * t1[0];
	f[3] = +m_f * t1[1];
}
void equations::bar2::internal_energy(void)
{
	U = E * A * L / 2 * m_e * m_e;
}