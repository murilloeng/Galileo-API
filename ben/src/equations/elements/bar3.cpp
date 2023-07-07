//std
#include <cmath>
#include <ctime>
#include <cstdio>
#include <cstdlib>
#include <cstring>

//mat
#include "misc/util.h"
#include "linear/vector.h"

//ben
#include "equations/elements/bar3.h"

//data
static bool m_strain = false;
static bool m_geometric = true;
static mat::vector X1(3), X2(3);
static double d[6], f[6], k[36];
static double E, v, G, A, L, U, m_f, m_k, m_e, m_de, m_he;

static void fune(double* Ux, const double* dx)
{
	memcpy(d, dx, 6 * sizeof(double));
	equations::bar3::apply();
	equations::bar3::internal_energy();
	memcpy(Ux, &U, 1 * sizeof(double));

}
static void funf(double* fx, const double* dx)
{
	memcpy(d, dx, 6 * sizeof(double));
	equations::bar3::apply();
	equations::bar3::internal_force();
	memcpy(fx, f, 6 * sizeof(double));
}
static void funs(double* kx, const double* dx)
{
	memcpy(d, dx, 6 * sizeof(double));
	equations::bar3::apply();
	equations::bar3::stiffness();
	memcpy(kx, k, 36 * sizeof(double));
}

void equations::bar3::test(void)
{
	setup();
	double dx[6];
	srand(time(nullptr));
	const bool mode = false;
	const unsigned n = 100000;
	for(unsigned i = 0; i < n; i++)
	{
		printf("%05d ", i);
		mat::vector(dx, 6).randu();
		if(mode)
		{
			if(!mat::drift(fune, funf, dx, 1, 6, 1e-2, 1e-8))
			{
				break;
			}
		}
		else
		{
			if(!mat::drift(funf, funs, dx, 6, 6, 1e-2, 1e-8))
			{
				break;
			}
		}
	}
}
void equations::bar3::setup(void)
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
	X1.randu();
	X2.randu();
	L = (X2 - X1).norm();
}

void equations::bar3::apply(void)
{
	//kinematics
	const mat::vector x1 = X1 + mat::vector(d + 0, 3);
	const mat::vector x2 = X2 + mat::vector(d + 3, 3);
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
void equations::bar3::stiffness(void)
{
	//kinematics
	const mat::vector x1 = X1 + mat::vector(d + 0, 3);
	const mat::vector x2 = X2 + mat::vector(d + 3, 3);
	//directions
	const double l = (x2 - x1).norm();
	mat::vector t1 = m_geometric ? (x2 - x1) / l : (X2 - X1) / L;
	//stiffness
	const double kg = m_geometric * m_f / l;
	k[0 + 6 * 0] = k[3 + 6 * 3] = +(m_k - kg) * t1[0] * t1[0] + kg;
	k[1 + 6 * 1] = k[4 + 6 * 4] = +(m_k - kg) * t1[1] * t1[1] + kg;
	k[2 + 6 * 2] = k[5 + 6 * 5] = +(m_k - kg) * t1[2] * t1[2] + kg;
	k[3 + 6 * 0] = k[0 + 6 * 3] = -(m_k - kg) * t1[0] * t1[0] - kg;
	k[4 + 6 * 1] = k[1 + 6 * 4] = -(m_k - kg) * t1[1] * t1[1] - kg;
	k[5 + 6 * 2] = k[2 + 6 * 5] = -(m_k - kg) * t1[2] * t1[2] - kg;
	k[1 + 6 * 0] = k[0 + 6 * 1] = k[4 + 6 * 3] = k[3 + 6 * 4] = +(m_k - kg) * t1[0] * t1[1];
	k[2 + 6 * 0] = k[0 + 6 * 2] = k[5 + 6 * 3] = k[3 + 6 * 5] = +(m_k - kg) * t1[0] * t1[2];
	k[2 + 6 * 1] = k[1 + 6 * 2] = k[5 + 6 * 4] = k[4 + 6 * 5] = +(m_k - kg) * t1[1] * t1[2];
	k[4 + 6 * 0] = k[0 + 6 * 4] = k[3 + 6 * 1] = k[1 + 6 * 3] = -(m_k - kg) * t1[0] * t1[1];
	k[5 + 6 * 0] = k[0 + 6 * 5] = k[3 + 6 * 2] = k[2 + 6 * 3] = -(m_k - kg) * t1[0] * t1[2];
	k[5 + 6 * 1] = k[1 + 6 * 5] = k[4 + 6 * 2] = k[2 + 6 * 4] = -(m_k - kg) * t1[1] * t1[2];
}
void equations::bar3::internal_force(void)
{
	//kinematics
	const mat::vector x1 = X1 + mat::vector(d + 0, 3);
	const mat::vector x2 = X2 + mat::vector(d + 3, 3);
	//directions
	const double l = (x2 - x1).norm();
	mat::vector t1 = m_geometric ? (x2 - x1) / l : (X2 - X1) / L;
	//internal force
	f[0] = -m_f * t1[0];
	f[1] = -m_f * t1[1];
	f[2] = -m_f * t1[2];
	f[3] = +m_f * t1[0];
	f[4] = +m_f * t1[1];
	f[5] = +m_f * t1[2];
}
void equations::bar3::internal_energy(void)
{
	U = E * A * L / 2 * m_e * m_e;
}