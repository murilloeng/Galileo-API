//std
#include <cmath>
#include <ctime>
#include <cstdio>
#include <cstring>

//mat
#include "mat/inc/misc/util.h"
#include "mat/inc/linear/vec3.h"
#include "mat/inc/linear/quat.h"
#include "mat/inc/linear/mat3.h"
#include "mat/inc/linear/dense.h"
#include "mat/inc/linear/vector.h"
#include "mat/inc/linear/matrix.h"

//ben
#include "ben/inc/equations/elements/beam3TL.h"

//data
static bool ho = true;
static mat::vec3 X1, X2;
static mat::vec3 s1, s2, s3;
static mat::quat q0, h1, h2;
static double L, E, G, v, A, J;
static double I22, I33, I222, I223, I233, I333;
static double I2222, I2223, I2233, I2333, I3333;

void mydev(double* A, const double* es)
{
	memset(A, 0, 54 * sizeof(double));
	A[0 + 9 * 1] = ho * es[1];
	A[0 + 9 * 2] = ho * es[2];
	A[4 + 9 * 3] = -ho * es[1];
	A[5 + 9 * 3] = -ho * es[2];
	A[7 + 9 * 3] = A[8 + 9 * 3] = +ho * es[3];
	A[4 + 9 * 1] = A[5 + 9 * 2] = -ho * es[3];
	A[1 + 9 * 1] = A[2 + 9 * 2] = A[3 + 9 * 3] = +1;
	A[4 + 9 * 0] = A[7 + 9 * 4] = A[6 + 9 * 5] = ho * es[4];
	A[5 + 9 * 0] = A[6 + 9 * 4] = A[8 + 9 * 5] = ho * es[5];
	A[0 + 9 * 0] = A[4 + 9 * 4] = A[5 + 9 * 5] = 1 + ho * es[0];
}

namespace equations
{
	namespace beam3TL
	{
		void test(void)
		{
			double x[12];
			srand(time(nullptr));
			const unsigned n = 1000;
			for(unsigned i = 0; i < n; i++)
			{
				setup();
				mat::randu(x, 12);
				printf("%03d ", i);
				if(!mat::drift(section_strains_h, mydev, x, 9, 6, 1e-3, 0))
				{
					break;
				}
			}
		}
		void setup(void)
		{
			//nodes
			X1.randu();
			X2.randu();
			L = (X2 - X1).norm();
			//triad
			s1 = (X2 - X1) / L;
			s1.triad(s2, s3);
			//quaternions
			h1.randu();
			h2.randu();
			q0 = mat::quat(s1, s2, s3);
			//material
			E = 2.00e+11;
			v = 3.00e-01;
			G = E / 2 / (1 + v);
			//section
			A = 6.00e-03;
			J = 3.40e-06;
			I22 = 1.80e-06;
			I33 = 5.00e-06;
		}

		void energy(double* U, const double* d)
		{
			// mat::vector es(6);
			// strains(es.mem(), d);
			// U[0] = L / 2 * es.inner(ks * es);
		}
		void stiffness(double* K, const double* d)
		{

		}
		void internal_force(double* f, const double* d)
		{
			// double B[72], H[144], e[6];
			// strains(e, d);
			// global_rotation(H, d);
			// strains_gradient(B, d);
			// mat::vector(f, 12) = L * mat::matrix(H, 12, 12).transpose() * mat::matrix(B, 6, 12).transpose() * ks * mat::vector(e, 6);
		}

		void section_strains_s(double* es, double* d)
		{
			//kinematics
			const mat::vec3 t1 = d + 3;
			const mat::vec3 t2 = d + 9;
			const mat::quat q1 = t1.quaternion() * h1;
			const mat::quat q2 = t2.quaternion() * h2;
			const mat::vec3 x1 = X1 + mat::vec3(d + 0);
			const mat::vec3 x2 = X2 + mat::vec3(d + 6);
			//strains
			const mat::vec3 xs = q1.conjugate(x2 - x1) / L;
			const mat::vec3 ws = q1.conjugate(q2).pseudo() / L;
			const mat::vec3 gs = (L * ws).rotation_gradient_inverse(xs) - s1;
			//components
			es[0] = gs.inner(s1);
			es[1] = gs.inner(s2);
			es[2] = gs.inner(s3);
			es[3] = ws.inner(s1);
			es[4] = ws.inner(s2);
			es[5] = ws.inner(s3);
		}
		void section_strains_h(double* eh, const double* es)
		{
			eh[1] = es[1];
			eh[2] = es[2];
			eh[3] = es[3];
			eh[6] = ho * es[4] * es[5];
			eh[7] = ho * (es[3] * es[3] + es[4] * es[4]) / 2;
			eh[8] = ho * (es[3] * es[3] + es[5] * es[5]) / 2;
			eh[4] = (1 + ho * es[0]) * es[4] - ho * es[1] * es[3];
			eh[5] = (1 + ho * es[0]) * es[5] - ho * es[2] * es[3];
			eh[0] = es[0] + ho * (es[0] * es[0] + es[1] * es[1] + es[2] * es[2]) / 2;
		}

		void global_rotation(double* H, const double* d)
		{
			const mat::vec3 t1(d + 3);
			const mat::vec3 t2(d + 9);
			mat::matrix(H, 12, 12).eye();
			mat::matrix(H, 12, 12).span(3, 3) = t1.rotation_gradient();
			mat::matrix(H, 12, 12).span(9, 9) = t2.rotation_gradient();
		}
	}
}