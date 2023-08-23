//std
#include <cmath>
#include <cstring>

//mat
#include "mat/inc/linear/vector.h"

//fea
#include "fea/inc/Mesh/Mesh.h"
#include "fea/inc/Model/Model.h"
#include "fea/inc/Mesh/Sections/Types.h"
#include "fea/inc/Mesh/Sections/Section.h"
#include "fea/inc/Mesh/Sections/Profile.h"
#include "fea/inc/Mesh/Sections/Rectangle.h"

//ben
#include "ben/inc/equations/elements/beam3.h"

//static
const static double E = 2.00e+11;
const static double v = 3.00e-01;
const static double L = 1.20e+00;
const static double w = 2.00e-01;
const static double h = 6.00e-01;
const static double wf = 6.00e-01;
const static double hw = 6.00e-01;
const static double tf = 6.00e-02;
const static double tw = 6.00e-02;
const static double G = E / 2 / (1 + v);

static void setup(fea::models::Model* model, fea::mesh::sections::type type)
{
	model->mesh()->add_section(type);
	if(type == fea::mesh::sections::type::rectangle)
	{
		((fea::mesh::sections::Rectangle*) model->mesh()->section(0))->width(w);
		((fea::mesh::sections::Rectangle*) model->mesh()->section(0))->height(h);
		((fea::mesh::sections::Rectangle*) model->mesh()->section(0))->size(fmin(w, h) / 10);
	}
	else
	{
		((fea::mesh::sections::Profile*) model->mesh()->section(0))->web_height(hw);
		((fea::mesh::sections::Profile*) model->mesh()->section(0))->web_thickness(tw);
		((fea::mesh::sections::Profile*) model->mesh()->section(0))->flange_top_width(wf);
		((fea::mesh::sections::Profile*) model->mesh()->section(0))->flange_bottom_width(wf);
		((fea::mesh::sections::Profile*) model->mesh()->section(0))->flange_top_thickness(tf);
		((fea::mesh::sections::Profile*) model->mesh()->section(0))->flange_bottom_thickness(tf);
		((fea::mesh::sections::Profile*) model->mesh()->section(0))->size(fmin(tw, tf) / 3);
	}
	model->mesh()->section(0)->prepare();
}

namespace equations
{
	namespace beam3
	{
		void section_case_1(void)
		{
			//data
			fea::models::Model model;
			setup(&model, fea::mesh::sections::type::profile_Z);
			mat::matrix H(2, 2), I(2, 2), Kb(2, 2), Ks(2, 2), Db(2, 8), Ds(2, 8), Eb(2, 8), Es(2, 8), Dc(8, 8), Di(8, 8);

			//misc
			I.eye();
			H(0, 1) = -1;
			H(1, 0) = +1;
			H(0, 0) = H(1, 1) = 0;

			//bending
			Kb(0, 1) = Kb(1, 0) = 0;
			Kb(0, 0) = E * model.mesh()->section(0)->inertia(1, 1);
			Kb(1, 1) = E * model.mesh()->section(0)->inertia(0, 0);

			//shear
			Ks(0, 0) = G * model.mesh()->section(0)->area(0, 0);
			Ks(1, 1) = G * model.mesh()->section(0)->area(1, 1);
			Ks(0, 1) = Ks(1, 0) = G * model.mesh()->section(0)->area(0, 1);

			//system
			Dc.zeros();
			Dc.span(0, 0, 2, 2) = I;
			Dc.span(2, 4, 2, 2) = I;
			Dc.span(6, 4, 2, 2) = I;
			Dc.span(4, 2, 2, 2) = L * I;
			Dc.span(6, 2, 2, 2) = -L * L  / 2 * H;
			Dc.span(4, 6, 2, 2) = -L * L  / 2 * Kb.inverse() * H * Ks;
			Dc.span(4, 0, 2, 2) = I - L * L  / 2 * Kb.inverse() * H * Ks * H;
			Dc.span(6, 0, 2, 2) = L * L * L  / 6 * H * Kb.inverse() * H * Ks * H;
			Dc.span(6, 6, 2, 2) = L * (I + L * L / 6 * H * Kb.inverse() * H * Ks);

			//open
			FILE* fdb = fopen("db.dat", "w");
			FILE* fds = fopen("ds.dat", "w");
			FILE* feb = fopen("eb.dat", "w");
			FILE* fes = fopen("es.dat", "w");

			//write
			Di = Dc.inverse();
			const unsigned n = 200;
			for(unsigned i = 0; i <= n; i++)
			{
				//position
				const double x = i * L / n;
				//interpolation es
				Es.span(0, 0, 2, 2) = H;
				Es.span(0, 2, 2, 2) = 0;
				Es.span(0, 4, 2, 2) = 0;
				Es.span(0, 6, 2, 2) = I;
				//interpolation eb
				Eb.span(0, 2, 2, 2) = I;
				Eb.span(0, 4, 2, 2) = 0;
				Eb.span(0, 6, 2, 2) = -x * Kb.inverse() * H * Ks;
				Eb.span(0, 0, 2, 2) = -x * Kb.inverse() * H * Ks * H;
				//interpolation db
				Db.span(0, 4, 2, 2) = 0;
				Db.span(0, 2, 2, 2) = x * I;
				Db.span(0, 6, 2, 2) = -x * x / 2 * Kb.inverse() * H * Ks;
				Db.span(0, 0, 2, 2) = I - x * x / 2 * Kb.inverse() * H * Ks * H;
				//interpolation ds
				Ds.span(0, 4, 2, 2) = I;
				Ds.span(0, 2, 2, 2) = -x * x / 2 * H;
				Ds.span(0, 0, 2, 2) = x * x * x / 6 * H * Kb.inverse() * H * Ks * H;
				Ds.span(0, 6, 2, 2) = x * I + x * x * x / 6 * H * Kb.inverse() * H * Ks;
				//print
				const mat::matrix Dbn = Db * Di;
				const mat::matrix Dsn = Ds * Di;
				const mat::matrix Ebn = Eb * Di;
				const mat::matrix Esn = Es * Di;
				for(unsigned j = 0; j < 8; j++)
				{
					for(unsigned k = 0; k < 2; k++)
					{
						fprintf(fdb, "%+.6e ", Dbn(k, j));
						fprintf(fds, "%+.6e ", Dsn(k, j));
						fprintf(feb, "%+.6e ", Ebn(k, j));
						fprintf(fes, "%+.6e ", Esn(k, j));
					}
				}
				fprintf(fdb, "\n");
				fprintf(fds, "\n");
				fprintf(feb, "\n");
				fprintf(fes, "\n");
			}

			//close
			fclose(fdb);
			fclose(fds);
			fclose(feb);
			fclose(fes);

		}
		void section_case_2(void)
		{
			//data
			mat::vector v(3);
			fea::models::Model model;
			setup(&model, fea::mesh::sections::type::profile_Z);
			mat::matrix H(3, 2), P(3, 3), D(3, 3), C(3, 3), S(3, 3);
			mat::matrix I2(2, 2), I3(3, 3), Kb(2, 2), Ks(3, 3), Kw(3, 3);
			mat::matrix Db(2, 16), Ds(3, 16), Eb(2, 16), Es(3, 16), Dc(16, 16), Di(16, 16);

			//misc
			I2.eye();
			I3.eye();
			H.zeros();
			H(1, 1) = -1;
			H(2, 0) = +1;

			//bending
			Kb(0, 1) = Kb(1, 0) = 0;
			Kb(0, 0) = E * model.mesh()->section(0)->inertia(1, 1);
			Kb(1, 1) = E * model.mesh()->section(0)->inertia(0, 0);

			//shear
			Ks.zeros();
			Ks(0, 0) = G * model.mesh()->section(0)->inertia();
			Ks(1, 1) = G * model.mesh()->section(0)->area(0, 0);
			Ks(2, 2) = G * model.mesh()->section(0)->area(1, 1);
			Ks(1, 2) = Ks(2, 1) = G * model.mesh()->section(0)->area(0, 1);

			//warping
			Kw(0, 0) = E * model.mesh()->section(0)->inertia_warping(0, 0);
			Kw(1, 1) = E * model.mesh()->section(0)->inertia_warping(1, 1);
			Kw(2, 2) = E * model.mesh()->section(0)->inertia_warping(2, 2);
			Kw(0, 1) = Kw(1, 0) = E * model.mesh()->section(0)->inertia_warping(0, 1);
			Kw(0, 2) = Kw(2, 0) = E * model.mesh()->section(0)->inertia_warping(0, 2);
			Kw(1, 2) = Kw(2, 1) = E * model.mesh()->section(0)->inertia_warping(1, 2);

			//eigen
			D.zeros();
			S.zeros();
			C.zeros();
			Ks.eigen_sym(v, P, Kw);
			for(unsigned i = 0; i < 3; i++)
			{
				D(i, i) = sqrt(v(i));
				C(i, i) = cosh(L * D(i, i));
				S(i, i) = sinh(L * D(i, i));
			}

			//system
			Dc.zeros();
			Dc.span( 2, 13, 3, 3) = P;
			Dc.span( 5,  0, 3, 2) = H;
			Dc.span(13,  0, 3, 2) = H;
			Dc.span( 0,  0, 2, 2) = I2;
			Dc.span( 2,  4, 3, 3) = I3;
			Dc.span( 5,  7, 3, 3) = I3;
			Dc.span(10,  4, 3, 3) = I3;
			Dc.span(13,  7, 3, 3) = I3;
			Dc.span( 5, 10, 3, 3) = P * D;
			Dc.span(10, 10, 3, 3) = P * S;
			Dc.span(10, 13, 3, 3) = P * C;
			Dc.span( 8,  2, 2, 2) = L * I2;
			Dc.span(13, 10, 3, 3) = P * D * C;
			Dc.span(13, 13, 3, 3) = P * D * S;
			Dc.span(10,  2, 3, 2) = -L * L / 2 * H;
			Dc.span( 8,  7, 2, 3) = L * L / 2 * Kb.inverse() * H.transpose() * Ks;
			Dc.span( 8,  0, 2, 2) = I2 + L * L / 2 * Kb.inverse() * H.transpose() * Ks * H;
			Dc.span(10,  0, 3, 2) = -L * L * L / 6 * H * Kb.inverse() * H.transpose() * Ks * H;
			Dc.span(10,  7, 3, 3) = L * I3 - L * L * L / 6 * H * Kb.inverse() * H.transpose() * Ks;

			//open
			FILE* fdb = fopen("db.dat", "w");
			FILE* fds = fopen("ds.dat", "w");
			FILE* feb = fopen("eb.dat", "w");
			FILE* fes = fopen("es.dat", "w");

			//write
			Di = Dc.inverse();
			const unsigned n = 200;
			for(unsigned i = 0; i <= n; i++)
			{
				//position
				const double x = i * L / n;
				for(unsigned j = 0; j < 3; j++)
				{
					C(j, j) = cosh(x * D(j, j));
					S(j, j) = sinh(x * D(j, j));
				}
				//interpolation es
				Es.span(0,  0, 3, 2) = H;
				Es.span(0,  2, 3, 2) = 0;
				Es.span(0,  4, 3, 3) = 0;
				Es.span(0,  7, 3, 3) = I3;
				Es.span(0, 10, 3, 3) = P * D * C;
				Es.span(0, 13, 3, 3) = P * D * S;
				//interpolation eb
				Eb.span(0,  4, 2, 3) = 0;
				Eb.span(0, 10, 2, 3) = 0;
				Eb.span(0, 13, 2, 3) = 0;
				Eb.span(0,  2, 2, 2) = I2;
				Eb.span(0,  7, 2, 3) = x * Kb.inverse() * H.transpose() * Ks;
				Eb.span(0,  0, 2, 2) = x * Kb.inverse() * H.transpose() * Ks * H;
				//interpolation db
				Db.span(0,  4, 2, 3) = 0;
				Db.span(0, 10, 2, 3) = 0;
				Db.span(0, 13, 2, 3) = 0;
				Db.span(0,  2, 2, 2) = x * I2;
				Db.span(0,  7, 2, 3) = x * x / 2 * Kb.inverse() * H.transpose() * Ks;
				Db.span(0,  0, 2, 2) = I2 + x * x / 2 * Kb.inverse() * H.transpose() * Ks * H;
				//interpolation ds
				Ds.span(0,  4, 3, 3) = I3;
				Ds.span(0, 10, 3, 3) = P * S;
				Ds.span(0, 13, 3, 3) = P * C;
				Ds.span(0,  2, 3, 2) = -x * x / 2 * H;
				Ds.span(0,  0, 3, 2) = -x * x * x / 6 * H * Kb.inverse() * H.transpose() * Ks * H;
				Ds.span(0,  7, 3, 3) = x * I3 - x * x * x / 6 * H * Kb.inverse() * H.transpose() * Ks;
				//print
				const mat::matrix Dbn = Db * Di;
				const mat::matrix Dsn = Ds * Di;
				const mat::matrix Ebn = Eb * Di;
				const mat::matrix Esn = Es * Di;
				for(unsigned j = 0; j < 16; j++)
				{
					for(unsigned k = 0; k < 2; k++)
					{
						fprintf(fdb, "%+.6e ", Dbn(k, j));
						fprintf(feb, "%+.6e ", Ebn(k, j));
					}
					for(unsigned k = 0; k < 3; k++)
					{
						fprintf(fds, "%+.6e ", Dsn(k, j));
						fprintf(fes, "%+.6e ", Esn(k, j));
					}
				}
				fprintf(fdb, "\n");
				fprintf(fds, "\n");
				fprintf(feb, "\n");
				fprintf(fes, "\n");
			}

			//close
			fclose(fdb);
			fclose(fds);
			fclose(feb);
			fclose(fes);
		}
		void section_case_3(void)
		{
			//data
			fea::models::Model model;
			setup(&model, fea::mesh::sections::type::profile_Z);

			const double A = model.mesh()->section(0)->area();
			const double J = model.mesh()->section(0)->inertia();
			const double A22 = model.mesh()->section(0)->area(0, 0);
			const double A23 = model.mesh()->section(0)->area(0, 1);
			const double A33 = model.mesh()->section(0)->area(1, 1);
			const double I22 = model.mesh()->section(0)->inertia(0, 0);
			const double I33 = model.mesh()->section(0)->inertia(1, 1);
			const double c2 = model.mesh()->section(0)->shear_center(0);
			const double c3 = model.mesh()->section(0)->shear_center(1);
			const double W11 = model.mesh()->section(0)->inertia_warping(0, 0);
			const double W22 = model.mesh()->section(0)->inertia_warping(1, 1);
			const double W33 = model.mesh()->section(0)->inertia_warping(2, 2);
			const double W12 = model.mesh()->section(0)->inertia_warping(0, 1);
			const double W13 = model.mesh()->section(0)->inertia_warping(0, 2);
			const double W23 = model.mesh()->section(0)->inertia_warping(1, 2);

			mat::vector v(3);
			mat::matrix H(3, 2), P(3, 3), D(3, 3), C(3, 3), S(3, 3);
			mat::matrix I2(2, 2), I3(3, 3), Kb(2, 2), K0(3, 3), Ks(3, 3), Kw(3, 3);
			mat::matrix Db(2, 16), Ds(3, 16), Du(3, 16), Eb(2, 16), Es(3, 16), Dc(16, 16), Di(16, 16);

			//misc
			I2.eye();
			I3.eye();
			H.zeros();
			H(1, 1) = -1;
			H(2, 0) = +1;

			//bending
			Kb(0, 0) = E * I33;
			Kb(1, 1) = E * I22;
			Kb(0, 1) = Kb(1, 0) = 0;

			//shear
			Ks.zeros();
			K0.zeros();
			Ks(0, 0) = G * J;
			Ks(1, 1) = G * A22;
			Ks(2, 2) = G * A33;
			K0(1, 1) = K0(2, 2) = G * A;
			Ks(1, 2) = Ks(2, 1) = G * A23;
			K0(0, 1) = K0(1, 0) = +G * c3 * A;
			K0(0, 2) = K0(2, 0) = -G * c2 * A;
			K0(0, 0) = G * (I22 + I33 + (c2 * c2 + c3 * c3) * A);

			//warping
			Kw(0, 0) = E * W11;
			Kw(1, 1) = E * W22;
			Kw(2, 2) = E * W33;
			Kw(0, 1) = Kw(1, 0) = E * W12;
			Kw(0, 2) = Kw(2, 0) = E * W13;
			Kw(1, 2) = Kw(2, 1) = E * W23;

			//eigen
			D.zeros();
			S.zeros();
			C.zeros();
			(Ks - Ks * K0.inverse() * Ks).eigen_sym(v, P, Kw);
			for(unsigned i = 0; i < 3; i++)
			{
				D(i, i) = sqrt(v(i));
				C(i, i) = cosh(L * D(i, i));
				S(i, i) = sinh(L * D(i, i));
			}

			//system
			Dc.zeros();
			Dc.span( 0,  0, 2, 2) = I2;
			Dc.span( 8,  0, 2, 2) = I2;
			Dc.span( 2, 13, 3, 3) = I3;
			Dc.span( 5,  4, 3, 3) = I3;
			Dc.span(10, 13, 3, 3) = I3;
			Dc.span(13,  4, 3, 3) = I3;
			Dc.span( 5, 10, 3, 3) = P * D;
			Dc.span(10,  0, 3, 2) = -L * H;
			Dc.span( 8,  2, 2, 2) = L * I2;
			Dc.span(13,  7, 3, 3) = P * D * S;
			Dc.span(13, 10, 3, 3) = P * D * C;
			Dc.span(10,  2, 3, 2) = -L * L / 2 * H;
			Dc.span( 2,  7, 3, 3) = (I3 - K0.inverse() * Ks) * P;
			Dc.span(10,  7, 3, 3) = (I3 - K0.inverse() * Ks) * P * C;
			Dc.span(10, 10, 3, 3) = (I3 - K0.inverse() * Ks) * P * S;
			Dc.span( 8,  4, 2, 3) = L * L / 2 * Kb.inverse() * H.transpose() * Ks;
			Dc.span(10,  4, 3, 3) = L * I3 - L * L * L / 6 * H * Kb.inverse() * H.transpose() * Ks;

			//open
			FILE* fdb = fopen("db.dat", "w");
			FILE* fds = fopen("ds.dat", "w");
			FILE* fdu = fopen("du.dat", "w");
			FILE* feb = fopen("eb.dat", "w");
			FILE* fes = fopen("es.dat", "w");

			//write
			Di = Dc.inverse();
			const unsigned n = 200;
			for(unsigned i = 0; i <= n; i++)
			{
				//position
				const double x = i * L / n;
				for(unsigned j = 0; j < 3; j++)
				{
					C(j, j) = cosh(x * D(j, j));
					S(j, j) = sinh(x * D(j, j));
				}
				//interpolation es
				Es.span(0,  0, 3, 2) = 0;
				Es.span(0,  2, 3, 2) = 0;
				Es.span(0, 13, 3, 3) = 0;
				Es.span(0,  4, 3, 3) = I3;
				Es.span(0,  7, 3, 3) = (I3 - K0.inverse() * Ks) * P * D * S;
				Es.span(0, 10, 3, 3) = (I3 - K0.inverse() * Ks) * P * D * C;
				//interpolation eb
				Eb.span(0,  0, 2, 2) = 0;
				Eb.span(0,  7, 2, 3) = 0;
				Eb.span(0, 10, 2, 3) = 0;
				Eb.span(0, 13, 2, 3) = 0;
				Eb.span(0,  2, 2, 2) = I2;
				Eb.span(0,  4, 2, 3) = x * Kb.inverse() * H.transpose() * Ks;
				//interpolation db
				Db.span(0,  7, 2, 3) = 0;
				Db.span(0, 10, 2, 3) = 0;
				Db.span(0, 13, 2, 3) = 0;
				Db.span(0,  0, 2, 2) = I2;
				Db.span(0,  2, 2, 2) = x * I2;
				Db.span(0,  4, 2, 3) = x * x / 2 * Kb.inverse() * H.transpose() * Ks;
				//interpolation ds
				Ds.span(0, 13, 3, 3) = I3;
				Ds.span(0,  0, 3, 2) = -x * H;
				Ds.span(0,  2, 3, 2) = -x * x / 2 * H;
				Ds.span(0,  7, 3, 3) = (I3 - K0.inverse() * Ks) * P * C;
				Ds.span(0, 10, 3, 3) = (I3 - K0.inverse() * Ks) * P * S;
				Ds.span(0,  4, 3, 3) = x * I3 - x * x * x / 6 * H * Kb.inverse() * H.transpose() * Ks;
				//interpolation du
				Du.span(0,  0, 3, 2) = 0;
				Du.span(0,  2, 3, 2) = 0;
				Du.span(0, 13, 3, 3) = 0;
				Du.span(0,  4, 3, 3) = I3;
				Du.span(0,  7, 3, 3) = P * D * S;
				Du.span(0, 10, 3, 3) = P * D * C;
				//print
				const mat::matrix Dbn = Db * Di;
				const mat::matrix Dsn = Ds * Di;
				const mat::matrix Dun = Du * Di;
				const mat::matrix Ebn = Eb * Di;
				const mat::matrix Esn = Es * Di;
				for(unsigned j = 0; j < 16; j++)
				{
					for(unsigned k = 0; k < 2; k++)
					{
						fprintf(fdb, "%+.6e ", Dbn(k, j));
						fprintf(feb, "%+.6e ", Ebn(k, j));
					}
					for(unsigned k = 0; k < 3; k++)
					{
						fprintf(fds, "%+.6e ", Dsn(k, j));
						fprintf(fdu, "%+.6e ", Dun(k, j));
						fprintf(fes, "%+.6e ", Esn(k, j));
					}
				}
				fprintf(fdb, "\n");
				fprintf(fds, "\n");
				fprintf(fdu, "\n");
				fprintf(feb, "\n");
				fprintf(fes, "\n");
			}

			//close
			fclose(fdb);
			fclose(fds);
			fclose(fdu);
			fclose(feb);
			fclose(fes);
		}
	}
}