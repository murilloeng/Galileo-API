//std
#include <cmath>

//fea
#include "fea/inc/Model/Model.h"

#include "fea/inc/Mesh/Mesh.h"
#include "fea/inc/Mesh/Nodes/Dof.h"
#include "fea/inc/Mesh/Cells/Types.h"
#include "fea/inc/Mesh/Cells/Line/Line.h"
#include "fea/inc/Mesh/Sections/Ring.h"
#include "fea/inc/Mesh/Sections/Types.h"
#include "fea/inc/Mesh/Elements/Types.h"
#include "fea/inc/Mesh/Materials/Types.h"
#include "fea/inc/Mesh/Materials/Mechanic/Steel.h"
#include "fea/inc/Mesh/Elements/Mechanic/Mechanic.h"
#include "fea/inc/Mesh/Elements/Mechanic/Frame/Bar.h"

#include "fea/inc/Boundary/Boundary.h"
#include "fea/inc/Boundary/Loads/Load_Case.h"

#include "fea/inc/Analysis/Analysis.h"
#include "fea/inc/Analysis/Solvers/Types.h"
#include "fea/inc/Analysis/Strategies/Types.h"
#include "fea/inc/Analysis/Solvers/Static_Nonlinear.h"

//ben
#include "ben/inc/benchmarks/mechanic/bar.h"

void tests::bar::static_nonlinear::tent_roof(void)
{
	//model
	fea::models::Model model("tent roof", "benchmarks/bar/static/nonlinear");

	//parameters
	const bool depl = true;
	const bool grav = true;
	const bool plas = false;
	const bool symm = true;

	const double g = 9.81e+0;
	const double l = 2.00e+1;
	const double h = 1.90e+0;
	const double H = 1.00e+0;
	const double d = 3.56e-1;
	const double t = 2.50e-2;
	const double S = 4.00e+8;
	const double r = 7.85e+3;
	const double E = 2.00e+11;

	const double wl = 6.00e+0;
	const double pl = 1.00e+4;
	const double pr = 1.00e+3;
	const double dp = 1.00e-2;

	const unsigned nd = 2;
	const unsigned nx = 10;
	const unsigned ny = 10;
	const unsigned nm = 10;
	const unsigned ns = 800;
	const unsigned nn = (nx + 1) * (ny + 1);
	const unsigned nw = (nx + 1) * ny / 2 + nx / 2;

	const double q = 4 * h / l;
	const double A = M_PI * (pow(d, 2) - pow(d - 2 * t, 2)) / 4;
	const double s = h == 0 ? l : l / 2 * (sqrt(1 + q * q) + asinh(q) / q);

	//points
	const double ds = s / nx;
	const double dy = l / ny;
	double x[nx + 1], z[nx + 1];
	for(unsigned i = 0; i <= nx; i++)
	{
		if(i == 0)
		{
			z[i] = 0;
			x[i] = -l / 2;
		}
		else
		{
			double r, dr;
			x[i] = x[i - 1] + ds;
			z[i] = h * (1 - pow(2 * x[i] / l, 2));
			while(true)
			{
				r = pow(x[i] - x[i - 1], 2) + pow(z[i] - z[i - 1], 2) - ds * ds;
				if(fabs(r) < 1e-10 * s)
				{
					break;
				}
				dr = 2 * (x[i] - x[i - 1]) - 16 * x[i] * h / (l * l) * (z[i] - z[i - 1]);
				x[i] += -r / dr;
				z[i] = h * (1 - pow(2 * x[i] / l, 2));
			}
		}
	}

	//nodes
	for(unsigned i = 0; i <= ny; i++)
	{
		for(unsigned j = 0; j <= nx; j++)
		{
			model.mesh()->add_node(x[j], i * dy, z[j]);
		}
	}
	for(unsigned i = 0; i < ny; i++)
	{
		for(unsigned j = 0; j < nx; j++)
		{
			const double dx = x[j + 1] - x[j];
			const double dz = z[j + 1] - z[j];
			const double ym = dy * i + dy / 2;
			const double xm = (x[j + 1] + x[j]) / 2;
			const double zm = (z[j + 1] + z[j]) / 2;
			model.mesh()->add_node(xm - H * dz / ds, ym, zm + H * dx / ds);
		}
	}

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::bar);

	//sections
	model.mesh()->add_section(fea::mesh::sections::type::ring);
	((fea::mesh::sections::Ring*) model.mesh()->section(0))->diameter(d);
	((fea::mesh::sections::Ring*) model.mesh()->section(0))->thickness(t);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->yield_stress(S);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->specific_mass(r);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->elastic_modulus(E);

	//elements
	fea::mesh::elements::Bar::strain(true);
	fea::mesh::elements::Mechanic::geometric(true);
	fea::mesh::elements::Mechanic::inelastic(plas);
	for(unsigned i = 0; i < ny; i++)
	{
		for(unsigned j = 0; j < nx; j++)
		{
			model.mesh()->add_element(fea::mesh::elements::type::bar3, {(nx + 1) * (i + 0) + j + 0, nn + nx * i + j});
			model.mesh()->add_element(fea::mesh::elements::type::bar3, {(nx + 1) * (i + 0) + j + 1, nn + nx * i + j});
			model.mesh()->add_element(fea::mesh::elements::type::bar3, {(nx + 1) * (i + 1) + j + 1, nn + nx * i + j});
			model.mesh()->add_element(fea::mesh::elements::type::bar3, {(nx + 1) * (i + 1) + j + 0, nn + nx * i + j});
			model.mesh()->add_element(fea::mesh::elements::type::bar3, {(nx + 1) * (i + 0) + j + 0, (nx + 1) * (i + 0) + j + 1});
			model.mesh()->add_element(fea::mesh::elements::type::bar3, {(nx + 1) * (i + 0) + j + 0, (nx + 1) * (i + 1) + j + 0});
			if(j + 1 < nx)
			{
				model.mesh()->add_element(fea::mesh::elements::type::bar3, {nn + nx * (i + 0) + j + 0, nn + nx * (i + 0) + j + 1});
			}
			else
			{
				model.mesh()->add_element(fea::mesh::elements::type::bar3, {(nx + 1) * (i + 0) + j + 1, (nx + 1) * (i + 1) + j + 1});
			}
			if(i + 1 < ny)
			{
				model.mesh()->add_element(fea::mesh::elements::type::bar3, {nn + nx * (i + 0) + j + 0, nn + nx * (i + 1) + j + 0});
			}
			else
			{
				model.mesh()->add_element(fea::mesh::elements::type::bar3, {(nx + 1) * (i + 1) + j + 0, (nx + 1) * (i + 1) + j + 1});
			}
		}
	}

	//supports
	for(unsigned i = 0; i <= ny; i++)
	{
		model.boundary()->add_support((nx + 1) * i, fea::mesh::nodes::dof::translation_1);
		model.boundary()->add_support((nx + 1) * i, fea::mesh::nodes::dof::translation_2);
		model.boundary()->add_support((nx + 1) * i, fea::mesh::nodes::dof::translation_3);
		model.boundary()->add_support((nx + 1) * i + nx, fea::mesh::nodes::dof::translation_1);
		model.boundary()->add_support((nx + 1) * i + nx, fea::mesh::nodes::dof::translation_2);
		model.boundary()->add_support((nx + 1) * i + nx, fea::mesh::nodes::dof::translation_3);
		model.boundary()->add_support((nx + 1) * i + nx / 2, fea::mesh::nodes::dof::translation_1);
		model.boundary()->add_support((nx + 1) * i + nx / 2, fea::mesh::nodes::dof::translation_2);
	}

	//self weight
	model.boundary()->add_self_weight("Gravity", fea::mesh::nodes::dof::translation_3);

	//overload
	model.boundary()->add_load_case("Overload");
	for(unsigned i = 0; i + 1 < ny; i++)
	{
		for(unsigned j = 0; j + 1 < nx; j++)
		{
			const double q = 0.20;
			const double f = symm || j + 1 == nx / 2 ? 1 : j + 1 < nx / 2 ? 1 - q : 1 + q;
			model.boundary()->load_case(1)->add_load_node(nn + nx * (i + 0) + j + 0, fea::mesh::nodes::dof::translation_3, -f * pr * dy * ds / 4);
			model.boundary()->load_case(1)->add_load_node(nn + nx * (i + 0) + j + 1, fea::mesh::nodes::dof::translation_3, -f * pr * dy * ds / 4);
			model.boundary()->load_case(1)->add_load_node(nn + nx * (i + 1) + j + 1, fea::mesh::nodes::dof::translation_3, -f * pr * dy * ds / 4);
			model.boundary()->load_case(1)->add_load_node(nn + nx * (i + 1) + j + 0, fea::mesh::nodes::dof::translation_3, -f * pr * dy * ds / 4);
		}
	}

	//solver
	model.analysis()->solver(fea::analysis::solvers::type::static_nonlinear);
	model.analysis()->solver()->watch_dof(nw, fea::mesh::nodes::dof::translation_3);

	//solve self weight
	if(grav)
	{
		dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->step_max(100);
		dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_max(1.00);
		dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_guess(0.05);
		dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->strategy(fea::analysis::strategies::type::control_load);
		model.analysis()->solve();
	}

	//solve deployment
	if(depl)
	{
		dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->step_max(ns);
		dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->dof_min(-wl);
		dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_max(pl);
//		dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->tolerance(1e-4);
		dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_adjust(true);
		dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->iteration_max(nm);
		dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_guess(dp);
		dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->iteration_desired(nd);
//		dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->strategy(fea::analysis::strategies::type::minimal_norm);
		dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->strategy(fea::analysis::strategies::type::control_state);
//		dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->strategy(fea::analysis::strategies::type::arc_length_cylindric);
		model.analysis()->solve(grav);
	}

	//save
	model.save();
}