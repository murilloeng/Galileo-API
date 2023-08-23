//std
#include <cmath>

//mat
#include "mat/inc/linear/dense.h"
#include "mat/inc/linear/linear.h"

//fea
#include "fea/inc/Model/Model.h"

#include "fea/inc/Mesh/Mesh.h"
#include "fea/inc/Mesh/Nodes/Dof.h"
#include "fea/inc/Mesh/Cells/Types.h"
#include "fea/inc/Mesh/Joints/Types.h"
#include "fea/inc/Mesh/Joints/Joint.h"
#include "fea/inc/Mesh/Sections/Box.h"
#include "fea/inc/Mesh/Sections/Round.h"
#include "fea/inc/Mesh/Sections/Types.h"
#include "fea/inc/Mesh/Elements/Types.h"
#include "fea/inc/Mesh/Cells/Line/Line.h"
#include "fea/inc/Mesh/Materials/Types.h"
#include "fea/inc/Mesh/Materials/Mechanic/Steel.h"
#include "fea/inc/Mesh/Elements/Mechanic/Mechanic.h"

#include "fea/inc/Boundary/Boundary.h"
#include "fea/inc/Boundary/Loads/Load_Case.h"

#include "fea/inc/Analysis/Analysis.h"
#include "fea/inc/Analysis/Solvers/Types.h"
#include "fea/inc/Analysis/Strategies/Types.h"
#include "fea/inc/Analysis/Solvers/Static_Nonlinear.h"

//ben
#include "ben/inc/benchmarks/mechanic/deployable.h"

//controls
const static bool base = false;		//has cable
const static bool unif = true;		//uniform load
const static bool asym = true;		//asymmetric load
const static bool depl = false;		//solve deployment
const static bool grav = false;		//solve self weight

//geometry
const static double S = 2.00e+00;	//span
const static double H = 3.00e-01;	//height
const static double t = 1.00e-01;	//thickness

//cross section
const static double w = 5.00e-02;	//width
const static double h = 5.00e-02;	//height
const static double a = 5.00e-03;	//thickness
const static double d = 1.00e-01;	//cable diameter

//material
const static double r = 7.85e+03;	//specific mass
const static double E = 2.00e+11;	//elastic modulus

//load
const static double y = 2.00e-01;	//asymmetric load

//analysis
const static double tl = 1.00e-03;	//tolerance
const static double Pl = 2.50e+06;	//load limit
const static double Pr = 1.00e+00;	//load reference value
const static double wr = 1.00e+00;	//uniform load reference
const static double dP = 1.00e+01;	//load initial increment

const static unsigned ns = 800;		//max steps
const static unsigned nm = 10;		//max iterations
const static unsigned nu = 10;		//number of units
const static unsigned nr = 1;		//units refinement
const static unsigned nd = 7;		//iterations desired

//data
static double ps[nu + 1], pr[nu], ds[nu], pm[nu * nu];

//parameters
const static double f = S / H / 2;
const static double R = H / 2 * (1 + f * f);
const static double q = 2 * asin(2 * f / (1 + f * f));

const static double qi = (M_PI - q) / 2;
const static double qj = (M_PI + q) / 2;

const static unsigned nw = nu % 2 == 0 ? 3 * nu + 3 : 3 * nu - 3;

//curve
static double xc(double t)
{
	return R * cos(t);
}
static double yc(double t)
{
	return R * sin(t);
}
static double dx(double t)
{
	return -R * sin(t);
}
static double dy(double t)
{
	return +R * cos(t);
}

//points
static bool points(void)
{
	//predictor
	double a = t / 2;
	double b = sqrt(t * t + a * a);
	for(unsigned i = 0; i <= nu; i++)
	{
		ps[i] = (qj - qi) * i / nu + qi;
	}
	//corrector
	for(unsigned i = 0; i < 20; i++)
	{
		//check
		for(unsigned j = 0; j < nu; j++)
		{
			const double xi = xc(ps[j + 0]);
			const double xj = xc(ps[j + 1]);
			const double yi = yc(ps[j + 0]);
			const double yj = yc(ps[j + 1]);
			pr[j] = pow(xj - xi, 2) / (a * a) + pow(yj - yi, 2) / (b * b) - 1;
		}
		const double e = mat::norm(pr, nu);
		printf("iteration: %02d error: %+.2e\n", i, e);
		if(e < 1e-5)
		{
			printf("points converged!\n");
			return true;
		}
		//update
		mat::clean(pm, nu * nu);
		for(unsigned j = 0; j < nu; j++)
		{
			const double xi = xc(ps[j + 0]);
			const double xj = xc(ps[j + 1]);
			const double yi = yc(ps[j + 0]);
			const double yj = yc(ps[j + 1]);
			if(j != 0)
			{
				pm[j + nu * (j - 1)] = -2 * (xj - xi) * dx(ps[j + 0]) / (a * a) - 2 * (yj - yi) * dy(ps[j + 0]) / (b * b);
			}
			if(j != nu - 1)
			{
				pm[j + nu * (j + 0)] = +2 * (xj - xi) * dx(ps[j + 1]) / (a * a) + 2 * (yj - yi) * dy(ps[j + 1]) / (b * b);
			}
			pm[j + nu * (nu - 1)] = -2 * pow(xj - xi, 2) / (a * a * a) - 2 * pow(yj - yi, 2) / (b * b * b) * (a / b);
		}
		mat::solve(ds, pm, pr, nu);
		for(unsigned j = 0; j < nu - 1; j++)
		{
			ps[j + 1] -= ds[j];
		}
		a -= ds[nu - 1];
		b = sqrt(a * a + t * t);
	}
	//return
	return false;
}

//model
void tests::deployable::static_nonlinear::arch_trans(void)
{
	//model
	fea::models::Model model("arch trans", "benchmarks/deployable/static/nonlinear");

	//points
	if(!points())
	{
		printf("\tError: Could not calculate points!\n");
		return;
	}

	//nodes
	for(unsigned i = 0; i < nu; i++)
	{
		const double xi = xc(ps[i + 0]);
		const double xj = xc(ps[i + 1]);
		const double yi = yc(ps[i + 0]);
		const double yj = yc(ps[i + 1]);
		const double xm = (xi + xj) / 2;
		const double ym = (yi + yj) / 2;
		model.mesh()->add_node(xm, ym, 0);
		model.mesh()->add_node(xm, ym, 0);
		model.mesh()->add_node(xi, yi - t / 2, 0);
		model.mesh()->add_node(xi, yi + t / 2, 0);
		model.mesh()->add_node(xj, yj - t / 2, 0);
		model.mesh()->add_node(xj, yj + t / 2, 0);
	}

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::bar);
	model.mesh()->add_cell(fea::mesh::cells::type::beam);
	((fea::mesh::cells::Line*) model.mesh()->cell(0))->section(1);
	((fea::mesh::cells::Line*) model.mesh()->cell(1))->section(0);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->specific_mass(r);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->elastic_modulus(E);

	//sections
	model.mesh()->add_section(fea::mesh::sections::type::box);
	model.mesh()->add_section(fea::mesh::sections::type::round);
	((fea::mesh::sections::Box*) model.mesh()->section(0))->width(w);
	((fea::mesh::sections::Box*) model.mesh()->section(0))->height(h);
	((fea::mesh::sections::Box*) model.mesh()->section(0))->thickness(a);
	((fea::mesh::sections::Round*) model.mesh()->section(1))->diameter(d);

	//elements
	for(unsigned i = 0; i < nu; i++)
	{
		model.mesh()->add_element(fea::mesh::elements::type::beam2C, {6 * i + 0, 6 * i + 2}, 0, 1);
		model.mesh()->add_element(fea::mesh::elements::type::beam2C, {6 * i + 0, 6 * i + 5}, 0, 1);
		model.mesh()->add_element(fea::mesh::elements::type::beam2C, {6 * i + 1, 6 * i + 3}, 0, 1);
		model.mesh()->add_element(fea::mesh::elements::type::beam2C, {6 * i + 1, 6 * i + 4}, 0, 1);
	}
	if(base)
	{
		model.mesh()->add_element(fea::mesh::elements::type::bar2, {6 * 0 + 2, 6 * (nu - 1) + 4}, 0, 0);
	}

	//sizes
	for(unsigned i = 0; i < nu; i++)
	{

		const double l0 = model.mesh()->cell(1)->edge(model.mesh()->element(4 * i + 0), 0);
		const double l1 = model.mesh()->cell(1)->edge(model.mesh()->element(4 * i + 1), 0);
		const double l2 = model.mesh()->cell(1)->edge(model.mesh()->element(4 * i + 2), 0);
		const double l3 = model.mesh()->cell(1)->edge(model.mesh()->element(4 * i + 3), 0);
		printf("unit: %02d sizes: (%.2f, %.2f)\n", i, l0 + l1, l2 + l3);
	}

	//joints
	for(unsigned i = 0; i < nu; i++)
	{
		model.mesh()->add_joint(fea::mesh::joints::type::pinned, {6 * i + 0, 6 * i + 1});
	}
	for(unsigned i = 0; i + 1 < nu; i++)
	{
		model.mesh()->add_joint(fea::mesh::joints::type::pinned, {6 * i + 4, 6 * (i + 1) + 2});
		model.mesh()->add_joint(fea::mesh::joints::type::pinned, {6 * i + 5, 6 * (i + 1) + 3});
	}

	//supports
	model.boundary()->add_support(6 * 0 + 2, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(6 * 0 + 2, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(6 * 0 + 2, fea::mesh::nodes::dof::translation_3);
	model.boundary()->add_support(6 * (nu - 1) + 4, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(6 * (nu - 1) + 4, fea::mesh::nodes::dof::translation_3);
	if(!base)
	{
		model.boundary()->add_support(6 * (nu - 1) + 4, fea::mesh::nodes::dof::translation_1);
	}

	//load cases
	model.boundary()->add_load_case("Deployment");
	model.boundary()->add_self_weight("Gravity", fea::mesh::nodes::dof::translation_2);
	if(unif)
	{
		for(unsigned i = 0; i < nu; i++)
		{
			const double xi = xc(ps[i + 0]);
			const double xj = xc(ps[i + 1]);
			const double lf = asym ? i < nu / 2 ? (1 - y) : (1 + y) : 1;
			model.boundary()->load_case(0)->add_load_node(6 * i + 3, fea::mesh::nodes::dof::translation_2, -lf * wr * fabs(xj - xi) / 2);
			model.boundary()->load_case(0)->add_load_node(6 * i + 5, fea::mesh::nodes::dof::translation_2, -lf * wr * fabs(xj - xi) / 2);
		}
	}
	else
	{
		model.boundary()->load_case(0)->add_load_node(nw, fea::mesh::nodes::dof::translation_2, -Pr);
	}

	//solver
	fea::mesh::elements::Mechanic::geometric(true);
	model.analysis()->solver(fea::analysis::solvers::type::static_nonlinear);
	model.analysis()->solver()->watch_dof(nw, fea::mesh::nodes::dof::translation_2);

	//solve gravity
	if(grav)
	{
		dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->step_max(100);
		dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_max(1e0);
		dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_guess(5e-2);
		dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->strategy(fea::analysis::strategies::type::control_load);
		model.analysis()->solve();
	}

	//solve deployment
	if(depl)
	{
		dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->step_max(ns);
		dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_max(Pl);
		dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->tolerance(tl);
		dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->dof_min(-2 * H);
		dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->iteration_max(nm);
		dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_adjust(true);
		dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_guess(dP);
		dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->iteration_desired(nd);
		dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->strategy(fea::analysis::strategies::type::minimal_norm);
		model.analysis()->solve(grav);
	}

	//save
	model.save();
}