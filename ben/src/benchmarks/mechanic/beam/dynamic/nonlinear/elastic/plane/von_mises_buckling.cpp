//std
#include <cmath>

//fea
#include "Model/Model.h"

#include "Mesh/Mesh.h"
#include "Mesh/Nodes/Dof.h"
#include "Mesh/Cells/Types.h"
#include "Mesh/Joints/Types.h"
#include "Mesh/Sections/Ring.h"
#include "Mesh/Sections/Types.h"
#include "Mesh/Elements/Types.h"
#include "Mesh/Materials/Types.h"
#include "Mesh/Materials/Mechanic/Steel.h"
#include "Mesh/Elements/Mechanic/Mechanic.h"

#include "Boundary/Boundary.h"
#include "Boundary/Time/Types.h"
#include "Boundary/Loads/Load_Case.h"
#include "Boundary/Time/Polynomial.h"

#include "Analysis/Analysis.h"
#include "Analysis/Solvers/Types.h"
#include "Analysis/Strategies/Types.h"
#include "Analysis/Solvers/Static_Nonlinear.h"
#include "Analysis/Solvers/Dynamic_Linear.h"
#include "Analysis/Solvers/Dynamic_Nonlinear.h"

//ben
#include "benchmarks/mechanic/beam.h"

static double shape(double a)
{
	const double b = 1 - 2 * a;
	return sqrt((1 - pow(b, 2)) / (1 - pow(b, 4)));
}

void tests::beam::dynamic_nonlinear::elastic::plane::von_mises_buckling(void)
{
	/*
	von mises truss buckling
	Literature: International Journal of nonlinear mechanics - F. Bazzucchi et al. (2017) pp. 11-20
	 */

	//data
	const unsigned ne = 10;
	const unsigned ns = 1200;
	const double e = 3.33e-03;
	const double a = 6.00e-02;
	const double q = 6.00e+01;
	const double d = 2.50e+00;
	const double E = 2.10e+11;
	const double v = 3.00e-01;
	const double P = 4.00e+04;
	const double T = 2.00e+00;
	const double C = 1.00e+02;
	const double h = 6.00e-02 * d;
	const double l = sqrt(d * d + h * h);
	const double D = 4 * l / q * shape(a);

	//model
	fea::models::Model model("von mises buckling", "benchmarks/beam/dynamic/nonlinear/elastic/plane");

	//nodes
	model.mesh()->add_node(-d, 0, 0);
	model.mesh()->add_node(-0, h, 0);
	model.mesh()->add_node(+0, h, 0);
	model.mesh()->add_node(+d, 0, 0);
	for(unsigned i = 1; i < ne; i++)
	{
		const double c = +d / l;
		const double s = +h / l;
		const double x = i * l / ne;
		const double y = e * l * sin(M_PI * x / l);
		model.mesh()->add_node(c * x + s * y - d, s * x - c * y, 0);
	}
	for(unsigned i = 1; i < ne; i++)
	{
		const double c = -d / l;
		const double s = +h / l;
		const double x = i * l / ne;
		const double y = e * l * sin(M_PI * x / l);
		model.mesh()->add_node(c * x - s * y + d, s * x + c * y, 0);
	}

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::beam);

	//sections
	model.mesh()->add_section(fea::mesh::sections::type::ring);
	((fea::mesh::sections::Ring*) model.mesh()->section(0))->diameter(D);
	((fea::mesh::sections::Ring*) model.mesh()->section(0))->thickness(a * D);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->poisson_ratio(v);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->elastic_modulus(E);

	//elements
	for(unsigned i = 0; i < ne; i++)
	{
		const unsigned a = i == 0 ? 0 : i + 3;
		const unsigned b = i == ne - 1 ? 1 : i + 4;
		model.mesh()->add_element(fea::mesh::elements::type::beam2C, {a, b});
	}
	for(unsigned i = 0; i < ne; i++)
	{
		const unsigned a = i == 0 ? 3 : i + ne + 2;
		const unsigned b = i == ne - 1 ? 2 : i + ne + 3;
		model.mesh()->add_element(fea::mesh::elements::type::beam2C, {a, b});
	}

	//joints
	// model.mesh()->add_joint(fea::mesh::joints::type::pinned, {1, 2});

	//dependencies
	model.boundary()->add_dependency(1, fea::mesh::nodes::dof::translation_1, { 2 }, {fea::mesh::nodes::dof::translation_1});
	model.boundary()->add_dependency(1, fea::mesh::nodes::dof::translation_2, { 2 }, {fea::mesh::nodes::dof::translation_2});

	//times
	model.boundary()->add_time(fea::boundary::time::type::polynomial);
	((fea::boundary::time::Polynomial*) model.boundary()->time(0))->pulse(true);
	((fea::boundary::time::Polynomial*) model.boundary()->time(0))->period(T / 20);
	((fea::boundary::time::Polynomial*) model.boundary()->time(0))->terms().push_back(1);

	//supports
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(3, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(3, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(1, fea::mesh::nodes::dof::translation_2, 0, C, 0);

	//loads
	model.boundary()->add_load_set();
	model.boundary()->add_load_case();
	model.boundary()->load_case(0)->add_load_node(1, fea::mesh::nodes::dof::translation_2, -P, 0);

	//solver
	fea::mesh::elements::Mechanic::geometric(true);
	// model.analysis()->solver(fea::analysis::solvers::type::static_nonlinear);
	// dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_set(0);
	// dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->step_max(ns);
	// dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->watch_dof(1, fea::mesh::nodes::dof::translation_2);

	model.analysis()->solver(fea::analysis::solvers::type::dynamic_linear);
	dynamic_cast<fea::analysis::solvers::Dynamic_Linear*>(model.analysis()->solver())->load_set(0);
	dynamic_cast<fea::analysis::solvers::Dynamic_Linear*>(model.analysis()->solver())->time_max(T);
	dynamic_cast<fea::analysis::solvers::Dynamic_Linear*>(model.analysis()->solver())->step_max(ns);
	dynamic_cast<fea::analysis::solvers::Dynamic_Linear*>(model.analysis()->solver())->watch_dof(1, fea::mesh::nodes::dof::translation_2);

	// model.analysis()->solver(fea::analysis::solvers::type::dynamic_nonlinear);
	// dynamic_cast<fea::analysis::solvers::Dynamic_Nonlinear*>(model.analysis()->solver())->load_set(0);
	// dynamic_cast<fea::analysis::solvers::Dynamic_Nonlinear*>(model.analysis()->solver())->time_max(T);
	// dynamic_cast<fea::analysis::solvers::Dynamic_Nonlinear*>(model.analysis()->solver())->step_max(ns);
	// dynamic_cast<fea::analysis::solvers::Dynamic_Nonlinear*>(model.analysis()->solver())->watch_dof(1, fea::mesh::nodes::dof::translation_2);
	// dynamic_cast<fea::analysis::solvers::Dynamic_Nonlinear*>(model.analysis()->solver())->convergence(unsigned(fea::analysis::solvers::convergence::fixed));

	//solve
	model.analysis()->solve();

	//save
	model.save();
}