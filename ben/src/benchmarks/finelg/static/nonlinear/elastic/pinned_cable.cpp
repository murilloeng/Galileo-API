//std
#include <cmath>

//fea
#include "fea/inc/Model/Model.h"

#include "fea/inc/Mesh/Mesh.h"
#include "fea/inc/Mesh/Nodes/Dof.h"
#include "fea/inc/Mesh/Cells/Types.h"
#include "fea/inc/Mesh/Joints/Types.h"
#include "fea/inc/Mesh/Sections/Types.h"
#include "fea/inc/Mesh/Elements/Types.h"
#include "fea/inc/Mesh/Cells/Line/Line.h"
#include "fea/inc/Mesh/Materials/Types.h"
#include "fea/inc/Mesh/Sections/Generic.h"
#include "fea/inc/Mesh/Materials/Mechanic/Steel.h"
#include "fea/inc/Mesh/Elements/Mechanic/Mechanic.h"
#include "fea/inc/Mesh/Elements/Mechanic/Frame/Beam.h"

#include "fea/inc/Boundary/Boundary.h"
#include "fea/inc/Boundary/Loads/Load_Case.h"

#include "fea/inc/Analysis/Analysis.h"
#include "fea/inc/Analysis/Solvers/Types.h"
#include "fea/inc/Analysis/Strategies/Types.h"
#include "fea/inc/Analysis/Solvers/Static_Nonlinear.h"

//ben
#include "ben/inc/benchmarks/finelg/finelg.h"

void tests::finelg::static_nonlinear::elastic::pinned_cable(void)
{
	/*
	Pinned cable
	Literature: Phd Thesis V. V. Goyet (1989) pp. 8.19
	*/

	//mesh
	const unsigned n = 8;

	//data
	const double L = 2.00e+01;
	const double A = 7.50e-06;
	const double I = 3.00e-07;
	const double v = 3.00e-01;
	const double E = 2.00e+11;
	const double F = 7.64e+02;
	const double P = 5.00e+03;
	const double p = 1.00e+03;

	//model
	fea::models::Model model("pinned cable", "benchmarks/finelg/static/nonlinear/elastic");

	//nodes
	for(unsigned i = 0; i <= n; i++)
	{
		model.mesh()->add_node(i * L / n, 0, 0);
	}

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::beam);

	//sections
	model.mesh()->add_section(fea::mesh::sections::type::generic);
	((fea::mesh::sections::Generic*) model.mesh()->section(0))->area(A);
	((fea::mesh::sections::Generic*) model.mesh()->section(0))->inertia(I);
	((fea::mesh::sections::Generic*) model.mesh()->section(0))->inertia(0, 0, I);
	((fea::mesh::sections::Generic*) model.mesh()->section(0))->inertia(1, 1, I);

	//elements
	for(unsigned i = 0; i < n; i++)
	{
		model.mesh()->add_element(fea::mesh::elements::type::beam3C, {i, i + 1});
	}

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->poisson_ratio(v);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->elastic_modulus(E);

	//supports
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_3);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_3);
	model.boundary()->add_support(n, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(n, fea::mesh::nodes::dof::translation_3);

	//loads
	model.boundary()->add_load_case();
	model.boundary()->load_case(0)->add_load_node(n, fea::mesh::nodes::dof::translation_1, F);
	model.boundary()->load_case(0)->add_load_node(n / 2, fea::mesh::nodes::dof::translation_3, P);
	for(unsigned i = 0; i < n; i++)
	{
		model.boundary()->load_case(0)->add_load_node(i + 0, fea::mesh::nodes::dof::translation_2, -p * L / n / 2);
		model.boundary()->load_case(0)->add_load_node(i + 1, fea::mesh::nodes::dof::translation_2, -p * L / n / 2);
	}

	//solver
	fea::mesh::elements::Mechanic::geometric(true);
	model.analysis()->solver(fea::analysis::solvers::type::static_nonlinear);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_max(1);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->step_max(100);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_guess(0.01);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->watch_dof(n, fea::mesh::nodes::dof::translation_1);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->strategy(fea::analysis::strategies::type::control_load);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}