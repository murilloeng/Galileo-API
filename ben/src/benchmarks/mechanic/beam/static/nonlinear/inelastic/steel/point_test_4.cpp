//std
#include <cmath>

//fea
#include "Model/Model.h"

#include "Mesh/Mesh.h"
#include "Mesh/Nodes/Dof.h"
#include "Mesh/Cells/Types.h"
#include "Mesh/Sections/Types.h"
#include "Mesh/Elements/Types.h"
#include "Mesh/Cells/Line/Line.h"
#include "Mesh/Materials/Types.h"
#include "Mesh/Sections/Rectangle.h"
#include "Mesh/Materials/Mechanic/Steel.h"
#include "Mesh/Elements/Mechanic/Mechanic.h"
#include "Mesh/Elements/Mechanic/Frame/Beam.h"

#include "Boundary/Boundary.h"
#include "Boundary/Loads/Load_Case.h"

#include "Analysis/Analysis.h"
#include "Analysis/Solvers/Types.h"
#include "Analysis/Strategies/Types.h"
#include "Analysis/Solvers/Static_Nonlinear.h"

//ben
#include "benchmarks/mechanic/beam.h"

void tests::beam::static_nonlinear::inelastic::steel::point_test_4(void)
{
	/*
	Pinned beam four point test
		Analytical solution:
		---Elastic limit load---
		Pe = sy w h^2 / 6 / L
		---Plastic limit load---
		Pp = sy w h^2 / 4 / L
	 */

	//data
	const double L = 1.00e+00;
	const double w = 2.00e-01;
	const double h = 4.00e-01;
	const double E = 2.00e+11;
	const double sy = 2.50e+08;
	const double kp = 2.00e+05;

	//model
	fea::models::Model model("point test 4", "benchmarks/beam/static/nonlinear/inelastic/steel");

	//nodes
	model.mesh()->add_node(0 * L, 0, 0);
	model.mesh()->add_node(1 * L, 0, 0);
	model.mesh()->add_node(2 * L, 0, 0);
	model.mesh()->add_node(3 * L, 0, 0);

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::beam);

	//sections
	model.mesh()->add_section(fea::mesh::sections::type::rectangle);
	((fea::mesh::sections::Rectangle*) model.mesh()->section(0))->width(w);
	((fea::mesh::sections::Rectangle*) model.mesh()->section(0))->height(h);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->yield_stress(sy);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->elastic_modulus(E);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->plastic_modulus(kp);

	//elements
	model.mesh()->add_element(fea::mesh::elements::type::beam2C, {0, 1});
	model.mesh()->add_element(fea::mesh::elements::type::beam2C, {1, 2});
	model.mesh()->add_element(fea::mesh::elements::type::beam2C, {2, 3});

	//supports
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_2);
	model.boundary()->add_support(3, fea::mesh::nodes::dof::rotation_1);
	model.boundary()->add_support(3, fea::mesh::nodes::dof::rotation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_3);
	model.boundary()->add_support(3, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(3, fea::mesh::nodes::dof::translation_3);

	//loads
	model.boundary()->add_load_set();
	model.boundary()->add_load_case();
	model.boundary()->load_case(0)->add_load_node(1, fea::mesh::nodes::dof::translation_2, -sy * w * h * h / 4 / L);
	model.boundary()->load_case(0)->add_load_node(2, fea::mesh::nodes::dof::translation_2, -sy * w * h * h / 4 / L);

	//solver
	fea::mesh::elements::Beam::shear(true);
	fea::mesh::elements::Mechanic::inelastic(true);
	fea::mesh::elements::Mechanic::geometric(false);
	model.analysis()->solver(fea::analysis::solvers::type::static_nonlinear);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->step_max(200);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_max(1.10);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_guess(0.05);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->watch_dof(1, fea::mesh::nodes::dof::translation_2);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->strategy(fea::analysis::strategies::type::minimal_norm);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}