//std
#include <cmath>

//fea
#include "fea/inc/Model/Model.h"

#include "fea/inc/Mesh/Mesh.h"
#include "fea/inc/Mesh/Nodes/Dof.h"
#include "fea/inc/Mesh/Cells/Types.h"
#include "fea/inc/Mesh/Sections/Types.h"
#include "fea/inc/Mesh/Elements/Types.h"
#include "fea/inc/Mesh/Materials/Types.h"
#include "fea/inc/Mesh/Sections/Rectangle.h"
#include "fea/inc/Mesh/Materials/Mechanic/Steel.h"
#include "fea/inc/Mesh/Elements/Mechanic/Mechanic.h"

#include "fea/inc/Boundary/Boundary.h"
#include "fea/inc/Boundary/Loads/Load_Case.h"

#include "fea/inc/Analysis/Analysis.h"
#include "fea/inc/Analysis/Solvers/Types.h"
#include "fea/inc/Analysis/Strategies/Types.h"
#include "fea/inc/Analysis/Solvers/Static_Nonlinear.h"

//ben
#include "ben/inc/benchmarks/mechanic/bar.h"

void tests::bar::static_nonlinear::space_truss(void)
{
	/*
	Space truss subjeted to vertical load
	Literature: S. Krenk - Nonlinear modeling and analysis of solids and models (2009) pp. 281
	 */

	//model
	fea::models::Model model("space truss", "benchmarks/bar/static/nonlinear");

	//nodes
	model.mesh()->add_node(-1.697, +1.000, +0.000);
	model.mesh()->add_node(-1.697, -1.000, +0.000);
	model.mesh()->add_node(-1.414, +0.000, +1.000);
	model.mesh()->add_node(+0.000, +1.000, +0.000);
	model.mesh()->add_node(+0.000, -1.000, +0.000);
	model.mesh()->add_node(+0.000, +0.000, +1.000);
	model.mesh()->add_node(+1.697, +1.000, +0.000);
	model.mesh()->add_node(+1.697, -1.000, +0.000);
	model.mesh()->add_node(+1.414, +0.000, +1.000);

	//sections
	model.mesh()->add_section(fea::mesh::sections::type::rectangle);
	((fea::mesh::sections::Rectangle*) model.mesh()->section(0))->width(0.1);
	((fea::mesh::sections::Rectangle*) model.mesh()->section(0))->height(0.1);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->elastic_modulus(200e9);

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::bar);

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::bar);

	//elements
	fea::mesh::elements::Mechanic::geometric(true);


	model.mesh()->add_element(fea::mesh::elements::type::bar3, {0, 2});
	model.mesh()->add_element(fea::mesh::elements::type::bar3, {1, 2});
	model.mesh()->add_element(fea::mesh::elements::type::bar3, {3, 5});
	model.mesh()->add_element(fea::mesh::elements::type::bar3, {4, 5});
	model.mesh()->add_element(fea::mesh::elements::type::bar3, {6, 8});
	model.mesh()->add_element(fea::mesh::elements::type::bar3, {7, 8});
	model.mesh()->add_element(fea::mesh::elements::type::bar3, {0, 5});
	model.mesh()->add_element(fea::mesh::elements::type::bar3, {1, 5});
	model.mesh()->add_element(fea::mesh::elements::type::bar3, {2, 5});
	model.mesh()->add_element(fea::mesh::elements::type::bar3, {6, 5});
	model.mesh()->add_element(fea::mesh::elements::type::bar3, {7, 5});
	model.mesh()->add_element(fea::mesh::elements::type::bar3, {8, 5});

	//dependencies
//	model.boundary()->add_dependency(8, 2, fea::mesh::nodes::dof::translation_1, fea::mesh::nodes::dof::translation_1, 0, -1);
//	model.boundary()->add_dependency(8, 2, fea::mesh::nodes::dof::translation_3, fea::mesh::nodes::dof::translation_3, 0, +1);

	//supports
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_3);
	model.boundary()->add_support(1, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(1, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(1, fea::mesh::nodes::dof::translation_3);
	model.boundary()->add_support(2, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(3, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(3, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(3, fea::mesh::nodes::dof::translation_3);
	model.boundary()->add_support(4, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(4, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(4, fea::mesh::nodes::dof::translation_3);
	model.boundary()->add_support(5, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(5, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(6, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(6, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(6, fea::mesh::nodes::dof::translation_3);
	model.boundary()->add_support(7, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(7, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(7, fea::mesh::nodes::dof::translation_3);
	model.boundary()->add_support(8, fea::mesh::nodes::dof::translation_2);

	//loads
	model.boundary()->add_load_case();
	model.boundary()->load_case(0)->add_load_node(2, fea::mesh::nodes::dof::translation_3, -3e9);
	model.boundary()->load_case(0)->add_load_node(5, fea::mesh::nodes::dof::translation_3, -2e9);
	model.boundary()->load_case(0)->add_load_node(8, fea::mesh::nodes::dof::translation_3, -3e9);

	//solver
	model.analysis()->solver(fea::analysis::solvers::type::static_nonlinear);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->step_max(500);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_max(0.1);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_guess(1e-2);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_increment_max(0.02);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->watch_dof(5, fea::mesh::nodes::dof::translation_3);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->strategy(fea::analysis::strategies::type::arc_length_linear);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}