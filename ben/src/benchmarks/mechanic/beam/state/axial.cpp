//std
#include <cmath>

//fea
#include "fea/inc/Mesh/Mesh.h"
#include "fea/inc/Model/Model.h"
#include "fea/inc/Mesh/Nodes/Dof.h"
#include "fea/inc/Mesh/Cells/Types.h"
#include "fea/inc/Boundary/Boundary.h"
#include "fea/inc/Analysis/Analysis.h"
#include "fea/inc/Boundary/Time/Time.h"
#include "fea/inc/Mesh/Sections/Types.h"
#include "fea/inc/Mesh/Elements/Types.h"
#include "fea/inc/Boundary/Time/Types.h"
#include "fea/inc/Mesh/Materials/Types.h"
#include "fea/inc/Analysis/Solvers/Types.h"
#include "fea/inc/Analysis/Solvers/State.h"
#include "fea/inc/Boundary/Time/Polynomial.h"
#include "fea/inc/Boundary/Supports/Support.h"
#include "fea/inc/Mesh/Elements/Mechanic/Mechanic.h"

//ben
#include "ben/inc/benchmarks/mechanic/beam.h"

//state
void tests::beam::state::axial(void)
{
	//model
	fea::models::Model model("axial", "benchmarks/beam/state");

	//nodes
	model.mesh()->add_node(0, 0, 0);
	model.mesh()->add_node(1, 0, 0);

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::beam);

	//sections
	model.mesh()->add_section(fea::mesh::sections::type::rectangle);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);

	//elements
	fea::mesh::elements::Mechanic::geometric(true);
	model.mesh()->add_element(fea::mesh::elements::type::beam3C, {0, 1});

	//supports
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_3);
	model.boundary()->add_support(1, fea::mesh::nodes::dof::rotation_1);
	model.boundary()->add_support(1, fea::mesh::nodes::dof::rotation_2);
	model.boundary()->add_support(1, fea::mesh::nodes::dof::rotation_3);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_3);
	model.boundary()->add_support(1, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(1, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(1, fea::mesh::nodes::dof::translation_3);

	//times
	model.boundary()->support(9)->state(0);
	model.boundary()->add_time(fea::boundary::time::type::polynomial);
	((fea::boundary::time::Polynomial*) model.boundary()->time(0))->terms().push_back(0);
	((fea::boundary::time::Polynomial*) model.boundary()->time(0))->terms().push_back(1);

	//solver
	model.analysis()->solver(fea::analysis::solvers::type::state);
	model.analysis()->solver()->watch_dof(1, fea::mesh::nodes::dof::translation_1);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}