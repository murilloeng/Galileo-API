//std
#include <cmath>

//fea
#include "Mesh/Mesh.h"
#include "Model/Model.h"
#include "Mesh/Nodes/Dof.h"
#include "Mesh/Cells/Types.h"
#include "Boundary/Boundary.h"
#include "Analysis/Analysis.h"
#include "Boundary/Time/Time.h"
#include "Mesh/Sections/Types.h"
#include "Mesh/Elements/Types.h"
#include "Boundary/Time/Types.h"
#include "Mesh/Materials/Types.h"
#include "Analysis/Solvers/Types.h"
#include "Analysis/Solvers/State.h"
#include "Boundary/Time/Polynomial.h"
#include "Boundary/Supports/Support.h"
#include "Mesh/Elements/Mechanic/Mechanic.h"

//ben
#include "benchmarks/mechanic/beam.h"

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