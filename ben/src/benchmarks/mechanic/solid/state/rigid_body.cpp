//std
#include <cmath>

//fea
#include "Model/Model.h"

#include "Mesh/Mesh.h"
#include "Mesh/Nodes/Dof.h"
#include "Mesh/Cells/Types.h"
#include "Mesh/Elements/Types.h"
#include "Mesh/Materials/Types.h"
#include "Mesh/Materials/Mechanic/Steel.h"

#include "Boundary/Boundary.h"
#include "Boundary/Loads/Types.h"
#include "Boundary/Loads/Load_Case.h"

#include "Analysis/Analysis.h"
#include "Analysis/Solvers/Types.h"
#include "Analysis/Solvers/State.h"

//ben
#include "benchmarks/mechanic/solid.h"

void tests::solid::state::rigid_body(void)
{
	//model
	fea::models::Model model("rigid body", "benchmarks/solid/state");

	//nodes
	model.mesh()->add_node(-1, -1, -1);
	model.mesh()->add_node(+1, -1, -1);
	model.mesh()->add_node(+1, +1, -1);
	model.mesh()->add_node(-1, +1, -1);
	model.mesh()->add_node(-1, -1, +1);
	model.mesh()->add_node(+1, -1, +1);
	model.mesh()->add_node(+1, +1, +1);
	model.mesh()->add_node(-1, +1, +1);

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::brick8);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);

	//elements
	model.mesh()->add_element(fea::mesh::elements::type::solid, {0, 1, 2, 3, 4, 5, 6, 7});

	//supports
	for(unsigned i = 0; i < 8; i++)
	{
		model.boundary()->add_support(i, fea::mesh::nodes::dof::translation_1);
		model.boundary()->add_support(i, fea::mesh::nodes::dof::translation_2);
		model.boundary()->add_support(i, fea::mesh::nodes::dof::translation_3);
	}

	//solver
	model.analysis()->solver(fea::analysis::solvers::type::state);
	model.analysis()->solver()->watch_dof(0, fea::mesh::nodes::dof::translation_1);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}