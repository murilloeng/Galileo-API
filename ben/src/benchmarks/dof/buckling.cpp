//fea
#include "Model/Model.h"

#include "Mesh/Mesh.h"
#include "Mesh/Nodes/Dof.h"

#include "Boundary/Boundary.h"

#include "Analysis/Analysis.h"
#include "Analysis/Solvers/Types.h"

//ben
#include "benchmarks/dof/dof.h"

void tests::dof::buckling(void)
{
	//data
	const double k = 1.00e+00;

	//model
	fea::models::Model model("buckling", "benchmarks/dof");

	//nodes
	model.mesh()->add_node(0, 0, 0);

	//supports
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_1, k, 0, 0);

	//solver
	model.analysis()->solver(fea::analysis::solvers::type::buckling);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}