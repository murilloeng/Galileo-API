//fea
#include "Model/Model.h"

#include "Mesh/Mesh.h"
#include "Mesh/Nodes/Dof.h"

#include "Boundary/Boundary.h"

#include "Analysis/Analysis.h"
#include "Analysis/Solvers/Types.h"

//ben
#include "benchmarks/dof/dof.h"

void tests::dof::modal(void)
{
	//data
	const double m = 1.00e+00;
	const double k = 4.00e+00;

	//model
	fea::models::Model model("modal", "benchmarks/dof");

	//nodes
	model.mesh()->add_node(0, 0, 0);

	//supports
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_1, k, 0, m);

	//solver
	model.analysis()->solver(fea::analysis::solvers::type::modal);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}