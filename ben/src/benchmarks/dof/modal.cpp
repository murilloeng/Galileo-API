//fea
#include "fea/inc/Model/Model.h"

#include "fea/inc/Mesh/Mesh.h"
#include "fea/inc/Mesh/Nodes/Dof.h"

#include "fea/inc/Boundary/Boundary.h"

#include "fea/inc/Analysis/Analysis.h"
#include "fea/inc/Analysis/Solvers/Types.h"

//ben
#include "ben/inc/benchmarks/dof/dof.h"

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