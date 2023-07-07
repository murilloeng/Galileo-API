//fea
#include "Model/Model.h"

#include "Mesh/Mesh.h"
#include "Mesh/Nodes/Dof.h"

#include "Boundary/Boundary.h"

#include "Analysis/Analysis.h"
#include "Analysis/Solvers/Types.h"
#include "Analysis/Solvers/Static_Linear.h"

//ben
#include "benchmarks/dof/dof.h"

void tests::dof::static_linear(void)
{
	//data
	const double K = 1.00e+00;
	const double P = 1.00e+00;

	//model
	fea::models::Model model("linear", "benchmarks/dof/static");

	//nodes
	model.mesh()->add_node(0, 0, 0);

	//supports
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_1, K, 0, 0);

	//loads
	model.boundary()->add_load_set();
	model.boundary()->add_load_case(0, fea::mesh::nodes::dof::translation_1, P);

	//solver
	model.analysis()->solver(fea::analysis::solvers::type::static_linear);
	dynamic_cast<fea::analysis::solvers::Static_Linear*>(model.analysis()->solver())->load_set(0);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}