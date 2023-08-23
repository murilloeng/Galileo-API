//fea
#include "fea/inc/Model/Model.h"

#include "fea/inc/Mesh/Mesh.h"
#include "fea/inc/Mesh/Nodes/Dof.h"

#include "fea/inc/Boundary/Boundary.h"

#include "fea/inc/Analysis/Analysis.h"
#include "fea/inc/Analysis/Solvers/Types.h"
#include "fea/inc/Analysis/Solvers/Static_Nonlinear.h"

//ben
#include "ben/inc/benchmarks/dof/dof.h"

void tests::dof::static_nonlinear(void)
{
	//data
	const double K = 1.00e+00;
	const double P = 1.00e+00;

	//model
	fea::models::Model model("nonlinear", "benchmarks/dof/static");

	//nodes
	model.mesh()->add_node(0, 0, 0);

	//supports
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_1, K, 0, 0);

	//loads
	model.boundary()->add_load_set();
	model.boundary()->add_load_case(0, fea::mesh::nodes::dof::translation_1, 1);

	//solver
	model.analysis()->solver(fea::analysis::solvers::type::static_nonlinear);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_set(0);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}