//std
#include <cmath>

//fea
#include "fea/inc/Mesh/Mesh.h"
#include "fea/inc/Model/Model.h"
#include "fea/inc/Mesh/Nodes/Dof.h"
#include "fea/inc/Boundary/Boundary.h"
#include "fea/inc/Analysis/Analysis.h"
#include "fea/inc/Boundary/Time/Time.h"
#include "fea/inc/Boundary/Time/Types.h"
#include "fea/inc/Analysis/Solvers/Types.h"
#include "fea/inc/Boundary/Time/Sine_Wave.h"
#include "fea/inc/Boundary/Dependencies/Dependency.h"

//ben
#include "ben/inc/benchmarks/dof/dof.h"

void tests::dof::dependencies(void)
{
	//model
	fea::models::Model model("dependencies", "benchmarks/dof");

	//nodes
	model.mesh()->add_node(0, 0, 0);

	//time
	model.boundary()->add_time(fea::boundary::time::type::sine_wave);
	((fea::boundary::time::Sine_Wave*) model.boundary()->time(0))->phase(-M_PI / 2);

	//supports
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_1, 0);

	//dependencies
	model.boundary()->add_dependency(0, fea::mesh::nodes::dof::translation_2, 0, fea::mesh::nodes::dof::translation_1);
	model.boundary()->dependency(0)->state([] (const double* d) { return d[0] * d[0]; });

	//loads
	model.boundary()->add_load_case();

	//solver
	model.analysis()->solver(fea::analysis::solvers::type::state);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}