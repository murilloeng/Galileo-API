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

//ben
#include "ben/inc/benchmarks/dof/dof.h"

void tests::dof::supports(void)
{
	//model
	fea::models::Model model("supports", "benchmarks/dof");

	//nodes
	model.mesh()->add_node(1, 0, 0);

	//time
	model.boundary()->add_time(fea::boundary::time::type::sine_wave);
	model.boundary()->add_time(fea::boundary::time::type::sine_wave);
	((fea::boundary::time::Sine_Wave*) model.boundary()->time(0))->phase(-M_PI / 2);

	//supports
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_1, 0);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_2, 1);

	//loads
	model.boundary()->add_load_case();

	//solver
	model.analysis()->solver(fea::analysis::solvers::type::state);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}