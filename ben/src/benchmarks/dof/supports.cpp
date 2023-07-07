//std
#include <cmath>

//fea
#include "Mesh/Mesh.h"
#include "Model/Model.h"
#include "Mesh/Nodes/Dof.h"
#include "Boundary/Boundary.h"
#include "Analysis/Analysis.h"
#include "Boundary/Time/Time.h"
#include "Boundary/Time/Types.h"
#include "Analysis/Solvers/Types.h"
#include "Boundary/Time/Sine_Wave.h"

//ben
#include "benchmarks/dof/dof.h"

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