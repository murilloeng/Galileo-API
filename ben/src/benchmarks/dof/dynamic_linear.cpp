//std
#include <cmath>

//fea
#include "Model/Model.h"

#include "Mesh/Mesh.h"
#include "Mesh/Nodes/Dof.h"

#include "Boundary/Boundary.h"
#include "Boundary/Time/Types.h"
#include "Boundary/Time/Sine_Wave.h"

#include "Analysis/Analysis.h"
#include "Analysis/Solvers/Types.h"
#include "Analysis/Solvers/Dynamic_Linear.h"

//ben
#include "benchmarks/dof/dof.h"

void tests::dof::dynamic_linear(void)
{
	//data
	const unsigned np = 50;
	const unsigned ns = 10000;
	const double M = 1.00e+00;
	const double C = 0.10e+00;
	const double K = 1.00e+00;
	const double u = 0.00e+00;
	const double v = 0.00e+00;
	const double P = 1.00e+00;
	const double w = 5.00e-01;
	const double q = C / 2 / sqrt(K * M);
	const double T = 2 * M_PI * np * sqrt(1 - q * q) * sqrt(K / M);

	//model
	fea::models::Model model("linear", "benchmarks/dof/dynamic");

	//nodes
	model.mesh()->add_node(0, 0, 0);

	//initials
	model.boundary()->add_initial(0, fea::mesh::nodes::dof::translation_1, u, v);

	//supports
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_1, K, C, M);

	//time
	model.boundary()->add_time(fea::boundary::time::type::sine_wave);
	((fea::boundary::time::Sine_Wave*) model.boundary()->time(0))->period(2 * M_PI / w);

	//loads
	model.boundary()->add_load_set();
	model.boundary()->add_load_case(0, fea::mesh::nodes::dof::translation_1, P, 0);

	//solver
	model.analysis()->solver(fea::analysis::solvers::type::dynamic_linear);
	dynamic_cast<fea::analysis::solvers::Dynamic_Linear*> (model.analysis()->solver())->load_set(0);
	dynamic_cast<fea::analysis::solvers::Dynamic_Linear*> (model.analysis()->solver())->time_max(T);
	dynamic_cast<fea::analysis::solvers::Dynamic_Linear*> (model.analysis()->solver())->step_max(ns);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}