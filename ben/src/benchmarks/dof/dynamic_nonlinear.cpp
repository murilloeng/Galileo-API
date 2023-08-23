//std
#include <cmath>
#include <functional>

//fea
#include "fea/inc/Model/Model.h"

#include "fea/inc/Mesh/Mesh.h"
#include "fea/inc/Mesh/Nodes/Dof.h"

#include "fea/inc/Boundary/Boundary.h"
#include "fea/inc/Boundary/Time/Types.h"
#include "fea/inc/Boundary/Time/Sine_Wave.h"

#include "fea/inc/Analysis/Analysis.h"
#include "fea/inc/Analysis/Solvers/Types.h"
#include "fea/inc/Analysis/Solvers/Dynamic_Nonlinear.h"

//ben
#include "ben/inc/benchmarks/dof/dof.h"

void tests::dof::dynamic_nonlinear(void)
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
	const unsigned c = (unsigned) fea::analysis::solvers::convergence::fixed;

	//model
	fea::models::Model model("nonlinear", "benchmarks/dof/dynamic");

	//nodes
	model.mesh()->add_node(0, 0, 0);

	//initials
	model.boundary()->add_initial(0, fea::mesh::nodes::dof::translation_1, u, v);

	//supports
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_1, K, C, M);

	//times
	model.boundary()->add_time(fea::boundary::time::type::sine_wave);
	((fea::boundary::time::Sine_Wave*) model.boundary()->time(0))->period(2 * M_PI / w);

	//loads
	model.boundary()->add_load_set();
	model.boundary()->add_load_case(0, fea::mesh::nodes::dof::translation_1, P, 0);

	//solver
	model.analysis()->solver(fea::analysis::solvers::type::dynamic_nonlinear);
	dynamic_cast<fea::analysis::solvers::Dynamic_Nonlinear*> (model.analysis()->solver())->load_set(0);
	dynamic_cast<fea::analysis::solvers::Dynamic_Nonlinear*> (model.analysis()->solver())->time_max(T);
	dynamic_cast<fea::analysis::solvers::Dynamic_Nonlinear*> (model.analysis()->solver())->step_max(ns);
	dynamic_cast<fea::analysis::solvers::Dynamic_Nonlinear*> (model.analysis()->solver())->convergence(c);
	dynamic_cast<fea::analysis::solvers::Dynamic_Nonlinear*> (model.analysis()->solver())->integration(fea::analysis::solvers::integration::newmark);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}