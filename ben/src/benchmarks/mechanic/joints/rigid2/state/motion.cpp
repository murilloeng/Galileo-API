//std
#include <cmath>
#include <ctime>

//fea
#include "fea/inc/Model/Model.h"

#include "fea/inc/Mesh/Mesh.h"
#include "fea/inc/Mesh/Nodes/Dof.h"
#include "fea/inc/Mesh/Joints/Types.h"
#include "fea/inc/Mesh/Elements/Mechanic/Mechanic.h"

#include "fea/inc/Boundary/Boundary.h"
#include "fea/inc/Boundary/Supports/Support.h"

#include "fea/inc/Analysis/Analysis.h"
#include "fea/inc/Analysis/Solvers/Types.h"
#include "fea/inc/Analysis/Solvers/State.h"

//ben
#include "ben/inc/benchmarks/mechanic/joints.h"

void tests::joint::rigid2::state::motion(void)
{
	/*
	Rigid link subjected to prescribed motion
	Closed-form response:
		vp(t) = v + L sin(t)
		up(t) = u + L (cos(t) - 1)
	 */

	//model
	fea::models::Model model("motion", "benchmarks/joints/rigid2/state");

	//nodes
	model.mesh()->add_node(+0, +0, +0);
	model.mesh()->add_node(+1, +0, +0);

	//joints
	model.mesh()->add_joint(fea::mesh::joints::type::rigid2, {0, 1});

	//supports
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_3);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_2);

	//kinematics
	// model.boundary()->support(1)->state([] (double t) { return t; });
	// model.boundary()->support(2)->state([] (double t) { return t; });
	// model.boundary()->support(0)->state([] (double t) { return 2 * M_PI * t; });

	//solver
	fea::mesh::elements::Mechanic::geometric(true);
	model.analysis()->solver(fea::analysis::solvers::type::state);
	dynamic_cast<fea::analysis::solvers::State*>(model.analysis()->solver())->time_max(2);
	dynamic_cast<fea::analysis::solvers::State*>(model.analysis()->solver())->step_max(1000);
	dynamic_cast<fea::analysis::solvers::State*>(model.analysis()->solver())->watch_dof(0, fea::mesh::nodes::dof::rotation_3);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}