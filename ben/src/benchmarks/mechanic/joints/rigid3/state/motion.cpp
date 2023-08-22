//std
#include <cmath>
#include <ctime>

//fea
#include "fea/inc/Model/Model.h"

#include "fea/inc/Mesh/Mesh.h"
#include "fea/inc/Mesh/Nodes/Dof.h"
#include "fea/inc/Mesh/Joints/Types.h"

#include "fea/inc/Boundary/Boundary.h"
#include "fea/inc/Boundary/Supports/Support.h"

#include "fea/inc/Analysis/Analysis.h"
#include "fea/inc/Analysis/Solvers/Types.h"
#include "fea/inc/Analysis/Solvers/State.h"

//ben
#include "ben/inc/benchmarks/mechanic/joints.h"

void tests::joint::rigid3::state::motion(void)
{
	//model
	fea::models::Model model("motion", "benchmarks/joints/rigid3/state");

	//nodes
	srand(time(nullptr));
	model.mesh()->add_node(+0, +0, +0);
	model.mesh()->add_node(+1, +0, +0);
	model.mesh()->add_node(+0, +1, +0);
	model.mesh()->add_node(-1, +0, +0);
	model.mesh()->add_node(+0, -1, +0);
	model.mesh()->add_node(+1, +1, +0);
	model.mesh()->add_node(-1, +1, +0);
	model.mesh()->add_node(-1, -1, +0);
	model.mesh()->add_node(+1, -1, +0);
	model.mesh()->add_node(+1, +0, -1);
	model.mesh()->add_node(+0, +1, -1);
	model.mesh()->add_node(-1, +0, -1);
	model.mesh()->add_node(+0, -1, -1);
	model.mesh()->add_node(+1, +1, -1);
	model.mesh()->add_node(-1, +1, -1);
	model.mesh()->add_node(-1, -1, -1);
	model.mesh()->add_node(+1, -1, -1);
	model.mesh()->add_node(+1, +0, +1);
	model.mesh()->add_node(+0, +1, +1);
	model.mesh()->add_node(-1, +0, +1);
	model.mesh()->add_node(+0, -1, +1);
	model.mesh()->add_node(+1, +1, +1);
	model.mesh()->add_node(-1, +1, +1);
	model.mesh()->add_node(-1, -1, +1);
	model.mesh()->add_node(+1, -1, +1);

	//joints
	model.mesh()->add_joint(fea::mesh::joints::type::rigid3, {0, 1, 2, 3, 4, 5, 6, 7, 8});
	for(unsigned i = 0; i < 8; i++)
	{
		model.mesh()->add_joint(fea::mesh::joints::type::rigid3, {i + 1, i +  9});
		model.mesh()->add_joint(fea::mesh::joints::type::rigid3, {i + 1, i + 17});
	}

	//supports
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_3);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_3);

	//kinematics
	const double ru = 5;
	const double at = 2 * M_PI * (double) rand() / RAND_MAX;
	const double bt = 2 * M_PI * (double) rand() / RAND_MAX;
	// model.boundary()->support(2)->state([bt] (double t) { return 2 * M_PI * sin(bt) * t; });
	// model.boundary()->support(3)->state([ru] (double t) { return ru * (1 + cos(2 * M_PI * t)); });
	// model.boundary()->support(4)->state([ru] (double t) { return ru * (0 + sin(2 * M_PI * t)); });
	// model.boundary()->support(0)->state([at, bt] (double t) { return 2 * M_PI * cos(at) * cos(bt) * t; });
	// model.boundary()->support(1)->state([at, bt] (double t) { return 2 * M_PI * sin(at) * cos(bt) * t; });

	//solver
	model.analysis()->solver(fea::analysis::solvers::type::state);

	model.analysis()->solver()->step_max(1000);
	model.analysis()->solver()->watch_dof(0, fea::mesh::nodes::dof::rotation_3);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}