//std
#include <cmath>

//fea
#include "Model/Model.h"

#include "Mesh/Mesh.h"
#include "Mesh/Nodes/Dof.h"
#include "Mesh/Joints/Types.h"
#include "Mesh/Joints/Hinge.h"

#include "Boundary/Boundary.h"

#include "Analysis/Analysis.h"
#include "Analysis/Solvers/Types.h"
#include "Analysis/Solvers/Static_Nonlinear.h"

//ben
#include "benchmarks/mechanic/joints.h"

void tests::joint::hinge::static_nonlinear::bending(void)
{
	//model
	fea::models::Model model("bending", "benchmarks/joints/hinge/static nonlinear");

	//nodes
	model.mesh()->add_node(0, 0, 0);
	model.mesh()->add_node(0, 0, 0);

	//joints
	model.mesh()->add_joint(fea::mesh::joints::type::hinge, {0, 1});
	((fea::mesh::joints::Hinge*) model.mesh()->joint(0))->stiffness(0, 1, 1, 1);

	//supports
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_3);
	model.boundary()->add_support(1, fea::mesh::nodes::dof::rotation_3);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_3);

	//loads
	model.boundary()->add_load_case(1, fea::mesh::nodes::dof::rotation_1, 1);

	//solver
	model.analysis()->solver(fea::analysis::solvers::type::static_nonlinear);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_max(1e2);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->step_max(2000);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->dof_max(M_PI / 18);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_guess(1e-4);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->watch_dof(1, fea::mesh::nodes::dof::rotation_1);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}