//std
#include <cmath>

//fea
#include "fea/inc/Model/Model.h"

#include "fea/inc/Mesh/Mesh.h"
#include "fea/inc/Mesh/Nodes/Dof.h"
#include "fea/inc/Mesh/Cells/Types.h"
#include "fea/inc/Mesh/Joints/Types.h"
#include "fea/inc/Mesh/Joints/Hinge.h"
#include "fea/inc/Mesh/Sections/Types.h"
#include "fea/inc/Mesh/Elements/Types.h"
#include "fea/inc/Mesh/Materials/Types.h"
#include "fea/inc/Mesh/Elements/Mechanic/Mechanic.h"

#include "fea/inc/Boundary/Boundary.h"

#include "fea/inc/Analysis/Analysis.h"
#include "fea/inc/Analysis/Solvers/Types.h"
#include "fea/inc/Analysis/Solvers/Static_Nonlinear.h"

//ben
#include "ben/inc/benchmarks/mechanic/joints.h"

void tests::joint::hinge::static_nonlinear::cantilever(void)
{
	//model
	fea::models::Model model("cantilever", "benchmarks/joints/hinge/static nonlinear");

	//nodes
	model.mesh()->add_node(0, 0, 0);
	model.mesh()->add_node(0, 0, 0);
	model.mesh()->add_node(1, 0, 0);

	//celss
	model.mesh()->add_cell(fea::mesh::cells::type::beam);

	//sections
	model.mesh()->add_section(fea::mesh::sections::type::rectangle);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);

	//elements
	fea::mesh::elements::Mechanic::geometric(true);
	model.mesh()->add_element(fea::mesh::elements::type::beam3C, {1, 2});

	//joints
	model.mesh()->add_joint(fea::mesh::joints::type::hinge, {0, 1});
	((fea::mesh::joints::Hinge*) model.mesh()->joint(0))->stiffness(0, 1.0e5, 1.0e5, 1.0e5);

	//supports
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_3);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_3);

	//loads
	model.boundary()->add_load_case(2, fea::mesh::nodes::dof::translation_2, 1.0e5);

	//solver
	model.analysis()->solver(fea::analysis::solvers::type::static_nonlinear);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->step_max(200);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_max(1e2);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->dof_max(M_PI / 2);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_guess(1e-2);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->watch_dof(1, fea::mesh::nodes::dof::rotation_3);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}