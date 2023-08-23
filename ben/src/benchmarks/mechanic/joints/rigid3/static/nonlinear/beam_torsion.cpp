//std
#include <cmath>
#include <ctime>

//fea
#include "fea/inc/Model/Model.h"

#include "fea/inc/Mesh/Mesh.h"
#include "fea/inc/Mesh/Nodes/Dof.h"
#include "fea/inc/Mesh/Cells/Types.h"
#include "fea/inc/Mesh/Joints/Types.h"
#include "fea/inc/Mesh/Sections/Types.h"
#include "fea/inc/Mesh/Elements/Types.h"
#include "fea/inc/Mesh/Cells/Line/Line.h"
#include "fea/inc/Mesh/Materials/Types.h"
#include "fea/inc/Mesh/Sections/Rectangle.h"
#include "fea/inc/Mesh/Materials/Mechanic/Steel.h"
#include "fea/inc/Mesh/Elements/Mechanic/Mechanic.h"

#include "fea/inc/Boundary/Boundary.h"

#include "fea/inc/Analysis/Analysis.h"
#include "fea/inc/Analysis/Solvers/Types.h"
#include "fea/inc/Analysis/Strategies/Types.h"
#include "fea/inc/Analysis/Solvers/Static_Nonlinear.h"

//ben
#include "ben/inc/benchmarks/mechanic/joints.h"

void tests::joint::rigid3::static_nonlinear::beam_torsion(void)
{
	/*
	Beam with rigid link force
	Closed-form response:
		P = 2 G J / b / L t /cos(t)
	 */

	//data
	const unsigned n = 1;
	const double b = 2.00e-01;
	const double h = 4.00e-01;
	const double l = 3.00e+00;
	const double E = 2.00e+11;
	const double v = 3.00e-01;
	const double P = 1.00e+06;

	//model
	fea::models::Model model("beam torsion", "benchmarks/joints/rigid3/static/nonlinear");

	//nodes
	model.mesh()->add_node(0, 0, 0);
	model.mesh()->add_node(l, 0, 0);
	model.mesh()->add_node(l, h / 2, 0);

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::beam);

	//joints
	model.mesh()->add_joint(fea::mesh::joints::type::rigid3, {1, 2});

	//sections
	model.mesh()->add_section(fea::mesh::sections::type::rectangle);
	((fea::mesh::sections::Rectangle*) model.mesh()->section(0))->width(b);
	((fea::mesh::sections::Rectangle*) model.mesh()->section(0))->height(h);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->poisson_ratio(v);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->elastic_modulus(E);

	//elements
	model.mesh()->add_element(fea::mesh::elements::type::beam3T, {0, 1}, 0, 0);

	//supports
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_3);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_3);
	model.boundary()->add_support(1, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(1, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(1, fea::mesh::nodes::dof::translation_3);

	//loads
	model.boundary()->add_load_case(2, fea::mesh::nodes::dof::translation_3, P);

	//solver
	fea::mesh::cells::Line::refine(0, n);
	fea::mesh::elements::Mechanic::geometric(true);
	model.analysis()->solver(fea::analysis::solvers::type::static_nonlinear);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_max(1e3);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->step_max(1e3);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_guess(1e+0);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->watch_dof(1, fea::mesh::nodes::dof::rotation_1);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->strategy(fea::analysis::strategies::type::control_load);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}