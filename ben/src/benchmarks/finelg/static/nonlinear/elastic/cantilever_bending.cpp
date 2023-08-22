//std
#include <cmath>

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
#include "fea/inc/Mesh/Sections/Profile_I.h"
#include "fea/inc/Mesh/Materials/Mechanic/Steel.h"
#include "fea/inc/Mesh/Elements/Mechanic/Mechanic.h"
#include "fea/inc/Mesh/Elements/Mechanic/Frame/Beam.h"

#include "fea/inc/Boundary/Boundary.h"
#include "fea/inc/Boundary/Loads/Load_Case.h"

#include "fea/inc/Analysis/Analysis.h"
#include "fea/inc/Analysis/Solvers/Types.h"
#include "fea/inc/Analysis/Strategies/Types.h"
#include "fea/inc/Analysis/Solvers/Static_Nonlinear.h"

//ben
#include "ben/inc/benchmarks/finelg/finelg.h"

void tests::finelg::static_nonlinear::elastic::cantilever_bending(void)
{
	/*
	Cantilever bending
	Literature: Phd Thesis V. V. Goyet (1989) pp. 8.34
	*/

	//data
	const double L = 1.20e+02;
	const double h = 8.88e+00;
	const double w = 1.04e+01;
	const double t = 6.80e-01;
	const double s = 1.11e+00;
	const double v = 3.00e-01;
	const double E = 2.90e+04;
	const double M = 1.00e+04;
	const double T = 1.00e-01;

	//model
	fea::models::Model model("cantilever bending", "benchmarks/finelg/static/nonlinear/elastic");

	//nodes
	model.mesh()->add_node(0 * L, 0, 0);
	model.mesh()->add_node(1 * L, 0, 0);
	model.mesh()->add_node(2 * L, 0, 0);
	model.mesh()->add_node(1 * L, -h / 2, 0);
	model.mesh()->add_node(1 * L, +h / 2, 0);
	model.mesh()->add_node(1 * L, 0, -h / 2);
	model.mesh()->add_node(1 * L, 0, +h / 2);

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::beam);

	//joints
	model.mesh()->add_joint(fea::mesh::joints::type::rigid3, {1, 3});
	model.mesh()->add_joint(fea::mesh::joints::type::rigid3, {1, 4});
	model.mesh()->add_joint(fea::mesh::joints::type::rigid3, {1, 5});
	model.mesh()->add_joint(fea::mesh::joints::type::rigid3, {1, 6});

	//sections
	model.mesh()->add_section(fea::mesh::sections::type::profile_I);
	((fea::mesh::sections::Profile_I*) model.mesh()->section(0))->web_height(h);
	((fea::mesh::sections::Profile_I*) model.mesh()->section(0))->web_thickness(t);
	((fea::mesh::sections::Profile_I*) model.mesh()->section(0))->flange_top_width(w);
	((fea::mesh::sections::Profile_I*) model.mesh()->section(0))->flange_bottom_width(w);
	((fea::mesh::sections::Profile_I*) model.mesh()->section(0))->flange_top_thickness(s);
	((fea::mesh::sections::Profile_I*) model.mesh()->section(0))->flange_bottom_thickness(s);

	//elements
	model.mesh()->add_element(fea::mesh::elements::type::beam3C, {0, 1});
	model.mesh()->add_element(fea::mesh::elements::type::beam3C, {1, 2});

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->poisson_ratio(v);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->elastic_modulus(E);

	//supports
//	model.boundary()->add_support(0, fea::mesh::nodes::dof::warping_1);
//	model.boundary()->add_support(2, fea::mesh::nodes::dof::warping_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_1);
	model.boundary()->add_support(2, fea::mesh::nodes::dof::rotation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_3);
	model.boundary()->add_support(2, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(2, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(2, fea::mesh::nodes::dof::translation_3);

	//loads
	model.boundary()->add_load_case("dead");
	model.boundary()->add_load_case("load");
	model.boundary()->load_case(0)->add_load_node(1, fea::mesh::nodes::dof::rotation_1, +T);
	model.boundary()->load_case(1)->add_load_node(0, fea::mesh::nodes::dof::rotation_3, -M);
	model.boundary()->load_case(1)->add_load_node(2, fea::mesh::nodes::dof::rotation_3, +M);

	//solver
	fea::mesh::cells::Line::refine(0, 5);
	fea::mesh::cells::Line::refine(1, 5);
	fea::mesh::elements::Beam::high_order(true);
	fea::mesh::elements::Mechanic::geometric(true);
	model.analysis()->solver(fea::analysis::solvers::type::static_nonlinear);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_max(3.5);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->step_max(100);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_guess(3.50e-2);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->watch_dof(1, fea::mesh::nodes::dof::rotation_1);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->strategy(fea::analysis::strategies::type::control_load);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}