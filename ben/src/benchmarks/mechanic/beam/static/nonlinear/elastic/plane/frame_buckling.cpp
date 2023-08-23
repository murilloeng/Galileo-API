//std
#include <cmath>

//fea
#include "fea/inc/Model/Model.h"

#include "fea/inc/Mesh/Mesh.h"
#include "fea/inc/Mesh/Nodes/Dof.h"
#include "fea/inc/Mesh/Cells/Types.h"
#include "fea/inc/Mesh/Sections/Types.h"
#include "fea/inc/Mesh/Elements/Types.h"
#include "fea/inc/Mesh/Cells/Line/Line.h"
#include "fea/inc/Mesh/Materials/Types.h"
#include "fea/inc/Mesh/Sections/Generic.h"
#include "fea/inc/Mesh/Materials/Mechanic/Steel.h"
#include "fea/inc/Mesh/Elements/Mechanic/Mechanic.h"

#include "fea/inc/Boundary/Boundary.h"
#include "fea/inc/Boundary/Loads/Load_Case.h"

#include "fea/inc/Analysis/Analysis.h"
#include "fea/inc/Analysis/Solvers/Types.h"
#include "fea/inc/Analysis/Strategies/Types.h"
#include "fea/inc/Analysis/Solvers/Static_Nonlinear.h"

//ben
#include "ben/inc/benchmarks/mechanic/beam.h"

void tests::beam::static_nonlinear::elastic::plane::frame_buckling(void)
{
	/* Frame buckling */

	//parameters
	const double l = 5.00e+00;
	const double E = 2.00e+08;
	const double A0 = 1.74e-02;
	const double A1 = 2.76e-02;
	const double I0 = 5.24e-04;
	const double I1 = 1.16e-03;

	//model
	fea::models::Model model("frame buckling", "benchmarks/beam/static/nonlinear/elastic/plane");

	//nodes
	model.mesh()->add_node(0, 0, 0);
	model.mesh()->add_node(0, l, 0);
	model.mesh()->add_node(2 * l, l, 0);
	model.mesh()->add_node(2 * l, 0, 0);

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::beam);
	model.mesh()->add_cell(fea::mesh::cells::type::beam);
	((fea::mesh::cells::Line*) model.mesh()->cell(0))->section(0);
	((fea::mesh::cells::Line*) model.mesh()->cell(1))->section(1);

	//sections
	model.mesh()->add_section(fea::mesh::sections::type::generic);
	model.mesh()->add_section(fea::mesh::sections::type::generic);
	((fea::mesh::sections::Generic*) model.mesh()->section(0))->area(A0);
	((fea::mesh::sections::Generic*) model.mesh()->section(1))->area(A1);
	((fea::mesh::sections::Generic*) model.mesh()->section(0))->inertia(1, 1, I0);
	((fea::mesh::sections::Generic*) model.mesh()->section(1))->inertia(1, 1, I1);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->elastic_modulus(E);

	//elements
	fea::mesh::elements::Mechanic::geometric(true);
	model.mesh()->add_element(fea::mesh::elements::type::beam2C, {0, 1}, 0, 0);
	model.mesh()->add_element(fea::mesh::elements::type::beam2C, {1, 2}, 0, 1);
	model.mesh()->add_element(fea::mesh::elements::type::beam2C, {2, 3}, 0, 0);

	//supports
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_3);
	model.boundary()->add_support(3, fea::mesh::nodes::dof::rotation_3);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(3, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(3, fea::mesh::nodes::dof::translation_2);

	//loads
	model.boundary()->add_load_set();
	model.boundary()->add_load_case();
	model.boundary()->load_case(0)->add_load_node(1, fea::mesh::nodes::dof::translation_1, +3e3);
	model.boundary()->load_case(0)->add_load_node(1, fea::mesh::nodes::dof::translation_2, -3e4);
	model.boundary()->load_case(0)->add_load_node(2, fea::mesh::nodes::dof::translation_2, -3e4);

	//solver
	fea::mesh::cells::Line::refine(0, 10);
	fea::mesh::cells::Line::refine(1, 10);
	fea::mesh::cells::Line::refine(2, 10);
	model.analysis()->solver(fea::analysis::solvers::type::static_nonlinear);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_set(0);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->step_max(900);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_max(1.0);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_guess(0.01);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->watch_dof(1, fea::mesh::nodes::dof::translation_1);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->strategy(fea::analysis::strategies::type::control_load);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}