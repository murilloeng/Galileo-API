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

void tests::beam::static_nonlinear::elastic::plane::frame_portal(void)
{
	/*
	Frame portal buckling
	Literature: Computers and Structures - M. B. Wong, F. T. Loi (1990) pp. 633-640
	 */

	//parameters
	const double l = 1.20e+02;
	const double A = 1.18e+01;
	const double I = 3.10e+02;
	const double E = 3.00e+07;

	//model
	fea::models::Model model("frame portal", "benchmarks/beam/static/nonlinear/elastic/plane");

	//nodes
	model.mesh()->add_node(0, 0, 0);
	model.mesh()->add_node(0, l, 0);
	model.mesh()->add_node(l, l, 0);
	model.mesh()->add_node(l, 0, 0);

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::beam);

	//sections
	model.mesh()->add_section(fea::mesh::sections::type::generic);
	((fea::mesh::sections::Generic*) model.mesh()->section(0))->area(A);
	((fea::mesh::sections::Generic*) model.mesh()->section(0))->inertia(1, 1, I);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->elastic_modulus(E);

	//elements
	model.mesh()->add_element(fea::mesh::elements::type::beam2C, {0, 1});
	model.mesh()->add_element(fea::mesh::elements::type::beam2C, {1, 2});
	model.mesh()->add_element(fea::mesh::elements::type::beam2C, {2, 3});

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
	model.boundary()->load_case(0)->add_load_node(1, fea::mesh::nodes::dof::translation_1, +1e3);
	model.boundary()->load_case(0)->add_load_node(1, fea::mesh::nodes::dof::translation_2, -1e6);
	model.boundary()->load_case(0)->add_load_node(2, fea::mesh::nodes::dof::translation_2, -1e6);

	//solver
	fea::mesh::cells::Line::refine(0, 10);
	fea::mesh::cells::Line::refine(1, 10);
	fea::mesh::cells::Line::refine(2, 10);
	fea::mesh::elements::Mechanic::geometric(true);
	model.analysis()->solver(fea::analysis::solvers::type::static_nonlinear);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_set(0);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->step_max(200);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_max(5.0);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_guess(0.1);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->watch_dof(1, fea::mesh::nodes::dof::translation_1);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->strategy(fea::analysis::strategies::type::arc_length_cylindric);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}