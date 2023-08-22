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
#include "fea/inc/Mesh/Sections/Rectangle.h"
#include "fea/inc/Mesh/Materials/Mechanic/Steel.h"
#include "fea/inc/Mesh/Elements/Mechanic/Mechanic.h"

#include "fea/inc/Boundary/Boundary.h"
#include "fea/inc/Boundary/Loads/Types.h"
#include "fea/inc/Boundary/Loads/Load_Case.h"
#include "fea/inc/Boundary/Loads/Elements/Element.h"
#include "fea/inc/Boundary/Loads/Elements/Mechanic/Line/Line_Force.h"

#include "fea/inc/Analysis/Analysis.h"
#include "fea/inc/Analysis/Solvers/Types.h"
#include "fea/inc/Analysis/Strategies/Types.h"
#include "fea/inc/Analysis/Solvers/Static_Nonlinear.h"

//ben
#include "ben/inc/benchmarks/mechanic/beam.h"

void tests::beam::static_nonlinear::elastic::plane::frame_roof(void)
{
	/*
	Elastic roof frame (primary path)
	Literature: International Journal of nonlinear mechanics - N. Xiao, H. Zhong (2012) pp. 633-640
	 */

	//model
	fea::models::Model model("frame roof", "benchmarks/beam/static/nonlinear/elastic/plane");

	//parameters
	const double l = 60;
	const double t = M_PI / 6;

	const double c = cos(t);
	const double s = sin(t);

	//nodes
	model.mesh()->add_node(0, 0, 0);
	model.mesh()->add_node(0, l, 0);
	model.mesh()->add_node(l * c, l * (1 + s), 0);
	model.mesh()->add_node(2 * l * c, l, 0);
	model.mesh()->add_node(2 * l * c, 0, 0);

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::beam);

	//sections
	model.mesh()->add_section(fea::mesh::sections::type::rectangle);
	((fea::mesh::sections::Rectangle*) model.mesh()->section(0))->width(3);
	((fea::mesh::sections::Rectangle*) model.mesh()->section(0))->height(1);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->elastic_modulus(720);

	//elements
	model.mesh()->add_element(fea::mesh::elements::type::beam2C, {0, 1});
	model.mesh()->add_element(fea::mesh::elements::type::beam2C, {1, 2});
	model.mesh()->add_element(fea::mesh::elements::type::beam2C, {2, 3});
	model.mesh()->add_element(fea::mesh::elements::type::beam2C, {3, 4});

	//supports
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_3);
	model.boundary()->add_support(4, fea::mesh::nodes::dof::rotation_3);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(4, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(4, fea::mesh::nodes::dof::translation_2);

	//loads
	model.boundary()->add_load_set();
	model.boundary()->add_load_case(fea::boundary::loads::type::line_force, {1, 2}, -8.33e-4);
	((fea::boundary::loads::Line_Force*) model.boundary()->load_case(0)->load_element(0))->direction(0, 1, 0);
	for(unsigned i = 0; i < 18; i++)
	{
		model.boundary()->load_case(0)->load_element(0)->elements().push_back(13 + i);
	}

	//solver
	fea::mesh::cells::Line::refine(0, 10);
	fea::mesh::cells::Line::refine(1, 10);
	fea::mesh::cells::Line::refine(2, 10);
	fea::mesh::cells::Line::refine(3, 10);
	fea::mesh::elements::Mechanic::geometric(true);
	model.analysis()->solver(fea::analysis::solvers::type::static_nonlinear);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_set(0);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->step_max(200);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_max(10.0);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_guess(1.00);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->watch_dof(2, fea::mesh::nodes::dof::translation_2);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->strategy(fea::analysis::strategies::type::arc_length_cylindric);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}