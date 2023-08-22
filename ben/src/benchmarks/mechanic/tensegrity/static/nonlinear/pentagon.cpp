//std
#include <cmath>

//fea
#include "fea/inc/Model/Model.h"

#include "fea/inc/Mesh/Mesh.h"
#include "fea/inc/Mesh/Nodes/Dof.h"
#include "fea/inc/Mesh/Cells/Types.h"
#include "fea/inc/Mesh/Joints/Types.h"
#include "fea/inc/Mesh/Joints/Joint.h"
#include "fea/inc/Mesh/Sections/Round.h"
#include "fea/inc/Mesh/Sections/Types.h"
#include "fea/inc/Mesh/Elements/Types.h"
#include "fea/inc/Mesh/Cells/Line/Bar.h"
#include "fea/inc/Mesh/Materials/Types.h"
#include "fea/inc/Mesh/Cells/Line/Line.h"
#include "fea/inc/Mesh/Materials/Mechanic/Steel.h"
#include "fea/inc/Mesh/Elements/Mechanic/Mechanic.h"
#include "fea/inc/Mesh/Elements/Mechanic/Frame/Bar.h"

#include "fea/inc/Boundary/Boundary.h"
#include "fea/inc/Boundary/Loads/Load_Case.h"

#include "fea/inc/Analysis/Analysis.h"
#include "fea/inc/Analysis/Solvers/Types.h"
#include "fea/inc/Analysis/Strategies/Types.h"
#include "fea/inc/Analysis/Solvers/Static_Nonlinear.h"

//ben
#include "ben/inc/benchmarks/mechanic/tensegrity.h"

//model
void tests::tensegrity::static_nonlinear::pentagon(void)
{
	//data
	const double s0 = 200e6;	//residual stress
	const double E0 = 115e9;	//elastic modulus: cable
	const double E1 = 210e9;	//elastic modulus: strut
	const double A0 = 1.5e-4;	//cross section area: cable
	const double A1 = 6.0e-4;	//cross section area: strut

	//model
	fea::models::Model model("pentagon", "benchmarks/tensegrity/static/nonlinear");

	//nodes
	model.mesh()->add_node(+0.000, +0.000, +3.894);
	model.mesh()->add_node(+0.000, +3.703, +1.203);
	model.mesh()->add_node(+0.000, +2.289, -3.150);
	model.mesh()->add_node(+0.000, -2.289, -3.150);
	model.mesh()->add_node(+0.000, -3.703, +1.203);
	model.mesh()->add_node(+2.500, +0.000, -3.894);
	model.mesh()->add_node(+2.500, -3.703, -1.203);
	model.mesh()->add_node(+2.500, -2.289, +3.150);
	model.mesh()->add_node(+2.500, +2.289, +3.150);
	model.mesh()->add_node(+2.500, +3.703, -1.203);
	model.mesh()->add_node(+5.000, +0.000, +3.894);
	model.mesh()->add_node(+5.000, +3.703, +1.203);
	model.mesh()->add_node(+5.000, +2.289, -3.150);
	model.mesh()->add_node(+5.000, -2.289, -3.150);
	model.mesh()->add_node(+5.000, -3.703, +1.203);

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::bar);
	model.mesh()->add_cell(fea::mesh::cells::type::bar);
	((fea::mesh::cells::Line*) model.mesh()->cell(0))->section(0);
	((fea::mesh::cells::Line*) model.mesh()->cell(1))->section(1);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);
	model.mesh()->add_material(fea::mesh::materials::type::steel);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->elastic_modulus(E0);
	((fea::mesh::materials::Steel*) model.mesh()->material(1))->elastic_modulus(E1);

	//sections
	model.mesh()->add_section(fea::mesh::sections::type::round);
	model.mesh()->add_section(fea::mesh::sections::type::round);
	((fea::mesh::sections::Round*) model.mesh()->section(0))->diameter(2 * sqrt(A0 / M_PI));
	((fea::mesh::sections::Round*) model.mesh()->section(1))->diameter(2 * sqrt(A1 / M_PI));

	//elements
	model.mesh()->add_element(fea::mesh::elements::type::bar3, { 0,  1}, 0, 0);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, { 1,  2}, 0, 0);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, { 2,  3}, 0, 0);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, { 3,  4}, 0, 0);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, { 4,  0}, 0, 0);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, {10, 11}, 0, 0);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, {11, 12}, 0, 0);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, {12, 13}, 0, 0);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, {13, 14}, 0, 0);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, {14, 10}, 0, 0);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, { 0,  8}, 0, 0);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, { 1,  8}, 0, 0);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, {10,  8}, 0, 0);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, {11,  8}, 0, 0);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, { 1,  9}, 0, 0);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, { 2,  9}, 0, 0);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, {11,  9}, 0, 0);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, {12,  9}, 0, 0);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, { 2,  5}, 0, 0);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, { 3,  5}, 0, 0);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, {12,  5}, 0, 0);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, {13,  5}, 0, 0);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, { 3,  6}, 0, 0);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, { 4,  6}, 0, 0);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, {13,  6}, 0, 0);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, {14,  6}, 0, 0);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, { 0,  7}, 0, 0);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, { 4,  7}, 0, 0);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, {10,  7}, 0, 0);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, {14,  7}, 0, 0);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, { 0, 11}, 1, 1);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, { 1, 12}, 1, 1);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, { 2, 13}, 1, 1);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, { 3, 14}, 1, 1);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, { 4, 10}, 1, 1);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, {14,  8}, 1, 1);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, { 8,  2}, 1, 1);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, {10,  9}, 1, 1);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, { 9,  3}, 1, 1);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, {11,  5}, 1, 1);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, { 5,  4}, 1, 1);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, {12,  6}, 1, 1);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, { 6,  0}, 1, 1);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, {13,  7}, 1, 1);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, { 7,  1}, 1, 1);

	//cables
	for(unsigned i = 0; i < 30; i++)
	{
		((fea::mesh::elements::Bar*) model.mesh()->element(i))->cable(true);
		((fea::mesh::elements::Bar*) model.mesh()->element(i))->residual_stress(s0);
	}

	//supports
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_3);
	model.boundary()->add_support(1, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(1, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(1, fea::mesh::nodes::dof::translation_3);
	model.boundary()->add_support(2, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(2, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(2, fea::mesh::nodes::dof::translation_3);
	model.boundary()->add_support(3, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(3, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(3, fea::mesh::nodes::dof::translation_3);
	model.boundary()->add_support(4, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(4, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(4, fea::mesh::nodes::dof::translation_3);

	//load cases
//	model.boundary()->add_load_case();
//	for(unsigned i = 0; i < 15; i++)
//	{
//		model.boundary()->load_case(0)->add_load_node((i), fea::mesh::nodes::dof::translation_2, -100);
//		((fea::mesh::elements::Bar*) model.mesh()->element(i))->residual_stress(115E09 * 0.05);
//	}
//	model.boundary()->load_case(0)->add_load_node(10, fea::mesh::nodes::dof::translation_2, 0);
//	model.boundary()->load_case(0)->add_load_node(11, fea::mesh::nodes::dof::translation_1, 0);
//	model.boundary()->load_case(0)->add_load_node(12, fea::mesh::nodes::dof::translation_1, 0);
//	model.boundary()->load_case(0)->add_load_node(13, fea::mesh::nodes::dof::translation_1, 0);
//	model.boundary()->load_case(0)->add_load_node(14, fea::mesh::nodes::dof::translation_1, 0);

	model.boundary()->add_self_weight("gravity", fea::mesh::nodes::dof::translation_1);

	//solver
	fea::mesh::elements::Mechanic::geometric(true);
	model.analysis()->solver(fea::analysis::solvers::type::static_nonlinear);
	model.analysis()->solver()->watch_dof(10, fea::mesh::nodes::dof::translation_3);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->step_max(2000);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_max(1.00);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_guess(0.05);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->strategy(fea::analysis::strategies::type::control_load);
	model.analysis()->solve();

	//save
	model.save();
}