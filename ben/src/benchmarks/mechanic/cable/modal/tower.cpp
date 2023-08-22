//std
#include <cmath>

//fea
#include "fea/inc/Model/Model.h"

#include "fea/inc/Mesh/Mesh.h"
#include "fea/inc/Mesh/Nodes/Dof.h"
#include "fea/inc/Mesh/Cells/Types.h"
#include "fea/inc/Mesh/Sections/Round.h"
#include "fea/inc/Mesh/Sections/Types.h"
#include "fea/inc/Mesh/Elements/Types.h"
#include "fea/inc/Mesh/Cells/Line/Line.h"
#include "fea/inc/Mesh/Materials/Types.h"
#include "fea/inc/Mesh/Materials/Mechanic/Steel.h"
#include "fea/inc/Mesh/Elements/Mechanic/Mechanic.h"
#include "fea/inc/Mesh/Elements/Mechanic/Frame/Bar.h"

#include "fea/inc/Boundary/Boundary.h"
#include "fea/inc/Boundary/Loads/Load_Case.h"
#include "fea/inc/Boundary/Dependencies/Dependency.h"

#include "fea/inc/Analysis/Analysis.h"
#include "fea/inc/Analysis/Solvers/Types.h"
#include "fea/inc/Analysis/Solvers/Modal.h"

//ben
#include "ben/inc/benchmarks/mechanic/cable.h"

void tests::cable::modal::tower(void)
{
	//parameters
	const double b = 4.00e+00;
	const double h = 2.00e+01;
	const double s = 2.50e+08;
	const double E = 2.10e+11;
	const double K = 2.10e+10;
	const double A1 = 3.50e-03;
	const double A2 = 7.50e-04;
	const double d1 = sqrt(4 * A1 / M_PI);
	const double d2 = sqrt(4 * A2 / M_PI);

	//model
	fea::models::Model model("tower", "benchmarks/cable/modal");

	//nodes
	model.mesh()->add_node(0, 0, h);
	model.mesh()->add_node(-b / 2, -b / 2, 0);
	model.mesh()->add_node(+b / 2, -b / 2, 0);
	model.mesh()->add_node(+b / 2, +b / 2, 0);
	model.mesh()->add_node(-b / 2, +b / 2, 0);
	model.mesh()->add_node(-b / 4, -b / 4, h / 2);
	model.mesh()->add_node(+b / 4, -b / 4, h / 2);
	model.mesh()->add_node(+b / 4, +b / 4, h / 2);
	model.mesh()->add_node(-b / 4, +b / 4, h / 2);

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::bar);
	model.mesh()->add_cell(fea::mesh::cells::type::bar);
	((fea::mesh::cells::Line*) model.mesh()->cell(0))->section(0);
	((fea::mesh::cells::Line*) model.mesh()->cell(1))->section(1);

	//sections
	model.mesh()->add_section(fea::mesh::sections::type::round);
	model.mesh()->add_section(fea::mesh::sections::type::round);
	((fea::mesh::sections::Round*) model.mesh()->section(0))->diameter(d1);
	((fea::mesh::sections::Round*) model.mesh()->section(1))->diameter(d2);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->yield_stress(s);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->break_strain(1e30);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->elastic_modulus(E);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->plastic_modulus(K);

	//elements
	model.mesh()->add_element(fea::mesh::elements::type::bar3, {0, 5}, 0, 0);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, {0, 6}, 0, 0);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, {0, 7}, 0, 0);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, {0, 8}, 0, 0);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, {5, 1}, 0, 0);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, {6, 2}, 0, 0);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, {7, 3}, 0, 0);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, {8, 4}, 0, 0);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, {5, 6}, 0, 1);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, {6, 7}, 0, 1);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, {7, 8}, 0, 1);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, {8, 5}, 0, 1);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, {5, 2}, 0, 1);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, {6, 1}, 0, 1);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, {6, 3}, 0, 1);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, {7, 2}, 0, 1);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, {7, 4}, 0, 1);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, {8, 3}, 0, 1);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, {8, 1}, 0, 1);
	model.mesh()->add_element(fea::mesh::elements::type::bar3, {5, 4}, 0, 1);

	//supports
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

	//solver
	model.analysis()->solver(fea::analysis::solvers::type::modal);
	dynamic_cast<fea::analysis::solvers::Modal*>(model.analysis()->solver())->scale(1e1);
	dynamic_cast<fea::analysis::solvers::Modal*>(model.analysis()->solver())->spectre(fea::analysis::solvers::spectre::full);
	dynamic_cast<fea::analysis::solvers::Modal*>(model.analysis()->solver())->watch_dof(0, fea::mesh::nodes::dof::translation_1);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}