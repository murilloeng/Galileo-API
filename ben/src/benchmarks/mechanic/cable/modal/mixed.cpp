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
#include "fea/inc/Mesh/Elements/Mechanic/Frame/Bar.h"

#include "fea/inc/Boundary/Boundary.h"

#include "fea/inc/Analysis/Analysis.h"
#include "fea/inc/Analysis/Solvers/Types.h"
#include "fea/inc/Analysis/Solvers/Modal.h"

//ben
#include "ben/inc/benchmarks/mechanic/cable.h"

void tests::cable::modal::mixed(void)
{
	//parameters
	const unsigned n = 50;
	const double b = 4.00e+00;
	const double H = 2.00e+01;
	const double l = 2.00e+01;
	const double h = 6.00e+00;
	const double E = 2.10e+11;
	const double A0 = 5.00e-05;
	const double A1 = 3.50e-03;
	const double A2 = 7.50e-04;
	const double d0 = sqrt(4 * A0 / M_PI);
	const double d1 = sqrt(4 * A1 / M_PI);
	const double d2 = sqrt(4 * A2 / M_PI);

	//model
	fea::models::Model model("mixed", "benchmarks/cable/modal");

	//nodes
	model.mesh()->add_node(0, 0, H);
	model.mesh()->add_node(-b / 2, -b / 2, 0);
	model.mesh()->add_node(+b / 2, -b / 2, 0);
	model.mesh()->add_node(+b / 2, +b / 2, 0);
	model.mesh()->add_node(-b / 2, +b / 2, 0);
	model.mesh()->add_node(-b / 4, -b / 4, H / 2);
	model.mesh()->add_node(+b / 4, -b / 4, H / 2);
	model.mesh()->add_node(+b / 4, +b / 4, H / 2);
	model.mesh()->add_node(-b / 4, +b / 4, H / 2);

	model.mesh()->add_node(l, 0, H);
	model.mesh()->add_node(l - b / 2, -b / 2, 0);
	model.mesh()->add_node(l + b / 2, -b / 2, 0);
	model.mesh()->add_node(l + b / 2, +b / 2, 0);
	model.mesh()->add_node(l - b / 2, +b / 2, 0);
	model.mesh()->add_node(l - b / 4, -b / 4, H / 2);
	model.mesh()->add_node(l + b / 4, -b / 4, H / 2);
	model.mesh()->add_node(l + b / 4, +b / 4, H / 2);
	model.mesh()->add_node(l - b / 4, +b / 4, H / 2);

	for(unsigned i = 0; i <= n; i++)
	{
		const double t = double(i) / n;
		model.mesh()->add_node(l * t, 0, H - 4 * h * t * (1 - t));
	}

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::bar);
	model.mesh()->add_cell(fea::mesh::cells::type::bar);
	model.mesh()->add_cell(fea::mesh::cells::type::bar);
	((fea::mesh::cells::Line*) model.mesh()->cell(0))->section(0);
	((fea::mesh::cells::Line*) model.mesh()->cell(1))->section(1);
	((fea::mesh::cells::Line*) model.mesh()->cell(2))->section(2);

	//sections
	model.mesh()->add_section(fea::mesh::sections::type::round);
	model.mesh()->add_section(fea::mesh::sections::type::round);
	model.mesh()->add_section(fea::mesh::sections::type::round);
	((fea::mesh::sections::Round*) model.mesh()->section(0))->diameter(d0);
	((fea::mesh::sections::Round*) model.mesh()->section(1))->diameter(d1);
	((fea::mesh::sections::Round*) model.mesh()->section(2))->diameter(d2);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->elastic_modulus(E);

	//elements
	for(unsigned i = 0; i < 2; i++)
	{
		model.mesh()->add_element(fea::mesh::elements::type::bar3, {9 * i + 0, 9 * i + 5}, 0, 1);
		model.mesh()->add_element(fea::mesh::elements::type::bar3, {9 * i + 0, 9 * i + 6}, 0, 1);
		model.mesh()->add_element(fea::mesh::elements::type::bar3, {9 * i + 0, 9 * i + 7}, 0, 1);
		model.mesh()->add_element(fea::mesh::elements::type::bar3, {9 * i + 0, 9 * i + 8}, 0, 1);
		model.mesh()->add_element(fea::mesh::elements::type::bar3, {9 * i + 5, 9 * i + 1}, 0, 1);
		model.mesh()->add_element(fea::mesh::elements::type::bar3, {9 * i + 6, 9 * i + 2}, 0, 1);
		model.mesh()->add_element(fea::mesh::elements::type::bar3, {9 * i + 7, 9 * i + 3}, 0, 1);
		model.mesh()->add_element(fea::mesh::elements::type::bar3, {9 * i + 8, 9 * i + 4}, 0, 1);
		model.mesh()->add_element(fea::mesh::elements::type::bar3, {9 * i + 5, 9 * i + 6}, 0, 2);
		model.mesh()->add_element(fea::mesh::elements::type::bar3, {9 * i + 6, 9 * i + 7}, 0, 2);
		model.mesh()->add_element(fea::mesh::elements::type::bar3, {9 * i + 7, 9 * i + 8}, 0, 2);
		model.mesh()->add_element(fea::mesh::elements::type::bar3, {9 * i + 8, 9 * i + 5}, 0, 2);
		model.mesh()->add_element(fea::mesh::elements::type::bar3, {9 * i + 5, 9 * i + 2}, 0, 2);
		model.mesh()->add_element(fea::mesh::elements::type::bar3, {9 * i + 6, 9 * i + 1}, 0, 2);
		model.mesh()->add_element(fea::mesh::elements::type::bar3, {9 * i + 6, 9 * i + 3}, 0, 2);
		model.mesh()->add_element(fea::mesh::elements::type::bar3, {9 * i + 7, 9 * i + 2}, 0, 2);
		model.mesh()->add_element(fea::mesh::elements::type::bar3, {9 * i + 7, 9 * i + 4}, 0, 2);
		model.mesh()->add_element(fea::mesh::elements::type::bar3, {9 * i + 8, 9 * i + 3}, 0, 2);
		model.mesh()->add_element(fea::mesh::elements::type::bar3, {9 * i + 8, 9 * i + 1}, 0, 2);
		model.mesh()->add_element(fea::mesh::elements::type::bar3, {9 * i + 5, 9 * i + 4}, 0, 2);
	}
	for(unsigned i = 0; i < n; i++)
	{
		model.mesh()->add_element(fea::mesh::elements::type::bar3, {18 + i, 19 + i}, 0, 0);
	}

	//supports
	for(unsigned i = 0; i < 2; i++)
	{
		for(unsigned j = 0; j < 4; j++)
		{
			model.boundary()->add_support(9 * i + j + 1, fea::mesh::nodes::dof::translation_1);
			model.boundary()->add_support(9 * i + j + 1, fea::mesh::nodes::dof::translation_2);
			model.boundary()->add_support(9 * i + j + 1, fea::mesh::nodes::dof::translation_3);
		}
	}
	for(unsigned i = 0; i < n; i++)
	{
		model.boundary()->add_support(18 + i, fea::mesh::nodes::dof::translation_2);
	}

	//dependencies
//	for(unsigned i = 0; i < 3; i++)
//	{
//		model.boundary()->add_dependency(0, fea::mesh::nodes::dof(1 << i), 18, fea::mesh::nodes::dof(1 << i));
//		model.boundary()->add_dependency(9, fea::mesh::nodes::dof(1 << i), 18 + n, fea::mesh::nodes::dof(1 << i));
//	}
	model.boundary()->add_dependency(0, fea::mesh::nodes::dof::translation_1, 18, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_dependency(0, fea::mesh::nodes::dof::translation_2, 18, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_dependency(0, fea::mesh::nodes::dof::translation_3, 18, fea::mesh::nodes::dof::translation_3);
	model.boundary()->add_dependency(9, fea::mesh::nodes::dof::translation_1, 18 + n, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_dependency(9, fea::mesh::nodes::dof::translation_2, 18 + n, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_dependency(9, fea::mesh::nodes::dof::translation_3, 18 + n, fea::mesh::nodes::dof::translation_3);

	//solver
	model.analysis()->solver(fea::analysis::solvers::type::modal);
//	dynamic_cast<fea::analysis::solvers::Modal*>(model.analysis()->solver())->tolerance(1e-4);
dynamic_cast<fea::analysis::solvers::Modal*>(model.analysis()->solver())->spectre(fea::analysis::solvers::spectre::full);
	dynamic_cast<fea::analysis::solvers::Modal*>(model.analysis()->solver())->watch_dof(18 + n / 2, fea::mesh::nodes::dof::translation_3);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}