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
#include "fea/inc/Mesh/Sections/Rectangle.h"
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
#include "ben/inc/benchmarks/mechanic/deployable.h"

void tests::deployable::static_nonlinear::slut_mult(void)
{
	//model
	fea::models::Model model("slut mult", "benchmarks/deployable/static/nonlinear");

	//parameters
	const double l = 1.00e+00;
	const double h = 4.00e-02;
	const double t = 1.00e-02;
	const double s = 1.00e-02;
	const double e = 5.00e-02;
	const double k = 1.00e+00;
	const double a = M_PI / 2;
	const double d = (t + s) / 2;

	const unsigned n = 10;

	//nodes
	model.mesh()->add_node(0, 0, 0 * l * sin(a));
	model.mesh()->add_node(0, 0, 2 * l * sin(a));
	for(unsigned i = 0; i < n; i++)
	{
		const double q = 2 * i * M_PI / n;
		model.mesh()->add_node(e * cos(q), e * sin(q), 0 * l * sin(a));
		model.mesh()->add_node(e * cos(q), e * sin(q), 2 * l * sin(a));
		model.mesh()->add_node((e + 0 * l * cos(a)) * cos(q) + d * sin(q), (e + 0 * l * cos(a)) * sin(q) - d * cos(q), 0 * l * sin(a));
		model.mesh()->add_node((e + 1 * l * cos(a)) * cos(q) + d * sin(q), (e + 1 * l * cos(a)) * sin(q) - d * cos(q), 1 * l * sin(a));
		model.mesh()->add_node((e + 2 * l * cos(a)) * cos(q) + d * sin(q), (e + 2 * l * cos(a)) * sin(q) - d * cos(q), 2 * l * sin(a));
		model.mesh()->add_node((e + 2 * l * cos(a)) * cos(q) - d * sin(q), (e + 2 * l * cos(a)) * sin(q) + d * cos(q), 0 * l * sin(a));
		model.mesh()->add_node((e + 1 * l * cos(a)) * cos(q) - d * sin(q), (e + 1 * l * cos(a)) * sin(q) + d * cos(q), 1 * l * sin(a));
		model.mesh()->add_node((e + 0 * l * cos(a)) * cos(q) - d * sin(q), (e + 0 * l * cos(a)) * sin(q) + d * cos(q), 2 * l * sin(a));
	}

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::beam);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);

	//sections
	model.mesh()->add_section(fea::mesh::sections::type::rectangle);
	((fea::mesh::sections::Rectangle*) model.mesh()->section(0))->width(t);
	((fea::mesh::sections::Rectangle*) model.mesh()->section(0))->height(h);

	//elements
	for(unsigned i = 0; i < n; i++)
	{
		const double q = 2 * i * M_PI / n;
		model.mesh()->add_element(fea::mesh::elements::type::beam3C, {8 * i + 2, 0});
		model.mesh()->add_element(fea::mesh::elements::type::beam3C, {8 * i + 3, 1});
		model.mesh()->add_element(fea::mesh::elements::type::beam3C, {8 * i + 4, 8 * i + 5});
		model.mesh()->add_element(fea::mesh::elements::type::beam3C, {8 * i + 5, 8 * i + 6});
		model.mesh()->add_element(fea::mesh::elements::type::beam3C, {8 * i + 7, 8 * i + 8});
		model.mesh()->add_element(fea::mesh::elements::type::beam3C, {8 * i + 8, 8 * i + 9});
		((fea::mesh::elements::Beam*) model.mesh()->element(6 * i + 0))->orientation(+sin(q), -cos(q), 0);
		((fea::mesh::elements::Beam*) model.mesh()->element(6 * i + 1))->orientation(+sin(q), -cos(q), 0);
		((fea::mesh::elements::Beam*) model.mesh()->element(6 * i + 2))->orientation(+sin(q), -cos(q), 0);
		((fea::mesh::elements::Beam*) model.mesh()->element(6 * i + 3))->orientation(+sin(q), -cos(q), 0);
		((fea::mesh::elements::Beam*) model.mesh()->element(6 * i + 4))->orientation(+sin(q), -cos(q), 0);
		((fea::mesh::elements::Beam*) model.mesh()->element(6 * i + 5))->orientation(+sin(q), -cos(q), 0);
	}

	//joints
	for(unsigned i = 0; i < n; i++)
	{
		const double q = 2 * i * M_PI / n;
		model.mesh()->add_joint(fea::mesh::joints::type::hinge, {8 * i + 2, 8 * i + 4});
		model.mesh()->add_joint(fea::mesh::joints::type::hinge, {8 * i + 3, 8 * i + 9});
		model.mesh()->add_joint(fea::mesh::joints::type::hinge, {8 * i + 5, 8 * i + 8});
		((fea::mesh::joints::Hinge*) model.mesh()->joint(3 * i + 2))->stiffness(0, 2, k);
		for(unsigned j = 0; j < 3; j++)
		{
			((fea::mesh::joints::Hinge*) model.mesh()->joint(3 * i + j))->axis(+sin(q), -cos(q), 0);
			((fea::mesh::joints::Hinge*) model.mesh()->joint(3 * i + j))->orientation(+cos(q), +sin(q), 0);
		}
	}

	//supports
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_3);
	model.boundary()->add_support(1, fea::mesh::nodes::dof::rotation_1);
	model.boundary()->add_support(1, fea::mesh::nodes::dof::rotation_2);
	model.boundary()->add_support(1, fea::mesh::nodes::dof::rotation_3);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_3);
	model.boundary()->add_support(1, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(1, fea::mesh::nodes::dof::translation_2);

	//loads
	model.boundary()->add_load_case();
	for(unsigned i = 0; i < n; i++)
	{
		const double q = 2 * i * M_PI / n;
		model.boundary()->load_case(0)->add_load_node(8 * i + 7, fea::mesh::nodes::dof::translation_1, k / l * cos(q));
		model.boundary()->load_case(0)->add_load_node(8 * i + 7, fea::mesh::nodes::dof::translation_2, k / l * sin(q));
	}

	//solver
	fea::mesh::elements::Mechanic::geometric(true);
	model.analysis()->solver(fea::analysis::solvers::type::static_nonlinear);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->step_max(700);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_max(7e1);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->dof_max(M_PI / 2);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_adjust(true);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_guess(1e-2);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->iteration_desired(4);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->watch_dof(4, fea::mesh::nodes::dof::rotation_2);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->strategy(fea::analysis::strategies::type::minimal_norm);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}