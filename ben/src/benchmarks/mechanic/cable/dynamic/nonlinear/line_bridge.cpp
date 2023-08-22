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

#include "fea/inc/Analysis/Analysis.h"
#include "fea/inc/Analysis/Solvers/Types.h"
#include "fea/inc/Analysis/Solvers/Dynamic_Nonlinear.h"

//ben
#include "ben/inc/benchmarks/mechanic/cable.h"

void tests::cable::dynamic_nonlinear::line_bridge(void)
{
	//model
	fea::models::Model model("line bridge", "benchmarks/cable/dynamic/nonlinear");

	//parameters
	const unsigned n = 40;
	const double q = 5.00e+0;
	const double h = 0.00e+1;
	const double w = 2.00e+1;
	const double v = 6.00e+0;
	const double A = 5.00e-5;
	const double E = 1.65e+8;
	const double d = sqrt(4 * A / M_PI);

	//nodes
	for(unsigned i = 0; i <= n; i++)
	{
		const double t = double(i) / n;
		model.mesh()->add_node(w * t, 4 * v * t * (t - 1), 0);
	}

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::bar);

	//sections
	model.mesh()->add_section(fea::mesh::sections::type::round);
	((fea::mesh::sections::Round*) model.mesh()->section(0))->diameter(d);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->elastic_modulus(E);

	//elements
	for(unsigned i = 0; i < n; i++)
	{
		model.mesh()->add_element(fea::mesh::elements::type::bar2, {i, i + 1});
		((fea::mesh::elements::Bar*) model.mesh()->element(i))->cable(true);
	}

	//supports
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(n, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(n, fea::mesh::nodes::dof::translation_2);
	for(unsigned i = 0; i <= n; i++)
	{
		model.boundary()->add_support(i, fea::mesh::nodes::dof::translation_3);
	}

	//loads
	model.boundary()->add_load_case();
	for(unsigned i = 0; i < n; i++)
	{
		model.boundary()->load_case(0)->add_load_node(i + 0, fea::mesh::nodes::dof::translation_2, -q * w / n / 2);
		model.boundary()->load_case(0)->add_load_node(i + 1, fea::mesh::nodes::dof::translation_2, -q * w / n / 2);
	}

	//solver
	fea::mesh::elements::Mechanic::geometric(true);
	model.analysis()->solver(fea::analysis::solvers::type::dynamic_nonlinear);
	dynamic_cast<fea::analysis::solvers::Dynamic_Nonlinear*> (model.analysis()->solver())->time_max(30);
	dynamic_cast<fea::analysis::solvers::Dynamic_Nonlinear*> (model.analysis()->solver())->step_max(20000);
	dynamic_cast<fea::analysis::solvers::Dynamic_Nonlinear*> (model.analysis()->solver())->watch_dof(n / 2, fea::mesh::nodes::dof::translation_2);
//	dynamic_cast<fea::analysis::solvers::Dynamic_Nonlinear*> (model.analysis()->solver())->integration(fea::analysis::solvers::integration::newmark);
	dynamic_cast<fea::analysis::solvers::Dynamic_Nonlinear*> (model.analysis()->solver())->integration(fea::analysis::solvers::integration::runge_kutta);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}