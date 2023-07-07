//std
#include <cmath>

//fea
#include "Model/Model.h"

#include "Mesh/Mesh.h"
#include "Mesh/Nodes/Dof.h"
#include "Mesh/Cells/Types.h"
#include "Mesh/Sections/Types.h"
#include "Mesh/Elements/Types.h"
#include "Mesh/Cells/Line/Line.h"
#include "Mesh/Materials/Types.h"
#include "Mesh/Sections/Rectangle.h"
#include "Mesh/Materials/Mechanic/Steel.h"
#include "Mesh/Elements/Mechanic/Mechanic.h"
#include "Mesh/Elements/Mechanic/Frame/Beam.h"

#include "Boundary/Boundary.h"

#include "Analysis/Analysis.h"
#include "Analysis/Solvers/Types.h"
#include "Analysis/Strategies/Types.h"
#include "Analysis/Solvers/Static_Nonlinear.h"

//ben
#include "benchmarks/mechanic/beam.h"

void tests::beam::static_nonlinear::elastic::space::dome_framed(void)
{
	/*
	Framed dome
	Literature: Phd Thesis J-M. Battini (2002) pp. 146
	 */

	//data
	const unsigned n = 6;
	const unsigned m = 8;
	const double w = 7.600e-01;
	const double h = 1.220e+00;
	const double E = 2.069e+04;
	const double v = 1.716e-01;
	const double r1 = 1.257e+01;
	const double r2 = 2.438e+01;
	const double h1 = 1.550e+00;
	const double h2 = 4.550e+00;

	//model
	fea::models::Model model("dome framed", "benchmarks/beam/static/nonlinear/elastic/space");

	//nodes
	for(unsigned i = 0; i < n; i++)
	{
		const double t = 2 * M_PI * i / n;
		const double q = 2 * M_PI * i / n + 1 * M_PI / 5000;
		model.mesh()->add_node(r2 * cos(t), r2 * sin(t), 0);
		model.mesh()->add_node(r1 * cos(q), r1 * sin(q), h2);
	}
	model.mesh()->add_node(0, 0, h1 + h2);

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::beam);

	//sections
	model.mesh()->add_section(fea::mesh::sections::type::rectangle);
	((fea::mesh::sections::Rectangle*) model.mesh()->section(0))->width(w);
	((fea::mesh::sections::Rectangle*) model.mesh()->section(0))->height(h);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->poisson_ratio(v);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->elastic_modulus(E);

	//elements
	for(unsigned i = 0; i < n; i++)
	{
		model.mesh()->add_element(fea::mesh::elements::type::beam3T, {2 * i + 1, 2 * n});
		model.mesh()->add_element(fea::mesh::elements::type::beam3T, {2 * i + 0, 2 * i + 1});
		model.mesh()->add_element(fea::mesh::elements::type::beam3T, {2 * i + 1, (2 * i + 3) % (2 * n)});
	}

	//orientation
	for(unsigned i = 0; i < n; i++)
	{
		const double t = 2 * M_PI * i / n + M_PI / 2;
		const double q = 2 * M_PI * i / n + M_PI / n;
		((fea::mesh::elements::Beam*) model.mesh()->element(3 * i + 0))->orientation(cos(t), sin(t), 0);
		((fea::mesh::elements::Beam*) model.mesh()->element(3 * i + 1))->orientation(cos(t), sin(t), 0);
		((fea::mesh::elements::Beam*) model.mesh()->element(3 * i + 2))->orientation(cos(q), sin(q), 0);
	}

	//refine
	for(unsigned i = 0; i < 3 * n; i++)
	{
		fea::mesh::cells::Line::refine(i, m);
	}

	//supports
	for(unsigned i = 0; i < n; i++)
	{
		model.boundary()->add_support(2 * i, fea::mesh::nodes::dof::rotation_1);
		model.boundary()->add_support(2 * i, fea::mesh::nodes::dof::rotation_2);
		model.boundary()->add_support(2 * i, fea::mesh::nodes::dof::rotation_3);
		model.boundary()->add_support(2 * i, fea::mesh::nodes::dof::translation_1);
		model.boundary()->add_support(2 * i, fea::mesh::nodes::dof::translation_2);
		model.boundary()->add_support(2 * i, fea::mesh::nodes::dof::translation_3);
	}

	//loads
	model.boundary()->add_load_set();
	model.boundary()->add_load_case(2 * n, fea::mesh::nodes::dof::translation_3, -1);

	//solver
	fea::mesh::elements::Mechanic::geometric(true);
	model.analysis()->solver(fea::analysis::solvers::type::static_nonlinear);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_set(0);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_guess(2);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_max(200);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->step_max(2000);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->watch_dof(2 * n, fea::mesh::nodes::dof::translation_3);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->strategy(fea::analysis::strategies::type::control_state);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}