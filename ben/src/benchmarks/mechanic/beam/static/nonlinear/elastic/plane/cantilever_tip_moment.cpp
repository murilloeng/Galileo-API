//std
#include <cmath>

//fea
#include "fea/inc/Model/Model.h"

#include "fea/inc/Mesh/Mesh.h"
#include "fea/inc/Mesh/Nodes/Dof.h"
#include "fea/inc/Mesh/Cells/Types.h"
#include "fea/inc/Mesh/Sections/Box.h"
#include "fea/inc/Mesh/Sections/Types.h"
#include "fea/inc/Mesh/Elements/Types.h"
#include "fea/inc/Mesh/Cells/Line/Line.h"
#include "fea/inc/Mesh/Cells/Line/Beam.h"
#include "fea/inc/Mesh/Materials/Types.h"
#include "fea/inc/Mesh/Materials/Mechanic/Steel.h"
#include "fea/inc/Mesh/Elements/Mechanic/Mechanic.h"

#include "fea/inc/Boundary/Boundary.h"

#include "fea/inc/Analysis/Analysis.h"
#include "fea/inc/Analysis/Solvers/Types.h"
#include "fea/inc/Analysis/Strategies/Types.h"
#include "fea/inc/Analysis/Solvers/Static_Nonlinear.h"

//ben
#include "ben/inc/benchmarks/mechanic/beam.h"

void tests::beam::static_nonlinear::elastic::plane::cantilever_tip_moment(void)
{
	/*
	Cantilever beam subjected to a tip moment
	Analytic:
		t = q / L
		q = M L / EI
		x = L sin(q x0 / L) / q
		y = L (1 - cos(q x0 / L)) / q
	 */

	//model
	fea::models::Model model("cantilever tip moment", "benchmarks/beam/static/nonlinear/elastic/plane");

	//parameters
	const double L = 4.00e+00;
	const double w = 2.00e-01;
	const double h = 4.00e-01;
	const double t = 6.00e-03;
	const double E = 2.00e+11;

	//nodes
	model.mesh()->add_node(0, 0, 0);
	model.mesh()->add_node(L, 0, 0);

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::beam);

	//sections
	model.mesh()->add_section(fea::mesh::sections::type::box);
	((fea::mesh::sections::Box*) model.mesh()->section(0))->width(w);
	((fea::mesh::sections::Box*) model.mesh()->section(0))->height(h);
	((fea::mesh::sections::Box*) model.mesh()->section(0))->thickness(t);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->elastic_modulus(E);

	//elements
	model.mesh()->add_element(fea::mesh::elements::type::beam2C, {0, 1});

	//supports
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_3);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_2);

	//loads
	model.boundary()->add_load_set();
	model.mesh()->section(0)->prepare();
	const double I = model.mesh()->section(0)->inertia(1, 1);
	model.boundary()->add_load_case(1, fea::mesh::nodes::dof::rotation_3, 2 * M_PI * E * I / L);

	//solver
	fea::mesh::cells::Line::refine(0, 10);
	fea::mesh::elements::Mechanic::geometric(true);
	model.analysis()->solver(fea::analysis::solvers::type::static_nonlinear);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_set(0);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_max(3);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->step_max(300);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_guess(0.01);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->watch_dof(1, fea::mesh::nodes::dof::rotation_3);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->strategy(fea::analysis::strategies::type::control_load);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}