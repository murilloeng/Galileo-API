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

#include "fea/inc/Analysis/Analysis.h"
#include "fea/inc/Analysis/Solvers/Types.h"
#include "fea/inc/Analysis/Strategies/Types.h"
#include "fea/inc/Analysis/Solvers/Static_Nonlinear.h"

//ben
#include "ben/inc/benchmarks/mechanic/beam.h"

void tests::beam::static_nonlinear::elastic::space::arc_cantilever(void)
{
	/*
	Arc cantilever bend subjected to tip force
	Literature: Phd Thesis P. Carotenuto (2006) pp. 95
	 */

	//data
	const double r = 1.00e+01;
	const double w = 1.00e-01;
	const double h = 1.00e-01;
	const double v = 0.00e+00;
	const double E = 1.00e+07;

	const unsigned ne = 8;
	const unsigned nn = ne + 1;

	//model
	fea::models::Model model("arc cantilever", "benchmarks/beam/static/nonlinear/elastic/space");

	//nodes
	for(unsigned i = 0; i < nn; i++)
	{
		const double t = M_PI * i / 4 / ne;
		model.mesh()->add_node(r * cos(t), r * sin(t), 0);
	}

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
	for(unsigned i = 0; i < ne; i++)
	{
		model.mesh()->add_element(fea::mesh::elements::type::beam3T, {i, i + 1});
	}

	//supports
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_3);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_3);

	//loads
	model.boundary()->add_load_set();
	model.boundary()->add_load_case(ne, fea::mesh::nodes::dof::translation_3, 50);

	//solver
	fea::mesh::elements::Mechanic::geometric(true);
	model.analysis()->solver(fea::analysis::solvers::type::static_nonlinear);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_set(0);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->step_max(60);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_max(1.0);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_guess(0.0025);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->strategy(fea::analysis::strategies::type::minimal_norm);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->watch_dof(nn - 1, fea::mesh::nodes::dof::translation_3);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}