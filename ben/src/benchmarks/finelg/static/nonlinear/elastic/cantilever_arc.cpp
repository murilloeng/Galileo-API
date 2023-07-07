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

#include "Boundary/Boundary.h"

#include "Analysis/Analysis.h"
#include "Analysis/Solvers/Types.h"
#include "Analysis/Strategies/Types.h"
#include "Analysis/Solvers/Static_Nonlinear.h"

//ben
#include "benchmarks/finelg/finelg.h"

void tests::finelg::static_nonlinear::elastic::cantilever_arc(void)
{
	/*
	Cantilever arc subjected to tip force
	Literature: Phd Thesis V. V. Goyet (1989) pp. 8.29
	 */

	//data
	const double r = 1.00e+02;
	const double w = 1.00e+00;
	const double h = 1.00e+00;
	const double v = 0.00e+00;
	const double E = 1.00e+07;

	const unsigned ne = 8;
	const unsigned nn = ne + 1;

	//model
	fea::models::Model model("cantilever arc", "benchmarks/finelg/static/nonlinear/elastic");

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
		model.mesh()->add_element(fea::mesh::elements::type::beam3C, {i, i + 1});
	}

	//supports
	model.boundary()->add_support(0, fea::mesh::nodes::dof::warping_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_3);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_3);

	//loads
	model.boundary()->add_load_case(ne, fea::mesh::nodes::dof::translation_3, 600);

	//solver
	fea::mesh::elements::Mechanic::geometric(true);
	model.analysis()->solver(fea::analysis::solvers::type::static_nonlinear);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->step_max(100);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_max(1.0);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_guess(0.01);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->watch_dof(ne, fea::mesh::nodes::dof::translation_3);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->strategy(fea::analysis::strategies::type::control_load);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}