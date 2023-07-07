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
#include "Boundary/Loads/Load_Case.h"

#include "Analysis/Analysis.h"
#include "Analysis/Solvers/Types.h"
#include "Analysis/Strategies/Types.h"
#include "Analysis/Solvers/Static_Nonlinear.h"

//ben
#include "benchmarks/mechanic/beam.h"

void tests::beam::static_nonlinear::elastic::plane::arc_shallow_pinned(void)
{
	/*
	Pinned shallow arc subjected to a compression vertical load
	Literature: UFOP Phd Thesis A. R. D. Silva (2009) pp. 130
	 */

	//data
	const unsigned ne = 10;
	const double h = 5.00e0;
	const double L = 5.00e1;
	const unsigned nn = ne + 1;

	//model
	fea::models::Model model("arc shallow pinned", "benchmarks/beam/static/nonlinear/elastic/plane");

	//nodes
	for(unsigned i = 0; i < nn; i++)
	{
		const double x = 2 * L * i / (nn - 1) - L;
		const double y = h * (1 - pow(x / L, 2));
		model.mesh()->add_node(x, y, 0);
	}

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::beam);

	//sections
	model.mesh()->add_section(fea::mesh::sections::type::rectangle);
	((fea::mesh::sections::Rectangle*) model.mesh()->section(0))->width(0.288675);
	((fea::mesh::sections::Rectangle*) model.mesh()->section(0))->height(3.46410);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->elastic_modulus(2000);

	//elements
	for(unsigned i = 0; i < ne; i++)
	{
		model.mesh()->add_element(fea::mesh::elements::type::beam2C, {i, i + 1});
	}

	//supports
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(nn - 1, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(nn - 1, fea::mesh::nodes::dof::translation_2);

	//loads
	model.boundary()->add_load_set();
	model.boundary()->add_load_case();
	model.boundary()->load_case(0)->add_load_node((nn - 1) / 2, fea::mesh::nodes::dof::rotation_3, 2);
	model.boundary()->load_case(0)->add_load_node((nn - 1) / 2, fea::mesh::nodes::dof::translation_2, -1);

	//solver
	fea::mesh::elements::Mechanic::geometric(true);
	model.analysis()->solver(fea::analysis::solvers::type::static_nonlinear);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_set(0);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->step_max(400);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_max(1.4);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_adjust(false);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_guess(0.1);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->iteration_desired(2);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->watch_dof((nn - 1) / 2, fea::mesh::nodes::dof::translation_2);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->strategy(fea::analysis::strategies::type::arc_length_cylindric);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}