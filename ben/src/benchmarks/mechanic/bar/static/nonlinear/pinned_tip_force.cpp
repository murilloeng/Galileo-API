//std
#include <cmath>

//fea
#include "fea/inc/Model/Model.h"

#include "fea/inc/Mesh/Mesh.h"
#include "fea/inc/Mesh/Nodes/Dof.h"
#include "fea/inc/Mesh/Cells/Types.h"
#include "fea/inc/Mesh/Sections/Types.h"
#include "fea/inc/Mesh/Elements/Types.h"
#include "fea/inc/Mesh/Materials/Types.h"
#include "fea/inc/Mesh/Sections/Rectangle.h"
#include "fea/inc/Mesh/Materials/Mechanic/Steel.h"
#include "fea/inc/Mesh/Elements/Mechanic/Mechanic.h"

#include "fea/inc/Boundary/Boundary.h"
#include "fea/inc/Boundary/Loads/Load_Case.h"

#include "fea/inc/Analysis/Analysis.h"
#include "fea/inc/Analysis/Solvers/Types.h"
#include "fea/inc/Analysis/Strategies/Types.h"
#include "fea/inc/Analysis/Solvers/Static_Nonlinear.h"

//ben
#include "ben/inc/benchmarks/mechanic/bar.h"

void tests::bar::static_nonlinear::pinned_tip_force(void)
{
	//model
	fea::models::Model model("pinned tip force", "benchmarks/bar/static/nonlinear");

	//parameters
	const double d = 1.00e-1;
	const double l = 1.00e+0;
	const double E = 2.00e11;

	const double A = d * d;
	const double P = E * A;

	//nodes
	model.mesh()->add_node(0, 0, 0);
	model.mesh()->add_node(l, 0, 0);

	//sections
	model.mesh()->add_section(fea::mesh::sections::type::rectangle);
	((fea::mesh::sections::Rectangle*) model.mesh()->section(0))->width(d);
	((fea::mesh::sections::Rectangle*) model.mesh()->section(0))->height(d);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->elastic_modulus(E);

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::bar);

	//elements
	fea::mesh::elements::Mechanic::geometric(true);
	model.mesh()->add_element(fea::mesh::elements::type::bar2, {0, 1});

	//supports
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_3);
	model.boundary()->add_support(1, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(1, fea::mesh::nodes::dof::translation_3);

	//loads
	model.boundary()->add_load_case(1, fea::mesh::nodes::dof::translation_1, P);

	//solver
	model.analysis()->solver(fea::analysis::solvers::type::static_nonlinear);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->step_max(200);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_max(3.0);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_guess(0.01);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_increment_max(0.05);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->watch_dof(1, fea::mesh::nodes::dof::translation_1);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->strategy(fea::analysis::strategies::type::arc_length_cylindric);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}