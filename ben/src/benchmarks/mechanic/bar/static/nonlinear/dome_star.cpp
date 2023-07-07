//std
#include <cmath>

//fea
#include "Model/Model.h"

#include "Mesh/Mesh.h"
#include "Mesh/Nodes/Dof.h"
#include "Mesh/Cells/Types.h"
#include "Mesh/Sections/Ring.h"
#include "Mesh/Sections/Types.h"
#include "Mesh/Elements/Types.h"
#include "Mesh/Materials/Types.h"
#include "Mesh/Materials/Mechanic/Steel.h"
#include "Mesh/Elements/Mechanic/Mechanic.h"

#include "Boundary/Boundary.h"
#include "Boundary/Loads/Load_Case.h"

#include "Analysis/Analysis.h"
#include "Analysis/Solvers/Types.h"
#include "Analysis/Strategies/Types.h"
#include "Analysis/Solvers/Static_Nonlinear.h"

//ben
#include "benchmarks/mechanic/bar.h"

void tests::bar::static_nonlinear::dome_star(void)
{
	//model
	fea::models::Model model("dome star", "benchmarks/bar/static/nonlinear");

	//parameters
	const unsigned n = 6;
	const double d = 1.00e-1;
	const double t = 1.00e-2;
	const double E = 2.00e11;
	const double r = 2.00e+0;
	const double R = 4.00e+0;
	const double h = 2.00e+0;
	const double H = 3.00e+0;

	//nodes
	model.mesh()->add_node(0, 0, H);
	for(unsigned i = 0; i < n; i++)
	{
		const double t = 2 * M_PI * i / n;
		model.mesh()->add_node(r * cos(t), r * sin(t), h);
	}
	for(unsigned i = 0; i < n; i++)
	{
		const double t = 2 * M_PI * i / n + M_PI / n;
		model.mesh()->add_node(R * cos(t), R * sin(t), 0);
	}

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::bar);

	//sections
	model.mesh()->add_section(fea::mesh::sections::type::ring);
	((fea::mesh::sections::Ring*) model.mesh()->section(0))->diameter(d);
	((fea::mesh::sections::Ring*) model.mesh()->section(0))->thickness(t);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->elastic_modulus(E);

	//elements
	for(unsigned i = 0; i < n; i++)
	{
		model.mesh()->add_element(fea::mesh::elements::type::bar3, {0, i + 1});
		model.mesh()->add_element(fea::mesh::elements::type::bar3, {i + 1, i + n + 1});
		model.mesh()->add_element(fea::mesh::elements::type::bar3, {i + 1, i + 1 != n ? i + 2 : 1});
		model.mesh()->add_element(fea::mesh::elements::type::bar3, {i + 1, i != 0 ? i + n : 2 * n});
	}

	//dependencies
	for(unsigned i = 0; i < n; i++)
	{
		const double t = 2 * M_PI * i / n;
		model.boundary()->add_dependency(i + 1, fea::mesh::nodes::dof::translation_3, 1, fea::mesh::nodes::dof::translation_3);
		model.boundary()->add_dependency(i + 1, fea::mesh::nodes::dof::translation_1, 1, fea::mesh::nodes::dof::translation_1, cos(t));
		model.boundary()->add_dependency(i + 1, fea::mesh::nodes::dof::translation_2, 1, fea::mesh::nodes::dof::translation_1, sin(t));
	}
	model.boundary()->remove_dependency(0);
	model.boundary()->remove_dependency(0);

	//supports
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_2);
	for(unsigned i = n + 1; i < 2 * n + 1; i++)
	{
		model.boundary()->add_support(i, fea::mesh::nodes::dof::translation_1);
		model.boundary()->add_support(i, fea::mesh::nodes::dof::translation_2);
		model.boundary()->add_support(i, fea::mesh::nodes::dof::translation_3);
	}

	//loads
	model.boundary()->add_load_case(0, fea::mesh::nodes::dof::translation_3, -1e6);

	//solver
	fea::mesh::elements::Mechanic::geometric(true);
	fea::mesh::elements::Mechanic::inelastic(false);
	model.analysis()->solver(fea::analysis::solvers::type::static_nonlinear);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_max(200);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->step_max(2000);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->dof_min(-2 * H);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->frequencies(true);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_guess(1.50);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->spectre(fea::analysis::solvers::spectre::full);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->watch_dof(0, fea::mesh::nodes::dof::translation_3);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->strategy(fea::analysis::strategies::type::arc_length_cylindric);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}