//std
#include <cmath>
#include <ctime>

//mat
#include "linear/dense.h"

//fea
#include "Model/Model.h"

#include "Mesh/Mesh.h"
#include "Mesh/Nodes/Dof.h"
#include "Mesh/Cells/Types.h"
#include "Mesh/Sections/Types.h"
#include "Mesh/Elements/Types.h"
#include "Mesh/Cells/Line/Line.h"
#include "Mesh/Materials/Types.h"
#include "Mesh/Elements/Mechanic/Mechanic.h"

#include "Boundary/Boundary.h"
#include "Boundary/Supports/Support.h"

#include "Analysis/Analysis.h"
#include "Analysis/Solvers/Types.h"
#include "Analysis/Solvers/State.h"

//ben
#include "benchmarks/mechanic/beam.h"

void tests::beam::state::cantilever_tip_moment(void)
{
	//data
	const double L = 1;
	const unsigned n = 2;

	//model
	fea::models::Model model("cantilever tip moment", "benchmarks/beam/state");

	//nodes
	for(unsigned i = 0; i <= n; i++)
	{
		model.mesh()->add_node(i * L / n, 0, 0);
	}

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::beam);

	//sections
	model.mesh()->add_section(fea::mesh::sections::type::rectangle);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);

	//elements
	for(unsigned i = 0; i < n; i++)
	{
		model.mesh()->add_element(fea::mesh::elements::type::beam3T, {i, i + 1});
	}

	//supports
	for(unsigned i = 0; i <= n; i++)
	{
		model.boundary()->add_support(i, fea::mesh::nodes::dof::rotation_1);
		model.boundary()->add_support(i, fea::mesh::nodes::dof::rotation_2);
		model.boundary()->add_support(i, fea::mesh::nodes::dof::rotation_3);
		model.boundary()->add_support(i, fea::mesh::nodes::dof::translation_1);
		model.boundary()->add_support(i, fea::mesh::nodes::dof::translation_2);
		model.boundary()->add_support(i, fea::mesh::nodes::dof::translation_3);
	}

	//kinematics
	for(unsigned i = 0; i <= n; i++)
	{
		const double s = i * L / n;
		// model.boundary()->support(6 * i + 2)->state([s] (double t) { t *= 2 * M_PI * s; return t; });
		// model.boundary()->support(6 * i + 3)->state([s] (double t) { t *= 2 * M_PI * s; return t ? s * (sin(t) / t - 1) : 0; });
		// model.boundary()->support(6 * i + 4)->state([s] (double t) { t *= 2 * M_PI * s; return t ? s * (1 - cos(t)) / t : 0; });
	}

	//solver
	fea::mesh::elements::Mechanic::geometric(true);
	model.analysis()->solver(fea::analysis::solvers::type::state)->watch_dof(n, fea::mesh::nodes::dof::rotation_3);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}