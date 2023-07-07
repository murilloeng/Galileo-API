//std
#include <cmath>

//fea
#include "Model/Model.h"

#include "Mesh/Mesh.h"
#include "Mesh/Nodes/Dof.h"
#include "Mesh/Cells/Types.h"
#include "Mesh/Sections/Types.h"
#include "Mesh/Elements/Types.h"
#include "Mesh/Materials/Types.h"
#include "Mesh/Sections/Rectangle.h"
#include "Mesh/Materials/Mechanic/Steel.h"
#include "Mesh/Elements/Mechanic/Mechanic.h"

#include "Boundary/Boundary.h"

#include "Analysis/Analysis.h"
#include "Analysis/Solvers/Types.h"
#include "Analysis/Solvers/Buckling.h"

//ben
#include "benchmarks/mechanic/bar.h"

//buckling
void tests::bar::buckling::single_bar(void)
{
	//parameters
	const double l = 1;
	const unsigned n = 10;

	//model
	fea::models::Model model("single bar", "benchmarks/bar/buckling");

	//nodes
	for(unsigned i = 0; i <= n; i++)
	{
		model.mesh()->add_node(l * i, 0, 0);
	}

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::bar);

	//sections
	model.mesh()->add_section(fea::mesh::sections::type::rectangle);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);

	//elements
	for(unsigned i = 0; i < n; i++)
	{
		model.mesh()->add_element(fea::mesh::elements::type::bar2, {i, i + 1});
	}

	//supports
	for(unsigned i = 0; i <= n; i++)
	{
		model.boundary()->add_support(i, fea::mesh::nodes::dof::translation_2);
	}

	//solver
	model.analysis()->solver(fea::analysis::solvers::type::buckling);
	model.analysis()->solver()->watch_dof(n, fea::mesh::nodes::dof::translation_1);
	dynamic_cast<fea::analysis::solvers::Buckling*>(model.analysis()->solver())->spectre(fea::analysis::solvers::spectre::full);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}