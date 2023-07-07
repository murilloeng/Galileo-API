//std
#include <cmath>

//fea
#include "Model/Model.h"

#include "Mesh/Mesh.h"
#include "Mesh/Nodes/Dof.h"
#include "Mesh/Cells/Types.h"
#include "Mesh/Sections/Round.h"
#include "Mesh/Sections/Types.h"
#include "Mesh/Elements/Types.h"
#include "Mesh/Cells/Line/Line.h"
#include "Mesh/Materials/Types.h"
#include "Mesh/Materials/Mechanic/Steel.h"
#include "Mesh/Elements/Mechanic/Frame/Bar.h"

#include "Boundary/Boundary.h"

#include "Analysis/Analysis.h"
#include "Analysis/Solvers/Types.h"
#include "Analysis/Solvers/Buckling.h"

//ben
#include "benchmarks/mechanic/cable.h"

void tests::cable::buckling::cable(void)
{
	//parameters
	const unsigned n = 50;
	const double l = 2.0e+1;
	const double h = 6.0e+0;
	const double A = 5.0e-5;
	const double E = 1.65e11;
	const double d = sqrt(4 * A / M_PI);

	//model
	fea::models::Model model("cable", "benchmarks/cable/buckling");

	//nodes
	for(unsigned i = 0; i <= n; i++)
	{
		const double t = double(i) / n;
		model.mesh()->add_node(l * t, 4 * h * t * (t - 1), 0);
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
		model.mesh()->add_element(fea::mesh::elements::type::bar3, {i, i + 1});
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

	//solver
	model.analysis()->solver(fea::analysis::solvers::type::buckling);
	dynamic_cast<fea::analysis::solvers::Buckling*>(model.analysis()->solver())->spectre(fea::analysis::solvers::spectre::full);
	dynamic_cast<fea::analysis::solvers::Buckling*>(model.analysis()->solver())->watch_dof(n / 2, fea::mesh::nodes::dof::translation_1);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}