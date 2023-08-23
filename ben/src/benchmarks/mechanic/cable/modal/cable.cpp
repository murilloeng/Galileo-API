//std
#include <cmath>

//fea
#include "fea/inc/Model/Model.h"

#include "fea/inc/Mesh/Mesh.h"
#include "fea/inc/Mesh/Nodes/Dof.h"
#include "fea/inc/Mesh/Cells/Types.h"
#include "fea/inc/Mesh/Sections/Round.h"
#include "fea/inc/Mesh/Sections/Types.h"
#include "fea/inc/Mesh/Elements/Types.h"
#include "fea/inc/Mesh/Cells/Line/Line.h"
#include "fea/inc/Mesh/Materials/Types.h"
#include "fea/inc/Mesh/Materials/Mechanic/Steel.h"
#include "fea/inc/Mesh/Elements/Mechanic/Mechanic.h"
#include "fea/inc/Mesh/Elements/Mechanic/Frame/Bar.h"

#include "fea/inc/Boundary/Boundary.h"
#include "fea/inc/Boundary/Loads/Load_Case.h"
#include "fea/inc/Boundary/Dependencies/Dependency.h"

#include "fea/inc/Analysis/Analysis.h"
#include "fea/inc/Analysis/Solvers/Types.h"
#include "fea/inc/Analysis/Solvers/Modal.h"

//ben
#include "ben/inc/benchmarks/mechanic/cable.h"

void tests::cable::modal::cable(void)
{
	//parameters
	const unsigned n = 50;
	const double l = 2.0e+1;
	const double h = 6.0e+0;
	const double A = 5.0e-5;
	const double E = 1.65e11;
	const double d = sqrt(4 * A / M_PI);

	//model
	fea::models::Model model("cable", "benchmarks/cable/modal");

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
		model.mesh()->add_element(fea::mesh::elements::type::bar2, {i, i + 1});
		((fea::mesh::elements::Bar*) model.mesh()->element(i))->cable(true);
	}

	//supports
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_3);
	model.boundary()->add_support(n, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(n, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(n, fea::mesh::nodes::dof::translation_3);

	//solver
	model.analysis()->solver(fea::analysis::solvers::type::modal);
	dynamic_cast<fea::analysis::solvers::Modal*>(model.analysis()->solver())->spectre(fea::analysis::solvers::spectre::full);
	dynamic_cast<fea::analysis::solvers::Modal*>(model.analysis()->solver())->watch_dof(n / 2, fea::mesh::nodes::dof::translation_2);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}