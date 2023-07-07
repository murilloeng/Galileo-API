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
#include "Mesh/Sections/Section.h"
#include "Mesh/Materials/Mechanic/Mechanic.h"

#include "Topology/Topology.h"
#include "Topology/Curves/Types.h"
#include "Topology/Curves/Curve.h"

#include "Boundary/Boundary.h"

#include "Analysis/Analysis.h"
#include "Analysis/Solvers/Types.h"
#include "Analysis/Solvers/Modal.h"

//ben
#include "benchmarks/mechanic/beam.h"

//state
void tests::beam::modal::pinned_beam(void)
{

	//Modal analysis of a pinned beam
	//Exact solution: wn = (an / L)**2 * sqrt(E * I / r / A)
	//where: an = n * pi

	//data
	const unsigned n = 50;
	const double L = 3.00e+00;

	//model
	fea::models::Model model("pinned beam", "benchmarks/beam/modal");

	//points
	model.topology()->add_point(0, 0, 0);
	model.topology()->add_point(L, 0, 0);

	//sections
	model.mesh()->add_section(fea::mesh::sections::type::profile_I);

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::beam);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);

	//curves
	model.topology()->add_curve(fea::topology::curves::type::line, {0, 1});
	model.topology()->curve(0)->cell(0);
	model.topology()->curve(0)->material(0);
	model.topology()->curve(0)->structured(n);
	model.topology()->curve(0)->element(fea::mesh::elements::type::beam3C);

	//mesh
	model.topology()->mesh(1);

	//supports
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_3);
	model.boundary()->add_support(1, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(1, fea::mesh::nodes::dof::translation_3);

	//solver
	model.analysis()->solver(fea::analysis::solvers::type::modal);
	dynamic_cast<fea::analysis::solvers::Modal*>(model.analysis()->solver())->spectre_min(0);
	dynamic_cast<fea::analysis::solvers::Modal*>(model.analysis()->solver())->spectre_max(20);
	dynamic_cast<fea::analysis::solvers::Modal*>(model.analysis()->solver())->spectre(fea::analysis::solvers::spectre::partial);

	//solve
	model.analysis()->solve();

	//exact
	const double A = model.mesh()->section(0)->area();
	const double I = model.mesh()->section(0)->inertia(1, 1);
	const double r = ((fea::mesh::materials::Mechanic*) model.mesh()->material(0))->specific_mass();
	const double E = ((fea::mesh::materials::Mechanic*) model.mesh()->material(0))->elastic_modulus();
	for(unsigned i = 1; i < 10; i++)
	{
		const double w = pow(i * M_PI / L, 2) * sqrt(E * I / r / A);
		printf("Mode: %04d | Angular frequency: %+.2e | Frequency: %+.2e |\n", i, w, w / 2 / M_PI);
	}

	//save
	model.save();
}