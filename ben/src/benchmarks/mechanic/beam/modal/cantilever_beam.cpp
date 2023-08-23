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
#include "fea/inc/Mesh/Sections/Section.h"
#include "fea/inc/Mesh/Materials/Mechanic/Mechanic.h"

#include "fea/inc/Topology/Topology.h"
#include "fea/inc/Topology/Curves/Types.h"
#include "fea/inc/Topology/Curves/Curve.h"

#include "fea/inc/Boundary/Boundary.h"

#include "fea/inc/Analysis/Analysis.h"
#include "fea/inc/Analysis/Solvers/Types.h"
#include "fea/inc/Analysis/Solvers/Modal.h"

//ben
#include "ben/inc/benchmarks/mechanic/beam.h"

//state
void tests::beam::modal::cantilever_beam(void)
{

	//Modal analysis of a cantilever beam
	//Exact solution: wn = (an / L)**2 * sqrt(E * I / r / A)
	//where cos(an) * cosh(an) + 1 = 0

	//data
	const unsigned n = 50;
	const double L = 3.00e+00;
	const double bl[] = {1.8751e+00, 4.6940e+00, 7.8547e+00, 1.0996e+01, 1.4137e+01};

	//model
	fea::models::Model model("cantilever beam", "benchmarks/beam/modal");

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
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_3);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_3);

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
		const double an = i < sizeof(bl) / sizeof(double) ? bl[i] : (2 * i - 1) * M_PI / 2;
		const double wn = pow(an / L, 2) * sqrt(E * I / r / A);
		printf("Mode: %04d | Angular frequency: %+.2e | Frequency: %+.2e |\n", i, wn, wn / 2 / M_PI);
	}

	//save
	model.save();
}