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
#include "Mesh/Materials/Mechanic/Steel.h"
#include "Mesh/Elements/Mechanic/Mechanic.h"

#include "Boundary/Boundary.h"

#include "Topology/Topology.h"
#include "Topology/Curves/Types.h"
#include "Topology/Curves/Curve.h"

#include "Analysis/Analysis.h"
#include "Analysis/Solvers/Types.h"
#include "Analysis/Solvers/Buckling.h"

//ben
#include "benchmarks/mechanic/beam.h"

void tests::beam::buckling::column(void)
{
	//model
	fea::models::Model model("column", "benchmarks/beam/buckling");

	//sections
	model.mesh()->add_section(fea::mesh::sections::type::profile_I);

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::beam);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);

	//points
	const double L = 3.00e+00;
	model.topology()->add_point(0, 0, 0);
	model.topology()->add_point(L, 0, 0);

	//curves
	model.topology()->add_curve(fea::topology::curves::type::line, {0, 1});
	model.topology()->curve(0)->cell(0);
	model.topology()->curve(0)->material(0);
	model.topology()->curve(0)->structured(20);
	model.topology()->curve(0)->element(fea::mesh::elements::type::beam2C);

	//mesh
	model.topology()->mesh(1);

	//supports
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_3);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_2);

	//loads
	model.boundary()->add_load_set();
	model.boundary()->add_load_case(1, fea::mesh::nodes::dof::translation_1, -1);

	//solver
	fea::mesh::elements::Mechanic::geometric(true);
	model.analysis()->solver(fea::analysis::solvers::type::buckling);
	model.analysis()->solver()->watch_dof(1, fea::mesh::nodes::dof::translation_2);
	dynamic_cast<fea::analysis::solvers::Buckling*>(model.analysis()->solver())->scale(2e2);
	dynamic_cast<fea::analysis::solvers::Buckling*>(model.analysis()->solver())->load_set(0);
	dynamic_cast<fea::analysis::solvers::Buckling*>(model.analysis()->solver())->spectre(fea::analysis::solvers::spectre::full);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}