//std
#include <cmath>
#include <ctime>

//mat
#include "linear/vec3.h"

//fea
#include "Model/Model.h"

#include "Mesh/Mesh.h"
#include "Mesh/Nodes/Dof.h"
#include "Mesh/Nodes/Node.h"
#include "Mesh/Cells/Cell.h"
#include "Mesh/Cells/Types.h"
#include "Mesh/Joints/Types.h"
#include "Mesh/Sections/Types.h"
#include "Mesh/Elements/Types.h"
#include "Mesh/Joints/Revolute.h"
#include "Mesh/Materials/Types.h"
#include "Mesh/Elements/Mechanic/Mechanic.h"
#include "Mesh/Cells/Quadrature/Quadrature.h"
#include "Mesh/Elements/Mechanic/Frame/Beam.h"
#include "Mesh/Elements/Mechanic/Frame/Beam3.h"
#include "Mesh/Elements/Mechanic/Frame/Warping.h"

#include "Boundary/Boundary.h"

#include "Analysis/Analysis.h"
#include "Analysis/Solvers/Types.h"
#include "Analysis/Solvers/Drift.h"

//ben
#include "benchmarks/solvers/solvers.h"

//buckling
void tests::solvers::drift::single(void)
{
	//model
	fea::models::Model model("single", "benchmarks/solvers/drift");

	//nodes
	mat::vec3 v;
	const unsigned n = 2;
	srand(time(nullptr));
	for(unsigned i = 0; i < n; i++)
	{
		model.mesh()->add_node(v.randu().mem());
	}

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::beam);

	//sections
	model.mesh()->add_section(fea::mesh::sections::type::rectangle);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);

	//elements
	model.mesh()->add_element(fea::mesh::elements::type::beam3C, {0, 1});

	//setup
	fea::mesh::elements::Beam::shear(false);
	fea::mesh::elements::Beam::high_order(false);
	fea::mesh::elements::Mechanic::geometric(true);
	fea::mesh::elements::Beam3::warping(fea::mesh::elements::warping::benscoter);
	model.mesh()->cell(0)->quadrature()->order(5);

	//solver
	model.analysis()->solver(fea::analysis::solvers::type::drift);
	dynamic_cast<fea::analysis::solvers::Drift*>(model.analysis()->solver())->dof_min(0);
	dynamic_cast<fea::analysis::solvers::Drift*>(model.analysis()->solver())->dof_max(1);
	dynamic_cast<fea::analysis::solvers::Drift*>(model.analysis()->solver())->scale(1e-5);
	dynamic_cast<fea::analysis::solvers::Drift*>(model.analysis()->solver())->step_max(10000);
	dynamic_cast<fea::analysis::solvers::Drift*>(model.analysis()->solver())->threshold(1e-3);
	dynamic_cast<fea::analysis::solvers::Drift*>(model.analysis()->solver())->field(fea::analysis::solvers::field::force);
	dynamic_cast<fea::analysis::solvers::Drift*>(model.analysis()->solver())->watch_dof(1, fea::mesh::nodes::dof::translation_1);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}