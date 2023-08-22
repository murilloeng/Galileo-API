//std
#include <cmath>
#include <ctime>

//fea
#include "fea/inc/Model/Model.h"

#include "fea/inc/Mesh/Mesh.h"
#include "fea/inc/Mesh/Nodes/Dof.h"
#include "fea/inc/Mesh/Nodes/Node.h"
#include "fea/inc/Mesh/Cells/Cell.h"
#include "fea/inc/Mesh/Cells/Types.h"
#include "fea/inc/Mesh/Joints/Types.h"
#include "fea/inc/Mesh/Sections/Types.h"
#include "fea/inc/Mesh/Elements/Types.h"
#include "fea/inc/Mesh/Joints/Revolute.h"
#include "fea/inc/Mesh/Materials/Types.h"
#include "fea/inc/Mesh/Elements/Mechanic/Mechanic.h"
#include "fea/inc/Mesh/Cells/Quadrature/Quadrature.h"
#include "fea/inc/Mesh/Elements/Mechanic/Frame/Beam.h"
#include "fea/inc/Mesh/Elements/Mechanic/Frame/Beam3.h"
#include "fea/inc/Mesh/Elements/Mechanic/Frame/Warping.h"

#include "fea/inc/Boundary/Boundary.h"

#include "fea/inc/Analysis/Analysis.h"
#include "fea/inc/Analysis/Solvers/Types.h"
#include "fea/inc/Analysis/Solvers/Drift.h"

//ben
#include "ben/inc/benchmarks/solvers/solvers.h"

//buckling
void tests::solvers::drift::single(void)
{
	//model
	fea::models::Model model("single", "benchmarks/solvers/drift");

	//nodes
	double v[3];
	const unsigned n = 2;
	srand(time(nullptr));
	for(unsigned i = 0; i < n; i++)
	{
		v[2] = 0;
		for(unsigned j = 0; j < 2; j++)
		{
			v[j] = (double) rand() / RAND_MAX;
		}
		model.mesh()->add_node(v);
	}

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::beam);

	//sections
	model.mesh()->add_section(fea::mesh::sections::type::rectangle);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);

	//elements
	model.mesh()->add_element(fea::mesh::elements::type::beam2T, {0, 1});

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