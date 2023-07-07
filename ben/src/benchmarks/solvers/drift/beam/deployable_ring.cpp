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
#include "Mesh/Elements/Mechanic/Frame/Beam.h"

#include "Boundary/Boundary.h"

#include "Analysis/Analysis.h"
#include "Analysis/Solvers/Types.h"
#include "Analysis/Solvers/Drift.h"

//ben
#include "benchmarks/solvers/solvers.h"

//buckling
void tests::solvers::drift::beam::deployable_ring(void)
{
	//data
	const unsigned n = 128;
	const unsigned q = n / 2;
	const double r = 1.20e+02;
	const double h = 6.00e+00;
	const double w = 6.00e-01;
	const double E = 2.00e+05;
	const double v = 3.00e-01;

	//model
	fea::models::Model model("deployable ring", "benchmarks/solvers/drift/beam");

	//nodes
	for(unsigned i = 0; i < n; i++)
	{
		const double t = 2 * M_PI * i / n;
		model.mesh()->add_node(r * cos(t), r * sin(t), 0);
	}

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::beam);

	//sections
	model.mesh()->add_section(fea::mesh::sections::type::rectangle);
	((fea::mesh::sections::Rectangle*) model.mesh()->section(0))->width(w);
	((fea::mesh::sections::Rectangle*) model.mesh()->section(0))->height(h);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->poisson_ratio(v);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->elastic_modulus(E);

	//elements
	for(unsigned i = 0; i < n; i++)
	{
		const double t = 2 * M_PI * i / n + M_PI / n;
		model.mesh()->add_element(fea::mesh::elements::type::beam3T, {i, (i + 1) % n});
		((fea::mesh::elements::Beam*) model.mesh()->element(i))->orientation(cos(t), sin(t), 0);
	}

	//supports
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_3);
	model.boundary()->add_support(q, fea::mesh::nodes::dof::rotation_2);
	model.boundary()->add_support(q, fea::mesh::nodes::dof::rotation_3);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_3);
	model.boundary()->add_support(q, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(q, fea::mesh::nodes::dof::translation_3);

	//solver
	fea::mesh::elements::Mechanic::geometric(true);
	model.analysis()->solver(fea::analysis::solvers::type::drift);
	dynamic_cast<fea::analysis::solvers::Drift*>(model.analysis()->solver())->dof_min(0);
	dynamic_cast<fea::analysis::solvers::Drift*>(model.analysis()->solver())->dof_max(1);
	dynamic_cast<fea::analysis::solvers::Drift*>(model.analysis()->solver())->scale(1e-6);
	dynamic_cast<fea::analysis::solvers::Drift*>(model.analysis()->solver())->step_max(10000);
	dynamic_cast<fea::analysis::solvers::Drift*>(model.analysis()->solver())->threshold(1e-2);
	dynamic_cast<fea::analysis::solvers::Drift*>(model.analysis()->solver())->field(fea::analysis::solvers::field::stiffness);
	dynamic_cast<fea::analysis::solvers::Drift*>(model.analysis()->solver())->watch_dof(1, fea::mesh::nodes::dof::translation_1);

	//solve
	// model.analysis()->solve();

	//save
	model.save();
}