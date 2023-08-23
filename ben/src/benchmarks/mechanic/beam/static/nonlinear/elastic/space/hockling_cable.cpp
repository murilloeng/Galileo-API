//std
#include <cmath>

//fea
#include "fea/inc/Model/Model.h"

#include "fea/inc/Mesh/Mesh.h"
#include "fea/inc/Mesh/Nodes/Dof.h"
#include "fea/inc/Mesh/Cells/Types.h"
#include "fea/inc/Mesh/Sections/Types.h"
#include "fea/inc/Mesh/Elements/Types.h"
#include "fea/inc/Mesh/Cells/Line/Line.h"
#include "fea/inc/Mesh/Materials/Types.h"
#include "fea/inc/Mesh/Sections/Generic.h"
#include "fea/inc/Mesh/Materials/Mechanic/Steel.h"
#include "fea/inc/Mesh/Elements/Mechanic/Mechanic.h"
#include "fea/inc/Mesh/Elements/Mechanic/Frame/Beam.h"

#include "fea/inc/Boundary/Boundary.h"

#include "fea/inc/Analysis/Analysis.h"
#include "fea/inc/Analysis/Solvers/Types.h"
#include "fea/inc/Analysis/Strategies/Types.h"
#include "fea/inc/Analysis/Solvers/Static_Nonlinear.h"

//ben
#include "ben/inc/benchmarks/mechanic/beam.h"

void tests::beam::static_nonlinear::elastic::space::hockling_cable(void)
{
	/*
	Hockling cable
	Literature: Phd Thesis V. L. Corvec (2012) pp. 121
	*/

	//data
	const double L = 2.40e-01;
	const double q = 4.00e-04;
	const double v = 3.10e-01;
	const double E = 7.12e+10;
	const double A = 1.00e-06;
	const double T = 2.50e-01;
	const double J = 2.16e-12;
	const double I22 = 8.33e-14;
	const double I33 = 8.33e-14;

	//model
	fea::models::Model model("hockling cable", "benchmarks/beam/static/nonlinear/elastic/space");

	//nodes
	model.mesh()->add_node(0, 0, 0);
	model.mesh()->add_node(L, q, 0);

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::beam);

	//sections
	model.mesh()->add_section(fea::mesh::sections::type::generic);
	((fea::mesh::sections::Generic*) model.mesh()->section(0))->area(A);
	((fea::mesh::sections::Generic*) model.mesh()->section(0))->inertia(J);
	((fea::mesh::sections::Generic*) model.mesh()->section(0))->inertia(0, 0, I22);
	((fea::mesh::sections::Generic*) model.mesh()->section(0))->inertia(1, 1, I33);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->poisson_ratio(v);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->elastic_modulus(E);

	//elements
	model.mesh()->add_element(fea::mesh::elements::type::beam3T, {0, 1});

	//supports
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_3);
	model.boundary()->add_support(1, fea::mesh::nodes::dof::rotation_2);
	model.boundary()->add_support(1, fea::mesh::nodes::dof::rotation_3);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_3);
	model.boundary()->add_support(1, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(1, fea::mesh::nodes::dof::translation_3);

	//loads
	model.boundary()->add_load_set();
	model.boundary()->add_load_case(1, fea::mesh::nodes::dof::rotation_1, T);

	//solver
	fea::mesh::cells::Line::refine(0, 20);
	fea::mesh::elements::Mechanic::geometric(true);
	model.analysis()->solver(fea::analysis::solvers::type::static_nonlinear);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_set(0);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_max(2);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->step_max(400);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_guess(0.01);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_adjust(true);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->iteration_desired(7);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->watch_dof(1, fea::mesh::nodes::dof::rotation_1);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->strategy(fea::analysis::strategies::type::minimal_norm);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}