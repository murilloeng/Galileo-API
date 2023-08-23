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
#include "fea/inc/Mesh/Sections/Rectangle.h"
#include "fea/inc/Mesh/Materials/Mechanic/Steel.h"
#include "fea/inc/Mesh/Elements/Mechanic/Mechanic.h"

#include "fea/inc/Topology/Topology.h"
#include "fea/inc/Topology/Curves/Types.h"
#include "fea/inc/Topology/Curves/Curve.h"

#include "fea/inc/Boundary/Boundary.h"

#include "fea/inc/Analysis/Analysis.h"
#include "fea/inc/Analysis/Solvers/Types.h"
#include "fea/inc/Analysis/Strategies/Types.h"
#include "fea/inc/Analysis/Solvers/Static_Nonlinear.h"

//ben
#include "ben/inc/benchmarks/mechanic/beam.h"

void tests::beam::static_nonlinear::elastic::space::cantilever_right_angle(void)
{
	/*
	Cantilever lateral buckling
	Literature: Phd Thesis L. V. Corvec (2012) pp. 119
	 */

	//data
	const unsigned ne = 5;
	const double L = 2.40e-01;
	const double q = 1.00e-06;
	const double w = 6.00e-04;
	const double h = 3.00e-02;
	const double v = 3.10e-01;
	const double E = 7.12e+10;
	const double P = 1.00e+00;

	//model
	fea::models::Model model("cantilever right angle", "benchmarks/beam/static/nonlinear/elastic/space");

	//points
	model.topology()->add_point(0, L, 0);
	model.topology()->add_point(L, L, 0);
	model.topology()->add_point(L, 0, q);

	//sections
	model.mesh()->add_section(fea::mesh::sections::type::rectangle);
	((fea::mesh::sections::Rectangle*) model.mesh()->section(0))->width(w);
	((fea::mesh::sections::Rectangle*) model.mesh()->section(0))->height(h);

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::beam);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->poisson_ratio(v);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->elastic_modulus(E);

	//curves
	model.topology()->add_curve(fea::topology::curves::type::line, {0, 1});
	model.topology()->add_curve(fea::topology::curves::type::line, {1, 2});
	model.topology()->curve(0)->cell(0);
	model.topology()->curve(1)->cell(0);
	model.topology()->curve(0)->material(0);
	model.topology()->curve(1)->material(0);
	model.topology()->curve(0)->structured(ne);
	model.topology()->curve(1)->structured(ne);
	model.topology()->curve(0)->element(fea::mesh::elements::type::beam3T);
	model.topology()->curve(1)->element(fea::mesh::elements::type::beam3T);

	//mesh
	model.topology()->mesh(1);

	//supports
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_3);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_3);

	//loads
	model.boundary()->add_load_set();
	model.boundary()->add_load_case(2, fea::mesh::nodes::dof::translation_1, P);

	//solver
	fea::mesh::elements::Mechanic::geometric(true);
	model.analysis()->solver(fea::analysis::solvers::type::static_nonlinear);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_set(0);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_max(2);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->step_max(800);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_adjust(true);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_guess(0.01);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->iteration_desired(3);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->watch_dof(2, fea::mesh::nodes::dof::translation_3);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->strategy(fea::analysis::strategies::type::minimal_norm);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}