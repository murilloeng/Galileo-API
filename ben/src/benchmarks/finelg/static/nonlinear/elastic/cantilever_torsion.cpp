//std
#include <cmath>

//fea
#include "Model/Model.h"

#include "Mesh/Mesh.h"
#include "Mesh/Nodes/Dof.h"
#include "Mesh/Nodes/Node.h"
#include "Mesh/Cells/Types.h"
#include "Mesh/Joints/Types.h"
#include "Mesh/Sections/Types.h"
#include "Mesh/Elements/Types.h"
#include "Mesh/Cells/Line/Line.h"
#include "Mesh/Materials/Types.h"
#include "Mesh/Sections/Rectangle.h"
#include "Mesh/Materials/Mechanic/Steel.h"
#include "Mesh/Elements/Mechanic/Mechanic.h"
#include "Mesh/Cells/Quadrature/Quadrature.h"
#include "Mesh/Elements/Mechanic/Frame/Beam3.h"
#include "Mesh/Elements/Mechanic/Frame/Warping.h"

#include "Topology/Topology.h"
#include "Topology/Curves/Curve.h"
#include "Topology/Curves/Types.h"

#include "Boundary/Boundary.h"

#include "Analysis/Analysis.h"
#include "Analysis/Solvers/Types.h"
#include "Analysis/Strategies/Types.h"
#include "Analysis/Solvers/Static_Nonlinear.h"

//ben
#include "benchmarks/finelg/finelg.h"

void tests::finelg::static_nonlinear::elastic::cantilever_torsion(void)
{
	/*
	Cantilever torsion
	Literature: Phd Thesis V. V. Goyet (1989) sec. 8.3.4 pp. 8.26
	*/

	//data
	const unsigned np = 2;
	const unsigned ne = 20;
	const double L = 1.00e+02;
	const double h = 1.00e+01;
	const double w = 5.00e-01;
	const double v = 3.33e-01;
	const double E = 2.10e+06;
	const double M = 1.00e+03;

	//model
	fea::models::Model model("cantilever torsion", "benchmarks/finelg/static/nonlinear/elastic");

	//points
	model.topology()->add_point(0, 0, 0);
	model.topology()->add_point(L, 0, 0);

	//sections
	model.mesh()->add_section(fea::mesh::sections::type::rectangle);
	((fea::mesh::sections::Rectangle*) model.mesh()->section(0))->width(w);
	((fea::mesh::sections::Rectangle*) model.mesh()->section(0))->height(h);
	((fea::mesh::sections::Rectangle*) model.mesh()->section(0))->size(w / 2);

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::beam);
	model.mesh()->cell(0)->quadrature()->order(np);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->poisson_ratio(v);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->elastic_modulus(E);

	//curves
	model.topology()->add_curve(fea::topology::curves::type::line, {0, 1});
	model.topology()->curve(0)->cell(0);
	model.topology()->curve(0)->material(0);
	model.topology()->curve(0)->structured(ne);
	model.topology()->curve(0)->element(fea::mesh::elements::type::beam3C);

	//mesh
	model.topology()->mesh(1);

	//supports
	model.boundary()->add_support(0, fea::mesh::nodes::dof::warping_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_3);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_3);

	//loads
	model.boundary()->add_load_set();
	model.boundary()->add_load_case(1, fea::mesh::nodes::dof::rotation_1, M);

	//setup
	fea::mesh::elements::Beam::high_order(true);
	fea::mesh::elements::Mechanic::geometric(true);
	fea::mesh::elements::Beam3::warping(fea::mesh::elements::warping::saint_venant);

	//solver
	model.analysis()->solver(fea::analysis::solvers::type::static_nonlinear);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_set(0);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_max(9);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->step_max(450);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_guess(0.02);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->watch_dof(1, fea::mesh::nodes::dof::rotation_1);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->strategy(fea::analysis::strategies::type::minimal_norm);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}