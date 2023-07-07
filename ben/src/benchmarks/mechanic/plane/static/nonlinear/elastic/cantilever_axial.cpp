//fea
#include "Model/Model.h"

#include "Mesh/Mesh.h"
#include "Mesh/Nodes/Dof.h"
#include "Mesh/Cells/Types.h"
#include "Mesh/Elements/Types.h"
#include "Mesh/Materials/Types.h"
#include "Mesh/Cells/Plane/Plane.h"
#include "Mesh/Materials/Mechanic/Steel.h"
#include "Mesh/Elements/Mechanic/Mechanic.h"

#include "Topology/Topology.h"
#include "Topology/Loops/Loop.h"
#include "Topology/Points/Point.h"
#include "Topology/Curves/Curve.h"
#include "Topology/Curves/Types.h"
#include "Topology/Surfaces/Surface.h"

#include "Boundary/Boundary.h"
#include "Boundary/Loads/Types.h"
#include "Boundary/Loads/Load_Case.h"
#include "Boundary/Loads/Elements/Mechanic/Plane/Plane_Force.h"

#include "Analysis/Analysis.h"
#include "Analysis/Solvers/Types.h"
#include "Analysis/Strategies/Types.h"
#include "Analysis/Solvers/Static_Nonlinear.h"

//ben
#include "benchmarks/mechanic/plane.h"

//mesh
const static unsigned nx = 10;
const static unsigned ny = 10;

//data
const static double l = 1.00e+00;
const static double h = 1.00e-01;
const static double t = 4.00e-02;
const static double E = 2.00e+11;
const static double v = 3.00e-01;
const static double s = 4.00e+08;
const static double K = 1.00e+08;

void tests::plane::static_nonlinear::elastic::cantilever_axial(void)
{
	//model
	fea::models::Model model("cantilever axial", "benchmarks/plane/static/nonlinear/elastic");

	//points
	model.topology()->add_point(0, 0, 0);
	model.topology()->add_point(l, 0, 0);
	model.topology()->add_point(l, h, 0);
	model.topology()->add_point(0, h, 0);

	//curves
	model.topology()->add_curve(fea::topology::curves::type::line, {0, 1});
	model.topology()->add_curve(fea::topology::curves::type::line, {1, 2});
	model.topology()->add_curve(fea::topology::curves::type::line, {2, 3});
	model.topology()->add_curve(fea::topology::curves::type::line, {3, 0});

	//surfaces
	//model.topology()->add_surface({{0, 1, 2, 3}});

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::quad4);
	((fea::mesh::cells::Plane*) model.mesh()->cell(0))->thickness(t);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->yield_stress(s);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->poisson_ratio(v);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->elastic_modulus(E);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->plastic_modulus(K);

	//surfaces
	model.topology()->add_surface();
	model.topology()->surface(0)->cell(0);
	model.topology()->surface(0)->add_loop();
	model.topology()->surface(0)->material(0);
	model.topology()->surface(0)->loop(0)->add_item(0, false);
	model.topology()->surface(0)->loop(0)->add_item(1, false);
	model.topology()->surface(0)->loop(0)->add_item(2, false);
	model.topology()->surface(0)->loop(0)->add_item(3, false);
	model.topology()->surface(0)->element(fea::mesh::elements::type::plane);

	//mesh
	model.topology()->mesh(2);

	//supports
	for(unsigned node : model.topology()->curve(3)->nodes())
	{
		model.boundary()->add_support(node, fea::mesh::nodes::dof::translation_1);
	}
	model.boundary()->add_support(model.topology()->point(0)->node(), fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(model.topology()->point(0)->node(), fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(model.topology()->point(3)->node(), fea::mesh::nodes::dof::translation_1);

	//loads

	//solver
	fea::mesh::elements::Mechanic::geometric(true);
	fea::mesh::elements::Mechanic::inelastic(false);
	model.analysis()->solver(fea::analysis::solvers::type::static_nonlinear);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->step_max(100);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_guess(0.01);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->strategy(fea::analysis::strategies::type::control_load);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->watch_dof(nx * (ny + 1), fea::mesh::nodes::dof::translation_1);

	//solve
	// model.analysis()->solve();

	//save
	model.save();
}