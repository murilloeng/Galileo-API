//fea
#include "fea/inc/Model/Model.h"

#include "fea/inc/Mesh/Mesh.h"
#include "fea/inc/Mesh/Nodes/Dof.h"
#include "fea/inc/Mesh/Cells/Types.h"
#include "fea/inc/Mesh/Elements/Types.h"
#include "fea/inc/Mesh/Materials/Types.h"
#include "fea/inc/Mesh/Cells/Plane/Plane.h"
#include "fea/inc/Mesh/Materials/Mechanic/Steel.h"

#include "fea/inc/Topology/Topology.h"
#include "fea/inc/Topology/Loops/Loop.h"
#include "fea/inc/Topology/Points/Point.h"
#include "fea/inc/Topology/Curves/Types.h"
#include "fea/inc/Topology/Curves/Curve.h"
#include "fea/inc/Topology/Surfaces/Surface.h"

#include "fea/inc/Boundary/Boundary.h"
#include "fea/inc/Boundary/Loads/Types.h"
#include "fea/inc/Boundary/Loads/Load_Case.h"
#include "fea/inc/Boundary/Loads/Elements/Mechanic/Plane/Plane_Force.h"

#include "fea/inc/Analysis/Analysis.h"
#include "fea/inc/Analysis/Solvers/Types.h"
#include "fea/inc/Analysis/Solvers/Static_Linear.h"

//ben
#include "ben/inc/benchmarks/mechanic/plane.h"

void tests::plane::static_linear::cantilever_axial(void)
{
	//data
	const unsigned n2 = 1;
	const unsigned n1 = 10;
	const double l = 1.00e+00;
	const double h = 1.00e-01;
	const double t = 4.00e-02;
	const double E = 2.00e+11;
	const double v = 2.50e-01;

	//model
	fea::models::Model model("cantilever axial", "benchmarks/plane/static/linear");

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::tri6);
	((fea::mesh::cells::Plane*) model.mesh()->cell(0))->thickness(t);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->poisson_ratio(v);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->elastic_modulus(E);

	//points
	model.topology()->add_point(0, 0, 0, h / 4);
	model.topology()->add_point(l, 0, 0, h / 4);
	model.topology()->add_point(l, h, 0, h / 4);
	model.topology()->add_point(0, h, 0, h / 4);

	//curves
	model.topology()->add_curve(fea::topology::curves::type::line, {0, 1});
	model.topology()->add_curve(fea::topology::curves::type::line, {1, 2});
	model.topology()->add_curve(fea::topology::curves::type::line, {2, 3});
	model.topology()->add_curve(fea::topology::curves::type::line, {3, 0});

	//surfaces
	//model.topology()->add_surface({{0, 1, 2, 3}});
	model.topology()->surface(0)->cell(0);
	model.topology()->surface(0)->add_loop();
	model.topology()->surface(0)->material(0);
	model.topology()->surface(0)->loop(0)->add_item(0, false);
	model.topology()->surface(0)->loop(0)->add_item(1, false);
	model.topology()->surface(0)->loop(0)->add_item(2, false);
	model.topology()->surface(0)->loop(0)->add_item(3, false);
	model.topology()->surface(0)->element(fea::mesh::elements::type::plane);

	//mesh
	model.topology()->order(2);
	model.topology()->mesh(2);

	//supports
	for(unsigned index : model.topology()->curve(3)->nodes())
	{
		model.boundary()->add_support(index, fea::mesh::nodes::dof::translation_1);
	}
	model.boundary()->add_support(model.topology()->point(0)->node(), fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(model.topology()->point(0)->node(), fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(model.topology()->point(3)->node(), fea::mesh::nodes::dof::translation_1);

	//loads
	model.boundary()->add_load_set();
	model.boundary()->add_load_case(fea::boundary::loads::type::plane_force);
	model.boundary()->load_case(0)->load_element(0)->elements().push_back(n1 - 1);

	//solver
	model.analysis()->solver(fea::analysis::solvers::type::static_linear);
	// model.analysis()->solver()->watch_dof((ny + 1) * nx, fea::mesh::nodes::dof::translation_1);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}