//std
#include <cmath>

//fea
#include "fea/inc/Mesh/Mesh.h"
#include "fea/inc/Model/Model.h"
#include "fea/inc/Mesh/Nodes/Dof.h"
#include "fea/inc/Mesh/Cells/Types.h"
#include "fea/inc/Boundary/Boundary.h"
#include "fea/inc/Analysis/Analysis.h"
#include "fea/inc/Topology/Topology.h"
#include "fea/inc/Boundary/Time/Time.h"
#include "fea/inc/Mesh/Elements/Types.h"
#include "fea/inc/Boundary/Time/Types.h"
#include "fea/inc/Mesh/Materials/Types.h"
#include "fea/inc/Topology/Curves/Types.h"
#include "fea/inc/Topology/Curves/Curve.h"
#include "fea/inc/Analysis/Solvers/Types.h"
#include "fea/inc/Mesh/Cells/Plane/Plane.h"
#include "fea/inc/Boundary/Time/Polynomial.h"
#include "fea/inc/Topology/Surfaces/Surface.h"
#include "fea/inc/Analysis/Solvers/Static_Linear.h"

//ben
#include "ben/inc/benchmarks/heat/heat.h"

void tests::heat::static_linear::plate(void)
{
	//data
	const double w = 1.00e+00;
	const double h = 1.00e+00;
	const double t = 1.00e-02;

	//model
	fea::models::Model model("plate", "benchmarks/heat/static/linear");

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::tri3);
	((fea::mesh::cells::Plane*) model.mesh()->cell(0))->thickness(t);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::heat);

	//points
	model.topology()->add_point(0, 0, 0);
	model.topology()->add_point(w, 0, 0);
	model.topology()->add_point(w, h, 0);
	model.topology()->add_point(0, h, 0);

	//curves
	model.topology()->add_curve(fea::topology::curves::type::line, {0, 1});
	model.topology()->add_curve(fea::topology::curves::type::line, {1, 2});
	model.topology()->add_curve(fea::topology::curves::type::line, {2, 3});
	model.topology()->add_curve(fea::topology::curves::type::line, {3, 0});

	//surfaces
	//model.topology()->add_surface({{0, 1, 2, 3}});
	model.topology()->surface(0)->cell(0);
	model.topology()->surface(0)->material(0);
	model.topology()->surface(0)->element(fea::mesh::elements::type::heat);

	//mesh
	model.topology()->mesh(2);

	//times
	model.boundary()->add_time(fea::boundary::time::type::polynomial);
	((fea::boundary::time::Polynomial*) model.boundary()->time(0))->terms().push_back(1);

	//supports
	for(unsigned node : model.topology()->curve(3)->nodes(true))
	{
		model.boundary()->add_support(node, fea::mesh::nodes::dof::temperature);
	}
	for(unsigned node : model.topology()->curve(1)->nodes(true))
	{
		model.boundary()->add_support(node, fea::mesh::nodes::dof::temperature, 0);
	}

	//solver
	model.analysis()->solver()->watch_dof(1, fea::mesh::nodes::dof::temperature);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}