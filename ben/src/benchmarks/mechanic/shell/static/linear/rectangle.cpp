//fea
#include "fea/inc/Model/Model.h"

#include "fea/inc/Mesh/Mesh.h"
#include "fea/inc/Mesh/Nodes/Dof.h"
#include "fea/inc/Mesh/Cells/Types.h"
#include "fea/inc/Mesh/Elements/Types.h"
#include "fea/inc/Mesh/Materials/Types.h"
#include "fea/inc/Mesh/Cells/Plane/Plane.h"
#include "fea/inc/Mesh/Cells/Quadrature/Quadrature.h"

#include "fea/inc/Boundary/Boundary.h"
#include "fea/inc/Boundary/Loads/Types.h"
#include "fea/inc/Boundary/Loads/Load_Case.h"
#include "fea/inc/Boundary/Loads/Elements/Mechanic/Plane/Plane_Force.h"

#include "fea/inc/Topology/Topology.h"
#include "fea/inc/Topology/Points/Point.h"
#include "fea/inc/Topology/Curves/Curve.h"
#include "fea/inc/Topology/Curves/Types.h"
#include "fea/inc/Topology/Surfaces/Surface.h"

#include "fea/inc/Analysis/Analysis.h"
#include "fea/inc/Analysis/Solvers/Solver.h"

//ben
#include "ben/inc/benchmarks/mechanic/shell.h"

void tests::shell::static_linear::rectangle(void)
{
	//data
	const double L = 3.00e+00;
	const double t = 1.00e-02;
	const double s = 7.50e-02;

	//model
	fea::models::Model model("rectangle", "benchmarks/shell/static/linear");

	//points
	model.topology()->add_point(0, 0, 0, s);
	model.topology()->add_point(L, 0, 0, s);
	model.topology()->add_point(L, L, 0, s);
	model.topology()->add_point(0, L, 0, s);

	//curves
	model.topology()->add_curve(fea::topology::curves::type::line, {0, 1});
	model.topology()->add_curve(fea::topology::curves::type::line, {1, 2});
	model.topology()->add_curve(fea::topology::curves::type::line, {2, 3});
	model.topology()->add_curve(fea::topology::curves::type::line, {3, 0});

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::tri6);
	((fea::mesh::cells::Plane*) model.mesh()->cell(0))->thickness(t);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);

	//surfaces
	//model.topology()->add_surface({{0, 1, 2, 3}});
	model.topology()->surface(0)->cell(0);
	model.topology()->surface(0)->material(0);
	model.topology()->surface(0)->element(fea::mesh::elements::type::shell);

	//mesh
	model.topology()->order(2);
	model.topology()->mesh(2);

	//supports
	for(fea::topology::points::Point* point : model.topology()->points())
	{
		model.boundary()->add_support(point->node(), fea::mesh::nodes::dof::rotation_1);
		model.boundary()->add_support(point->node(), fea::mesh::nodes::dof::rotation_2);
		model.boundary()->add_support(point->node(), fea::mesh::nodes::dof::translation_3);
	}
	for(fea::topology::curves::Curve* curve : model.topology()->curves())
	{
		for(unsigned index : curve->nodes())
		{
			model.boundary()->add_support(index, fea::mesh::nodes::dof::rotation_1);
			model.boundary()->add_support(index, fea::mesh::nodes::dof::rotation_2);
			model.boundary()->add_support(index, fea::mesh::nodes::dof::translation_3);
		}
	}

	//loads
	model.boundary()->add_load_set();
	model.boundary()->add_load_case();
	model.boundary()->load_case(0)->add_load_element(fea::boundary::loads::type::plane_force);
	((fea::boundary::loads::Plane_Force*) model.boundary()->load_case(0)->load_element(0))->value(1e5);
	((fea::boundary::loads::Plane_Force*) model.boundary()->load_case(0)->load_element(0))->direction(0, 0, -1);
	for(unsigned index : model.topology()->surface(0)->elements())
	{
		model.boundary()->load_case(0)->load_element(0)->elements().push_back(index);
	}

	//solver
	model.analysis()->solver()->load_set(0);
	model.analysis()->solver()->watch_dof(0, fea::mesh::nodes::dof::translation_3);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}