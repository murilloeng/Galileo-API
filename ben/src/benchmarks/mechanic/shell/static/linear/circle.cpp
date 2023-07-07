//std
#include <cmath>

//fea
#include "Model/Model.h"

#include "Mesh/Mesh.h"
#include "Mesh/Nodes/Dof.h"
#include "Mesh/Cells/Types.h"
#include "Mesh/Elements/Types.h"
#include "Mesh/Materials/Types.h"
#include "Mesh/Cells/Plane/Plane.h"
#include "Mesh/Cells/Quadrature/Quadrature.h"

#include "Boundary/Boundary.h"
#include "Boundary/Loads/Types.h"
#include "Boundary/Loads/Load_Case.h"
#include "Boundary/Loads/Elements/Mechanic/Plane/Plane_Force.h"

#include "Topology/Topology.h"
#include "Topology/Points/Point.h"
#include "Topology/Curves/Curve.h"
#include "Topology/Curves/Types.h"
#include "Topology/Surfaces/Surface.h"

#include "Analysis/Analysis.h"
#include "Analysis/Solvers/Solver.h"

//ben
#include "benchmarks/mechanic/shell.h"

void tests::shell::static_linear::circle(void)
{
	//data
	const double R = 1.50e+00;
	const double t = 1.00e-02;
	const double s = 7.50e-02;

	//model
	fea::models::Model model("circle", "benchmarks/shell/static/linear");

	//points
	const double dq = 2 * M_PI / 3;
	model.topology()->add_point(0, 0, 0, s);
	model.topology()->add_point(R * cos(0 * dq), R * sin(0 * dq), 0, s);
	model.topology()->add_point(R * cos(1 * dq), R * sin(1 * dq), 0, s);
	model.topology()->add_point(R * cos(2 * dq), R * sin(2 * dq), 0, s);

	//curves
	model.topology()->add_curve(fea::topology::curves::type::circle_arc, {1, 0, 2});
	model.topology()->add_curve(fea::topology::curves::type::circle_arc, {2, 0, 3});
	model.topology()->add_curve(fea::topology::curves::type::circle_arc, {3, 0, 1});

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::tri3);
	((fea::mesh::cells::Plane*) model.mesh()->cell(0))->thickness(t);
	((fea::mesh::cells::Plane*) model.mesh()->cell(0))->quadrature()->order(2);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);

	//surfaces
	//model.topology()->add_surface({{0, 1, 2}});
	model.topology()->surface(0)->cell(0);
	model.topology()->surface(0)->material(0);
	model.topology()->surface(0)->element(fea::mesh::elements::type::shell);

	//mesh
	model.topology()->mesh(2);

	//supports
	for(fea::topology::points::Point* point : model.topology()->points())
	{
		if(point->index() != 0)
		{
			model.boundary()->add_support(point->node(), fea::mesh::nodes::dof::rotation_1);
			model.boundary()->add_support(point->node(), fea::mesh::nodes::dof::rotation_2);
			model.boundary()->add_support(point->node(), fea::mesh::nodes::dof::translation_3);
		}
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
	model.analysis()->solver()->watch_dof(1, fea::mesh::nodes::dof::translation_3);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}