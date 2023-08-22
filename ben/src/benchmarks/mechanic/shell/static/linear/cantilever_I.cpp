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
#include "fea/inc/Analysis/Solvers/Types.h"
#include "fea/inc/Analysis/Solvers/Solver.h"
#include "fea/inc/Analysis/Solvers/Buckling.h"

//ben
#include "ben/inc/benchmarks/mechanic/shell.h"

void tests::shell::static_linear::cantilever_I(void)
{
	//data
	const double L = 3.00e+00;
	const double w = 1.00e-01;
	const double h = 1.00e-01;
	const double t = 1.00e-02;
	const double s = 5.00e-01;

	//model
	fea::models::Model model("cantilever I", "benchmarks/shell/static/linear");

	//points
	model.topology()->add_point(0, 0, 0, s);
	model.topology()->add_point(L, 0, 0, s);
	model.topology()->add_point(0, -h / 2, 0, s);
	model.topology()->add_point(0, +h / 2, 0, s);
	model.topology()->add_point(L, -h / 2, 0, s);
	model.topology()->add_point(L, +h / 2, 0, s);
	model.topology()->add_point(0, -h / 2, -w / 2, s);
	model.topology()->add_point(0, -h / 2, +w / 2, s);
	model.topology()->add_point(0, +h / 2, -w / 2, s);
	model.topology()->add_point(0, +h / 2, +w / 2, s);
	model.topology()->add_point(L, -h / 2, -w / 2, s);
	model.topology()->add_point(L, -h / 2, +w / 2, s);
	model.topology()->add_point(L, +h / 2, -w / 2, s);
	model.topology()->add_point(L, +h / 2, +w / 2, s);

	//curves
	model.topology()->add_curve(fea::topology::curves::type::line, { 0,  2});
	model.topology()->add_curve(fea::topology::curves::type::line, { 0,  3});
	model.topology()->add_curve(fea::topology::curves::type::line, { 1,  4});
	model.topology()->add_curve(fea::topology::curves::type::line, { 1,  5});
	model.topology()->add_curve(fea::topology::curves::type::line, { 2,  6});
	model.topology()->add_curve(fea::topology::curves::type::line, { 2,  7});
	model.topology()->add_curve(fea::topology::curves::type::line, { 3,  8});
	model.topology()->add_curve(fea::topology::curves::type::line, { 3,  9});
	model.topology()->add_curve(fea::topology::curves::type::line, { 4, 10});
	model.topology()->add_curve(fea::topology::curves::type::line, { 4, 11});
	model.topology()->add_curve(fea::topology::curves::type::line, { 5, 12});
	model.topology()->add_curve(fea::topology::curves::type::line, { 5, 13});
	model.topology()->add_curve(fea::topology::curves::type::line, { 0,  1});
	model.topology()->add_curve(fea::topology::curves::type::line, { 2,  4});
	model.topology()->add_curve(fea::topology::curves::type::line, { 3,  5});
	model.topology()->add_curve(fea::topology::curves::type::line, { 6, 10});
	model.topology()->add_curve(fea::topology::curves::type::line, { 7, 11});
	model.topology()->add_curve(fea::topology::curves::type::line, { 8, 12});
	model.topology()->add_curve(fea::topology::curves::type::line, { 9, 13});

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::tri3);
	((fea::mesh::cells::Plane*) model.mesh()->cell(0))->thickness(t);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);

	//surfaces
	//model.topology()->add_surface({{{0, true}, {13, true}, { 2, false}, {12, false}}});
	//model.topology()->add_surface({{{1, true}, {14, true}, { 3, false}, {12, false}}});
	//model.topology()->add_surface({{{4, true}, {15, true}, { 8, false}, {13, false}}});
	//model.topology()->add_surface({{{5, true}, {16, true}, { 9, false}, {13, false}}});
	//model.topology()->add_surface({{{6, true}, {17, true}, {10, false}, {14, false}}});
	//model.topology()->add_surface({{{7, true}, {18, true}, {11, false}, {14, false}}});
	for(fea::topology::surfaces::Surface* surface : model.topology()->surfaces())
	{
		surface->cell(0);
		surface->material(0);
		surface->element(fea::mesh::elements::type::shell);
	}

	//mesh
	model.topology()->mesh(2);

	//supports
	const unsigned lc[] = {0, 1, 4, 5, 6, 7};
	const unsigned lp[] = {0, 2, 3, 6, 7, 8, 9};
	for(const unsigned& curve : lc)
	{
		for(const unsigned& node : model.topology()->curve(curve)->nodes())
		{
			model.boundary()->add_support(node, fea::mesh::nodes::dof::rotation_1);
			model.boundary()->add_support(node, fea::mesh::nodes::dof::rotation_2);
			model.boundary()->add_support(node, fea::mesh::nodes::dof::rotation_3);
			model.boundary()->add_support(node, fea::mesh::nodes::dof::translation_1);
			model.boundary()->add_support(node, fea::mesh::nodes::dof::translation_2);
			model.boundary()->add_support(node, fea::mesh::nodes::dof::translation_3);
		}
	}
	for(const unsigned& point : lp)
	{
		model.boundary()->add_support(model.topology()->point(point)->node(), fea::mesh::nodes::dof::rotation_1);
		model.boundary()->add_support(model.topology()->point(point)->node(), fea::mesh::nodes::dof::rotation_2);
		model.boundary()->add_support(model.topology()->point(point)->node(), fea::mesh::nodes::dof::rotation_3);
		model.boundary()->add_support(model.topology()->point(point)->node(), fea::mesh::nodes::dof::translation_1);
		model.boundary()->add_support(model.topology()->point(point)->node(), fea::mesh::nodes::dof::translation_2);
		model.boundary()->add_support(model.topology()->point(point)->node(), fea::mesh::nodes::dof::translation_3);
	}

	//loads
	model.boundary()->add_load_set();
	model.boundary()->add_load_case(1, fea::mesh::nodes::dof::translation_2, -1e3);

	//solver
	// model.analysis()->solver()->load_set(0);
	// model.analysis()->solver()->watch_dof(1, fea::mesh::nodes::dof::translation_2);
	model.analysis()->solver(fea::analysis::solvers::type::buckling);
	dynamic_cast<fea::analysis::solvers::Buckling*>(model.analysis()->solver())->spectre(fea::analysis::solvers::spectre::full);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}