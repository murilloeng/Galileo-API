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

void tests::shell::static_linear::cantilever_rectangle(void)
{
	//data
	const double L = 3.00e+00;
	const double w = 4.00e-02;
	const double h = 1.00e-01;
	const double s = 5.00e-02;

	//model
	fea::models::Model model("cantilever rectangle", "benchmarks/shell/static/linear");

	//points
	model.topology()->add_point(0, 0, 0, s);
	model.topology()->add_point(L, 0, 0, s);
	model.topology()->add_point(L, h, 0, s);
	model.topology()->add_point(0, h, 0, s);

	//curves
	model.topology()->add_curve(fea::topology::curves::type::line, {0, 1});
	model.topology()->add_curve(fea::topology::curves::type::line, {1, 2});
	model.topology()->add_curve(fea::topology::curves::type::line, {2, 3});
	model.topology()->add_curve(fea::topology::curves::type::line, {3, 0});

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::tri3);
	((fea::mesh::cells::Plane*) model.mesh()->cell(0))->thickness(w);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);

	//surfaces
	//model.topology()->add_surface({{0, 1, 2, 3}});
	model.topology()->surface(0)->cell(0);
	model.topology()->surface(0)->material(0);
	model.topology()->surface(0)->element(fea::mesh::elements::type::shell);

	//mesh
	model.topology()->mesh(2);

	//supports
	const unsigned lc[] = {3};
	const unsigned lp[] = {0, 3};
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
	// model.boundary()->add_load_set();
	// model.boundary()->add_load_case(1, fea::mesh::nodes::dof::translation_2, -1e3);

	//solver
	// model.analysis()->solver()->load_set(0);
	// model.analysis()->solver()->watch_dof(1, fea::mesh::nodes::dof::translation_2);
	model.analysis()->solver(fea::analysis::solvers::type::buckling);
	dynamic_cast<fea::analysis::solvers::Buckling*>(model.analysis()->solver())->spectre_min(0);
	dynamic_cast<fea::analysis::solvers::Buckling*>(model.analysis()->solver())->spectre_max(50);
	dynamic_cast<fea::analysis::solvers::Buckling*>(model.analysis()->solver())->spectre(fea::analysis::solvers::spectre::partial);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}