//std
#include <numeric>

//fea
#include "fea/inc/Model/Model.h"

#include "fea/inc/Mesh/Mesh.h"
#include "fea/inc/Mesh/Nodes/Dof.h"
#include "fea/inc/Mesh/Nodes/Node.h"
#include "fea/inc/Mesh/Cells/Cell.h"
#include "fea/inc/Mesh/Cells/Types.h"
#include "fea/inc/Mesh/Elements/Types.h"
#include "fea/inc/Mesh/Materials/Types.h"
#include "fea/inc/Mesh/Elements/Element.h"
#include "fea/inc/Mesh/Elements/Mechanic/Plane/Warping.h"

#include "fea/inc/Topology/Topology.h"
#include "fea/inc/Topology/Curves/Types.h"
#include "fea/inc/Topology/Curves/Curve.h"
#include "fea/inc/Topology/Surfaces/Surface.h"

#include "fea/inc/Boundary/Boundary.h"
#include "fea/inc/Boundary/Loads/Types.h"
#include "fea/inc/Boundary/Loads/Load_Case.h"
#include "fea/inc/Boundary/Loads/Elements/Element.h"

#include "fea/inc/Analysis/Analysis.h"
#include "fea/inc/Analysis/Solvers/Types.h"
#include "fea/inc/Analysis/Solvers/Solver.h"

//ben
#include "ben/inc/benchmarks/mechanic/warping.h"

void tests::warping::static_linear::rectangle(void)
{
	//data
	const unsigned nw = 3;
	const unsigned nh = 21;
	const double w = 6.00e-01;
	const double h = 6.00e+00;

	//model
	fea::models::Model model("rectangle", "benchmarks/warping/static/linear");

	//points
	model.topology()->add_point(-w / 2, -h / 2, 0);
	model.topology()->add_point(+w / 2, -h / 2, 0);
	model.topology()->add_point(+w / 2, +h / 2, 0);
	model.topology()->add_point(-w / 2, +h / 2, 0);

	//curves
	model.topology()->add_curve(fea::topology::curves::type::line, {0, 1});
	model.topology()->add_curve(fea::topology::curves::type::line, {1, 2});
	model.topology()->add_curve(fea::topology::curves::type::line, {2, 3});
	model.topology()->add_curve(fea::topology::curves::type::line, {3, 0});

	//surface
	//model.topology()->add_surface({{0, 1, 2, 3}});
	model.topology()->surface(0)->element(fea::mesh::elements::type::warping);

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::tri6);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);

	//mesh
	model.topology()->order(2);
	model.topology()->size(w / 2);
	model.topology()->mesh(2);
	const unsigned ne = model.mesh()->elements().size();

	//supports
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_3);

	//loads
	std::vector<unsigned> list(ne);
	std::iota(list.begin(), list.end(), 0);
	model.boundary()->add_load_case(fea::boundary::loads::type::plane_force, list);

	//solver
	model.analysis()->solver(fea::analysis::solvers::type::static_linear);
	model.analysis()->solver()->watch_dof(0, fea::mesh::nodes::dof::translation_3);
	model.analysis()->solver()->adapt_increment([&model] (void) {
		const double w = model.mesh()->mean(fea::mesh::nodes::dof::translation_3);
		for(fea::mesh::nodes::Node* node : model.mesh()->nodes())
		{
			node->state(fea::mesh::nodes::dof::translation_3) -= w;
		}
	});

	//solve
	model.analysis()->solve();

	//save
	model.save();

	//stiffness
	double k = 0;
	for(const fea::mesh::elements::Element* element : model.mesh()->elements())
	{
		k += ((fea::mesh::elements::Warping*) element)->stiffness();
	}
	printf("stiffness: %+.6e\n", k);
}