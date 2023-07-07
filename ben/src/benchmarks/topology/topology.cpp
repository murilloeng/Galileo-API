//std
#include <cmath>

//mat
#include "linear/vec3.h"

//fea
#include "Mesh/Mesh.h"
#include "Model/Model.h"
#include "Mesh/Cells/Types.h"
#include "Topology/Topology.h"
#include "Mesh/Elements/Types.h"
#include "Mesh/Materials/Types.h"
#include "Topology/Points/Point.h"
#include "Topology/Curves/Curve.h"
#include "Topology/Curves/Types.h"
#include "Mesh/Cells/Plane/Plane.h"
#include "Topology/Surfaces/Surface.h"

//ben
#include "benchmarks/topology/topology.h"

void tests::topology::topology(void)
{
	//data
	const double s = 1.00;
	const double t = 0.10;

	//model
	fea::models::Model model("topology", "benchmarks/topology");

	//points
	model.topology()->add_point(-s, -s, 0);
	model.topology()->add_point(+s, -s, 0);
	model.topology()->add_point(+s, +s, 0);
	model.topology()->add_point(-s, +s, 0);

	//curves
	model.topology()->add_curve(fea::topology::curves::type::line, {0,  1});
	model.topology()->add_curve(fea::topology::curves::type::line, {1,  2});
	model.topology()->add_curve(fea::topology::curves::type::line, {2,  3});
	model.topology()->add_curve(fea::topology::curves::type::line, {3,  0});

	//transform
	model.topology()->move_curves({0, 1, 2, 3}, {-2 * s, -2 * s, 0}, true);
	model.topology()->move_curves({0, 1, 2, 3}, {+2 * s, -2 * s, 0}, true);
	model.topology()->move_curves({0, 1, 2, 3}, {+2 * s, +2 * s, 0}, true);
	model.topology()->move_curves({0, 1, 2, 3}, {-2 * s, +2 * s, 0}, true);
	model.topology()->scale_curves({0, 1, 2, 3}, {0, 0, 0}, 4, false);

	//loops
	// std::vector<fea::topology::Oriented> l0 = { 0,  1,  2,  3};
	// std::vector<fea::topology::Oriented> l1 = { 4,  5,  6,  7};
	// std::vector<fea::topology::Oriented> l2 = { 8,  9, 10, 11};
	// std::vector<fea::topology::Oriented> l3 = {12, 13, 14, 15};
	// std::vector<fea::topology::Oriented> l4 = {16, 17, 18, 19};

	//surfaces
	//model.topology()->add_surface({l0, l1, l2, l3, l4});

	//topology
	model.topology()->surface(0)->cell(0);
	model.topology()->surface(0)->material(0);
	model.topology()->surface(0)->element(fea::mesh::elements::type::plane);

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::tri3);
	((fea::mesh::cells::Plane*) model.mesh()->cell(0))->thickness(t);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);

	//mesh
	model.topology()->mesh(2);

	//save
	model.save();
}