//fea
#include "fea/inc/Model/Model.h"

#include "fea/inc/Mesh/Mesh.h"
#include "fea/inc/Mesh/Nodes/Dof.h"
#include "fea/inc/Mesh/Nodes/Node.h"
#include "fea/inc/Mesh/Cells/Types.h"
#include "fea/inc/Mesh/Elements/Types.h"
#include "fea/inc/Mesh/Materials/Types.h"
#include "fea/inc/Mesh/Elements/Mechanic/Plane/Warping.h"

#include "fea/inc/Topology/Topology.h"
#include "fea/inc/Topology/Curves/Types.h"
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

//data
static const double r = 1.00e-02;
static const double wf = 1.00e-01;
static const double hw = 1.00e-01;
static const double tw = 1.00e-02;
static const double tf = 1.00e-02;

//misc
static void topology_1(fea::models::Model& model)
{
	//points
	model.topology()->add_point(-tw / 2, -tf / 2, 0);
	model.topology()->add_point(-tw / 2, -hw / 2, 0);
	model.topology()->add_point(+tw / 2, -hw / 2, 0);
	model.topology()->add_point(+tw / 2, -tf / 2, 0);
	model.topology()->add_point(+wf / 2, -tf / 2, 0);
	model.topology()->add_point(+wf / 2, +tf / 2, 0);
	model.topology()->add_point(+tw / 2, +tf / 2, 0);
	model.topology()->add_point(+tw / 2, +hw / 2, 0);
	model.topology()->add_point(-tw / 2, +hw / 2, 0);
	model.topology()->add_point(-tw / 2, +tf / 2, 0);
	model.topology()->add_point(-wf / 2, +tf / 2, 0);
	model.topology()->add_point(-wf / 2, -tf / 2, 0);
	//curves
	model.topology()->add_curve(fea::topology::curves::type::line, { 0,  1});
	model.topology()->add_curve(fea::topology::curves::type::line, { 1,  2});
	model.topology()->add_curve(fea::topology::curves::type::line, { 2,  3});
	model.topology()->add_curve(fea::topology::curves::type::line, { 3,  4});
	model.topology()->add_curve(fea::topology::curves::type::line, { 4,  5});
	model.topology()->add_curve(fea::topology::curves::type::line, { 5,  6});
	model.topology()->add_curve(fea::topology::curves::type::line, { 6,  7});
	model.topology()->add_curve(fea::topology::curves::type::line, { 7,  8});
	model.topology()->add_curve(fea::topology::curves::type::line, { 8,  9});
	model.topology()->add_curve(fea::topology::curves::type::line, { 9, 10});
	model.topology()->add_curve(fea::topology::curves::type::line, {10, 11});
	model.topology()->add_curve(fea::topology::curves::type::line, {11,  0});
	//surfaces
	//model.topology()->add_surface({{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}});
}
static void topology_2(fea::models::Model& model)
{
	//points
	model.topology()->add_point(-tw / 2, -hw / 2, 0);
	model.topology()->add_point(+tw / 2, -hw / 2, 0);
	model.topology()->add_point(+wf / 2, -tf / 2, 0);
	model.topology()->add_point(+wf / 2, +tf / 2, 0);
	model.topology()->add_point(+tw / 2, +hw / 2, 0);
	model.topology()->add_point(-tw / 2, +hw / 2, 0);
	model.topology()->add_point(-wf / 2, +tf / 2, 0);
	model.topology()->add_point(-wf / 2, -tf / 2, 0);
	model.topology()->add_point(-tw / 2 - r, -tf / 2, 0);
	model.topology()->add_point(-tw / 2, -tf / 2 - r, 0);
	model.topology()->add_point(+tw / 2, -tf / 2 - r, 0);
	model.topology()->add_point(+tw / 2 + r, -tf / 2, 0);
	model.topology()->add_point(+tw / 2 + r, +tf / 2, 0);
	model.topology()->add_point(+tw / 2, +tf / 2 + r, 0);
	model.topology()->add_point(-tw / 2, +tf / 2 + r, 0);
	model.topology()->add_point(-tw / 2 - r, +tf / 2, 0);
	model.topology()->add_point(-tw / 2 - r, -tf / 2 - r, 0);
	model.topology()->add_point(+tw / 2 + r, -tf / 2 - r, 0);
	model.topology()->add_point(+tw / 2 + r, +tf / 2 + r, 0);
	model.topology()->add_point(-tw / 2 - r, +tf / 2 + r, 0);
	//curves
	model.topology()->add_curve(fea::topology::curves::type::line, { 0,  1});
	model.topology()->add_curve(fea::topology::curves::type::line, { 1, 10});
	model.topology()->add_curve(fea::topology::curves::type::line, {11,  2});
	model.topology()->add_curve(fea::topology::curves::type::line, { 2,  3});
	model.topology()->add_curve(fea::topology::curves::type::line, { 3, 12});
	model.topology()->add_curve(fea::topology::curves::type::line, {13,  4});
	model.topology()->add_curve(fea::topology::curves::type::line, { 4,  5});
	model.topology()->add_curve(fea::topology::curves::type::line, { 5, 14});
	model.topology()->add_curve(fea::topology::curves::type::line, {15,  6});
	model.topology()->add_curve(fea::topology::curves::type::line, { 6,  7});
	model.topology()->add_curve(fea::topology::curves::type::line, { 7,  8});
	model.topology()->add_curve(fea::topology::curves::type::line, { 9,  0});
	model.topology()->add_curve(fea::topology::curves::type::circle_arc, {10, 17, 11});
	model.topology()->add_curve(fea::topology::curves::type::circle_arc, {12, 18, 13});
	model.topology()->add_curve(fea::topology::curves::type::circle_arc, {14, 19, 15});
	model.topology()->add_curve(fea::topology::curves::type::circle_arc, { 8, 16,  9});
	//surfaces
	//model.topology()->add_surface({{0, 1, 12, 2, 3, 4, 13, 5, 6, 7, 14, 8, 9, 10, 15, 11}});
}

void tests::warping::static_linear::profile_X(void)
{
	//model
	fea::models::Model model("profile X", "benchmarks/warping/static/linear");

	//topology
	r == 0 ? topology_1(model) : topology_2(model);

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::tri6);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);

	//surfaces
	model.topology()->surface(0)->cell(0);
	model.topology()->surface(0)->material(0);
	model.topology()->surface(0)->element(fea::mesh::elements::type::warping);

	//mesh
	model.topology()->order(2);
	model.topology()->size(tw / 3);
	model.topology()->mesh(2);

	//save
	model.save();
}