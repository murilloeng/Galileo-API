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
const static double r = 0.00e-02;
const static double hw = 1.00e-01;
const static double tw = 1.00e-02;
const static double wbf = 1.00e-01;
const static double tbf = 1.00e-02;
const static double wtf = 1.00e-01;
const static double ttf = 1.00e-02;

//misc
static void topology_1(fea::models::Model& model)
{
	//points
	model.topology()->add_point(0, 0, 0);
	model.topology()->add_point(wbf, 0, 0);
	model.topology()->add_point(tw, tbf, 0);
	model.topology()->add_point(wbf, tbf, 0);
	model.topology()->add_point(tw, tbf + hw, 0);
	model.topology()->add_point(wtf, tbf + hw, 0);
	model.topology()->add_point(0, tbf + hw + ttf, 0);
	model.topology()->add_point(wtf, tbf + hw + ttf, 0);
	//curves
	model.topology()->add_curve(fea::topology::curves::type::line, { 0,  1});
	model.topology()->add_curve(fea::topology::curves::type::line, { 1,  3});
	model.topology()->add_curve(fea::topology::curves::type::line, { 3,  2});
	model.topology()->add_curve(fea::topology::curves::type::line, { 2,  4});
	model.topology()->add_curve(fea::topology::curves::type::line, { 4,  5});
	model.topology()->add_curve(fea::topology::curves::type::line, { 5,  7});
	model.topology()->add_curve(fea::topology::curves::type::line, { 7,  6});
	model.topology()->add_curve(fea::topology::curves::type::line, { 6,  0});
	//surfaces
	//model.topology()->add_surface({{0, 1, 2, 3, 4, 5, 6, 7}});
}
static void topology_2(fea::models::Model& model)
{
	//points
	model.topology()->add_point(0, 0, 0);
	model.topology()->add_point(wbf, 0, 0);
	model.topology()->add_point(wbf, tbf, 0);
	model.topology()->add_point(tw + r, tbf, 0);
	model.topology()->add_point(tw, tbf + r, 0);
	model.topology()->add_point(wtf, tbf + hw, 0);
	model.topology()->add_point(tw + r, tbf + r, 0);
	model.topology()->add_point(tw, tbf + hw - r, 0);
	model.topology()->add_point(tw + r, tbf + hw, 0);
	model.topology()->add_point(0, tbf + hw + ttf, 0);
	model.topology()->add_point(wtf, tbf + hw + ttf, 0);
	model.topology()->add_point(tw + r, tbf + hw - r, 0);
	//curves
	model.topology()->add_curve(fea::topology::curves::type::line, { 0,  1});
	model.topology()->add_curve(fea::topology::curves::type::line, { 1,  2});
	model.topology()->add_curve(fea::topology::curves::type::line, { 2,  3});
	model.topology()->add_curve(fea::topology::curves::type::line, { 4,  7});
	model.topology()->add_curve(fea::topology::curves::type::line, { 8,  5});
	model.topology()->add_curve(fea::topology::curves::type::line, { 5, 10});
	model.topology()->add_curve(fea::topology::curves::type::line, {10,  9});
	model.topology()->add_curve(fea::topology::curves::type::line, { 9,  0});
	model.topology()->add_curve(fea::topology::curves::type::circle_arc, { 3,  6,  4});
	model.topology()->add_curve(fea::topology::curves::type::circle_arc, { 7, 11,  8});
	//surfaces
	//model.topology()->add_surface({{0, 1, 2, 8, 3, 9, 4, 5, 6, 7}});
}

void tests::warping::static_linear::profile_C(void)
{
	//model
	fea::models::Model model("profile C", "benchmarks/warping/static/linear");

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