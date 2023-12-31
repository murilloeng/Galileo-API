//fea
#include "fea/inc/Model/Model.h"

#include "fea/inc/Mesh/Mesh.h"
#include "fea/inc/Mesh/Nodes/Dof.h"
#include "fea/inc/Mesh/Cells/Cell.h"
#include "fea/inc/Mesh/Cells/Types.h"
#include "fea/inc/Mesh/Sections/Types.h"
#include "fea/inc/Mesh/Elements/Types.h"
#include "fea/inc/Mesh/Materials/Types.h"
#include "fea/inc/Mesh/Sections/Rectangle.h"
#include "fea/inc/Mesh/Materials/Mechanic/Steel.h"
#include "fea/inc/Mesh/Cells/Quadrature/Quadrature.h"
#include "fea/inc/Mesh/Elements/Mechanic/Frame/Beam3.h"
#include "fea/inc/Mesh/Elements/Mechanic/Frame/Warping.h"

#include "fea/inc/Topology/Topology.h"
#include "fea/inc/Topology/Curves/Types.h"
#include "fea/inc/Topology/Curves/Curve.h"

#include "fea/inc/Boundary/Boundary.h"

#include "fea/inc/Analysis/Analysis.h"
#include "fea/inc/Analysis/Solvers/Types.h"
#include "fea/inc/Analysis/Solvers/Static_Linear.h"

//ben
#include "ben/inc/benchmarks/mechanic/beam.h"

void tests::beam::static_linear::cantilever_tip_force(void)
{
	//data
	const unsigned ne = 10;
	const double L = 1.20e+01;
	const double w = 2.00e-01;
	const double h = 6.00e-01;
	const double v = 3.00e-01;
	const double E = 2.00e+11;
	const double P = 1.00e+04;

	//model
	fea::models::Model model("cantilever tip force", "benchmarks/beam/static/linear");

	//points
	model.topology()->add_point(0, 0, 0);
	model.topology()->add_point(L, 0, 0);

	//curves
	model.topology()->add_curve(fea::topology::curves::type::line, {0, 1});

	//sections
	model.mesh()->add_section(fea::mesh::sections::type::rectangle);
	((fea::mesh::sections::Rectangle*) model.mesh()->section(0))->width(w);
	((fea::mesh::sections::Rectangle*) model.mesh()->section(0))->height(h);

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::beam);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->poisson_ratio(v);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->elastic_modulus(E);

	//mesh
	model.topology()->curve(0)->cell(0);
	model.topology()->curve(0)->material(0);
	model.topology()->curve(0)->structured(ne);
	model.topology()->curve(0)->element(fea::mesh::elements::type::beam3C);
	model.topology()->mesh(1);

	//supports
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_3);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_3);

	//loads
	model.boundary()->add_load_set();
	model.boundary()->add_load_case(1, fea::mesh::nodes::dof::translation_2, -P);

	//solver
	model.analysis()->solver(fea::analysis::solvers::type::static_linear);
	fea::mesh::elements::Beam3::warping(fea::mesh::elements::warping::saint_venant);
	dynamic_cast<fea::analysis::solvers::Static_Linear*>(model.analysis()->solver())->load_set(0);
	dynamic_cast<fea::analysis::solvers::Static_Linear*>(model.analysis()->solver())->watch_dof(1, fea::mesh::nodes::dof::translation_2);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}