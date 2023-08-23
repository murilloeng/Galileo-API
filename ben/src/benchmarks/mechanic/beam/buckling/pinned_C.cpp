//std
#include <cmath>

//fea
#include "fea/inc/Model/Model.h"

#include "fea/inc/Mesh/Mesh.h"
#include "fea/inc/Mesh/Nodes/Dof.h"
#include "fea/inc/Mesh/Cells/Types.h"
#include "fea/inc/Mesh/Sections/Types.h"
#include "fea/inc/Mesh/Elements/Types.h"
#include "fea/inc/Mesh/Materials/Types.h"
#include "fea/inc/Mesh/Sections/Profile.h"
#include "fea/inc/Mesh/Materials/Mechanic/Steel.h"
#include "fea/inc/Mesh/Cells/Quadrature/Quadrature.h"
#include "fea/inc/Mesh/Elements/Mechanic/Frame/Beam3.h"
#include "fea/inc/Mesh/Elements/Mechanic/Frame/Warping.h"

#include "fea/inc/Boundary/Boundary.h"

#include "fea/inc/Topology/Topology.h"
#include "fea/inc/Topology/Curves/Types.h"
#include "fea/inc/Topology/Curves/Curve.h"

#include "fea/inc/Analysis/Analysis.h"
#include "fea/inc/Analysis/Solvers/Types.h"
#include "fea/inc/Analysis/Solvers/Buckling.h"

//ben
#include "ben/inc/benchmarks/mechanic/beam.h"

void tests::beam::buckling::pinned_C(void)
{
	//data
	const double L = 1.50e+00;
	const double w = 1.00e-01;
	const double h = 9.60e-02;
	const double t = 2.00e-03;
	const double E = 2.10e+11;
	const double v = 3.00e-01;
	const double P = 1.00e+03;
	const double s = 1.00e+02;

	//model
	fea::models::Model model("pinned C", "benchmarks/beam/buckling");

	//sections
	model.mesh()->add_section(fea::mesh::sections::type::profile_C);
	((fea::mesh::sections::Profile*) model.mesh()->section(0))->size(t / 2);
	((fea::mesh::sections::Profile*) model.mesh()->section(0))->web_height(h);
	((fea::mesh::sections::Profile*) model.mesh()->section(0))->web_thickness(t);
	((fea::mesh::sections::Profile*) model.mesh()->section(0))->flange_top_width(w);
	((fea::mesh::sections::Profile*) model.mesh()->section(0))->flange_bottom_width(w);
	((fea::mesh::sections::Profile*) model.mesh()->section(0))->flange_top_thickness(t);
	((fea::mesh::sections::Profile*) model.mesh()->section(0))->flange_bottom_thickness(t);

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::beam);
	model.mesh()->cell(0)->quadrature()->order(5);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);

	//points
	model.topology()->add_point(0, 0, 0);
	model.topology()->add_point(L, 0, 0);

	//curves
	model.topology()->add_curve(fea::topology::curves::type::line, {0, 1});
	model.topology()->curve(0)->cell(0);
	model.topology()->curve(0)->material(0);
	model.topology()->curve(0)->structured(20);
	model.topology()->curve(0)->element(fea::mesh::elements::type::beam3C);

	//mesh
	model.topology()->mesh(1);

	//supports
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_1);
	model.boundary()->add_support(1, fea::mesh::nodes::dof::rotation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_3);
	model.boundary()->add_support(1, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(1, fea::mesh::nodes::dof::translation_3);

	//loads
	model.boundary()->add_load_set();
	model.boundary()->add_load_case(1, fea::mesh::nodes::dof::translation_1, -P);

	//setup
	fea::mesh::elements::Beam3::geometric(true);
	fea::mesh::elements::Beam3::high_order(true);
	fea::mesh::elements::Beam3::warping(fea::mesh::elements::warping::vlasov);

	//solver
	model.analysis()->solver(fea::analysis::solvers::type::buckling);
	model.analysis()->solver()->watch_dof(1, fea::mesh::nodes::dof::translation_2);
	dynamic_cast<fea::analysis::solvers::Buckling*>(model.analysis()->solver())->scale(s);
	dynamic_cast<fea::analysis::solvers::Buckling*>(model.analysis()->solver())->load_set(0);
	dynamic_cast<fea::analysis::solvers::Buckling*>(model.analysis()->solver())->spectre(fea::analysis::solvers::spectre::full);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}