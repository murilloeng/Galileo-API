//std
#include <cmath>

//fea
#include "Model/Model.h"

#include "Mesh/Mesh.h"
#include "Mesh/Nodes/Dof.h"
#include "Mesh/Cells/Cell.h"
#include "Mesh/Cells/Types.h"
#include "Mesh/Joints/Types.h"
#include "Mesh/Sections/Types.h"
#include "Mesh/Elements/Types.h"
#include "Mesh/Materials/Types.h"
#include "Mesh/Sections/Profile_C.h"
#include "Mesh/Materials/Mechanic/Steel.h"
#include "Mesh/Elements/Mechanic/Mechanic.h"
#include "Mesh/Cells/Quadrature/Quadrature.h"
#include "Mesh/Elements/Mechanic/Frame/Beam3.h"
#include "Mesh/Elements/Mechanic/Frame/Warping.h"

#include "Topology/Topology.h"
#include "Topology/Curves/Curve.h"
#include "Topology/Curves/Types.h"

#include "Boundary/Boundary.h"

#include "Analysis/Analysis.h"
#include "Analysis/Solvers/Types.h"
#include "Analysis/Strategies/Types.h"
#include "Analysis/Solvers/Static_Nonlinear.h"

//ben
#include "benchmarks/mechanic/beam.h"

void tests::beam::static_nonlinear::elastic::space::cantilever_C(void)
{
	/*
	Cantilever C
	Literature: F. Gruttmann et al. (1998) Computer methods in applied mechanics and engineering pp. 383 - 400
	*/

	//data
	const double L = 9.00e+00;
	const double h = 2.68e-01;
	const double w = 1.00e-01;
	const double t = 1.60e-02;
	const double s = 1.00e-02;
	const double v = 3.00e-01;
	const double E = 2.10e+11;
	const double P = 1.00e+03;
	const double z = (s * h * s / 2 + 2 * w * t * w / 2) / (s * h + 2 * w * t) - s / 2;

	//model
	fea::models::Model model("cantilever C", "benchmarks/beam/static/nonlinear/elastic/space");

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::beam);

	//sections
	model.mesh()->add_section(fea::mesh::sections::type::profile_C);
	((fea::mesh::sections::Profile_C*) model.mesh()->section(0))->size(s / 2);
	((fea::mesh::sections::Profile_C*) model.mesh()->section(0))->web_height(h);
	((fea::mesh::sections::Profile_C*) model.mesh()->section(0))->web_thickness(s);
	((fea::mesh::sections::Profile_C*) model.mesh()->section(0))->flange_top_width(w);
	((fea::mesh::sections::Profile_C*) model.mesh()->section(0))->flange_bottom_width(w);
	((fea::mesh::sections::Profile_C*) model.mesh()->section(0))->flange_top_thickness(t);
	((fea::mesh::sections::Profile_C*) model.mesh()->section(0))->flange_bottom_thickness(t);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->poisson_ratio(v);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->elastic_modulus(E);

	//points
	model.topology()->add_point(0, 0, 0);
	model.topology()->add_point(L, 0, 0);
	model.topology()->add_point(L, h / 2 + t, z);

	//curves
	model.topology()->add_curve(fea::topology::curves::type::line, {0, 1});
	model.topology()->curve(0)->cell(0);
	model.topology()->curve(0)->material(0);
	model.topology()->curve(0)->structured(20);
	model.topology()->curve(0)->element(fea::mesh::elements::type::beam3C);

	//mesh
	model.topology()->mesh(1);

	//joints
	model.mesh()->add_joint(fea::mesh::joints::type::rigid3, {1, 2});

	//supports
	model.boundary()->add_support(0, fea::mesh::nodes::dof::warping_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::warping_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::warping_3);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_3);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_3);

	//loads
	model.boundary()->add_load_set();
	model.boundary()->add_load_case(2, fea::mesh::nodes::dof::translation_2, -P);

	//setup
	fea::mesh::elements::Beam3::geometric(true);
	fea::mesh::elements::Beam3::high_order(true);
	fea::mesh::elements::Beam3::warping(fea::mesh::elements::warping::vlasov);
	model.mesh()->cell(0)->quadrature()->order(5);

	//solver
	model.analysis()->solver(fea::analysis::solvers::type::static_nonlinear);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_set(0);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_max(20);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->step_max(2000);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->iteration_max(20);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_guess(2.00e-2);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->watch_dof(2, fea::mesh::nodes::dof::translation_2);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->strategy(fea::analysis::strategies::type::minimal_norm);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}