//std
#include <cmath>

//fea
#include "fea/inc/Model/Model.h"

#include "fea/inc/Mesh/Mesh.h"
#include "fea/inc/Mesh/Nodes/Dof.h"
#include "fea/inc/Mesh/Cells/Types.h"
#include "fea/inc/Mesh/Sections/Types.h"
#include "fea/inc/Mesh/Elements/Types.h"
#include "fea/inc/Mesh/Cells/Line/Line.h"
#include "fea/inc/Mesh/Materials/Types.h"
#include "fea/inc/Mesh/Sections/Profile_C.h"
#include "fea/inc/Mesh/Materials/Mechanic/Steel.h"
#include "fea/inc/Mesh/Elements/Mechanic/Mechanic.h"

#include "fea/inc/Boundary/Boundary.h"
#include "fea/inc/Boundary/Loads/Load_Case.h"
#include "fea/inc/Boundary/Loads/Nodes/Node.h"

#include "fea/inc/Analysis/Analysis.h"
#include "fea/inc/Analysis/Solvers/Types.h"
#include "fea/inc/Analysis/Solvers/Dynamic_Nonlinear.h"

//ben
#include "ben/inc/benchmarks/mechanic/beam.h"

void tests::beam::dynamic_nonlinear::elastic::space::cantilever_C(void)
{
	/*
	Cantilever beam with profile C
	Literature: T. N. Le et al. (2014) - Computer and Structures - pp. 112 - 127
	 */

	//data
	const double L = 9.00e+00;
	const double r = 7.85e+03;
	const double E = 2.10e+11;
	const double v = 3.30e-01;
	const double w = 1.00e-01;
	const double h = 2.68e-01;
	const double tw = 1.6e-02;
	const double th = 1.0e-02;
	const double T1 = 2.0e-01;
	const double T2 = 6.0e-01;
	const double P = 1.20e+04;

	//model
	fea::models::Model model("cantilever C", "benchmarks/beam/dynamic/nonlinear/elastic/space");

	//nodes
	model.mesh()->add_node(0, 0, 0);
	model.mesh()->add_node(L, 0, 0);

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::beam);

	//sections
	model.mesh()->add_section(fea::mesh::sections::type::profile_C);
	((fea::mesh::sections::Profile_C*) model.mesh()->section(0))->web_height(h);
	((fea::mesh::sections::Profile_C*) model.mesh()->section(0))->web_thickness(th);
	((fea::mesh::sections::Profile_C*) model.mesh()->section(0))->flange_top_width(w);
	((fea::mesh::sections::Profile_C*) model.mesh()->section(0))->flange_bottom_width(w);
	((fea::mesh::sections::Profile_C*) model.mesh()->section(0))->flange_top_thickness(tw);
	((fea::mesh::sections::Profile_C*) model.mesh()->section(0))->flange_bottom_thickness(tw);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->specific_mass(r);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->poisson_ratio(v);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->elastic_modulus(E);

	//elements
	model.mesh()->add_element(fea::mesh::elements::type::beam3C, {0, 1});

	//refine
	fea::mesh::cells::Line::refine(0, 40);

	//supports
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_3);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_3);

	//loads
	model.boundary()->add_load_case();
	model.boundary()->load_case(0)->add_load_node(1, fea::mesh::nodes::dof::translation_2, -P);
	// model.boundary()->load_case(0)->load_node(0)->function([T1, T2] (double t) { return t < T1 ? t / T1 : t < T2 ? (T2 - t) / (T2 - T1) : 0; });

	//solver
	fea::mesh::elements::Mechanic::geometric(true);
	model.analysis()->solver(fea::analysis::solvers::type::dynamic_nonlinear);
	dynamic_cast<fea::analysis::solvers::Dynamic_Nonlinear*> (model.analysis()->solver())->time_max(4.0);
	dynamic_cast<fea::analysis::solvers::Dynamic_Nonlinear*> (model.analysis()->solver())->step_max(1500);
	dynamic_cast<fea::analysis::solvers::Dynamic_Nonlinear*> (model.analysis()->solver())->newmark().hht(true);
	dynamic_cast<fea::analysis::solvers::Dynamic_Nonlinear*> (model.analysis()->solver())->newmark().alpha(0.05);
	dynamic_cast<fea::analysis::solvers::Dynamic_Nonlinear*> (model.analysis()->solver())->watch_dof(1, fea::mesh::nodes::dof::translation_2);
	dynamic_cast<fea::analysis::solvers::Dynamic_Nonlinear*> (model.analysis()->solver())->integration(fea::analysis::solvers::integration::newmark);

	//solve
//	model.analysis()->solve();

	//save
	model.save();
}