//std
#include <cmath>

//fea
#include "Model/Model.h"

#include "Mesh/Mesh.h"
#include "Mesh/Nodes/Dof.h"
#include "Mesh/Cells/Types.h"
#include "Mesh/Sections/Types.h"
#include "Mesh/Elements/Types.h"
#include "Mesh/Cells/Line/Line.h"
#include "Mesh/Materials/Types.h"
#include "Mesh/Sections/Profile_I.h"
#include "Mesh/Materials/Mechanic/Steel.h"
#include "Mesh/Elements/Mechanic/Mechanic.h"

#include "Boundary/Boundary.h"
#include "Boundary/Loads/Load_Case.h"
#include "Boundary/Loads/Nodes/Node.h"

#include "Analysis/Analysis.h"
#include "Analysis/Solvers/Types.h"
#include "Analysis/Solvers/Dynamic_Nonlinear.h"

//ben
#include "benchmarks/mechanic/beam.h"

void tests::beam::dynamic_nonlinear::elastic::space::cantilever_I(void)
{
	/*
	Cantilever beam with profile I
	Literature: T. N. Le et al. (2014) - Computer and Structures - pp. 112 - 127
	 */

	//data
	const double L = 1.50e+00;
	const double r = 7.85e+03;
	const double E = 2.10e+11;
	const double v = 3.30e-01;
	const double w = 5.00e-02;
	const double h = 8.00e-02;
	const double t = 1.00e-02;
	const double P = 1.00e+03;
	const double T = 5.00e-02;

	//model
	fea::models::Model model("cantilever I", "benchmarks/beam/dynamic/nonlinear/elastic/space");

	//nodes
	model.mesh()->add_node(0 * L, 0, 0);
	model.mesh()->add_node(1 * L, 0, 0);
	model.mesh()->add_node(2 * L, 0, 0);

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::beam);

	//sections
	model.mesh()->add_section(fea::mesh::sections::type::profile_I);
	((fea::mesh::sections::Profile_I*) model.mesh()->section(0))->web_height(h);
	((fea::mesh::sections::Profile_I*) model.mesh()->section(0))->web_thickness(t);
	((fea::mesh::sections::Profile_I*) model.mesh()->section(0))->flange_top_width(w);
	((fea::mesh::sections::Profile_I*) model.mesh()->section(0))->flange_bottom_width(w);
	((fea::mesh::sections::Profile_I*) model.mesh()->section(0))->flange_top_thickness(t);
	((fea::mesh::sections::Profile_I*) model.mesh()->section(0))->flange_bottom_thickness(t);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->specific_mass(r);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->elastic_modulus(E);

	//elements
	model.mesh()->add_element(fea::mesh::elements::type::beam3C, {0, 1});
	model.mesh()->add_element(fea::mesh::elements::type::beam3C, {1, 2});

	//refine
	fea::mesh::cells::Line::refine(0, 10);
	fea::mesh::cells::Line::refine(1, 10);

	//supports
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_3);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_3);

	//loads
	model.boundary()->add_load_case();
	model.boundary()->load_case(0)->add_load_node(1, fea::mesh::nodes::dof::translation_2, +20 * P);
	model.boundary()->load_case(0)->add_load_node(1, fea::mesh::nodes::dof::translation_3, -10 * P);
	model.boundary()->load_case(0)->add_load_node(2, fea::mesh::nodes::dof::translation_2, -10 * P);
	model.boundary()->load_case(0)->add_load_node(2, fea::mesh::nodes::dof::translation_3, +5  * P);
	// model.boundary()->load_case(0)->load_node(0)->function([T](double t) { return t < T ? t / T : 2 - t / T; });
	// model.boundary()->load_case(0)->load_node(1)->function([T](double t) { return t < T ? t / T : 2 - t / T; });
	// model.boundary()->load_case(0)->load_node(2)->function([T](double t) { return t < T ? t / T : 2 - t / T; });
	// model.boundary()->load_case(0)->load_node(3)->function([T](double t) { return t < T ? t / T : 2 - t / T; });

	//solver
	fea::mesh::elements::Mechanic::geometric(true);
	model.analysis()->solver(fea::analysis::solvers::type::dynamic_nonlinear);
	dynamic_cast<fea::analysis::solvers::Dynamic_Nonlinear*>(model.analysis()->solver())->time_max(2.0);
	dynamic_cast<fea::analysis::solvers::Dynamic_Nonlinear*>(model.analysis()->solver())->step_max(1500);
	dynamic_cast<fea::analysis::solvers::Dynamic_Nonlinear*>(model.analysis()->solver())->newmark().hht(true);
	dynamic_cast<fea::analysis::solvers::Dynamic_Nonlinear*>(model.analysis()->solver())->newmark().alpha(0.05);
	dynamic_cast<fea::analysis::solvers::Dynamic_Nonlinear*>(model.analysis()->solver())->watch_dof(1, fea::mesh::nodes::dof::translation_2);
	dynamic_cast<fea::analysis::solvers::Dynamic_Nonlinear*>(model.analysis()->solver())->integration(fea::analysis::solvers::integration::newmark);

	//solve
//	model.analysis()->solve();

	//save
	model.save();
}