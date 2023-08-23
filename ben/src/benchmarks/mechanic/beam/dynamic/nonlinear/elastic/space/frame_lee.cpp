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
#include "fea/inc/Mesh/Sections/Rectangle.h"
#include "fea/inc/Mesh/Materials/Mechanic/Steel.h"
#include "fea/inc/Mesh/Elements/Mechanic/Mechanic.h"
#include "fea/inc/Mesh/Elements/Mechanic/Frame/Beam2.h"

#include "fea/inc/Boundary/Boundary.h"
#include "fea/inc/Boundary/Loads/Load_Case.h"
#include "fea/inc/Boundary/Loads/Nodes/Node.h"

#include "fea/inc/Analysis/Analysis.h"
#include "fea/inc/Analysis/Solvers/Types.h"
#include "fea/inc/Analysis/Solvers/Dynamic_Nonlinear.h"

//ben
#include "ben/inc/benchmarks/mechanic/beam.h"

void tests::beam::dynamic_nonlinear::elastic::space::frame_lee(void)
{
	/*
	Lee's frame
	Literature: T. N. Le et al. (2014) - Computer Methods in Applied Mechanics and Engineering - pp. 538 - 565
	 */

	//data
	const double L = 6.00e+00;
	const double r = 7.85e+03;
	const double E = 2.10e+11;
	const double w = 3.00e-01;
	const double h = 2.00e-01;
	const double P = 3.00e+06;
	const double T = 1.00e-01;

	//model
	fea::models::Model model("frame lee", "benchmarks/beam/dynamic/nonlinear/elastic/space");

	//nodes
	model.mesh()->add_node(0 * L, 0 * L, 0);
	model.mesh()->add_node(0 * L, 1 * L, 0);
	model.mesh()->add_node(0 * L, 2 * L, 0);
	model.mesh()->add_node(1 * L, 2 * L, 0);
	model.mesh()->add_node(2 * L, 2 * L, 0);

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::beam);

	//sections
	model.mesh()->add_section(fea::mesh::sections::type::rectangle);
	((fea::mesh::sections::Rectangle*) model.mesh()->section(0))->width(w);
	((fea::mesh::sections::Rectangle*) model.mesh()->section(0))->height(h);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->specific_mass(r);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->elastic_modulus(E);

	//elements
	model.mesh()->add_element(fea::mesh::elements::type::beam3T, {0, 1});
	model.mesh()->add_element(fea::mesh::elements::type::beam3T, {1, 2});
	model.mesh()->add_element(fea::mesh::elements::type::beam3T, {2, 3});
	model.mesh()->add_element(fea::mesh::elements::type::beam3T, {3, 4});

	//refine
	fea::mesh::cells::Line::refine(0, 10);
	fea::mesh::cells::Line::refine(1, 10);
	fea::mesh::cells::Line::refine(2, 10);
	fea::mesh::cells::Line::refine(3, 10);

	//supports
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_3);
	model.boundary()->add_support(4, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(4, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(4, fea::mesh::nodes::dof::translation_3);

	//loads
	model.boundary()->add_load_case();
	model.boundary()->load_case(0)->add_load_node(1, fea::mesh::nodes::dof::translation_3, -P);
	model.boundary()->load_case(0)->add_load_node(3, fea::mesh::nodes::dof::translation_3, +P);
	// model.boundary()->load_case(0)->load_node(0)->function([T](double t) { return t < T ? t / T : t < 2 * T ? 2 - t / T : 0; });
	// model.boundary()->load_case(0)->load_node(1)->function([T](double t) { return t < T ? t / T : t < 2 * T ? 2 - t / T : 0; });

	//solver
	fea::mesh::elements::Mechanic::geometric(true);
	model.analysis()->solver(fea::analysis::solvers::type::dynamic_nonlinear);
	dynamic_cast<fea::analysis::solvers::Dynamic_Nonlinear*> (model.analysis()->solver())->step_max(3000);
	dynamic_cast<fea::analysis::solvers::Dynamic_Nonlinear*> (model.analysis()->solver())->time_max(8.00e-01);
	dynamic_cast<fea::analysis::solvers::Dynamic_Nonlinear*> (model.analysis()->solver())->tolerance(1.00e-00);
	dynamic_cast<fea::analysis::solvers::Dynamic_Nonlinear*> (model.analysis()->solver())->newmark().hht(false);
	dynamic_cast<fea::analysis::solvers::Dynamic_Nonlinear*> (model.analysis()->solver())->newmark().alpha(0.0);
	dynamic_cast<fea::analysis::solvers::Dynamic_Nonlinear*> (model.analysis()->solver())->watch_dof(3, fea::mesh::nodes::dof::translation_3);
	dynamic_cast<fea::analysis::solvers::Dynamic_Nonlinear*> (model.analysis()->solver())->integration(fea::analysis::solvers::integration::newmark);
	dynamic_cast<fea::analysis::solvers::Dynamic_Nonlinear*> (model.analysis()->solver())->convergence((unsigned) fea::analysis::solvers::convergence::fixed);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}