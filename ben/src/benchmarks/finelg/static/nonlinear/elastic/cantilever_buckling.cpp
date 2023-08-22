//std
#include <cmath>

//fea
#include "fea/inc/Model/Model.h"

#include "fea/inc/Mesh/Mesh.h"
#include "fea/inc/Mesh/Nodes/Dof.h"
#include "fea/inc/Mesh/Cells/Types.h"
#include "fea/inc/Mesh/Joints/Types.h"
#include "fea/inc/Mesh/Sections/Types.h"
#include "fea/inc/Mesh/Elements/Types.h"
#include "fea/inc/Mesh/Cells/Line/Line.h"
#include "fea/inc/Mesh/Materials/Types.h"
#include "fea/inc/Mesh/Sections/Rectangle.h"
#include "fea/inc/Mesh/Materials/Mechanic/Steel.h"
#include "fea/inc/Mesh/Elements/Mechanic/Mechanic.h"
#include "fea/inc/Mesh/Elements/Mechanic/Frame/Beam3.h"
#include "fea/inc/Mesh/Elements/Mechanic/Frame/Warping.h"

#include "fea/inc/Topology/Topology.h"
#include "fea/inc/Topology/Curves/Curve.h"
#include "fea/inc/Topology/Curves/Types.h"

#include "fea/inc/Boundary/Boundary.h"
#include "fea/inc/Boundary/Loads/Types.h"
#include "fea/inc/Boundary/Loads/Set_Case.h"
#include "fea/inc/Boundary/Loads/Load_Set.h"
#include "fea/inc/Boundary/Loads/Load_Case.h"
#include "fea/inc/Boundary/Loads/Elements/Element.h"
#include "fea/inc/Boundary/Loads/Elements/Mechanic/Mechanic.h"

#include "fea/inc/Analysis/Analysis.h"
#include "fea/inc/Analysis/Solvers/Types.h"
#include "fea/inc/Analysis/Strategies/Types.h"
#include "fea/inc/Analysis/Solvers/Static_Nonlinear.h"

//ben
#include "ben/inc/benchmarks/finelg/finelg.h"

void tests::finelg::static_nonlinear::elastic::cantilever_buckling(void)
{
	/*
	Cantilever buckling
	Literature: Phd Thesis V. V. Goyet (1989) pp. 8.31
	*/

	//data
	const double L = 8.00e+01;
	const double h = 2.00e+00;
	const double w = 1.24e-01;
	const double v = 3.00e-01;
	const double E = 2.90e+07;
	const double q = 6.95e-02;
	const double P = 1.20e+01;
	const double t = 5.45e-03;

	//model
	fea::models::Model model("cantilever buckling", "benchmarks/finelg/static/nonlinear/elastic");

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::beam);

	//sections
	model.mesh()->add_section(fea::mesh::sections::type::rectangle);
	((fea::mesh::sections::Rectangle*) model.mesh()->section(0))->width(w);
	((fea::mesh::sections::Rectangle*) model.mesh()->section(0))->height(h);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->poisson_ratio(v);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->elastic_modulus(E);

	//points
	model.topology()->add_point(0, 0, 0);
	model.topology()->add_point(L, 0, 0);

	//curves
	model.topology()->add_curve(fea::topology::curves::type::line, {0, 1});
	model.topology()->curve(0)->cell(0);
	model.topology()->curve(0)->material(0);
	model.topology()->curve(0)->structured(10);
	model.topology()->curve(0)->element(fea::mesh::elements::type::beam3C);

	//mesh
	model.topology()->mesh(1);

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
	model.boundary()->add_load_case();
	model.boundary()->add_load_case();
	model.boundary()->load_set(0)->add_set_case(false, 1, 1);
	model.boundary()->load_set(0)->set_case(0)->fixed(true);
	model.boundary()->load_case(0)->add_load_element(fea::boundary::loads::type::line_force, {}, -q);
	model.boundary()->load_case(1)->add_load_node(1, fea::mesh::nodes::dof::translation_2, -P * cos(t));
	model.boundary()->load_case(1)->add_load_node(1, fea::mesh::nodes::dof::translation_3, +P * sin(t));
	((fea::boundary::loads::Mechanic*) model.boundary()->load_case(0)->load_element(0))->direction(0, 1, 0);
	for(unsigned i = 0; i < model.mesh()->elements().size(); i++)
	{
		model.boundary()->load_case(0)->load_element(0)->elements().push_back(i);
	}

	//setup
	fea::mesh::elements::Beam3::geometric(true);
	fea::mesh::elements::Beam3::high_order(true);
	fea::mesh::elements::Beam3::warping(fea::mesh::elements::warping::saint_venant);

	//solver
	model.analysis()->solver(fea::analysis::solvers::type::static_nonlinear);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_set(0);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_max(1.0);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->step_max(2000);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->iteration_max(10);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_guess(5.00e-2);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->watch_dof(1, fea::mesh::nodes::dof::translation_3);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->strategy(fea::analysis::strategies::type::minimal_norm);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}