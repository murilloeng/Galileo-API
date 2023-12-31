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
#include "fea/inc/Mesh/Sections/Profile_I.h"
#include "fea/inc/Mesh/Materials/Mechanic/Steel.h"
#include "fea/inc/Mesh/Elements/Mechanic/Mechanic.h"
#include "fea/inc/Mesh/Elements/Mechanic/Frame/Beam.h"

#include "fea/inc/Boundary/Boundary.h"
#include "fea/inc/Boundary/Loads/Types.h"
#include "fea/inc/Boundary/Loads/Load_Case.h"
#include "fea/inc/Boundary/Loads/Elements/Element.h"

#include "fea/inc/Topology/Topology.h"
#include "fea/inc/Topology/Curves/Curve.h"
#include "fea/inc/Topology/Curves/Types.h"

#include "fea/inc/Analysis/Analysis.h"
#include "fea/inc/Analysis/Solvers/Types.h"
#include "fea/inc/Analysis/Strategies/Types.h"
#include "fea/inc/Analysis/Solvers/Buckling.h"

//ben
#include "ben/inc/benchmarks/finelg/finelg.h"

void tests::finelg::buckling::pinned_bending(void)
{
	//data
	const unsigned n = 10;
	const double L = 1.60e+01;
	const double E = 2.10e+08;
	const double v = 3.00e-01;
	const double hw = 1.65e+00;
	const double wt = 6.00e-01;
	const double wb = 3.00e-01;
	const double tw = 1.20e-02;
	const double tt = 1.60e-02;
	const double tb = 2.00e-02;

	//model
	fea::models::Model model("pinned bending", "benchmarks/finelg/buckling");

	//sections
	model.mesh()->add_section(fea::mesh::sections::type::profile_I);
	((fea::mesh::sections::Profile_I*) model.mesh()->section(0))->web_height(hw);
	((fea::mesh::sections::Profile_I*) model.mesh()->section(0))->web_thickness(tw);
	((fea::mesh::sections::Profile_I*) model.mesh()->section(0))->flange_top_width(wt);
	((fea::mesh::sections::Profile_I*) model.mesh()->section(0))->flange_bottom_width(wb);
	((fea::mesh::sections::Profile_I*) model.mesh()->section(0))->flange_top_thickness(tt);
	((fea::mesh::sections::Profile_I*) model.mesh()->section(0))->flange_bottom_thickness(tb);

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::beam);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->poisson_ratio(v);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->elastic_modulus(E);

	//points
	model.topology()->add_point(0, 0, 0);
	model.topology()->add_point(L, 0, 0);

	//curves
	model.topology()->add_curve(fea::topology::curves::type::line, {0, 1});
	((fea::topology::curves::Curve*) model.topology()->curve(0))->cell(0);
	((fea::topology::curves::Curve*) model.topology()->curve(0))->material(0);
	((fea::topology::curves::Curve*) model.topology()->curve(0))->structured(n);
	((fea::topology::curves::Curve*) model.topology()->curve(0))->element(fea::mesh::elements::type::beam3C);

	//topology
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
	model.boundary()->add_load_case(fea::boundary::loads::type::line_force, {}, -10);
	for(unsigned i = 0; i < n; i++)
	{
		model.boundary()->load_case(0)->load_element(0)->elements().push_back(i);
	}

	//solver
	fea::mesh::elements::Mechanic::geometric(true);
	model.analysis()->solver(fea::analysis::solvers::type::buckling);
	model.analysis()->solver()->watch_dof(n / 2 + 1, fea::mesh::nodes::dof::rotation_1);
	dynamic_cast<fea::analysis::solvers::Buckling*>(model.analysis()->solver())->scale(1);
	dynamic_cast<fea::analysis::solvers::Buckling*>(model.analysis()->solver())->load_set(0);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}