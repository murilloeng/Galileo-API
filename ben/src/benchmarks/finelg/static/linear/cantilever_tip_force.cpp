//fea
#include "Model/Model.h"

#include "Mesh/Mesh.h"
#include "Mesh/Nodes/Dof.h"
#include "Mesh/Cells/Types.h"
#include "Mesh/Sections/Types.h"
#include "Mesh/Elements/Types.h"
#include "Mesh/Cells/Line/Line.h"
#include "Mesh/Materials/Types.h"
#include "Mesh/Sections/Profile.h"
#include "Mesh/Materials/Mechanic/Steel.h"
#include "Mesh/Elements/Mechanic/Mechanic.h"
#include "Mesh/Elements/Mechanic/Frame/Beam3.h"
#include "Mesh/Elements/Mechanic/Frame/Warping.h"

#include "Boundary/Boundary.h"

#include "Topology/Topology.h"
#include "Topology/Curves/Curve.h"
#include "Topology/Curves/Types.h"

#include "Analysis/Analysis.h"
#include "Analysis/Solvers/Types.h"
#include "Analysis/Solvers/Static_Linear.h"

//ben
#include "benchmarks/finelg/finelg.h"

void tests::finelg::static_linear::cantilever_tip_force(void)
{
	//data
	const unsigned ne = 10;
	const double L = 1.00e+00;
	const double w = 1.00e-01;
	const double h = 1.00e-01;
	const double t = 1.00e-02;
	const double v = 3.00e-01;
	const double E = 2.00e+11;
	const double P = 1.00e+04;

	//model
	fea::models::Model model("cantilever tip force", "benchmarks/finelg/static/linear");

	//points
	model.topology()->add_point(0, 0, 0);
	model.topology()->add_point(L, 0, 0);

	//sections
	model.mesh()->add_section(fea::mesh::sections::type::profile_I);
	model.mesh()->add_section(fea::mesh::sections::type::profile_T);
	model.mesh()->add_section(fea::mesh::sections::type::profile_C);
	model.mesh()->add_section(fea::mesh::sections::type::profile_L);
	for(unsigned i = 0; i < 4; i++)
	{
		((fea::mesh::sections::Profile*) model.mesh()->section(i))->web_height(h);
		((fea::mesh::sections::Profile*) model.mesh()->section(i))->web_thickness(t);
		((fea::mesh::sections::Profile*) model.mesh()->section(i))->flange_top_width(w);
		((fea::mesh::sections::Profile*) model.mesh()->section(i))->flange_bottom_width(w);
		((fea::mesh::sections::Profile*) model.mesh()->section(i))->flange_top_thickness(t);
		((fea::mesh::sections::Profile*) model.mesh()->section(i))->flange_bottom_thickness(t);
	}

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::beam);
	model.mesh()->add_cell(fea::mesh::cells::type::beam);
	model.mesh()->add_cell(fea::mesh::cells::type::beam);
	model.mesh()->add_cell(fea::mesh::cells::type::beam);
	((fea::mesh::cells::Line*) model.mesh()->cell(0))->section(0);
	((fea::mesh::cells::Line*) model.mesh()->cell(1))->section(1);
	((fea::mesh::cells::Line*) model.mesh()->cell(2))->section(2);
	((fea::mesh::cells::Line*) model.mesh()->cell(3))->section(3);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->poisson_ratio(v);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->elastic_modulus(E);

	//curves
	model.topology()->add_curve(fea::topology::curves::type::line, {0, 1});

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
	dynamic_cast<fea::analysis::solvers::Static_Linear*>(model.analysis()->solver())->step_max(100);
	dynamic_cast<fea::analysis::solvers::Static_Linear*>(model.analysis()->solver())->watch_dof(1, fea::mesh::nodes::dof::translation_2);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}