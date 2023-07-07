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
#include "Mesh/Sections/Rectangle.h"
#include "Mesh/Materials/Mechanic/Steel.h"
#include "Mesh/Elements/Mechanic/Mechanic.h"
#include "Mesh/Materials/Mechanic/Concrete.h"

#include "Boundary/Boundary.h"
#include "Boundary/Loads/Load_Case.h"

#include "Analysis/Analysis.h"
#include "Analysis/Solvers/Types.h"
#include "Analysis/Strategies/Types.h"
#include "Analysis/Solvers/Static_Nonlinear.h"

//ben
#include "benchmarks/mechanic/beam.h"

void tests::beam::static_nonlinear::inelastic::concrete::column_buckling(void)
{
	/*
	Concrete column buckling
	Literature: International Journal of Solids and Structures - S. Bratina et al. (2004) vol. 41 pp. 7181-7207
	 */

	//data
	const double L = 2.25e+00;
	const double w = 1.50e-01;
	const double h = 2.00e-01;
	const double c = 2.00e-02;
	const double d = 1.20e-02;

	const double es = 2.00e-02;
	const double bs = 4.66e+08;
	const double ss = 4.65e+08;
	const double Es = 2.00e+11;

	const double uc = 5.00e+00;
	const double ec = 3.50e-03;
	const double Ec = 2.86e+10;
	const double sc = 3.83e+07;
	const double tc = 3.83e+05;
	const double bc = 4.60e+07;

	//model
	fea::models::Model model("column buckling", "benchmarks/beam/static/nonlinear/inelastic/concrete");

	//nodes
	model.mesh()->add_node(0, 0, 0);
	model.mesh()->add_node(0, L, 0);

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::beam);

	//sections
	typedef fea::mesh::sections::Rebar rebar;
	model.mesh()->add_section(fea::mesh::sections::type::rectangle);
	((fea::mesh::sections::Rectangle*) model.mesh()->section(0))->width(w);
	((fea::mesh::sections::Rectangle*) model.mesh()->section(0))->height(h);
	((fea::mesh::sections::Rectangle*) model.mesh()->section(0))->rebars().push_back(rebar(d, (d - w) / 2 + c, (d - h) / 2 + c));
	((fea::mesh::sections::Rectangle*) model.mesh()->section(0))->rebars().push_back(rebar(d, (w - d) / 2 - c, (d - h) / 2 + c));
	((fea::mesh::sections::Rectangle*) model.mesh()->section(0))->rebars().push_back(rebar(d, (d - w) / 2 + c, (h - d) / 2 - c));
	((fea::mesh::sections::Rectangle*) model.mesh()->section(0))->rebars().push_back(rebar(d, (w - d) / 2 - c, (h - d) / 2 - c));

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->break_strain(es);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->break_stress(bs);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->yield_stress(ss);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->elastic_modulus(Es);

	model.mesh()->add_material(fea::mesh::materials::type::concrete);
	((fea::mesh::materials::Concrete*) model.mesh()->material(1))->softening(uc);
	((fea::mesh::materials::Concrete*) model.mesh()->material(1))->break_strain(ec);
	((fea::mesh::materials::Concrete*) model.mesh()->material(1))->elastic_modulus(Ec);
	((fea::mesh::materials::Concrete*) model.mesh()->material(1))->yield_stress_tension(tc);
	((fea::mesh::materials::Concrete*) model.mesh()->material(1))->yield_stress_biaxial(bc);
	((fea::mesh::materials::Concrete*) model.mesh()->material(1))->yield_stress_compression(sc);

	//elements
	model.mesh()->add_element(fea::mesh::elements::type::beam2C, {0, 1}, 1, 0);

	//supports
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_3);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_2);

	//loads
	model.boundary()->add_load_case();
	model.boundary()->load_case(0)->add_load_node(1, fea::mesh::nodes::dof::rotation_3, 7.5e3);
	model.boundary()->load_case(0)->add_load_node(1, fea::mesh::nodes::dof::translation_2, -500e3);

	//solver
	fea::mesh::elements::Mechanic::geometric(true);
	fea::mesh::elements::Mechanic::inelastic(true);
	model.analysis()->solver(fea::analysis::solvers::type::static_nonlinear);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->step_max(300);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_max(2.0);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_guess(0.01);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->strategy(fea::analysis::strategies::type::minimal_norm);

	//solve
//	model.analysis()->solve();

	//save
	model.save();
}