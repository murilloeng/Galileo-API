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
#include "Mesh/Elements/Mechanic/Frame/Beam2.h"

#include "Boundary/Boundary.h"
#include "Boundary/Loads/Load_Case.h"
#include "Boundary/Loads/Nodes/Node.h"

#include "Analysis/Analysis.h"
#include "Analysis/Solvers/Types.h"
#include "Analysis/Solvers/Dynamic_Nonlinear.h"

//ben
#include "benchmarks/mechanic/beam.h"

void tests::beam::dynamic_nonlinear::elastic::plane::arc_shallow(void)
{
	/*
	Shallow arc
	Literature: T. N. Le et al. (2011) - Computational Mechanics - pp. 153 - 161
	 */

	//data
	const unsigned n = 10;
	const double a = M_PI / 6;
	const double R = 1.00e+01;
	const double r = 7.85e+03;
	const double E = 2.10e+11;
	const double A = 8.70e-02;
	const double I = 3.56e-03;
	const double P = 8.00e+07;
	const double f = 1.00e+03;
	const double L = R * sin(a);
	const double h = sqrt(12 * I / A);
	const double w = 12 * I / h / h / h;

	//model
	fea::models::Model model("arc shallow", "benchmarks/beam/dynamic/nonlinear/elastic/plane");

	//nodes
	for(unsigned i = 0; i <= n; i++)
	{
		const double t = 2 * a * i / n - a + M_PI / 2;
		model.mesh()->add_node(R * cos(t), R * sin(t), 0);
	}

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
	for(unsigned i = 0; i < n; i++)
	{
		model.mesh()->add_element(fea::mesh::elements::type::beam2C, {i, i + 1});
	}

	//supports
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_3);
	model.boundary()->add_support(n, fea::mesh::nodes::dof::rotation_1);
	model.boundary()->add_support(n, fea::mesh::nodes::dof::rotation_2);
	model.boundary()->add_support(n, fea::mesh::nodes::dof::rotation_3);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_3);
	model.boundary()->add_support(n, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(n, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(n, fea::mesh::nodes::dof::translation_3);

	//loads
	model.boundary()->add_load_case();
	// model.boundary()->load_case(0)->add_load_node(n / 2, fea::mesh::nodes::dof::translation_2, -P, [f](double t) { return sin(f * t); });

	//solver
	fea::mesh::elements::Mechanic::geometric(true);
	model.analysis()->solver(fea::analysis::solvers::type::dynamic_nonlinear);
	dynamic_cast<fea::analysis::solvers::Dynamic_Nonlinear*> (model.analysis()->solver())->time_max(0.05);
	dynamic_cast<fea::analysis::solvers::Dynamic_Nonlinear*> (model.analysis()->solver())->step_max(1500);
	dynamic_cast<fea::analysis::solvers::Dynamic_Nonlinear*> (model.analysis()->solver())->newmark().hht(true);
	dynamic_cast<fea::analysis::solvers::Dynamic_Nonlinear*> (model.analysis()->solver())->newmark().alpha(0.1);
	dynamic_cast<fea::analysis::solvers::Dynamic_Nonlinear*> (model.analysis()->solver())->watch_dof(n / 2, fea::mesh::nodes::dof::translation_2);
	dynamic_cast<fea::analysis::solvers::Dynamic_Nonlinear*> (model.analysis()->solver())->integration(fea::analysis::solvers::integration::newmark);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}