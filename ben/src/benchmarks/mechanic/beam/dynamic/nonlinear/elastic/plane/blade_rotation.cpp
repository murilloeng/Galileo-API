//std
#include <cmath>

//fea
#include "Mesh/Mesh.h"
#include "Model/Model.h"
#include "Mesh/Nodes/Dof.h"
#include "Mesh/Cells/Types.h"
#include "Analysis/Analysis.h"
#include "Boundary/Boundary.h"
#include "Boundary/Time/Time.h"
#include "Boundary/Time/Types.h"
#include "Mesh/Sections/Types.h"
#include "Mesh/Elements/Types.h"
#include "Mesh/Cells/Line/Line.h"
#include "Mesh/Materials/Types.h"
#include "Boundary/Time/Custom.h"
#include "Mesh/Sections/Generic.h"
#include "Analysis/Solvers/Types.h"
#include "Boundary/Supports/Support.h"
#include "Mesh/Materials/Mechanic/Steel.h"
#include "Mesh/Elements/Mechanic/Mechanic.h"
#include "Analysis/Solvers/Dynamic_Nonlinear.h"

//ben
#include "benchmarks/mechanic/beam.h"

void tests::beam::dynamic_nonlinear::elastic::plane::blade_rotation(void)
{
	/*
	Blade subjeted to rotation
	H. B. Coda and R. B. Paccola (2014) - Finite Elements in Analysis and Design - pp. 1 - 15
	 */

	//data
	const double L = 1.00e+01;
	const double r = 8.57e+03;
	const double E = 2.00e+11;
	const double A = 1.40e-04;
	const double I = 7.00e-08;

	//model
	fea::models::Model model("blade rotation", "benchmarks/beam/dynamic/nonlinear/elastic/plane");

	//nodes
	model.mesh()->add_node(0, 0, 0);
	model.mesh()->add_node(L, 0, 0);

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::beam);

	//sections
	model.mesh()->add_section(fea::mesh::sections::type::generic);
	((fea::mesh::sections::Generic*) model.mesh()->section(0))->area(A);
	((fea::mesh::sections::Generic*) model.mesh()->section(0))->inertia(0, 0, I);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->specific_mass(r);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->elastic_modulus(E);

	//elements
	model.mesh()->add_element(fea::mesh::elements::type::beam2C, {0, 1});

	//refine
	fea::mesh::cells::Line::refine(0, 10);

	//supports
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_3);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_3);
	for(unsigned i = 1; i < model.mesh()->nodes().size(); i++)
	{
		model.boundary()->add_support(i, fea::mesh::nodes::dof::rotation_1);
		model.boundary()->add_support(i, fea::mesh::nodes::dof::rotation_2);
		model.boundary()->add_support(i, fea::mesh::nodes::dof::translation_3);
	}

	//times
	std::function<double(double)> a = [](double t)
	{
		return t < 15 ? 0.4 * (1 - cos(2 * M_PI * t / 15)) : 0;
	};
	std::function<double(double)> v = [](double t)
	{
		return t < 15 ? 0.4 * (t - 15 / (2 * M_PI) * sin(2 * M_PI * t / 15)) : 6;
	};
	std::function<double(double)> u = [](double t)
	{
		return t < 15 ? 0.4 * (t * t / 2 + pow(15 / (2 * M_PI), 2) * (cos(2 * M_PI * t / 15) - 1)) : 6 * t - 45;
	};

	((fea::boundary::time::Custom*) model.boundary()->add_time(fea::boundary::time::type::custom))->function(u);
	((fea::boundary::time::Custom*) model.boundary()->add_time(fea::boundary::time::type::custom))->function(v);
	((fea::boundary::time::Custom*) model.boundary()->add_time(fea::boundary::time::type::custom))->function(a);

	model.boundary()->support(2)->state(0);
	model.boundary()->support(2)->velocity(1);
	model.boundary()->support(2)->acceleration(2);

	//loads
	model.boundary()->add_load_case();

	//solver
	fea::mesh::elements::Mechanic::geometric(true);
	model.analysis()->solver(fea::analysis::solvers::type::dynamic_nonlinear);
	dynamic_cast<fea::analysis::solvers::Dynamic_Nonlinear*> (model.analysis()->solver())->time_max(30);
	dynamic_cast<fea::analysis::solvers::Dynamic_Nonlinear*> (model.analysis()->solver())->step_max(20000);
	dynamic_cast<fea::analysis::solvers::Dynamic_Nonlinear*> (model.analysis()->solver())->watch_dof(1, fea::mesh::nodes::dof::rotation_3);
	dynamic_cast<fea::analysis::solvers::Dynamic_Nonlinear*> (model.analysis()->solver())->integration(fea::analysis::solvers::integration::newmark);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}