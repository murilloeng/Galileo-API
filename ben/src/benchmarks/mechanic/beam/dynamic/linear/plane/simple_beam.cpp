//std
#include <cmath>

//fea
#include "fea/inc/Model/Model.h"

#include "fea/inc/Mesh/Mesh.h"
#include "fea/inc/Mesh/Nodes/Dof.h"
#include "fea/inc/Mesh/Cells/Types.h"
#include "fea/inc/Mesh/Sections/Types.h"
#include "fea/inc/Mesh/Elements/Types.h"
#include "fea/inc/Mesh/Materials/Types.h"

#include "fea/inc/Topology/Topology.h"
#include "fea/inc/Topology/Curves/Curve.h"
#include "fea/inc/Topology/Curves/Types.h"

#include "fea/inc/Boundary/Boundary.h"
#include "fea/inc/Boundary/Time/Types.h"
#include "fea/inc/Boundary/Time/Sine_Wave.h"
#include "fea/inc/Boundary/Loads/Load_Case.h"

#include "fea/inc/Analysis/Analysis.h"
#include "fea/inc/Analysis/Solvers/Types.h"
#include "fea/inc/Analysis/Solvers/Dynamic_Linear.h"

//ben
#include "ben/inc/benchmarks/mechanic/beam.h"

void tests::beam::dynamic_linear::plane::simple_beam(void)
{
	//data
	const unsigned ns = 10000;
	const double L = 3.00e+00;
	const double P = 1.00e+04;
	const double f = 1.00e+00;
	const double T = 1.00e+01;

	//model
	fea::models::Model model("simple beam", "benchmarks/beam/dynamic/linear/plane");

	//sections
	model.mesh()->add_section(fea::mesh::sections::type::profile_I);

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::beam);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);

	//points
	model.topology()->add_point(0, 0, 0);
	model.topology()->add_point(L, 0, 0);

	//curves
	model.topology()->add_curve(fea::topology::curves::type::line, {0, 1});
	model.topology()->curve(0)->cell(0);
	model.topology()->curve(0)->material(0);
	model.topology()->curve(0)->structured(20);
	model.topology()->curve(0)->element(fea::mesh::elements::type::beam2C);

	//mesh
	model.topology()->mesh(1);

	//times
	model.boundary()->add_time(fea::boundary::time::type::sine_wave);
	((fea::boundary::time::Sine_Wave*) model.boundary()->time(0))->frequency(f);

	//supports
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_3);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_2);

	//loads
	model.boundary()->add_load_set();
	model.boundary()->add_load_case(1, fea::mesh::nodes::dof::translation_2, -P, 0);

	//solver
	model.analysis()->solver(fea::analysis::solvers::type::dynamic_linear);
	dynamic_cast<fea::analysis::solvers::Dynamic_Linear*>(model.analysis()->solver())->load_set(0);
	dynamic_cast<fea::analysis::solvers::Dynamic_Linear*>(model.analysis()->solver())->time_max(T);
	dynamic_cast<fea::analysis::solvers::Dynamic_Linear*>(model.analysis()->solver())->step_max(ns);
	dynamic_cast<fea::analysis::solvers::Dynamic_Linear*>(model.analysis()->solver())->watch_dof(1, fea::mesh::nodes::dof::translation_2);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}