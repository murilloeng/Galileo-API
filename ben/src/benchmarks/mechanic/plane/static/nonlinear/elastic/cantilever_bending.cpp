//std
#include <cmath>

//fea
#include "Model/Model.h"

#include "Mesh/Mesh.h"
#include "Mesh/Nodes/Dof.h"
#include "Mesh/Cells/Types.h"
#include "Mesh/Elements/Types.h"
#include "Mesh/Materials/Types.h"
#include "Mesh/Cells/Plane/Plane.h"
#include "Mesh/Materials/Mechanic/Steel.h"
#include "Mesh/Elements/Mechanic/Mechanic.h"

#include "Boundary/Boundary.h"
#include "Boundary/Loads/Types.h"
#include "Boundary/Loads/Load_Case.h"
#include "Boundary/Loads/Elements/Mechanic/Plane/Plane_Force.h"

#include "Analysis/Analysis.h"
#include "Analysis/Solvers/Types.h"
#include "Analysis/Strategies/Types.h"
#include "Analysis/Solvers/Static_Nonlinear.h"

//ben
#include "benchmarks/mechanic/plane.h"

//data
const static double l = 1.00e+00;
const static double h = 1.00e-01;
const static double t = 4.00e-02;
const static double E = 2.00e+11;
const static double v = 2.50e-01;

//mesh
const static unsigned ny = 10;
const static unsigned nx = 50;

void tests::plane::static_nonlinear::elastic::cantilever_bending(void)
{
	//model
	fea::models::Model model("cantilever bending", "benchmarks/plane/static/nonlinear/elastic");

	//nodes
	for(unsigned i = 0; i <= nx; i++)
	{
		for(unsigned j = 0; j <= ny; j++)
		{
			const double x = l * i / nx;
			const double y = h * j / ny;
			model.mesh()->add_node(x, y, 0);
		}
	}

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::quad4);
	((fea::mesh::cells::Plane*) model.mesh()->cell(0))->thickness(t);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->poisson_ratio(v);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->elastic_modulus(E);

	//elements
	for(unsigned i = 0; i < nx; i++)
	{
		for(unsigned j = 0; j < ny; j++)
		{
			const unsigned n0 = (ny + 1) * (i + 0) + j + 0;
			const unsigned n1 = (ny + 1) * (i + 1) + j + 0;
			const unsigned n2 = (ny + 1) * (i + 1) + j + 1;
			const unsigned n3 = (ny + 1) * (i + 0) + j + 1;
			model.mesh()->add_element(fea::mesh::elements::type::plane, {n0, n1, n2, n3});
		}
	}

	//supports
	for(unsigned i = 0; i <= ny; i++)
	{
		model.boundary()->add_support(i, fea::mesh::nodes::dof::translation_1);
	}
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_2);

	//loads
	model.boundary()->add_load_case();
	for(unsigned i = 0; i < ny; i++)
	{
		const double y = h / 2 * (2 * i + 1) / ny - h / 2;
		// model.boundary()->load_case(0)->add_load_element(fea::boundary::loads::type::plane_edge);
		// ((fea::boundary::loads::Plane_Edge*) model.boundary()->load_case(0)->load_element(i))->edge(1);
		// ((fea::boundary::loads::Plane_Edge*) model.boundary()->load_case(0)->load_element(i))->direction(0);
		// ((fea::boundary::loads::Plane_Edge*) model.boundary()->load_case(0)->load_element(i))->value(-2 * M_PI * E * y / l);
		// ((fea::boundary::loads::Plane_Edge*) model.boundary()->load_case(0)->load_element(i))->elements().push_back((nx - 1) * ny + i);
	}

	//solver
	fea::mesh::elements::Mechanic::geometric(true);
	model.analysis()->solver(fea::analysis::solvers::type::static_nonlinear);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->step_max(100);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_guess(0.01);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->strategy(fea::analysis::strategies::type::control_load);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->watch_dof(nx * (ny + 1), fea::mesh::nodes::dof::translation_1);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}