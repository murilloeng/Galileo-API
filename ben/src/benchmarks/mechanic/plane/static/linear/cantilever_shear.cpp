//mat
#include <cmath>

//fea
#include "fea/inc/Model/Model.h"

#include "fea/inc/Mesh/Mesh.h"
#include "fea/inc/Mesh/Nodes/Dof.h"
#include "fea/inc/Mesh/Cells/Types.h"
#include "fea/inc/Mesh/Elements/Types.h"
#include "fea/inc/Mesh/Materials/Types.h"
#include "fea/inc/Mesh/Cells/Plane/Plane.h"
#include "fea/inc/Mesh/Materials/Mechanic/Steel.h"

#include "fea/inc/Boundary/Boundary.h"
#include "fea/inc/Boundary/Loads/Types.h"
#include "fea/inc/Boundary/Loads/Load_Case.h"
#include "fea/inc/Boundary/Loads/Elements/Mechanic/Plane/Plane_Force.h"

#include "fea/inc/Analysis/Analysis.h"
#include "fea/inc/Analysis/Solvers/Types.h"
#include "fea/inc/Analysis/Solvers/Static_Linear.h"

//ben
#include "ben/inc/benchmarks/mechanic/plane.h"

//mesh
const static unsigned ny = 10;
const static unsigned nx = 50;

//data
const static double l = 1.00e+00;
const static double h = 1.00e-01;
const static double t = 4.00e-02;
const static double E = 2.00e+11;
const static double v = 2.50e-01;
const static double s = E * t * h * h / l / l / 4;

void tests::plane::static_linear::cantilever_shear(void)
{
	//model
	fea::models::Model model("cantilever shear", "benchmarks/plane/static/linear");

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
	model.boundary()->add_load_case(fea::boundary::loads::type::plane_force);
	for(unsigned i = 0; i < ny; i++)
	{
		model.boundary()->load_case(0)->load_element(0)->elements().push_back(ny * (nx - 1) + i);
	}
	// ((fea::boundary::loads::Plane_Edge*) model.boundary()->load_case(0)->load_element(0))->value(s);
	// ((fea::boundary::loads::Plane_Edge*) model.boundary()->load_case(0)->load_element(0))->direction(M_PI / 2);
	// ((fea::boundary::loads::Plane_Edge*) model.boundary()->load_case(0)->load_element(0))->edges().push_back(1);

	//solver
	model.analysis()->solver(fea::analysis::solvers::type::static_linear);
	model.analysis()->solver()->watch_dof((ny + 1) * nx, fea::mesh::nodes::dof::translation_2);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}