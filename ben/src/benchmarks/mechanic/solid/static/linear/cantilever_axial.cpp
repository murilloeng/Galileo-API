//std
#include <cmath>

//fea
#include "fea/inc/Model/Model.h"

#include "fea/inc/Mesh/Mesh.h"
#include "fea/inc/Mesh/Nodes/Dof.h"
#include "fea/inc/Mesh/Cells/Types.h"
#include "fea/inc/Mesh/Elements/Types.h"
#include "fea/inc/Mesh/Materials/Types.h"
#include "fea/inc/Mesh/Materials/Mechanic/Steel.h"

#include "fea/inc/Boundary/Boundary.h"
#include "fea/inc/Boundary/Loads/Types.h"
#include "fea/inc/Boundary/Loads/Load_Case.h"
#include "fea/inc/Boundary/Loads/Elements/Mechanic/Solid/Solid_Force.h"

#include "fea/inc/Analysis/Analysis.h"
#include "fea/inc/Analysis/Solvers/Types.h"
#include "fea/inc/Analysis/Solvers/Static_Linear.h"

//ben
#include "ben/inc/benchmarks/mechanic/solid.h"

//data
const static double h = 1.00e-01;
const static double t = 4.00e-02;
const static double l = 1.00e+00;
const static double v = 2.50e-01;
const static double E = 2.00e+11;

//mesh
const static unsigned nx = 10;
const static unsigned ny = 10;
const static unsigned nz = 10;

static unsigned index_node(unsigned i, unsigned j, unsigned k)
{
	return (ny + 1) * (nx + 1) * i + (nx + 1) * j + k;
}
static unsigned index_element(unsigned i, unsigned j, unsigned k)
{
	return ny * nx * i + nx * j + k;
}

void tests::solid::static_linear::cantilever_axial(void)
{
	//model
	fea::models::Model model("cantilever axial", "benchmarks/solid/static/linear");

	//nodes
	for(unsigned i = 0; i <= nz; i++)
	{
		for(unsigned j = 0; j <= ny; j++)
		{
			for(unsigned k = 0; k <= nx; k++)
			{
				const double x = h * k / nx;
				const double y = t * j / ny;
				const double z = l * i / nz;
				model.mesh()->add_node(x, y, z);
			}
		}
	}

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::brick8);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->poisson_ratio(v);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->elastic_modulus(E);

	//elements
	for(unsigned i = 0; i < nz; i++)
	{
		for(unsigned j = 0; j < ny; j++)
		{
			for(unsigned k = 0; k < nx; k++)
			{
				const unsigned n0 = index_node(i + 0, j + 0, k + 0);
				const unsigned n1 = index_node(i + 0, j + 0, k + 1);
				const unsigned n2 = index_node(i + 0, j + 1, k + 1);
				const unsigned n3 = index_node(i + 0, j + 1, k + 0);
				const unsigned n4 = index_node(i + 1, j + 0, k + 0);
				const unsigned n5 = index_node(i + 1, j + 0, k + 1);
				const unsigned n6 = index_node(i + 1, j + 1, k + 1);
				const unsigned n7 = index_node(i + 1, j + 1, k + 0);
				model.mesh()->add_element(fea::mesh::elements::type::solid, {n0, n1, n2, n3, n4, n5, n6, n7});
			}
		}
	}

	//supports
	for(unsigned i = 0; i <= ny; i++)
	{
		for(unsigned j = 0; j <= nx; j++)
		{
			model.boundary()->add_support(index_node(0, i, j), fea::mesh::nodes::dof::translation_3);
		}
	}
	model.boundary()->add_support(index_node(0, 0, 0), fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(index_node(0, 0, 0), fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(index_node(0, 0, nx), fea::mesh::nodes::dof::translation_2);

	//loads
	model.boundary()->add_load_set();
	model.boundary()->add_load_case();
	// model.boundary()->load_case(0)->add_load_element(fea::boundary::loads::type::solid_face, {}, E);
	// ((fea::boundary::loads::Solid_Face*) model.boundary()->load_case(0)->load_element(0))->face(5);
	// ((fea::boundary::loads::Solid_Face*) model.boundary()->load_case(0)->load_element(0))->direction(0, M_PI / 2);
	for(unsigned i = 0; i < ny; i++)
	{
		for(unsigned j = 0; j < nx; j++)
		{
			// model.boundary()->load_case(0)->load_element(0)->elements().push_back(index_element(nz - 1, i, j));
		}
	}

	//solver
	model.analysis()->solver(fea::analysis::solvers::type::static_linear);
	model.analysis()->solver()->watch_dof(index_node(nz, 0, 0), fea::mesh::nodes::dof::translation_3);

	//solve
	// model.analysis()->solve();

	//save
	model.save();
}