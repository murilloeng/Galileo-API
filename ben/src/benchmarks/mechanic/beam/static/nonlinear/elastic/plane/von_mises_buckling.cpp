//fea
#include "fea/inc/Model/Model.h"

#include "fea/inc/Mesh/Mesh.h"
#include "fea/inc/Mesh/Nodes/Dof.h"
#include "fea/inc/Mesh/Cells/Types.h"
#include "fea/inc/Mesh/Joints/Types.h"
#include "fea/inc/Mesh/Sections/Ring.h"
#include "fea/inc/Mesh/Sections/Types.h"
#include "fea/inc/Mesh/Elements/Types.h"
#include "fea/inc/Mesh/Materials/Types.h"
#include "fea/inc/Mesh/Materials/Mechanic/Steel.h"
#include "fea/inc/Mesh/Elements/Mechanic/Mechanic.h"

#include "fea/inc/Boundary/Boundary.h"
#include "fea/inc/Boundary/Loads/Load_Case.h"

#include "fea/inc/Analysis/Analysis.h"
#include "fea/inc/Analysis/Solvers/Types.h"
#include "fea/inc/Analysis/Strategies/Types.h"
#include "fea/inc/Analysis/Solvers/Static_Nonlinear.h"

//ben
#include "ben/inc/benchmarks/mechanic/beam.h"

static double shape(double a)
{
	const double b = 1 - 2 * a;
	return sqrt((1 - pow(b, 2)) / (1 - pow(b, 4)));
}

void tests::beam::static_nonlinear::elastic::plane::von_mises_buckling(void)
{
	/*
	von mises truss buckling
	Literature: International Journal of nonlinear mechanics - F. Bazzucchi et al. (2017) pp. 11-20
	 */

	//data
	const unsigned n = 10;
	const double e = 3.33e-03;
	const double a = 6.00e-02;
	const double q = 6.00e+01;
	const double d = 2.50e+00;
	const double E = 2.10e+11;
	const double v = 3.00e-01;
	const double P = 1.00e+03;
	const double h = 6.00e-02 * d;
	const double l = sqrt(d * d + h * h);
	const double D = 4 * l / q * shape(a);

	//model
	fea::models::Model model("von mises buckling", "benchmarks/beam/static/nonlinear/elastic/plane");

	//nodes
	model.mesh()->add_node(-d, 0, 0);
	model.mesh()->add_node(-0, h, 0);
	model.mesh()->add_node(+0, h, 0);
	model.mesh()->add_node(+d, 0, 0);
	for(unsigned i = 1; i < n; i++)
	{
		const double c = +d / l;
		const double s = +h / l;
		const double x = i * l / n;
		const double y = e * l * sin(M_PI * x / l);
		model.mesh()->add_node(c * x + s * y - d, s * x - c * y, 0);
	}
	for(unsigned i = 1; i < n; i++)
	{
		const double c = -d / l;
		const double s = +h / l;
		const double x = i * l / n;
		const double y = e * l * sin(M_PI * x / l);
		model.mesh()->add_node(c * x - s * y + d, s * x + c * y, 0);
	}

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::beam);

	//sections
	model.mesh()->add_section(fea::mesh::sections::type::ring);
	((fea::mesh::sections::Ring*) model.mesh()->section(0))->diameter(D);
	((fea::mesh::sections::Ring*) model.mesh()->section(0))->thickness(a * D);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->poisson_ratio(v);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->elastic_modulus(E);

	//elements
	for(unsigned i = 0; i < n; i++)
	{
		const unsigned a = i == 0 ? 0 : i + 3;
		const unsigned b = i == n - 1 ? 1 : i + 4;
		model.mesh()->add_element(fea::mesh::elements::type::beam2C, {a, b});
	}
	for(unsigned i = 0; i < n; i++)
	{
		const unsigned a = i == 0 ? 3 : i + n + 2;
		const unsigned b = i == n - 1 ? 2 : i + n + 3;
		model.mesh()->add_element(fea::mesh::elements::type::beam2C, {a, b});
	}

	//joints
	model.mesh()->add_joint(fea::mesh::joints::type::pinned, {1, 2});

	//supports
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(3, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(3, fea::mesh::nodes::dof::translation_2);

	//loads
	model.boundary()->add_load_set();
	model.boundary()->add_load_case();
	model.boundary()->load_case(0)->add_load_node(1, fea::mesh::nodes::dof::translation_2, -P);

	//solver
	fea::mesh::elements::Mechanic::geometric(true);
	model.analysis()->solver(fea::analysis::solvers::type::static_nonlinear);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_set(0);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_max(60);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->step_max(1000);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_guess(2.00e-00);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->watch_dof(1, fea::mesh::nodes::dof::translation_2);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->strategy(fea::analysis::strategies::type::minimal_norm);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}