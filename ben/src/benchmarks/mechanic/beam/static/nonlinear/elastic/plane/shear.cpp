//std
#include <cmath>

//mat
#include "mat/inc/linear/quat.h"
#include "mat/inc/linear/vec3.h"
#include "mat/inc/linear/mat3.h"

//fea
#include "fea/inc/Model/Model.h"

#include "fea/inc/Mesh/Mesh.h"
#include "fea/inc/Mesh/Nodes/Dof.h"
#include "fea/inc/Mesh/Nodes/Node.h"
#include "fea/inc/Mesh/Cells/Types.h"
#include "fea/inc/Mesh/Sections/Box.h"
#include "fea/inc/Mesh/Sections/Types.h"
#include "fea/inc/Mesh/Elements/Types.h"
#include "fea/inc/Mesh/Cells/Line/Line.h"
#include "fea/inc/Mesh/Materials/Types.h"
#include "fea/inc/Mesh/Materials/Mechanic/Steel.h"
#include "fea/inc/Mesh/Materials/Mechanic/Mechanic.h"
#include "fea/inc/Mesh/Elements/Mechanic/Frame/Beam.h"

#include "fea/inc/Boundary/Boundary.h"
#include "fea/inc/Boundary/Loads/Types.h"
#include "fea/inc/Boundary/Loads/Load_Case.h"
#include "fea/inc/Boundary/Loads/Elements/Element.h"

#include "fea/inc/Analysis/Analysis.h"
#include "fea/inc/Analysis/Solvers/Types.h"
#include "fea/inc/Analysis/Strategies/Types.h"
#include "fea/inc/Analysis/Solvers/Static_Nonlinear.h"

//ben
#include "ben/inc/benchmarks/mechanic/beam.h"

void blar_c(double& x, double& s, double& m, fea::mesh::elements::Element* e, unsigned n, unsigned i)
{
	//kinematics
	mat::vec3 x1, x2;
	e->node(0)->position(x1.mem());
	e->node(1)->position(x2.mem());
	const mat::vec3 X1 = e->node(0)->coordinates();
	const mat::vec3 X2 = e->node(1)->coordinates();
	//triad
	const double L = (X2 - X1).norm();
	const mat::vec3 s1 = (X2 - X1) / L;
	const mat::vec3 s3 = ((fea::mesh::elements::Beam*) e)->orientation();
	const mat::vec3 s2 = s3.cross(s1);
	//quaternion
	const mat::quat q1 = e->node(0)->quaternion();
	const mat::quat q2 = e->node(1)->quaternion();
	//section
	const double A = e->mesh()->section(0)->area();
	const double J = e->mesh()->section(0)->inertia();
	const double I22 = e->mesh()->section(0)->inertia(0, 0);
	const double I33 = e->mesh()->section(0)->inertia(1, 1);
	//material
	const double G = ((fea::mesh::materials::Mechanic*) e->mesh()->material(0))->shear_modulus();
	const double E = ((fea::mesh::materials::Mechanic*) e->mesh()->material(0))->elastic_modulus();
	//strains
	const mat::vec3 xe = q1.conjugate(x2 - x1) / L;
	const mat::vec3 we = q1.conjugate(q2).pseudo() / L;
	const mat::vec3 ge = (L * we).rotation_gradient_inverse(xe) - s1;
	//resultants
	const mat::vec3 ne = E * A * s1.inner(ge) * s1 + G * A * s2.inner(ge) * s2 + G * A * s3.inner(ge) * s3;
	const mat::vec3 me = G * J * s1.inner(we) * s1 + E * I33 * s2.inner(we) * s2 + E * I22 * s3.inner(we) * s3;
	//results
	m = me[2];
	s = ne[1];
	x = X1[0] + i * L / n;
}
void blar_y(double& x, double& s, double& m, fea::mesh::elements::Element* e, unsigned n, unsigned i)
{
	//kinematics
	mat::vec3 x1, x2;
	e->node(0)->position(x1.mem());
	e->node(1)->position(x2.mem());
	const mat::vec3 X1 = e->node(0)->coordinates();
	const mat::vec3 X2 = e->node(1)->coordinates();
	//triad
	const double L = (X1 - X2).norm();
	const mat::vec3 s1 = (X2 - X1) / L;
	const mat::vec3 s3 = ((fea::mesh::elements::Beam*) e)->orientation();
	const mat::vec3 s2 = s3.cross(s1);
	//quaternion
	const mat::quat q1 = e->node(0)->quaternion();
	const mat::quat q2 = e->node(1)->quaternion();
	//section
	const double A = e->mesh()->section(0)->area();
	const double J = e->mesh()->section(0)->inertia();
	const double I22 = e->mesh()->section(0)->inertia(0, 0);
	const double I33 = e->mesh()->section(0)->inertia(1, 1);
	//material
	const double G = ((fea::mesh::materials::Mechanic*) e->mesh()->material(0))->shear_modulus();
	const double E = ((fea::mesh::materials::Mechanic*) e->mesh()->material(0))->elastic_modulus();
	//section strains
	const mat::vec3 we = q1.conjugate(q2).pseudo() / L;
	const mat::vec3 ge = (L * we).rotation_gradient_inverse(q1.conjugate(x2 - x1)) / L - s1;
	//section stiffness
	const mat::mat3 C = E * A * s1.outer(s1) + G * A * s2.outer(s2) + G * A * s3.outer(s3);
	const mat::mat3 D = G * J * s1.outer(s1) + E * I33 * s2.outer(s2) + E * I22 * s3.outer(s3);
	//section resultants
	const mat::vec3 n0 = q1.rotate((L * we).rotation_gradient(C * ge));
	const mat::vec3 x0 = q1.rotate((L * we).rotation_class(ge + s1, 2));
	const mat::vec3 m0 = q1.rotate((L * we).rotation_gradient(D * we)) - L * n0.cross(x0);
	const mat::vec3 xr = i * L / n * q1.rotate((i * L / n * we).rotation_gradient(s1 + ge));
	//results
	s = n0[1];
	x = X1[0] + i * L / n;
	m = (m0 + n0.cross(xr))[2];
}
void blar(double& x, double& s, double& m, fea::mesh::elements::Element* e, unsigned n, unsigned i)
{
	if(e->type() == fea::mesh::elements::type::beam3T)
	{
		blar_c(x, s, m, e, n, i);
	}
	else
	{
		blar_y(x, s, m, e, n, i);
	}
}

void tests::beam::static_nonlinear::elastic::plane::shear(void)
{
	//data
	const unsigned n = 5;
	const double v = 3.00e-1;
	const double E = 2.00e11;
	const double w = 2.00e-1;
	const double h = 4.00e-1;
	const double t = 6.00e-3;
	const double L = 2.00e+0;
	const double P = 4.29e+7;

	//model
	fea::models::Model model("shear", "benchmarks/beam/static/nonlinear/elastic/plane");

	//nodes
	model.mesh()->add_node(0 * L, 0, 0);
	model.mesh()->add_node(1 * L, 0, 0);
	model.mesh()->add_node(2 * L, 0, 0);

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::beam);

	//sections
	model.mesh()->add_section(fea::mesh::sections::type::box);
	((fea::mesh::sections::Box*) model.mesh()->section(0))->width(w);
	((fea::mesh::sections::Box*) model.mesh()->section(0))->height(h);
	((fea::mesh::sections::Box*) model.mesh()->section(0))->thickness(t);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->poisson_ratio(v);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->elastic_modulus(E);

	//elements
	model.mesh()->add_element(fea::mesh::elements::type::beam3T, {0, 1});
	model.mesh()->add_element(fea::mesh::elements::type::beam3T, {1, 2});

	//supports
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::rotation_3);
	model.boundary()->add_support(2, fea::mesh::nodes::dof::rotation_1);
	model.boundary()->add_support(2, fea::mesh::nodes::dof::rotation_2);
	model.boundary()->add_support(2, fea::mesh::nodes::dof::rotation_3);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_3);
	model.boundary()->add_support(2, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(2, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(2, fea::mesh::nodes::dof::translation_3);

	//loads
	model.boundary()->add_load_case(1, fea::mesh::nodes::dof::translation_2, -P);

	//solver
	fea::mesh::cells::Line::refine(0, n);
	fea::mesh::cells::Line::refine(1, n);
	model.analysis()->solver(fea::analysis::solvers::type::static_nonlinear);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->step_max(100);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_max(1.00);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->load_guess(0.01);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->watch_dof(1, fea::mesh::nodes::dof::translation_2);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model.analysis()->solver())->strategy(fea::analysis::strategies::type::control_load);

	//solve
	model.analysis()->solve();

	//save
	model.save();

	double x, s, m;
	const unsigned np = 5;
	FILE* file = fopen("stress.dat", "w");
	for(unsigned i = 0; i < model.mesh()->elements().size(); i++)
	{
		for(unsigned j = 0; j <= np; j++)
		{
			blar(x, s, m, model.mesh()->element(i), np, j);
			fprintf(file, "%+.6e %+.6e %+.6e\n", x, s, m);
		}
	}
	fclose(file);
}