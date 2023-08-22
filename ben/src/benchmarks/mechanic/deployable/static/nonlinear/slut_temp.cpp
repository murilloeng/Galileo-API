//std
#include <cmath>
#include <ctime>
#include <cstring>

//mat
#include "mat/inc/misc/util.h"
#include "mat/inc/linear/dense.h"

//fea
#include "fea/inc/Model/Model.h"

#include "fea/inc/Mesh/Mesh.h"
#include "fea/inc/Mesh/Nodes/Dof.h"
#include "fea/inc/Mesh/Cells/Types.h"
#include "fea/inc/Mesh/Joints/Types.h"
#include "fea/inc/Mesh/Joints/Hinge.h"
#include "fea/inc/Mesh/Sections/Box.h"
#include "fea/inc/Mesh/Sections/Types.h"
#include "fea/inc/Mesh/Elements/Types.h"
#include "fea/inc/Mesh/Cells/Line/Line.h"
#include "fea/inc/Mesh/Materials/Types.h"
#include "fea/inc/Mesh/Materials/Mechanic/Steel.h"
#include "fea/inc/Mesh/Elements/Mechanic/Mechanic.h"
#include "fea/inc/Mesh/Elements/Mechanic/Frame/Beam3.h"

#include "fea/inc/Boundary/Boundary.h"
#include "fea/inc/Boundary/Loads/Load_Case.h"

#include "fea/inc/Analysis/Analysis.h"
#include "fea/inc/Analysis/Solvers/Types.h"
#include "fea/inc/Analysis/Strategies/Types.h"
#include "fea/inc/Analysis/Solvers/Static_Nonlinear.h"

//ben
#include "ben/inc/benchmarks/mechanic/deployable.h"

//controls
const static int hinge = 1;
const static bool open = true;

//geometry
const static double L = 1.00e+00;
const static double h = 0.50e+00;
const static double H = 0.40e+00;

const static double r = 5.00e-02;
const static double s = 1.40e-02;

//joint
const static char* names[] = {"soft", "hard", "rigid"};
const static double kt[] = {7.83e+02, 1.13e+03, 1.00e+05};
const static double kr[] = {1.98e+07, 4.80e+07, 1.00e+10};

//section
const static double hs = 4.00e-02;
const static double ws = 1.00e-02;
const static double ts = 1.00e-03;
const static double As = 2 * (hs + ws - 2 * ts) * ts;

//material
const static double ra = 2.70e+03;
const static double Ea = 7.00e+10;

//analysis
static unsigned nc = 43696;
const static unsigned nd = 4;
const static unsigned nr = 4;
const static unsigned na = 50000;

//parameters
const static double D = 2 * L * sin(M_PI / nd);
const static double v = r * (4 * sin(M_PI / nd) - 2);
const static double l = sqrt(h * h + pow(L - 2 * r, 2));
const static double e = sqrt(H * H + pow(D - 2 * r, 2)) / 2;
const static double x = (pow(D - 2 * r, 2) - v * v) / (2 * sqrt(4 * e * e - v * v) * l - 2 * h * H);

const static double d = x * l;
const static double b = (1 - x) * l;
const static double z = h - (1 - x) / x * H;
const static double c = sqrt(4 * e * e - v * v) - d;
const static double a = (1 - x) / x * c;

//data
static double p0[3], p1[3];
static double pa[nd][2][3], pb[nd][2][3], po[nd][2][3];
static double pc[nd][2][3], pd[nd][7][3], pe[nd][7][3];

//create
static void set_points(void)
{
	double t;
	p0[0] = p0[1] = 0;
	p1[0] = p1[1] = 0;
	p0[2] = open ? z : d - a;
	p1[2] = open ? h : d + b;
	for(unsigned i = 0; i < nd; i++)
	{
		t = (2 * i + 1) * M_PI / nd;
		pa[i][1][0] = r * cos(t) + s * sin(t);
		pa[i][1][1] = r * sin(t) - s * cos(t);
		pb[i][1][0] = r * cos(t) - s * sin(t);
		pb[i][1][1] = r * sin(t) + s * cos(t);
		pa[i][0][0] = pb[i][0][0] = r * cos(t);
		pa[i][0][1] = pb[i][0][1] = r * sin(t);
		pa[i][0][2] = pa[i][1][2] = open ? z : d - a;
		pb[i][0][2] = pb[i][1][2] = open ? h : d + b;
		po[i][0][2] = po[i][1][2] = open ? x * h : d;
		pc[i][0][2] = pc[i][1][2] = open ? H / 2 : (d + c) / 2;
		pd[i][4][0] = (open ? L - r : r) * cos(t) + s * sin(t);
		pd[i][4][1] = (open ? L - r : r) * sin(t) - s * cos(t);
		pe[i][4][0] = (open ? L - r : r) * cos(t) - s * sin(t);
		pe[i][4][1] = (open ? L - r : r) * sin(t) + s * cos(t);
		pd[i][0][0] = pe[i][0][0] = (open ? L : 2 * r) * cos(t);
		pd[i][0][1] = pe[i][0][1] = (open ? L : 2 * r) * sin(t);
		pd[i][1][0] = pe[i][1][0] = (open ? L - r : r) * cos(t);
		pd[i][1][1] = pe[i][1][1] = (open ? L - r : r) * sin(t);
		po[i][0][0] = (open ? r + (1 - x) * (L - 2 * r) : r) * cos(t) + s * sin(t);
		po[i][0][1] = (open ? r + (1 - x) * (L - 2 * r) : r) * sin(t) - s * cos(t);
		po[i][1][0] = (open ? r + (1 - x) * (L - 2 * r) : r) * cos(t) - s * sin(t);
		po[i][1][1] = (open ? r + (1 - x) * (L - 2 * r) : r) * sin(t) + s * cos(t);
		pc[i][0][0] = ((open ? L : 2 * r) * cos(M_PI / nd) - s) * cos(t + M_PI / nd);
		pc[i][0][1] = ((open ? L : 2 * r) * cos(M_PI / nd) - s) * sin(t + M_PI / nd);
		pc[i][1][0] = ((open ? L : 2 * r) * cos(M_PI / nd) + s) * cos(t + M_PI / nd);
		pc[i][1][1] = ((open ? L : 2 * r) * cos(M_PI / nd) + s) * sin(t + M_PI / nd);
		pd[i][2][0] = pe[i][2][0] = (open ? L : 2 * r) * cos(t) + r * sin(t - M_PI / nd);
		pd[i][2][1] = pe[i][2][1] = (open ? L : 2 * r) * sin(t) - r * cos(t - M_PI / nd);
		pd[i][3][0] = pe[i][3][0] = (open ? L : 2 * r) * cos(t) - r * sin(t + M_PI / nd);
		pd[i][3][1] = pe[i][3][1] = (open ? L : 2 * r) * sin(t) + r * cos(t + M_PI / nd);
		pd[i][5][0] = (open ? L : 2 * r) * cos(t) + r * sin(t - M_PI / nd) + s * cos(t - M_PI / nd);
		pd[i][5][1] = (open ? L : 2 * r) * sin(t) - r * cos(t - M_PI / nd) + s * sin(t - M_PI / nd);
		pd[i][6][0] = (open ? L : 2 * r) * cos(t) - r * sin(t + M_PI / nd) - s * cos(t + M_PI / nd);
		pd[i][6][1] = (open ? L : 2 * r) * sin(t) + r * cos(t + M_PI / nd) - s * sin(t + M_PI / nd);
		pe[i][5][0] = (open ? L : 2 * r) * cos(t) + r * sin(t - M_PI / nd) - s * cos(t - M_PI / nd);
		pe[i][5][1] = (open ? L : 2 * r) * sin(t) - r * cos(t - M_PI / nd) - s * sin(t - M_PI / nd);
		pe[i][6][0] = (open ? L : 2 * r) * cos(t) - r * sin(t + M_PI / nd) + s * cos(t + M_PI / nd);
		pe[i][6][1] = (open ? L : 2 * r) * sin(t) + r * cos(t + M_PI / nd) + s * sin(t + M_PI / nd);
		pe[i][0][2] = pe[i][1][2] = pe[i][2][2] = pe[i][3][2] = pe[i][4][2] = pe[i][5][2] = pe[i][6][2] = 0;
		pd[i][0][2] = pd[i][1][2] = pd[i][2][2] = pd[i][3][2] = pd[i][4][2] = pd[i][5][2] = pd[i][6][2] = open ? H : d + c;
	}
}

static void create_nodes(fea::models::Model* model)
{
	model->mesh()->add_node(p0);
	model->mesh()->add_node(p1);
	for(unsigned i = 0; i < nd; i++)
	{
		model->mesh()->add_node(pa[i][0]);
		model->mesh()->add_node(pa[i][1]);
		model->mesh()->add_node(pb[i][0]);
		model->mesh()->add_node(pb[i][1]);
		model->mesh()->add_node(po[i][0]);
		model->mesh()->add_node(po[i][1]);
		model->mesh()->add_node(pc[i][0]);
		model->mesh()->add_node(pc[i][1]);
		model->mesh()->add_node(pd[i][0]);
		model->mesh()->add_node(pd[i][1]);
		model->mesh()->add_node(pd[i][2]);
		model->mesh()->add_node(pd[i][3]);
		model->mesh()->add_node(pd[i][4]);
		model->mesh()->add_node(pd[i][5]);
		model->mesh()->add_node(pd[i][6]);
		model->mesh()->add_node(pe[i][0]);
		model->mesh()->add_node(pe[i][1]);
		model->mesh()->add_node(pe[i][2]);
		model->mesh()->add_node(pe[i][3]);
		model->mesh()->add_node(pe[i][4]);
		model->mesh()->add_node(pe[i][5]);
		model->mesh()->add_node(pe[i][6]);
	}
}
static void create_cells(fea::models::Model* model)
{
	model->mesh()->add_cell(fea::mesh::cells::type::beam);
}
static void create_joints(fea::models::Model* model)
{
	for(unsigned i = 0; i < nd; i++)
	{
		const double t = (2 * i + 1) * M_PI / nd;
		const double s1[] = {+cos(t), +sin(t), 0};
		const double s2[] = {-sin(t), +cos(t), 0};
		const double u1[] = {+cos(t - M_PI / nd), +sin(t - M_PI / nd), 0};
		const double u2[] = {-sin(t - M_PI / nd), +cos(t - M_PI / nd), 0};
		const double e1[] = {+cos(t + M_PI / nd), +sin(t + M_PI / nd), 0};
		const double e2[] = {-sin(t + M_PI / nd), +cos(t + M_PI / nd), 0};
		const double* ha[] = {s2, s2, s2, e1, s2, u1, e1, s2, u1, e1};
		const double* ho[] = {s1, s1, s1, e2, s1, u2, e2, s1, u2, e2};
		model->mesh()->add_joint(fea::mesh::joints::type::hinge, {22 * i +  2, 22 * i +  3});
		model->mesh()->add_joint(fea::mesh::joints::type::hinge, {22 * i +  4, 22 * i +  5});
		model->mesh()->add_joint(fea::mesh::joints::type::hinge, {22 * i +  6, 22 * i +  7});
		model->mesh()->add_joint(fea::mesh::joints::type::hinge, {22 * i +  8, 22 * i +  9});
		model->mesh()->add_joint(fea::mesh::joints::type::hinge, {22 * i + 11, 22 * i + 14});
		model->mesh()->add_joint(fea::mesh::joints::type::hinge, {22 * i + 12, 22 * i + 15});
		model->mesh()->add_joint(fea::mesh::joints::type::hinge, {22 * i + 13, 22 * i + 16});
		model->mesh()->add_joint(fea::mesh::joints::type::hinge, {22 * i + 18, 22 * i + 21});
		model->mesh()->add_joint(fea::mesh::joints::type::hinge, {22 * i + 19, 22 * i + 22});
		model->mesh()->add_joint(fea::mesh::joints::type::hinge, {22 * i + 20, 22 * i + 23});
		for(unsigned j = 0; j < 10; j++)
		{
			((fea::mesh::joints::Hinge*) model->mesh()->joint(10 * i + j))->axis(ha[j]);
			((fea::mesh::joints::Hinge*) model->mesh()->joint(10 * i + j))->orientation(ho[j]);
			((fea::mesh::joints::Hinge*) model->mesh()->joint(10 * i + j))->stiffness(0, 0, kt[hinge]);
			((fea::mesh::joints::Hinge*) model->mesh()->joint(10 * i + j))->stiffness(0, 1, kt[hinge]);
			((fea::mesh::joints::Hinge*) model->mesh()->joint(10 * i + j))->stiffness(1, 0, kr[hinge]);
			((fea::mesh::joints::Hinge*) model->mesh()->joint(10 * i + j))->stiffness(1, 1, kr[hinge]);
		}
	}
}
static void create_sections(fea::models::Model* model)
{
	model->mesh()->add_section(fea::mesh::sections::type::box);
	((fea::mesh::sections::Box*) model->mesh()->section(0))->width(ws);
	((fea::mesh::sections::Box*) model->mesh()->section(0))->height(hs);
	((fea::mesh::sections::Box*) model->mesh()->section(0))->thickness(ts);
}
static void create_elements(fea::models::Model* model)
{
	double r[6];
	srand(time(nullptr));
	const double h1 = 0.0e-3;
	const double h2 = 1.0e-3;
	memset(r, 0, 6 * sizeof(double));
	for(unsigned i = 0; i < nd; i++)
	{
		const double t = (2 * i + 1) * M_PI / nd;
		const double s2[] = {-sin(t), +cos(t), 0};
		const double u1[] = {+cos(t - M_PI / nd), +sin(t - M_PI / nd), 0};
		const double e1[] = {+cos(t + M_PI / nd), +sin(t + M_PI / nd), 0};
		const double* bo[] = {s2, s2, s2, s2, s2, s2, e1, e1, s2, u1, e1, s2, u1, e1, e1, e1};
		model->mesh()->add_element(fea::mesh::elements::type::beam3C, {22 * i +  2, 0});
		model->mesh()->add_element(fea::mesh::elements::type::beam3C, {22 * i +  4, 1});
		model->mesh()->add_element(fea::mesh::elements::type::beam3C, {22 * i +  3, 22 * i +  6});
		model->mesh()->add_element(fea::mesh::elements::type::beam3C, {22 * i +  5, 22 * i +  7});
		model->mesh()->add_element(fea::mesh::elements::type::beam3C, {22 * i +  6, 22 * i + 14});
		model->mesh()->add_element(fea::mesh::elements::type::beam3C, {22 * i +  7, 22 * i + 21});
		model->mesh()->add_element(fea::mesh::elements::type::beam3C, {22 * i +  8, 22 * i + 16});
		model->mesh()->add_element(fea::mesh::elements::type::beam3C, {22 * i +  9, 22 * i + 23});
		model->mesh()->add_element(fea::mesh::elements::type::beam3C, {22 * i + 10, 22 * i + 11});
		model->mesh()->add_element(fea::mesh::elements::type::beam3C, {22 * i + 10, 22 * i + 12});
		model->mesh()->add_element(fea::mesh::elements::type::beam3C, {22 * i + 10, 22 * i + 13});
		model->mesh()->add_element(fea::mesh::elements::type::beam3C, {22 * i + 17, 22 * i + 18});
		model->mesh()->add_element(fea::mesh::elements::type::beam3C, {22 * i + 17, 22 * i + 19});
		model->mesh()->add_element(fea::mesh::elements::type::beam3C, {22 * i + 17, 22 * i + 20});
		model->mesh()->add_element(fea::mesh::elements::type::beam3C, {22 * i +  8, 22 * (i + 1 != nd ? i + 1 : 0) + 22});
		model->mesh()->add_element(fea::mesh::elements::type::beam3C, {22 * i +  9, 22 * (i + 1 != nd ? i + 1 : 0) + 15});
		for(unsigned j = 0; j < 16; j++)
		{
			const double u = double(rand()) / RAND_MAX;
			const double v = double(rand()) / RAND_MAX;
			r[0] = (u > 0.5 ? +1 : -1) * (h1 + (h2 - h1) * v) * Ea * As;
			// ((fea::mesh::elements::Beam3*) model->mesh()->element(16 * i + j))->residue(r);
			((fea::mesh::elements::Beam3*) model->mesh()->element(16 * i + j))->orientation(bo[j]);
		}
	}
}
static void refine_elements(fea::models::Model* model)
{
	for(unsigned i = 0; i < nd; i++)
	{
		fea::mesh::cells::Line::refine(16 * i +  2, nr);
		fea::mesh::cells::Line::refine(16 * i +  3, nr);
		fea::mesh::cells::Line::refine(16 * i +  4, nr);
		fea::mesh::cells::Line::refine(16 * i +  5, nr);
		fea::mesh::cells::Line::refine(16 * i +  6, nr);
		fea::mesh::cells::Line::refine(16 * i +  7, nr);
		fea::mesh::cells::Line::refine(16 * i + 14, nr);
		fea::mesh::cells::Line::refine(16 * i + 15, nr);
	}
}
static void create_materials(fea::models::Model* model)
{
	model->mesh()->add_material(fea::mesh::materials::type::steel);
	((fea::mesh::materials::Steel*) model->mesh()->material(0))->specific_mass(ra);
	((fea::mesh::materials::Steel*) model->mesh()->material(0))->elastic_modulus(Ea);
}

static void create_supports(fea::models::Model* model)
{
	model->boundary()->add_support(0, fea::mesh::nodes::dof::rotation_1);
	model->boundary()->add_support(0, fea::mesh::nodes::dof::rotation_2);
	model->boundary()->add_support(0, fea::mesh::nodes::dof::rotation_3);
	model->boundary()->add_support(0, fea::mesh::nodes::dof::translation_1);
	model->boundary()->add_support(0, fea::mesh::nodes::dof::translation_2);
	model->boundary()->add_support(0, fea::mesh::nodes::dof::translation_3);
}

static void create_solver(fea::models::Model* model)
{
	fea::mesh::elements::Mechanic::geometric(true);
	model->analysis()->solver(fea::analysis::solvers::type::static_nonlinear);
	model->analysis()->solver()->watch_dof(1, fea::mesh::nodes::dof::translation_3);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model->analysis()->solver())->step_max(0);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model->analysis()->solver())->attempt_max(1);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model->analysis()->solver())->spectre_min(0);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model->analysis()->solver())->spectre_max(2);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model->analysis()->solver())->tolerance(1e-2);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model->analysis()->solver())->frequencies(true);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model->analysis()->solver())->iteration_max(100);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model->analysis()->solver())->spectre(fea::analysis::solvers::spectre::partial);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model->analysis()->solver())->strategy(fea::analysis::strategies::type::control_load);
	dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model->analysis()->solver())->convergence((unsigned) fea::analysis::solvers::convergence::fixed);
	model->analysis()->solve();
}

static void save_data(fea::models::Model* model)
{
	const double* e = model->analysis()->solver()->eigen_values();
	if(dynamic_cast<fea::analysis::solvers::Static_Nonlinear*>(model->analysis()->solver())->equilibrium())
	{
		char path[800];
		sprintf(path, "../models/benchmarks/deployable/static/nonlinear/slut temp/data_50k_%s.dat", names[hinge]);
		FILE* file = fopen(path, "a");
		printf("%04d: %+.2e %+.2e %+.2e \n", nc++, e[0], e[1], e[2]);
		fprintf(file, "%+.6e %+.6e %+.6e \n", e[0], e[1], e[2]);
		fclose(file);
	}
}

//model
void tests::deployable::static_nonlinear::slut_temp(void)
{
	while(nc < na)
	{
		//model
		fea::models::Model model("slut temp", "benchmarks/deployable/static/nonlinear");

		//mesh
		set_points();
		create_nodes(&model);
		create_cells(&model);
		create_joints(&model);
		create_sections(&model);
		create_elements(&model);
		refine_elements(&model);
		create_materials(&model);

		//boundary
		create_supports(&model);

		//analysis
		create_solver(&model);

		//data
		save_data(&model);
	}
}