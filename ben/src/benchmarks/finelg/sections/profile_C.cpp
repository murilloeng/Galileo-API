//std
#include <cstdio>

//mat
#include "linear/vec3.h"
#include "geometry/Line.h"

//fea
#include "Model/Model.h"

#include "Mesh/Mesh.h"
#include "Mesh/Sections/Mesh.h"
#include "Mesh/Sections/Types.h"
#include "Mesh/Sections/Profile.h"

#include "Topology/Topology.h"
#include "Topology/Points/Point.h"

//ben
#include "benchmarks/finelg/finelg.h"

void tests::finelg::sections::profile_C(void)
{
	//data
	FILE* file[3];
	char path[3][800];
	double u, xp[3], xw[2][3];
	
	const unsigned d = 0;
	const unsigned np = 2000;
	const double tw = 1.00e-01;
	const double hw = 1.00e+01;
	const double ttf = 1.00e-01;
	const double tbf = 1.00e-01;
	const double wtf = 5.00e+00;
	const double wbf = 5.00e+00;
	const char* dir = "../models/benchmarks/finelg/sections/profile-C";

	//model
	fea::models::Model model("profile-C", "benchmarks/finelg/sections");

	//path
	sprintf(path[0], "%s/wall-1-%d.txt", dir, d);
	sprintf(path[1], "%s/wall-2-%d.txt", dir, d);
	sprintf(path[2], "%s/wall-3-%d.txt", dir, d);

	//open
	file[0] = fopen(path[0], "w");
	file[1] = fopen(path[1], "w");
	file[2] = fopen(path[2], "w");

	//section
	model.mesh()->add_section(fea::mesh::sections::type::profile_C);
	((fea::mesh::sections::Profile*) model.mesh()->section(0))->web_height(hw);
	((fea::mesh::sections::Profile*) model.mesh()->section(0))->web_thickness(tw);
	((fea::mesh::sections::Profile*) model.mesh()->section(0))->flange_top_width(wtf);
	((fea::mesh::sections::Profile*) model.mesh()->section(0))->flange_bottom_width(wbf);
	((fea::mesh::sections::Profile*) model.mesh()->section(0))->flange_top_thickness(ttf);
	((fea::mesh::sections::Profile*) model.mesh()->section(0))->flange_bottom_thickness(tbf);

	//setup
	model.mesh()->section(0)->prepare();
	const double As = model.mesh()->section(0)->area(d, d);
	const double t = model.mesh()->section(0)->mesh_local()->box_angle();
	const double* x = model.mesh()->section(0)->mesh_local()->box_center();

	//wall 1
	((fea::mesh::sections::Profile*) model.mesh()->section(0))->wall(0, xw[0], xw[1]);
	mat::vec3(xw[0] + 0) = mat::vec3(0, 0, -t).rotate(mat::vec3(xw[0]) - mat::vec3(x));
	mat::vec3(xw[1] + 0) = mat::vec3(0, 0, -t).rotate(mat::vec3(xw[1]) - mat::vec3(x));
	for(unsigned i = 0; i <= np; i++)
	{
		mat::Line(xw[0], xw[1]).point(xp, double(i) / np);
		model.mesh()->section(0)->mesh_local()->warping(u, xp, d + 1);
		fprintf(file[0], "%+.6e %+.6e\n", 5 * double(i) / np, -(u - xp[d]) / As);
	}
	//wall 2
	((fea::mesh::sections::Profile*) model.mesh()->section(0))->wall(1, xw[0], xw[1]);
	mat::vec3(xw[0] + 0) = mat::vec3(0, 0, -t).rotate(mat::vec3(xw[0]) - mat::vec3(x));
	mat::vec3(xw[1] + 0) = mat::vec3(0, 0, -t).rotate(mat::vec3(xw[1]) - mat::vec3(x));
	for(unsigned i = 0; i <= np; i++)
	{
		mat::Line(xw[0], xw[1]).point(xp, double(i) / np);
		model.mesh()->section(0)->mesh_local()->warping(u, xp, d + 1);
		fprintf(file[1], "%+.6e %+.6e\n", 5 + 10 * double(i) / np, -(u - xp[d]) / As);
	}
	//wall 3
	((fea::mesh::sections::Profile*) model.mesh()->section(0))->wall(2, xw[0], xw[1]);
	mat::vec3(xw[0] + 0) = mat::vec3(0, 0, -t).rotate(mat::vec3(xw[0]) - mat::vec3(x));
	mat::vec3(xw[1] + 0) = mat::vec3(0, 0, -t).rotate(mat::vec3(xw[1]) - mat::vec3(x));
	for(unsigned i = 0; i <= np; i++)
	{
		mat::Line(xw[0], xw[1]).point(xp, double(i) / np);
		model.mesh()->section(0)->mesh_local()->warping(u, xp, d + 1);
		fprintf(file[2], "%+.6e %+.6e\n", 15 + 5 * double(i) / np, -(u - xp[d]) / As);
	}

	//close
	fclose(file[0]);
	fclose(file[1]);
	fclose(file[2]);
}