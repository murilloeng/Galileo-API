//std
#include <cstdio>

//fea
#include "fea/inc/Model/Model.h"

#include "fea/inc/Mesh/Mesh.h"
#include "fea/inc/Mesh/Sections/Mesh.h"
#include "fea/inc/Mesh/Sections/Types.h"
#include "fea/inc/Mesh/Sections/Rectangle.h"

//ben
#include "ben/inc/benchmarks/finelg/finelg.h"

void tests::finelg::sections::rectangle(void)
{
	//data
	char path[800];
	double u, x[3];
	const unsigned np = 2000;
	const double w = 1.00e+00;
	const double h = 1.00e+01;
	const char* dir = "../models/benchmarks/finelg/sections/rectangle";

	//model
	fea::models::Model model("rectangle", "benchmarks/finelg/sections");

	//section
	model.mesh()->add_section(fea::mesh::sections::type::rectangle);
	((fea::mesh::sections::Rectangle*) model.mesh()->section(0))->width(w);
	((fea::mesh::sections::Rectangle*) model.mesh()->section(0))->height(h);

	//setup
	sprintf(path, "%s/wall.txt", dir);
	model.mesh()->section(0)->prepare();
	const double As = model.mesh()->section(0)->area(1, 1);

	//warping
	FILE* file = fopen(path, "w");
	for(unsigned i = 0; i <= np; i++)
	{
		x[0] = x[2] = 0;
		x[1] = h * i / np - h / 2;
		model.mesh()->section(0)->mesh_local()->warping(u, x, 2);
		fprintf(file, "%+.6e %+.6e\n", 10 * double(i) / np, -(u - x[1]) / As);
	}
	fclose(file);
}