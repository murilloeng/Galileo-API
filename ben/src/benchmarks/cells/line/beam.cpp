//std
#include <string>
#include <algorithm>
#include <filesystem>

//fea
#include "Mesh/Mesh.h"
#include "Model/Model.h"
#include "Mesh/Cells/Types.h"
#include "Mesh/Sections/Types.h"
#include "Mesh/Elements/Types.h"
#include "Mesh/Materials/Types.h"
#include "Mesh/Cells/Line/Beam.h"
#include "Mesh/Sections/Section.h"
#include "Mesh/Materials/Mechanic/Steel.h"
#include "Mesh/Elements/Mechanic/Frame/Beam3.h"
#include "Mesh/Elements/Mechanic/Frame/Warping.h"

//ben
#include "benchmarks/cells/cells.h"

void tests::cells::line::beam(void)
{
	//data
	char p1[100], p2[100];
	const unsigned np = 100;
	const double L = 5.00e-01;
	const double E = 2.00e+11;
	const double v = 3.00e-01;
	const std::string folder = "../models/benchmarks/cells/line/beam";

	const fea::mesh::sections::type types[] = {
		fea::mesh::sections::type::profile_I,
		fea::mesh::sections::type::profile_C,
		fea::mesh::sections::type::profile_L
	};
	const fea::mesh::elements::warping modes[] = {
		fea::mesh::elements::warping::vlasov,
		fea::mesh::elements::warping::benscoter,
		fea::mesh::elements::warping::saint_venant
	};

	//model
	fea::models::Model model("beam", "benchmarks/cells/line");

	//nodes
	model.mesh()->add_node(0, 0, 0);
	model.mesh()->add_node(L, 0, 0);

	//sections
	model.mesh()->add_section(types[0]);
	model.mesh()->add_section(types[1]);
	model.mesh()->add_section(types[2]);

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::beam);
	model.mesh()->add_cell(fea::mesh::cells::type::beam);
	model.mesh()->add_cell(fea::mesh::cells::type::beam);
	((fea::mesh::cells::Line*) model.mesh()->cell(0))->section(0);
	((fea::mesh::cells::Line*) model.mesh()->cell(1))->section(1);
	((fea::mesh::cells::Line*) model.mesh()->cell(2))->section(2);

	//elements
	model.mesh()->add_element(fea::mesh::elements::type::beam3C, {0, 1});

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->poisson_ratio(v);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->elastic_modulus(E);

	//print
	for(unsigned type = 0; type < 3; type++)
	{
		model.mesh()->section(type)->prepare();
		for(fea::mesh::elements::warping mode : modes)
		{
			//setup
			fea::mesh::elements::Beam3::warping(mode);
			std::string name = model.mesh()->section(type)->name();
			name.erase(std::remove(name.begin(), name.end(), ' '), name.end());
			sprintf(p1, "%s/%s/%d", folder.c_str(), name.c_str(), unsigned(mode));
			//sizes
			const unsigned nf = 
				mode == fea::mesh::elements::warping::vlasov ? 9 : 
				mode == fea::mesh::elements::warping::benscoter ? 9 : 
				mode == fea::mesh::elements::warping::saint_venant ? 6 : 0;
			const unsigned ns = 
				mode == fea::mesh::elements::warping::vlasov ? 9 : 
				mode == fea::mesh::elements::warping::benscoter ? 12 : 
				mode == fea::mesh::elements::warping::saint_venant ? 6 : 0;
			//fields
			mat::matrix Pf(nf, 2 * nf);
			std::filesystem::create_directories(p1);
			FILE** ff = (FILE**) alloca(nf * sizeof(FILE*));
			for(unsigned i = 0; i < nf; i++)
			{
				sprintf(p2, "%s/fields_%d.txt", p1, i);
				ff[i] = fopen(p2, "w");
			}
			for(unsigned i = 0; i <= np; i++)
			{
				const double s = 2 * double(i) / np - 1;
				((fea::mesh::cells::Beam*) model.mesh()->cell(type))->function(Pf, model.mesh()->element(0), s);
				for(unsigned j = 0; j < nf; j++)
				{
					fprintf(ff[j], "%+.6e ", L / 2 * (1 + s));
					for(unsigned k = 0; k < 2 * nf; k++)
					{
						fprintf(ff[j], "%+.6e ", Pf(j, k));
					}
					fprintf(ff[j], "\n");
				}
			}
			for(unsigned i = 0; i < nf; i++)
			{
				fclose(ff[i]);
			}
			//strains
			mat::matrix Ps(ns, 2 * nf);
			FILE** fs = (FILE**) alloca(ns * sizeof(FILE*));
			for(unsigned i = 0; i < ns; i++)
			{
				sprintf(p2, "%s/strains_%d.txt", p1, i);
				fs[i] = fopen(p2, "w");
			}
			for(unsigned i = 0; i <= np; i++)
			{
				const double s = 2 * double(i) / np - 1;
				((fea::mesh::cells::Beam*) model.mesh()->cell(type))->strain(Ps, model.mesh()->element(0), s);
				for(unsigned j = 0; j < ns; j++)
				{
					fprintf(fs[j], "%+.6e ", L / 2 * (1 + s));
					for(unsigned k = 0; k < 2 * nf; k++)
					{
						fprintf(fs[j], "%+.6e ", Ps(j, k));
					}
					fprintf(fs[j], "\n");
				}
			}
			for(unsigned i = 0; i < ns; i++)
			{
				fclose(fs[i]);
			}
		}
	}
}