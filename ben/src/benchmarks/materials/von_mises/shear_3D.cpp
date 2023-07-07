//std
#include <cstring>
#include <filesystem>

//mat
#include "misc/util.h"
#include "misc/stress.h"

//fea
#include "Mesh/Mesh.h"
#include "Model/Model.h"
#include "Mesh/Materials/Types.h"
#include "Mesh/Points/Mechanic/Mechanic.h"
#include "Mesh/Materials/Mechanic/Steel.h"

//ben
#include "benchmarks/materials/materials.h"

void tests::materials::von_mises::shear_3D(void)
{
	//data
	const unsigned n = 1000;
	const double E = 2.00e+11;
	const double v = 3.00e-01;
	const double kp = 0.00e+05;
	const double sy = 2.50e+06;

	//stress
	const unsigned st = 
		unsigned(mat::stress::s11) | 
		unsigned(mat::stress::s12) | 
		unsigned(mat::stress::s13);

	//memory
	const unsigned nt = mat::bit_count(st);
	double* e = (double*) alloca(nt * sizeof(double));
	double* s = (double*) alloca(nt * sizeof(double));
	double* C = (double*) alloca(nt * nt * sizeof(double));

	//model
	fea::models::Model model;
	fea::mesh::points::Mechanic point;
	const std::string path = "../models/benchmarks/materials/von_mises/shear_3D/";

	//material
	model.mesh()->add_material(fea::mesh::materials::type::steel);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->yield_stress(sy);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->poisson_ratio(v);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->elastic_modulus(E);
	((fea::mesh::materials::Steel*) model.mesh()->material(0))->plastic_modulus(kp);
	const unsigned nd = ((fea::mesh::materials::Mechanic*) model.mesh()->material(0))->damage();
	const unsigned nh = ((fea::mesh::materials::Mechanic*) model.mesh()->material(0))->hardening();

	//open
	std::filesystem::create_directories(path);
	FILE* file = fopen((path + "data.txt").c_str(), "w");

	//write
	const double a = 3.00;
	const double ey = sy / E;
	point.prepare(st, nd, nh);
	for(unsigned i = 0; i < n; i++)
	{
		//return mapping
		e[0] = 2 * ey * i / n;
		e[1] = 2 * a * ey * i / n;
		e[2] = 2 * a * ey * i / n;
		((fea::mesh::materials::Mechanic*) model.mesh()->material(0))->return_mapping(s, C, e, point);
		//print
		for(unsigned j = 0; j < nt; j++)
		{
			fprintf(file, "%+.6e %+.6e %+.6e ", e[j] / ey, s[j] / sy, point.m_plastic_strain_new[j] / ey);
		}
		const double s1 = mat::stress_principal_s1(s, nt);
		const double s2 = mat::stress_principal_s2(s, nt);
		const double s3 = mat::stress_principal_s3(s, nt);
		fprintf(file, "%+.6e %+.6e %+.6e\n", s1 / sy, s2 / sy, s3 / sy);
		//update
		point.update();
	}

	//close
	fclose(file);
}