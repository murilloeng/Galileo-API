//std
#include <cmath>

//fea
#include "fea/inc/Mesh/Mesh.h"
#include "fea/inc/Model/Model.h"
#include "fea/inc/Mesh/Sections/Types.h"
#include "fea/inc/Mesh/Materials/Types.h"
#include "fea/inc/Mesh/Sections/Rectangle.h"
#include "fea/inc/Mesh/Sections/Resultant.h"
#include "fea/inc/Mesh/Materials/Mechanic/Steel.h"

//ben
#include "ben/inc/benchmarks/sections/sections.h"

namespace tests
{
	namespace sections
	{
		void rectangle(void)
		{
			//data
			const double w = 2.00e-01;
			const double h = 6.00e-01;
			const double E = 2.00e+11;
			const double sy = 4.00e+08;
			const double kp = 0.00e+00;

			//model
			fea::models::Model model;

			//section
			model.mesh()->add_section(fea::mesh::sections::type::rectangle);
			((fea::mesh::sections::Rectangle*) model.mesh()->section(0))->width(w);
			((fea::mesh::sections::Rectangle*) model.mesh()->section(0))->height(h);

			//material
			model.mesh()->add_material(fea::mesh::materials::type::steel);
			((fea::mesh::materials::Steel*) model.mesh()->material(0))->yield_stress(sy);
			((fea::mesh::materials::Steel*) model.mesh()->material(0))->elastic_modulus(E);
			((fea::mesh::materials::Steel*) model.mesh()->material(0))->plastic_modulus(kp);

			//compute
			model.mesh()->section(0)->prepare();
			fea::mesh::sections::Resultant* resultant = model.mesh()->section(0)->resultant();

			//resultant
			resultant->mode(0);
			resultant->index(0, 2);
			resultant->index(1, 4);
			resultant->compute();
			resultant->save();
		}
	}
}