#pragma once

//fea
#include "Mesh/Materials/Material.h"

namespace fea
{
	namespace mesh
	{
		namespace points
		{
			class Mechanic;
		}
	}
}

namespace fea
{
	namespace mesh
	{
		namespace materials
		{
			class Mechanic : public Material
			{
			public:
				//constructors
				Mechanic(void);

				//destructor
				virtual ~Mechanic(void);

				//properties
				virtual double bulk_modulus(void) const;
				virtual double shear_modulus(void) const;

				virtual double break_strain(void) const = 0;
				virtual double poisson_ratio(void) const = 0;
				virtual double elastic_modulus(void) const = 0;
				virtual double thermal_coefficient(void) const = 0;

				virtual double* elastic_stiffness(double*, unsigned) const;
				virtual double* elastic_flexibility(double*, unsigned) const;

				//material point
				virtual unsigned damage(void) const = 0;
				virtual unsigned hardening(void) const = 0;

				//return mapping
				virtual double stress(const double*, unsigned) const = 0;
				virtual void stress(double*, const double*, const points::Mechanic&) const = 0;
				virtual void stiffness(double*, const double*, const points::Mechanic&) const = 0;
				virtual void return_mapping(double*, double*, const double*, points::Mechanic&) const = 0;
			};
		}
	}
}