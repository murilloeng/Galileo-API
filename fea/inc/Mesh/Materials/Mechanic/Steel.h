#pragma once

//fea
#include "fea/inc/Mesh/Materials/Mechanic/Mechanic.h"

namespace fea
{
	namespace mesh
	{
		namespace materials
		{
			class Steel : public Mechanic
			{
				friend class Material;

			protected:
				//constructors
				Steel(void);

				//destructor
				virtual ~Steel(void) override;

				//serialization
				virtual void load(FILE*) override;
				virtual void save(FILE*) const override;

			public:
				//type
				virtual materials::type type(void) const;

				//data
				virtual double yield_stress(double);
				virtual double yield_stress(void) const;

				virtual double break_stress(double);
				virtual double break_stress(void) const;

				virtual double break_strain(double);
				virtual double break_strain(void) const override;

				virtual double poisson_ratio(double);
				virtual double poisson_ratio(void) const override;

				virtual double elastic_modulus(double);
				virtual double elastic_modulus(void) const override;

				virtual double plastic_modulus(double);
				virtual double plastic_modulus(void) const;

				virtual double thermal_coefficient(void) const override;

				//material point
				virtual unsigned damage(void) const override;
				virtual unsigned hardening(void) const override;

				//yield criteria
				virtual void yield_hessian(double*, const double*, unsigned) const;
				virtual void yield_gradient(double*, const double*, unsigned) const;
				virtual double yield_function(const double*, double, unsigned) const;

				//return mapping
				virtual bool broken(double*, double*, const double*, points::Mechanic&) const;
				virtual bool predictor(double*, double*, const double*, points::Mechanic&) const;
				virtual bool corrector(double*, double*, const double*, points::Mechanic&) const;

				virtual double stress(const double*, unsigned) const override;
				virtual void stress(double*, const double*, const points::Mechanic&) const override;
				virtual void stiffness(double*, const double*, const points::Mechanic&) const override;
				virtual void return_mapping(double*, double*, const double*, points::Mechanic&) const override;

			protected:
				//data
				double m_yield_stress;
				double m_break_strain;
				double m_break_stress;
				double m_poisson_ratio;
				double m_elastic_modulus;
			};
		}
	}
}