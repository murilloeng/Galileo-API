#pragma once

//fea
#include "fea/inc/Mesh/Materials/Mechanic/Mechanic.h"

namespace fea
{
	namespace mesh
	{
		namespace materials
		{
			class Concrete : public Mechanic
			{
				friend class Material;

			protected:
				//constructors
				Concrete(void);

				//destructor
				virtual ~Concrete(void) override;

				//serialization
				virtual void load(FILE*) override;
				virtual void save(FILE*) const override;

			public:
				//type
				virtual materials::type type(void) const override;

				//data
				virtual double softening(double);
				virtual double softening(void) const;

				virtual double break_strain(double);
				virtual double break_strain(void) const;

				virtual double poisson_ratio(void) const override;

				virtual double elastic_modulus(double);
				virtual double elastic_modulus(void) const override;

				virtual double yield_stress_tension(double);
				virtual double yield_stress_tension(void) const;

				virtual double yield_stress_biaxial(double);
				virtual double yield_stress_biaxial(void) const;

				virtual double yield_stress_compression(double);
				virtual double yield_stress_compression(void) const;

				virtual double thermal_coefficient(void) const override;

				//material point
				virtual unsigned damage(void) const override;
				virtual unsigned hardening(void) const override;

				//yield criterion
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
				double m_softening;
				double m_break_strain;
				double m_elastic_modulus;
				double m_yield_stress_tension;
				double m_yield_stress_biaxial;
				double m_yield_stress_compression;
			};
		}
	}
}