#pragma once

//fea
#include "fea/inc/Mesh/Elements/Mechanic/Mechanic.h"

namespace fea
{
	namespace mesh
	{
		namespace elements
		{
			class Warping : public Mechanic
			{
				friend class Element;

			protected:
				//constructors
				Warping(void);

				//destructor
				virtual ~Warping(void) override;

			public:
				//types
				virtual unsigned cell_set(void) const override;
				virtual unsigned load_set(void) const override;
				virtual uint64_t state_set(void) const override;
				virtual unsigned stress_set(void) const override;
				virtual unsigned dof_set(unsigned) const override;

				virtual elements::type type(void) const override;

				//section
				double stiffness(void) const;

			protected:
				//analysis
				virtual void apply(void) override;
				virtual void record(void) override;

				//formulation
				virtual double kinetic_energy(void) const override;
				virtual double internal_energy(void) const override;

				virtual double* internal_force(double*) const override;
				virtual double* reference_force(double*, const boundary::loads::Element*) const override;

				virtual double* inertia(double*) const override;
				virtual double* damping(double*) const override;
				virtual double* stiffness(double*) const override;
			};
		}
	}
}