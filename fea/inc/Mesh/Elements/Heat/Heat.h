#pragma once

//fea
#include "fea/inc/Mesh/Elements/Element.h"

namespace fea
{
	namespace mesh
	{
		namespace elements
		{
			class Heat : public Element
			{
				friend class Element;

			protected:
				//constructors
				Heat(void);

				//destructor
				virtual ~Heat(void) override;

			public:
				//types
				virtual unsigned cell_set(void) const override;
				virtual unsigned load_set(void) const override;
				virtual uint64_t state_set(void) const override;
				virtual unsigned dof_set(unsigned) const override;

				virtual points::type point(void) const override;
				virtual elements::type type(void) const override;

			protected:
				//analysis
				virtual void apply(void) override;

				virtual void record(void) override;

				//formulation
				virtual double kinetic_energy(void) const override;
				virtual double internal_energy(void) const override;

				virtual double* reference_force(double*, const boundary::loads::Element*) const override;

				virtual double* inertia(double*) const override;
				virtual double* damping(double*) const override;
				virtual double* stiffness(double*) const override;
			};
		}
	}
}