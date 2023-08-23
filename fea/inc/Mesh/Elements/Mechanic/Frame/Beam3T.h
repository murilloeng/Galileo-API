#pragma once

//fea
#include "fea/inc/Mesh/Elements/Mechanic/Frame/Beam3.h"

namespace fea
{
	namespace mesh
	{
		namespace elements
		{
			class Beam3T : public Beam3
			{
				friend class Element;

			protected:
				//constructors
				Beam3T(void);

				//destructor
				virtual ~Beam3T(void) override;

			public:
				//types
				virtual elements::type type(void) const override;

			protected:
				//analysis
				virtual void apply(void) override;
				virtual void record(void) override;

				//formulation
				virtual double kinetic_energy(void) const override;
				virtual double internal_energy(void) const override;

				virtual double* kinetic_force(double*) const override;
				virtual double* internal_force(double*) const override;

				virtual double* inertia(double*) const override;
				virtual double* damping(double*) const override;
				virtual double* stiffness(double*) const override;

				//data
				double m_es[6], m_fs[6], m_ks[36];
			};
		}
	}
}