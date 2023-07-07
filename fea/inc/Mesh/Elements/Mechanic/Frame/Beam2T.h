#pragma once

//fea
#include "Mesh/Elements/Mechanic/Frame/Beam2.h"

namespace fea
{
	namespace boundary
	{
		namespace loads
		{
			class Axial;
			class Shear;
			class Bending;
		}
	}
}

namespace fea
{
	namespace mesh
	{
		namespace elements
		{
			class Beam2T : public Beam2
			{
				friend class Element;

			protected:
				//constructors
				Beam2T(void);

				//destructor
				virtual ~Beam2T(void) override;

			public:
				//types
				virtual elements::type type(void) const override;

			protected:
				//analysis
				virtual void apply(void) override;
				virtual void record(void) override;
				virtual void prepare(void) override;

				//formulation
				virtual double kinetic_energy(void) const override;
				virtual double internal_energy(void) const override;

				virtual double* internal_force(double*) const override;

				virtual double* inertia(double*) const override;
				virtual double* damping(double*) const override;
				virtual double* stiffness(double*) const override;

				//data
				double m_es[3], m_fs[3], m_ks[9];
			};
		}
	}
}