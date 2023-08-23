#pragma once

//fea
#include "fea/inc/Mesh/Elements/Mechanic/Frame/Beam2.h"

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
				~Beam2T(void) override;

			public:
				//types
				elements::type type(void) const override;

			protected:
				//local
				void local_elastic(void);
				void local_plastic(void);
				void strain_hessian(double*) const;
				void strain_gradient(double*) const;

				//analysis
				void apply(void) override;
				void record(void) override;
				void prepare(void) override;

				//formulation
				double kinetic_energy(void) const override;
				double internal_energy(void) const override;

				double* internal_force(double*) const override;

				double* inertia(double*) const override;
				double* damping(double*) const override;
				double* stiffness(double*) const override;

				//data
				double m_l0, m_a0, m_es[3], m_fs[3], m_ks[9];
			};
		}
	}
}