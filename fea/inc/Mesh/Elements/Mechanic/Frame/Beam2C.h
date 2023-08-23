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
			class Beam2C : public Beam2
			{
				friend class Element;

			protected:
				//constructors
				Beam2C(void);

				//destructor
				virtual ~Beam2C(void) override;

			public:
				//types
				virtual elements::type type(void) const override;

			protected:
				//analysis
				virtual void apply(void) override;
				virtual void record(void) override;
				virtual void update(void) override;
				virtual void restore(void) override;
				virtual void prepare(void) override;

				//formulation
				virtual double kinetic_energy(void) const override;
				virtual double internal_energy(void) const override;

				virtual double* internal_force(double*) const override;

				virtual double* inertia(double*) const override;
				virtual double* damping(double*) const override;
				virtual double* stiffness(double*) const override;

				virtual void local_elastic(double);
				virtual void local_plastic(double);

				virtual void section_gradient(double*, double) const;
				virtual void cell_gradient(double*, double, double) const;
				virtual void section_stress(const double*, unsigned) const;

				//data
				double m_d[3];
				double m_f[3];
				double m_k[9];
				double m_tr_old;
				double m_tr_new;
			};
		}
	}
}