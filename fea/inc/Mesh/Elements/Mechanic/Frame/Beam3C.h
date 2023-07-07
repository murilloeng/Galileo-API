#pragma once

//mat
#include "linear/matrix.h"

//fea
#include "Mesh/Elements/Mechanic/Frame/Beam3.h"

namespace fea
{
	namespace mesh
	{
		namespace points
		{
			class Fiber;
		}
	}
}

namespace fea
{
	namespace mesh
	{
		namespace elements
		{
			class Beam3C : public Beam3
			{
				friend class Element;

			protected:
				//constructors
				Beam3C(void);

				//destructor
				virtual ~Beam3C(void) override;

			public:
				//types
				virtual elements::type type(void) const override;

			protected:
				//sizes
				virtual unsigned size_strains(void) const;
				virtual unsigned size_local_dof(void) const;
				virtual unsigned size_global_dof(void) const;

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

				virtual void local_mixed(double);
				virtual void local_elastic(double);
				virtual void local_plastic(double);
				virtual void local_inertia(double*) const;
				virtual void local_rotation_force(double*) const;
				virtual void local_rotation_stiffness(double*) const;
				virtual void global_rotation_force(double*, const double*) const;
				virtual void global_rotation_stiffness(double*, const double*, const double*) const;

				virtual void shear_state(void);
				virtual void shear_force(void);
				virtual void connection_state(void);
				virtual void connection_force(void);

				virtual void cell_gradient(unsigned) const;
				virtual void cell_strain_nonlinear(mat::vector&, const mat::vector&) const;
				virtual void cell_hessian_nonlinear(mat::matrix&, const mat::vector&) const;
				virtual void cell_gradient_nonlinear(mat::matrix&, const mat::vector&) const;

				virtual void section_stiffness(void);
				virtual void section_gradient(mat::matrix&, const points::Fiber&, unsigned) const;

				//data
				double m_U;
				double m_d[12];
				double m_f[12];
				double m_K[144];
				mat::matrix m_Kh;
				mat::matrix* m_Bs;
			};
		}
	}
}