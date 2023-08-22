#pragma once

#include "fea/inc/Mesh/Elements/Mechanic/Mechanic.h"

namespace fea
{
	namespace boundary
	{
		namespace loads
		{
			class Plane_Face;
		}
	}
}

namespace fea
{
	namespace mesh
	{
		namespace elements
		{
			class Shell : public Mechanic
			{
				friend class Element;

			protected:
				//constructors
				Shell(void);

				//destructor
				virtual ~Shell(void) override;

			public:
				//types
				virtual unsigned cell_set(void) const override;
				virtual unsigned load_set(void) const override;
				virtual uint64_t state_set(void) const override;
				virtual unsigned stress_set(void) const override;
				virtual unsigned dof_set(unsigned) const override;

				virtual elements::type type(void) const override;

			protected:
				//analysis
				virtual void apply(void) override;
				virtual void record(void) override;

				//formulation
				virtual double* inertia(double*) const override;
				virtual double* damping(double*) const override;
				virtual double* stiffness(double*) const override;

				virtual double* reference_force(double*, const boundary::loads::Element*) const override;

				//cell
				virtual void cell_setup(void) const;
				virtual void cell_shape(void) const;
				virtual void cell_stiffness(void) const;
				virtual void cell_mapping(const double*) const;
				virtual void cell_kinematics(const double*) const;
				virtual void cell_function(const double*, unsigned) const;

				//static data
				static double m_x[3][2];
				static double m_A, m_s[3], m_c[3], m_ds[3][2], m_R[9], m_h[9], m_B[108], m_P[81], m_Bp[27], m_Kt[36];
			};
		}
	}
}