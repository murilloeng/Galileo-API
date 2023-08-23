#pragma once

//fea
#include "fea/inc/Mesh/Elements/Mechanic/Frame/Bar.h"

namespace fea
{
	namespace boundary
	{
		namespace loads
		{
			class Axial;
		}
	}
}

namespace fea
{
	namespace mesh
	{
		namespace elements
		{
			class Bar3 : public Bar
			{
				friend class Element;

			protected:
				//contructors
				Bar3(void);

				//destructors
				virtual ~Bar3(void) override;

			public:
				//types
				virtual elements::type type(void) const override;
				virtual unsigned dof_set(unsigned) const override;

			protected:
				//formulation
				virtual double kinetic_energy(void) const override;
				virtual double internal_energy(void) const override;

				virtual double* internal_force(double*) const override;
				virtual double* load_line_force(double*, const boundary::loads::Line_Force*) const;

				virtual double* inertia(double*) const override;
				virtual double* damping(double*) const override;
				virtual double* stiffness(double*) const override;
			};
		}
	}
}