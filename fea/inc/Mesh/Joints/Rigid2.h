#pragma once

//fea
#include "Mesh/Joints/Joint.h"

namespace fea
{
	namespace mesh
	{
		namespace joints
		{
			class Rigid2 : public Joint
			{
				friend class Joint;

			protected:
				//constructors
				Rigid2(void);

				//destructor
				virtual ~Rigid2(void) override;

			public:
				//types
				virtual joints::type type(void) const override;
				virtual unsigned nodes_max(void) const override;
				virtual unsigned state_set(void) const override;
				virtual unsigned dof_set(unsigned) const override;

			protected:
				//analysis
				virtual void apply(void) override;
				virtual void configure(void) override;

				//formulation
				virtual double* internal_force(double*) const override;

				virtual double* inertia(double*) const override;
				virtual double* damping(double*) const override;
				virtual double* stiffness(double*) const override;

				//draw
				virtual void draw(void) const override;
			};
		}
	}
}