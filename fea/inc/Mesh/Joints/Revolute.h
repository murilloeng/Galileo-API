#pragma once

//fea
#include "fea/inc/Mesh/Joints/Joint.h"

namespace fea
{
	namespace mesh
	{
		namespace joints
		{
			class Revolute : public Joint
			{
				friend class Joint;

			protected:
				//constructors
				Revolute(void);

				//destructor
				virtual ~Revolute(void);

				//serialization
				virtual void load(FILE*);
				virtual void save(FILE*) const;

			public:
				//types
				virtual joints::type type(void) const override;
				virtual unsigned nodes_max(void) const override;
				virtual unsigned state_set(void) const override;
				virtual unsigned dof_set(unsigned) const override;

				//data
				double stiffness_axial(double);
				double stiffness_shear(double);
				double stiffness_torsion(double);
				double stiffness_bending(double);

				double stiffness_axial(void) const;
				double stiffness_shear(void) const;
				double stiffness_torsion(void) const;
				double stiffness_bending(void) const;

			protected:
				//analysis
				virtual void apply(void) override;

				//formulation
				virtual double* internal_force(double*) const override;

				virtual double* inertia(double*) const override;
				virtual double* damping(double*) const override;
				virtual double* stiffness(double*) const override;

				//draw
				virtual void draw(void) const override;

				//data
				double m_ka;
				double m_ks;
				double m_kb;
				double m_kt;
			};
		}
	}
}