#pragma once

//fea
#include "fea/inc/Mesh/Joints/Joint.h"

namespace fea
{
	namespace mesh
	{
		namespace joints
		{
			class Hinge : public Joint
			{
				friend class Joint;

			protected:
				//constructors
				Hinge(void);

				//destructor
				virtual ~Hinge(void);

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
				const double* axis(void) const;
				const double* axis(const double*);
				const double* axis(unsigned, double);
				const double* axis(double, double, double);

				const double* orientation(void) const;
				const double* orientation(const double*);
				const double* orientation(unsigned, double);
				const double* orientation(double, double, double);

				const double* stiffness(unsigned) const;
				const double* stiffness(unsigned, const double*);
				const double* stiffness(unsigned, unsigned, double);
				const double* stiffness(unsigned, double, double, double);

				double moment_yield(void) const;
				double moment_yield(double);
				double plastic_modulus(void) const;
				double plastic_modulus(double);

				double limit_rotation(void) const;
				double limit_rotation(double);
				double limit_translation(void) const;
				double limit_translation(double);

			protected:
				//analysis
				virtual void apply(void) override;

				virtual void prepare(void) override;

				virtual void record(void) override;
				virtual void update(void) override;
				virtual void restore(void) override;

				//formulation
				virtual double internal_energy(void) const override;

				virtual double* internal_force(double*) const override;

				virtual double* inertia(double*) const override;
				virtual double* damping(double*) const override;
				virtual double* stiffness(double*) const override;

				//draw
				virtual void draw(void) const override;

				//data
				double m_axis[3];
				double m_orientation[3];

				double m_stiffness[2][3];

				double m_moment_yield;
				double m_plastic_modulus;

				double m_limit_rotation;
				double m_limit_translation;

				double m_hardening_old;
				double m_hardening_new;
				double m_plastic_rotation_old;
				double m_plastic_rotation_new;

				double m_U;
				double m_f[6];
				double m_k[18];
			};
		}
	}
}